// generated with help from https://gemini.google.com/app/19c8818984b5f80c
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h> // For struct platform_device, used with device tree
#include <linux/gpio/consumer.h> // For gpiod_get_from_of_node, gpiod_to_irq, gpiod_direction_input, etc. (modern GPIO API)
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/spinlock.h>
#include <linux/pwm.h>          // For pwm_get, pwm_config, pwm_enable/disable, pwm_put
#include <linux/err.h>

/*
 * BUILD_DATE is expected to be passed in from the Makefile, e.g.,
 *   EXTRA_CFLAGS += -DBUILD_DATE= some date string without spaces'
 */
#ifndef BUILD_DATE
#define BUILD_DATE "unknown"
#endif

// --- Device Data Structure ---
struct argo_radio_servo_data {
	struct device *dev;
	struct gpio_desc *radio_rudder_gpiod;
	int radio_rudder_irq;
	ktime_t radio_rudder_rising_ts;
	long long radio_rudder_pw_us;
	ktime_t radio_rudder_last_pulse_ts;

	struct gpio_desc *radio_sail_gpiod;
	int radio_sail_irq;
	ktime_t radio_sail_rising_ts;
	long long radio_sail_pw_us;
	ktime_t radio_sail_last_pulse_ts;

	struct pwm_device *servo_rudder_pwm;
	struct pwm_device *servo_sail_pwm;

	unsigned long current_servo_rudder_pw;
	unsigned long current_servo_sail_pw;

	struct hrtimer print_timer;
	ktime_t module_load_ts;
	struct kobject kobj;
};

// --- Common Servo/PWM Parameters ---
#define SERVO_PERIOD_NS               20000000UL // 20ms period in nanoseconds
#define SERVO_MIN_PW_NS               900000UL   // 900us minimum pulse width in nanoseconds
#define SERVO_MAX_PW_NS               2100000UL  // 2100us maximum pulse width in nanoseconds
#define NO_PULSE_TIMEOUT_NS           (1 * NSEC_PER_SEC) // 1 second in nanoseconds

// --- Shared Resources ---
static DEFINE_SPINLOCK(capture_data_lock); // Spinlock to protect measured data
static DEFINE_SPINLOCK(output_control_lock); // Spinlock to protect PWM output writes

// --- Function Prototypes ---
static int argo_radio_servo_probe(struct platform_device *pdev);
static int argo_radio_servo_remove(struct platform_device *pdev);

// --- Sysfs Function Prototypes ---
static ssize_t radio_rudder_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t radio_sail_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t servo_rudder_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t servo_rudder_pw_us_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t servo_sail_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t servo_sail_pw_us_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);

// --- Release function for the kobject ---
static void argo_radio_servo_kobj_release(struct kobject *kobj)
{
	// The container_of() macro calculates the address of the parent structure
	// from the address of its member.
	struct argo_radio_servo_data *data = container_of(kobj, struct argo_radio_servo_data, kobj);
	dev_info(data->dev, "kobject released.\n");
	// The memory for 'data' is managed by devm_kzalloc, so we don't kfree() it here.
}

static struct kobj_type argo_ktype = {
	.release = argo_radio_servo_kobj_release,
	.sysfs_ops = &kobj_sysfs_ops,
};

// --- Sysfs Attribute Definitions ---
// group and group permission for dialout group are set in argo-radio-servo-postinit.sh which runs once as a boot service
// radio input readonly files
static struct kobj_attribute radio_rudder_pw_us_attribute =
	__ATTR_RO(radio_rudder_pw_us);

static struct kobj_attribute radio_sail_pw_us_attribute =
	__ATTR_RO(radio_sail_pw_us);

// servo output writable files
static struct kobj_attribute servo_rudder_pw_us_attribute =
	__ATTR_RW(servo_rudder_pw_us);
static struct kobj_attribute servo_sail_pw_us_attribute =
	__ATTR_RW(servo_sail_pw_us);

static struct attribute *argo_attrs[] = {
	&radio_rudder_pw_us_attribute.attr,
	&radio_sail_pw_us_attribute.attr,
	&servo_rudder_pw_us_attribute.attr,
	&servo_sail_pw_us_attribute.attr,
	NULL,
};

static struct attribute_group argo_attr_group = {
	.attrs = argo_attrs,
};


// --- Input Measurement ISRs ---
static irqreturn_t radio_rudder_gpio_isr(int irq, void *dev_id)
{
	ktime_t current_time;
	unsigned long flags;
	struct argo_radio_servo_data *data = dev_id;

	current_time = ktime_get();

	spin_lock_irqsave(&capture_data_lock, flags);
	data->radio_rudder_last_pulse_ts = current_time; // Update last pulse timestamp

	if (gpiod_get_value(data->radio_rudder_gpiod)) { // GPIO is High (Rising Edge)
		data->radio_rudder_rising_ts = current_time;
	} else { // GPIO is Low (Falling Edge)
		if (ktime_compare(current_time, data->radio_rudder_rising_ts) > 0) {
			long long duration_ns = ktime_to_ns(ktime_sub(current_time, data->radio_rudder_rising_ts));
			data->radio_rudder_pw_us = duration_ns / 1000;
		} else {
			data->radio_rudder_pw_us = 0; // Reset if invalid sequence
			dev_warn(data->dev, "Radio Rudder: Spurious falling edge or falling edge before rising edge.\n");
		}
	}

	spin_unlock_irqrestore(&capture_data_lock, flags);
	return IRQ_HANDLED;
}

static irqreturn_t radio_sail_gpio_isr(int irq, void *dev_id)
{
	ktime_t current_time;
	unsigned long flags;
	struct argo_radio_servo_data *data = dev_id;

	current_time = ktime_get();

	spin_lock_irqsave(&capture_data_lock, flags);
	data->radio_sail_last_pulse_ts = current_time; // Update last pulse timestamp

	if (gpiod_get_value(data->radio_sail_gpiod)) { // GPIO is High (Rising Edge)
		data->radio_sail_rising_ts = current_time;
	} else { // GPIO is Low (Falling Edge)
		if (ktime_compare(current_time, data->radio_sail_rising_ts) > 0) {
			long long duration_ns = ktime_to_ns(ktime_sub(current_time, data->radio_sail_rising_ts));
			data->radio_sail_pw_us = duration_ns / 1000;
		} else {
			data->radio_sail_pw_us = 0; // Reset if invalid sequence
			dev_warn(data->dev, "Radio Sail: Spurious falling edge or falling edge before rising edge.\n");
		}
	}

	spin_unlock_irqrestore(&capture_data_lock, flags);
	return IRQ_HANDLED;
}

// --- High-Resolution Timer Callback for Periodic Printk ---
static enum hrtimer_restart print_timer_callback(struct hrtimer *timer)
{
	long long rudder_pw, sail_pw;
	ktime_t current_time = ktime_get();
	unsigned long flags;
	struct argo_radio_servo_data *data = from_timer(data, timer, print_timer);
	s64 elapsed_ns = ktime_to_ns(ktime_sub(current_time, data->module_load_ts));

	spin_lock_irqsave(&capture_data_lock, flags);

	// Check for timeout for Radio Rudder
	if (ktime_to_ns(ktime_sub(current_time, data->radio_rudder_last_pulse_ts)) > NO_PULSE_TIMEOUT_NS) {
		if (data->radio_rudder_pw_us != 0) { // Check if it was non-zero before clearing
			if (elapsed_ns < (s64)60 * NSEC_PER_SEC) { // Only print if within the first minute
				dev_info(data->dev, "Radio rudder pulses disappeared, setting PW to 0.\n");
			}
		}
		data->radio_rudder_pw_us = 0;
	}
	rudder_pw = data->radio_rudder_pw_us;

	// Check for timeout for Radio Sail
	if (ktime_to_ns(ktime_sub(current_time, data->radio_sail_last_pulse_ts)) > NO_PULSE_TIMEOUT_NS) {
		if (data->radio_sail_pw_us != 0) { // Check if it was non-zero before clearing
			if (elapsed_ns < (s64)60 * NSEC_PER_SEC) { // Only print if within the first minute
				dev_info(data->dev, "Radio sail pulses disappeared, setting PW to 0.\n");
			}
		}
		data->radio_sail_pw_us = 0;
	}
	sail_pw = data->radio_sail_pw_us;

	spin_unlock_irqrestore(&capture_data_lock, flags);

	// Only print periodic updates if within the first minute
	if (elapsed_ns < (s64)60 * NSEC_PER_SEC) {
		dev_info(data->dev, "Radio Rudder PW: %lld us, Radio Sail PW: %lld us\n",
			 rudder_pw, sail_pw);
	}

	hrtimer_forward_now(timer, ktime_set(5, 0)); // Fire every 5 seconds
	return HRTIMER_RESTART;
}

// --- Sysfs Show/Store Functions ---
static ssize_t radio_rudder_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	long long current_high_time;
	unsigned long flags;
	struct argo_radio_servo_data *data = container_of(kobj, struct argo_radio_servo_data, kobj);

	spin_lock_irqsave(&capture_data_lock, flags);
	current_high_time = data->radio_rudder_pw_us;
	spin_unlock_irqrestore(&capture_data_lock, flags);

	return sprintf(buf, "%lld\n", current_high_time);
}

static ssize_t radio_sail_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	long long current_high_time;
	unsigned long flags;
	struct argo_radio_servo_data *data = container_of(kobj, struct argo_radio_servo_data, kobj);

	spin_lock_irqsave(&capture_data_lock, flags);
	current_high_time = data->radio_sail_pw_us;
	spin_unlock_irqrestore(&capture_data_lock, flags);

	return sprintf(buf, "%lld\n", current_high_time);
}

static void set_servo_pulse_width(struct pwm_device *pwm_dev, unsigned long pulse_width_us)
{
	unsigned long flags;
	u64 pulse_width_ns = pulse_width_us * NSEC_PER_USEC;
	u64 period_ns = SERVO_PERIOD_NS;

	// Sanity check pulse_width
	if (pulse_width_ns < SERVO_MIN_PW_NS)
		pulse_width_ns = SERVO_MIN_PW_NS;
	if (pulse_width_ns > SERVO_MAX_PW_NS)
		pulse_width_ns = SERVO_MAX_PW_NS;

	spin_lock_irqsave(&output_control_lock, flags);

	// Configure the PWM device. This applies period and duty cycle.
	pwm_config(pwm_dev, pulse_width_ns, period_ns);

	spin_unlock_irqrestore(&output_control_lock, flags);

	pr_info("Set PWM device %s to %lu us (duty_ns %llu, period_ns %llu)\n",
		pwm_dev->label, pulse_width_us, pulse_width_ns, period_ns);
}


static ssize_t servo_rudder_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct argo_radio_servo_data *data = container_of(kobj, struct argo_radio_servo_data, kobj);
	return sprintf(buf, "%lu\n", data->current_servo_rudder_pw);
}

static ssize_t servo_rudder_pw_us_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	long pulse_width_us;
	int ret;
	struct argo_radio_servo_data *data = container_of(kobj, struct argo_radio_servo_data, kobj);

	ret = kstrtol(buf, 10, &pulse_width_us);
	if (ret)
		return ret;

	if (pulse_width_us < (long)SERVO_MIN_PW_NS / NSEC_PER_USEC ||
	    pulse_width_us > (long)SERVO_MAX_PW_NS / NSEC_PER_USEC) {
		dev_warn(data->dev, "Servo Rudder pulse width %ld out of range (%lu-%lu us).\n",
			 pulse_width_us, SERVO_MIN_PW_NS / NSEC_PER_USEC, SERVO_MAX_PW_NS / NSEC_PER_USEC);
		return -EINVAL;
	}

	set_servo_pulse_width(data->servo_rudder_pwm, (unsigned long)pulse_width_us);
	data->current_servo_rudder_pw = (unsigned long)pulse_width_us;
	return count;
}

static ssize_t servo_sail_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	struct argo_radio_servo_data *data = container_of(kobj, struct argo_radio_servo_data, kobj);
	return sprintf(buf, "%lu\n", data->current_servo_sail_pw);
}

static ssize_t servo_sail_pw_us_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	long pulse_width_us;
	int ret;
	struct argo_radio_servo_data *data = container_of(kobj, struct argo_radio_servo_data, kobj);

	ret = kstrtol(buf, 10, &pulse_width_us);
	if (ret)
		return ret;

	if (pulse_width_us < (long)SERVO_MIN_PW_NS / NSEC_PER_USEC ||
	    pulse_width_us > (long)SERVO_MAX_PW_NS / NSEC_PER_USEC) {
		dev_warn(data->dev, "Servo Sail pulse width %ld out of range (%lu-%lu us).\n",
			 pulse_width_us, SERVO_MIN_PW_NS / NSEC_PER_USEC, SERVO_MAX_PW_NS / NSEC_PER_USEC);
		return -EINVAL;
	}

	set_servo_pulse_width(data->servo_sail_pwm, (unsigned long)pulse_width_us);
	data->current_servo_sail_pw = (unsigned long)pulse_width_us;
	return count;
}


// --- Platform Driver Probe Function ---
static int argo_radio_servo_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct argo_radio_servo_data *data;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	platform_set_drvdata(pdev, data);
	data->dev = dev;

	dev_info(dev, "Initializing for Allwinner H618. Built %s.\n", BUILD_DATE);

	// Capture module load timestamp
	data->module_load_ts = ktime_get();

	// --- Initialize state variables ---
	data->radio_rudder_last_pulse_ts = ktime_get();
	data->radio_sail_last_pulse_ts = ktime_get();
	data->current_servo_rudder_pw = 1500;
	data->current_servo_sail_pw = 1500;

	// --- 1. Request PWM devices using modern devm_pwm_get ---
	// This looks for PWMs with the labels "pwm2" and "pwm4" in the device tree,
	// which are provided by the standard armbian overlays (pi-pwm2, pi-pwm4).
	data->servo_rudder_pwm = devm_pwm_get(dev, "pwm2");
	if (IS_ERR(data->servo_rudder_pwm)) {
		ret = PTR_ERR(data->servo_rudder_pwm);
		dev_err(dev, "Failed to get PWM device for Servo Rudder (pwm2) (Error: %d).\n", ret);
		goto err_no_pwm_rudder;
	}
	dev_info(dev, "Successfully got PWM device: %s (pwm2, Servo Rudder)\n", data->servo_rudder_pwm->label);

	data->servo_sail_pwm = devm_pwm_get(dev, "pwm4");
	if (IS_ERR(data->servo_sail_pwm)) {
		ret = PTR_ERR(data->servo_sail_pwm);
		dev_err(dev, "Failed to get PWM device for Servo Sail (pwm4) (Error: %d).\n", ret);
		goto err_no_pwm_sail;
	}
	dev_info(dev, "Successfully got PWM device: %s (pwm4, Servo Sail)\n", data->servo_sail_pwm->label);

	// --- 2. Configure and Enable PWM outputs ---
	set_servo_pulse_width(data->servo_rudder_pwm, data->current_servo_rudder_pw);
	set_servo_pulse_width(data->servo_sail_pwm, data->current_servo_sail_pw);

	pwm_enable(data->servo_rudder_pwm);
	pwm_enable(data->servo_sail_pwm);
	dev_info(dev, "Enabled PWM2 and PWM4 outputs.\n");

	// --- 3. Setup GPIOs for Input Measurement from Device Tree ---
	data->radio_rudder_gpiod = devm_gpiod_get(dev, "radio_rudder", GPIOD_IN);
	if (IS_ERR(data->radio_rudder_gpiod)) {
		ret = PTR_ERR(data->radio_rudder_gpiod);
		dev_err(dev, "Failed to get radio_rudder_gpio (Error: %d).\n", ret);
		goto err_disable_pwm_outputs;
	}
	data->radio_rudder_irq = gpiod_to_irq(data->radio_rudder_gpiod);
	if (data->radio_rudder_irq < 0) {
		ret = data->radio_rudder_irq;
		dev_err(dev, "Failed to get IRQ for radio_rudder_gpio (Error: %d).\n", ret);
		goto err_disable_pwm_outputs;
	}
	ret = devm_request_irq(dev, data->radio_rudder_irq, radio_rudder_gpio_isr,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       "radio_rudder_irq", data);
	if (ret) {
		dev_err(dev, "Failed to request IRQ %d (Radio Rudder, Error: %d).\n", data->radio_rudder_irq, ret);
		goto err_disable_pwm_outputs;
	}
	dev_info(dev, "Radio Rudder GPIO mapped to IRQ %d.\n", data->radio_rudder_irq);

	data->radio_sail_gpiod = devm_gpiod_get(dev, "radio_sail", GPIOD_IN);
	if (IS_ERR(data->radio_sail_gpiod)) {
		ret = PTR_ERR(data->radio_sail_gpiod);
		dev_err(dev, "Failed to get radio_sail_gpio (Error: %d).\n", ret);
		goto err_disable_pwm_outputs;
	}
	data->radio_sail_irq = gpiod_to_irq(data->radio_sail_gpiod);
	if (data->radio_sail_irq < 0) {
		ret = data->radio_sail_irq;
		dev_err(dev, "Failed to get IRQ for radio_sail_gpio (Error: %d).\n", ret);
		goto err_disable_pwm_outputs;
	}
	ret = devm_request_irq(dev, data->radio_sail_irq, radio_sail_gpio_isr,
			       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			       "radio_sail_irq", data);
	if (ret) {
		dev_err(dev, "Failed to request IRQ %d (Radio Sail, Error: %d).\n", data->radio_sail_irq, ret);
		goto err_disable_pwm_outputs;
	}
	dev_info(dev, "Radio Sail GPIO mapped to IRQ %d.\n", data->radio_sail_irq);

	// --- 4. Initialize and start the high-resolution timer for periodic printk ---
	hrtimer_init(&data->print_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	data->print_timer.function = print_timer_callback;
	hrtimer_start(&data->print_timer, ktime_set(5, 0), HRTIMER_MODE_REL); // Fire after 5 seconds

	// --- 5. Create sysfs directory and files ---
	ret = kobject_init_and_add(&data->kobj, &argo_ktype, kernel_kobj, "argo_radio_servo");
	if (ret) {
		dev_err(dev, "Failed to initialize and add kobject.\n");
		goto err_timer_cancel;
	}

	ret = sysfs_create_group(&data->kobj, &argo_attr_group);
	if (ret) {
		dev_err(dev, "Failed to create sysfs group (Error: %d).\n", ret);
		kobject_put(&data->kobj);
		goto err_timer_cancel;
	}
	dev_info(dev, "Created sysfs entries at /sys/kernel/argo_radio_servo/.\n");

	return 0; // Initialization successful

// --- Error Handling and Cleanup (in reverse order of setup) ---
err_timer_cancel:
	hrtimer_cancel(&data->print_timer);
err_disable_pwm_outputs:
	// devm_* resources are freed automatically on probe failure
err_no_pwm_sail:
err_no_pwm_rudder:
	return ret;
}

// --- Platform Driver Remove Function ---
static int argo_radio_servo_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct argo_radio_servo_data *data = platform_get_drvdata(pdev);

	dev_info(dev, "Exiting...\n");

	// 1. Remove sysfs files and directory
	// kobject_put will remove the sysfs group and call the release function.
	kobject_put(&data->kobj);

	// 2. Cancel the high-resolution timer
	hrtimer_cancel(&data->print_timer);
	dev_info(dev, "Timer cancelled.\n");

	// 3. PWMs, GPIOs, and IRQs are managed by devm_* functions and are freed automatically.
	// pwm_disable is also called automatically for devm_pwm_get managed devices.

	dev_info(dev, "Cleaned up and exited successfully.\n");
	return 0;
}

static const struct of_device_id argo_radio_servo_of_match[] = {
	{ .compatible = "argo,radio-servo-gpio" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, argo_radio_servo_of_match);

static struct platform_driver argo_radio_servo_driver = {
	.probe = argo_radio_servo_probe,
	.remove = argo_radio_servo_remove,
	.driver = {
		.name = "argo_radio_servo",
		.of_match_table = argo_radio_servo_of_match,
	},
};

module_platform_driver(argo_radio_servo_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tobi Delbruck");
MODULE_DESCRIPTION("Allwinner H618 Argo Radio Servo Module for Pulse Measurement and Control");
MODULE_VERSION("0.4");
