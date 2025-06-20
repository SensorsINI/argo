// generated with help from https://gemini.google.com/app/19c8818984b5f80c
#include <linux/init.h>
#include <linux/module.h>       // Needed for all kernel modules
#include <linux/kernel.h>       // Needed for KERN_INFO, printk
#include <linux/platform_device.h> // For struct platform_device, used with device tree
#include <linux/of_gpio.h>      // For of_get_named_gpio
#include <linux/gpio/consumer.h> // For gpiod_get_from_of_node, gpiod_to_irq, gpiod_direction_input, etc. (modern GPIO API)
#include <linux/io.h>           // For ioremap, iounmap, readl, writel (still needed for some non-pwm_get cases if any)
#include <linux/interrupt.h>    // For request_irq, free_irq, irqreturn_t
#include <linux/hrtimer.h>      // For high-resolution timers
#include <linux/ktime.h>        // For ktime_get(), ktime_to_ns, ktime_sub, ktime_compare
#include <linux/sysfs.h>        // For sysfs_create_file, sysfs_remove_file
#include <linux/kobject.h>      // For kobject, kobject_create_and_add, kobject_put
#include <linux/string.h>       // For sprintf, kstrtol
#include <linux/slab.h>         // For kzalloc, kfree
#include <linux/spinlock.h>     // For spinlock_t
#include <linux/of_address.h>   // For of_iomap (though pwm_get replaces much of this for PWM)
#include <linux/pwm.h>          // NEW: For pwm_get, pwm_config, pwm_enable/disable, pwm_put
#include <linux/err.h>          // For IS_ERR, PTR_ERR

/*
   argo_radio_servo_module.c:
    -signals are as follows 
     - PI11 (GPIO267, pin 32) is used for Radio Rudder input (PWM1_RADIO_RUDDER)
     - PI13 (GPIO269, pin 7) is used for Radio Sail input (PWM3_RADIO_SAIL)
     - PI12 (PWM2, pin 33) is used for Servo Rudder output (PWM2_SERVO_RUDDER)
     - PI14 (PWM4, pin 16) is used for Servo Sail output (PWM4_SERVO_SAIL)
   - Measures pulse width in microseconds on PI11 (PWM1_RADIO_RUDDER) and PI13 (PWM3_RADIO_SAIL)
     using GPIO interrupts and ktime_get(). If no pulses for 1s, sets PW to 0.
   - Exposes measured values via sysfs:
     - /sys/argo_radio_servo/radio_rudder_pw_us (for PI11)
     - /sys/argo_radio_servo/radio_sail_pw_us (for PI13)
     - These files are group and owner readable (0444), allowing user-space scripts to read radio input pulse widths.
   - Controls servo outputs on PWM2_SERVO_RUDDER (PI12) and PWM4_SERVO_SAIL (PI14) via kernel's standard PWM API.
    - Servo frequency is fixed to 50Hz (20ms period).
    - Uses PWM framework to set pulse widths for Servo Rudder and Servo Sail.
   - Exposes servo output pulse width values via sysfs:
     - /sys/argo_radio_servo/servo_rudder_pw_us (for PI12, PWM2)
     - /sys/argo_radio_servo/servo_sail_pw_us (for PI14, PWM4)
     - These files are group and owner writable (0664), allowing user-space scripts to write servo output pulse widths.
        but sysfs files in /sys/kernel/pwm/pwmchipX/pwmY/ are used for testing.
     - PWM2 (PI12) is used for Servo Rudder, PWM4 (PI14) is used for Servo Sail.
     
     - The standard sysfs files for PWM output are not created by this module.
   - Our sysfs files for output are created for simpler control by user program, but the actual control is done via the kernel's PWM framework.
     /sys/class/pwm/pwmchipX/pwmY/period and /duty_cycle.
   - Uses high-resolution timer to print pulse widths every 5 seconds for the first minute after module load.
   - Uses spinlocks to protect access to shared data (pulse widths and timestamps).
   - Uses GPIO interrupts for both PI11 and PI13 to measure pulse widths.
   - Uses ktime for high-resolution time measurement.
   - Uses kernel's PWM framework to control servo outputs.
   - Uses device tree for GPIO and PWM configuration.
   - Uses modern GPIO API (gpiod_get_value, gpiod_to_irq, gpiod_direction_input, etc.)
   - Uses kobject for sysfs entry creation.
   - Uses ktime for high-resolution time measurement.
   - Uses kstrtol for converting sysfs input to long integers.
   - Uses printk for logging.
   - Uses module_init and module_exit for module loading and unloading.
   - Uses module parameters for configurable pulse widths (not implemented yet, but can be added).  
   - Prints measured pulse width to dmesg only for the first minute after module insertion.
*/

// --- GPIO Pin Definitions (Global GPIO numbers - VERIFY THESE) ---
// These are derived from PI11 (Bank I, Pin 11) and PI13 (Bank I, Pin 13).
// Bank I is typically GPIO chip 8 (A=0, B=1, ... I=8).
#define PI11_GLOBAL_GPIO_NUM (8 * 32 + 11) // = 267
#define PI13_GLOBAL_GPIO_NUM (8 * 32 + 13) // = 269
// Note: PI12 and PI14 are now controlled via PWM framework, not direct GPIO request by this module

// --- Common Servo/PWM Parameters ---
#define SERVO_PERIOD_NS               20000000UL // 20ms period in nanoseconds
#define SERVO_MIN_PW_NS               900000UL   // 900us minimum pulse width in nanoseconds
#define SERVO_MAX_PW_NS               2100000UL  // 2100us maximum pulse width in nanoseconds
#define NO_PULSE_TIMEOUT_NS           (1 * NSEC_PER_SEC) // 1 second in nanoseconds

// --- Global Variables for Input Measurement ---
static struct gpio_desc *radio_rudder_gpio_desc; // PI11
static unsigned int radio_rudder_irq_num;
static ktime_t radio_rudder_rising_edge_timestamp;
static long long radio_rudder_high_time_us = 0;
static ktime_t radio_rudder_last_pulse_timestamp; // Timestamp of the last detected edge


static struct gpio_desc *radio_sail_gpio_desc;   // PI13
static unsigned int radio_sail_irq_num;
static ktime_t radio_sail_rising_edge_timestamp;
static long long radio_sail_high_time_us = 0;
static ktime_t radio_sail_last_pulse_timestamp;   // Timestamp of the last detected edge

// --- Global Variables for PWM Output Control (via kernel PWM framework) ---
static struct pwm_device *servo_rudder_pwm_dev; // PWM device for Servo Rudder (PWM2, PI12)
static struct pwm_device *servo_sail_pwm_dev;   // PWM device for Servo Sail (PWM4, PI14)

// store the current write values (for sysfs show)
// These are only for the test script's display now, not for actual sysfs files.
static unsigned long current_servo_rudder_pw = 1500; // Initialize to default center
static unsigned long current_servo_sail_pw = 1500;   // Initialize to default center

// --- Shared Resources ---
static struct hrtimer print_timer; // High-resolution timer for periodic printk
static DEFINE_SPINLOCK(capture_data_lock); // Spinlock to protect measured data
static DEFINE_SPINLOCK(output_control_lock); // Spinlock to protect PWM output writes

static struct kobject *argo_radio_servo_kobj; // Kobject for /sys/argo_radio_servo directory

// --- Variable for timed printk ---
static ktime_t module_load_timestamp;

// --- Function Prototypes ---
static int __init argo_radio_servo_init(void);
static void __exit argo_radio_servo_exit(void);

// Input Measurement ISRs
static irqreturn_t radio_rudder_gpio_isr(int irq, void *dev_id);
static irqreturn_t radio_sail_gpio_isr(int irq, void *dev_id);

// Periodic Printk
static enum hrtimer_restart print_timer_callback(struct hrtimer *timer);

// Sysfs Show Functions for Input Measurement
static ssize_t radio_rudder_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t radio_sail_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);


// --- Sysfs Attribute Definitions ---
static struct kobj_attribute radio_rudder_pw_us_attribute =
    __ATTR(radio_rudder_pw_us, 0664, radio_rudder_pw_us_show, NULL); // rw-rw-r--

static struct kobj_attribute radio_sail_pw_us_attribute =
    __ATTR(radio_sail_pw_us, 0664, radio_sail_pw_us_show, NULL); // rw-rw-r--


// --- Input Measurement ISRs ---
static irqreturn_t radio_rudder_gpio_isr(int irq, void *dev_id)
{
    ktime_t current_time;
    unsigned long flags;
    bool current_gpio_state;

    current_time = ktime_get();
    current_gpio_state = gpiod_get_value(radio_rudder_gpio_desc);

    spin_lock_irqsave(&capture_data_lock, flags);
    radio_rudder_last_pulse_timestamp = current_time; // Update last pulse timestamp

    if (current_gpio_state) { // GPIO is High (Rising Edge)
        radio_rudder_rising_edge_timestamp = current_time;
    } else { // GPIO is Low (Falling Edge)
        if (ktime_compare(current_time, radio_rudder_rising_edge_timestamp) > 0) {
            long long duration_ns = ktime_to_ns(ktime_sub(current_time, radio_rudder_rising_edge_timestamp));
            radio_rudder_high_time_us = duration_ns / 1000;
        } else {
            radio_rudder_high_time_us = 0; // Reset if invalid sequence
            printk(KERN_WARNING "Radio Rudder: Spurious falling edge or falling edge before rising edge.\n");
        }
    }

    spin_unlock_irqrestore(&capture_data_lock, flags);
    return IRQ_HANDLED;
}

static irqreturn_t radio_sail_gpio_isr(int irq, void *dev_id)
{
    ktime_t current_time;
    unsigned long flags;
    bool current_gpio_state;

    current_time = ktime_get();
    current_gpio_state = gpiod_get_value(radio_sail_gpio_desc);

    spin_lock_irqsave(&capture_data_lock, flags);
    radio_sail_last_pulse_timestamp = current_time; // Update last pulse timestamp

    if (current_gpio_state) { // GPIO is High (Rising Edge)
        radio_sail_rising_edge_timestamp = current_time;
    } else { // GPIO is Low (Falling Edge)
        if (ktime_compare(current_time, radio_sail_rising_edge_timestamp) > 0) {
            long long duration_ns = ktime_to_ns(ktime_sub(current_time, radio_sail_rising_edge_timestamp));
            radio_sail_high_time_us = duration_ns / 1000;
        } else {
            radio_sail_high_time_us = 0; // Reset if invalid sequence
            printk(KERN_WARNING "Radio Sail: Spurious falling edge or falling edge before rising edge.\n");
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
    s64 elapsed_ns = ktime_to_ns(ktime_sub(current_time, module_load_timestamp));

    spin_lock_irqsave(&capture_data_lock, flags);

    // Check for timeout for Radio Rudder
    if (ktime_to_ns(ktime_sub(current_time, radio_rudder_last_pulse_timestamp)) > NO_PULSE_TIMEOUT_NS) {
        if(radio_rudder_high_time_us != 0){ // Check if it was non-zero before clearing
            if (elapsed_ns < (s64)60 * NSEC_PER_SEC) { // Only print if within the first minute
                printk(KERN_INFO "Argo Radio Servo: Radio rudder pulses disappeared, setting PW to 0.\n");
            }
        }
        radio_rudder_high_time_us = 0;
    }
    rudder_pw = radio_rudder_high_time_us;

    // Check for timeout for Radio Sail
    if (ktime_to_ns(ktime_sub(current_time, radio_sail_last_pulse_timestamp)) > NO_PULSE_TIMEOUT_NS) {
        if(radio_sail_high_time_us != 0){ // Check if it was non-zero before clearing
            if (elapsed_ns < (s64)60 * NSEC_PER_SEC) { // Only print if within the first minute
                printk(KERN_INFO "Argo Radio Servo: Radio sail pulses disappeared, setting PW to 0.\n");
            }
        }
        radio_sail_high_time_us = 0;
    }
    sail_pw = radio_sail_high_time_us;

    spin_unlock_irqrestore(&capture_data_lock, flags);

    // Only print periodic updates if within the first minute
    if (elapsed_ns < (s64)60 * NSEC_PER_SEC) {
        printk(KERN_INFO "Argo Radio Servo: Radio Rudder PW: %lld us, Radio Sail PW: %lld us\n",
               rudder_pw, sail_pw);
    }

    hrtimer_forward_now(timer, ktime_set(5, 0)); // Fire every 5 seconds
    return HRTIMER_RESTART;
}

// --- Sysfs Show Functions for Input Measurement ---
static ssize_t radio_rudder_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    long long current_high_time;
    unsigned long flags;

    spin_lock_irqsave(&capture_data_lock, flags);
    current_high_time = radio_rudder_high_time_us;
    spin_unlock_irqrestore(&capture_data_lock, flags);

    return sprintf(buf, "%lld\n", current_high_time);
}

static ssize_t radio_sail_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    long long current_high_time;
    unsigned long flags;

    spin_lock_irqsave(&capture_data_lock, flags);
    current_high_time = radio_sail_high_time_us;
    spin_unlock_irqrestore(&capture_data_lock, flags);

    return sprintf(buf, "%lld\n", current_high_time);
}

// --- Sysfs Show/Store Functions for Output Control (via kernel PWM framework) ---
static void set_servo_pulse_width(struct pwm_device *pwm_dev, unsigned long pulse_width_us)
{
    unsigned long flags; // Declared at the top of the function
    u64 pulse_width_ns = pulse_width_us * NSEC_PER_USEC; // Convert to nanoseconds
    u64 period_ns = SERVO_PERIOD_NS;

    // Sanity check pulse_width
    if (pulse_width_ns < SERVO_MIN_PW_NS) pulse_width_ns = SERVO_MIN_PW_NS;
    if (pulse_width_ns > SERVO_MAX_PW_NS) pulse_width_ns = SERVO_MAX_PW_NS;

    spin_lock_irqsave(&output_control_lock, flags);

    // Configure the PWM device. This applies period and duty cycle.
    pwm_config(pwm_dev, pulse_width_ns, period_ns);

    spin_unlock_irqrestore(&output_control_lock, flags);

    printk(KERN_INFO "Argo Radio Servo: Set PWM device %s to %lu us (duty_ns %llu, period_ns %llu)\n",
           pwm_dev->label, pulse_width_us, pulse_width_ns, period_ns);
}


// These functions are now *only* for the test script's internal logic,
// and do NOT create sysfs files. They control the actual PWM devices.
// They are kept as static to compile, but their __ATTR are removed.
static ssize_t servo_rudder_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%lu\n", current_servo_rudder_pw);
}

static ssize_t servo_rudder_pw_us_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    long pulse_width_us;
    int ret;

    ret = kstrtol(buf, 10, &pulse_width_us);
    if (ret) return ret;

    if (pulse_width_us < (long)SERVO_MIN_PW_NS / NSEC_PER_USEC ||
        pulse_width_us > (long)SERVO_MAX_PW_NS / NSEC_PER_USEC) {
        printk(KERN_WARNING "Argo Radio Servo: Servo Rudder pulse width %ld out of range (%lu-%lu us).\n",
               pulse_width_us, SERVO_MIN_PW_NS / NSEC_PER_USEC, SERVO_MAX_PW_NS / NSEC_PER_USEC);
        return -EINVAL;
    }

    // Call the internal helper to configure the PWM device
    set_servo_pulse_width(servo_rudder_pwm_dev, (unsigned long)pulse_width_us);
    current_servo_rudder_pw = (unsigned long)pulse_width_us; // Update stored value
    return count;
}

// These functions are now *only* for the test script's internal logic,
static ssize_t servo_sail_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%lu\n", current_servo_sail_pw);
}

static ssize_t servo_sail_pw_us_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    long pulse_width_us;
    int ret;

    ret = kstrtol(buf, 10, &pulse_width_us);
    if (ret) return ret;

    if (pulse_width_us < (long)SERVO_MIN_PW_NS / NSEC_PER_USEC ||
        pulse_width_us > (long)SERVO_MAX_PW_NS / NSEC_PER_USEC) {
        printk(KERN_WARNING "Argo Radio Servo: Servo Sail pulse width %ld out of range (%lu-%lu us).\n",
               pulse_width_us, SERVO_MIN_PW_NS / NSEC_PER_USEC, SERVO_MAX_PW_NS / NSEC_PER_USEC);
        return -EINVAL;
    }

    // Call the internal helper to configure the PWM device
    set_servo_pulse_width(servo_sail_pwm_dev, (unsigned long)pulse_width_us);
    current_servo_sail_pw = (unsigned long)pulse_width_us; // Update stored value
    return count;
}


// --- Module Initialization Function ---
static int __init argo_radio_servo_init(void)
{
    int ret;
    struct device_node *np_pio; // np_pwm is no longer directly used for pwm_get
    // unsigned long flags; // Declared at the top of the function for spin_lock_irqsave, but unused here


    printk(KERN_INFO "Argo Radio Servo Module: Initializing for Allwinner H618.\n");

    // Capture module load timestamp
    module_load_timestamp = ktime_get();

    // --- Initialize last_pulse_timestamp for inputs ---
    radio_rudder_last_pulse_timestamp = ktime_get();
    radio_sail_last_pulse_timestamp = ktime_get();


    // --- 1. Get PWM devices via kernel framework ---
    // For PWM2 (Servo Rudder)
    // The name "pwm-2" or "2" depends on how the base DT exposes the PWM channels.
    // Common labels are "pwmchip0:pwm-0", "pwmchip0:pwm-1" etc.
    // Try by index if name fails: pwm_get(NULL, 2)
    // Using a specific consumer device node (e.g., &platform_device->dev) is preferred over NULL for robust drivers.
    // For this test module, NULL (global lookup) is acceptable.
    servo_rudder_pwm_dev = pwm_get(NULL, "pwm2"); // Try getting by name "pwm2"
    if (IS_ERR(servo_rudder_pwm_dev)) {
        printk(KERN_ERR "Argo Radio Servo: Failed to get PWM device for Servo Rudder (PWM2) (Error: %ld). Trying by index.\n", PTR_ERR(servo_rudder_pwm_dev));
        servo_rudder_pwm_dev = pwm_get(NULL, "2"); // Try getting by index "2" (channel index 2)
        if (IS_ERR(servo_rudder_pwm_dev)) {
            printk(KERN_ERR "Argo Radio Servo: Failed to get PWM device for Servo Rudder (PWM2) by index (Error: %ld).\n", PTR_ERR(servo_rudder_pwm_dev));
            ret = PTR_ERR(servo_rudder_pwm_dev);
            goto err_no_pwm_rudder;
        }
    }
    printk(KERN_INFO "Argo Radio Servo: Successfully got PWM device: %s (PWM2, Servo Rudder)\n", servo_rudder_pwm_dev->label);


    // For PWM4 (Servo Sail)
    servo_sail_pwm_dev = pwm_get(NULL, "pwm4"); // Try getting by name "pwm4"
    if (IS_ERR(servo_sail_pwm_dev)) {
        printk(KERN_ERR "Argo Radio Servo: Failed to get PWM device for Servo Sail (PWM4) (Error: %ld). Trying by index.\n", PTR_ERR(servo_sail_pwm_dev));
        servo_sail_pwm_dev = pwm_get(NULL, "4"); // Try getting by index "4" (channel index 4)
        if (IS_ERR(servo_sail_pwm_dev)) {
            printk(KERN_ERR "Argo Radio Servo: Failed to get PWM device for Servo Sail (PWM4) by index (Error: %ld).\n", PTR_ERR(servo_sail_pwm_dev));
            ret = PTR_ERR(servo_sail_pwm_dev);
            goto err_no_pwm_sail;
        }
    }
    printk(KERN_INFO "Argo Radio Servo: Successfully got PWM device: %s (PWM4, Servo Sail)\n", servo_sail_pwm_dev->label);

    // --- 2. Configure and Enable PWM outputs ---
    // Set initial period and duty cycle, then enable
    set_servo_pulse_width(servo_rudder_pwm_dev, current_servo_rudder_pw);
    set_servo_pulse_width(servo_sail_pwm_dev, current_servo_sail_pw);

    // Enable the PWM outputs
    pwm_enable(servo_rudder_pwm_dev);
    pwm_enable(servo_sail_pwm_dev);
    printk(KERN_INFO "Argo Radio Servo: Enabled PWM2 and PWM4 outputs.\n");

    // --- 3. Setup GPIOs for Input Measurement (PI11 and PI13) ---
    // PIO controller node for fetching GPIOs.
    np_pio = of_find_compatible_node(NULL, NULL, "allwinner,sun50i-h616-pinctrl");
    if (!np_pio) {
        printk(KERN_ERR "Argo Radio Servo: Failed to find pinctrl device tree node for GPIO inputs.\n");
        ret = -ENODEV;
        goto err_disable_pwm_outputs; // Added cleanup for PWM outputs
    }

    // 3.1 Setup Radio Rudder (PI11)
    ret = gpio_request(PI11_GLOBAL_GPIO_NUM, "radio_rudder_pw");
    if (ret) {
        printk(KERN_ERR "Argo Radio Servo: Failed to request GPIO %d (Radio Rudder, Error: %d).\n", PI11_GLOBAL_GPIO_NUM, ret);
        goto err_put_pio_node;
    }
    ret = gpio_direction_input(PI11_GLOBAL_GPIO_NUM);
    if (ret) {
        printk(KERN_ERR "Argo Radio Servo: Failed to set GPIO %d as input (Radio Rudder, Error: %d).\n", PI11_GLOBAL_GPIO_NUM, ret);
        goto err_free_rr_gpio;
    }
    radio_rudder_irq_num = gpio_to_irq(PI11_GLOBAL_GPIO_NUM);
    if (radio_rudder_irq_num < 0) {
        printk(KERN_ERR "Argo Radio Servo: Failed to get IRQ for GPIO %d (Radio Rudder, Error: %d).\n", PI11_GLOBAL_GPIO_NUM, radio_rudder_irq_num);
        ret = radio_rudder_irq_num;
        goto err_free_rr_gpio;
    }
    ret = request_irq(radio_rudder_irq_num, radio_rudder_gpio_isr,
                      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED,
                      "radio_rudder_irq", (void *)PI11_GLOBAL_GPIO_NUM); // Use GPIO num as dev_id
    if (ret) {
        printk(KERN_ERR "Argo Radio Servo: Failed to request IRQ %d (Radio Rudder, Error: %d).\n", radio_rudder_irq_num, ret);
        goto err_free_rr_gpio;
    }
    radio_rudder_gpio_desc = gpio_to_desc(PI11_GLOBAL_GPIO_NUM);
    if (IS_ERR(radio_rudder_gpio_desc)) { // Check for error using IS_ERR
        printk(KERN_ERR "Argo Radio Servo: Failed to get GPIO descriptor for %d (Radio Rudder) (Error: %ld).\n", PI11_GLOBAL_GPIO_NUM, PTR_ERR(radio_rudder_gpio_desc));
        ret = PTR_ERR(radio_rudder_gpio_desc); // Get error code from pointer
        goto err_free_rr_irq;
    }
    printk(KERN_INFO "Argo Radio Servo: Radio Rudder (GPIO %d) mapped to IRQ %d.\n", PI11_GLOBAL_GPIO_NUM, radio_rudder_irq_num);


    // 3.2 Setup Radio Sail (PI13)
    ret = gpio_request(PI13_GLOBAL_GPIO_NUM, "radio_sail_pw");
    if (ret) {
        printk(KERN_ERR "Argo Radio Servo: Failed to request GPIO %d (Radio Sail, Error: %d).\n", PI13_GLOBAL_GPIO_NUM, ret);
        goto err_free_rr_irq; // If radio rudder IRQ was requested, free it here
    }
    ret = gpio_direction_input(PI13_GLOBAL_GPIO_NUM);
    if (ret) {
        printk(KERN_ERR "Argo Radio Servo: Failed to set GPIO %d as input (Radio Sail, Error: %d).\n", PI13_GLOBAL_GPIO_NUM, ret);
        goto err_free_rs_gpio;
    }
    radio_sail_irq_num = gpio_to_irq(PI13_GLOBAL_GPIO_NUM);
    if (radio_sail_irq_num < 0) {
        printk(KERN_ERR "Argo Radio Servo: Failed to get IRQ for GPIO %d (Radio Sail, Error: %d).\n", PI13_GLOBAL_GPIO_NUM, radio_sail_irq_num);
        ret = radio_sail_irq_num;
        goto err_free_rs_gpio;
    }
    ret = request_irq(radio_sail_irq_num, radio_sail_gpio_isr,
                      IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED,
                      "radio_sail_irq", (void *)PI13_GLOBAL_GPIO_NUM); // Use GPIO num as dev_id
    if (ret) {
        printk(KERN_ERR "Argo Radio Servo: Failed to request IRQ %d (Radio Sail, Error: %d).\n", radio_sail_irq_num, ret);
        goto err_free_rs_gpio;
    }
    radio_sail_gpio_desc = gpio_to_desc(PI13_GLOBAL_GPIO_NUM);
    if (IS_ERR(radio_sail_gpio_desc)) { // Check for error using IS_ERR
        printk(KERN_ERR "Argo Radio Servo: Failed to get GPIO descriptor for %d (Radio Sail) (Error: %ld).\n", PI13_GLOBAL_GPIO_NUM, PTR_ERR(radio_sail_gpio_desc));
        ret = PTR_ERR(radio_sail_gpio_desc); // Get error code from pointer
        goto err_free_rs_irq;
    }
    printk(KERN_INFO "Argo Radio Servo: Radio Sail (GPIO %d) mapped to IRQ %d.\n", PI13_GLOBAL_GPIO_NUM, radio_sail_irq_num);

    if (np_pio) of_node_put(np_pio); // Only put if it was successfully found

    // --- 4. Initialize and start the high-resolution timer for periodic printk ---
    hrtimer_init(&print_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    print_timer.function = print_timer_callback;
    hrtimer_start(&print_timer, ktime_set(5, 0), HRTIMER_MODE_REL); // Fire after 5 seconds

    // --- 5. Create sysfs directory and files ---
    argo_radio_servo_kobj = kobject_create_and_add("argo_radio_servo", kernel_kobj);
    if (!argo_radio_servo_kobj) {
        printk(KERN_ERR "Argo Radio Servo: Failed to create sysfs kobject 'argo_radio_servo'.\n");
        ret = -ENOMEM;
        goto err_timer_cancel;
    }

    ret = sysfs_create_file(argo_radio_servo_kobj, &radio_rudder_pw_us_attribute.attr);
    if (ret) {
        printk(KERN_ERR "Argo Radio Servo: Failed to create sysfs file radio_rudder_pw_us (Error: %d).\n", ret);
        goto err_remove_kobj;
    }
    ret = sysfs_create_file(argo_radio_servo_kobj, &radio_sail_pw_us_attribute.attr);
    if (ret) {
        printk(KERN_ERR "Argo Radio Servo: Failed to create sysfs file radio_sail_pw_us (Error: %d).\n", ret);
        goto err_remove_rr_sysfs;
    }
    // Servo output control is now via standard /sys/class/pwm, not custom sysfs files from this module.
    // The attributes servo_rudder_pw_us_attribute and servo_sail_pw_us_attribute are no longer created.
    printk(KERN_INFO "Argo Radio Servo: Created sysfs entry at /sys/argo_radio_servo/ for input measurements.\n");

    return 0; // Initialization successful

// --- Error Handling and Cleanup (in reverse order of setup) ---
err_remove_rr_sysfs:
    sysfs_remove_file(argo_radio_servo_kobj, &radio_rudder_pw_us_attribute.attr);
err_remove_kobj:
    kobject_put(argo_radio_servo_kobj);
err_timer_cancel:
    hrtimer_cancel(&print_timer);
err_free_rs_irq:
    if (radio_sail_irq_num > 0) free_irq(radio_sail_irq_num, (void *)PI13_GLOBAL_GPIO_NUM);
err_free_rs_gpio:
    gpio_free(PI13_GLOBAL_GPIO_NUM);
err_free_rr_irq:
    if (radio_rudder_irq_num > 0) free_irq(radio_rudder_irq_num, (void *)PI11_GLOBAL_GPIO_NUM);
err_free_rr_gpio:
    gpio_free(PI11_GLOBAL_GPIO_NUM);
err_put_pio_node:
    if (np_pio) of_node_put(np_pio); // Only put if it was successfully found
err_disable_pwm_outputs: // Cleanup for PWM output devices
    if (servo_sail_pwm_dev) {
        pwm_disable(servo_sail_pwm_dev);
        pwm_put(servo_sail_pwm_dev);
    }
err_no_pwm_sail:
    if (servo_rudder_pwm_dev) {
        pwm_disable(servo_rudder_pwm_dev);
        pwm_put(servo_rudder_pwm_dev);
    }
err_no_pwm_rudder:
    return ret;
}

// --- Module Exit Function ---
static void __exit argo_radio_servo_exit(void)
{
    printk(KERN_INFO "Argo Radio Servo Module: Exiting...\n");

    // 1. Disable and free PWM output devices
    if (servo_sail_pwm_dev) {
        pwm_disable(servo_sail_pwm_dev);
        pwm_put(servo_sail_pwm_dev);
        printk(KERN_INFO "Argo Radio Servo: Released PWM4 (Servo Sail).\n");
    }
    if (servo_rudder_pwm_dev) {
        pwm_disable(servo_rudder_pwm_dev);
        pwm_put(servo_rudder_pwm_dev);
        printk(KERN_INFO "Argo Radio Servo: Released PWM2 (Servo Rudder).\n");
    }

    // 2. Cancel the high-resolution timer
    hrtimer_cancel(&print_timer);
    printk(KERN_INFO "Argo Radio Servo: Timer cancelled.\n");

    // 3. Free GPIOs and IRQs for input measurements
    if (radio_sail_irq_num > 0) free_irq(radio_sail_irq_num, (void *)PI13_GLOBAL_GPIO_NUM);
    gpio_free(PI13_GLOBAL_GPIO_NUM);
    printk(KERN_INFO "Argo Radio Servo: Freed GPIO %d (Radio Sail) and IRQ %d.\n", PI13_GLOBAL_GPIO_NUM, radio_sail_irq_num);

    if (radio_rudder_irq_num > 0) free_irq(radio_rudder_irq_num, (void *)PI11_GLOBAL_GPIO_NUM);
    gpio_free(PI11_GLOBAL_GPIO_NUM);
    printk(KERN_INFO "Argo Radio Servo: Freed GPIO %d (Radio Rudder) and IRQ %d.\n", PI11_GLOBAL_GPIO_NUM, radio_rudder_irq_num);

    // 4. Remove sysfs files and directory
    sysfs_remove_file(argo_radio_servo_kobj, &radio_sail_pw_us_attribute.attr);
    sysfs_remove_file(argo_radio_servo_kobj, &radio_rudder_pw_us_attribute.attr);
    kobject_put(argo_radio_servo_kobj);
    printk(KERN_INFO "Argo Radio Servo: Removed sysfs entry.\n");

    printk(KERN_INFO "Argo Radio Servo Module: Cleaned up and exited successfully.\n");
}

// --- Module Information and Entry/Exit Points ---
module_init(argo_radio_servo_init);
module_exit(argo_radio_servo_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tobi Delbruck");
MODULE_DESCRIPTION("Allwinner H618 Argo Radio Servo Module for Pulse Measurement and Control");
MODULE_VERSION("0.1");
MODULE_ALIAS("platform:argo_radio_servo"); // For platform device matching
