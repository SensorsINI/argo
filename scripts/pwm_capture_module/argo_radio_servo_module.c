// generated with help from https://gemini.google.com/app/19c8818984b5f80c
#include <linux/init.h>
#include <linux/module.h>       // Needed for all kernel modules
#include <linux/kernel.h>       // Needed for KERN_INFO, printk
#include <linux/platform_device.h> // For struct platform_device, used with device tree
#include <linux/of_gpio.h>      // For of_get_named_gpio
#include <linux/gpio/consumer.h> // For gpiod_get_from_of_node, gpiod_to_irq, gpiod_direction_input, etc. (modern GPIO API)
#include <linux/io.h>           // For ioremap, iounmap, readl, writel, used for PWM output registers
#include <linux/interrupt.h>    // For request_irq, free_irq, irqreturn_t
#include <linux/hrtimer.h>      // For high-resolution timers
#include <linux/ktime.h>        // For ktime_get(), ktime_to_ns, ktime_sub, ktime_compare
#include <linux/sysfs.h>        // For sysfs_create_file, sysfs_remove_file
#include <linux/kobject.h>      // For kobject, kobject_create_and_add, kobject_put
#include <linux/string.h>       // For sprintf, kstrtol
#include <linux/slab.h>         // For kzalloc, kfree
#include <linux/spinlock.h>     // For spinlock_t

/*
   argo_radio_servo_module.c:
   - Measures pulse width in microseconds on PI11 (PWM1_RADIO_RUDDER) and PI13 (PWM3_RADIO_SAIL)
     using GPIO interrupts and ktime_get().
   - Exposes measured values via sysfs:
     - /sys/argo_radio_servo/radio_rudder_pw_us (for PI11)
     - /sys/argo_radio_servo/radio_sail_pw_us (for PI13)
   - Allows writing pulse width values to control servos on PWM2 (PI12) and PWM4 (PI14).
   - Controls servo outputs via sysfs:
     - /sys/argo_radio_servo/servo_rudder_pw_us (for PWM2 - PI12)
     - /sys/argo_radio_servo/servo_sail_pw_us (for PWM4 - PI14)
*/

// --- GPIO Pin Definitions (Global GPIO numbers - VERIFY THESE) ---
// These are derived from PI11 (Bank I, Pin 11) and PI13 (Bank I, Pin 13).
// Bank I is typically GPIO chip 8 (A=0, B=1, ... I=8).
#define PI11_GLOBAL_GPIO_NUM (8 * 32 + 11) // = 267
#define PI13_GLOBAL_GPIO_NUM (8 * 32 + 13) // = 269
#define PI12_GLOBAL_GPIO_NUM (8 * 32 + 12) // = 268 (PWM2 Output)
#define PI14_GLOBAL_GPIO_NUM (8 * 32 + 14) // = 270 (PWM4 Output)

// --- PWM Output Register Offsets (from H618 User Manual) ---
#define ALLWINNER_PWM_BASE_ADDRESS    0x0300A000UL

// Clock configuration registers for PWM outputs
#define PCCR23_REG                    0x0024       // PWM23 Clock Configuration Register
#define PCCR45_REG                    0x0028       // PWM45 Clock Configuration Register
#define PER_REG                       0x0040       // PWM Enable Register (Global enable for PWM outputs)

// Channel N registers (PCR, PPR, PCNTR) for PWM2 (N=2) and PWM4 (N=4)
// PCR = 0x0060 + N*0x20
// PPR = 0x0064 + N*0x20
// PCNTR = 0x0068 + N*0x20

// PWM2 (N=2)
#define PWM_CH2_PCR_REG               (0x0060 + 2*0x0020)  // 0x00A0
#define PWM_CH2_PPR_REG               (0x0064 + 2*0x0020)  // 0x00A4
#define PWM_CH2_PCNTR_REG             (0x0068 + 2*0x0020)  // 0x00A8

// PWM4 (N=4)
#define PWM_CH4_PCR_REG               (0x0060 + 4*0x0020)  // 0x00E0
#define PWM_CH4_PPR_REG               (0x0064 + 4*0x0020)  // 0x00E4
#define PWM_CH4_PCNTR_REG             (0x0068 + 4*0x0020)  // 0x00E8

// Bit definitions for PWM Output Registers (Assumed based on PCR manual, verify in H618 manual)
// For PER (PWM Enable Register)
#define PWM_CH2_GLOBAL_OUTPUT_ENABLE  (1 << 2)     // Bit 2 enables PWM output for Channel 2
#define PWM_CH4_GLOBAL_OUTPUT_ENABLE  (1 << 4)     // Bit 4 enables PWM output for Channel 4

// For PCR (PWM Control Register) - per channel
#define PWM_PCR_PWM_PUL_START         (1 << 10)    // Set to 1 to start pulse output (clears automatically)
#define PWM_PCR_PWM_MODE_CYCLE        (0 << 9)     // 0: Cycle mode, 1: Pulse mode
#define PWM_PCR_PWM_ACT_STA_HIGH      (1 << 8)     // 0: Low active, 1: High active
#define PWM_PCR_PWM_PRESCAL_K(k)      ((k) & 0xFF) // Bits 7:0 for K, actual prescale (K+1)

// Clock config for PCCR (PWMxx Clock Configuration Register) - per group
#define PCCR_CLK_SRC_OSC24M           (0b00 << 7)  // Bits 8:7 = 00 for OSC24M
#define PCCR_CLK_GATING_PASS          (1 << 4)     // Bit 4 = 1 for Pass (enable clock)
#define PCCR_CLK_DIV_M_DIV(m)         (((m) & 0xF) << 0) // Bits 3:0 for M divisor (0=/1, 1=/2, etc.)

// --- Common Servo/PWM Parameters ---
#define SERVO_PERIOD_US               20000UL // 20ms period
#define SERVO_MIN_PW_US               900UL   // 900us minimum pulse width
#define SERVO_MAX_PW_US               2100UL  // 2100us maximum pulse width

// --- Global Variables for Input Measurement ---
static struct gpio_desc *radio_rudder_gpio_desc; // PI11
static unsigned int radio_rudder_irq_num;
static ktime_t radio_rudder_rising_edge_timestamp;
static long long radio_rudder_high_time_us = 0;

static struct gpio_desc *radio_sail_gpio_desc;   // PI13
static unsigned int radio_sail_irq_num;
static ktime_t radio_sail_rising_edge_timestamp;
static long long radio_sail_high_time_us = 0;

// --- Global Variables for Output Control ---
static void __iomem *pwm_output_regs_base; // Mapped base address for PWM output registers

// --- Shared Resources ---
static struct hrtimer print_timer; // High-resolution timer for periodic printk
static DEFINE_SPINLOCK(capture_data_lock); // Spinlock to protect measured data
static DEFINE_SPINLOCK(output_control_lock); // Spinlock to protect PWM output writes

static struct kobject *argo_radio_servo_kobj; // Kobject for /sys/argo_radio_servo directory

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

// Sysfs Show/Store Functions for Output Control
static ssize_t servo_rudder_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t servo_rudder_pw_us_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);
static ssize_t servo_sail_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static ssize_t servo_sail_pw_us_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);


// --- Sysfs Attribute Definitions ---
static struct kobj_attribute radio_rudder_pw_us_attribute =
    __ATTR(radio_rudder_pw_us, 0444, radio_rudder_pw_us_show, NULL);

static struct kobj_attribute radio_sail_pw_us_attribute =
    __ATTR(radio_sail_pw_us, 0444, radio_sail_pw_us_show, NULL);

static struct kobj_attribute servo_rudder_pw_us_attribute =
    __ATTR(servo_rudder_pw_us, 0664, servo_rudder_pw_us_show, servo_rudder_pw_us_store);

static struct kobj_attribute servo_sail_pw_us_attribute =
    __ATTR(servo_sail_pw_us, 0664, servo_sail_pw_us_show, servo_sail_pw_us_store);


// --- Input Measurement ISRs ---
static irqreturn_t radio_rudder_gpio_isr(int irq, void *dev_id)
{
    ktime_t current_time;
    unsigned long flags;
    bool current_gpio_state;

    current_time = ktime_get();
    current_gpio_state = gpiod_get_value(radio_rudder_gpio_desc);

    spin_lock_irqsave(&capture_data_lock, flags);

    if (current_gpio_state) { // GPIO is High (Rising Edge)
        radio_rudder_rising_edge_timestamp = current_time;
    } else { // GPIO is Low (Falling Edge)
        if (ktime_compare(current_time, radio_rudder_rising_edge_timestamp) > 0) {
            long long duration_ns = ktime_to_ns(ktime_sub(current_time, radio_rudder_rising_edge_timestamp));
            radio_rudder_high_time_us = duration_ns / 1000;
        } else {
            radio_rudder_high_time_us = 0;
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

    if (current_gpio_state) { // GPIO is High (Rising Edge)
        radio_sail_rising_edge_timestamp = current_time;
    } else { // GPIO is Low (Falling Edge)
        if (ktime_compare(current_time, radio_sail_rising_edge_timestamp) > 0) {
            long long duration_ns = ktime_to_ns(ktime_sub(current_time, radio_sail_rising_edge_timestamp));
            radio_sail_high_time_us = duration_ns / 1000;
        } else {
            radio_sail_high_time_us = 0;
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
    unsigned long flags;

    spin_lock_irqsave(&capture_data_lock, flags);
    rudder_pw = radio_rudder_high_time_us;
    sail_pw = radio_sail_high_time_us;
    spin_unlock_irqrestore(&capture_data_lock, flags);

    printk(KERN_INFO "Argo Radio Servo: Radio Rudder PW: %lld us, Radio Sail PW: %lld us\n",
           rudder_pw, sail_pw);

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

// --- Sysfs Show/Store Functions for Output Control ---
// Helper to set PWM Duty Cycle
static void set_pwm_duty_cycle(int channel_n, unsigned long pulse_width_us)
{
    u32 reg_val;
    unsigned long flags;
    u32 duty_count;
    u32 period_count;

    // Convert pulse width in microseconds to counter ticks
    // Assuming 1MHz counter clock (prescale K=23 from 24MHz OSC)
    // 1us = 1 tick.
    // Duty cycle is pulse_width_us. Period is SERVO_PERIOD_US.
    duty_count = (u32)pulse_width_us;
    period_count = (u32)SERVO_PERIOD_US;

    // Ensure duty_count does not exceed period_count
    if (duty_count > period_count) {
        duty_count = period_count;
    }

    spin_lock_irqsave(&output_control_lock, flags);

    if (channel_n == 2) { // PWM2
        writel(period_count, pwm_output_regs_base + PWM_CH2_PPR_REG); // Set Period
        writel(duty_count, pwm_output_regs_base + PWM_CH2_PCNTR_REG); // Set Duty Cycle
        // Trigger pulse start (if in pulse mode, or ensure cycle mode is active)
        // For cycle mode, writing to PCNTR/PPR should update the waveform.
        // If PWM_PCR_PWM_PUL_START is needed for continuous cycle, set it here.
        // It's R/W1S, clears automatically.
        // reg_val = readl(pwm_output_regs_base + PWM_CH2_PCR_REG);
        // reg_val |= PWM_PCR_PWM_PUL_START;
        // writel(reg_val, pwm_output_regs_base + PWM_CH2_PCR_REG);
    } else if (channel_n == 4) { // PWM4
        writel(period_count, pwm_output_regs_base + PWM_CH4_PPR_REG); // Set Period
        writel(duty_count, pwm_output_regs_base + PWM_CH4_PCNTR_REG); // Set Duty Cycle
        // Trigger pulse start as above if needed
        // reg_val = readl(pwm_output_regs_base + PWM_CH4_PCR_REG);
        // reg_val |= PWM_PCR_PWM_PUL_START;
        // writel(reg_val, pwm_output_regs_base + PWM_CH4_PCR_REG);
    }
    spin_unlock_irqrestore(&output_control_lock, flags);

    printk(KERN_INFO "Argo Radio Servo: Set PWM%d duty to %lu us (duty_count %u, period_count %u)\n",
           channel_n, pulse_width_us, duty_count, period_count);
}


static ssize_t servo_rudder_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    // For output PWM, we can read the current PCNTR value if the hardware exposes it.
    // Or return a stored software value if we tracked it.
    // For simplicity, let's return a nominal value or 0 if not set.
    return sprintf(buf, "%lu\n", SERVO_MIN_PW_US + (SERVO_MAX_PW_US - SERVO_MIN_PW_US) / 2); // Return a default center value
}

static ssize_t servo_rudder_pw_us_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    long pulse_width_us;
    int ret;

    ret = kstrtol(buf, 10, &pulse_width_us);
    if (ret) {
        return ret;
    }

    if (pulse_width_us < SERVO_MIN_PW_US || pulse_width_us > SERVO_MAX_PW_US) {
        printk(KERN_WARNING "Argo Radio Servo: Servo Rudder pulse width %ld out of range (%lu-%lu us).\n",
               pulse_width_us, SERVO_MIN_PW_US, SERVO_MAX_PW_US);
        return -EINVAL;
    }

    set_pwm_duty_cycle(2, (unsigned long)pulse_width_us); // Channel 2
    return count;
}

static ssize_t servo_sail_pw_us_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    return sprintf(buf, "%lu\n", SERVO_MIN_PW_US + (SERVO_MAX_PW_US - SERVO_MIN_PW_US) / 2); // Return a default center value
}

static ssize_t servo_sail_pw_us_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    long pulse_width_us;
    int ret;

    ret = kstrtol(buf, 10, &pulse_width_us);
    if (ret) {
        return ret;
    }

    if (pulse_width_us < SERVO_MIN_PW_US || pulse_width_us > SERVO_MAX_PW_US) {
        printk(KERN_WARNING "Argo Radio Servo: Servo Sail pulse width %ld out of range (%lu-%lu us).\n",
               pulse_width_us, SERVO_MIN_PW_US, SERVO_MAX_PW_US);
        return -EINVAL;
    }

    set_pwm_duty_cycle(4, (unsigned long)pulse_width_us); // Channel 4
    return count;
}


// --- Module Initialization Function ---
static int __init argo_radio_servo_init(void)
{
    int ret;
    struct device_node *np_pio, *np_pwm;
    u32 reg_val; // For writing to PWM registers

    printk(KERN_INFO "Argo Radio Servo Module: Initializing for Allwinner H618.\n");

    // --- 1. Map PWM Output Registers ---
    // The PWM module is at a single base address for all channels.
    np_pwm = of_find_compatible_node(NULL, NULL, "allwinner,sun50i-h616-pwm");
    if (!np_pwm) {
        printk(KERN_ERR "Argo Radio Servo: Failed to find PWM device tree node for output.\n");
        return -ENODEV;
    }
    pwm_output_regs_base = of_iomap(np_pwm, 0);
    if (!pwm_output_regs_base) {
        printk(KERN_ERR "Argo Radio Servo: Failed to ioremap PWM output registers.\n");
        of_node_put(np_pwm);
        return -EFAULT;
    }
    of_node_put(np_pwm);
    printk(KERN_INFO "Argo Radio Servo: Mapped PWM output registers at %p.\n", pwm_output_regs_base);

    // --- 2. Configure PWM Output Channels (PWM2 and PWM4) ---
    // Assuming 24MHz OSC clock, pre-scale K=23 (for 1MHz counter clock, 24MHz / (23+1) = 1MHz)
    // Period = 20000 ticks for 20ms.
    u32 pwm_prescal_k = 23; // K=23 for (K+1)=24 prescaler to get 1MHz from 24MHz
    u32 pwm_period_count = SERVO_PERIOD_US; // 20000 ticks for 20ms at 1MHz counter
    u32 pwm_initial_duty_count = 1500; // 1500us default center for servos

    // 2.1 Configure PCCR23 (for PWM2 & PWM3)
    reg_val = PCCR_CLK_SRC_OSC24M | PCCR_CLK_GATING_PASS | PCCR_CLK_DIV_M_DIV(0); // Divisor 0 for /1 (not directly used by PCR prescaler)
    writel(reg_val, pwm_output_regs_base + PCCR23_REG);
    printk(KERN_INFO "Argo Radio Servo: Configured PCCR23. Value written: 0x%08x. Current: 0x%08x\n", reg_val, readl(pwm_output_regs_base + PCCR23_REG));

    // 2.2 Configure PCCR45 (for PWM4 & PWM5)
    reg_val = PCCR_CLK_SRC_OSC24M | PCCR_CLK_GATING_PASS | PCCR_CLK_DIV_M_DIV(0); // Divisor 0 for /1
    writel(reg_val, pwm_output_regs_base + PCCR45_REG);
    printk(KERN_INFO "Argo Radio Servo: Configured PCCR45. Value written: 0x%08x. Current: 0x%08x\n", reg_val, readl(pwm_output_regs_base + PCCR45_REG));

    // 2.3 Configure PWM2 (Servo Rudder)
    spin_lock_irqsave(&output_control_lock, flags);
    writel(PWM_PCR_PWM_MODE_CYCLE | PWM_PCR_PWM_ACT_STA_HIGH | PWM_PCR_PWM_PRESCAL_K(pwm_prescal_k),
           pwm_output_regs_base + PWM_CH2_PCR_REG);
    writel(pwm_period_count, pwm_output_regs_base + PWM_CH2_PPR_REG);
    writel(pwm_initial_duty_count, pwm_output_regs_base + PWM_CH2_PCNTR_REG);
    spin_unlock_irqrestore(&output_control_lock, flags);
    printk(KERN_INFO "Argo Radio Servo: Configured PWM2 (Servo Rudder). PCR: 0x%08x, PPR: 0x%08x, PCNTR: 0x%08x\n",
           readl(pwm_output_regs_base + PWM_CH2_PCR_REG),
           readl(pwm_output_regs_base + PWM_CH2_PPR_REG),
           readl(pwm_output_regs_base + PWM_CH2_PCNTR_REG));


    // 2.4 Configure PWM4 (Servo Sail)
    spin_lock_irqsave(&output_control_lock, flags);
    writel(PWM_PCR_PWM_MODE_CYCLE | PWM_PCR_PWM_ACT_STA_HIGH | PWM_PCR_PWM_PRESCAL_K(pwm_prescal_k),
           pwm_output_regs_base + PWM_CH4_PCR_REG);
    writel(pwm_period_count, pwm_output_regs_base + PWM_CH4_PPR_REG);
    writel(pwm_initial_duty_count, pwm_output_regs_base + PWM_CH4_PCNTR_REG);
    spin_unlock_irqrestore(&output_control_lock, flags);
    printk(KERN_INFO "Argo Radio Servo: Configured PWM4 (Servo Sail). PCR: 0x%08x, PPR: 0x%08x, PCNTR: 0x%08x\n",
           readl(pwm_output_regs_base + PWM_CH4_PCR_REG),
           readl(pwm_output_regs_base + PWM_CH4_PPR_REG),
           readl(pwm_output_regs_base + PWM_CH4_PCNTR_REG));

    // 2.5 Globally Enable PWM Outputs (PWM2 and PWM4) in PER_REG
    reg_val = readl(pwm_output_regs_base + PER_REG);
    reg_val |= (PWM_CH2_GLOBAL_OUTPUT_ENABLE | PWM_CH4_GLOBAL_OUTPUT_ENABLE);
    writel(reg_val, pwm_output_regs_base + PER_REG);
    printk(KERN_INFO "Argo Radio Servo: Enabled PWM2 and PWM4 outputs via PER. Current PER: 0x%08x\n", readl(pwm_output_regs_base + PER_REG));


    // --- 3. Setup GPIOs for Input Measurement (PI11 and PI13) ---
    // PIO controller node for fetching GPIOs.
    np_pio = of_find_compatible_node(NULL, NULL, "allwinner,sun50i-h616-pinctrl");
    if (!np_pio) {
        printk(KERN_ERR "Argo Radio Servo: Failed to find pinctrl device tree node for GPIO inputs.\n");
        ret = -ENODEV;
        goto err_unmap_pwm;
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
    if (!radio_rudder_gpio_desc) {
        printk(KERN_ERR "Argo Radio Servo: Failed to get GPIO descriptor for %d (Radio Rudder).\n", PI11_GLOBAL_GPIO_NUM);
        ret = -ENODEV;
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
    if (!radio_sail_gpio_desc) {
        printk(KERN_ERR "Argo Radio Servo: Failed to get GPIO descriptor for %d (Radio Sail).\n", PI13_GLOBAL_GPIO_NUM);
        ret = -ENODEV;
        goto err_free_rs_irq;
    }
    printk(KERN_INFO "Argo Radio Servo: Radio Sail (GPIO %d) mapped to IRQ %d.\n", PI13_GLOBAL_GPIO_NUM, radio_sail_irq_num);

    of_node_put(np_pio); // Finished with pio node

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
    ret = sysfs_create_file(argo_radio_servo_kobj, &servo_rudder_pw_us_attribute.attr);
    if (ret) {
        printk(KERN_ERR "Argo Radio Servo: Failed to create sysfs file servo_rudder_pw_us (Error: %d).\n", ret);
        goto err_remove_rs_sysfs;
    }
    ret = sysfs_create_file(argo_radio_servo_kobj, &servo_sail_pw_us_attribute.attr);
    if (ret) {
        printk(KERN_ERR "Argo Radio Servo: Failed to create sysfs file servo_sail_pw_us (Error: %d).\n", ret);
        goto err_remove_sr_sysfs;
    }
    printk(KERN_INFO "Argo Radio Servo: Created sysfs entry at /sys/argo_radio_servo/.\n");

    return 0; // Initialization successful

// --- Error Handling and Cleanup (in reverse order of setup) ---
err_remove_sr_sysfs:
    sysfs_remove_file(argo_radio_servo_kobj, &servo_rudder_pw_us_attribute.attr);
err_remove_rs_sysfs:
    sysfs_remove_file(argo_radio_servo_kobj, &radio_sail_pw_us_attribute.attr);
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
    of_node_put(np_pio); // Only put if it was successfully found
err_unmap_pwm:
    if (pwm_output_regs_base) iounmap(pwm_output_regs_base);
    return ret;
}

// --- Module Exit Function ---
static void __exit argo_radio_servo_exit(void)
{
    u32 reg_val;

    printk(KERN_INFO "Argo Radio Servo Module: Exiting...\n");

    // 1. Disable PWM outputs and unmap registers
    if (pwm_output_regs_base) {
        // Disable PWM2 and PWM4 outputs in PER_REG
        reg_val = readl(pwm_output_regs_base + PER_REG);
        reg_val &= ~(PWM_CH2_GLOBAL_OUTPUT_ENABLE | PWM_CH4_GLOBAL_OUTPUT_ENABLE);
        writel(reg_val, pwm_output_regs_base + PER_REG);
        printk(KERN_INFO "Argo Radio Servo: Disabled PWM2 and PWM4 outputs.\n");

        iounmap(pwm_output_regs_base);
        printk(KERN_INFO "Argo Radio Servo: Unmapped PWM output registers.\n");
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
    sysfs_remove_file(argo_radio_servo_kobj, &servo_sail_pw_us_attribute.attr);
    sysfs_remove_file(argo_radio_servo_kobj, &servo_rudder_pw_us_attribute.attr);
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
