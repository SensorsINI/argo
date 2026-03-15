# Argo System Heartbeat LED Overlay

## 1. Purpose

This system provides a reliable, kernel-driven status LED on the `PH4` GPIO pin, which is connected to the green LED in the power button. It ensures that there is visual feedback (a "heartbeat" blink) from very early in the boot process, long before any userspace applications like `argo_power_control.py` have started.

## 2. Problem Solved

The main `argo_power_control.py` service starts relatively late in the boot sequence. This means that during the initial boot phase, there is no visual indication that the Orange Pi is booting correctly. This overlay solves that problem by handing control of the green LED to the Linux kernel itself, which starts managing it as soon as the device tree is loaded.

## 3. How It Works

This functionality is achieved through a combination of a Device Tree Overlay and the kernel's `sysfs` interface.

-   **Device Tree Overlay (`argo-ph4-led-overlay.dts`)**: This file declares the GPIO pin `PH4` as a system LED. It instructs the kernel's `gpio-leds` driver to take control of this pin.
-   **Kernel `heartbeat` Trigger**: The overlay configures the default behavior of this new LED to be the `heartbeat` trigger. This tells the kernel to automatically blink the LED at a regular interval to show that the system is alive and the kernel is running.

## 4. Integration with `argo_power_control.py`

This system is designed to provide the "best of both worlds": a kernel-driven boot indicator and application-specific patterns, both using the same green LED.

#### A. Taking Control (Application Start)

When the `argo_power_control.py` service starts, it needs to take exclusive control of the green LED to display its own complex patterns (e.g., for button presses, recording status, etc.). It does this by:

1.  Locating the kernel-managed LED in `sysfs`.
    - Preferred custom label: `/sys/class/leds/argo:green:heartbeat`
    - Compatibility label seen on some systems: `/sys/class/leds/green_led`
2.  Writing the string `none` to the `trigger` file within that directory:
    ```bash
    echo none > /sys/class/leds/<active-green-led>/trigger
    ```
3.  This action detaches the kernel's `heartbeat` trigger, allowing the LED to be controlled manually. The script can then turn the LED on or off by writing to the `brightness` file.

#### B. Releasing Control (Application Exit)

When `argo_power_control.py` shuts down cleanly (e.g., on service stop or system shutdown), its `cleanup()` function returns control to the kernel by:

1.  Writing the original trigger name, `heartbeat`, back to the `trigger` file:
    ```bash
    echo heartbeat > /sys/class/leds/<active-green-led>/trigger
    ```
2.  The kernel immediately resumes the heartbeat blinking pattern, ensuring the LED continues to indicate system status.

### LED Naming Note

The sysfs LED directory name is a label, not a different GPIO pin. Both `argo:green:heartbeat` and `green_led` can map to the same physical PH4 LED depending on overlay/boot configuration. The current `argo_power_control.py` and `argo-ph4-led-postinit.sh` handle both labels.

## 5. Installation and Management

The entire process is managed by the included `Makefile`.

-   **To Install**:
    ```bash
    make install
    ```
    This command will:
    1.  Compile the `argo-ph4-led-overlay.dts` file into a binary overlay (`.dtbo`).
    2.  Copy the compiled overlay to `/boot/overlays/`.
    3.  Run the centralized `manage_overlays.sh` script to safely add `argo-ph4-led` to the `user_overlays` line in `/boot/orangepiEnv.txt`.
    4.  A reboot is required for the changes to take effect.

-   **To Uninstall**:
    ```bash
    make uninstall
    ```
    This reverses the installation process, removing the overlay file and updating `orangepiEnv.txt`.

## 6. Hardware Configuration

-   **GPIO Pin**: `PH4`
-   **Physical Pin**: Pin 18 (on the 40-pin header)
-   **Device**: Green LED in the main power button (controlled via NFET).
-   **Activation**: The overlay configures the pin as `ACTIVE_HIGH` (NFET control: GPIO HIGH = LED ON, GPIO LOW = LED OFF).
