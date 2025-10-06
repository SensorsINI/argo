# Argo Power Control System - ROS2 Node

This package provides a ROS2 node for the Argo power control system, which manages power button functionality, LED indicators, and system health monitoring.

## Hardware
- PI3 (pin 40) `!POW`: open-drain output to power relay (line 259)
- PI9 (pin 28) `!POW_BUT`: power button input (line 265)
- PH4 (pin 18) green LED (line 228)
- PI1 (pin 12) blue LED (line 257)
- PI3 (pin 40) shutdown pulse to external cutoff (line 259)
- PC12 (pin 36) !CHARGING from MP2672GD that indicates when CHARGING=0 that the MP2672GD is currently charging the battery (line 76)
- PH9 (pin 26) !ACOK from MP2672GD that indicates with ACOK=0 that the MP2672GD gets input power from USB (line 233)

## Power circuits
The board power relay (PRL3-5V-DC-1A)	is latched on by power button. The other coil in relay latches it off from the shutdown pulse on PI3. The latching is magnetic inside the relay.

The main PCB battery power (a 2S LiPo with fully-charged voltage about 8.V) is routed through a 7A fuse to the relay. The relay output is called MAIN. 

MAIN powers a 5V Traco switching regulator that supplies power called 5V. 

5V powers the OrangePi Zero 2W and peripheral circuits (MAX11612 ADC, Sparkfun_IMU_ICM-20948 IMU, Sparkfun_NEO-N9M-SMA GPS, 3x SDP32 custom anemometer). 

MAIN power also powers the servos and radio, via the servo cables. 

The OrangePi board 3.3V regulator called 3.3 supplies power to the humidity sensor (SHT5-ADB1B) and long range radio (RA-01 SX1278 LORA from ai-thinker).

## Critical low battery halting
In normal shutdown, the relay is turned off to completely disconnect MAIN and reduce battery consumption to a trickle of about 40uA consumed by the battery charger chip (MP2672GD). This relay disconnection is done by a special shutdown hook that runs as the very last operation before halt; see [`argo_poweroff.shutdown`](argo_poweroff.shutdown).

During normal battery-powered operation, if the battery voltage ever gets critically low (below about 6.5V), the OrangePi should be halted to conserve power, but the relay should *not* disconnect, because this would prevent sailing argo. If the OrangePi is halted, the PWM output pins are placed in high impedance mode and then the radio input pulses drive the servo output pins through resistors. That will allow sailing argo by hand.

Therefore, there needs to be a special shutdown mode that prevents the normal shutdown hook from running in the case of critical low battery halting. 

## Components
- `argo_power_control.py`: ROS2 node for button monitoring, LEDs, relay control, graceful shutdown, and recording control integration
- `argo_power_control.service`: systemd service that runs independently of argo-launch.service
- `argo_poweroff.shutdown`: late shutdown hook that asserts PI3 HIGH briefly to cut power (only during poweroff/halt, not reboot)
- `Makefile`: install helpers for the service and shutdown hook

## Important: Why not gpio-poweroff overlay on Zero 2W?
- The Orange Pi Zero 2W uses an AXP PMIC that registers `pm_power_off` first during boot.
- When `pm_power_off` is already registered, the kernel `gpio-poweroff` driver refuses to bind and logs:
  - `poweroff-gpio: gpio_poweroff_probe: pm_power_off function already registered`
- Result: any `gpio-poweroff` DT node (including overlays) will not take effect.
- Solution: a systemd late shutdown hook claims PI3 and pulses it HIGH just before poweroff. This avoids conflicts and is reliably last.
- The shutdown hook intelligently differentiates between shutdown modes:
  - **Poweroff/Halt**: Triggers power cut sequence (PI3 HIGH pulse)
  - **Reboot**: Skips power cut to allow normal restart

## Prerequisites
- Tools: `python3-libgpiod` or `gpiod`/`libgpiod-tools` (installed by `make check_deps`).

## Install
```bash
cd /home/orangepi/argo/power_control
make check_deps
make install_service enable_service
make install_shutdown_hook
```
- Service controls buttons/LEDs/relay; the shutdown hook handles final power cut on PI3.

## Test
- Manual hook pulse (poweroff mode):
```bash
/home/orangepi/argo/power_control/argo_poweroff.shutdown poweroff
```
- Manual hook test (reboot mode - should exit without pulse):
```bash
/home/orangepi/argo/power_control/argo_poweroff.shutdown reboot
```
- Shutdown test (watch PI3 on pin 40):
```bash
sync
sudo systemctl poweroff
```
You should see PI3 go HIGH for ~0.5 s just before power is cut.
- Reboot test (PI3 should remain LOW):
```bash
sync
sudo systemctl reboot
```
System should reboot normally without triggering power cut.

## CLI for the daemon
```bash
# help
python3 /home/orangepi/argo/launch/argo_power_control.py --help
# run normally
sudo /home/orangepi/argo/launch/argo_power_control.py
# test mode (no shutdown)
sudo /home/orangepi/argo/launch/argo_power_control.py --test-mode
# custom threshold
sudo /home/orangepi/argo/launch/argo_power_control.py --threshold 2.0
```

## GPIO quick refs (H616)
- PI3: 259, PI9: 265, PH4: 228, PI1: 257, PC12: 76, PH9: 233

## Uninstall
```bash
make clean_power_control
```
Removes service file and the shutdown hook.
