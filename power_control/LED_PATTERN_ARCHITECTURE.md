# LED Pattern Architecture Explanation

## Overview

The LED control system has two types of patterns:
1. **Main Heartbeat Pattern** - Continuous status indication (runs in `green_led_heartbeat()` thread)
2. **Special Patterns** - Override patterns (SOS, charging, shutdown) that pause the heartbeat

## Main Heartbeat Pattern

### Pattern Generation (`_generate_led_pattern()`)
- Creates a **10-character string** representing 10 slots
- Characters: `'g'` (green), `'r'` (red), `'b'` (blue), `'.'` (off)
- Example: `"ggrrrr...."` = 2 green, 4 red, 4 off slots

### Pattern Execution (`_execute_led_pattern()`)
- Executes over **10 slots**, each slot = **0.3 seconds total**
- Each slot structure:
  - **ON period**: 0.03s (10% duty cycle) - LED should be ON if pattern says so
  - **OFF period**: 0.27s (90% duty cycle) - All LEDs should be OFF
- Total cycle duration: 10 slots × 0.3s = **3 seconds**

### State Tracking
- Tracks LED states for **both ON and OFF periods** within each slot
- Total states tracked: **20** (10 slots × 2 periods each)
- Summary format (exactly 20 characters per LED, excluding the "R:"/"G:"/"B:" prefix):
  ```
  R:----r-r-r-r--------
  G:g-g-----------------
  B:--------------------
  ```
- Each pair of characters = one slot: `[ON_period][OFF_period]`
- Characters: `r`/`g`/`b` = LED ON, `-` = LED OFF
- Example for pattern "ggrrrr...." (10 slots):
  - **Green LED tracking:**
    - Slot 0: g → ON period: `g`, OFF period: `-` → `g-`
    - Slot 1: g → ON period: `g`, OFF period: `-` → `g-`
    - Slots 2-9: . → ON period: `-`, OFF period: `-` → `----------------` (16 dashes)
    - Result: `g-g-----------------` = **20 characters** (2+2+16)
  - **Red LED tracking:**
    - Slots 0-1: . → ON period: `-`, OFF period: `-` → `----` (4 dashes)
    - Slots 2-5: r → ON period: `r`, OFF period: `-` → `r-r-r-r-` (8 chars: r-, r-, r-, r-)
    - Slots 6-9: . → ON period: `-`, OFF period: `-` → `--------` (8 dashes)
    - Result: `----r-r-r-r--------` = **20 characters** (4+8+8)

## Special Patterns (Override Main Heartbeat)

These patterns run in **separate threads** and **pause the heartbeat**:

1. **SOS Pattern** (`sos_led_pattern()`)
   - Runs when: Low battery detected
   - Controls: Red LED only
   - Calls: `pause_heartbeat("low_battery")` → `resume_heartbeat()` when done

2. **Charging State Pattern** (`charge_state_led_pattern()`)
   - Runs when: AC power present and charging
   - Controls: Blue LED (duty cycle = charge percentage)
   - Calls: `pause_heartbeat("charging_state")` → `resume_heartbeat()` when done

3. **Shutdown Pattern** (`shutdown_led_pattern()`)
   - Runs when: Shutdown sequence initiated
   - Controls: All LEDs (short-long pattern)
   - Calls: `pause_heartbeat()` → heartbeat never resumes (system shutting down)

## Pattern Flow

```
Main Loop (green_led_heartbeat):
  while running:
    if heartbeat_paused:
      sleep(0.1)  # Wait for special pattern to finish
      continue
    
    pattern = _generate_led_pattern()  # "ggrrrr...."
    _execute_led_pattern(pattern)       # Execute over 10 slots (3 seconds)
    # Loop repeats
```

## Key Points

1. **Main heartbeat** generates and executes 10-slot patterns continuously
2. **Each slot** = 0.3s total (0.03s ON + 0.27s OFF)
3. **Special patterns** pause the heartbeat and take direct control
4. **State tracking** shows 20 states (10 slots × 2 periods) to detect if LEDs stay ON during OFF periods
5. **Pattern string** is 10 characters, but execution tracks 20 states (ON+OFF for each slot)
