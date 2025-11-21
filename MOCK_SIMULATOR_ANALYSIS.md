# Mock Simulator Analysis: Stall Decay and Tacking Logic

## 1. Stall Decay Rate Usage ✅

**Location**: Lines 283-286 in `mock_sailboat_simulator.py`

```python
if effective_no_go:
    adjustment = np.clip(speed_diff * 1.5 * self.dt,
                         -self.stall_decay_rate * self.dt,
                         self.stall_decay_rate * self.dt)
```

**Status**: ✅ **CORRECTLY USED**

When the boat is in the no-go zone (`effective_no_go = True`):
- Target speed is set to 0.1 m/s (line 263)
- Speed adjustment is clamped by `stall_decay_rate * dt`
- With `stall_decay_rate = 1.5 m/s²`, the boat can decelerate at up to 1.5 m/s²
- This means at 10 Hz (dt=0.1s), max deceleration per step is 0.15 m/s

**Your change from 0.5 to 1.5**: Makes the boat decelerate 3x faster when stalled, which is more realistic for a sailboat losing momentum in irons.

---

## 2. Tacking Logic Analysis

### 2.1 Tack Initiation (Lines 178-205)

**Conditions for starting a tack**:
1. `abs(rudder_angle) > 0.25` - Significant rudder input (25% deflection)
2. `boat_speed > 0.35 m/s` - Minimum speed threshold
3. `abs(wind_angle) < 57°` - Within no-go zone + 12° buffer (45° + 12°)

**Issue**: The tack can start when wind angle is 0-57°, which includes the no-go zone. This is realistic - you initiate tacks from close-hauled.

### 2.2 Tack Execution (Lines 210-256)

**During tack**:
- `effective_no_go = False` (line 256) - Boat can continue through no-go zone
- Speed is NOT clamped by stall_decay_rate (uses normal speed adjustment)
- Turn rate is modified by `tack_turn_boost` (line 321)

**Tack Completion Conditions** (lines 235-242):
1. **`crossing_complete`**: 
   - Wind angle >= 57° (45° + 12° exit buffer)
   - AND wind sign flipped (crossed from port to starboard or vice versa)
   
2. **`sign_flip_without_clearance`**:
   - Wind sign flipped
   - AND wind angle >= 45° (just cleared no-go zone)
   - AND previous sign != current sign (detected the flip)

**Potential Issues**:

#### Issue 1: `tack_turn_boost = 0.8` is a REDUCTION, not a boost! ⚠️

**Location**: Line 321
```python
if in_tack_window:
    turn_rate *= self.tack_turn_boost  # 0.8 = 20% REDUCTION
```

**Problem**: During a tack, you want MORE turn rate, not less. A value of 0.8 reduces turn rate by 20%, making tacks slower and more likely to fail.

**Recommendation**: Should be > 1.0 (e.g., 1.5-2.0) to increase turn rate during tacks.

#### Issue 2: Tack completion logic may be too strict

The tack completes only when:
- Wind angle >= 57° AND sign flipped, OR
- Sign flipped AND angle >= 45° AND previous sign detected

**Problem**: If the boat turns quickly, it might miss the "previous_sign != current_sign" check and the tack might not complete properly.

#### Issue 3: Speed during tack

During tack, `effective_no_go = False`, so:
- Target speed is calculated normally (line 275)
- Speed adjustment is NOT clamped (line 288)
- Boat can maintain speed through the tack

**This is realistic** - boats maintain momentum during tacks, but the logic doesn't account for speed loss when crossing the wind.

---

## 3. Realistic Behavior Assessment

### ✅ Realistic Aspects:
1. **Stall decay**: Correctly applies deceleration in no-go zone
2. **Tack initiation**: Requires speed and rudder input
3. **Tack failure**: Detects low speed and timeout conditions
4. **Fall-off behavior**: When stalled, boat turns toward downwind (lines 297-315)

### ⚠️ Potentially Unrealistic Aspects:
1. **Turn rate reduction during tack**: `tack_turn_boost = 0.8` reduces turn rate when it should increase
2. **Speed maintenance**: Boat doesn't lose speed when crossing through the wind during tack
3. **Tack completion detection**: Complex logic might miss completion in edge cases

---

## 4. Recommendations (IMPLEMENTED ✅)

1. **✅ Fix `tack_turn_boost`**: Changed from 0.8 to 1.8 to increase turn rate by 80% during tacks
2. **✅ Add speed loss during tack**: Apply `stall_decay_rate` deceleration when crossing through the no-go zone during a tack
   - Logic: `if effective_no_go or (in_tack_window and is_actually_in_no_go)`
   - This causes speed to drop when wind angle < 45° during tack, making tacks more likely to fail if not executed properly
3. **Simplify tack completion**: Consider using just wind sign flip + minimum angle as completion criteria (not implemented)

## 5. Changes Made

### argo.yaml
- `tack_turn_boost`: 0.8 → 1.8 (increased for more aggressive tacks)

### mock_sailboat_simulator.py
- Lines 283-290: Added deceleration during tack when in no-go zone
- Now applies `stall_decay_rate` deceleration when:
  - Normally stalled (`effective_no_go = True`), OR
  - During tack AND actually in no-go zone (`in_tack_window and abs_apparent < no_go_angle_deg`)

**Expected behavior**: Tacks will now lose speed when crossing through the wind (wind angle < 45°), making them more likely to fail if:
- Not enough initial speed
- Turn rate too slow
- Takes too long to cross through the wind

This should make the simulator more realistic - tacks can now fail due to speed loss, not just timeout.

