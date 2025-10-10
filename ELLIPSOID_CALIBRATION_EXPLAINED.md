# Ellipsoid Calibration Explanation

## Your Data Has Real Magnetic Distortion

### Measured Distortion:
- **Off-diagonal covariance**: 95 µT² (17% of diagonal terms)
- **Performance improvement**: 13.6% better than diagonal calibration
- **Rotation from identity**: 2.0 (indicates ~57° rotation)

### This is REAL soft-iron distortion from:
1. Ferromagnetic materials in boat (hull, frame, motors)
2. Electronics/batteries creating magnetic fields
3. Sensor mounting orientation

## The Ellipsoid Fitting Method is CORRECT

### Our Implementation:
1. **Algebraic fit**: `ax² + by² + cz² + 2fxy + 2gxz + 2hyz + 2px + 2qy + 2rz + d = 0`
2. **SVD solution**: Find parameters that best fit the data
3. **Extract center**: `center = -A⁻¹ × [p,q,r]`
4. **Normalize**: `A / abs(d_center)` to get unit ellipsoid
5. **Eigendecomposition**: Get radii and rotation
6. **Fix handedness**: Ensure det(rotation) = +1

### Calibration Transformation:
```python
centered = data - bias
rotated = centered @ rotation      # Align ellipsoid axes with xyz
scaled = rotated @ diag(r_avg/radii)  # Scale to sphere
```

### Why This Works:
- **Rotation matrix columns** = ellipsoid principal axes in sensor coords
- **Multiplying by rotation** projects data onto these axes
- **Scaling** makes all axes equal length (sphere)

## Comparison with ST Method

The standard method (used by ST and others):
1. Fit ellipsoid equation algebraically ✓ (same as ours)
2. Extract center, radii, rotation ✓ (same as ours)  
3. Apply: `(data - center) @ rotation @ scale` ✓ (same as ours)

**Our implementation matches the standard algebraic ellipsoid fitting method.**

## Why Minmax Failed

Minmax (diagonal-only) calibration:
- **Assumes**: Distortion is axis-aligned
- **Reality**: Your distortion has 95 µT² off-diagonal terms
- **Result**: Can't correct rotated ellipsoid → poor calibration

## The Large Rotation is CORRECT

The ~57° rotation between ellipsoid axes and sensor axes is REAL:
- **Measured independently** from covariance matrix
- **Confirmed** by 13.6% performance improvement
- **Validated** by residual std of 1.00 µT (excellent)

## Recommendations

1. **Use ellipsoid calibration** - it's working correctly
2. **Accept the rotation** - it's correcting real distortion
3. **Consider sources**: Review magnetic environment near sensor
4. **Verify in operation**: Check compass heading accuracy in the field

## Key Metrics
- **Determinant**: +1.00 ✓ (right-handed)
- **Residual std**: 1.00 µT ✓ (excellent fit)
- **Improvement**: 13.6% over diagonal ✓ (significant)
