# Navigation Drift Executive Summary

## Key Issue

The robot appears to know its own pose, but object positions on the map are disorganized and recovery from a bad start takes too long.

This is not a simple TF loss. The current system has a frame and map alignment mismatch between:

- Cartographer localization on `officeMap.pbstream`
- the planner using `officeMap.yaml`
- EKF fused odometry published on `/odom`

## Core Root Cause

Cartographer is being forced by launch parameters to publish `map -> base_link`, while the system should instead publish `map -> odom` and let EKF provide `odom -> base_link`.

That means the localization chain is not properly anchored to the EKF odometry frame, so object pose references and planner map alignment become inconsistent.

## Why It Matters

- Robot pose may look correct locally, but the global map reference drifts.
- Static planner obstacles and live Cartographer localization do not share the same origin.
- Recovery is slow because the system searches the wrong transform chain during relocalization.

## Current Data Flow Problem

### Actual flow today

```
[LiDAR] -> RF2O -> EKF -> /odom
                      \-> base_link
                            |
                        Cartographer
                            |
                       map -> base_link
                            |
                       move_base / map_server
```

### Desired flow

```
[LiDAR] -> RF2O -> EKF -> /odom -> Cartographer
                      \-> base_link        |
                                                 v
                                             map -> odom
                                                 |
                                         move_base / map_server
```

## Recommended Fixes

1. Change `carto_localization.launch` so Cartographer publishes `map -> odom`.
2. Keep Cartographer config `provide_odom_frame = false` and `published_frame = "odom"`.
3. Ensure `move_base` uses a planner map generated from the same saved map used by Cartographer.
4. Publish a known initial pose at startup to speed relocalization.

## Expected Outcome

- Stable `map -> odom -> base_link` pose chain
- Consistent object positions between Cartographer localization and the planner map
- Faster relocalization and reduced map pose disorder
- More reliable navigation planning and execution

## Action Plan

- Inspect `carto_localization.launch` and remove the `published_frame = base_link` override.
- Confirm `limo_localization_fast.lua` and `limo_2d_reloc.lua` publish `odom` only.
- Verify `map_server` loads the same map source as Cartographer.
- Validate with `tf_echo map odom` and `tf_echo odom base_link`.
- If needed, use Cartographer’s live occupancy grid instead of a separate static YAML map.

## Management Summary

Fixing the frame anchor and map source alignment is the highest-value change. Once Cartographer is properly anchored through EKF odometry and the planner map is unified, object poses will appear organized and relocalization will recover much more quickly.