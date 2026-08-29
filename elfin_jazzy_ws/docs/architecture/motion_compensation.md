# Motion compensation

Both the RealSense D435 and the planned Livox Mid-360 are mounted on the
end-effector adapter, so every arm motion moves them with 6 degrees of freedom.
This document fixes how that motion is compensated before any geometry is
estimated.

## Two distinct errors

They have different causes and different fixes. Do not treat them as one
"drift" problem.

| Error | Cause | Fix |
|---|---|---|
| Intra-frame distortion | points inside one message were measured at different sensor poses | per-point deskew |
| Inter-frame misalignment | streams are dated differently, or a transform was taken at the wrong time | look up TF at each message's own stamp, pair inside a window |

A Mid-360 message spans close to a full scan period (about 100 ms at 10 Hz).
Transforming it once with `header.stamp` smears box edges and bends planes.
A depth image is much closer to a single instant and needs no per-pixel deskew,
but accumulating several of them with a single latest transform produces the
same smear.

## Pose source is forward kinematics, not the IMU

The sensor pose is a deterministic function of the joint angles:

```text
T_world_sensor(t) = FK(q(t)) * T_flange_sensor
```

- `q(t)` comes from `/joint_states`, published at 50 Hz by the joint state
  broadcaster ([elfin_controllers_sim.yaml](../../src/elfin_control/config/elfin_controllers_sim.yaml)).
- `T_flange_sensor` is a constant from the URDF, via `eef_mount_adapter`.
- Interpolate **positions**. Do not integrate the reported `velocity` field;
  the settle tests already document that it is unreliable on this arm.

Rules:

- Joint data must not be repackaged as a synthetic `/imu` topic and fed to an
  IMU-based deskew implementation. That discards an absolute pose and replaces
  it with something that has to be integrated, and therefore drifts.
- The number of joints is not the point; the whole base-to-sensor chain is. All
  six Elfin joints are required for a complete end-effector pose.
- The Livox IMU is a secondary, high-rate source for **attitude between joint
  samples only**. Position always comes from FK.

The preprocessor keeps a pose trajectory buffer (0.5-1 s, see
[sensor_data_pipeline.md](sensor_data_pipeline.md)) that answers
`T_world_sensor(t)` for arbitrary `t` by interpolation.

## Per-point transform

For a point measured at `t_i` and a chosen reference time `t_ref` (scan end, or
the camera stamp it is being paired with):

```text
p_ref = inv(T_world_sensor(t_ref)) * T_world_sensor(t_i) * p_i
```

Or directly to world, when the consumer works in world coordinates:

```text
p_world = T_world_sensor(t_i) * p_i
```

A point with no known `t_i` cannot be compensated. In that case the whole
message is treated as a single instant and the snapshot must carry
`deskewed=false`.

## Lidar: Mid-360 specifics

The sensor and the official driver perform **no ego-motion compensation**.
The firmware outputs ranges plus per-point timing; `livox_ros_driver2` only
packs them into messages at `publish_freq`. Deskew is our responsibility.

Facts that the implementation depends on:

- Non-repetitive scanning. There is no usable ring/line index for interpolation;
  use time, not azimuth.
- Per-point time is available as `offset_time` relative to `timebase` in the
  Livox `CustomMsg` (`xfer_format: 1`), or as the `timestamp` field of the
  `PointXYZRTLT` cloud (`xfer_format: 0`).
- The built-in IMU (ICM-40609) pushes at 200 Hz by default, on UDP port 56400
  to host port 56401, and can be toggled by `imu_data_en`.
- Protocol units are rad/s for the gyroscope and **g** for the accelerometer.
  Convert to m/s^2 before use.
- The IMU chip sits at (11.0, 23.29, -44.12) mm in the point-cloud frame. That
  lever arm is not negligible at centimetre tolerances; do not assume the IMU
  and the optical origin coincide.
- The lidar clock must be synchronised with the host (PTP or GPS) for the
  per-point times to be comparable with `/joint_states`.

Procedure: compute absolute `t_i` per point, interpolate `T_world_mid360(t_i)`,
transform, then declare the resulting frame in the snapshot.

## Depth: no per-pixel deskew

`rgbd_camera` and the D435 do not expose per-pixel readout times, so
interpolating across rows would be guessing at the sensor's internals.

- Single-frame detection: reproject with that frame's `CameraInfo`, drop
  non-finite values, transform once **at the frame stamp**.
- Multi-frame accumulation: transform each frame with its own stamp before
  fusing. Reusing the latest transform for older frames is what produces
  smeared maps that get misreported as drift.
- Flying-pixel rejection at depth discontinuities is allowed and is a different
  concern from deskew.

## TF lookups

Always look up transforms at the data stamp:

```python
# Correct
tf = buffer.lookup_transform(target, source, stamp, Duration(seconds=0.2))

# Wrong: silently uses the newest transform for older data
tf = buffer.lookup_transform(target, source, rclpy.time.Time())
```

`_transform_points_to_world` in
[luggage_detector_node.py](../../src/luggage_perception/scripts/luggage_detector_node.py)
is the reference implementation. A failed stamped lookup is a dropped frame,
not a reason to fall back to the latest transform.

## Compensation does not replace the motion gate

`MotionStabilityGate`
([motion_stability_filter.py](../../src/luggage_perception/luggage_perception/motion_stability_filter.py))
stays. Encoder quantisation, FK latency, gearbox backlash, and mount
flexibility all leave residuals that compensation cannot see.

- Keep a veto on `motion_score` after compensation. Thresholds may be looser
  than the "fully settled" gate used for the OctoMap relay, but they may not be
  removed.
- Detection at `pickup_observe` runs after settle, so depth needs no deskew
  there. That is a consequence of the gate, not a licence to skip stamped
  transforms.

## Simulation caveats

- Gazebo publishes no Mid-360 today. Nothing may assume the lidar stream exists.
- If a simulated lidar is added without per-point times, set `deskewed=false`
  and only mark `lidar_ok` after settle. Do not pretend a compensated cloud was
  produced.
