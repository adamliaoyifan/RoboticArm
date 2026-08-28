# Perception module architecture

How perception and planning code is split into layers, who owns state, and what
an algorithm class is allowed to hand out.

## Two layers

The split is by **role**, not by mixing eval into the live detect path.

| Layer | Role | May import | Owns |
|---|---|---|---|
| Algorithm module | the maths | numpy, stdlib, other algorithm modules | computation and state |
| Node module | the ROS surface | rclpy, messages, tf2, algorithm modules | topics, services, actions, parameters, TF |
| Eval module | N-trial gates, dumps, YOLO windows | algorithm modules, numpy, stdlib | reports; never a published pose |

Eval code lives in
[luggage_perception/eval/](../../src/luggage_perception/luggage_perception/eval/)
(`detection_accuracy`, `detection_gate_sampling`, `yolo_window_stats`).
Its tests are in
[test/eval/](../../src/luggage_perception/test/eval/).
Drivers (`scripts/detection_gt_gate_run.py`, `scripts/yolo_two_class_window.py`)
import that package. `DetectLuggage` must not.

The package must stay importable without a ROS environment. That is what makes
`test/test_*.py` runnable under plain `pytest`, and it is why
[luggage_box_estimator.py](../../src/luggage_perception/luggage_perception/luggage_box_estimator.py)
and [motion_stability_filter.py](../../src/luggage_perception/luggage_perception/motion_stability_filter.py)
have no ROS imports today.

Node modules may live in either place:

- `<pkg>/scripts/*.py`, installed via `install(PROGRAMS ...)`, as
  [luggage_detector_node.py](../../src/luggage_perception/scripts/luggage_detector_node.py) does; or
- `<pkg>/<pkg>/*.py` exposing `main()`, with a few-line executable wrapper in
  `<pkg>/scripts/`, as
  [scene_tf_publisher.py](../../src/luggage_description/luggage_description/scene_tf_publisher.py)
  and `zero_joint_state_publisher.py` do.

The second form is preferred for anything worth importing in a test. It comes
with one obligation: keep `rclpy`, `tf2_ros`, and message imports **inside** the
functions or the node class, so importing the package still works without ROS.

Rules:

- No `rclpy`, `rospy`, `tf2_ros`, or message imports in an algorithm module,
  including lazy imports inside a method. If a class needs a transform, the
  node looks it up and passes plain numbers in. Building `Marker` or
  `ColorRGBA` inside an algorithm class is a violation even when the import is
  deferred.
- No `rospy` in code that is built. A `rospy` file that is **not** listed in
  its package's `install(PROGRAMS ...)` is an unported ROS 1 reference: read
  it, port it, never extend it. `scripts/ros1_reference/` is one such holding
  area, but `luggage_planning/scripts/` and `luggage_packing/scripts/` still
  hold unported files directly.
- A node holds no algorithm state of its own. It converts messages to plain
  arrays, calls the class, converts the result back to messages.
- Nodes stay thin. If a node file grows past roughly 500 lines, the logic
  inside it belongs in an algorithm module. The ROS 1
  `motion_planner_node.py` (~3200 lines) is the anti-pattern being retired.

### Message adapter modules

The conversion a node performs may be lifted into its own module when it
grows past a few helpers. Such a module is **node-layer**, not an algorithm
module: it imports messages at the top level by design, and the "no ROS
imports" rule does not apply to it.

[ros_message_adapters.py](../../src/luggage_perception/luggage_perception/ros_message_adapters.py)
is the reference. It converts `sensor_msgs` to and from the structures in
[sensor_types.py](../../src/luggage_perception/luggage_perception/sensor_types.py)
and nothing else: no buffering, no pairing, no publishing. `SensorPreprocessor`
must never import it, which is what keeps the core testable without ROS.

Two obligations come with the split:

- The intermediate structure carries enough to rebuild an equivalent message.
  `CameraInfoFrame` holds `rectification` / `projection` verbatim for this
  reason; deriving them from `fx/fy/cx/cy` at encode time silently discards a
  driver's real rectification.
- A decoder returns `None` for a layout it does not support, and the node
  drops the message. It must not fall back to a guess. A structurally valid
  message carrying zero points is not an error and decodes to an empty array;
  conflating the two would let a dropout publish as valid geometry.

These modules are worth testing directly. Byte-level unpacking is where
layout bugs hide: `PointCloud2` field offsets inside `point_step`,
endianness, and truncated buffers are all invisible until a specific driver
sends a specific layout. See
[test_ros_message_adapters.py](../../src/luggage_perception/test/test_ros_message_adapters.py),
which guards the module behind `pytest.importorskip("sensor_msgs")` so the
ROS-free suite still runs under plain `pytest`.

## State ownership: update and copy out

An algorithm class that carries state across calls must expose exactly this
shape:

```python
class Foo:
    def __init__(self, ...):
        self._output = None          # private, never handed out

    def update(self, data, stamp, frame_id):
        """Compute and store. Returns nothing, or a small status value."""

    def copy_output(self):
        """Return an independent copy. Caller mutation must not corrupt state."""
```

- Internal buffers stay private (`_name`).
- Getters return copies. Returning `self._array` is a defect: the caller can
  mutate the class from outside, and two consumers then race.
  `MotionStabilityGate.diagnostics()` and `last_stats` already return `dict(...)`
  copies; follow that.
- `update` never publishes and never blocks on I/O.

Current violation to fix when touched:
[semantic_segmenter.py](../../src/luggage_perception/luggage_perception/semantic_segmenter.py)
exposes `instance_map` as a direct reference to `self._instance_map` and its
`segment(rgb_image)` neither takes nor returns a stamp.

## Every output carries a stamp and a frame

Any value that leaves an algorithm class and describes the world must carry:

- `stamp`: the acquisition time of the **input data**, not `now()`.
- `frame_id`: the coordinate frame the numbers are expressed in.

The `now()` prohibition applies to derived and forwarded data: a mask, a cloud,
a detection, or a republished image inherits the stamp of the measurement it
came from. A node that **originates** data with no upstream measurement may
stamp with the current clock; `zero_joint_state_publisher` synthesising a
`JointState` is the legitimate case. Republishers do not qualify:
[depth_image_republisher.py](../../src/luggage_gazebo/scripts/depth_image_republisher.py)
correctly copies `msg.header` through.

For 2D results (masks, bounding boxes) the frame is the image's optical frame
and the stamp is the image stamp. For 3D results the frame must be the frame
the points were actually transformed into, which is not always the frame in the
incoming header; see [sensor_data_pipeline.md](sensor_data_pipeline.md).

Publishing with `stamp = now()` breaks TF lookups and alignment downstream, and
it is invisible while the arm is parked. Do not do it, even in a stub.

## ROS interface surface

New topics, services, and actions go through `luggage_msgs`. The service versus
action split, timeouts, cancel semantics, and idempotency are already fixed in
[phase2_interfaces.md](../status/phase2_interfaces.md); this document does not
restate them. Short queries stay services (`DetectLuggage`, `GetCurrentBox`),
long motions are actions (`PlanMotion`, `GoToRobotPose`).

Internal per-frame structures such as `SyncedObservation` are **plain Python
objects, not ROS messages**. They exist inside the preprocessor process and
must not be added to `luggage_msgs`. The preprocessor node publishes a
synchronised set of standard sensor messages plus a JSON status string.
Downstream nodes subscribe to those topics; they do not import the snapshot
class.

Backend adapters (for example the Gazebo depth metre-to-millimetre
republisher) may rewrite a **single** stream. Cross-stream pairing stays in
the preprocessor.

## Node parameter conventions

- Declare every parameter with `declare_parameter` and a default; read it once
  into a private field in `__init__`.
- Anything that depends on the simulation backend behaving differently from
  hardware must be a parameter, not a constant. The Gazebo point-cloud
  `data_frame` (`camera_link` despite an optical header) is a preprocessor
  parameter; evidence lives in
  [m2_perception_occlusion_problem.md](../status/m2_perception_occlusion_problem.md).
