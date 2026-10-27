# Worker integration

Workers implement `uran.worker.jsonl.v1` through the installed
`uran-worker-sdk`. Runtime instances use
`uran_worker_adapter/WorkerProcessPlugin` and configure an explicit executable
array such as:

```yaml
class_type: uran_worker_adapter/WorkerProcessPlugin
config:
  command: ["/opt/uran/venvs/cyberdog2/bin/uran-cyberdog2-worker"]
  message_types: ["uran.motion.command.v1"]
  capabilities: ["motion.base.velocity", "motion.base.safety"]
  worker_config:
    backend: simulation
```

Production paths use absolute executables from an installed, locked venv. They
do not source this repository. The simulator must be selected explicitly in a
Profile and its results carry `simulation: true`.

## SO101 arm

`motion/so101` is a ROS-independent, Python 3.6-compatible SO101 worker. It
accepts Cartesian velocity and pose commands, `gripper_action(toggle)`, and
`safety_action`; public joint-angle control is deliberately absent. Its fixed
period controller uses spatial DLS IK, measured feedback, joint rate limits,
a monotonic watchdog, and STL-derived capsule/OBB self-collision checks.

Use the robot's completed LeRobot calibration JSON directly; unrelated safety
profiles never create or override calibration. Torque starts disabled. Use a
persistent `/dev/serial/by-id/...` path in the profile. The CyberDog2 calibration
wrapper runs the reference-style interactive homing and range acquisition.
See [SO101 implementation and validation](../motion/so101/README.md) for
configuration, command semantics, collision limitations, and deployment checks.

The media worker also restores the legacy state fields `media_camera_list`,
`media_channel_count`, and `media_active_protocol`. The first lists every
configured channel, while the latter two describe currently active sessions;
these are existing fields and do not add keys to the MQTT state message.
