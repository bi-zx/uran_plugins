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

`motion/so101` is a ROS-independent worker for the six-joint SO101 follower.
It accepts `arm_joints`, Cartesian `arm_pose` (damped least-squares IK), and
`safety_action` commands. `serial_port` belongs in the profile's
`worker_config`; use a persistent `/dev/serial/by-id/...` path on hardware or
`auto` during discovery. The LeRobot safety JSON and a YAML calibration file
are separate inputs. Torque is disabled by default, and the calibration tool
is available as `uran-so101-calibrate probe`, `record-center`, and
`record-limits`.
