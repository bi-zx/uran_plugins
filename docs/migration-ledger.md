# Migration ledger

The source repositories remain unchanged. The fixed migration sources are
`uran_ws/main` at `300d703` and `uran_ws/origin/ROS2-refactored` at `92dab37`.
`main` is authoritative for later business fixes; the refactored branch was
used to check package boundaries. ROS1 and PX4 are outside this migration.

| Target | Source at `300d703` | Decision |
| --- | --- | --- |
| `protocols/mqtt` | `src/uran_core/uran_core/mqtt_client.py`, downlink helpers in `uran_core_node.py` | Preserve connection lock, failed-connect cleanup, username compatibility, registration timeout mode; extract strict cloud codec |
| `motion/cyberdog2` | `src/uran_move/uran_move_node.py`, `plugins/cyberdog2_plugin.py`, `test_move_safety.py` | Extract gating and vendor policy; preserve safety-command bypass, zero-first stop, 20 Hz timeout loop, idle stand, BMS charging recovery, ESTOP wait and control suspension |
| `motion/control_arbiter` | `uran_move_node.py` motion lock and `uran_autotask` task ownership | Replace global lock with `(device_id, resource_group)` leases and component ownership |
| `media` | `src/uran_media/uran_media/*` | Retain WebRTC/RTSP/CyberDog/RealSense backends; replace successful stubs with explicit errors |
| `sensors/localization` | `src/uran_autotask/uran_autotask/localization/*`, `geo_utils.py` | Remove ROS message construction and retain pure fusion/alignment logic |
| `sensors/navigation` | `src/uran_autotask/uran_autotask/navigation/*` | Preserve main-only depth scan, route follower, safety gate, velocity shaper and remaining-distance behavior |
| `tasks/autotask` | `src/uran_autotask/uran_autotask/{task_*,outdoor,mission}` | Preserve parser, outdoor planner contract, mission state machine and resume fixes; use installed localization/navigation packages |

Generated files, build output, user service files with workstation paths, ROS
message packages, and the monolithic ROS nodes were not copied. Their platform
translation belongs to `uran_ros2_adapter`.

The migrated workers now run as independently packaged JSONL processes. The
native Runtime has been exercised with the control-arbiter -> simulation chain,
including terminal result forwarding, task-side `call_local` lease acquisition,
operation cancellation and lease-expiry stop. This is software validation only;
ROS1/PX4, physical hardware, and `uran_profiles` remain outside this phase.
