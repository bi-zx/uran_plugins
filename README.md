# URAN business plugins

This repository contains ROS-independent business code and optional ROS/vendor
backends migrated from `uran_ws`. Each top-level plugin directory is an
independently buildable Python distribution. Runtime workers use
`uran.worker.jsonl.v1` from the separately installed `uran-worker-sdk`; no
package imports source from the old workspace.

## Packages

| Distribution | Runtime role | Hardware-free backend |
| --- | --- | --- |
| `uran-mqtt-plugin` | MQTT transport and legacy cloud boundary codec | codec-only mode |
| `uran-cyberdog2-plugin` | base motion policy, failsafe and vendor mapping | deterministic simulator |
| `uran-control-arbiter-plugin` | per-resource control leases | pure state machine |
| `uran-px4-plugin` | ROS 1/PX4 aerial motion and OFFBOARD-safe setpoints | deterministic simulator |
| `uran-media-plugin` | media sessions, WebRTC/RTSP and camera adapters | explicit memory simulator |
| `uran-localization-plugin` | GPS/VO alignment, geo projection and fusion | pure algorithms |
| `uran-navigation-plugin` | depth scan, obstacle avoidance, safety gate and route follower | pure algorithms |
| `uran-autotask-plugin` | waypoint/outdoor parsing and mission lifecycle | worker task store |
| `uran-simulation-plugin` | independent base/dual-arm/gripper components | always simulated |

Install packages into an isolated worker environment. A released deployment
installs the `uran-worker-sdk` and `uran-interfaces` wheels from its locked
wheelhouse; it does not require a sibling checkout. For local source testing,
opt in explicitly:

```bash
URAN_ALLOW_SOURCE_DEPS=1 ./scripts/test.sh
```

For an interactive source environment:

```bash
python3 -m venv .venv
.venv/bin/pip install ../uran_framework/python/uran_worker_sdk ../uran_interfaces
.venv/bin/pip install -e protocols/mqtt -e motion/cyberdog2 -e motion/control_arbiter -e motion/px4 -e media
.venv/bin/pip install -e sensors/localization -e sensors/navigation -e tasks/autotask
.venv/bin/pip install -e simulation
```

Run a worker through the framework SDK, for example:

```bash
.venv/bin/python -m uran_worker_sdk uran_cyberdog2_plugin.worker:worker_factory
```

See [limitations](docs/limitations.md) before selecting a real backend.
