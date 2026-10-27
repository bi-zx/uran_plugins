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
| `uran-frpc-plugin` | explicitly authorized temporary maintenance transport | certificate/configuration checks |
| `uran-cyberdog2-plugin` | base motion policy, failsafe and vendor mapping | deterministic simulator |
| `uran-control-arbiter-plugin` | per-resource control leases | pure state machine |
| `uran-px4-plugin` | ROS 1/PX4 aerial motion and OFFBOARD-safe setpoints | deterministic simulator |
| `uran-media-plugin` | media sessions, WebRTC/RTSP and camera adapters | explicit memory simulator |
| `uran-localization-plugin` | GPS/VO alignment, geo projection and fusion | pure algorithms |
| `uran-navigation-plugin` | depth scan, obstacle avoidance, safety gate and route follower | pure algorithms |
| `uran-autotask-plugin` | waypoint/outdoor parsing and mission lifecycle | worker task store |
| `uran-simulation-plugin` | independent base/dual-arm/gripper components | always simulated |
| `uran-so101-plugin` | SO101/Feetech arm bus, LeRobot-compatible FK/IK and safety | simulation backend |

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
.venv/bin/pip install -e simulation -e motion/so101
```

Run a worker through the framework SDK, for example:

```bash
.venv/bin/python -m uran_worker_sdk uran_cyberdog2_plugin.worker:worker_factory
```

See [limitations](docs/limitations.md) before selecting a real backend.

## Product configuration

MQTT uses the Core-provided device identity or an explicitly matching
`device_id`; it does not invent a device or product template. `template_id` is
optional and must be selected by the device/Profile when its cloud requires it.
Task recording requires `media_actions.record_channel_id`; no camera channel
is selected by robot model. SO101 calibration write commands require an explicit
output path. Choose `URAN_PYTHON` to override the test interpreter, otherwise
the test runner uses `python3` from `PATH`.
# 受控设备管理接入

MQTT 新增 `management_cmd` 下行，原样将签名 envelope 路由到 Core 的管理 worker；旧 `frpc_ctrl` 被拒绝。管理任务的用户授权与事务由 framework 执行。新的 bootstrap 要求 MQTT/TLS，旧业务协议字段保持原语义。

`network/frpc` 是可选的临时 SSH 维护 provider。它从本地监督进程接收已授权的服务选择，使用固定的单条 loopback SSH 代理和短期 SSH 用户证书；默认 bootstrap 不安装或启动 FRPC。SSHD/FRPS、本地服务定义和 SSH CA 必须由管理员预置，不能由 agent 下发任意配置。完整流程见 framework 的 `docs/受控部署与Bootstrap.md`。
