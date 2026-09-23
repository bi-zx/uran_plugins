# Dependencies

| Package/backend | Required dependency | Ownership and status |
| --- | --- | --- |
| all process workers | `uran-worker-sdk` 1.x from `uran_framework` | required, installed package |
| contract validation and typed documents | `uran-interfaces` 1.x | required by deployment/integration tests |
| MQTT transport | `paho-mqtt` 1.6 through 2.x, broker credentials | optional extra; codec works without it |
| CyberDog2 real motion | ROS2 `rclpy`, Xiaomi `protocol.msg`/`protocol.srv` | vendor environment; not redistributed |
| WebRTC | `aiortc`, `av`, `numpy` | optional `webrtc` extra |
| RTSP | GObject introspection, GStreamer, `GstRtspServer`, x264 | system/vendor packages |
| CyberDog2 camera | Xiaomi `protocol.srv.CameraService` | vendor environment |
| RealSense lifecycle | ROS2 `lifecycle_msgs`, configured launch package | ROS environment |

No dependency is loaded through a sibling checkout or an old-workspace
`PYTHONPATH` by the default test/CI path. `scripts/test.sh` accepts
`URAN_DEPENDENCY_WHEELHOUSE` for an offline locked build, or the two explicit
`URAN_FRAMEWORK_WHEEL`/`URAN_INTERFACES_WHEEL` paths. A local source checkout
requires the deliberate `URAN_ALLOW_SOURCE_DEPS=1` opt-in. Formal releases
must lock wheels and system/vendor package versions in the product profile.
