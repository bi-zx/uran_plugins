# Limitations and validation status

The pure algorithms, workers and simulation backends are tested locally. The
arbiter-to-simulation worker chain has also been exercised through the native
Runtime. No physical robot movement was performed. CyberDog2 ROS2 motion,
camera, RealSense, GStreamer RTSP and live WebRTC paths retain real
vendor/backend calls but are not hardware-validated here.

The framework worker adapter loads process workers as message plugins. MQTT
publishing, cloud decoding and asynchronous targeted `host_request` dispatch
are implemented. A real broker, credentials and reconnect/failure behavior
still require deployment validation; codec and worker tests do not constitute
broker acceptance.

The autotask worker validates and manages task lifecycle. Physical navigation
requires a ROS2 backend in `uran_ros2_adapter` to translate task dispatch,
pose/scan observations and media actions. Missing backends fail; they are not
reported as successful operations. The ROS1/PX4 software migration now has a
Noetic catkin bridge, deterministic PX4 controller tests, and a Python 3.8
compatible worker path. Vendor PX4 message packages, real flight validation,
and the `uran_profiles` deployment repository remain outside this phase.
