# Behavior matrix

| Behavior | Implementation | Automated evidence |
| --- | --- | --- |
| Legacy `timestamp_ms` conversion without nanosecond precision loss | MQTT `CloudCodec` | `test_codec.py` |
| MQTT connection serialization and cleanup | migrated `MqttClient` | source-preservation and worker integration tests |
| Stop/e-stop bypass failsafe, mode and lease gates | `MotionController` | `test_controller.py` |
| Vector/angular limits reject non-finite input | `MotionController` | `test_controller.py` |
| CyberDog command timeout publishes zero | `CyberDogPolicyBackend` | `test_vendor_policy.py` |
| Left/right arm resource leases and state are isolated | control arbiter and simulation | `test_arbiter.py`, `test_robot.py` |
| Idle stand retry cooldown | `CyberDogPolicyBackend` | `test_vendor_policy.py` |
| Charging status clears only after BMS disconnect timeout | `CyberDogPolicyBackend` | `test_vendor_policy.py` |
| ESTOP recovery waits for NORMAL | `CyberDogPolicyBackend` | `test_vendor_policy.py` |
| Missing media libraries are reported as failures | media backends | `test_media.py` |
| Depth cloud conversion and scan quality gates | navigation package | migrated `test_depth_scan_adapter.py` |
| Acceleration shaping, route following, safety gate, resume remainder | navigation/task packages | migrated `test_navigation_control.py` |
| GPS/VO alignment and dead reckoning | localization package | migrated localization tests |
| Outdoor route parsing and start alignment | autotask package | migrated outdoor tests |
