# CyberDog2 laser SLAM

This target rebuilds the installed mapper with a time-ordered IMU and odometry collector. The vendor scan matcher, calibration, mapping services and graph format come from the installed SDK. The collector preserves acquisition timestamps and discards measurements older than an already integrated interval.

On the CyberDog2, source the normal URAN ROS environment and run the following from the development workspace. The source archives match the board SDK. The PCL package is only unpacked into the build directory.

```bash
out="$PWD/build/ordered_slam"
mkdir -p "$out"/{vendor/abseil,vendor/protobuf,include_deps}
curl -fL https://codeload.github.com/abseil/abseil-cpp/tar.gz/refs/tags/20211102.0 | tar -xz --strip-components=1 -C "$out/vendor/abseil"
curl -fL https://codeload.github.com/protocolbuffers/protobuf/tar.gz/refs/tags/v3.19.2 | tar -xz --strip-components=1 -C "$out/vendor/protobuf"
(cd "$out/include_deps" && apt-get download libpcl-dev && dpkg-deb -x ./libpcl-dev_*.deb .)
bash uran_plugins/motion/cyberdog2/laser_slam/build.bash "$out"
```

The indoor mapping launcher selects `build/ordered_slam/mapping` when present. Set `LASER_SLAM_BIN` to select an explicit executable. The board build links Ceres 1.14 from the installed MiVINS shared library. Set `CERES_LIBRARY` to a standalone Ceres 1.14 library on an SDK that supplies one.
