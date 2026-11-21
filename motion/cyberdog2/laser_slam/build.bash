#!/usr/bin/env bash
# Build the CyberDog2 mapper against the board SDK in a development directory.
set -e -o pipefail
src=$(cd "$(dirname "$0")" && pwd)
out=$(realpath -m "${1:?Usage: build.bash BUILD_DIRECTORY}")
sdk=$(ros2 pkg prefix laser_slam)
ros=$(ros2 pkg prefix rclcpp)
mkdir -p "$out"
cd "$out"
export TMPDIR="$out"
jobs=${JOBS:-3}
pcl=${PCL_INCLUDE_DIR:-$out/include_deps/usr/include/pcl-1.8}
if [[ ! -d "$pcl" ]]; then pcl=/usr/include/pcl-1.8; fi

cmake -S vendor/protobuf/cmake -B protobuf-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG' -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_BUILD_PROTOC_BINARIES=OFF
cmake --build protobuf-build --target libprotobuf --parallel "$jobs"
cmake -S vendor/abseil -B absl-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG' -DBUILD_TESTING=OFF
cmake --build absl-build --target raw_hash_set hash synchronization --parallel "$jobs"

includes=(-I"$sdk/include" -I"$ros/include" -I/usr/include/eigen3 -I/usr/include/opencv4 -I/usr/include/cairo -I"$pcl")
cxx=${CXX:-g++}
"$cxx" -std=c++17 -DNDEBUG -O1 "${includes[@]}" -c "$src/mapping_node.cpp" -o entry.o
"$cxx" -std=c++17 -DNDEBUG -O2 "${includes[@]}" -c "$src/measurement_collector.cc" -o collector.o
archives=()
for pkg in laser_slam pose_graph pose_extrapolator range_data_matching rviz_display sensor transform protos common; do
    archives+=("$sdk/lib/$pkg/lib$pkg.a")
done
mapfile -t absl < <(find absl-build -name 'libabsl_*.a')
mapfile -t shared < <(ldd "$sdk/lib/laser_slam/mapping" | awk '/=> \/.* [(]/{print $3}')
# The board's MiVINS library exports the same Ceres 1.14 implementation as the stock mapper.
ceres=${CERES_LIBRARY:-$sdk/lib/libvins_lib.so}
"$cxx" entry.o collector.o -Wl,--start-group "${archives[@]}" protobuf-build/libprotobuf.a "${absl[@]}" -Wl,--end-group "$ceres" "${shared[@]}" -lz -lpthread -ldl -o mapping.new
mv mapping.new mapping
