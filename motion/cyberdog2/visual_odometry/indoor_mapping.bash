#!/usr/bin/env bash
# Start an isolated indoor survey on a CyberDog2 development installation.
set -e -o pipefail
if [ "$#" -ne 2 ]; then echo "Usage: $0 ROBOT_NAMESPACE NEW_MAP_DIRECTORY" >&2; exit 2; fi
base=/${1#/}; base=${base%/}
room=$(realpath -m "$2")
survey=$base/uran_survey
map_id=$(basename "$room")
here=$(cd "$(dirname "$0")" && pwd)
visual=${VISUAL_ODOMETRY_BIN:-$PWD/build/visual_odometry/visual_odometry}
exe="$(ros2 pkg prefix uran_ros2_adapter)/lib/uran_ros2_adapter"
laser="$(ros2 pkg prefix laser_slam)/lib/laser_slam/mapping"
saver="$(ros2 pkg prefix nav2_map_server)/lib/nav2_map_server/map_saver_cli"
export PYTHONPATH="$here/../src:${PYTHONPATH:-}" OPENBLAS_NUM_THREADS=1
test -x "$visual" || { echo "Build the visual_odometry executable first" >&2; exit 2; }
if [ -e "$room/coverage.sqlite" ] || [ -e "$room/graph.pbstream" ]; then echo "Map already contains observations: $room" >&2; exit 2; fi
mkdir -p "$room/config" "$room/log"
# The desktop subscribes to one survey namespace per robot.
exec 9>"/tmp/uran-visual-${base//\//_}.lock"
flock -n 9 || { echo "Indoor mapping is already running for $base" >&2; exit 2; }
python3 "$here/prepare.py" "$room/config/visual" --namespace "$base"
python3 -m uran_cyberdog2_plugin.slam_params "$room/config" "$room" --namespace "$survey" --odom-topic visual_odom --skip-correlative-search
profile=${CYCLONEDDS_URI:-file:///etc/mi/cyclonedds.xml}
python3 - "${profile#file://}" "$room/config/cyclonedds.xml" <<'PY'
import sys,xml.etree.ElementTree as E
tree=E.parse(sys.argv[1]);root=tree.getroot();ns=root.tag[1:].split('}')[0]
E.register_namespace('',ns);E.register_namespace('xsi','http://www.w3.org/2001/XMLSchema-instance')
tag=lambda s:'{'+ns+'}'+s
domain=root.find(tag('Domain'));internal=domain.find(tag('Internal'))
if internal is None:internal=E.SubElement(domain,tag('Internal'))
size=internal.find(tag('MinimumSocketReceiveBufferSize'))
if size is None:size=E.SubElement(internal,tag('MinimumSocketReceiveBufferSize'))
size.text='4MB';tree.write(sys.argv[2],encoding='utf-8',xml_declaration=True)
PY
pids=(); active=0; raised=0
previous_cap=$(cat /proc/sys/net/core/rmem_max)
restore_cap() {
    if (( raised )); then sudo -n sysctl -qw net.core.rmem_max="$previous_cap"; raised=0; fi
}
close_survey() {
    status=$?
    trap - EXIT INT TERM
    set +e
    restore_cap
    # Save clients can use the host's normal buffer after the mapping sockets open.
    export CYCLONEDDS_URI="$profile"
    if (( active )); then
        timeout 24 "$saver" -t "$survey/survey_map" -f "$room/occupancy" --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=12.0 || status=1
        timeout 35 python3 - "$survey/stop_mapping" "$map_id" <<'PY' || status=1
import sys,rclpy
from visualization.srv import Stop
rclpy.init(args=[]);n=rclpy.create_node('save_visual_survey');c=n.create_client(Stop,sys.argv[1])
if not c.wait_for_service(timeout_sec=5):raise RuntimeError('Map save service unavailable')
q=Stop.Request();q.finish=True;q.map_name=sys.argv[2]
f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=25)
if not f.done() or not f.result().success:raise RuntimeError('Could not save localization graph')
n.destroy_node();rclpy.shutdown()
PY
        if [ -s "$room/occupancy.yaml" ]; then
            python3 - "$room/occupancy.yaml" <<'PY'
from pathlib import Path
import sys,yaml
p=Path(sys.argv[1]);d=yaml.safe_load(p.read_text());d['image']=Path(d['image']).name;p.write_text(yaml.safe_dump(d,default_flow_style=False))
PY
        fi
    fi
    for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null; done
    wait
    exit "$status"
}
trap close_survey EXIT
trap 'exit 0' INT TERM
if (( previous_cap < 4194304 )); then
    sudo -n sysctl -qw net.core.rmem_max=4194304
    raised=1
fi
export CYCLONEDDS_URI="file://$room/config/cyclonedds.xml"
start() {
    local name=$1; shift
    setsid "$@" 9>&- </dev/null >"$room/log/$name.log" 2>&1 &
    pids+=("$!")
}
start mapping "$laser" --ros-args -r __node:=map_builder -r __ns:="$survey" --params-file "$room/config/mapping_node.yaml" \
    -r "$survey/camera/imu:=$base/camera/imu" -r "$survey/visual_odom:=$base/visual_odom" -r "$survey/scan_fused:=$base/scan_fused" -r /tf:="$survey/tf"
depth_args=(-p depth_image_topic:=camera/depth/image_rect_raw -p depth_camera_info_topic:=camera/depth/camera_info \
    -p ground_frame:=visual_odom -p odom_topic:=visual_odom -p sensor_adapter:=uran_cyberdog2_plugin.map_sensors:bind_tof -p rear_tof_topic:=rear_tof_payload)
start sensors nice -n 4 "$exe/survey_sensors" --ros-args -r __ns:="$base" "${depth_args[@]}" \
    -p unknown_range:=zero -p depth_pixel_stride:=4 -p max_wait_s:=1.1 \
    -r map_observations:__node:=survey_observations \
    -r /uran/runtime/dispatch:="${OBSERVATION_TOPIC:-$survey/observations}" -p map_id:="$map_id" -p map_frame:=laser_odom -p pose_topic:="$survey/dog_pose" \
    -p device_id:="${DEVICE_ID:-${base##*/}}" -p component_id:=base -p reference_mode:=local -r survey_observations:scan:="$base/scan_fused" -p depth_rate_hz:=1.5 -p surface_topic:="$survey/surfaces"
start grid nice -n 4 "$exe/survey_map" --ros-args -r __ns:="$survey" -p map_id:="$map_id" -p scan_topic:="$base/scan_fused" -p pose_topic:=dog_pose \
    -p map_frame:=laser_odom -p coverage_file:="$room/coverage.sqlite" -p surface_topic:="$survey/surfaces"
python3 - "$base" "$survey" <<'PY'
import sys,time,rclpy
from lifecycle_msgs.srv import ChangeState
base,survey=sys.argv[1:];rclpy.init(args=[]);n=rclpy.create_node('start_visual_survey')
need={('fused_scan',base),('survey_observations',base),('survey_map',survey),('map_builder',survey)}
end=time.monotonic()+40
while time.monotonic()<end:
    if need.issubset(n.get_node_names_and_namespaces()):break
    rclpy.spin_once(n,timeout_sec=.1)
else:raise RuntimeError('Mapping nodes did not initialize; see the map log directory')
def call(cl,q):
    if not cl.wait_for_service(timeout_sec=10):raise RuntimeError(cl.srv_name+' unavailable')
    f=cl.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=15)
    if not f.done() or not f.result().success:raise RuntimeError(cl.srv_name+' failed')
client=n.create_client(ChangeState,survey+'/map_builder/change_state')
for step in (1,3):
    q=ChangeState.Request();q.transition.id=step;call(client,q)
n.destroy_node();rclpy.shutdown()
PY
# Let the map processes finish their CPU-heavy initialization before stereo tracking starts.
start visual "$visual" --ros-args -r __ns:="$base" --params-file "$room/config/visual/node.yaml"
python3 - "$base/visual_odom" "$survey/start_mapping" <<'PY'
import sys,time,rclpy
from nav_msgs.msg import Odometry
from collections import deque
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import SetBool
rclpy.init(args=[]);n=rclpy.create_node('await_visual_tracking')
client=n.create_client(SetBool,sys.argv[2])
ticks=deque(maxlen=3)
def observe(msg):
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    if not 0 <= time.time()-stamp <= 0.75:
        ticks.clear()
        return
    if ticks and not 0 < stamp-ticks[-1] <= 0.5:
        ticks.clear()
    ticks.append(stamp)
n.create_subscription(Odometry,sys.argv[1],observe,qos_profile_sensor_data)
end=time.monotonic()+65
while time.monotonic()<end and len(ticks)<3:rclpy.spin_once(n,timeout_sec=.1)
if len(ticks)<3:raise RuntimeError('Fresh visual tracking did not initialize; see visual.log')
if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Mapping service unavailable')
q=SetBool.Request();q.data=True;f=client.call_async(q)
rclpy.spin_until_future_complete(n,f,timeout_sec=15)
if not f.done() or not f.result().success:raise RuntimeError('Could not start mapping')
n.destroy_node();rclpy.shutdown()
PY
active=1
restore_cap
echo "Visual indoor survey running. Press Ctrl+C to save $room."
wait -n || exit 1
echo "A survey process exited; see $room/log" >&2
exit 1
