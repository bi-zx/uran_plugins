"""Prepare MiVINS for the live rectified stereo streams on a stationary device."""
import argparse
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from uran_ros2_adapter.map_geometry import matrix, rectified_depth_points


def starting_floor(points):
    points = points[(points[:,0] > .45) & (points[:,0] < 1.4) & (abs(points[:,1]) < .35)
                    & (points[:,2] < -.07) & (points[:,2] > -.48)]
    if len(points) < 80:
        return None
    # Find a near-horizontal plane with support across the visible floor, then refine it.
    rng = np.random.RandomState(12)
    best = []
    for _ in range(35):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1]-sample[0], sample[2]-sample[0])
        if abs(normal[2]) < .001 or np.linalg.norm(normal[:2]) > .27*abs(normal[2]):
            continue
        support = points[abs((points-sample[0])@normal) < .018*abs(normal[2])]
        if len(support) > len(best):
            best = support
    if len(best) < 80 or np.ptp(best[:,0]) < .3 or np.ptp(best[:,1]) < .25:
        return None
    return np.linalg.lstsq(np.c_[best[:,:2],np.ones(len(best))],best[:,2],rcond=None)[0]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("folder", type=Path)
    parser.add_argument("--namespace", required=True)
    parser.add_argument("--device-calibration", default="/params/camera/calibration")
    args = parser.parse_args()
    folder = args.folder.resolve()
    folder.mkdir(parents=True,exist_ok=True)
    vendor = Path(get_package_share_directory("vins"))
    sys.path.insert(0,str(vendor/"launch"))
    from convert_calibration import ConvertCalibration
    calib_file = folder/"stereo.yaml"
    if not ConvertCalibration(args.device_calibration,str(calib_file)):
        raise RuntimeError("MiVINS device calibration unavailable")
    calibration = yaml.safe_load(calib_file.read_text())

    rclpy.init(args=[])
    node = rclpy.create_node("visual_reference",namespace=args.namespace)
    transforms = Buffer()
    listener = TransformListener(transforms,node)
    infos,planes = {},[]
    def depth(msg):
        if "depth" not in infos:
            return
        try:
            mount = matrix(transforms.lookup_transform("base_link",msg.header.frame_id,Time()).transform)
        except TransformException:
            return
        points = rectified_depth_points(msg,infos["depth"],8)@mount[:3,:3].T + mount[:3,3]
        plane = starting_floor(points)
        if plane is not None:
            planes.append(plane)

    for sensor in ("infra1","infra2","depth"):
        node.create_subscription(CameraInfo,"camera/"+sensor+"/camera_info",
                                 lambda m,k=sensor:infos.update({k:m}),qos_profile_sensor_data)
    node.create_subscription(Image,"camera/depth/image_rect_raw",depth,qos_profile_sensor_data)
    end = time.monotonic()+18
    while (len(infos)<3 or len(planes)<10) and time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.1)
    node.destroy_node()
    rclpy.shutdown()
    if len(infos)<3 or len(planes)<10:
        raise RuntimeError("Need both stereo calibrations and a visible starting floor")
    floor = np.median(planes,axis=0)
    if np.max(np.abs(np.asarray(planes)[:,2]-floor[2])) > .025:
        raise RuntimeError("Starting floor is inconsistent across frames")

    # CameraInfo.P describes the rectified images and their metric baseline.
    left,right = infos["infra1"],infos["infra2"]
    baseline = left.p[3]/left.p[0]-right.p[3]/right.p[0]
    if baseline <= 0:
        raise RuntimeError("Invalid rectified stereo baseline")
    imu_from_left = np.array(calibration["cameras"][0]["T_B_C"]["data"]).reshape(4,4)
    imu_from_left[:3,:3] = imu_from_left[:3,:3] @ np.array(left.r).reshape(3,3).T
    for idx,info in enumerate((left,right)):
        entry = calibration["cameras"][idx]
        camera = entry["camera"]
        camera["image_width"],camera["image_height"] = info.width,info.height
        camera["intrinsics"]["data"] = [float(info.p[i]) for i in (0,5,2,6)]
        camera["distortion"]["parameters"]["data"] = [0.0]*4
        transform = imu_from_left.copy()
        transform[:3,3] += transform[:3,0]*baseline*idx
        entry["T_B_C"]["data"] = transform.ravel().tolist()
    calib_file.write_text(yaml.safe_dump(calibration,default_flow_style=False))
    config = yaml.safe_load((vendor/"param/d430/realsense_vio_stereo.yaml").read_text())
    # Stereo provides metric translation. The frontend uses IMU attitude between images.
    # Translational IMU preintegration drifts across image gaps at a stationary start.
    config.update(use_odom=False,use_processed_odom=False,use_constraint_odom=False,backend_with_odom=False,
                  use_loose_couple=False,use_pose_update=False,use_imu=True,backend_with_imu=False,
                  automatic_reinitialization=False,save_pose_path=str(folder/"camera_poses.txt"),
                  save_kf_pose_path=str(folder/"camera_keyframes.txt"))
    config_file = folder/"estimator.yaml"
    config_file.write_text(yaml.safe_dump(config,default_flow_style=False))
    params = dict(config=str(config_file),calibration=str(calib_file),starting_floor=floor.tolist(),
                  imu_from_body=calibration["odometry_params"]["T_B_O"]["data"])
    node_key = "/"+(args.namespace.strip("/")+"/visual_odometry").lstrip("/")
    (folder/"node.yaml").write_text(yaml.safe_dump({node_key:{"ros__parameters":params}},default_flow_style=False))
    print("Stereo baseline %.4f m. Starting floor z = %.4f x + %.4f y + %.4f m" % (baseline,*floor))


if __name__=="__main__":
    main()
