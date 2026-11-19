"""Generate the installed laser SLAM configuration for a URAN map directory."""
import argparse
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_dir",type=Path)
    parser.add_argument("map_dir",type=Path)
    parser.add_argument("--namespace",default="",help="Fully qualify node keys for direct executable launch")
    parser.add_argument("--scan-topic",default="scan",help="Topic relative to the vendor node namespace")
    parser.add_argument("--odom-topic",help="Odometry topic relative to the vendor node namespace")
    parser.add_argument("--skip-correlative-search",action="store_true",help="Start local scan matching at the predicted pose when mapping")
    args = parser.parse_args()
    source = Path(get_package_share_directory("laser_slam"))/"param"
    args.output_dir.mkdir(parents=True,exist_ok=True)
    args.map_dir.mkdir(parents=True,exist_ok=True)
    dest = args.map_dir.resolve()
    for name in ("mapping_node.yaml","localization.yaml"):
        with (source/name).open() as stream:
            doc = yaml.safe_load(stream)
        node = next(iter(doc))
        params = doc[node]["ros__parameters"]
        params.update(laser_scan_topic="/"+args.scan_topic.strip("/"),frame_id="laser_odom",
                      max_range=5.0,missing_data_ray_length=0.0,map_save_path=str(dest)+"/",
                      pose_save_path=str(dest/Path(params["pose_save_path"]).name))
        if args.odom_topic:
            params["odometry_topic"] = "/"+args.odom_topic.strip("/")
        if name == "mapping_node.yaml":
            params["use_real_time_map_beauti"] = False
            if args.skip_correlative_search:
                params["use_real_time_correlative_scan_matching"] = False
        if args.namespace:
            doc = {"/"+args.namespace.strip("/")+"/"+node:doc[node]}
        with (args.output_dir/name).open("w") as stream:
            yaml.safe_dump(doc,stream,default_flow_style=False)


if __name__ == "__main__":
    main()
