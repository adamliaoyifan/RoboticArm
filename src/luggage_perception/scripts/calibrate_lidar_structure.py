#!/usr/bin/env python3
"""Build empirical structure association from an observed, identified empty patch.

NPZ must contain points_container and origins_container (Nx3), after stamped TF.
This command never reads simulator truth or resets/clears an occupancy map.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from luggage_perception.lidar_structure import calibrate_structure


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--normal',type=float,nargs=3,required=True)
    parser.add_argument('--offset-m',type=float,required=True)
    for field in ('geometry-hash','sensor-frame','sensor-mount-id','source-id'):
        parser.add_argument('--'+field,required=True)
    args=parser.parse_args()
    with np.load(args.input,allow_pickle=False) as data:
        record=calibrate_structure(data['points_container'],origins=data['origins_container'],
            normal=args.normal,offset_m=args.offset_m,geometry_hash=args.geometry_hash,
            sensor_frame=args.sensor_frame,sensor_mount_id=args.sensor_mount_id,source_id=args.source_id)
    with args.output.open('x') as stream:json.dump(record,stream,indent=2,allow_nan=False)
    print(json.dumps(dict(calibration_id=record['calibration_id'],sample_count=record['sample_count'],
        residual_m=[record['residual_min_m'],record['residual_max_m']],scope=record['uncertainty_scope'])))


if __name__=='__main__':main()
