#!/usr/bin/env python3
"""Check expanded simulator K against native Ogre2 depth, including negative control.

Requires g++, pkg-config, ROS xacro, ignition-rendering6, ignition-utils1,
and sdformat12.
Runs an isolated in-process renderer only. Preserves all inputs and logs.
"""
import argparse
import json
from pathlib import Path
import shlex
import subprocess
import xml.etree.ElementTree as ET


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    root = Path(__file__).resolve().parents[2]
    macro = root / 'src/luggage_description/urdf/realsense_d435.urdf.xacro'
    wrapper = args.output / 'camera.xacro'
    wrapper.write_text('<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="probe">'
                       f'<xacro:include filename="{macro}"/>'
                       '<link name="base"/><xacro:realsense_d435 parent="base">'
                       '<origin xyz="0 0 0" rpy="0 0 0"/>'
                       '</xacro:realsense_d435></robot>')
    expanded = subprocess.check_output(['xacro', str(wrapper)], timeout=30)
    (args.output / 'camera.urdf').write_bytes(expanded)
    camera = ET.fromstring(expanded).find('.//sensor/camera')
    intrinsics = camera.find('lens/intrinsics')
    values = [float(intrinsics.findtext(k)) for k in ('fx', 'fy', 'cx', 'cy')]
    hfov = float(camera.findtext('horizontal_fov'))
    sensor = ET.fromstring(expanded).find('.//sensor')
    sensor_file = args.output / 'sensor.sdf'
    sensor_file.write_text('<sdf version="1.9"><model name="projection_probe">'
                          '<static>true</static><link name="camera">' +
                          ET.tostring(sensor, encoding='unicode') +
                          '</link></model></sdf>')
    hardware_reference = (323.1775,322.8994,317.7526,178.0294)
    within_profile = all(abs(v/r-1)<=.02 for v,r in zip(values,hardware_reference))
    binary = args.output / 'probe'
    source = Path(__file__).with_name('depth_projection_probe.cc')
    flags = shlex.split(subprocess.check_output(
        ['pkg-config','--cflags','--libs','ignition-rendering6','ignition-utils1','sdformat12'],text=True))
    with (args.output / 'build.log').open('w') as log:
        subprocess.run(['g++',str(source),'-o',str(binary),*flags],stdout=log,
                       stderr=subprocess.STDOUT,timeout=60,check=True)
    runs=[]
    for name,k in [('configured',values),('old_hardware_K',hardware_reference)]:
        command=[str(binary.resolve()),*map(str,k),str(hfov)]
        if name == 'configured':
            command.append(str(sensor_file.resolve()))
        with (args.output / (name+'.log')).open('w') as log:
            result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,timeout=30)
        runs.append({'name':name,'command':command,'returncode':result.returncode})
    passed=within_profile and runs[0]['returncode']==0 and runs[1]['returncode']==4
    report={'pass':passed,'scope':'nine pixels on tilted analytic plane; native renderer, no robot/full-world validation',
            'K':values,'hfov':hfov,'native_K_P_and_geometry_pass':runs[0]['returncode']==0,'within_hardware_profile_2pct':within_profile,
            'runs':runs,'revision':subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip(),
            'dirty_state':subprocess.check_output(['git','status','--porcelain'],cwd=root,text=True)}
    (args.output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    return 0 if passed else 1


if __name__=='__main__':
    raise SystemExit(main())
