#!/usr/bin/env python3
"""Build an isolated Gazebo 6.18 Physics system with contact extras exported.

The pinned upstream code already obtains normal/depth from the engine, but its
ContactSensorData conversion discards them. The patch below changes only that
conversion; integrator, engine, contacts, forces and parameter defaults are kept.
No system installation is modified. A fixture must explicitly select the binary.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.request

REVISION='b90cd2c41a193102d6951857c2d83e13f59bb64b'  # Gazebo 6.18.0 release
EXPECTED={
 'Physics.cc':'80396c542654490b0eb2aad3890520c720839c01012fb741aa6d3943135f3a26',
 'Physics.hh':'9b9b417d4fd0b02bbdf28d71ae1296e5a62f9ce9c3fdf0198056d6cab5c972cd',
 'EntityFeatureMap.hh':'4464e0b070db9d9be43488bb3b41753b11be1c5258f02c35e39e4d59a53785bf',
 'CanonicalLinkModelTracker.hh':'886c7ffac3c2310a67a0584090e8e41925af4d9609ab4cb361fe4436dd60b252'}
FILES=tuple(EXPECTED)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--upstream-cache',type=Path)
    args=parser.parse_args();output=args.output.resolve();output.mkdir(parents=True,exist_ok=False)
    source=output/'source';source.mkdir();manifest=dict(upstream_revision=REVISION,files={})
    for name in FILES:
        url=f'https://raw.githubusercontent.com/gazebosim/gz-sim/{REVISION}/src/systems/physics/{name}'
        if args.upstream_cache:data=(args.upstream_cache/name).read_bytes()
        else:
            with urllib.request.urlopen(url,timeout=15) as response:data=response.read()
        if hashlib.sha256(data).hexdigest()!=EXPECTED[name]:raise ValueError('UPSTREAM_HASH_MISMATCH:'+name)
        (source/name).write_bytes(data)
        manifest['files'][name]=dict(url=url,sha256=hashlib.sha256(data).hexdigest())
    original=(source/'Physics.cc').read_text();modified=original
    replacements=[
      ('  for (const auto &contactComposite : allContacts)\n',
       '  using Extra = physics::GetContactsFromLastStepFeature::ExtraContactDataT<physics::FeaturePolicy3d>;\n'
       '  std::unordered_map<const WorldShapeType::ContactPoint *, const Extra *> extras;\n'
       '  std::unordered_map<const WorldShapeType::ContactPoint *, Entity> firstEntities;\n'
       '  for (const auto &contactComposite : allContacts)\n'),
      ('      entityContactMap[coll1Entity][coll2Entity].push_back(&contact);',
       '      extras[&contact] = contactComposite.Query<Extra>();\n'
       '      firstEntities[&contact] = coll1Entity;\n'
       '      entityContactMap[coll1Entity][coll2Entity].push_back(&contact);'),
      ('            position->set_z(contact->point.z());',
       '            position->set_z(contact->point.z());\n'
       '            if (const auto *extra = extras[contact])\n'
       '            {\n'
       '              const double sign = firstEntities[contact] == _collEntity1 ? 1.0 : -1.0;\n'
       '              auto *normal = contactMsg->add_normal();\n'
       '              normal->set_x(sign * extra->normal.x());\n'
       '              normal->set_y(sign * extra->normal.y());\n'
       '              normal->set_z(sign * extra->normal.z());\n'
       '              contactMsg->add_depth(extra->depth);\n'
       '            }')]
    for before,after in replacements:
        if modified.count(before)!=1:raise ValueError('UPSTREAM_PREIMAGE_MISMATCH')
        modified=modified.replace(before,after)
    (source/'Physics.cc').write_text(modified)
    (output/'contact_extras.patch').write_text(''.join(difflib.unified_diff(
        original.splitlines(True),modified.splitlines(True),fromfile='upstream/Physics.cc',tofile='observed/Physics.cc')))
    (source/'CMakeLists.txt').write_text('''cmake_minimum_required(VERSION 3.16)
project(uld_upper_observed_physics LANGUAGES CXX)
find_package(ignition-gazebo6 6.18 REQUIRED)
find_package(ignition-physics5 REQUIRED COMPONENTS sdf mesh heightmap)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
add_library(uld_upper_observed_physics SHARED Physics.cc)
target_compile_features(uld_upper_observed_physics PRIVATE cxx_std_17)
target_link_libraries(uld_upper_observed_physics PRIVATE ignition-gazebo6::ignition-gazebo6
 ignition-physics5::ignition-physics5 ignition-physics5::ignition-physics5-sdf
 ignition-physics5::ignition-physics5-mesh ignition-physics5::ignition-physics5-heightmap
 ignition-plugin1::ignition-plugin1)
''')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    with (output/'build.log').open('w') as log:
        subprocess.run(['cmake','-S',str(source),'-B',str(output/'build')],stdout=log,stderr=subprocess.STDOUT,check=True)
        subprocess.run(['cmake','--build',str(output/'build'),'-j2'],stdout=log,stderr=subprocess.STDOUT,check=True)
    binary=output/'build/libuld_upper_observed_physics.so'
    manifest['binary_sha256']=hashlib.sha256(binary.read_bytes()).hexdigest()
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(binary)


if __name__=='__main__':main()
