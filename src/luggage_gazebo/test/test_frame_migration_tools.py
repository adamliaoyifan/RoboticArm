"""ST09 tool boundaries: independent geometry, readiness and evidence negatives."""
import copy
import itertools
import json
import math
import os
from pathlib import Path
import tempfile
import unittest

from luggage_description.frame_contracts import ContractError
from luggage_description.frame_transforms import box_to_tool_target, rotate_vector
from luggage_description.runtime_profile import resolve_profile, flatten_profile, validate_identity
from luggage_description.runtime_readiness import ReadinessGate
from luggage_gazebo.frame_dump import (
    SCHEMA, adapt_legacy_surface, read_dump, write_dump, world_marker_payload,
)
from luggage_gazebo.integration_acceptance import EVENT_ORDER, FROZEN_HASHES, extract_acceptance, capture_window


def identity():
    return dict(contract_version=2, scene_revision="scene", calibration_revision="cal",
                urdf_sha256="a" * 64, srdf_sha256="b" * 64, tool_link="suction_contact_frame",
                joint_names=["j1", "j2"], geometry_hash="hull", atlas_sha256="c" * 64,
                atlas_certification_sha256="d" * 64,
                endpoints={"/execute_v2": "luggage_msgs/action/PlanMotionV2"})


def surface(floor=0.53):
    return dict(contract_version=2, kind="surface", frame_id="container_link", length_unit="m",
                stamp_ns=9007199254740993, time_basis="acquisition", scene_revision="scene",
                request_id="request", geometry_hash="hull", map_revision=3,
                height_datum="container_origin", floor_z_container=floor, ceiling_z_container=floor+2,
                resolution_m=0.1, origin_xy_container=[-1, -0.5],
                surface_z_container=[[floor, floor+0.32, None]],
                state=[["free", "occupied", "unknown"]],
                confidence=[["sensor", "sensor", "unknown"]])


EXPECTED = dict(scene_revision="scene", geometry_hash="hull", request_id="request", map_revision=3)


def dump_fixture(yaw=0, origin=(0,0,0), floor=0.53, support=0, sign=1):
    return dict(schema=SCHEMA, contract_version=2, record_kind="measured", **EXPECTED,
                pose_profile="slot", body_dimensions_m=[0.8,0.5,0.32], surface=surface(floor),
                pose=dict(contract_version=2, kind="pose", frame_id="container_link", length_unit="m",
                          stamp_ns=9007199254740993, time_basis="scene", scene_revision="scene",
                          request_id="request", reference="box_center", orientation_semantics="box_axis_pi",
                          position=[0.2,-0.1,floor+support+0.16],
                          quaternion_xyzw=[0,0,sign*math.sin(0.15),sign*math.cos(0.15)]),
                transform_snapshot=dict(source_frame="container_link", target_frame="world",
                                        scene_revision="scene", stamp_ns=None, translation=list(origin),
                                        quaternion_xyzw=[0,0,math.sin(yaw/2),math.cos(yaw/2)]),
                provenance={"sensor": "fixture-independent-geometry"})


class GeometryTools(unittest.TestCase):
    def test_64_geometry_cases_dump_replay_markers(self):
        cases = itertools.product((0,math.pi/2,math.pi,math.atan2(4,3)),
                                  ((0,0,0),(1.5,-0.4,0.2)), (0.53,0.71), (0,0.3), (0,0.1))
        with tempfile.TemporaryDirectory() as directory:
            for number, (yaw,origin,floor,support,offset) in enumerate(cases):
                with self.subTest(case=number):
                    for sign in (1,-1):
                        dump = dump_fixture(yaw,origin,floor,support,sign)
                        path = Path(directory)/f"{number}-{sign}.json"
                        write_dump(path,dump,**EXPECTED)
                        replay = read_dump(path,**EXPECTED)
                        self.assertEqual(replay,dump)
                        from rclpy.serialization import serialize_message, deserialize_message
                        from luggage_description.frame_diagnostics import BoundaryContext
                        from luggage_description.ros_frame_bindings.poses import pose_to_message, pose_from_message
                        from luggage_msgs.msg import PoseContractV2
                        context = BoundaryContext(interface_id="F23", producer="dump", consumer="replay",
                                                  request_id="request", scene_revision="scene", frame_id="container_link",
                                                  reference="box_center", time_basis="scene")
                        message = pose_to_message(replay["pose"], "slot", context)
                        serialized = serialize_message(message)
                        replay["pose"] = pose_from_message(deserialize_message(serialized, PoseContractV2), "slot", context)
                        self.assertEqual(replay["pose"], dump["pose"])
                        marker = world_marker_payload(replay,**EXPECTED)
                        center = [origin[0]+math.cos(yaw)*0.2+math.sin(yaw)*0.1,
                                  origin[1]+math.sin(yaw)*0.2-math.cos(yaw)*0.1,
                                  origin[2]+floor+support+0.16]
                        for a,b in zip(marker["position"],center): self.assertAlmostEqual(a,b,delta=1e-9)
                        for corner in itertools.product((-0.4,0.4),(-0.25,0.25),(-0.16,0.16)):
                            rotated=rotate_vector(marker["quaternion_xyzw"],corner)
                            expected=[math.cos(yaw+0.3)*corner[0]-math.sin(yaw+0.3)*corner[1],
                                      math.sin(yaw+0.3)*corner[0]+math.cos(yaw+0.3)*corner[1],corner[2]]
                            for a,b in zip(rotated,expected): self.assertAlmostEqual(a,b,delta=1e-9)
                        normal=rotate_vector(marker["quaternion_xyzw"],(0,0,1))
                        for a,b in zip(normal,(0,0,1)): self.assertAlmostEqual(a,b,delta=1e-9)
                        tool=box_to_tool_target(dict(replay["pose"],position=marker["position"],
                                                 quaternion_xyzw=marker["quaternion_xyzw"],frame_id="world"),
                                              dict(scene_revision="scene",source_frame="suction_contact_frame",target_frame="box",translation=[offset,-offset/2,0.16],quaternion_xyzw=[1,0,0,0]))
                        self.assertAlmostEqual(tool["position"][0],center[0]+math.cos(yaw+0.3)*offset+math.sin(yaw+0.3)*offset/2,delta=1e-9)
                        self.assertEqual(marker["stamp_ns"],9007199254740993)
                        self.assertIsNone(replay["surface"]["surface_z_container"][0][2])
                        log=os.environ.get("FRAME_MIGRATION_CASE_LOG")
                        if log:
                            with open(log,"a") as stream: stream.write(json.dumps(dict(case=number,sign=sign,input=dump,expected_center=center,marker=marker,status="pass"))+"\n")
            self.assertEqual(number+1,64)

    def test_dump_negatives_and_no_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"dump.json"
            write_dump(path,dump_fixture(),**EXPECTED)
            with self.assertRaises(FileExistsError): write_dump(path,dump_fixture(),**EXPECTED)
            for field,value,code in (("schema","unknown","UNKNOWN_DUMP_SCHEMA"),
                                     ("request_id","stale","DUMP_IDENTITY_MISMATCH")):
                dump=dump_fixture(); dump[field]=value
                with self.assertRaises(ContractError) as error: world_marker_payload(dump,**EXPECTED)
                self.assertEqual(error.exception.code,code)

    def test_explicit_legacy_unknowns_and_datum(self):
        original=surface(); legacy=dict(height=[[0,0.32,None]],state=original["state"],
                                       confidence=original["confidence"],origin_xy_container=[-1,-0.5],resolution_m=0.1)
        args=dict(source_format="surface_floor_relative_v1",frame_id="container_link",
                  height_datum="floor_relative",floor_z_container=0.53,ceiling_z_container=2.53,
                  identity={k:original[k] for k in ("stamp_ns","time_basis","scene_revision","request_id","geometry_hash","map_revision")})
        converted=adapt_legacy_surface(legacy,**args)
        self.assertEqual(converted["surface_z_container"],original["surface_z_container"])
        for change,code in ((dict(source_format="guess"),"UNKNOWN_LEGACY_FORMAT"),
                            (dict(height_datum="container_origin"),"LEGACY_DATUM_CONFLICT")):
            with self.assertRaises(ContractError) as error: adapt_legacy_surface(legacy,**dict(args,**change))
            self.assertEqual(error.exception.code,code)


class StartupContracts(unittest.TestCase):
    def test_real_launch_profile_resolution(self):
        from launch import LaunchContext, LaunchDescription
        from launch.actions import DeclareLaunchArgument, OpaqueFunction
        from luggage_description.launch_profile import with_profile
        with tempfile.TemporaryDirectory() as directory:
            profile = Path(directory)/"profile.yaml"
            profile.write_text("group:\n  speed: 2\n  enabled: false\n")
            context = LaunchContext()
            context.launch_configurations.update(profile_config=str(profile), speed="3")
            wrapped = with_profile(LaunchDescription([
                DeclareLaunchArgument("speed", default_value="1"),
                DeclareLaunchArgument("enabled", default_value="true")]))
            for entity in wrapped.entities:
                actions = entity.execute(context)
                if isinstance(entity, OpaqueFunction):
                    for action in actions: action.execute(context)
            self.assertEqual(context.launch_configurations["speed"], "3")
            self.assertEqual(context.launch_configurations["enabled"], "false")

    def test_resolution_precedence_and_unknown(self):
        self.assertEqual(resolve_profile(dict(a=1,b=2),dict(a=3),dict(a=4,b="__profile__")),dict(a=4,b=2))
        with self.assertRaises(ContractError): resolve_profile(dict(a=1),dict(typo=2),{})
        with self.assertRaises(ContractError): flatten_profile(dict(x=dict(a=1),y=dict(a=2)))
        for key in identity():
            bad=identity(); del bad[key]
            with self.subTest(key=key), self.assertRaises(ContractError): validate_identity(bad)

    def report(self,peer="motion",incarnation="one"):
        return dict(peer=peer,incarnation=incarnation,identity=identity(),resolved_config=dict(profile="sim"),
                    package_prefixes=dict(luggage_description="/tmp/install/luggage_description"),
                    artifacts=[dict(kind=kind,path="/tmp/install/"+kind,sha256="e"*64)
                               for kind in ("module","generated_message","type_support")])

    def test_readiness_effects_and_restart(self):
        now=[0.0]; gate=ReadinessGate(identity(),["motion"],"/tmp/install",clock=lambda:now[0])
        providers={"/execute_v2":["luggage_msgs/action/PlanMotionV2"]}; effects=[]
        def request(version=2):
            gate.require_ready(providers,version); effects.append("execute")
        with self.assertRaises(ContractError): request()
        self.assertEqual(effects,[])
        gate.observe(self.report()); request(); self.assertEqual(effects,["execute"])
        with self.assertRaises(ContractError) as error: request(1)
        self.assertEqual(error.exception.code,"LEGACY_ENDPOINT_DISABLED")
        with self.assertRaises(ContractError) as error: gate.observe(self.report(incarnation="two"))
        self.assertEqual(error.exception.code,"PEER_RESTART")
        with self.assertRaises(ContractError): request()
        self.assertEqual(effects,["execute"])

    def test_fault_codes_and_zero_effects(self):
        for fault,code in (("atlas","STALE_ATLAS"),("overlay","WRONG_OVERLAY"),
                           ("duplicate","DUPLICATE_PROVIDER"),("type","ENDPOINT_TYPE_MISMATCH"),
                           ("lease","PEER_LEASE_EXPIRED")):
            gate=ReadinessGate(identity(),["motion"],"/tmp/install",clock=lambda:0)
            report=self.report(); providers={"/execute_v2":["luggage_msgs/action/PlanMotionV2"]}; effects=[]
            with self.subTest(fault=fault), self.assertRaises(ContractError) as error:
                if fault=="atlas": report["identity"]["atlas_sha256"]="f"*64
                if fault=="overlay": report["artifacts"][0]["path"]="/other/module.py"
                gate.observe(report)
                if fault=="duplicate": providers["/execute_v2"]*=2
                if fault=="type": providers["/execute_v2"]=["luggage_msgs/action/PlanMotion"]
                if fault=="lease": gate.clock=lambda:3
                gate.require_ready(providers); effects.append("execute")
            self.assertEqual(error.exception.code,code); self.assertEqual(effects,[])


def frozen_inputs():
    return dict(hashes={key: "a"*64 for key in FROZEN_HASHES}, release_sample_interval_s=0.1,
                drift_sample_interval_s=0.1, event_order=list(EVENT_ORDER), replay_command=["replay", "case.json"])


def valid_evidence():
    cases=[]
    for number,box in enumerate(("carryon","standard")):
        request=str(number)
        cases.append(dict(box_id=box,request_id=request,
                          events=[dict(stage=stage,request_id=request,stamp_ns=i,success=True,scene_revision="a"*64,geometry_hash="a"*64) for i,stage in enumerate(EVENT_ORDER)],
                          measured=dict(integration_success=True,return_success=True,payload_retained=True,
                                        input_map_revision=number,output_map_revision=number+1,
                                        sensor_occupied_cells=1,floor_through_candidates=0),
                          gt_reference=dict(xy_error_m=0.04,z_error_m=-0.03,drift_m=0.02,
                                            roll_deg=-2,pitch_deg=2,center_inside_hull=True,
                                            footprint_reference_cells=10,footprint_sensor_cells=4)))
    return dict(complete=True,hashes=frozen_inputs()["hashes"],cases=cases)


class IntegrationExtractor(unittest.TestCase):
    def test_missing_stale_threshold_and_truncated(self):
        frozen=frozen_inputs(); evidence=valid_evidence()
        self.assertEqual(extract_acceptance(evidence,frozen)["status"],"pass")
        mutations=[lambda e:e["cases"][0]["events"].pop(),
                   lambda e:e["cases"][0]["gt_reference"].pop("xy_error_m"),
                   lambda e:e["cases"][1]["measured"].update(input_map_revision=0),
                   lambda e:e["cases"][0]["gt_reference"].update(drift_m=0.021),
                   lambda e:e.update(complete=False),
                   lambda e:e["cases"][0]["gt_reference"].update(footprint_reference_cells=0),
                   lambda e:e["cases"][0]["measured"].update(sensor_occupied_cells=0)]
        for mutation in mutations:
            broken=copy.deepcopy(evidence); mutation(broken)
            self.assertNotEqual(extract_acceptance(broken,frozen)["status"],"pass")

    def test_latency_requires_observations(self):
        self.assertEqual(capture_window([0.7,1.1],[2.3]),dict(pre_seconds=3,post_seconds=3))
        for upstream,downstream in (([],[1]),([1],[]),([0],[1])):
            with self.assertRaises(ValueError): capture_window(upstream,downstream)


if __name__=="__main__": unittest.main()
