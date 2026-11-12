# Ordered aperture verification replay

Run from the workspace root after the recorded production build. Fixtures use
isolated ROS domains 173/174/175 and their own full scenes; do not interpret them
as S20/Gazebo/controller success. Each evidence directory must be absent before
launch. The commands below describe the executed test procedures; use a fresh
suffix for replay so original failure/success bundles remain immutable.

```bash
aperture_ev="$PWD/docs/status/evidence/moveit_atlas_joint_plan/g1/ST-4-APERTURE-ROUTING"
aperture_bin="$PWD/install-production/luggage_planning/lib/luggage_planning"
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export JOINT_NATIVE_ENDPOINT_EXECUTABLE="$aperture_bin/grasp_local_check_node"
export JOINT_NATIVE_CONNECTOR_EXECUTABLE="$aperture_bin/snapshot_joint_path_node"
export JOINT_NATIVE_CERTIFIER_EXECUTABLE="$aperture_bin/validate_route_node"

scripts/production/run.sh /usr/bin/python3 -m pytest -q src/luggage_planning/test/test_aperture_routing.py src/luggage_planning/test/test_joint_operation_scene_admission.py src/luggage_planning/test/test_joint_operation_planner.py src/luggage_planning/test/test_joint_operation_connection.py

JOINT_NATIVE_EVIDENCE_DIR="$aperture_ev/replay_connector" scripts/production/run.sh /usr/bin/python3 -m pytest -q src/luggage_planning/test/test_snapshot_joint_path_native.py

JOINT_ENDPOINT_EVIDENCE_DIR="$aperture_ev/replay_aperture" scripts/production/run.sh /usr/bin/python3 -m pytest -q src/luggage_planning/test/test_joint_operation_aperture_native.py

JOINT_ENDPOINT_EVIDENCE_DIR="$aperture_ev/replay_endpoints" scripts/production/run.sh /usr/bin/python3 -m pytest -q src/luggage_planning/test/test_joint_operation_endpoints_native.py

JOINT_CERTIFICATION_EVIDENCE_DIR="$aperture_ev/replay_certification" JOINT_NATIVE_EVIDENCE_DIR="$aperture_ev/replay_certification_connector" scripts/production/run.sh /usr/bin/python3 -m pytest -q src/luggage_planning/test/test_joint_operation_certification_native.py

JOINT_ENDPOINT_EVIDENCE_DIR="$aperture_ev/replay_prepare_g" scripts/production/run.sh /usr/bin/python3 -c 'import sys,pytest;sys.path.insert(0,"src/luggage_perception/test");sys.exit(pytest.main(["-q","src/luggage_planning/test/test_joint_operation_preparation_native.py"]))'

JOINT_ENDPOINT_EVIDENCE_DIR="$aperture_ev/replay_prepare_o" scripts/production/run.sh /usr/bin/python3 -c 'import sys,pytest;sys.path.insert(0,"src/luggage_perception/test");sys.exit(pytest.main(["-q","src/luggage_planning/test/test_joint_operation_observer_native.py"]))'
```

G and O modules run separately with fresh fixture paths; they do not launch both
map owners together. Native parameters, exact model/config, launch arguments,
serialized service inputs/outputs and executable hashes are in fixture manifests.
The full-model static replay script references already archived S20 URDF/SRDF and
CDR in the sibling integration evidence tree; copy that script to a fresh sibling
run folder before replaying because its JSON output filename is fixed.

The final build receipt captures all 13 package input hashes, dirty state and
command. `summary.json` separates harness failures/skips from accepted cases and
states all untested deployment claims. No production-cycle reliability or latency
threshold is inferred from fixture timing.
