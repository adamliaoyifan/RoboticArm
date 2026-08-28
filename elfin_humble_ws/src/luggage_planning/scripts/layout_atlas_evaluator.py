#!/usr/bin/env python3
"""Offline layout atlas evaluator: sweep robot-base X/Y/Z positions.

Staged evaluation of robot-base movement along all three world axes:

  Phase 1  Coarse kinematic 3D sweep over (dx, dy, dz) base offsets.
           Relative-motion equivalence: base +(dx,dy,dz) <=> container -(dx,dy,dz),
           so each sample shifts the container_link translation. Ranking only.
  Phase 1b Optional local axial refine around the greedy-selected coarse stops.
  Phase 2  Collision-aware rebuild of the selected stops at full resolution.
  Output   Authoritative reliable union (REACHABLE + opening_connected) across the
           selected stops, the base-movement envelope (X/Y/Z min/max), and a
           decision (fixed / multi_axis_promising / multi_axis_insufficient).

The atlas grid lives in ``container_link`` and is invariant across base poses, so
the multi-stop union is a cell-wise OR over the shared grid. Coarse-phase slices
use a (possibly) different yaw/resolution than the authoritative phase and are
used for ranking only; the authoritative union is built from Phase 2 slices that
all share one grid definition.

Usage:
    rosrun luggage_planning layout_atlas_evaluator.py \
        _x_min:=-0.5 _x_max:=0.5 _x_step:=0.5 \
        _y_min:=-0.8 _y_max:=0.4 _y_step:=0.4 \
        _z_min:=-0.4 _z_max:=0.4 _z_step:=0.4 \
        _output_dir:=$(rospack find luggage_planning)/data/layout_atlas \
        _collision_aware_top_k:=6
"""

from __future__ import division

import copy
import math
import os
import sys
import time
import tempfile

import numpy as np
import rospy
import rospkg
import yaml

PLAN_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_planning"), "scripts")
if PLAN_SCRIPTS not in sys.path:
    sys.path.insert(0, PLAN_SCRIPTS)
DESC_SCRIPTS = os.path.join(rospkg.RosPack().get_path("luggage_description"), "scripts")
if DESC_SCRIPTS not in sys.path:
    sys.path.insert(0, DESC_SCRIPTS)

from scene_tf_config_utils import (  # noqa: E402
    load_scene_tf_config, resolve_scene_tf_config_path,
)
import layout_atlas as la  # noqa: E402

# Load builder via importlib (bypass catkin wrapper).
import importlib.util as _ilu
_spec = _ilu.spec_from_file_location(
    "reachability_atlas_builder",
    os.path.join(PLAN_SCRIPTS, "reachability_atlas_builder.py"))
_builder_mod = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_builder_mod)
ReachabilityAtlasBuilder = _builder_mod.ReachabilityAtlasBuilder


def _offset_key(offset):
    return tuple(round(float(v), 4) for v in offset)


class LayoutAtlasEvaluator:
    def __init__(self):
        self._scene_tf_path = rospy.get_param(
            "~scene_tf_config", resolve_scene_tf_config_path())
        self._base_config = load_scene_tf_config(self._scene_tf_path)
        self._baseline_xyz = la.baseline_container_xyz(self._base_config)

        # Coarse spatial sampling ranges (base offset in world, meters).
        self._x_min = float(rospy.get_param("~x_min", -0.5))
        self._x_max = float(rospy.get_param("~x_max", 0.5))
        self._x_step = float(rospy.get_param("~x_step", 0.5))
        self._y_min = float(rospy.get_param("~y_min", -0.8))
        self._y_max = float(rospy.get_param("~y_max", 0.4))
        self._y_step = float(rospy.get_param("~y_step", 0.4))
        self._z_min = float(rospy.get_param("~z_min", -0.4))
        self._z_max = float(rospy.get_param("~z_max", 0.4))
        self._z_step = float(rospy.get_param("~z_step", 0.4))

        # Refine (Phase 1b). refine_step <= 0 disables.
        self._refine_step = float(rospy.get_param("~refine_step", 0.0))
        self._max_refine_slices = int(rospy.get_param("~max_refine_slices", 48))

        # Atlas resolution / yaw per phase.
        self._coarse_resolution = float(rospy.get_param("~coarse_resolution_xyz", 0.15))
        self._refine_resolution = float(rospy.get_param("~resolution_xyz", 0.15))
        self._coarse_yaw_bins = self._parse_yaw_bins("~coarse_yaw_bins", [0.0])
        self._yaw_bins = self._parse_yaw_bins("~yaw_bins", [0.0, math.pi / 2])

        # Set-cover / collision-aware knobs.
        self._collision_aware_top_k = int(rospy.get_param("~collision_aware_top_k", 6))
        self._set_cover_max_stops = int(rospy.get_param("~set_cover_max_stops", 8))
        self._target_coverage = float(rospy.get_param("~target_coverage", 0.95))

        # Default output dir is model-specific (data/layout_atlas_s20|s30) so
        # S20 and S30 sweeps never collide. /robot_name is set by active_loading
        # from arm_model; fall back to s20 if unset.
        _default_prefix = la.model_prefix_from_robot_name(
            rospy.get_param("/robot_name", "elfin_s20_with_camera"))
        self._output_dir = rospy.get_param(
            "~output_dir",
            os.path.join(rospkg.RosPack().get_path("luggage_planning"),
                         "data", "layout_atlas_%s" % _default_prefix))

        # scene_manager node name (for setting its private scene_tf_config during
        # collision-aware sync). We deliberately do NOT touch the global
        # /luggage/scene_tf_config, which ~15 other live nodes read.
        self._scene_manager_node = rospy.get_param("~scene_manager_node", "/scene_manager")

        rospy.loginfo(
            "Layout atlas evaluator: 3D sweep X[%.2f,%.2f]/%.2f "
            "Y[%.2f,%.2f]/%.2f Z[%.2f,%.2f]/%.2f baseline_container_xyz=%s",
            self._x_min, self._x_max, self._x_step,
            self._y_min, self._y_max, self._y_step,
            self._z_min, self._z_max, self._z_step, self._baseline_xyz)

    # ── Sampling ──────────────────────────────────────────────────────

    @staticmethod
    def _parse_yaw_bins(name, default):
        """Read a yaw-bins param, accepting a list or a YAML/CSV string.

        ``rosrun node _yaw_bins:="[0.0, 1.57]"`` delivers a string, which would
        otherwise be iterated character-by-character. Launch ``<param>`` values
        arrive as already-parsed lists.
        """
        raw = rospy.get_param(name, default)
        if isinstance(raw, str):
            stripped = raw.strip()
            try:
                raw = yaml.safe_load(stripped)
            except yaml.YAMLError:
                raw = [float(v) for v in stripped.strip("[]").split(",") if v.strip() != ""]
        if not isinstance(raw, (list, tuple)) or not raw:
            return list(default)
        return [float(v) for v in raw]

    @staticmethod
    def _axis_values(vmin, vmax, step):
        vals = []
        v = vmin
        while v <= vmax + 1e-6:
            vals.append(round(v, 4))
            v += step
        return vals

    def _coarse_offsets(self):
        xs = self._axis_values(self._x_min, self._x_max, self._x_step)
        ys = self._axis_values(self._y_min, self._y_max, self._y_step)
        zs = self._axis_values(self._z_min, self._z_max, self._z_step)
        offsets = [(x, y, z) for x in xs for y in ys for z in zs]
        rospy.loginfo("Coarse grid: %d x-values x %d y-values x %d z-values = %d offsets",
                      len(xs), len(ys), len(zs), len(offsets))
        return offsets

    def _refine_candidates(self, selected_offsets):
        """Axial ±refine_step neighbors of selected stops (dedup, bounded)."""
        seen = set(_offset_key(o) for o in selected_offsets)
        candidates = []
        for off in selected_offsets:
            dx, dy, dz = off
            for axis in range(3):
                for sign in (-1.0, 1.0):
                    delta = [0.0, 0.0, 0.0]
                    delta[axis] = sign * self._refine_step
                    noff = (round(dx + delta[0], 4),
                            round(dy + delta[1], 4),
                            round(dz + delta[2], 4))
                    key = _offset_key(noff)
                    if key in seen:
                        continue
                    seen.add(key)
                    candidates.append(noff)
        if len(candidates) > self._max_refine_slices:
            rospy.logwarn(
                "Refine candidates %d > max_refine_slices %d; truncating "
                "(increase max_refine_slices or refine_step to cover more).",
                len(candidates), self._max_refine_slices)
            candidates = candidates[:self._max_refine_slices]
        return candidates

    # ── Slice building ────────────────────────────────────────────────

    def _write_effective_yaml(self, dx, dy, dz):
        eff_config = la.effective_scene_tf_xyz(self._base_config, dx, dy, dz)
        fd, path = tempfile.mkstemp(suffix=".yaml", prefix="layout_eff_")
        with os.fdopen(fd, "w") as f:
            yaml.safe_dump(eff_config, f, default_flow_style=False, sort_keys=False)
        return path

    def _sync_scene_with_config(self, config_path, label="", settle=2.0):
        """Sync scene_manager's PlanningScene against a specific scene_tf config.

        Sets scene_manager's PRIVATE ``~scene_tf_config`` to ``config_path`` so
        it loads that config (container at the shifted pose), calls
        ``sync_static_scene``, then deletes the private param so scene_manager
        reverts to its normal resolution. The global ``/luggage/scene_tf_config``
        is never touched. Returns True on success.
        """
        sm_param = self._scene_manager_node + "/scene_tf_config"
        try:
            from std_srvs.srv import Trigger
            rospy.set_param(sm_param, config_path)
            sync = rospy.ServiceProxy("/scene_manager/sync_static_scene", Trigger)
            sync.wait_for_service(timeout=5.0)
            sync()
            rospy.sleep(settle)
            return True
        except Exception as exc:
            rospy.logwarn("Scene sync failed (%s): %s", label or config_path, exc)
            return False
        finally:
            try:
                rospy.delete_param(sm_param)
            except Exception:
                pass

    def _build_slice(self, offset, avoid_collisions, resolution, yaw_bins):
        """Build one atlas slice for a (dx, dy, dz) base offset.

        Returns a slice dict or None on failure.
        """
        dx, dy, dz = offset
        eff_path = self._write_effective_yaml(dx, dy, dz)
        try:
            rospy.set_param("~scene_tf_config", eff_path)
            rospy.set_param("~avoid_collisions", avoid_collisions)
            rospy.set_param("~resolution_xyz", resolution)
            rospy.set_param("~yaw_bins", list(yaw_bins))
            rospy.set_param("~output_dir", "")  # compute() does not save

            if avoid_collisions:
                # PlanningScene holds the shifted mesh after sync; the private
                # param is deleted inside the helper so the global stays pristine.
                if not self._sync_scene_with_config(eff_path, label="offset=%s" % (offset,)):
                    return None

            builder = ReachabilityAtlasBuilder()
            data, meta = builder.compute()
            score = la.score_fixed_layout(data, meta)
            meta_layout = copy.deepcopy(meta)
            meta_layout["layout"] = {
                "sweep_axes": ["x", "y", "z"],
                "base_offset": [dx, dy, dz],
                "base_x": dx, "base_y": dy, "base_z": dz,
                "equivalent_container_offset": [-dx, -dy, -dz],
                "baseline_container_xyz": list(self._baseline_xyz),
            }
            return {
                "offset": (dx, dy, dz),
                "base_offset": [dx, dy, dz],
                "data": data,
                "meta": meta_layout,
                "score": score,
                "mask": la.reliable_coverage_mask(data),
                "opening_connected": data.get("opening_connected"),
                "joint_margin": data.get("joint_margin"),
                "neighbor_confidence": data.get("neighbor_confidence"),
            }
        except Exception as exc:
            rospy.logwarn("Slice offset=%s failed: %s", offset, exc)
            return None
        finally:
            try:
                os.unlink(eff_path)
            except OSError:
                pass
            rospy.set_param("~scene_tf_config", self._scene_tf_path)

    @staticmethod
    def _verify_union_grid(slices):
        """Log and drop slices whose grid differs from the first (no silent mix)."""
        if len(slices) <= 1:
            return slices
        base_meta = slices[0]["meta"]
        kept = [slices[0]]
        for s in slices[1:]:
            ok, reason = la.verify_grid_compatibility(base_meta, s["meta"])
            if ok:
                kept.append(s)
            else:
                rospy.logwarn("Dropping offset %s from union: %s", s["offset"], reason)
        return kept

    # ── Run ───────────────────────────────────────────────────────────

    def run(self):
        t0 = time.time()
        try:
            self._run_sweep(t0)
        finally:
            # Restore the PlanningScene to the baseline container pose so the
            # running system isn't left with a shifted container mesh after the
            # sweep (collision-aware slices reposition it per offset).
            rospy.loginfo("Restoring PlanningScene to baseline %s ...", self._scene_tf_path)
            self._sync_scene_with_config(
                self._scene_tf_path, label="baseline restore", settle=1.0)

    def _run_sweep(self, t0):
        # ── Phase 1: coarse kinematic 3D sweep ────────────────────────
        coarse_offsets = self._coarse_offsets()
        rospy.loginfo("=== Phase 1: coarse kinematic sweep (%d offsets) ===",
                      len(coarse_offsets))
        coarse_slices = []
        for i, off in enumerate(coarse_offsets):
            rospy.loginfo("[%d/%d] coarse offset=%s ...", i + 1, len(coarse_offsets), off)
            sl = self._build_slice(off, avoid_collisions=False,
                                   resolution=self._coarse_resolution,
                                   yaw_bins=self._coarse_yaw_bins)
            if sl is not None:
                coarse_slices.append(sl)
                rospy.loginfo("  offset=%s coverage=%.1f%%",
                              off, sl["score"]["coverage_rate"] * 100)

        if not coarse_slices:
            rospy.logerr("No valid coarse slices. Aborting.")
            return

        coarse_cover = la.greedy_set_cover(
            coarse_slices, max_stops=self._set_cover_max_stops,
            target_coverage=self._target_coverage)
        rospy.loginfo("Coarse union=%.1f%% (%d stops selected of %d)",
                      coarse_cover["coverage_rate"] * 100,
                      len(coarse_cover["selected"]), len(coarse_slices))

        # ── Phase 1b: optional local refine around selected stops ─────
        pool = [coarse_slices[i] for i in coarse_cover["selected"]]
        if self._refine_step > 0:
            candidates = self._refine_candidates([s["offset"] for s in pool])
            rospy.loginfo("=== Phase 1b: refine %d candidates (step=%.2f) ===",
                          len(candidates), self._refine_step)
            refine_slices = []
            for i, off in enumerate(candidates):
                rospy.loginfo("[%d/%d] refine offset=%s ...", i + 1, len(candidates), off)
                sl = self._build_slice(off, avoid_collisions=False,
                                       resolution=self._coarse_resolution,
                                       yaw_bins=self._coarse_yaw_bins)
                if sl is not None:
                    refine_slices.append(sl)
            pool = pool + refine_slices

        refined_cover = la.greedy_set_cover(
            pool, max_stops=self._set_cover_max_stops,
            target_coverage=self._target_coverage)
        refined_selected = [pool[i] for i in refined_cover["selected"]]
        rospy.loginfo("Refined kinematic union=%.1f%% (%d stops)",
                      refined_cover["coverage_rate"] * 100, len(refined_selected))

        # ── Phase 2: collision-aware rebuild of top-K refined stops ───
        # Rank by reliable coverage contribution (mask size), take top-K offsets.
        refined_selected.sort(key=lambda s: -int(np.count_nonzero(s["mask"])))
        top_k = min(self._collision_aware_top_k, len(refined_selected))
        coll_offsets = [s["offset"] for s in refined_selected[:top_k]]
        rospy.loginfo("=== Phase 2: collision-aware top-%d ===", len(coll_offsets))
        coll_slices = []
        for i, off in enumerate(coll_offsets):
            rospy.loginfo("[%d/%d] collision-aware offset=%s ...",
                          i + 1, len(coll_offsets), off)
            sl = self._build_slice(off, avoid_collisions=True,
                                   resolution=self._refine_resolution,
                                   yaw_bins=self._yaw_bins)
            if sl is not None:
                coll_slices.append(sl)
                rospy.loginfo("  offset=%s coverage=%.1f%%",
                              off, sl["score"]["coverage_rate"] * 100)

        # Fallback: if collision-aware produced nothing, use refined kinematic
        # stops (rebuilt at fine resolution/yaw) so the union is still defined.
        if coll_slices:
            union_slices = coll_slices
            union_mode = "collision_aware"
        else:
            rospy.logwarn("No collision-aware slices; falling back to refined kinematic.")
            union_slices = []
            for off in coll_offsets:
                sl = self._build_slice(off, avoid_collisions=False,
                                       resolution=self._refine_resolution,
                                       yaw_bins=self._yaw_bins)
                if sl is not None:
                    union_slices.append(sl)
            union_mode = "kinematic_fallback"
            if not union_slices:
                rospy.logerr("No usable slices for union. Aborting.")
                return

        union_slices = self._verify_union_grid(union_slices)

        # ── Authoritative union + envelope + decision ─────────────────
        union_cover = la.greedy_set_cover(
            union_slices, max_stops=self._set_cover_max_stops,
            target_coverage=self._target_coverage)
        union_artifact = la.build_union_artifact(union_cover, union_slices)
        envelope = la.base_movement_envelope(union_cover["selected_offsets"])

        # Baseline (0,0,0) and best fixed single pose.
        zero = (0.0, 0.0, 0.0)
        baseline_slice = next(
            (s for s in union_slices if _offset_key(s["offset"]) == _offset_key(zero)),
            None)
        if baseline_slice is None:
            # (0,0,0) not in collision-aware top-K: build a real baseline slice
            # so the decision compares against the actual origin. Falling back to
            # union_slices[0] is wrong when union_slices[0] is also best_fixed --
            # then baseline_score == best_score and `best_rate <= baseline_rate*1.05`
            # is trivially true, masking any real multi-axis gain (E4).
            rospy.loginfo("Baseline (0,0,0) not in top-K; building real baseline slice.")
            baseline_slice = self._build_slice(
                zero, avoid_collisions=(union_mode == "collision_aware"),
                resolution=self._refine_resolution, yaw_bins=self._yaw_bins)
            if baseline_slice is None:
                rospy.logwarn(
                    "Baseline (0,0,0) build failed; falling back to union_slices[0].")
                baseline_slice = union_slices[0]
        baseline_score = baseline_slice["score"]
        best_fixed = max(union_slices, key=lambda s: s["score"]["score"])
        best_score = best_fixed["score"]

        decision = la.evaluate_decision(
            baseline_score, best_score, union_cover, multi_axis=True)

        region_blind = la.depth_lateral_layer_stats(
            union_cover["union_mask"], union_slices[0]["meta"]) \
            if union_cover["union_mask"] is not None else {}

        elapsed = time.time() - t0
        summary = {
            "run_id": time.strftime("%Y%m%d_%H%M%S"),
            "baseline_container_xyz": list(self._baseline_xyz),
            "union_mode": union_mode,
            "sweep": {
                "axes": ["x", "y", "z"],
                "x": {"min": self._x_min, "max": self._x_max, "step": self._x_step},
                "y": {"min": self._y_min, "max": self._y_max, "step": self._y_step},
                "z": {"min": self._z_min, "max": self._z_max, "step": self._z_step},
                "coarse_resolution_xyz": self._coarse_resolution,
                "refine_resolution_xyz": self._refine_resolution,
                "refine_step": self._refine_step,
                "coarse_yaw_bins": list(self._coarse_yaw_bins),
                "yaw_bins": list(self._yaw_bins),
            },
            "coarse": {
                "n_offsets": len(coarse_offsets),
                "n_valid": len(coarse_slices),
                "coverage_rate": coarse_cover["coverage_rate"],
            },
            "refined_kinematic": {
                "n_stops": len(refined_selected),
                "coverage_rate": refined_cover["coverage_rate"],
            },
            "collision_aware_slices": [
                {"offset": list(s["offset"]),
                 "coverage_rate": s["score"]["coverage_rate"],
                 "score": s["score"]["score"]}
                for s in coll_slices
            ],
            "selected_stops": [
                {"offset": list(union_slices[i]["offset"]),
                 "coverage_rate": union_slices[i]["score"]["coverage_rate"]}
                for i in union_cover["selected"]
            ],
            "base_movement_envelope": envelope,
            "union": {
                "mode": union_mode,
                "coverage_rate": union_cover["coverage_rate"],
                "covered_cells": union_cover["total_covered"],
                "total_cells": union_cover["total_cells"],
                "remaining_blind": union_cover["remaining_blind"],
                "region_coverage": region_blind,
            },
            "baseline": {"offset": list(baseline_slice["offset"]), "score": baseline_score},
            "best_fixed": {"offset": list(best_fixed["offset"]), "score": best_score},
            "decision": decision,
            "computation_time_sec": round(elapsed, 1),
        }

        # ── Save ──────────────────────────────────────────────────────
        os.makedirs(self._output_dir, exist_ok=True)
        summary_path = os.path.join(self._output_dir, "summary.yaml")
        with open(summary_path, "w") as f:
            yaml.safe_dump(summary, f, default_flow_style=False, sort_keys=False)

        if union_artifact:
            union_path = os.path.join(self._output_dir, "union.npz")
            np.savez_compressed(union_path, **union_artifact)

        # Save grid + container meta so layout_atlas_viz is self-contained and
        # does not require a separately-built reachability atlas. Grid/container
        # are arm-independent (they describe the container_link frame).
        if union_slices:
            viz_meta = {
                "grid": union_slices[0]["meta"].get("grid"),
                "container": union_slices[0]["meta"].get("container"),
                "source": "layout_atlas_evaluator sweep",
                "robot_model": rospy.get_param("/robot_name", "elfin_s20_with_camera"),
            }
            meta_out_path = os.path.join(self._output_dir, "meta.yaml")
            with open(meta_out_path, "w") as f:
                yaml.safe_dump(viz_meta, f, default_flow_style=False, sort_keys=False)

        rospy.loginfo("=== Layout atlas 3D evaluation complete (%.1fs) ===", elapsed)
        rospy.loginfo("Decision: %s - %s",
                      decision["recommendation"], decision["reason"])
        rospy.loginfo("Best fixed offset=%s coverage=%.1f%%",
                      best_fixed["offset"], best_score["coverage_rate"] * 100)
        rospy.loginfo("Union coverage=%.1f%% (%d stops, mode=%s)",
                      union_cover["coverage_rate"] * 100,
                      len(union_cover["selected"]), union_mode)
        rospy.loginfo(
            "Base movement envelope: X[%.2f,%.2f] Y[%.2f,%.2f] Z[%.2f,%.2f] (%d stops)",
            envelope["x"]["min"] or 0.0, envelope["x"]["max"] or 0.0,
            envelope["y"]["min"] or 0.0, envelope["y"]["max"] or 0.0,
            envelope["z"]["min"] or 0.0, envelope["z"]["max"] or 0.0,
            envelope["x"]["count"])
        rospy.loginfo("Summary saved to %s", summary_path)


def main():
    rospy.init_node("layout_atlas_evaluator")
    evaluator = LayoutAtlasEvaluator()
    evaluator.run()


if __name__ == "__main__":
    main()
