#!/usr/bin/env python3
"""Diagnostic: trace hop destinations of the first graph plan (one candidate)."""
import json, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import g2_common as common
import g2_frozen_select as frozen


class _Stop(BaseException):
    pass


def main():
    out = sys.argv[1]
    import rclpy
    from rclpy.node import Node
    from luggage_description.scene_tf_config_utils import load_scene_tf_config
    from luggage_msgs.srv import SelectPlacePathV2, ValidateRouteV2
    from luggage_planning.place_service import fill_select_response_route
    from luggage_planning.routing import grid_search as gps
    import slot64_classify_replay as classify

    stack = common.MoveItStack(out, "debug_hops")
    stack.start()
    rclpy.init()
    node = Node("g2_debug_hops")
    stop, thread = common.score._spin(node)
    try:
        request_msg = common.load_request()
        surface = json.load(open(common.SURFACE_JSON, encoding="utf-8"))
        surface["floor_z_container"] = float(request_msg.surface.floor_z_container)
        scene_config = load_scene_tf_config(common.SCENE_TF)
        node.create_client(ValidateRouteV2, "/validate_route_v2").wait_for_service(120.0)
        client = common.paired._scene_client(node)
        classify._sync_world(node, scene_config, {"world_objects": []})
        common.paired._purge(client)
        producer, clock, _ = frozen.build_host_and_producer(node, request_msg, surface, scene_config)
        producer.edge_builder.wait_ready(timeout_sec=60.0)
        from luggage_planning.atlas import graph as rg
        refine_log = []
        orig_ref = rg.GraphRoutePlanner._refine_one

        def ref(self, position, quaternion, seed_q):
            result = orig_ref(self, position, quaternion, seed_q)
            refine_log.append({"pos": [round(v, 3) for v in position], "quat": [round(v, 3) for v in quaternion],
                               "seed": [round(v, 3) for v in seed_q],
                               "q": None if result is None else [round(v, 3) for v in result]})
            return result
        rg.GraphRoutePlanner._refine_one = ref
        orig_cs = gps.RouteSearch._chain_from

        def chain(self, frontier, hops, goal_nodes, acc_edges, acc_nodes, scene=None):
            try:
                return orig_cs(self, frontier, hops, goal_nodes, acc_edges, acc_nodes, scene)
            except gps.RoutePlanningError as err:
                if err.detail.get("retreat_target_missing"):
                    print(json.dumps({"retreat_missing_at": frontier.node_id, "region": frontier.region,
                                      "load": str(frontier.load_state), "q": [round(v, 3) for v in frontier.q],
                                      "hop": str(hops[0]), "frontier_quat": [round(v,4) for v in frontier.tcp_quaternion_xyzw], "entry_ids": [n[:22] for n in getattr(self, "entry_node_ids", ())],
                                      "entry_nodes": [(n.node_id[:22], n.region, str(n.load_state), n.ik_family, [round(v,3) for v in n.q], [round(v,3) for v in n.tcp_position], [round(v,4) for v in n.tcp_quaternion_xyzw]) for n in (self.graph.nodes.get(i) for i in getattr(self, "entry_node_ids", ())) if n is not None], "portals": [(n.node_id[:22], str(n.load_state), n.ik_family, [round(v,4) for v in n.tcp_quaternion_xyzw], n.node_id in (self_planner[0].scope or ()), [round(v,3) for v in n.q], [round(v,4) for v in n.tcp_position], list(__import__("luggage_planning.routing.grid_search", fromlist=["x"]).descend_pair_rank(frontier.q, n.q))) for n in self.graph.nodes_by_region("portal")], "last_refines": refine_log[-4:],
                                      "region_poses": {k: [round(x, 3) for x in v] for k, v in (self_planner[0].region_poses or {}).items()}}, default=str))
                    raise _Stop()
                raise
        self_planner = [None]
        gps.RouteSearch._chain_from = chain
        orig_init = rg.GraphRoutePlanner.__init__

        def init(self, *a, **k):
            orig_init(self, *a, **k)
            self_planner[0] = self
        rg.GraphRoutePlanner.__init__ = init
        try:
            fill_select_response_route(request_msg, SelectPlacePathV2.Response(), producer,
                                       surface_validity={"occupancy_confirmed": True, "change_event": ""})
        except _Stop:
            pass
        producer.edge_builder.reset_scene()
    finally:
        stop.set(); thread.join(timeout=2.0); node.destroy_node(); rclpy.shutdown(); stack.stop()


main()
