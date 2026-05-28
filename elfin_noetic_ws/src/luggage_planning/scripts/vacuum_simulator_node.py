#!/usr/bin/env python3
"""Phase 0 stub — vacuum on/off simulation."""

import rospy
from luggage_msgs.srv import VacuumCommand, VacuumCommandResponse


class VacuumSimulator:
    def handle(self, req):
        # TODO: Gazebo link attacher + MoveIt attached collision object
        state = "ON" if req.enable else "OFF"
        rospy.logwarn("VacuumSimulator.handle not implemented — vacuum %s", state)
        return VacuumCommandResponse(success=True, message="stub vacuum %s" % state)


def main():
    rospy.init_node("vacuum_simulator")
    sim = VacuumSimulator()
    rospy.Service("~vacuum_command", VacuumCommand, sim.handle)
    rospy.loginfo("vacuum_simulator ready (stub)")
    rospy.spin()


if __name__ == "__main__":
    main()
