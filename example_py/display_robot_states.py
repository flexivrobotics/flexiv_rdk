#!/usr/bin/env python

"""display_robot_states.py

Print received robot states without enabling the robot.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib/python/x64/")
import flexivrdk
# fmt: on


def main():
    # Parse Arguments
    # =============================================================================
    argparser = argparse.ArgumentParser()
    argparser.add_argument('robot_ip', help='IP address of the robot server')
    argparser.add_argument('local_ip', help='IP address of the workstation PC')
    args = argparser.parse_args()

    # Define alias
    # =============================================================================
    robot_states = flexivrdk.RobotStates()
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    try:
        # RDK Initialization
        # =============================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_ip, args.local_ip)

        # Application-specific Code
        # =============================================================================
        while True:
            robot.getRobotStates(robot_states)
            print("q: ", robot_states.m_q)
            print("theta: ", robot_states.m_theta)
            print("dq: ", robot_states.m_dq)
            print("dtheta: ", robot_states.m_dtheta)
            print("tau: ", robot_states.m_tau)
            print("tau_des: ", robot_states.m_tauDes)
            print("tau_dot: ", robot_states.m_tauDot)
            print("tau_ext: ", robot_states.m_tauExt)
            print("tcp_pose: ", robot_states.m_tcpPose)
            print("tcp_pose_d: ", robot_states.m_tcpPoseDes)
            print("tcp_velocity: ", robot_states.m_tcpVel)
            print("camera_pose: ", robot_states.m_camPose)
            print("flange_pose: ", robot_states.m_flangePose)
            print("end_link_pose: ", robot_states.m_endLinkPose)
            print("F_ext_tcp_frame: ", robot_states.m_extForceInTcpFrame)
            print("F_ext_base_frame: ", robot_states.m_extForceInBaseFrame)
            log.info("==" * 30)
            time.sleep(1)

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
