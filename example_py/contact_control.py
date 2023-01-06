#!/usr/bin/env python

"""contact_control.py

Contact control using a dedicated contact primitive.
Note: Collision detection example can be found in 
NRT_cartesian_impedance_control.py.
"""

__copyright__ = "Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import argparse

# Utility methods
from utility import parse_pt_states
from utility import list2str

# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
# fmt: on


def main():
    # Parse Arguments
    # =============================================================================
    argparser = argparse.ArgumentParser()
    argparser.add_argument('robot_ip', help='IP address of the robot server')
    argparser.add_argument('local_ip', help='IP address of this PC')
    args = argparser.parse_args()

    # Define alias
    # =============================================================================
    log = flexivrdk.Log()
    mode = flexivrdk.Mode

    try:
        # RDK Initialization
        # =============================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_ip, args.local_ip)

        # Clear fault on robot server if any
        if robot.isFault():
            log.warn("Fault occurred on robot server, trying to clear ...")
            # Try to clear the fault
            robot.clearFault()
            time.sleep(2)
            # Check again
            if robot.isFault():
                log.error("Fault cannot be cleared, exiting ...")
                return
            log.info("Fault on robot server is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...")
        robot.enable()

        # Wait for the robot to become operational
        seconds_waited = 0
        while not robot.isOperational():
            time.sleep(1)
            seconds_waited += 1
            if seconds_waited == 10:
                log.warn(
                    "Still waiting for robot to become operational, please "
                    "check that the robot 1) has no fault, 2) is booted "
                    "into Auto mode")

        log.info("Robot is now operational")

        # Application-specific Code
        # =============================================================================
        # Set to primitive mode
        robot.setMode(mode.MODE_PRIMITIVE_EXECUTION)

        # Wait for the mode to be switched
        while (robot.getMode() != mode.MODE_PRIMITIVE_EXECUTION):
            time.sleep(1)

        # Move robot to home pose first
        log.info("Executing primitive: Home")
        robot.executePrimitive("Home()")
        # Wait for primitive completion
        while (robot.isBusy()):
            time.sleep(1)

        # IMPORTANT: must calibrate force/torque sensor before any contact
        # operation
        log.info("Executing primitive: CaliForceSensor")
        robot.executePrimitive("CaliForceSensor()")
        # Wait for primitive completion
        while (robot.isBusy()):
            time.sleep(1)

        """
        Description of the primitive [ContactCompliance]:
        This primitive maintains force control and compliantly moves the robot
        in a predefined direction with a set force until it contacts with the
        environment. Based on parameter setting, the robot stops or retracts
        away immediately when its contact force with the object reaches the set
        maxContactForce value. Users can specify the moving speed and contact
        force as input parameters. By default, the robot stops upon reaching
        maxContactForce value, but if the optional retraction target is
        specified, the robot will retract away from the contact object
        immediately, or after a period of time delay if specified, after
        reaching the set maxContactForce value. CaliForceSensor should always be
        used before executing Contact. Difference with the Contact primitive:
        Contact has higher motion stiffness which prioritize motion tracking
        performance over the accuracy of contact force control, which is not
        suitable for tasks where the object is stiff and contact force accuracy
        is more important than motion tracking accuracy. Tasks like this can be
        better achieved by ContactCompliance.
        """
        # Now calling ContactCompliance primitive with the following params:
        # 1. movingDirCoord[]: Reference coordinate for the moving direction,
        # "world" or "tcp", default to "world" if not specified.
        # 2. movingDir[]: Moving direction at user-defined coordinate (world or
        # tcp), default to "0 0 -1" if not specified.
        # 3. contactVel[m/s]: Cartesian linear velocity while moving to
        # contact, default to 0.02 m/s if not specified.
        # 4. maxContactForce[N]: the threshold contact force (absolute value)
        # which determines if a contact has happened, default to 5.0 N if not
        # specified.
        # 5. stiffnessScaling[]: scaling factor to adjust robot TCP stiffness
        # along 6 directions individually during free-space movement and
        # contact, with 1.0 being most stiff, and 0.1 being most soft, default
        # to 1.0 if not specified.
        # 6. holdTime[sec]: After reaching the set contact force, hold for this
        # period of time before terminating the primitive or retraction,
        # default to 0 seconds (no hold) if not specified.
        # 7. retractDist[m]: the distance of the retraction motion, where the
        # robot moves along the opposite of movingDir after contact, default
        # to 0 meters (no retraction) if not specified.
        # 8. retractVel[m/s]: Cartesian linear velocity while retracting,
        # default to 0.02 m/s if not specified.
        log.info("Executing primitive: ContactCompliance")
        robot.executePrimitive(
            "ContactCompliance(movingDirCoord=world, movingDir=0 0 -1, "
            "contactVel=0.02, maxContactForce=10.0, stiffnessScaling=1.0 1.0 "
            "1.0 1.0 1.0 1.0, holdTime=1.5, retractDist=0.1, retractVel=0.1)")
        # Wait for primitive completion
        while (robot.isBusy()):
            time.sleep(1)

        # A simplified call specifying only the commonly used parameters
        log.info("Executing primitive: ContactCompliance")
        robot.executePrimitive(
            "ContactCompliance(contactVel=0.02, maxContactForce=10.0, "
            "retractDist=0.02, retractVel=0.1)")
        # Wait for primitive completion
        while (robot.isBusy()):
            time.sleep(1)

        # Do some other actions after contact, for example, go to home pose
        log.info("Executing primitive: Home")
        robot.executePrimitive("Home()")
        # Wait for primitive completion
        while (robot.isBusy()):
            time.sleep(1)

        log.info("Program finished")

    except Exception as e:
        # Print exception error message
        log.error(str(e))


if __name__ == "__main__":
    main()
