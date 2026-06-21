#!/usr/bin/env python

"""basics6_gripper_control.py

This tutorial does position and force (if available) control of grippers supported by Flexiv.
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import time
import sys
import argparse
import threading
import spdlog  # pip install spdlog
import flexivrdk  # pip install flexivrdk


def print_gripper_states(gripper, logger, stop_event):
    """
    Print gripper states data @ 1Hz.

    """
    while not stop_event.is_set():
        # Print all gripper states, round all float values to 2 decimals
        logger.info("Current gripper states:")
        gripper_states = gripper.states()
        for group, states in gripper_states.items():
            print(f"[{flexivrdk.kJointGroupNames[group]}]")
            print(f"width: {round(states.width, 2)}")
            print(f"force: {round(states.force, 2)}")
            print(f"is_moving: {states.is_moving}")
        print("", flush=True)
        time.sleep(1)


def main():
    # Create an event to signal the thread to stop
    stop_event = threading.Event()

    # Program Setup
    # ==============================================================================================
    # Parse arguments
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        "robot_sn",
        help="Serial number of the robot to connect. Remove any space, e.g. Enlight-L-123456",
    )
    argparser.add_argument(
        "gripper_device_name",
        help="Full name of the device representing the gripper, can be found in Flexiv Elements->Settings->Device",
    )
    argparser.add_argument(
        "gripper_tool_name",
        help="Full name of the tool representing the gripper, can be found in Flexiv Elements->Settings->Tool",
    )
    args = argparser.parse_args()

    # Define alias
    logger = spdlog.ConsoleLogger("Example")
    mode = flexivrdk.Mode

    # Print description
    logger.info(
        ">>> Tutorial description <<<\nThis tutorial does position and force (if available) "
        "control of grippers supported by Flexiv.\n"
    )

    try:
        # RDK Initialization
        # ==========================================================================================
        # Instantiate robot interface
        robot = flexivrdk.Robot(args.robot_sn)

        # Clear fault on the connected robot if any
        if robot.fault():
            logger.warn("Fault occurred on the connected robot, trying to clear ...")
            # Try to clear the fault
            if not robot.ClearFault():
                logger.error("Fault cannot be cleared, exiting ...")
                return 1
            logger.info("Fault on the connected robot is cleared")

        # Servo on the robot, make sure the E-stop is released
        logger.info("Servo on the robot ...")
        robot.ServoOn()

        # Wait for the robot to become operational
        while not robot.operational():
            time.sleep(1)

        logger.info("Robot is now operational")

        # Gripper Control
        # ==========================================================================================
        # Instantiate gripper control interface
        gripper = flexivrdk.Gripper(robot)

        # Instantiate tool interface. Gripper is categorized as both a device and a tool. The
        # device attribute allows a gripper to be interactively controlled by the user; whereas the
        # tool attribute tells the robot to account for its mass properties and TCP location.
        tool = flexivrdk.Tool(robot)

        # Grippers can only be assigned to single-arm joint groups
        single_arm_groups = robot.info().single_arm_groups
        if not single_arm_groups:
            raise RuntimeError("No single-arm joint group found on the connected robot")

        # Enable the specified gripper as a device. This is equivalent to enabling the specified
        # gripper in Flexiv Elements -> Settings -> Device
        logger.info(
            f"Enabling gripper device [{args.gripper_device_name}] for all available single-arm joint groups"
        )
        for group in single_arm_groups:
            gripper.Enable(group, args.gripper_device_name)

        # Print parameters of the enabled gripper
        logger.info("Gripper params:")
        gripper_params = gripper.params()
        for group, params in gripper_params.items():
            print(f"[{flexivrdk.kJointGroupNames[group]}]")
            print(f"name: {params.name}")
            print(f"min_width: {round(params.min_width, 2)}")
            print(f"max_width: {round(params.max_width, 2)}")
            print(f"min_force: {round(params.min_force, 2)}")
            print(f"max_force: {round(params.max_force, 2)}")
            print(f"min_vel: {round(params.min_vel, 2)}")
            print(f"max_vel: {round(params.max_vel, 2)}")
            print("", flush=True)

        # Switch robot tool to gripper so the gravity compensation and TCP location is updated
        logger.info(
            f"Switching robot tool to [{args.gripper_tool_name}] for all available single-arm joint groups"
        )
        for group in single_arm_groups:
            tool.Switch(group, args.gripper_tool_name)

        # User needs to determine if this gripper requires manual initialization
        logger.info(
            "Manually trigger initialization for the gripper now? Choose Yes if it's a 48v Grav "
            "gripper"
        )
        print("[1] No, it has already initialized automatically when power on")
        print("[2] Yes, it does not initialize itself when power on")
        choice = int(input(""))

        # Trigger manual initialization based on input
        if choice == 1:
            logger.info("Skipped manual initialization")
        elif choice == 2:
            for group in single_arm_groups:
                gripper.Init(group)
            # User determines if the manual initialization is finished
            logger.info(
                "Triggered manual initialization, press Enter when the initialization is finished to continue"
            )
            input()
        else:
            logger.error("Invalid choice")
            return 1

        # Start a separate thread to print gripper states
        print_thread = threading.Thread(
            target=print_gripper_states, args=[gripper, logger, stop_event]
        )
        print_thread.start()

        # Position control
        logger.info("Closing gripper")
        for group, params in gripper_params.items():
            gripper.Move(group, params.min_width, params.max_vel, 0.25 * params.max_force)
        time.sleep(2)
        logger.info("Opening gripper")
        for group, params in gripper_params.items():
            gripper.Move(group, params.max_width, params.max_vel, 0.25 * params.max_force)
        time.sleep(2)

        # Stop
        logger.info("Closing gripper")
        for group, params in gripper_params.items():
            gripper.Move(group, params.min_width, params.max_vel, 0.25 * params.max_force)
        time.sleep(0.5)
        logger.info("Stopping gripper")
        for group in gripper_params:
            gripper.Stop(group)
        time.sleep(2)
        logger.info("Closing gripper")
        for group, params in gripper_params.items():
            gripper.Move(group, params.min_width, params.max_vel, 0.25 * params.max_force)
        time.sleep(2)
        logger.info("Opening gripper")
        for group, params in gripper_params.items():
            gripper.Move(group, params.max_width, params.max_vel, 0.25 * params.max_force)
        time.sleep(0.5)
        logger.info("Stopping gripper")
        for group in gripper_params:
            gripper.Stop(group)
        time.sleep(2)

        # Force control, if available (sensed force is not zero)
        gripper_states = gripper.states()
        has_force_control = any(
            abs(states.force) > sys.float_info.epsilon
            for states in gripper_states.values()
        )
        if has_force_control:
            logger.info("Gripper running zero force control")
            for group in gripper_states:
                gripper.Grasp(group, 0)
            # Exit after 10 seconds
            time.sleep(10)

        # Finished
        for group in gripper_states:
            gripper.Stop(group)

        # Stop all threads
        logger.info("Stopping print thread")
        stop_event.set()
        print_thread.join()
        logger.info("Print thread exited")
        logger.info("Program finished")

    except Exception as e:
        logger.error(str(e))
        return 1


if __name__ == "__main__":
    main()
