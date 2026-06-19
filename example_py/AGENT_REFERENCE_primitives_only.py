#!/usr/bin/env python

"""AGENT_REFERENCE_primitives_only.py

Comprehensive primitive-only reference script for code-generation agents.

Primitives demonstrated (in order):
    1. Home          - move robot to the platform-defined home posture.
    2. ZeroFTSensor  - zero force/torque sensors for all active single-arm groups.
    3. MoveJ         - joint-space move with waypoints and joint-velocity scaling.
    4. MoveL         - Cartesian linear move in WORLD frame with velocity, blending (zoneRadius).
    5. MoveL         - multi-waypoint sweep in WORLD frame.
    6. Home          - return to home before TCP-frame relative move.
    7. MoveL         - relative move in TCP frame (TRAJ::START), orientation-only rotation.
    8. MoveJ         - return to upright posture (all joints 0).
    9. Home          - return to home.

Key patterns shown:
    - Fault clear, enable, wait operational, mode switch (essential setup).
    - exec_prim() wrapper accepts group-indexed PrimitiveArgs for per-group primitives.
    - Transition keys must be looked up per primitive from Flexiv primitive documentation:
        https://www.flexiv.com/software/primitives
    - Reading tcp_pose from each joint group to extract live orientation.
    - Converting quaternion to Euler ZYX degrees using utility.quat2eulerZYX for MoveL orientation.
    - ZeroFTSensor is executed for all selected groups with explicit completion checks.
    - MoveL with optional velocity, zoneRadius blending, and waypoints.
    - MoveJ with optional waypoints list and jntVelScale.
    - All JPos calls use 7-element arm-joint list; second (external-axis) argument is optional.
    - All Coord calls use positional args: (position, orientation_eulerZYX_deg, ref_frame_list).
"""

__copyright__ = "Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved."
__author__ = "Flexiv"

import argparse
import time
import spdlog  # pip install spdlog
import flexivrdk  # pip install flexivrdk
import utility


# ──────────────────────────────────────────────────────────────────────────────
# Helpers
# ──────────────────────────────────────────────────────────────────────────────
def normalize_euler_deg(euler_deg):
    """Normalize one angle or a 3-element Euler list to the robot's [-180, 180] convention."""

    def normalize_angle(angle_deg):
        normalized = (angle_deg + 180.0) % 360.0 - 180.0
        if normalized == -180.0 and angle_deg > 0:
            return 180.0
        return normalized

    if isinstance(euler_deg, (int, float)):
        return normalize_angle(euler_deg)

    return [normalize_angle(angle) for angle in euler_deg]


def wait_until_operational(robot, dt=1.0, timeout_s=120.0):
    """Block until robot.operational() is True, or raise TimeoutError."""
    t0 = time.time()
    while not robot.operational():
        if time.time() - t0 > timeout_s:
            raise TimeoutError("Timed out waiting for robot to become operational")
        time.sleep(dt)


def wait_primitive_transition(robot, transition_keys_by_group, dt=0.2, timeout_s=60.0):
    """Block until primitive_states() for all groups satisfy their transition key.

    Use the primitive's default transition key unless a custom transition condition
    is intended. Check primitive docs to confirm available primitive state keys.
    """
    t0 = time.time()
    while True:
        if utility.primitive_state_true_for_groups(
            robot.primitive_states(),
            transition_keys_by_group,
        ):
            return

        if time.time() - t0 > timeout_s:
            raise TimeoutError(
                "Timed out waiting for primitive transition states: "
                f"{ {flexivrdk.kJointGroupNames[g]: k for g, k in transition_keys_by_group.items()} }"
            )
        time.sleep(dt)


def exec_prim(robot, primitive_args_by_group, transition_keys_by_group):
    """Execute primitive command(s) and block until all transitions are satisfied.

    Args:
        robot:          flexivrdk.Robot instance.
        primitive_args_by_group:
            Dict[JointGroup, flexivrdk.PrimitiveArgs] for per-group primitive commands.
        transition_keys_by_group:
            Dict[JointGroup, str] where each value is the transition key for that group.
            This can be the primitive's default key or another primitive state key.
            Look up primitive states from:
            https://www.flexiv.com/software/primitives

        Note:
        This helper waits on primitive_states() only. If transition depends on
        robot states, implement a custom wait loop at call site.
    """
    if not primitive_args_by_group:
        raise ValueError("primitive_args_by_group cannot be empty")

    if set(primitive_args_by_group.keys()) != set(transition_keys_by_group.keys()):
        raise ValueError(
            "primitive_args_by_group and transition_keys_by_group must have identical groups"
        )

    robot.ExecutePrimitive(primitive_args_by_group)
    wait_primitive_transition(robot, transition_keys_by_group)


def prepare_robot(robot_sn, logger):
    """Connect, clear faults, enable robot, and switch to primitive execution mode."""
    robot = flexivrdk.Robot(robot_sn)

    if robot.fault():
        logger.warn("Fault detected on connected robot, trying to clear")
        if not robot.ClearFault():
            raise RuntimeError("Failed to clear robot fault")
        logger.info("Fault cleared")

    logger.info("Enabling robot")
    robot.ServoOn()
    wait_until_operational(robot)
    logger.info("Robot is operational")

    robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
    return robot


def current_euler_zyx_deg(robot, group):
    """Return live TCP orientation of a specific joint group as Euler ZYX [x,y,z] degrees.

    tcp_pose layout: [x, y, z, q_w, q_x, q_y, q_z] (SI units, quaternion).
    utility.quat2eulerZYX() expects [w, x, y, z] order.
    The returned Euler values are normalized to the robot's [-180, 180] convention.
    """
    all_states = robot.states()
    if group not in all_states:
        raise ValueError(
            f"Requested joint group has no available robot state: "
            f"{flexivrdk.kJointGroupNames[group]}"
        )

    pose = all_states[group].tcp_pose
    return utility.quat2eulerZYX([pose[3], pose[4], pose[5], pose[6]], degree=True)


# ──────────────────────────────────────────────────────────────────────────────
# Main sequence
# ──────────────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "robot_sn",
        help="Serial number of the robot to connect. Remove spaces, e.g. Enlight-L-123456",
    )
    args = parser.parse_args()

    logger = spdlog.ConsoleLogger("AgentReference")

    try:
        robot = prepare_robot(args.robot_sn, logger)
        single_arm_groups = robot.info().single_arm_groups
        if not single_arm_groups:
            raise RuntimeError("No single-arm joint group found on the connected robot")

        # ── 1) Move to home pose ──────────────────────────────────────────────
        logger.info("Step 1: Move to home pose")
        robot.Home()

        # ── 2) ZeroFTSensor ───────────────────────────────────────────────────
        # Use this primitive's documented default transition condition.
        # The robot must not be in contact with anything during zeroing.
        logger.info("Step 2: ZeroFTSensor")
        logger.warn("Zeroing F/T sensors - ensure nothing contacts the robot")
        exec_prim(
            robot,
            {
                group: flexivrdk.PrimitiveArgs(
                    "ZeroFTSensor",
                    {
                        "dataCollectTime": 0.2,
                        "enableStaticCheck": False,
                        "calibExtraPayload": False,
                    },
                )
                for group in single_arm_groups
            },
            {group: "terminated" for group in single_arm_groups},
        )
        logger.info("F/T sensor zeroing complete")

        # ── 3) MoveJ with waypoints and velocity scaling ──────────────────────
        # target / waypoints: flexivrdk.JPos(q_m) where q_m is 7-element degrees list.
        # jntVelScale: percentage of max joint speed [1-100].
        logger.info("Step 3: MoveJ with waypoints")
        exec_prim(
            robot,
            {
                group: flexivrdk.PrimitiveArgs(
                    "MoveJ",
                    {
                        "target": flexivrdk.JPos([30, -45, 0, 90, 0, 40, 30]),
                        "waypoints": [
                            flexivrdk.JPos([10, -30, 10, 30, 10, 15, 10]),
                            flexivrdk.JPos([20, -60, -10, 60, -10, 30, 20]),
                        ],
                        "jntVelScale": 50,  # 50% of max joint speed
                        "zoneRadius": "Z50",
                        "targetTolerLevel": 1,
                        "enableRelativeMove": False,
                        "jntAccMultiplier": 1,
                    },
                )
                for group in single_arm_groups
            },
            {group: "reachedTarget" for group in single_arm_groups},
        )

        # ── 4) MoveL in WORLD frame with velocity and blending ────────────────
        # target / waypoints: flexivrdk.Coord(position_m, orientation_eulerZYX_deg, ref_frame).
        # ref_frame is a 2-element list: [frame_type, frame_origin].
        # vel: TCP linear speed in m/s.
        # zoneRadius: blending zone size string, e.g. "Z50". Use "Z0" for exact stops.
        step4_primitive_args_by_group = {}
        for group in single_arm_groups:
            live_euler = current_euler_zyx_deg(robot, group)
            wp1_euler = normalize_euler_deg(
                [live_euler[0] + 20.0, live_euler[1] - 20.0, live_euler[2] + 10.0]
            )
            wp2_euler = normalize_euler_deg(
                [live_euler[0] - 30.0, live_euler[1] + 30.0, live_euler[2] - 30.0]
            )
            target_euler = normalize_euler_deg(
                [live_euler[0] + 40.0, live_euler[1] - 20.0, live_euler[2] + 20.0]
            )
            logger.info(
                f"Step 4 [{flexivrdk.kJointGroupNames[group]}] live Euler ZYX (deg): {live_euler}"
            )
            logger.info(
                f"Step 4 [{flexivrdk.kJointGroupNames[group]}] waypoint1 Euler ZYX (deg): {wp1_euler}"
            )
            logger.info(
                f"Step 4 [{flexivrdk.kJointGroupNames[group]}] waypoint2 Euler ZYX (deg): {wp2_euler}"
            )
            logger.info(
                f"Step 4 [{flexivrdk.kJointGroupNames[group]}] target Euler ZYX (deg): {target_euler}"
            )

            step4_primitive_args_by_group[group] = flexivrdk.PrimitiveArgs(
                "MoveL",
                {
                    "target": flexivrdk.Coord(
                        [0.65, -0.3, 0.3], target_euler, ["WORLD", "WORLD_ORIGIN"]
                    ),
                    "waypoints": [
                        flexivrdk.Coord(
                            [0.45, 0.1, 0.3], wp1_euler, ["WORLD", "WORLD_ORIGIN"]
                        ),
                        flexivrdk.Coord(
                            [0.45, -0.3, 0.3], wp2_euler, ["WORLD", "WORLD_ORIGIN"]
                        ),
                    ],
                    "vel": 0.3,  # m/s TCP linear speed
                    "zoneRadius": "Z50",  # blended corners; use "Z0" for hard stops
                    "targetTolerLevel": 3,
                    "acc": 1.5,
                    "angVel": 150,
                    "enableFixRefJntPos": False,
                    "refJntPos": flexivrdk.JPos([0, -40, 0, 90, 0, 40, 0]),
                    "jerk": 50,
                    "configOptObj": [0, 0, 0.5],
                    "enableSixAxisJntCtrl": False,
                    "enableNullspaceTraj": False,
                },
            )

        logger.info("Step 4: MoveL in WORLD frame with speed and blending")
        exec_prim(
            robot,
            step4_primitive_args_by_group,
            {group: "reachedTarget" for group in single_arm_groups},
        )

        # ── 5) MoveL multi-point sweep, exact stops ───────────────────────────
        logger.info("Step 5: MoveL sweep with exact stops")
        for target_pos in [
            [0.5, 0.2, 0.4],
            [0.5, -0.2, 0.4],
            [0.5, -0.2, 0.2],
            [0.5, 0.2, 0.2],
        ]:
            exec_prim(
                robot,
                {
                    group: flexivrdk.PrimitiveArgs(
                        "MoveL",
                        {
                            "target": flexivrdk.Coord(
                                target_pos, [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]
                            ),
                            "vel": 0.2,
                            "zoneRadius": "Z0",  # exact stop at each corner
                            "targetTolerLevel": 3,
                            "acc": 1.5,
                            "angVel": 150,
                            "enableFixRefJntPos": False,
                            "refJntPos": flexivrdk.JPos([0, -40, 0, 90, 0, 40, 0]),
                            "jerk": 50,
                            "configOptObj": [0, 0, 0.5],
                            "enableSixAxisJntCtrl": False,
                            "enableNullspaceTraj": False,
                        },
                    )
                    for group in single_arm_groups
                },
                {group: "reachedTarget" for group in single_arm_groups},
            )

        # ── 6) Home before TCP-frame relative move ────────────────────────────
        logger.info("Step 6: Move to home pose")
        robot.Home()

        # ── 7) MoveL in TCP frame (TRAJ::START) - relative orientation change ─
        # TRAJ::START means both translation and orientation targets are relative
        # to the current TCP pose at primitive start.
        # Setting position to [0, 0, 0] keeps the TCP at the same location.
        # Orientation [20, 0, 0] means a +20 deg relative rotation around X.
        logger.info("Step 7: MoveL relative orientation-only rotation in TCP frame")
        logger.info("Relative target Euler ZYX (deg): [20.0, 0.0, 0.0]")
        exec_prim(
            robot,
            {
                group: flexivrdk.PrimitiveArgs(
                    "MoveL",
                    {
                        # Zero position delta: stay in place. Apply a 20-degree rotation around X.
                        "target": flexivrdk.Coord(
                            [0.0, 0.0, 0.0],
                            [20.0, 0.0, 0.0],
                            ["TRAJ", "START"],
                        ),
                        "vel": 0.1,
                        "zoneRadius": "Z0",
                        "targetTolerLevel": 3,
                        "acc": 1.0,
                        "angVel": 120,
                        "enableFixRefJntPos": False,
                        "refJntPos": flexivrdk.JPos([0, -40, 0, 90, 0, 40, 0]),
                        "jerk": 50,
                        "configOptObj": [0, 0, 0.5],
                        "enableSixAxisJntCtrl": False,
                        "enableNullspaceTraj": False,
                    },
                )
                for group in single_arm_groups
            },
            {group: "reachedTarget" for group in single_arm_groups},
        )

        # ── 8) MoveJ to upright posture ───────────────────────────────────────
        # Upright convention: all arm joints at 0 degrees.
        logger.info("Step 8: MoveJ to upright posture")
        exec_prim(
            robot,
            {
                group: flexivrdk.PrimitiveArgs(
                    "MoveJ",
                    {
                        "target": flexivrdk.JPos([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                        "jntVelScale": 40,
                        "zoneRadius": "Z50",
                        "targetTolerLevel": 1,
                        "enableRelativeMove": False,
                        "jntAccMultiplier": 1,
                    },
                )
                for group in single_arm_groups
            },
            {group: "reachedTarget" for group in single_arm_groups},
        )

        # ── 9) Home ───────────────────────────────────────────────────────────
        logger.info("Step 9: Move to home pose")
        robot.Home()

        logger.info("Reference sequence completed successfully")
        robot.Stop()

    except Exception as e:
        logger.error(f"Unhandled exception: {e}")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
