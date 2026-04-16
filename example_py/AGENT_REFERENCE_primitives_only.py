#!/usr/bin/env python

"""AGENT_REFERENCE_primitives_only.py

Comprehensive primitive-only reference script for code-generation agents.

Primitives demonstrated (in order):
  1. Home          - send robot to home with custom optional target joint positions.
  2. ZeroFTSensor  - conditionally zero force/torque sensors when FT sensor exists.
  3. MoveJ         - joint-space move with waypoints and joint-velocity scaling.
  4. MoveL         - Cartesian linear move in WORLD frame with velocity, blending (zoneRadius).
  5. MoveL         - multi-waypoint sweep in WORLD frame.
  6. Home          - return to home before TCP-frame relative move.
  7. MoveL         - relative move in TCP frame (TRAJ::START), orientation-only rotation.
  8. MoveJ         - return to upright posture (all joints 0).
  9. Home          - return to home.

Key patterns shown:
  - Fault clear, enable, wait operational, mode switch (essential setup).
  - exec_prim() wrapper parameterised with the correct transition key per primitive.
  - Transition keys must be looked up per primitive from Flexiv primitive documentation:
      https://primitive.flexiv.com/primitives/en/3.11/rizon4/index.html
  - Reading tcp_pose to extract live orientation (quaternion [w,x,y,z] at indices [3..6]).
  - Converting quaternion to Euler ZYX degrees using utility.quat2eulerZYX for MoveL orientation.
  - ZeroFTSensor is gated by robot.info().has_FT_sensor.
  - MoveL with optional velocity, zoneRadius blending, and waypoints.
  - MoveJ with optional waypoints list and jntVelScale.
  - All JPos calls use 7-element arm-joint list; second (external-axis) argument is optional.
  - All Coord calls use positional args: (position, orientation_eulerZYX_deg, ref_frame_list).
"""

import argparse
import time
import spdlog  # pip install spdlog
import flexivrdk  # pip install flexivrdk
from utility import quat2eulerZYX


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


def wait_primitive_transition(robot, state_key, dt=0.2, timeout_s=300.0):
    """Block until primitive_states()[state_key] is True, or raise TimeoutError.

    Use the primitive's default transition key unless a custom transition condition
    is intended. Check primitive docs to confirm available primitive state keys.
    """
    t0 = time.time()
    while not robot.primitive_states()[state_key]:
        if time.time() - t0 > timeout_s:
            raise TimeoutError(
                f"Timed out waiting for primitive transition state '{state_key}'"
            )
        time.sleep(dt)


def exec_prim(robot, name, params, transition_key):
    """Execute one primitive and block until transition_key is True.

    Args:
        robot:          flexivrdk.Robot instance.
        name:           Primitive name string, e.g. "Home", "MoveJ", "MoveL".
        params:         Dict of primitive parameters.
        transition_key: Primitive state key selected by caller for transition.
            This can be the primitive's default key or another primitive state key.
            Look up primitive states from:
            https://primitive.flexiv.com/primitives/en/3.11/rizon4/index.html

        Note:
        This helper waits on primitive_states() only. If transition depends on
        robot states, implement a custom wait loop at call site.
    """
    robot.ExecutePrimitive(name, params)
    wait_primitive_transition(robot, transition_key)


def prepare_robot(robot_sn, logger):
    """Connect, clear faults, enable robot, and switch to primitive execution mode."""
    robot = flexivrdk.Robot(robot_sn)

    if robot.fault():
        logger.warn("Fault detected on connected robot, trying to clear")
        if not robot.ClearFault():
            raise RuntimeError("Failed to clear robot fault")
        logger.info("Fault cleared")

    logger.info("Enabling robot")
    robot.Enable()
    wait_until_operational(robot)
    logger.info("Robot is operational")

    robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
    return robot


def current_euler_zyx_deg(robot):
    """Return live TCP orientation as Euler ZYX [x,y,z] degrees.

    tcp_pose layout: [x, y, z, q_w, q_x, q_y, q_z] (SI units, quaternion).
    quat2eulerZYX() expects [w, x, y, z] order.
    The returned Euler values are normalized to the robot's [-180, 180] convention.
    """
    pose = robot.states().tcp_pose
    return quat2eulerZYX([pose[3], pose[4], pose[5], pose[6]], degree=True)


# ──────────────────────────────────────────────────────────────────────────────
# Main sequence
# ──────────────────────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "robot_sn",
        help="Serial number of the robot to connect. Remove spaces, e.g. Rizon4s-123456",
    )
    args = parser.parse_args()

    logger = spdlog.ConsoleLogger("AgentReference")

    try:
        robot = prepare_robot(args.robot_sn, logger)

        # ── 1) Home ───────────────────────────────────────────────────────────
        # All Home parameters are optional. Without a target, the robot goes to
        # the platform-defined home posture. An explicit target joint position
        # can be passed to customise the joint-space goal.
        logger.info("Step 1: Home (default target)")
        exec_prim(robot, "Home", {}, transition_key="reachedTarget")

        # ── 2) ZeroFTSensor (if available) ────────────────────────────────────
        # Use this primitive's documented default transition condition.
        # The robot must not be in contact with anything during zeroing.
        if robot.info().has_FT_sensor:
            logger.info("Step 2: ZeroFTSensor")
            logger.warn("Zeroing F/T sensors - ensure nothing contacts the robot")
            exec_prim(robot, "ZeroFTSensor", {}, transition_key="terminated")
            logger.info("F/T sensor zeroing complete")
        else:
            logger.info("Step 2: Skip ZeroFTSensor (no FT sensor installed)")

        # ── 3) MoveJ with waypoints and velocity scaling ──────────────────────
        # target / waypoints: flexivrdk.JPos(q_m) where q_m is 7-element degrees list.
        # jntVelScale: percentage of max joint speed [1-100].
        logger.info("Step 3: MoveJ with waypoints")
        exec_prim(
            robot,
            "MoveJ",
            {
                "target": flexivrdk.JPos([30, -45, 0, 90, 0, 40, 30]),
                "waypoints": [
                    flexivrdk.JPos([10, -30, 10, 30, 10, 15, 10]),
                    flexivrdk.JPos([20, -60, -10, 60, -10, 30, 20]),
                ],
                "jntVelScale": 50,  # 50% of max joint speed
            },
            transition_key="reachedTarget",
        )

        # ── 4) MoveL in WORLD frame with velocity and blending ────────────────
        # target / waypoints: flexivrdk.Coord(position_m, orientation_eulerZYX_deg, ref_frame).
        # ref_frame is a 2-element list: [frame_type, frame_origin].
        # vel: TCP linear speed in m/s.
        # zoneRadius: blending zone size string, e.g. "Z50". Use "Z0" for exact stops.
        logger.info("Step 4: MoveL in WORLD frame with speed and blending")
        exec_prim(
            robot,
            "MoveL",
            {
                "target": flexivrdk.Coord(
                    [0.65, -0.3, 0.3], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]
                ),
                "waypoints": [
                    flexivrdk.Coord(
                        [0.45, 0.1, 0.3], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]
                    ),
                    flexivrdk.Coord(
                        [0.45, -0.3, 0.3], [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]
                    ),
                ],
                "vel": 0.3,  # m/s TCP linear speed
                "zoneRadius": "Z50",  # blended corners; use "Z0" for hard stops
            },
            transition_key="reachedTarget",
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
                "MoveL",
                {
                    "target": flexivrdk.Coord(
                        target_pos, [180, 0, 180], ["WORLD", "WORLD_ORIGIN"]
                    ),
                    "vel": 0.2,
                    "zoneRadius": "Z0",  # exact stop at each corner
                },
                transition_key="reachedTarget",
            )

        # ── 6) Home before TCP-frame relative move ────────────────────────────
        logger.info("Step 6: Home")
        exec_prim(robot, "Home", {}, transition_key="reachedTarget")

        # ── 7) MoveL in TCP frame (TRAJ::START) - relative orientation change ─
        # TRAJ::START means both translation and orientation targets are relative
        # to the current TCP pose at primitive start.
        # Setting position to [0, 0, 0] keeps the TCP at the same location.
        # Orientation [20, 0, 0] means a +20 deg relative rotation around X.
        logger.info("Step 7: MoveL relative orientation-only rotation in TCP frame")
        logger.info("Relative target Euler ZYX (deg): [20.0, 0.0, 0.0]")
        exec_prim(
            robot,
            "MoveL",
            {
                # Zero position delta: stay in place. Apply a 20-degree rotation around X.
                "target": flexivrdk.Coord(
                    [0.0, 0.0, 0.0],
                    [20.0, 0.0, 0.0],
                    ["TRAJ", "START"],
                ),
                "vel": 0.1,
            },
            transition_key="reachedTarget",
        )

        # ── 8) MoveJ to upright posture ───────────────────────────────────────
        # Upright convention: all arm joints at 0 degrees.
        logger.info("Step 8: MoveJ to upright posture")
        exec_prim(
            robot,
            "MoveJ",
            {
                "target": flexivrdk.JPos([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                "jntVelScale": 40,
            },
            transition_key="reachedTarget",
        )

        # ── 9) Home ───────────────────────────────────────────────────────────
        logger.info("Step 9: Home")
        exec_prim(robot, "Home", {}, transition_key="reachedTarget")

        logger.info("Reference sequence completed successfully")
        robot.Stop()

    except Exception as e:
        logger.error(f"Unhandled exception: {e}")
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
