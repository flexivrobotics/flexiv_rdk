/**
 * @file mode.h
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_MODE_H_
#define FLEXIVRDK_MODE_H_

namespace flexiv {

/**
 * @enum Mode
 * @brief Robot control modes. Must put the robot into the correct control mode before sending any
 * corresponding commands.
 */
enum Mode
{
    /** Mode not set */
    UNKNOWN = 0,

    /**
     * No operation to execute, the robot holds position and waits for new command.
     */
    IDLE,

    /**
     * Run real-time joint torque control to track continuous commands @1kHz.
     * @note Real-time (RT) mode
     * @see flexiv::Robot::StreamJointTorque()
     */
    RT_JOINT_TORQUE,

    /**
     * Run real-time joint impedance control to track continuous commands @ 1kHz.
     * @note Real-time (RT) mode
     * @see flexiv::Robot::StreamJointPosition()
     */
    RT_JOINT_IMPEDANCE,

    /**
     * Run non-real-time joint impedance control to track discrete commands (smoothened by internal
     * motion generator).
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::SendJointPosition()
     */
    NRT_JOINT_IMPEDANCE,

    /**
     * Run real-time joint position control to track continuous commands @ 1kHz.
     * @note Real-time (RT) mode
     * @see flexiv::Robot::StreamJointPosition()
     */
    RT_JOINT_POSITION,

    /**
     * Run non-real-time joint position control to track discrete commands (smoothened by internal
     * motion generator).
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::SendJointPosition()
     */
    NRT_JOINT_POSITION,

    /**
     * Execute pre-configured robot task plans.
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::ExecutePlan()
     */
    NRT_PLAN_EXECUTION,

    /**
     * Execute robot primitives (unit skills).
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::ExecutePrimitive()
     * @see [Flexiv Primitives](https://www.flexiv.com/primitives/) documentation
     */
    NRT_PRIMITIVE_EXECUTION,

    /**
     * Run real-time Cartesian motion-force control to track continuous commands @ 1kHz.
     * @note Real-time (RT) mode
     * @see flexiv::Robot::StreamCartesianMotionForce()
     */
    RT_CARTESIAN_MOTION_FORCE,

    /**
     * Run non-real-time Cartesian motion-force control to track discrete commands, smoothened by
     * internal motion generator.
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::SendCartesianMotionForce()
     */
    NRT_CARTESIAN_MOTION_FORCE,

    /** Total number of control modes */
    MODES_CNT,
};

/** String names of the above control modes */
static const std::array<std::string, MODES_CNT> kModeNames
    = {"UNKNOWN", "IDLE", "RT_JOINT_TORQUE", "RT_JOINT_IMPEDANCE", "NRT_JOINT_IMPEDANCE",
        "RT_JOINT_POSITION", "NRT_JOINT_POSITION", "NRT_PLAN_EXECUTION", "NRT_PRIMITIVE_EXECUTION",
        "RT_CARTESIAN_MOTION_FORCE", "NRT_CARTESIAN_MOTION_FORCE"};

} /* namespace flexiv */

#endif /* FLEXIVRDK_MODE_H_ */
