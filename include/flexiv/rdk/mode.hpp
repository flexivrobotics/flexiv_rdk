/**
 * @file mode.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_MODE_HPP_
#define FLEXIV_RDK_MODE_HPP_

namespace flexiv {
namespace rdk {

/**
 * @enum Mode
 * @brief Robot control modes. The robot needs to be switched into the correct control mode before
 * the corresponding commands can be sent. Refer to Flexiv RDK Manual for more details.
 * @see Robot::SwitchMode().
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
     * @note Real-time (RT) mode.
     * @see Robot::StreamJointTorque().
     */
    RT_JOINT_TORQUE,

    /**
     * Run real-time joint impedance control to track continuous commands @ 1kHz.
     * @note Real-time (RT) mode.
     * @see Robot::StreamJointPosition().
     */
    RT_JOINT_IMPEDANCE,

    /**
     * Run non-real-time joint impedance control to track discrete commands (smoothened by internal
     * motion generator).
     * @note Non-real-time (NRT) mode.
     * @see Robot::SendJointPosition().
     */
    NRT_JOINT_IMPEDANCE,

    /**
     * Run real-time joint position control to track continuous commands @ 1kHz.
     * @note Real-time (RT) mode.
     * @see Robot::StreamJointPosition().
     */
    RT_JOINT_POSITION,

    /**
     * Run non-real-time joint position control to track discrete commands (smoothened by internal
     * motion generator).
     * @note Non-real-time (NRT) mode.
     * @see Robot::SendJointPosition().
     */
    NRT_JOINT_POSITION,

    /**
     * Execute pre-configured robot task plans.
     * @note Non-real-time (NRT) mode.
     * @see Robot::ExecutePlan().
     */
    NRT_PLAN_EXECUTION,

    /**
     * Execute robot primitives (unit skills).
     * @note Non-real-time (NRT) mode.
     * @see Robot::ExecutePrimitive().
     * @see [Flexiv Primitives](https://www.flexiv.com/primitives/) documentation.
     */
    NRT_PRIMITIVE_EXECUTION,

    /**
     * Run real-time Cartesian motion-force control to track continuous commands @ 1kHz.
     * @note Real-time (RT) mode.
     * @see Robot::StreamCartesianMotionForce().
     */
    RT_CARTESIAN_MOTION_FORCE,

    /**
     * Run non-real-time Cartesian motion-force control to track discrete commands, smoothened by
     * internal motion generator.
     * @note Non-real-time (NRT) mode.
     * @see Robot::SendCartesianMotionForce().
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

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_MODE_HPP_ */
