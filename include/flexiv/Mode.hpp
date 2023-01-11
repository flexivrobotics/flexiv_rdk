/**
 * @file Mode.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_MODE_HPP_
#define FLEXIVRDK_MODE_HPP_

namespace flexiv {

/**
 * @enum Mode
 * @brief Operation modes of the robot.
 */
enum Mode
{
    /** Mode not set */
    MODE_UNKNOWN = -1,

    /**
     * No operation to execute, the robot holds position and waits for new
     * command.
     */
    MODE_IDLE,

    /**
     * Execute continuous joint torque command @ 1kHz.
     * @note Real-time (RT) mode
     * @see flexiv::Robot::streamJointTorque()
     */
    MODE_JOINT_TORQUE,

    /**
     * Execute continuous joint position command @ 1kHz.
     * @note Real-time (RT) mode
     * @see flexiv::Robot::streamJointPosition()
     */
    MODE_JOINT_POSITION,

    /**
     * Execute discrete joint position command (smoothened by internal motion
     * generator).
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::sendJointPosition()
     */
    MODE_JOINT_POSITION_NRT,

    /**
     * Execute pre-configured motion plans.
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::executePlanByIndex()
     * @see flexiv::Robot::executePlanByName()
     */
    MODE_PLAN_EXECUTION,

    /**
     *  Execute robot primitives.
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::executePrimitive()
     * @see [Flexiv Primitives](https://www.flexiv.com/primitives/)
     * documentation
     */
    MODE_PRIMITIVE_EXECUTION,

    /**
     * Execute continuous TCP pose control command using Cartesian impedance
     * controller @ 1kHz.
     * @note Real-time (RT) mode
     * @see flexiv::Robot::streamTcpPose()
     */
    MODE_CARTESIAN_IMPEDANCE,

    /**
     * Execute discrete TCP pose control command (smoothened by internal motion
     * generator) using Cartesian impedance controller.
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::sendTcpPose()
     */
    MODE_CARTESIAN_IMPEDANCE_NRT,
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_MODE_HPP_ */
