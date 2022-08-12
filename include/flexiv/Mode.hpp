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
    MODE_UNKNOWN = -1,

    MODE_IDLE, ///< No operation to execute, the robot holds position and waits
               ///< for new command.

    /**
     * @note Real-time (RT) mode
     * @see flexiv::Robot::streamJointTorque()
     */
    MODE_JOINT_TORQUE, ///< Execute continuous joint torque command @ 1kHz.

    /**
     * @note Real-time (RT) mode
     * @see flexiv::Robot::streamJointPosition()
     */
    MODE_JOINT_POSITION, ///< Execute continuous joint position command @ 1kHz.

    /**
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::sendJointPosition()
     */
    MODE_JOINT_POSITION_NRT, ///< Execute discrete joint position command
                             ///< (smoothened by internal motion generator).

    /**
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::executePlanByIndex()
     * @see flexiv::Robot::executePlanByName()
     */
    MODE_PLAN_EXECUTION, ///< Execute pre-configured motion plans.

    /**
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::executePrimitive()
     */
    MODE_PRIMITIVE_EXECUTION, ///< Execute robot primitives.

    /**
     * @note Real-time (RT) mode
     * @see flexiv::Robot::streamTcpPose()
     */
    MODE_CARTESIAN_IMPEDANCE, ///< Execute continuous TCP pose command with
                              ///< Cartesian impedance controller @ 1kHz.

    /**
     * @note Non-real-time (NRT) mode
     * @see flexiv::Robot::sendTcpPose()
     */
    MODE_CARTESIAN_IMPEDANCE_NRT, ///< Execute discrete TCP pose command
                                  ///< (smoothened by internal motion generator)
                                  ///< with Cartesian impedance controller.
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_MODE_HPP_ */
