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

    /**
     * @note This is the default mode upon initialization
     */
    MODE_IDLE, ///< The robot holds position and waits for commands

    /**
     * @note Real-time control mode running at 1kHz
     */
    MODE_JOINT_POSITION, ///< Joint position control

    /**
     * @note Real-time control mode running at 1kHz
     */
    MODE_JOINT_TORQUE, ///< Joint torque control

    /**
     * @note Non-realtime control mode. The user selects a plan and executes it.
     * Upon completion, the user can send another plan
     */
    MODE_PLAN_EXECUTION, ///< Execute the plan specified by user

    /**
     * @note Non-realtime control mode. The user sends a primitive command with
     * the name and the associated parameters and executes it. Upon completion,
     * the user can send another primitive command
     */
    MODE_PRIMITIVE_EXECUTION, ///< Execute the primitive specified by user

    /**
     * @note Real-time control mode running at 1kHz
     */
    MODE_CARTESIAN_IMPEDANCE, ///< Cartesian impedance control
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_MODE_HPP_ */
