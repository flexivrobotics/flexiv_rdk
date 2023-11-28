/**
 * @file Gripper.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_GRIPPER_HPP_
#define FLEXIVRDK_GRIPPER_HPP_

#include "Robot.hpp"
#include <memory>

namespace flexiv {

/**
 * @class Gripper
 * @brief Interface with grippers supported by Flexiv.
 */
class Gripper
{
public:
    /**
     * @brief [Non-blocking] Create a flexiv::Gripper instance and initialize
     * robot gripper control.
     * @param[in] robot Reference to the instance of flexiv::Robot.
     * @throw InitException if the initialization sequence failed.
     */
    Gripper(const Robot& robot);
    virtual ~Gripper();

    /**
     * @brief [Blocking] Grasp with direct force control. Requires the mounted
     * gripper to support direct force control.
     * @param[in] force Target gripping force. Positive: closing force,
     * negative: opening force [N].
     * @note Applicable operation modes: all modes except IDLE.
     * @throw ExecutionException if error occurred during execution.
     * @warning Target inputs outside the valid range (specified in gripper's
     * configuration file) will be saturated.
     * @warning This function blocks until the request is successfully delivered
     * to the robot.
     */
    void grasp(double force);

    /**
     * @brief [Blocking] Move the gripper fingers with position control.
     * @param[in] width Target opening width [m].
     * @param[in] velocity Closing/opening velocity, cannot be 0 [m/s].
     * @param[in] forceLimit Maximum output force during movement [N]. If not
     * specified, default force limit of the mounted gripper will be used.
     * @note Applicable operation modes: all modes except IDLE.
     * @throw ExecutionException if error occurred during execution.
     * @warning Target inputs outside the valid range (specified in gripper's
     * configuration file) will be saturated.
     * @warning This function blocks until the request is successfully delivered
     * to the robot.
     */
    void move(double width, double velocity, double forceLimit = 0);

    /**
     * @brief [Blocking] Stop the gripper.
     * @note Applicable operation modes: all modes.
     * @throw ExecutionException if error occurred during execution.
     * @warning This function blocks until the request is successfully delivered
     * to the robot and gripper control is transferred back to plan/primitive.
     */
    void stop(void);

    /**
     * @brief [Non-blocking] Get current gripper states.
     * @param[out] output Reference of output data object.
     */
    void getGripperStates(GripperStates& output);

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_GRIPPER_HPP_ */
