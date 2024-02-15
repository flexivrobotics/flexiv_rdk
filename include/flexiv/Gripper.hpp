/**
 * @file Gripper.hpp
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_GRIPPER_HPP_
#define FLEXIVRDK_GRIPPER_HPP_

#include "Robot.hpp"
#include <memory>

namespace flexiv {

/**
 * @class Gripper
 * @brief Interface with the robot gripper.
 */
class Gripper
{
public:
    /**
     * @brief [Non-blocking] Create an instance and initialize gripper control interface.
     * @param[in] robot Reference to the instance of flexiv::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    Gripper(const Robot& robot);
    virtual ~Gripper();

    /**
     * @brief [Non-blocking] Grasp with direct force control. Requires the mounted gripper to
     * support direct force control.
     * @param[in] force Target gripping force. Positive: closing force, negative: opening force [N].
     * @note Applicable control modes: all modes except IDLE.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @warning Target inputs outside the valid range (specified in gripper's configuration file)
     * will be saturated.
     */
    void grasp(double force);

    /**
     * @brief [Non-blocking] Move the gripper fingers with position control.
     * @param[in] width Target opening width [m].
     * @param[in] velocity Closing/opening velocity, cannot be 0 [m/s].
     * @param[in] forceLimit Maximum output force during movement [N]. If not specified, default
     * force limit of the mounted gripper will be used.
     * @note Applicable control modes: all modes except IDLE.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @warning Target inputs outside the valid range (specified in gripper's configuration file)
     * will be saturated.
     */
    void move(double width, double velocity, double forceLimit = 0);

    /**
     * @brief [Blocking] Stop the gripper.
     * @note Applicable control modes: all modes.
     * @note This function blocks until the gripper control is transferred back to plan/primitive.
     */
    void stop();

    /**
     * @brief [Non-blocking] Access the current gripper states.
     * @return GripperStates data struct.
     * @note Real-time (RT).
     */
    const GripperStates& states() const;

    /**
     * @brief [Non-blocking] Get current gripper states.
     * @param[out] output Reference to an existing GripperStates instance.
     * @note Call this function periodically to keep the output data object up to date.
     */
    [[deprecated("Use states() instead")]] void getGripperStates(GripperStates& output) const;

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_GRIPPER_HPP_ */
