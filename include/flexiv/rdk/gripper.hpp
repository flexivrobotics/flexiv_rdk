/**
 * @file gripper.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_GRIPPER_HPP_
#define FLEXIV_RDK_GRIPPER_HPP_

#include "robot.hpp"
#include <memory>

namespace flexiv {
namespace rdk {

/**
 * @class Gripper
 * @brief Interface with the robot gripper.
 */
class Gripper
{
public:
    /**
     * @brief [Non-blocking] Create an instance and initialize gripper control interface.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    Gripper(const Robot& robot);
    virtual ~Gripper();

    /**
     * @brief [Blocking] Manually trigger the initialization of the connected gripper. This step is
     * not needed for grippers that automatically initialize upon power-on.
     * @note Applicable control modes: all modes except IDLE.
     * @note This function blocks until the request is successfully delivered.
     * @warning This function returns before the initialization is finished, thus additional wait is
     * needed before the gripper can take any commands.
     */
    void Init();

    /**
     * @brief [Non-blocking] Grasp with direct force control. Requires the mounted gripper to
     * support direct force control.
     * @param[in] force Target gripping force. Positive: closing force, negative: opening force [N].
     * @throw std::logic_error if robot is not in the correct control mode.
     * @note Applicable control modes: all modes except IDLE.
     * @warning Target inputs outside the valid range (specified in gripper's configuration file)
     * will be saturated.
     */
    void Grasp(double force);

    /**
     * @brief [Non-blocking] Move the gripper fingers with position control.
     * @param[in] width Target opening width [m].
     * @param[in] velocity Closing/opening velocity, cannot be 0 [m/s].
     * @param[in] force_limit Maximum output force during movement [N]. If not specified, default
     * force limit of the mounted gripper will be used.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @note Applicable control modes: all modes except IDLE.
     * @warning Target inputs outside the valid range (specified in gripper's configuration file)
     * will be saturated.
     */
    void Move(double width, double velocity, double force_limit = 0);

    /**
     * @brief [Blocking] Stop the gripper.
     * @note Applicable control modes: all modes.
     * @note This function blocks until the gripper control is transferred back to plan/primitive.
     */
    void Stop();

    /**
     * @brief [Non-blocking] Whether the gripper fingers are moving.
     * @return True: moving, false: stopped.
     */
    bool moving() const;

    /**
     * @brief [Non-blocking] Access the current gripper states.
     * @return GripperStates value copy.
     * @note Real-time (RT).
     */
    const GripperStates states() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_GRIPPER_HPP_ */
