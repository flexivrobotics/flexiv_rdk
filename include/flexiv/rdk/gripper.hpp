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
 * @brief Interface with the robot gripper. Because gripper is also a type of device, this API uses
 * the same underlying infrastructure as rdk::Device, but with functions tailored specifically for
 * gripper controls.
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
     * @brief [Blocking] Enable the specified gripper as a device, same as Device::Enable().
     * @param[in] name Name of the gripper device to enable, must be an existing one.
     * @throw std::logic_error if the specified gripper device does not exist or a gripper is
     * already enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot or failed
     * to sync gripper parameters.
     * @note This function blocks until the request is successfully delivered.
     * @note There can only be one enabled gripper at a time, call Disable() on the currently
     * enabled gripper before enabling another gripper.
     * @warning There's no enforced check on whether the enabled device is a gripper or not. Using
     * this API on a non-gripper device will likely lead to undefined behaviors.
     */
    void Enable(const std::string& name);

    /**
     * @brief [Blocking] Disable the currently enabled gripper, similar to Device::Disable().
     * @throw std::logic_error if no gripper device is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Disable();

    /**
     * @brief [Blocking] Manually trigger the initialization of the enabled gripper. This step is
     * not needed for grippers that automatically initialize upon power-on.
     * @throw std::logic_error if no gripper device is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     * @warning This function does not wait for the initialization sequence to finish, the user may
     * need to implement wait after calling this function before commanding the gripper.
     */
    void Init();

    /**
     * @brief [Blocking] Grasp with direct force control. Requires the mounted gripper to support
     * direct force control.
     * @param[in] force Target gripping force. Positive: closing force, negative: opening force [N].
     * @throw std::logic_error if no gripper device is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     * @warning Target inputs outside the valid range (see params()) will be saturated.
     */
    void Grasp(double force);

    /**
     * @brief [Blocking] Move the gripper fingers with position control.
     * @param[in] width Target opening width [m].
     * @param[in] velocity Closing/opening velocity, cannot be 0 [m/s].
     * @param[in] force_limit Maximum output force during movement [N]. If not specified, default
     * force limit of the mounted gripper will be used.
     * @throw std::logic_error if no gripper device is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     * @warning Target inputs outside the valid range (see params()) will be saturated.
     */
    void Move(double width, double velocity, double force_limit = 0);

    /**
     * @brief [Blocking] Stop the gripper and hold its current finger width.
     * @throw std::logic_error if no gripper device is enabled.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
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
