/**
 * @file Device.hpp
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_DEVICE_HPP_
#define FLEXIVRDK_DEVICE_HPP_

#include "Robot.hpp"
#include <map>

namespace flexiv {

/**
 * @class Device
 * @brief Interface with the robot device(s).
 */
class Device
{
public:
    /**
     * @brief [Non-blocking] Create an instance and initialize device control interface.
     * @param[in] robot Reference to the instance of flexiv::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    Device(const Robot& robot);
    virtual ~Device();

    /**
     * @brief [Blocking] Request a list of existing devices and their status (enabled/disabled).
     * @return A map of {device_name, is_enabled}. For example,
     * {{"Mirka-AIROS-550CV", true}, {"LinearRail", false}}.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    std::map<std::string, bool> list() const;

    /**
     * @brief [Blocking] Enable the specified device.
     * @param[in] name Name of the device to enable, must be an existing device.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     * @warning Enabling a nonexistent device will trigger an error on the connected robot.
     */
    void enable(const std::string& name);

    /**
     * @brief [Blocking] Disable the specified device.
     * @param[in] name Name of the device to disable, must be an existing device.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     * @warning Disabling a nonexistent device will trigger an error on the connected robot.
     */
    void disable(const std::string& name);

    /**
     * @brief [Blocking] Send command(s) for the specified device.
     * @param[in] name Name of the device to send command(s) for, must be an existing device.
     * @param[in] cmds A map of {command_name, command_value}. Use number 1 and 0 to represent
     * booleans. For example, {{"setSpeed", 6000}, {"startMotor", 1}}.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     * @warning Commanding a disabled or nonexistent device will trigger an error on the robot.
     */
    void command(const std::string& name, const std::map<std::string, double>& cmds);

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_DEVICE_HPP_ */