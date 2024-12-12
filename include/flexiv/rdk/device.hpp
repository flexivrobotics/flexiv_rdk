/**
 * @file device.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_DEVICE_HPP_
#define FLEXIV_RDK_DEVICE_HPP_

#include "robot.hpp"
#include <map>

namespace flexiv {
namespace rdk {

/**
 * @class Device
 * @brief Interface with the robot device(s).
 */
class Device
{
public:
    /**
     * @brief [Non-blocking] Create an instance and initialize device control interface.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    Device(const Robot& robot);
    virtual ~Device();

    /**
     * @brief [Blocking] Get a list of existing devices and their status (enabled/disabled).
     * @return A map of {device_name, is_enabled}. For example,
     * {{"Mirka-AIROS-550CV", true}, {"LinearRail", false}}.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    const std::map<std::string, bool> list() const;

    /**
     * @brief [Blocking] Whether the specified device already exists.
     * @param[in] name Name of the device to check.
     * @return True if the specified device exists.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    bool exist(const std::string& name) const;

    /**
     * @brief [Blocking] Get configuration parameters of the specified device.
     * @param[in] name Name of the device to get parameters for, must be an existing one.
     * @return A map of {param_name, param_value}. Booleans are represented by int 1
     * and 0. For example,
     * {{"maxWidth", 0.5}, {"absolutePosition", {0.7, -0.4, 0.05}}, {"conveyorName", "conveyor0"}}.
     * @throw std::logic_error if the specified device does not exist.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    const std::map<std::string, std::variant<int, double, std::string, std::vector<double>>> params(
        const std::string& name) const;

    /**
     * @brief [Blocking] Enable the specified device.
     * @param[in] name Name of the device to enable, must be an existing device.
     * @throw std::logic_error if the specified device does not exist.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Enable(const std::string& name);

    /**
     * @brief [Blocking] Disable the specified device.
     * @param[in] name Name of the device to disable, must be an existing device.
     * @throw std::logic_error if the specified device does not exist.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Disable(const std::string& name);

    /**
     * @brief [Blocking] Send command(s) for the specified device.
     * @param[in] name Name of the device to send command(s) for, must be an existing device.
     * @param[in] cmds A map of {command_name, command_value}. Use int 1 and 0 to represent
     * booleans. For example, {{"setSpeed", 6000}, {"startMotor", 1}}.
     * @throw std::logic_error if the specified device does not exist or not enabled yet.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Command(
        const std::string& name, const std::map<std::string, std::variant<int, double>>& cmds);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_DEVICE_HPP_ */
