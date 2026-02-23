/**
 * @file device.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_DEVICE_HPP_
#define FLEXIV_RDK_DEVICE_HPP_

#include "robot.hpp"
#include <map>

namespace flexiv {
namespace rdk {

using DeviceParamDataTypes
    = std::variant<int, double, std::string, std::vector<double>, std::vector<std::string>>;

/**
 * @class Device
 * @brief Interface to control the peripheral device(s) connected to the robot.
 */
class Device
{
public:
    /**
     * @brief [Non-blocking] Instantiate the device control interface.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    Device(const Robot& robot);
    virtual ~Device();

    /**
     * @brief [Non-blocking] A list of all existing devices.
     * @return Device names as a string list.
     */
    std::vector<std::string> list() const;

    /**
     * @brief [Non-blocking] Whether the specified device already exists.
     * @param[in] name Name of the device to check.
     * @return True if the specified device exists.
     */
    bool exist(const std::string& name) const;

    /**
     * @brief [Blocking] Whether the specified device is enabled.
     * @param[in] name Name of the device to check.
     * @return True if the specified device is enabled.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    bool enabled(const std::string& name) const;

    /**
     * @brief [Blocking] Whether the specified device is connected.
     * @param[in] name Name of the device to check.
     * @return True if the specified device is connected.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    bool connected(const std::string& name) const;

    /**
     * @brief [Blocking] Configuration parameters of the specified device.
     * @param[in] name Name of the device to get parameters for.
     * @return A map of {param_name, param_value}. Booleans are represented by int 1 and 0. For
     * example, {{"maxVel", 0.5}, {"absolutePosition", {0.7, -0.4, 0.05}}, {"conveyorName",
     * "conveyor0"}}.
     * @throw std::invalid_argument if the specified device does not exist.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    std::map<std::string, DeviceParamDataTypes> params(const std::string& name) const;

    /**
     * @brief [Blocking] Enable the specified device.
     * @param[in] name Name of the device to enable.
     * @throw std::invalid_argument if the specified device does not exist.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Enable(const std::string& name);

    /**
     * @brief [Blocking] Disable the specified device.
     * @param[in] name Name of the device to disable.
     * @throw std::invalid_argument if the specified device does not exist.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Disable(const std::string& name);

    /**
     * @brief [Blocking] Send command(s) to the specified device.
     * @param[in] name Name of the device to send command(s) to.
     * @param[in] commands A map of {command_name, command_value}. For example, {{"setSpeed", 6000},
     * {"openLaser", true}}. All commands in the map will be sent to the device simultaneously. Make
     * sure the command name(s) are valid and can be accepted by the specified device.
     * @throw std::invalid_argument if the specified device does not exist.
     * @throw std::logic_error if the specified device is not enabled yet.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Command(const std::string& name,
        const std::map<std::string, std::variant<bool, int, double>>& commands);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_DEVICE_HPP_ */
