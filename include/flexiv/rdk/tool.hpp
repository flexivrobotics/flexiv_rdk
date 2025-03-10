/**
 * @file tool.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_TOOL_HPP_
#define FLEXIV_RDK_TOOL_HPP_

#include "robot.hpp"

namespace flexiv {
namespace rdk {

/**
 * @struct ToolParams
 * @brief Data structure containing robot tool parameters.
 * @see Tool::params().
 */
struct ToolParams
{
    /** Total mass. Unit: \f$ [kg] \f$ */
    double mass = 0.0;

    /** Center of mass in robot flange frame: \f$ [x, y, z] \f$. Unit: \f$ [m] \f$ */
    std::array<double, 3> CoM = {};

    /** Inertia at center of mass: \f$ [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] \f$. Unit: \f$ [kg m^2] \f$ */
    std::array<double, 6> inertia = {};

    /** Position and orientation of the tool center point (TCP) in flange frame. Consists of \f$
     * \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x,
     * y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$ */
    std::array<double, kPoseSize> tcp_location = {};
};

/**
 * @class Tool
 * @brief Interface to online update and interact with the robot tools. All updates will take effect
 * immediately without a power cycle. However, the robot must be in IDLE mode when applying changes.
 */
class Tool
{
public:
    /**
     * @brief [Non-blocking] Create an instance and initialize the interface.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    Tool(const Robot& robot);
    virtual ~Tool();

    /**
     * @brief [Blocking] Get a list of all configured tools.
     * @return Tool names as a string list.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    const std::vector<std::string> list() const;

    /**
     * @brief [Blocking] Get name of the tool that the robot is currently using.
     * @return Name of the current tool. Return "Flange" if there's no active tool.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    const std::string name() const;

    /**
     * @brief [Blocking] Whether the specified tool already exists.
     * @param[in] name Name of the tool to check.
     * @return True if the specified tool exists.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    bool exist(const std::string& name) const;

    /**
     * @brief [Blocking] Get parameters of the tool that the robot is currently using.
     * @return ToolParams value copy.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    const ToolParams params() const;

    /**
     * @brief [Blocking] Get parameters of the specified tool.
     * @param[in] name Name of the tool to get parameters for, must be an existing one.
     * @return ToolParams value copy.
     * @throw std::invalid_argument if the specified tool does not exist.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    const ToolParams params(const std::string& name) const;

    /**
     * @brief [Blocking] Add a new tool with user-specified parameters.
     * @param[in] name Name of the new tool, must be unique.
     * @param[in] params Parameters of the new tool.
     * @throw std::invalid_argument if the specified tool already exists.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     */
    void Add(const std::string& name, const ToolParams& params);

    /**
     * @brief [Blocking] Switch to an existing tool. All following robot operations will default to
     * use this tool.
     * @param[in] name Name of the tool to switch to, must be an existing one.
     * @throw std::invalid_argument if the specified tool does not exist.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     */
    void Switch(const std::string& name);

    /**
     * @brief [Blocking] Update the parameters of an existing tool.
     * @param[in] name Name of the tool to update, must be an existing one.
     * @param[in] params New parameters for the specified tool.
     * @throw std::invalid_argument if the specified tool does not exist.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     */
    void Update(const std::string& name, const ToolParams& params);

    /**
     * @brief [Blocking] Remove an existing tool.
     * @param[in] name Name of the tool to remove, must be an existing one but cannot be "Flange".
     * @throw std::invalid_argument if the specified tool does not exist.
     * @throw std::logic_error if robot is not in the correct control mode or trying to remove
     * "Flange".
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     */
    void Remove(const std::string& name);

    /**
     * @brief [Blocking] Calibrate the payload parameters (mass, CoM, and inertia) of a tool.
     * @param[in] tool_mounted Whether the tool to be calibrated is mounted on the robot flange when
     * triggering this calibration process. See details below.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if fault occurred during the calibration or failed to get the
     * calibration result.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the calibration is finished.
     * @warning [tcp_location] in the returned struct will be zeros and should be ignored.
     * @par How to properly calibrate the payload parameters of a tool?
     * 1. Call Switch("Flange") to disable any active tool from the robot software.
     * 2. Physically mount the tool to be calibrated to robot flange.
     * 3. Call this function with [tool_mounted] set to TRUE, then wait for completion. If the robot
     * has a force-torque (FT) sensor (e.g. Rizon4s, Rizon10s), then the returned result will be
     * accurate enough and the optional steps can be skipped. If the robot does not have an FT
     * sensor (e.g. Rizon4, Rizon10), then the optional steps are recommended to improve accuracy.
     * 4. (Optional) Physically unmount the tool from robot flange.
     * 5. (Optional) Call this function again but with [tool_mounted] set to FALSE, then wait for
     * completion.
     * 6. Review the returned result and call Add() or Update() to apply the calibrated payload
     * parameters to a new or existing tool. Note that [tcp_location] in the returned struct is
     * invalid and cannot be used directly.
     */
    const ToolParams CalibratePayloadParams(bool tool_mounted);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_TOOL_HPP_ */
