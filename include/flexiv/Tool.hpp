/**
 * @file Tool.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_TOOL_HPP_
#define FLEXIVRDK_TOOL_HPP_

#include "Robot.hpp"

namespace flexiv {

/**
 * @class Tool
 * @brief Interface to online update and interact with the robot tools. All changes made to the
 * robot tool system will take effect immediately without needing to reboot. However, the robot must
 * be put into IDLE mode when making these changes.
 */
class Tool
{
public:
    /**
     * @brief [Non-blocking] Create a flexiv::Tool instance and initialize tool update interface.
     * @param[in] robot Reference to the instance of flexiv::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    Tool(const Robot& robot);
    virtual ~Tool();

    /**
     * @brief [Blocking] Add a new tool with user-specified parameters to the tools pool.
     * @param[in] name Name of the new tool, must be unique.
     * @param[in] params Parameters of the new tool.
     * @note Applicable control modes: IDLE.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @warning This function blocks until the request is successfully executed.
     */
    void add(const std::string& name, const ToolParams& params);

    /**
     * @brief [Blocking] Switch to an existing tool. All following robot operations will default to
     * use this tool.
     * @param[in] name Name of the tool to switch to, must exist in the tools pool.
     * @note Applicable control modes: IDLE.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @warning This function blocks until the request is successfully executed.
     */
    void switchTo(const std::string& name);

    /**
     * @brief [Blocking] Update the parameters of an existing tool.
     * @param[in] name Name of the tool to update, must exist in the tools pool.
     * @param[in] params New parameters for the specified tool.
     * @note Applicable control modes: IDLE.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @warning This function blocks until the request is successfully executed.
     */
    void update(const std::string& name, const ToolParams& params);

    /**
     * @brief [Blocking] Remove an existing tool.
     * @param[in] name Name of the tool to remove, must exist in the tools pool.
     * @note Applicable control modes: IDLE.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @warning This function blocks until the request is successfully executed.
     */
    void remove(const std::string& name);

    /**
     * @brief [Blocking] Get name of the tool that the robot is currently using.
     * @return Name of the current tool. Return "Flange" if there's no active tool.
     * @note Applicable control modes: All.
     * @throw std::runtime_error if failed to get a reply from the robot.
     * @warning This function blocks until the reply from the robot is received.
     */
    std::string getCurrentToolName(void) const;

    /**
     * @brief [Blocking] Get parameters of the tool that the robot is currently using.
     * @return Parameters result.
     * @note Applicable control modes: All.
     * @throw std::runtime_error if failed to get a reply from the robot.
     * @warning This function blocks until the reply from the robot is received.
     */
    ToolParams getCurrentToolParams(void) const;

    /**
     * @brief [Blocking] Get parameters of an existing tool.
     * @param[in] name Name of the tool to get parameters for, must exist in the tools pool.
     * @return Parameters result.
     * @note Applicable control modes: All.
     * @throw std::runtime_error if failed to get a reply from the robot.
     * @warning This function blocks until the reply from the robot is received.
     */
    ToolParams getToolParams(const std::string& name) const;

    /**
     * @brief [Blocking] Get a name list of all tools currently in the tools pool.
     * @return Tool names in the format of a string list.
     * @throw std::runtime_error if failed to get a reply from the robot.
     * @warning This function blocks until the reply from the robot is received.
     */
    std::vector<std::string> getToolList(void) const;

    /**
     * @brief [Blocking] Check if a tool exists in the tools pool.
     * @param[in] name Name of the tool to check.
     * @return True if the specified tool exists.
     * @throw std::runtime_error if failed to get a reply from the robot.
     * @warning This function blocks until the reply from the robot is received.
     */
    bool isExist(const std::string& name) const;

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_TOOL_HPP_ */
