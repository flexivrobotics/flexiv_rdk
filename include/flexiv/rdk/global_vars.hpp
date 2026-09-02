/**
 * @file global_vars.hpp
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_GLOBAL_VARS_HPP_
#define FLEXIV_RDK_GLOBAL_VARS_HPP_

#include "robot.hpp"

namespace flexiv::rdk {

/**
 * @class GlobalVars
 * @brief Interface to manage global variables of the robot. All updates take effect immediately
 * without a power cycle, and adding or removing a variable is also written to the robot's global
 * variable file, so it survives a power cycle and is visible in Flexiv Elements. However, the robot
 * must be in IDLE mode when adding or removing, because that changes the set of variables a running
 * plan may be referring to. Updating a value is allowed in any control mode.
 * @note Thread safety: all functions of this class are thread-safe and can be called concurrently
 * from multiple threads.
 */
class RDK_API GlobalVars
{
public:
    /**
     * @brief [Non-blocking] Instantiate the global variable interface.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    GlobalVars(const Robot& robot);
    virtual ~GlobalVars();

    /**
     * @brief [Blocking] A list of all existing global variables.
     * @return Global variable names as a string list.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    std::vector<std::string> list() const;

    /**
     * @brief [Blocking] Whether the specified global variable already exists.
     * @param[in] name Name of the global variable to check.
     * @return True if the specified global variable exists.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    bool exist(const std::string& name) const;

    /**
     * @brief [Blocking] Current value of an existing global variable.
     * @param[in] name Name of the global variable to get value for, must be an existing one.
     * @return Value of the global variable. Booleans are represented by int 1 and 0.
     * @throw std::invalid_argument if the specified global variable does not exist.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     * @see values().
     */
    FlexivDataTypes value(const std::string& name) const;

    /**
     * @brief [Blocking] All existing global variables and their current values.
     * @return A map of {global_var_name, global_var_value(s)}. Booleans are represented by int 1
     * and 0. For example, {{"camera_offset", {0.1, -0.2, 0.3}}, {"start_plan", 1}}.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     * @note The robot reports the data type of each variable, so the customized data types of
     * Flexiv Elements are recovered exactly. Data types that have no counterpart in
     * rdk::FlexivDataTypes, such as "POSE" and "VEC_3d", are reported as std::vector<double> or
     * std::vector<int>, and any remaining type as std::string.
     * @see value().
     */
    std::map<std::string, FlexivDataTypes> values() const;

    /**
     * @brief [Blocking] Add a new global variable with an initial value.
     * @param[in] name Name of the new global variable, must be unique.
     * @param[in] initial_value Initial value of the new global variable. Its data type also decides
     * the data type of the created variable, which cannot be changed afterwards. Use int 1 and 0 to
     * create a variable holding a boolean.
     * @throw std::invalid_argument if the specified global variable already exists.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     * @note The new variable is also saved to the robot's global variable file, so it survives a
     * power cycle and appears in Flexiv Elements.
     * @warning A string value must not contain whitespace: the robot stores it via a
     * whitespace-delimited text channel, so anything after the first space is dropped. This
     * applies to std::string values and to each element of a std::vector<std::string> value.
     */
    void Add(const std::string& name, const FlexivDataTypes& initial_value);

    /**
     * @brief [Blocking] Update the value of an existing global variable.
     * @param[in] name Name of the global variable to update, must be an existing one.
     * @param[in] value New value for the specified global variable. Use int 1 and 0 to represent
     * booleans.
     * @throw std::invalid_argument if the specified global variable does not exist.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: all.
     * @note This function blocks until the request is successfully delivered.
     * @note The data type held by the robot is authoritative: this updates the value only, it
     * cannot change a variable's data type.
     * @warning A string value must not contain whitespace: the robot stores it via a
     * whitespace-delimited text channel, so anything after the first space is dropped. This
     * applies to std::string values and to each element of a std::vector<std::string> value.
     */
    void Update(const std::string& name, const FlexivDataTypes& value);

    /**
     * @overload
     * @brief [Blocking] Update the values of several existing global variables in one request.
     * @param[in] global_vars A map of {global_var_name, global_var_value(s)}. Use int 1 and 0 to
     * represent booleans. For example, {{"camera_offset", {0.1, -0.2, 0.3}}, {"start_plan", 1}}.
     * @throw std::length_error if [global_vars] is empty.
     * @throw std::invalid_argument if any of the specified global variables does not exist.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: all.
     * @note This function blocks until the request is successfully delivered.
     * @note Either all variables are updated or none is: the robot validates every entry before
     * applying any of them.
     * @warning A string value must not contain whitespace: the robot stores it via a
     * whitespace-delimited text channel, so anything after the first space is dropped. This
     * applies to std::string values and to each element of a std::vector<std::string> value.
     */
    void Update(const std::map<std::string, FlexivDataTypes>& global_vars);

    /**
     * @brief [Blocking] Remove an existing global variable.
     * @param[in] name Name of the global variable to remove, must be an existing one.
     * @throw std::invalid_argument if the specified global variable does not exist.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     * @warning The removal is also saved to the robot's global variable file, so it is permanent.
     * A plan that still refers to the removed variable will fail to execute.
     */
    void Remove(const std::string& name);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace flexiv::rdk */

#endif /* FLEXIV_RDK_GLOBAL_VARS_HPP_ */
