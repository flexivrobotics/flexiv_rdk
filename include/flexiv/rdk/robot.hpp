/**
 * @file robot.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_ROBOT_HPP_
#define FLEXIV_RDK_ROBOT_HPP_

#include "data.hpp"
#include "mode.hpp"
#include <vector>
#include <memory>
#include <exception>
#include <map>

namespace flexiv {
namespace rdk {

/**
 * @class Robot
 * @brief Main interface with the robot, containing several function categories and background
 * services.
 */
class Robot
{
public:
    /**
     * @brief [Blocking] Create an instance as the main robot control interface. RDK services will
     * initialize and connection with the robot will be established.
     * @param[in] robot_sn Serial number of the robot to connect. The accepted formats are:
     * "Rizon 4s-123456" and "Rizon4s-123456".
     * @param[in] network_interface_whitelist Limit the network interface(s) that can be used to try
     * to establish connection with the specified robot. The whitelisted network interface is
     * defined by its associated IPv4 address. For example, {"10.42.0.1", "192.168.2.102"}. If left
     * empty, all available network interfaces will be tried when searching for the specified robot.
     * @throw std::invalid_argument if the format of [robot_sn] is invalid.
     * @throw std::runtime_error if the initialization sequence failed.
     * @throw std::logic_error if the connected robot does not have a valid RDK license; or this RDK
     * library version is incompatible with the connected robot; or model of the connected robot is
     * not supported.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with the robot is established.
     */
    Robot(const std::string& robot_sn,
        const std::vector<std::string>& network_interface_whitelist = {});
    virtual ~Robot();

    //========================================= ACCESSORS ==========================================
    /**
     * @brief [Non-blocking] Whether the connection with the robot is established.
     * @return True: connected; false: disconnected.
     */
    bool connected() const;

    /**
     * @brief [Non-blocking] General information about the connected robot.
     * @return RobotInfo value copy.
     */
    const RobotInfo info() const;

    /**
     * @brief [Non-blocking] Current control mode of the robot.
     * @return flexiv::rdk::Mode enum.
     */
    Mode mode() const;

    /**
     * @brief [Non-blocking] Current states data of the robot.
     * @return RobotStates value copy.
     * @note Real-time (RT).
     */
    const RobotStates states() const;

    /**
     * @brief [Non-blocking] Whether the robot has come to a complete stop.
     * @return True: stopped; false: still moving.
     */
    bool stopped() const;

    /**
     * @brief [Non-blocking] Whether the robot is ready to be operated, which requires the
     * following conditions to be met: enabled, brakes fully released, in auto mode, no fault, and
     * not in reduced state.
     * @param[in] verbose Whether to print warning message indicating why the robot is not
     * operational when this function returns false.
     * @return True: operational (operational_status() == READY); false: not operational.
     * @warning The robot won't execute any user command until it's ready to be operated.
     */
    bool operational(bool verbose = true) const;

    /**
     * @brief [Non-blocking] Current operational status of the robot.
     * @return OperationalStatus enum.
     */
    OperationalStatus operational_status() const;

    /**
     * @brief [Non-blocking] Whether the robot is currently executing a task. This includes any
     * user commanded operations that requires the robot to execute. For example, plans, primitives,
     * Cartesian and joint motions, etc.
     * @return True: busy; false: idle.
     * @warning Some exceptions exist for primitives, see ExecutePrimitive() for more details.
     */
    bool busy() const;

    /**
     * @brief [Non-blocking] Whether the robot is in fault state.
     * @return True: robot has fault; false: robot normal.
     */
    bool fault() const;

    /**
     * @brief [Non-blocking] Whether the robot is in reduced state.
     * @return True: in reduced state; false: not in reduced state.
     * @par Reduced state
     * The robot will enter reduced state if a) the safety input for reduced state goes low or b)
     * robot TCP passes through any safety plane. The safety limits are lowered in reduced state
     * compared to normal state. Specific values for the safety limits can be configured in Flexiv
     * Elements under Settings -> Safety Configuration. Please refer to the robot user manual for
     * more details about system reduced state.
     */
    bool reduced() const;

    /**
     * @brief [Non-blocking] Whether the robot is in recovery state.
     * @return True: in recovery state; false: not in recovery state.
     * @note Use RunAutoRecovery() to execute automatic recovery operation.
     * @par Recovery state
     * The robot will enter recovery state if it needs to recover from joint position limit
     * violation (a critical system fault that requires a recovery operation, during which the
     * joints that moved outside the allowed position range will need to move very slowly back into
     * the allowed range). Please refer to the robot user manual for more details about system
     * recovery state.
     */
    bool recovery() const;

    /**
     * @brief [Non-blocking] Whether the emergency stop is released.
     * @return True: released; false: pressed.
     */
    bool estop_released() const;

    /**
     * @brief [Non-blocking] Whether the enabling button is pressed.
     * @return True: pressed; false: released.
     */
    bool enabling_button_pressed() const;

    /**
     * @brief [Non-blocking] Get multilingual log messages of the connected robot.
     * @return Robot log messages stored since the last successful instantiation of this class. Each
     * element in the string list corresponds to one message with timestamp and log level added. New
     * message is pushed to the back of the vector.
     * @note Possible log level tags are: [info], [warning], [error], and [critical].
     * @warning Messages before the last successful instantiation of this class are not available.
     */
    const std::vector<std::string> mu_log() const;

    //======================================= SYSTEM CONTROL =======================================
    /**
     * @brief [Blocking] Enable the robot, if E-stop is released and there's no fault, the robot
     * will release brakes, and becomes operational a few seconds later.
     * @throw std::logic_error if the robot is not connected.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Enable();

    /**
     * @brief [Blocking] Force robot brakes to engage or release during normal operation.
     * Restrictions apply, see warning.
     * @param[in] engage True: engage brakes; false: release brakes.
     * @throw std::logic_error if the connected robot is not a medical one or the robot is moving.
     * @throw std::runtime_error if failed to engage/release the brakes.
     * @note This function blocks until the brakes are successfully engaged/released.
     * @warning This function is accessible only if a) the connected robot is a medical one AND
     * b) the robot is not moving.
     */
    void Brake(bool engage);

    /**
     * @brief [Blocking] Switch to a new control mode and wait until mode transition is finished.
     * @param[in] mode flexiv::rdk::Mode enum.
     * @throw std::invalid_argument if the requested mode is invalid or unlicensed.
     * @throw std::logic_error if robot is in an unknown control mode or is not operational.
     * @throw std::runtime_error if failed to transit the robot into the specified control mode
     * after several attempts.
     * @note This function blocks until the robot has successfully transited into the specified
     * control mode.
     * @warning If the robot is still moving when this function is called, it will automatically
     * stop then make the mode transition.
     */
    void SwitchMode(Mode mode);

    /**
     * @brief [Blocking] Stop the robot and transit robot mode to IDLE.
     * @throw std::runtime_error if failed to stop the robot.
     * @note This function blocks until the robot comes to a complete stop.
     */
    void Stop();

    /**
     * @brief [Blocking] Try to clear minor or critical fault of the robot without a power cycle.
     * @param[in] timeout_sec Maximum time in seconds to wait for the fault to be successfully
     * cleared. Normally, a minor fault should take no more than 3 seconds to clear, and a critical
     * fault should take no more than 30 seconds to clear.
     * @return True: successfully cleared fault; false: failed to clear fault.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the fault is successfully cleared or [timeout_sec] has
     * elapsed.
     * @warning Clearing a critical fault through this function without a power cycle requires a
     * dedicated device, which may not be installed in older robot models.
     */
    bool ClearFault(unsigned int timeout_sec = 30);

    /**
     * @brief [Blocking] Run automatic recovery to bring joints that are outside the allowed
     * position range back into allowed range.
     * @throw std::runtime_error if failed to enter automatic recovery mode.
     * @note Refer to user manual for more details.
     * @note This function blocks until the automatic recovery process is finished.
     * @see recovery().
     */
    void RunAutoRecovery();

    /**
     * @brief [Blocking] Set values to global variables that already exist in the robot.
     * @param[in] global_vars A map of {global_var_name, global_var_value(s)}. Use int 1 and 0 to
     * represent booleans. For example, {{"camera_offset", {0.1, -0.2, 0.3}}, {"start_plan", 1}}.
     * @throw std::length_error if [global_vars] is empty or too long to transmit in one request.
     * @throw std::logic_error if any of the specified global variables does not exist.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the global variables are successfully set.
     * @warning The specified global variables need to be created first using Flexiv Elements.
     * @see global_variables().
     */
    void SetGlobalVariables(const std::map<std::string, FlexivDataTypes>& global_vars);

    [[deprecated("[Removing in v1.6] Use the other SetGlobalVariables() instead")]] void
    SetGlobalVariables(const std::string& global_vars);

    /**
     * @brief [Blocking] Existing global variables and their current values.
     * @return A map of {global_var_name, global_var_value(s)}. Booleans are represented by int 1
     * and 0. For example, {{"camera_offset", {0.1, -0.2, 0.3}}, {"start_plan", 1}}.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     * @see SetGlobalVariables().
     */
    std::map<std::string, FlexivDataTypes> global_variables() const;

    [[deprecated("[Removing in v1.6] Use the other global_variables() instead")]] const std::vector<
        std::string>
    global_variables(bool dummy); ///< Unused parameter [dummy] is needed for function overloading

    //======================================= PLAN EXECUTION =======================================
    /**
     * @brief [Blocking] Execute a plan by specifying its index.
     * @param[in] index Index of the plan to execute, can be obtained via plan_list().
     * @param[in] continue_exec Whether to continue executing the plan when the RDK program is
     * closed or the connection is lost.
     * @param[in] block_until_started Whether to wait for the commanded plan to finish loading
     * and start execution before the function returns. Depending on the amount of computation
     * needed to get the plan ready, the loading process typically takes no more than 200 ms.
     * @throw std::invalid_argument if [index] is outside the valid range.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until the request is successfully delivered if
     * [block_until_started] is disabled, or until the plan has started execution if
     * [block_until_started] is enabled.
     * @note busy() can be used to check if the plan has finished.
     */
    void ExecutePlan(
        unsigned int index, bool continue_exec = false, bool block_until_started = true);

    /**
     * @brief [Blocking] Execute a plan by specifying its name.
     * @param[in] name Name of the plan to execute, can be obtained via plan_list().
     * @param[in] continue_exec Whether to continue executing the plan when the RDK program is
     * closed or the connection is lost.
     * @param[in] block_until_started Whether to wait for the commanded plan to finish loading
     * and start execution before the function returns. Depending on the amount of computation
     * needed to get the plan ready, the loading process typically takes no more than 200 ms.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until the request is successfully delivered if
     * [block_until_started] is disabled, or until the plan has started execution if
     * [block_until_started] is enabled.
     * @note busy() can be used to check if the plan has finished.
     */
    void ExecutePlan(
        const std::string& name, bool continue_exec = false, bool block_until_started = true);

    /**
     * @brief [Blocking] Pause or resume the execution of the current plan.
     * @param[in] pause True: pause plan; false: resume plan.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until the request is successfully delivered.
     * @warning Internal plans (not created by user) cannot be resumed due to safety concerns.
     */
    void PausePlan(bool pause);

    /**
     * @brief [Blocking] Get a list of all available plans.
     * @return Available plans in the format of a string list.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    const std::vector<std::string> plan_list() const;

    /**
     * @brief [Blocking] Get detailed information about the currently executing plan. Contains
     * information like plan name, primitive name, node name, node path, node path time period, etc.
     * @return PlanInfo data struct.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until a reply is received.
     */
    const PlanInfo plan_info() const;

    /**
     * @brief [Blocking] Enable or disable the breakpoint mode during plan execution. When enabled,
     * the currently executing plan will pause at the pre-defined breakpoints. Use StepBreakpoint()
     * to continue the execution and pause at the next breakpoint.
     * @param[in] is_enabled True: enable; false: disable. By default, breakpoint mode is disabled.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until the request is successfully delivered.
     */
    void SetBreakpointMode(bool is_enabled);

    /**
     * @brief [Blocking] If breakpoint mode is enabled, step to the next breakpoint. The plan
     * execution will continue and pause at the next breakpoint.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: NRT_PLAN_EXECUTION.
     * @note This function blocks until the request is successfully delivered.
     * @note Use PlanInfo::waiting_for_step to check if the plan is currently waiting for user
     * signal to step the breakpoint.
     */
    void StepBreakpoint();

    /**
     * @brief [Blocking] Set overall velocity scale for robot motions during plan and primitive
     * execution.
     * @param[in] velocity_scale Percentage scale to adjust the overall velocity of robot motions.
     * Valid range: [0, 100]. Setting to 100 means to move with 100% of specified motion velocity,
     * and 0 means not moving at all.
     * @throw std::invalid_argument if [velocity_scale] is outside the valid range.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: NRT_PLAN_EXECUTION, NRT_PRIMITIVE_EXECUTION.
     * @note This function blocks until the request is successfully delivered.
     */
    void SetVelocityScale(unsigned int velocity_scale);

    //==================================== PRIMITIVE EXECUTION =====================================
    /**
     * @brief [Blocking] Execute a primitive by specifying its name and parameters, which can be
     * found in the [Flexiv Primitives documentation](https://www.flexiv.com/primitives/).
     * @param[in] primitive_name Primitive name. For example, "Home", "MoveL", "ZeroFTSensor", etc.
     * @param[in] input_params Specify basic and advanced parameters of the primitive via a map of
     * {input_parameter_name, input_parameter_value(s)}. Use int 1 and 0 to represent booleans. E.g.
     * {{"target", rdk::Coord({0.65, -0.3, 0.2}, {180, 0, 180}, {"WORLD", "WORLD_ORIGIN"})}, {"vel",
     * 0.6}, {"zoneRadius", "Z50"}}.
     * @param[in] properties Specify properties of the primitive via a map of {property_name,
     * property_value(s)}. Use int 1 and 0 to represent booleans. E.g. {{"lockExternalAxes", 0}}.
     * @param[in] block_until_started Whether to wait for the commanded primitive to finish loading
     * and start execution before the function returns. Depending on the amount of computation
     * needed to get the primitive ready, the loading process typically takes no more than 200 ms.
     * @throw std::length_error if [input_params] is too long to transmit in one request.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: NRT_PRIMITIVE_EXECUTION.
     * @note This function blocks until the request is successfully delivered if
     * [block_until_started] is disabled, or until the primitive has started execution if
     * [block_until_started] is enabled.
     * @warning The primitive input parameters may not use SI units, please refer to the Flexiv
     * Primitives documentation for exact unit definition.
     * @warning Most primitives won't exit by themselves and require users to explicitly trigger
     * transitions based on specific primitive states. In such case, busy() will stay true even if
     * it seems everything is done for that primitive.
     */
    void ExecutePrimitive(const std::string& primitive_name,
        const std::map<std::string, FlexivDataTypes>& input_params,
        const std::map<std::string, FlexivDataTypes>& properties = {},
        bool block_until_started = true);

    [[deprecated("[Removing in v1.6] Use the other ExecutePrimitive() instead")]] void
    ExecutePrimitive(const std::string& pt_cmd, bool block_until_started = true);

    /**
     * @brief [Blocking] State parameters of the executing primitive and their current values.
     * @return A map of {pt_state_name, pt_state_value(s)}. Booleans are represented by int 1
     * and 0. For example,
     * {{"reachedTarget", 1}, {"timePeriod", 5.6}, {"forceOffset", {0.1, 0.2, -1.3}}}.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    std::map<std::string, FlexivDataTypes> primitive_states() const;

    [[deprecated("[Removing in v1.6] Use the other primitive_states() instead")]] const std::vector<
        std::string>
    primitive_states(bool dummy); ///< Unused parameter [dummy] is needed for function overloading

    //==================================== DIRECT JOINT CONTROL ====================================
    /**
     * @brief [Non-blocking] Continuously stream joint torque command to the robot.
     * @param[in] torques Target joint torques: \f$ {\tau_J}_d \in \mathbb{R}^{n \times 1} \f$.
     * Unit: \f$ [Nm] \f$.
     * @param[in] enable_gravity_comp Enable/disable robot gravity compensation.
     * @param[in] enable_soft_limits Enable/disable soft limits to keep the joints from moving
     * outside allowed position range, which will trigger a safety fault that requires recovery
     * operation.
     * @throw std::invalid_argument if size of any input vector does not match robot DoF.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if number of timeliness failures has reached limit.
     * @note Applicable control modes: RT_JOINT_TORQUE.
     * @note Real-time (RT).
     * @warning Always stream smooth and continuous commands to avoid sudden movements.
     */
    void StreamJointTorque(const std::vector<double>& torques, bool enable_gravity_comp = true,
        bool enable_soft_limits = true);

    /**
     * @brief [Non-blocking] Continuously stream joint position, velocity, and acceleration command
     * to the robot. The commands are tracked by either the joint impedance controller or the joint
     * position controller, depending on the control mode.
     * @param[in] positions Target joint positions: \f$ q_d \in \mathbb{R}^{n \times 1} \f$. Unit:
     * \f$ [rad] \f$.
     * @param[in] velocities Target joint velocities: \f$ \dot{q}_d \in \mathbb{R}^{n \times 1}
     * \f$. Unit: \f$ [rad/s] \f$.
     * @param[in] accelerations Target joint accelerations: \f$ \ddot{q}_d \in \mathbb{R}^{n \times
     * 1} \f$. Unit: \f$ [rad/s^2] \f$.
     * @throw std::invalid_argument if size of any input vector does not match robot DoF.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if number of timeliness failures has reached limit.
     * @note Applicable control modes: RT_JOINT_IMPEDANCE, RT_JOINT_POSITION.
     * @note Real-time (RT).
     * @warning Always stream smooth and continuous commands to avoid sudden movements.
     * @see SetJointImpedance().
     */
    void StreamJointPosition(const std::vector<double>& positions,
        const std::vector<double>& velocities, const std::vector<double>& accelerations);

    /**
     * @brief [Non-blocking] Discretely send joint position, velocity, and acceleration command to
     * the robot. The robot's internal motion generator will smoothen the discrete commands, which
     * are tracked by either the joint impedance controller or the joint position controller,
     * depending on the control mode.
     * @param[in] positions Target joint positions: \f$ q_d \in \mathbb{R}^{n \times 1} \f$. Unit:
     * \f$ [rad] \f$.
     * @param[in] velocities Target joint velocities: \f$ \dot{q}_d \in \mathbb{R}^{n \times 1}
     * \f$. Each joint will maintain this amount of velocity when it reaches the target position.
     * Unit: \f$ [rad/s] \f$.
     * @param[in] accelerations Target joint accelerations: \f$ \ddot{q}_d \in \mathbb{R}^{n \times
     * 1} \f$. Each joint will maintain this amount of acceleration when it reaches the target
     * position. Unit: \f$ [rad/s^2] \f$.
     * @param[in] max_vel Maximum joint velocities for the planned trajectory: \f$ \dot{q}_{max} \in
     * \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s] \f$.
     * @param[in] max_acc Maximum joint accelerations for the planned trajectory: \f$ \ddot{q}_{max}
     * \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s^2] \f$.
     * @throw std::invalid_argument if size of any input vector does not match robot DoF.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @note Applicable control modes: NRT_JOINT_IMPEDANCE, NRT_JOINT_POSITION.
     * @warning Calling this function a second time while the motion from the previous call is still
     * ongoing will trigger an online re-planning of the joint trajectory, such that the previous
     * command is aborted and the new command starts to execute.
     * @see SetJointImpedance().
     */
    void SendJointPosition(const std::vector<double>& positions,
        const std::vector<double>& velocities, const std::vector<double>& accelerations,
        const std::vector<double>& max_vel, const std::vector<double>& max_acc);

    /**
     * @brief [Blocking] Set impedance properties of the robot's joint motion controller used in
     * the joint impedance control modes.
     * @param[in] K_q Joint motion stiffness: \f$ K_q \in \mathbb{R}^{n \times 1} \f$.
     * Setting motion stiffness of a joint axis to 0 will make this axis free-floating. Valid range:
     * [0, RobotInfo::K_q_nom]. Unit: \f$ [Nm/rad] \f$.
     * @param[in] Z_q Joint motion damping ratio: \f$ Z_q \in \mathbb{R}^{n \times 1} \f$.
     * Valid range: [0.3, 0.8]. The nominal (safe) value is provided as default.
     * @throw std::invalid_argument if [K_q] or [Z_q] contains any value outside the valid range or
     * size of any input vector does not match robot DoF.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @note Applicable control modes: RT_JOINT_IMPEDANCE, NRT_JOINT_IMPEDANCE.
     * @note This function blocks until the request is successfully delivered.
     * @warning Changing damping ratio [Z_q] to a non-nominal value may lead to performance and
     * stability issues, please use with caution.
     */
    void SetJointImpedance(const std::vector<double>& K_q,
        const std::vector<double>& Z_q = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7});

    //================================== DIRECT CARTESIAN CONTROL ==================================
    /**
     * @brief [Non-blocking] Continuously stream Cartesian motion and/or force command for the robot
     * to track using its unified motion-force controller, which allows doing force control in zero
     * or more Cartesian axes and motion control in the rest axes.
     * @param[in] pose Target TCP pose in world frame: \f$ {^{O}T_{TCP}}_{d} \in \mathbb{R}^{7
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     * @param[in] wrench Target TCP wrench (force and moment) in the force control reference frame
     * (configured by SetForceControlFrame()): \f$ ^{0}F_d \in \mathbb{R}^{6 \times 1} \f$. The
     * robot will track the target wrench using an explicit force controller. Consists of \f$
     * \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y,
     * f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]:[Nm] \f$.
     * @param[in] velocity Target TCP velocity (linear and angular) in world frame: \f$
     * ^{0}\dot{x}_d \in \mathbb{R}^{6 \times 1} \f$. Providing properly calculated target velocity
     * can improve the robot's overall tracking performance at the cost of reduced robustness.
     * Leaving this input 0 can maximize robustness at the cost of reduced tracking performance.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear and \f$ \mathbb{R}^{3 \times 1} \f$
     * angular velocity. Unit: \f$ [m/s]:[rad/s] \f$.
     * @param[in] acceleration Target TCP acceleration (linear and angular) in world frame: \f$
     * ^{0}\ddot{x}_d \in \mathbb{R}^{6 \times 1} \f$. Feeding forward target acceleration can
     * improve the robot's tracking performance for highly dynamic motions, but it's also okay to
     * leave this input 0. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular acceleration. Unit: \f$ [m/s^2]:[rad/s^2] \f$.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if number of timeliness failures has reached limit.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE.
     * @note Real-time (RT).
     * @warning Always stream smooth and continuous motion commands to avoid sudden movements. The
     * force commands don't need to be continuous.
     * @warning Same as Flexiv Elements, the target wrench is expressed as wrench sensed at TCP
     * instead of wrench exerted by TCP. E.g. commanding f_z = +5 N will make the end-effector move
     * towards -Z direction, so that upon contact, the sensed force will be +5 N.
     * @par How to achieve pure motion control?
     * Use SetForceControlAxis() to disable force control for all Cartesian axes to achieve pure
     * motion control. This function does pure motion control by default.
     * @par How to achieve pure force control?
     * Use SetForceControlAxis() to enable force control for all Cartesian axes to achieve pure
     * force control, active or passive.
     * @par How to achieve unified motion-force control?
     * Use SetForceControlAxis() to enable force control for one or more Cartesian axes and leave
     * the rest axes motion-controlled, then provide target pose for the motion-controlled axes and
     * target wrench for the force-controlled axes.
     * @see SetCartesianImpedance(), SetMaxContactWrench(), SetNullSpacePosture(),
     * SetForceControlAxis(), SetForceControlFrame(), SetPassiveForceControl().
     */
    void StreamCartesianMotionForce(const std::array<double, kPoseSize>& pose,
        const std::array<double, kCartDoF>& wrench = {},
        const std::array<double, kCartDoF>& velocity = {},
        const std::array<double, kCartDoF>& acceleration = {});

    /**
     * @brief [Non-blocking] Discretely send Cartesian motion and/or force command for the robot to
     * track using its unified motion-force controller, which allows doing force control in zero or
     * more Cartesian axes and motion control in the rest axes. The robot's internal motion
     * generator will smoothen the discrete commands.
     * @param[in] pose Target TCP pose in world frame: \f$ {^{O}T_{TCP}}_{d} \in \mathbb{R}^{7
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     * @param[in] wrench Target TCP wrench (force and moment) in the force control reference frame
     * (configured by SetForceControlFrame()): \f$ ^{0}F_d \in \mathbb{R}^{6 \times 1} \f$. The
     * robot will track the target wrench using an explicit force controller. Consists of \f$
     * \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y,
     * f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]:[Nm] \f$.
     * @param[in] max_linear_vel  Maximum Cartesian linear velocity when moving to the target pose.
     * A safe value is provided as default. Unit: \f$ [m/s] \f$.
     * @param[in] max_angular_vel  Maximum Cartesian angular velocity when moving to the target
     * pose. A safe value is provided as default. Unit: \f$ [rad/s] \f$.
     * @param[in] max_linear_acc  Maximum Cartesian linear acceleration when moving to the target
     * pose. A safe value is provided as default. Unit: \f$ [m/s^2] \f$.
     * @param[in] max_angular_acc  Maximum Cartesian angular acceleration when moving to the target
     * pose. A safe value is provided as default. Unit: \f$ [rad/s^2] \f$.
     * @throw std::invalid_argument if any of the last 4 input parameters is negative.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @note Applicable control modes: NRT_CARTESIAN_MOTION_FORCE.
     * @warning Same as Flexiv Elements, the target wrench is expressed as wrench sensed at TCP
     * instead of wrench exerted by TCP. E.g. commanding f_z = +5 N will make the end-effector move
     * towards -Z direction, so that upon contact, the sensed force will be +5 N.
     * @par How to achieve pure motion control?
     * Use SetForceControlAxis() to disable force control for all Cartesian axes to achieve pure
     * motion control. This function does pure motion control by default.
     * @par How to achieve pure force control?
     * Use SetForceControlAxis() to enable force control for all Cartesian axes to achieve pure
     * force control, active or passive.
     * @par How to achieve unified motion-force control?
     * Use SetForceControlAxis() to enable force control for one or more Cartesian axes and leave
     * the rest axes motion-controlled, then provide target pose for the motion-controlled axes and
     * target wrench for the force-controlled axes.
     * @see SetCartesianImpedance(), SetMaxContactWrench(), SetNullSpacePosture(),
     * SetForceControlAxis(), SetForceControlFrame(), SetPassiveForceControl().
     */
    void SendCartesianMotionForce(const std::array<double, kPoseSize>& pose,
        const std::array<double, kCartDoF>& wrench = {}, double max_linear_vel = 0.5,
        double max_angular_vel = 1.0, double max_linear_acc = 2.0, double max_angular_acc = 5.0);

    /**
     * @brief [Blocking] Set impedance properties of the robot's Cartesian motion controller
     * used in the Cartesian motion-force control modes.
     * @param[in] K_x Cartesian motion stiffness: \f$ K_x \in \mathbb{R}^{6 \times 1} \f$.
     * Setting motion stiffness of a motion-controlled Cartesian axis to 0 will make this axis
     * free-floating. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear stiffness and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular stiffness: \f$ [k_x, k_y, k_z, k_{Rx}, k_{Ry}, k_{Rz}]^T
     * \f$. Valid range: [0, RobotInfo::K_x_nom]. Unit: \f$ [N/m]:[Nm/rad] \f$.
     * @param[in] Z_x Cartesian motion damping ratio: \f$ Z_x \in \mathbb{R}^{6 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear damping ratio and \f$ \mathbb{R}^{3 \times
     * 1} \f$ angular damping ratio: \f$ [\zeta_x, \zeta_y, \zeta_z, \zeta_{Rx}, \zeta_{Ry},
     * \zeta_{Rz}]^T \f$. Valid range: [0.3, 0.8]. The nominal (safe) value is provided as default.
     * @throw std::invalid_argument if [K_x] or [Z_x] contains any value outside the valid range.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE.
     * @note This function blocks until the request is successfully delivered.
     * @warning Changing damping ratio [Z_x] to a non-nominal value may lead to performance and
     * stability issues, please use with caution.
     */
    void SetCartesianImpedance(const std::array<double, kCartDoF>& K_x,
        const std::array<double, kCartDoF>& Z_x = {0.7, 0.7, 0.7, 0.7, 0.7, 0.7});

    /**
     * @brief [Blocking] Set maximum contact wrench for the motion control part of the Cartesian
     * motion-force control modes. The controller will regulate its output to maintain contact
     * wrench (force and moment) with the environment under the set values.
     * @param[in] max_wrench Maximum contact wrench (force and moment): \f$ F_max \in \mathbb{R}^{6
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ maximum force and \f$
     * \mathbb{R}^{3 \times 1} \f$ maximum moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit:
     * \f$ [N]:[Nm] \f$.
     * @throw std::invalid_argument if [max_wrench] contains any negative value.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @note The maximum contact wrench regulation only applies to the motion control part.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE.
     * @note This function blocks until the request is successfully delivered.
     * @warning The maximum contact wrench regulation cannot be enabled if any of the rotational
     * Cartesian axes is enabled for moment control.
     */
    void SetMaxContactWrench(const std::array<double, kCartDoF>& max_wrench);

    /**
     * @brief [Blocking] Set reference joint positions for the null-space posture control module
     * used in the Cartesian motion-force control modes.
     * @param[in] ref_positions Reference joint positions for the null-space posture control:
     * \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Valid range: [RobotInfo::q_min,
     * RobotInfo::q_max]. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if [ref_positions] contains any value outside the valid
     * range or size of any input vector does not match robot DoF.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE.
     * @note This function blocks until the request is successfully delivered.
     * @warning The reference joint positions will be automatically reset to the robot's current
     * joint positions upon re-entering the applicable control modes.
     * @par Null-space posture control
     * Similar to human arm, a robotic arm with redundant joint-space degree(s) of freedom (DoF > 6)
     * can change its overall posture without affecting the ongoing primary task. This is achieved
     * through a technique called "null-space control". After the reference joint positions of a
     * desired robot posture are set using this function, the robot's null-space control module will
     * try to pull the arm as close to this posture as possible without affecting the primary
     * Cartesian motion-force control task.
     */
    void SetNullSpacePosture(const std::vector<double>& ref_positions);

    /**
     * @brief [Blocking] Set weights of the three optimization objectives while computing the
     * robot's null-space posture. Change the weights to optimize robot performance for different
     * use cases.
     * @param[in] linear_manipulability Increase this weight to improve the robot's capability to
     * translate freely in Cartesian space, i.e. a broader range of potential translation movements.
     * Valid range: [0.0, 1.0].
     * @param[in] angular_manipulability Increase this weight to improve the robot's capability to
     * rotate freely in Cartesian space, i.e. a broader range of potential rotation movements. Valid
     * range: [0.0, 1.0].
     * @param[in] ref_positions_tracking Increase this weight to make the robot track closer to the
     * reference joint positions specified using SetNullSpacePosture(). Valid range: [0.1, 1.0].
     * @throw std::invalid_argument if any of the input parameters is outside its valid range.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note The default value is provided for each parameter.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE.
     * @note This function blocks until the request is successfully delivered.
     * @warning The optimization weights will be automatically reset to the provided default values
     * upon re-entering the applicable control modes.
     */
    void SetNullSpaceObjectives(double linear_manipulability = 0.0,
        double angular_manipulability = 0.0, double ref_positions_tracking = 0.5);

    /**
     * @brief [Blocking] Set Cartesian axes to enable force control while in the Cartesian
     * motion-force control modes. Axes not enabled for force control will be motion-controlled.
     * @param[in] enabled_axes Flags to enable/disable force control for certain Cartesian axes in
     * the force control reference frame (configured by SetForceControlFrame()). The axis order is
     * \f$ [X, Y, Z, Rx, Ry, Rz] \f$.
     * @param[in] max_linear_vel For linear Cartesian axes that are enabled for force control, limit
     * the moving velocity to these values as a protection mechanism in case of contact loss. The
     * axis order is \f$ [X, Y, Z] \f$. Valid range: [0.005, 2.0]. Unit: \f$ [m/s] \f$.
     * @throw std::invalid_argument if [max_linear_vel] contains any value outside the valid range.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE.
     * @note This function blocks until the request is successfully delivered.
     * @note If not set, force control is disabled for all Cartesian axes by default.
     * @warning The maximum linear velocity protection for force control axes is only effective
     * under active force control (i.e. passive force control disabled), see
     * SetPassiveForceControl().
     */
    void SetForceControlAxis(const std::array<bool, kCartDoF>& enabled_axes,
        const std::array<double, kCartDoF / 2>& max_linear_vel = {1.0, 1.0, 1.0});

    /**
     * @brief [Blocking] Set reference frame for force control while in the Cartesian motion-force
     * control modes. The force control frame is defined by specifying its transformation with
     * regard to the root coordinate.
     * @param[in] root_coord Reference coordinate of [T_in_root].
     * @param[in] T_in_root Transformation from [root_coord] to the user-defined force control
     * frame: \f$ ^{root}T_{force} \in \mathbb{R}^{7 \times 1} \f$. Consists of \f$ \mathbb{R}^{3
     * \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z, q_w,
     * q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$. If root coordinate is a fixed one (e.g. WORLD),
     * then the force control frame will also be fixed; if root coordinate is a moving one (e.g.
     * TCP), then the force control frame will also be moving with the root coordinate. An identity
     * transformation is provided as default.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE, NRT_CARTESIAN_MOTION_FORCE.
     * @note This function blocks until the request is successfully delivered.
     * @note If not set, the robot will use WORLD origin as the force control frame by default.
     * @par Force control frame
     * In Cartesian motion-force control modes, the reference frame of motion control is always the
     * world frame, but the reference frame of force control can be an arbitrary one. While the
     * world frame is the commonly used global coordinate, the current TCP frame is a dynamic local
     * coordinate whose transformation with regard to the world frame changes as the robot TCP
     * moves. When using world frame with no transformation as the force control frame, the
     * force-controlled axes and motion-controlled axes are guaranteed to be orthogonal. Otherwise,
     * the force-controlled axes and motion-controlled axes are NOT guaranteed to be orthogonal
     * because different reference frames are used. In this case, it's recommended but not required
     * to set the target pose such that the actual robot motion direction(s) are orthogonal to force
     * direction(s). If they are not orthogonal, the motion control's vector component(s) in the
     * force direction(s) will be eliminated.
     */
    void SetForceControlFrame(CoordType root_coord,
        const std::array<double, kPoseSize>& T_in_root = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});

    /**
     * @brief [Blocking] Enable or disable passive force control for the Cartesian motion-force
     * control modes. When enabled, an open-loop force controller will be used to feed forward the
     * target wrench, i.e. passive force control. When disabled, a closed-loop force controller will
     * be used to track the target wrench, i.e. active force control.
     * @param[in] is_enabled True: enable; false: disable. By default, passive force control is
     * disabled and active force control is used.
     * @throw std::logic_error if robot is not in an applicable control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     * @note If not set, the passive force control is disabled by default.
     * @par Difference between active and passive force control
     * Active force control uses a feedback loop to reduce the error between target wrench and
     * measured wrench. This method results in better force tracking performance, but at the cost of
     * additional Cartesian damping which could potentially decrease motion tracking performance. On
     * the other hand, passive force control simply feeds forward the target wrench. This methods
     * results in worse force tracking performance, but is more robust and does not introduce
     * additional Cartesian damping. The choice of active or passive force control depends on the
     * actual application.
     */
    void SetPassiveForceControl(bool is_enabled);

    //======================================== IO CONTROL ========================================
    /**
     * @brief [Blocking] Set one or more digital output ports, including 16 on the control box plus
     * 2 inside the wrist connector.
     * @param[in] port_idx Index of port(s) to set, can be a single port or multiple ports.
     * E.g. {0, 5, 7, 15} or {1, 3, 10} or {8}. Valid range of the index number is [0–17].
     * @param[in] values Corresponding values to set to the specified ports. True: set port high,
     * false: set port low. Vector size must match the size of port_idx.
     * @throw std::invalid_argument if [port_idx] contains any index number outside the valid range.
     * @throw std::length_error if the two input vectors have different sizes.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void SetDigitalOutputs(
        const std::vector<unsigned int>& port_idx, const std::vector<bool>& values);

    [[deprecated("[Removing in v1.6] Use SetDigitalOutputs() instead")]] void WriteDigitalOutput(
        const std::vector<unsigned int>& port_idx, const std::vector<bool>& values);

    /**
     * @brief [Non-blocking] Current reading of all digital input ports, including 16 on the control
     * box plus 2 inside the wrist connector.
     * @return A boolean array whose index corresponds to that of the digital input ports.
     * True: port high; false: port low.
     */
    const std::array<bool, kIOPorts> digital_inputs();

    [[deprecated(
        "[Removing in v1.6] Use digital_inputs() instead")]] const std::array<bool, kIOPorts>
    ReadDigitalInput();

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;

    friend class Device;
    friend class FileIO;
    friend class Gripper;
    friend class Model;
    friend class Tool;
    friend class WorkCoord;
};

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_ROBOT_HPP_ */
