/**
 * @file Robot.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_ROBOT_HPP_
#define FLEXIVRDK_ROBOT_HPP_

#include "Data.hpp"
#include "Mode.hpp"

#include <vector>
#include <memory>
#include <exception>

namespace flexiv {

/**
 * @class Robot
 * @brief Main interface with the robot, containing several function categories and background
 * services.
 */
class Robot
{
public:
    /**
     * @brief [Blocking] Create a flexiv::Robot instance as the main robot interface. RDK services
     * will initialize and connection with the robot will be established.
     * @param[in] robotSN Serial number of the robot to connect.
     * @throw std::runtime_error if the initialization sequence failed.
     * @throw std::logic_error if the connected robot does not have a valid RDK license; or this RDK
     * library version is incompatible with the connected robot; or model of the connected robot is
     * not supported.
     * @warning This constructor blocks until the initialization sequence is successfully finished
     * and connection with the robot is established.
     */
    Robot(const std::string& robotSN);
    virtual ~Robot();

    /**
     * @brief [Non-blocking] Access general information of the robot.
     * @return RobotInfo instance.
     */
    RobotInfo info(void) const;

    //======================================= SYSTEM CONTROL =======================================
    /**
     * @brief [Blocking] Enable the robot, if E-stop is released and there's no fault, the robot
     * will release brakes, and becomes operational a few seconds later.
     * @throw std::logic_error if the robot is not connected.
     * @throw std::runtime_error if failed to execute the request.
     * @warning This function blocks until the request is successfully executed.
     */
    void enable(void);

    /**
     * @brief [Blocking] Stop the robot and transit robot mode to Idle.
     * @throw std::runtime_error if failed to stop the robot.
     * @warning This function blocks until the robot comes to a complete stop.
     */
    void stop(void);

    /**
     * @brief [Non-blocking] Check if the robot has come to a complete stop.
     * @return True: stopped, false: still moving.
     */
    bool isStopped(void) const;

    /**
     * @brief [Non-blocking] Check if the robot is normally operational, which requires the
     * following conditions to be met: enabled, brakes fully released, in auto mode, no fault, and
     * not in reduced state.
     * @param[in] verbose Whether to print warning message indicating why the robot is not
     * operational when this function returns false.
     * @return True: operational, false: not operational.
     * @warning The robot won't execute any user command until it becomes normally operational.
     */
    bool isOperational(bool verbose = true) const;

    /**
     * @brief [Non-blocking] Check if the robot is in fault state.
     * @return True: robot has fault, false: robot normal.
     */
    bool isFault(void) const;

    /**
     * @brief [Blocking] Clear minor fault of the robot.
     * @throw std::runtime_error if failed to execute the request.
     * @warning This function blocks until the request is successfully executed.
     */
    void clearFault(void);

    /**
     * @brief [Non-blocking] Check if the robot is currently executing a task. This includes any
     * user commanded operations that requires the robot to execute. For example, plans, primitives,
     * Cartesian and joint motions, etc.
     * @return True: busy, false: idle.
     * @warning Some exceptions exist for primitives, see executePrimitive()
     * warning for more details.
     */
    bool isBusy(void) const;

    /**
     * @brief [Non-blocking] Check if the connection with the robot is established.
     * @return True: connected, false: disconnected.
     */
    bool isConnected(void) const;

    /**
     * @brief [Non-blocking] Check if the emergency stop is released.
     * @return True: released, false: pressed.
     */
    bool isEstopReleased(void) const;

    /**
     * @brief [Non-blocking] Check if the enabling button is pressed.
     * @return True: pressed, false: released.
     */
    bool isEnablingButtonPressed(void) const;

    /**
     * @brief [Non-blocking] Check if the robot system is in recovery state.
     * @return True: in recovery state, false: not in recovery state.
     * @note Use runAutoRecovery() to carry out automatic recovery operation.
     * @par Recovery state
     * The robot system will enter recovery state if it needs to recover from joint position limit
     * violation (a critical system fault that requires a recovery operation, during which the
     * joints that moved outside the allowed position range will need to move very slowly back into
     * the allowed range). Refer to user manual for more details about system recovery state.
     */
    bool isRecoveryState(void) const;

    /**
     * @brief [Blocking] Run automatic recovery to bring joints that are outside the allowed
     * position range back into allowed range.
     * @throw std::runtime_error if failed to enter automatic recovery mode.
     * @note Refer to user manual for more details.
     * @warning This function blocks until the automatic recovery process is finished.
     * @see isRecoveryState()
     */
    void runAutoRecovery(void);

    /**
     * @brief [Blocking] Set a new control mode and wait until the mode transition is finished.
     * @param[in] mode flexiv::Mode enum.
     * @throw std::invalid_argument if the requested mode is invalid or unlicensed.
     * @throw std::logic_error if robot is in an unknown control mode or is not operational.
     * @throw std::runtime_error if failed to transit the robot into the specified control mode
     * after several attempts.
     * @warning If the robot is still moving when this function is called, it will automatically
     * stop then make the mode transition.
     * @warning This function blocks until the robot has successfully transited into the specified
     * control mode.
     */
    void setMode(Mode mode);

    /**
     * @brief [Non-blocking] Get the current control mode of the robot.
     * @return flexiv::Mode enum.
     */
    Mode getMode(void) const;

    /**
     * @brief [Non-blocking] Get the current robot states.
     * @param[out] output Reference to an existing RobotStates instance.
     * @note Call this function periodically to keep the output data object up to date.
     */
    void getRobotStates(RobotStates& output) const;

    /**
     * @brief [Non-blocking] Get the current robot states.
     * @return RobotStates instance.
     * @warning This function is less efficient than the other overloaded one as additional runtime
     * memory allocation and data copying are performed.
     */
    RobotStates getRobotStates(void) const;

    //======================================= PLAN EXECUTION =======================================
    /**
     * @brief [Blocking] Execute a plan by specifying its index.
     * @param[in] index Index of the plan to execute, can be obtained via getPlanNameList().
     * @param[in] velocityScale Percentage scale to adjust robot motion velocity, from 0 to 100.
     * 100 means to move with 100% of configured motion velocity, and 0 means not moving at all.
     * @param[in] continueExec Whether to continue executing the plan when
     * the RDK program is closed or the connection is lost.
     * @throw std::invalid_argument if index or velocityScale is invalid.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @note isBusy() can be used to check if a plan task has finished.
     * @warning This function blocks until the request is successfully delivered to the robot.
     */
    void executePlan(
        unsigned int index, unsigned int velocityScale = 100, bool continueExec = false);

    /**
     * @brief [Blocking] Execute a plan by specifying its name.
     * @param[in] name Name of the plan to execute, can be obtained via getPlanNameList().
     * @param[in] velocityScale Percentage scale to adjust robot motion velocity, from 0 to 100.
     * 100 means to move with 100% of configured motion velocity, and 0 means not moving at all.
     * @param[in] continueExec Whether to continue executing the plan when
     * the RDK program is closed or the connection is lost.
     * @throw std::invalid_argument if velocityScale is invalid.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @note isBusy() can be used to check if a plan task has finished.
     * @warning This function blocks until the request is successfully delivered to the robot.
     */
    void executePlan(
        const std::string& name, unsigned int velocityScale = 100, bool continueExec = false);

    /**
     * @brief [Blocking] Pause or resume the execution of the current plan.
     * @param[in] pause True: pause plan, false: resume plan.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @warning This function blocks until the request is successfully delivered to the robot.
     */
    void pausePlan(bool pause);

    /**
     * @brief [Blocking] Get a list of all available plans from the robot.
     * @return Available plans in the format of a string list.
     * @throw std::runtime_error if failed to get a reply from the robot.
     * @warning This function blocks until the reply from the robot is received.
     */
    std::vector<std::string> getPlanNameList(void) const;

    /**
     * @brief [Blocking] Get detailed information about the currently executing plan. Contains
     * information like plan name, primitive name, node name, node path, node path time period, etc.
     * @return PlanInfo instance.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to get a reply from the robot.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @warning This function blocks until the reply from the robot is received.
     */
    PlanInfo getPlanInfo(void) const;

    /**
     * @brief [Blocking] Set global variables for the robot by specifying name and value.
     * @param[in] globalVars Command to set global variables using the format:
     * globalVar1=value(s), globalVar2=value(s), ...
     * @throw std::length_error if size of globalVars exceeds the limit (10 Kb).
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @warning The specified global variable(s) must have already been created in the robot using
     * Flexiv Elements, otherwise setting a nonexistent global variable will have no effect. To
     * check if a global variable is successfully set, use getGlobalVariables().
     * @warning This function blocks until the request is successfully delivered to the robot.
     */
    void setGlobalVariables(const std::string& globalVars);

    /**
     * @brief [Blocking] Get available global variables from the robot.
     * @return Global variables in the format of a string list.
     * @throw std::runtime_error if failed to get a reply from the robot.
     * @warning This function blocks until the reply from the robot is received.
     */
    std::vector<std::string> getGlobalVariables(void) const;

    /**
     * @brief [Blocking] Enable or disable the breakpoint mode during plan execution. When enabled,
     * the currently executing plan will pause at the pre-defined breakpoints. Use stepBreakpoint()
     * to continue the execution and pause at the next breakpoint.
     * @param[in] isEnabled True: enable, false: disable. By default, breakpoint mode is disabled.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @warning This function blocks until the request is successfully executed.
     */
    void setBreakpointMode(bool isEnabled);

    /**
     * @brief [Blocking] If breakpoint mode is enabled, step to the next breakpoint. The plan
     * execution will continue and pause at the next breakpoint.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @note Use PlanInfo::waitingForStep to check if the plan is currently waiting for user signal
     * to step the breakpoint.
     * @warning This function blocks until the request is successfully executed.
     */
    void stepBreakpoint();

    //==================================== PRIMITIVE EXECUTION =====================================
    /**
     * @brief [Blocking] Execute a primitive by specifying its name and parameters, which can be
     * found in the [Flexiv Primitives documentation](https://www.flexiv.com/primitives/).
     * @param[in] ptCmd Primitive command with the following string format:
     * "primitiveName(inputParam1=xxx, inputParam2=xxx, ...)".
     * @param[in] velocityScale Percentage scale to adjust robot motion velocity, from 0 to 100.
     * 100 means to move with 100% of configured motion velocity, and 0 means not moving at all.
     * @throw std::length_error if size of ptCmd exceeds the limit (10 Kb).
     * @throw std::invalid_argument if velocityScale is invalid.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @note Applicable control mode: NRT_PRIMITIVE_EXECUTION.
     * @warning The primitive input parameters may not use SI units, please refer to the Flexiv
     * Primitives documentation for exact unit definition.
     * @warning Some primitives may not terminate automatically and require users to manually
     * terminate them based on specific primitive states, for example, most [Move] primitives. In
     * such case, isBusy() will stay true even if it seems everything is done for that primitive.
     * @warning This function blocks until the request is successfully delivered to the robot.
     */
    void executePrimitive(const std::string& ptCmd, unsigned int velocityScale = 100);

    /**
     * @brief [Blocking] Get feedback states of the currently executing primitive.
     * @return Primitive states in the format of a string list.
     * @throw std::runtime_error if failed to get a reply from the robot.
     * @warning This function blocks until the reply from the robot is received.
     */
    std::vector<std::string> getPrimitiveStates(void) const;

    //==================================== DIRECT JOINT CONTROL ====================================
    /**
     * @brief [Non-blocking] Continuously stream joint torque command to the robot.
     * @param[in] torques Target joint torques: \f$ {\tau_J}_d \in \mathbb{R}^{n \times 1} \f$.
     * Unit: \f$ [Nm] \f$.
     * @param[in] enableGravityComp Enable/disable robot gravity compensation.
     * @param[in] enableSoftLimits Enable/disable soft limits to keep the joints from moving outside
     * allowed position range, which will trigger a safety fault that requires recovery operation.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if number of timeliness failures has reached limit.
     * @note Applicable control mode: RT_JOINT_TORQUE.
     * @note Real-time (RT).
     * @warning Always stream smooth and continuous commands to avoid sudden movements.
     */
    void streamJointTorque(const std::array<double, k_jointDOF>& torques,
        bool enableGravityComp = true, bool enableSoftLimits = true);

    /**
     * @brief [Non-blocking] Continuously stream joint position, velocity, and acceleration command
     * to the robot.
     * @param[in] positions Target joint positions: \f$ q_d \in \mathbb{R}^{n \times 1} \f$. Unit:
     * \f$ [rad] \f$.
     * @param[in] velocities Target joint velocities: \f$ \dot{q}_d \in \mathbb{R}^{n \times 1}
     * \f$. Unit: \f$ [rad/s] \f$.
     * @param[in] accelerations Target joint accelerations: \f$ \ddot{q}_d \in \mathbb{R}^{n \times
     * 1} \f$. Unit: \f$ [rad/s^2] \f$.
     * @throw std::invalid_argument if input is invalid.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if number of timeliness failures has reached limit.
     * @note Applicable control mode: RT_JOINT_POSITION.
     * @note Real-time (RT).
     * @warning Always stream smooth and continuous commands to avoid sudden movements.
     */
    void streamJointPosition(const std::array<double, k_jointDOF>& positions,
        const std::array<double, k_jointDOF>& velocities,
        const std::array<double, k_jointDOF>& accelerations);

    /**
     * @brief [Non-blocking] Discretely send joint position, velocity, and acceleration command. The
     * robot's internal motion generator will smoothen the discrete commands.
     * @param[in] positions Target joint positions: \f$ q_d \in \mathbb{R}^{DOF \times 1} \f$. Unit:
     * \f$ [rad] \f$.
     * @param[in] velocities Target joint velocities: \f$ \dot{q}_d \in \mathbb{R}^{DOF \times 1}
     * \f$. Each joint will maintain this amount of velocity when it reaches the target position.
     * Unit: \f$ [rad/s] \f$.
     * @param[in] accelerations Target joint accelerations: \f$ \ddot{q}_d \in \mathbb{R}^{DOF
     * \times 1} \f$. Each joint will maintain this amount of acceleration when it reaches the
     * target position. Unit: \f$ [rad/s^2] \f$.
     * @param[in] maxVel Maximum joint velocities for the planned trajectory: \f$ \dot{q}_{max} \in
     * \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s] \f$.
     * @param[in] maxAcc Maximum joint accelerations for the planned trajectory: \f$ \ddot{q}_{max}
     * \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s^2] \f$.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @note Applicable control mode: NRT_JOINT_POSITION.
     * @warning Calling this function a second time while the motion from the previous call is still
     * ongoing will trigger an online re-planning of the joint trajectory, such that the previous
     * command is aborted and the new command starts to execute.
     */
    void sendJointPosition(const std::array<double, k_jointDOF>& positions,
        const std::array<double, k_jointDOF>& velocities,
        const std::array<double, k_jointDOF>& accelerations,
        const std::array<double, k_jointDOF>& maxVel, const std::array<double, k_jointDOF>& maxAcc);

    //================================== DIRECT CARTESIAN CONTROL ==================================
    /**
     * @brief [Non-blocking] Continuously stream Cartesian motion and/or force command for the robot
     * to track using its unified motion-force controller, which allows doing force control in zero
     * or more Cartesian axes and motion control in the rest axes.
     * @param[in] pose Target TCP pose in world frame: \f$ {^{O}T_{TCP}}_{d} \in \mathbb{R}^{7
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     * @param[in] wrench Target TCP wrench (force and moment) in the force control reference frame
     * (configured by setForceControlFrame()): \f$ ^{0}F_d \in \mathbb{R}^{6 \times 1} \f$. The
     * robot will track the target wrench using an explicit force controller. Consists of \f$
     * \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y,
     * f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if number of timeliness failures has reached limit.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE.
     * @note Real-time (RT).
     * @warning Always stream smooth and continuous motion commands to avoid sudden movements. The
     * force commands don't need to be continuous.
     * @par How to achieve pure motion control?
     * Use setForceControlAxis() to disable force control for all Cartesian axes to achieve pure
     * motion control. This function does pure motion control out of the box.
     * @par How to achieve pure force control?
     * Use setForceControlAxis() to enable force control for all Cartesian axes to achieve pure
     * force control, active or passive.
     * @par How to achieve unified motion-force control?
     * Use setForceControlAxis() to enable force control for one or more Cartesian axes and leave
     * the rest axes motion-controlled, then provide target pose for the motion-controlled axes and
     * target wrench for the force-controlled axes.
     * @see setCartesianStiffness(), setMaxContactWrench(), setNullSpacePosture(),
     * setForceControlAxis(), setForceControlFrame(), setPassiveForceControl().
     */
    void streamCartesianMotionForce(const std::array<double, k_poseSize>& pose,
        const std::array<double, k_cartDOF>& wrench = {});

    /**
     * @brief [Non-blocking] Discretely send Cartesian motion and/or force command for the robot to
     * track using its unified motion-force controller, which allows doing force control in zero or
     * more Cartesian axes and motion control in the rest axes. The robot's internal motion
     * generator will smoothen the discrete commands.
     * @param[in] pose Target TCP pose in world frame: \f$ {^{O}T_{TCP}}_{d} \in \mathbb{R}^{7
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     * @param[in] wrench Target TCP wrench (force and moment) in the force control reference frame
     * (configured by setForceControlFrame()): \f$ ^{0}F_d \in \mathbb{R}^{6 \times 1} \f$. The
     * robot will track the target wrench using an explicit force controller. Consists of \f$
     * \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y,
     * f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
     * @param[in] maxLinearVel  Maximum Cartesian linear velocity when moving to the target pose.
     * Default maximum linear velocity is used when set to 0. Unit: \f$ [m/s] \f$.
     * @param[in] maxAngularVel  Maximum Cartesian angular velocity when moving to the target pose.
     * Default maximum angular velocity is used when set to 0. Unit: \f$ [rad/s] \f$.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @note Applicable control modes: NRT_CARTESIAN_MOTION_FORCE.
     * @par How to achieve pure motion control?
     * Use setForceControlAxis() to disable force control for all Cartesian axes to achieve pure
     * motion control. This function does pure motion control out of the box.
     * @par How to achieve pure force control?
     * Use setForceControlAxis() to enable force control for all Cartesian axes to achieve pure
     * force control, active or passive.
     * @par How to achieve unified motion-force control?
     * Use setForceControlAxis() to enable force control for one or more Cartesian axes and leave
     * the rest axes motion-controlled, then provide target pose for the motion-controlled axes and
     * target wrench for the force-controlled axes.
     * @see setCartesianStiffness(), setMaxContactWrench(), setNullSpacePosture(),
     * setForceControlAxis(), setForceControlFrame(), setPassiveForceControl().
     */
    void sendCartesianMotionForce(const std::array<double, k_poseSize>& pose,
        const std::array<double, k_cartDOF>& wrench = {}, double maxLinearVel = 0.0,
        double maxAngularVel = 0.0);

    /**
     * @brief [Non-blocking] Set motion stiffness for the Cartesian motion-force control modes.
     * @param[in] stiffness Cartesian motion stiffness: \f$ K_d \in \mathbb{R}^{6 \times 1} \f$.
     * Setting motion stiffness of a motion-controlled Cartesian axis to 0 will make this axis
     * free-floating. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear stiffness and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular stiffness: \f$ [k_x, k_y, k_z, k_{Rx}, k_{Ry}, k_{Rz}]^T
     * \f$. Unit: \f$ [N/m]~[Nm/rad] \f$.
     * @throw std::invalid_argument if input is invalid.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     * @warning The robot will automatically reset to its nominal stiffness upon re-entering the
     * applicable control modes.
     */
    void setCartesianStiffness(const std::array<double, k_cartDOF>& stiffness);

    /**
     * @brief [Non-blocking] Reset motion stiffness for the Cartesian motion-force control modes to
     * nominal value.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     */
    void resetCartesianStiffness(void);

    /**
     * @brief [Non-blocking] Set maximum contact wrench for the motion control part of the Cartesian
     * motion-force control modes. The controller will regulate its output to maintain contact
     * wrench (force and moment) with the environment under the set values.
     * @param[in] maxWrench Maximum contact wrench (force and moment): \f$ F_max \in \mathbb{R}^{6
     * \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ maximum force and \f$
     * \mathbb{R}^{3 \times 1} \f$ maximum moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit:
     * \f$ [N]~[Nm] \f$.
     * @throw std::invalid_argument if input is invalid.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @note The maximum contact wrench regulation only applies to the motion control part.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     * @warning The maximum contact wrench regulation will automatically reset to disabled upon
     * re-entering the applicable control modes.
     * @warning The maximum contact wrench regulation cannot be enabled if any of the rotational
     * Cartesian axes is enabled for moment control.
     */
    void setMaxContactWrench(const std::array<double, k_cartDOF>& maxWrench);

    /**
     * @brief [Non-blocking] Reset max contact wrench regulation to nominal state, i.e. disabled.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     */
    void resetMaxContactWrench(void);

    /**
     * @brief [Non-blocking] Set preferred joint positions for the null-space posture control module
     * used in the Cartesian motion-force control modes.
     * @param[in] preferredPositions Preferred joint positions for the null-space posture control:
     * \f$ q_{ns} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if input is invalid.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     * @warning The robot will automatically reset to its nominal preferred joint positions upon
     * re-entering the applicable control modes.
     * @par Null-space posture control
     * Similar to human arm, a robotic arm with redundant joint-space degree(s) of freedom (DOF > 6)
     * can change its overall posture without affecting the ongoing primary task. This is achieved
     * through a technique called "null-space control". After the preferred joint positions for a
     * desired robot posture is set using this function, the robot's null-space control module will
     * try to pull the arm as close to this posture as possible without affecting the primary
     * Cartesian motion-force control task.
     */
    void setNullSpacePosture(const std::array<double, k_jointDOF>& preferredPositions);

    /**
     * @brief [Non-blocking] Reset preferred joint positions to the robot's home posture.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     */
    void resetNullSpacePosture(void);

    /**
     * @brief [Blocking] Set force-controlled Cartesian axis(s) for the Cartesian motion-force
     * control modes. The axis(s) not enabled for force control will be motion controlled. This
     * function can only be called when the robot is in IDLE mode.
     * @param[in] enabledAxis Flags to enable/disable force control for certain Cartesian axis(s) in
     * the force control reference frame (configured by setForceControlFrame()). The corresponding
     * order is \f$ [X, Y, Z, Rx, Ry, Rz] \f$. By default, force control is disabled for all
     * Cartesian axes.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @note Applicable control modes: IDLE.
     * @warning This setting will reset to all axes disabled upon disconnection.
     * @warning This function blocks until the request is successfully executed.
     */
    void setForceControlAxis(const std::array<bool, k_cartDOF>& enabledAxis);

    /**
     * @brief [Blocking] Set force control reference frame for the Cartesian motion-force control
     * modes. This function can only be called when the robot is in IDLE mode.
     * @param[in] referenceFrame The reference frame to use for force control. Options are: "TCP"
     * and "WORLD". The target wrench and force control axis should also be expressed in the
     * selected reference frame. By default, world frame is used for force control.
     * @throw std::invalid_argument if input is invalid.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @note Applicable control modes: IDLE.
     * @warning This setting will reset to world frame upon disconnection.
     * @warning This function blocks until the request is successfully executed.
     * @par Force control reference frame
     * In Cartesian motion-force control modes, the reference frame of motion control is always the
     * world frame, but the reference frame of force control can be either world frame or the
     * robot's current TCP frame. While the world frame is the commonly used global coordinate,
     * the current TCP frame is a dynamic local coordinate whose transformation with regard to the
     * world frame changes as the robot TCP moves. When using world frame for force control, the
     * force-controlled axis(s) and motion-controlled axis(s) are guaranteed to be orthogonal.
     * However, when using current TCP frame for force control, the force-controlled axis(s) and
     * motion-controlled axis(s) are NOT guaranteed to be orthogonal because different reference
     * frames are used. In this case, it's recommended but not required to set the target pose such
     * that the actual robot motion direction(s) are orthogonal to force direction(s). If they are
     * not orthogonal, the motion control's vector component(s) in the force direction(s) will be
     * eliminated.
     */
    void setForceControlFrame(const std::string& referenceFrame);

    /**
     * @brief [Blocking] Enable or disable passive force control for the Cartesian motion-force
     * control modes. When enabled, an open-loop force controller will be used to feed forward the
     * target wrench, i.e. passive force control. When disabled, a closed-loop force controller will
     * be used to track the target wrench, i.e. active force control. This function can only be
     * called when the robot is in IDLE mode.
     * @param[in] isEnabled True: enable, false: disable. By default, passive force control is
     * disabled and active force control is used.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to execute the request.
     * @note Applicable control modes: IDLE.
     * @warning This setting will reset to disabled upon disconnection.
     * @warning This function blocks until the request is successfully executed.
     * @par Difference between active and passive force control
     * Active force control uses a feedback loop to reduce the error between target wrench and
     * measured wrench. This method results in better force tracking performance, but at the cost of
     * additional Cartesian damping which could potentially decrease motion tracking performance. On
     * the other hand, passive force control simply feeds forward the target wrench. This methods
     * results in worse force tracking performance, but is more robust and does not introduce
     * additional Cartesian damping. The choice of active or passive force control depends on the
     * actual application.
     */
    void setPassiveForceControl(bool isEnabled);

    //======================================== IO CONTROL ========================================
    /**
     * @brief [Blocking] Write to single or multiple digital output port(s) on the control box.
     * @param[in] portIdx Index of port(s) to write, can be a single port or multiple ports.
     * E.g. {0, 5, 7, 15} or {1, 3, 10} or {8}. Valid range of the index number is [0–15].
     * @param[in] values Corresponding values to write to the specified ports. True: set port high,
     * false: set port low. Vector size must match the size of portIdx.
     * @throw std::invalid_argument if any index number in portIdx is not within [0–15].
     * @throw std::length_error if the two input vectors have different sizes.
     * @throw std::runtime_error if failed to execute the request.
     * @warning This function blocks until the request is successfully executed.
     */
    void writeDigitalOutput(
        const std::vector<unsigned int>& portIdx, const std::vector<bool>& values);

    /**
     * @brief [Non-blocking] Read all digital input ports on the control box.
     * @return Digital input readings array whose index corresponds to the digital input port index.
     * True: port high, false: port low.
     */
    std::array<bool, k_IOPorts> readDigitalInput(void);

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;

    friend class Model;
    friend class Gripper;
    friend class Tool;
    friend class FileIO;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_ROBOT_HPP_ */
