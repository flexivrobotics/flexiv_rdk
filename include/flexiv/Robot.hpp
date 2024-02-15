/**
 * @file Robot.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_ROBOT_HPP_
#define FLEXIVRDK_ROBOT_HPP_

#include "Data.hpp"
#include "Mode.hpp"

#include <array>
#include <vector>
#include <memory>
#include <functional>

namespace flexiv {

/**
 * @class Robot
 * @brief Main interface with the robot, containing several function categories
 * and background services.
 */
class Robot
{
public:
    /**
     * @brief [Blocking] Create a flexiv::Robot instance as the main robot
     * interface. RDK services will initialize and connection with the robot
     * will be established.
     * @param[in] serverIP IP address of the robot to connect (remote).
     * @param[in] localIP IP address of the workstation PC (local).
     * @throw InitException if the instance failed to initialize.
     * @throw CompatibilityException if the RDK library version is incompatible
     * with the connected robot.
     * @throw CommException if failed to establish connection with the robot.
     * @warning This constructor blocks until the initialization sequence is
     * successfully finished and connection with the robot is established.
     */
    Robot(const std::string& serverIP, const std::string& localIP);
    virtual ~Robot();

    /**
     * @brief [Non-blocking] Access general information of the robot.
     * @return RobotInfo struct.
     */
    const RobotInfo info(void);

    //============================= SYSTEM CONTROL =============================
    /**
     * @brief [Blocking] Enable the robot, if E-stop is released and there's no
     * fault, the robot will release brakes, and becomes operational a few
     * seconds later.
     * @throw ExecutionException if error occurred during execution.
     * @throw CommException if the robot is not connected.
     * @warning This function blocks until the request is successfully delivered
     * to the robot.
     */
    void enable(void);

    /**
     * @brief [Blocking] Stop the robot and transit robot mode to Idle.
     * @throw ExecutionException if error occurred during execution.
     * @warning This function blocks until the robot comes to a complete stop.
     */
    void stop(void);

    /**
     * @brief [Non-blocking] Check if the robot is normally operational, which
     * requires the following conditions to be met: enabled, brakes fully
     * released, in auto mode, no fault, and not in reduced state.
     * @param[in] verbose Whether to print warning message indicating why the
     * robot is not operational when this function returns false.
     * @return True: operational, false: not operational.
     * @warning The robot won't execute any user command until it becomes
     * normally operational.
     */
    bool isOperational(bool verbose = true) const;

    /**
     * @brief [Non-blocking] Check if the robot is currently executing a task.
     * This includes any user commanded operations that requires the robot to
     * execute. For example, plans, primitives, Cartesian and joint motions,
     * etc.
     * @return True: busy, false: idle.
     * @warning Some exceptions exist for primitives, see executePrimitive()
     * warning for more details.
     */
    bool isBusy(void) const;

    /**
     * @brief [Non-blocking] Check if the connection with the robot is
     * established.
     * @return True: connected, false: disconnected.
     */
    bool isConnected(void) const;

    /**
     * @brief [Non-blocking] Check if the robot is in fault state.
     * @return True: robot has fault, false: robot normal.
     */
    bool isFault(void) const;

    /**
     * @brief [Non-blocking] Check if the Emergency Stop is released.
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
     * The robot system will enter recovery state if it needs to recover
     * from joint position limit violation (a critical system fault that
     * requires a recovery operation, during which the joints that moved outside
     * the allowed position range will need to move very slowly back into the
     * allowed range). Refer to user manual for more details about system
     * recovery state.
     */
    bool isRecoveryState(void) const;

    /**
     * @brief [Blocking] Try establishing connection with the robot.
     * @throw CommException if failed to establish connection.
     * @warning This function blocks until the reconnection is finished.
     */
    void connect(void);

    /**
     * @brief [Blocking] Disconnect with the robot.
     * @throw ExecutionException if error occurred during execution.
     * @warning This function blocks until the disconnection is finished.
     */
    void disconnect(void);

    /**
     * @brief [Blocking] Clear minor fault of the robot.
     * @throw ExecutionException if error occurred during execution.
     * @warning This function blocks until the request is successfully delivered
     * to the robot.
     */
    void clearFault(void);

    /**
     * @brief [Blocking] Set a new control mode to the robot and wait until
     * the mode transition is finished.
     * @param[in] mode flexiv::Mode enum.
     * @throw InputException if requested mode is invalid.
     * @throw LogicException if robot is in an unknown control mode.
     * @throw ServerException if robot is not operational.
     * @throw ExecutionException if failed to transit the robot into specified
     * control mode after several attempts.
     * @warning If the robot is still moving when this function is called, it
     * will automatically stop then make the mode transition.
     * @warning This function blocks until the robot has successfully transited
     * into the specified control mode.
     */
    void setMode(Mode mode);

    /**
     * @brief [Non-blocking] Get the current control mode of the robot.
     * @return flexiv::Mode enum.
     */
    Mode getMode(void) const;

    //============================ ROBOT OPERATIONS ============================
    /**
     * @brief [Non-blocking] Get the latest robot states.
     * @param[out] output Reference of output data object.
     * @note Call this function periodically to keep the output data object up
     * to date.
     */
    void getRobotStates(RobotStates& output);

    /**
     * @brief [Blocking] Execute a plan by specifying its index.
     * @param[in] index Index of the plan to execute, can be obtained via
     * getPlanNameList().
     * @param[in] continueExec Whether to continue executing the plan when
     * the RDK program is closed or the connection is lost.
     * @throw InputException if index is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @note isBusy() can be used to check if a plan task has finished.
     * @warning This function blocks until the request is successfully delivered
     * to the robot and fully processed.
     */
    void executePlan(unsigned int index, bool continueExec = false);

    /**
     * @brief [Blocking] Execute a plan by specifying its name.
     * @param[in] name Name of the plan to execute, can be obtained via
     * getPlanNameList().
     * @param[in] continueExec Whether to continue executing the plan when
     * the RDK program is closed or the connection is lost.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @note isBusy() can be used to check if a plan task has finished.
     * @warning This function blocks until the request is successfully delivered
     * to the robot and fully processed.
     */
    void executePlan(const std::string& name, bool continueExec = false);

    /**
     * @brief [Blocking] Pause or resume the execution of the current plan.
     * @param[in] pause True: pause plan, false: resume plan.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @warning This function blocks until the request is successfully delivered
     * to the robot and fully processed.
     */
    void pausePlan(bool pause);

    /**
     * @brief [Blocking] Get a list of all available plans from the robot.
     * @return Available plans in the format of a string list.
     * @throw CommException if there's no response from the robot.
     * @throw ExecutionException if error occurred during execution.
     * @warning This function blocks until the reply from the robot is received.
     */
    std::vector<std::string> getPlanNameList(void) const;

    /**
     * @brief [Blocking] Get detailed information about the currently running
     * plan. Contains information like plan name, primitive name, node name,
     * node path, node path time period, etc.
     * @param[out] output Reference of output data object.
     * @throw CommException if there's no response from the robot.
     * @throw ExecutionException if error occurred during execution.
     * @warning This function blocks until the reply from the robot is received.
     */
    void getPlanInfo(PlanInfo& output);

    /**
     * @brief [Blocking] Execute a primitive by specifying its name and
     * parameters, which can be found in the [Flexiv Primitives
     * documentation](https://www.flexiv.com/primitives/).
     * @param[in] ptCmd Primitive command with the following string format:
     * "primitiveName(inputParam1=xxx, inputParam2=xxx, ...)".
     * @throw InputException if size of the input string is greater than 5kb.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control mode: NRT_PRIMITIVE_EXECUTION.
     * @warning The primitive input parameters may not use SI units, please
     * refer to the Flexiv Primitives documentation for exact unit definition.
     * @warning Some primitives may not terminate automatically and require
     * users to manually terminate them based on specific primitive states,
     * for example, most [Move] primitives. In such case, isBusy() will stay
     * true even if it seems everything is done for that primitive.
     * @warning This function blocks until the request is successfully delivered
     * to the robot and fully processed.
     */
    void executePrimitive(const std::string& ptCmd);

    /**
     * @brief [Blocking] Get feedback states of the currently executing
     * primitive.
     * @return Primitive states in the format of a string list.
     * @throw CommException if there's no response from the robot or the result
     * is invalid.
     * @throw ExecutionException if error occurred during execution.
     * @warning This function blocks until the reply from the robot is received.
     */
    std::vector<std::string> getPrimitiveStates(void) const;

    /**
     * @brief [Blocking] Set global variables for the robot by specifying name
     * and value.
     * @param[in] globalVars Command to set global variables using the format:
     * globalVar1=value(s), globalVar2=value(s), ...
     * @throw InputException if size of the input string is greater than 5kb.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @warning The specified global variable(s) must have already been created
     * in the robot using Flexiv Elements, otherwise setting a nonexistent
     * global variable will have no effect. To check if a global variable is
     * successfully set, use getGlobalVariables().
     * @warning This function blocks until the request is successfully delivered
     * to the robot and fully processed.
     */
    void setGlobalVariables(const std::string& globalVars);

    /**
     * @brief [Blocking] Get available global variables from the robot.
     * @return Global variables in the format of a string list.
     * @throw CommException if there's no response from the robot.
     * @throw ExecutionException if error occurred during execution.
     * @warning This function blocks until the reply from the robot is received.
     */
    std::vector<std::string> getGlobalVariables(void) const;

    /**
     * @brief [Non-blocking] Check if the robot has come to a complete stop.
     * @return True: stopped, false: still moving.
     */
    bool isStopped(void) const;

    /**
     * @brief [Blocking] If the mounted tool has more than one TCP, switch the
     * TCP being used by the robot. Default to the 1st one (index = 0).
     * @param[in] index Index of the TCP on the mounted tool to switch to.
     * @throw ExecutionException if error occurred during execution.
     * @note No need to call this function if the mounted tool on the robot has
     * only one TCP, it'll be used by default.
     * @note New TCP index will take effect upon control mode switch, or upon
     * sending a new primitive command. However, this function has no effect in
     * plan execution mode as TCP index should be defined in the plan itself.
     * @warning This function blocks until the request is successfully delivered
     * to the robot.
     */
    void switchTcp(unsigned int index);

    /**
     * @brief [Blocking] Run automatic recovery to bring joints that are outside
     * the allowed position range back into allowed range.
     * @throw ExecutionException if error occurred during execution.
     * @note Refer to user manual for more details.
     * @warning This function blocks until the automatic recovery process is
     * finished.
     * @see isRecoveryState()
     */
    void runAutoRecovery(void);

    //========================== DIRECT JOINT CONTROL ==========================
    /**
     * @brief [Non-blocking] Continuously stream joint torque command to the
     * robot.
     * @param[in] torques Target joint torques: \f$ {\tau_J}_d \in
     * \mathbb{R}^{DOF \times 1} \f$. Unit: \f$ [Nm] \f$.
     * @param[in] enableGravityComp Enable/disable robot gravity compensation.
     * @param[in] enableSoftLimits Enable/disable soft limits to keep the
     * joints from moving outside the allowed position range, which will
     * trigger a safety fault that requires recovery operation.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control mode: RT_JOINT_TORQUE.
     * @note Real-time (RT).
     * @warning Always stream smooth and continuous commands to avoid sudden
     * movements.
     */
    void streamJointTorque(const std::vector<double>& torques,
        bool enableGravityComp = true, bool enableSoftLimits = true);

    /**
     * @brief [Non-blocking] Continuously stream joint position, velocity, and
     * acceleration command to the robot.
     * @param[in] positions Target joint positions: \f$ q_d \in \mathbb{R}^{DOF
     * \times 1} \f$. Unit: \f$ [rad] \f$.
     * @param[in] velocities Target joint velocities: \f$ \dot{q}_d \in
     * \mathbb{R}^{DOF \times 1} \f$. Unit: \f$ [rad/s] \f$.
     * @param[in] accelerations Target joint accelerations: \f$ \ddot{q}_d \in
     * \mathbb{R}^{DOF \times 1} \f$. Unit: \f$ [rad/s^2] \f$.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control mode: RT_JOINT_POSITION.
     * @note Real-time (RT).
     * @warning Always stream smooth and continuous commands to avoid sudden
     * movements.
     */
    void streamJointPosition(const std::vector<double>& positions,
        const std::vector<double>& velocities,
        const std::vector<double>& accelerations);

    /**
     * @brief [Non-blocking] Discretely send joint position, velocity, and
     * acceleration command. The robot's internal motion generator will smoothen
     * the discrete commands.
     * @param[in] positions Target joint positions: \f$ q_d \in \mathbb{R}^{DOF
     * \times 1} \f$. Unit: \f$ [rad] \f$.
     * @param[in] velocities Target joint velocities: \f$ \dot{q}_d \in
     * \mathbb{R}^{DOF \times 1} \f$. Each joint will maintain this amount of
     * velocity when it reaches the target position. Unit: \f$ [rad/s] \f$.
     * @param[in] accelerations Target joint accelerations: \f$ \ddot{q}_d \in
     * \mathbb{R}^{DOF \times 1} \f$. Each joint will maintain this amount of
     * acceleration when it reaches the target position. Unit: \f$ [rad/s^2]
     * \f$.
     * @param[in] maxVel Maximum joint velocities for the planned trajectory:
     * \f$ \dot{q}_{max} \in \mathbb{R}^{DOF \times 1} \f$. Unit: \f$ [rad/s]
     * \f$.
     * @param[in] maxAcc Maximum joint accelerations for the planned trajectory:
     * \f$ \ddot{q}_{max} \in \mathbb{R}^{DOF \times 1} \f$. Unit: \f$ [rad/s^2]
     * \f$.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control mode: NRT_JOINT_POSITION.
     * @warning Calling this function a second time while the motion from the
     * previous call is still ongoing will trigger an online re-planning of
     * the joint trajectory, such that the previous command is aborted and the
     * new command starts to execute.
     */
    void sendJointPosition(const std::vector<double>& positions,
        const std::vector<double>& velocities,
        const std::vector<double>& accelerations,
        const std::vector<double>& maxVel, const std::vector<double>& maxAcc);

    //======================== DIRECT CARTESIAN CONTROL ========================
    /**
     * @brief [Non-blocking] Continuously stream Cartesian motion and/or force
     * command for the robot to track using its unified motion-force controller,
     * which allows doing force control in zero or more Cartesian axes and
     * motion control in the rest axes.
     * @param[in] pose Target TCP pose in base frame: \f$ {^{O}T_{TCP}}_{d} \in
     * \mathbb{R}^{7 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     * @param[in] wrench Target TCP wrench (force and moment) in the force
     * control reference frame (configured by setForceControlFrame()): \f$
     * ^{0}F_d \in \mathbb{R}^{6 \times 1} \f$. The robot will track the target
     * wrench using an explicit force controller. Consists of \f$ \mathbb{R}^{3
     * \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x,
     * f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE.
     * @note Real-time (RT).
     * @warning Always stream smooth and continuous motion commands to avoid
     * sudden movements. The force commands don't need to be continuous.
     * @par How to achieve pure motion control?
     * Use setForceControlAxis() to disable force control for all Cartesian axes
     * to achieve pure motion control. This function does pure motion control
     * out of the box.
     * @par How to achieve pure force control?
     * Use setForceControlAxis() to enable force control for all Cartesian axes
     * to achieve pure force control, active or passive.
     * @par How to achieve unified motion-force control?
     * Use setForceControlAxis() to enable force control for one or more
     * Cartesian axes and leave the rest axes motion-controlled, then provide
     * target pose for the motion-controlled axes and target wrench for the
     * force-controlled axes.
     * @see setCartesianStiffness(), setMaxContactWrench(),
     * setNullSpacePosture(), setForceControlAxis(), setForceControlFrame(),
     * setPassiveForceControl().
     */
    void streamCartesianMotionForce(const std::vector<double>& pose,
        const std::vector<double>& wrench = std::vector<double>(6));

    /**
     * @brief [Non-blocking] Discretely send Cartesian motion and/or force
     * command for the robot to track using its unified motion-force controller,
     * which allows doing force control in zero or more Cartesian axes and
     * motion control in the rest axes. The robot's internal motion generator
     * will smoothen the discrete commands.
     * @param[in] pose Target TCP pose in base frame: \f$ {^{O}T_{TCP}}_{d} \in
     * \mathbb{R}^{7 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     * @param[in] wrench Target TCP wrench (force and moment) in the force
     * control reference frame (configured by setForceControlFrame()): \f$
     * ^{0}F_d \in \mathbb{R}^{6 \times 1} \f$. The robot will track the target
     * wrench using an explicit force controller. Consists of \f$ \mathbb{R}^{3
     * \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x,
     * f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
     * @param[in] maxLinearVel  Maximum Cartesian linear velocity when moving to
     * the target pose. A safe value is provided as default. Unit: \f$ [m/s]
     * \f$.
     * @param[in] maxAngularVel  Maximum Cartesian angular velocity when moving
     * to the target pose. A safe value is provided as default. Unit: \f$
     * [rad/s] \f$.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control modes: NRT_CARTESIAN_MOTION_FORCE.
     * @par How to achieve pure motion control?
     * Use setForceControlAxis() to disable force control for all Cartesian axes
     * to achieve pure motion control. This function does pure motion control
     * out of the box.
     * @par How to achieve pure force control?
     * Use setForceControlAxis() to enable force control for all Cartesian axes
     * to achieve pure force control, active or passive.
     * @par How to achieve unified motion-force control?
     * Use setForceControlAxis() to enable force control for one or more
     * Cartesian axes and leave the rest axes motion-controlled, then provide
     * target pose for the motion-controlled axes and target wrench for the
     * force-controlled axes.
     * @see setCartesianStiffness(), setMaxContactWrench(),
     * setNullSpacePosture(), setForceControlAxis(), setForceControlFrame(),
     * setPassiveForceControl().
     */
    void sendCartesianMotionForce(const std::vector<double>& pose,
        const std::vector<double>& wrench = std::vector<double>(6),
        double maxLinearVel = 0.5, double maxAngularVel = 1.0);

    /**
     * @brief [Non-blocking] Set motion stiffness for the Cartesian motion-force
     * control modes.
     * @param[in] stiffness Cartesian motion stiffness: \f$ K_d \in
     * \mathbb{R}^{6 \times 1} \f$. Setting motion stiffness of a
     * motion-controlled Cartesian axis to 0 will make this axis free-floating.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear stiffness and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular stiffness: \f$ [k_x, k_y, k_z,
     * k_{Rx}, k_{Ry}, k_{Rz}]^T \f$. Valid range: 0 to RobotInfo::nominalK.
     * Unit: \f$ [N/m]~[Nm/rad] \f$.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     * @warning The robot will automatically reset to its nominal stiffness upon
     * re-entering the applicable control modes.
     */
    void setCartesianStiffness(const std::vector<double>& stiffness);

    /**
     * @brief [Non-blocking] Reset motion stiffness for the Cartesian
     * motion-force control modes to nominal value.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     */
    void resetCartesianStiffness(void);

    /**
     * @brief [Non-blocking] Set maximum contact wrench for the motion control
     * part of the Cartesian motion-force control modes. The controller will
     * regulate its output to maintain contact wrench (force and moment) with
     * the environment under the set values.
     * @param[in] maxWrench Maximum contact wrench (force and moment): \f$ F_max
     * \in \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1}
     * \f$ maximum force and \f$ \mathbb{R}^{3 \times 1} \f$ maximum moment: \f$
     * [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @note The maximum contact wrench regulation only applies to the motion
     * control part.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     * @warning The maximum contact wrench regulation will automatically reset
     * to disabled upon re-entering the applicable control modes.
     * @warning The maximum contact wrench regulation cannot be enabled if any
     * of the rotational Cartesian axes is enabled for moment control.
     */
    void setMaxContactWrench(const std::vector<double>& maxWrench);

    /**
     * @brief [Non-blocking] Reset max contact wrench regulation to nominal
     * state, i.e. disabled.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     */
    void resetMaxContactWrench(void);

    /**
     * @brief [Non-blocking] Set preferred joint positions for the null-space
     * posture control module used in the Cartesian motion-force control modes.
     * @param[in] preferredPositions Preferred joint positions for the
     * null-space posture control: \f$ q_{ns} \in \mathbb{R}^{DOF \times 1} \f$.
     * Unit: \f$ [rad] \f$.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     * @warning Upon entering the applicable control modes, the robot will
     * automatically set its current joint positions as the preferred joint
     * positions.
     * @par Null-space posture control
     * Similar to human arm, a robotic arm with redundant degree(s) of
     * freedom (DOF > 6) can change its overall posture without affecting the
     * ongoing primary task. This is achieved through a technique called
     * "null-space control". After the preferred joint positions for a desired
     * robot posture is set using this function, the robot's null-space control
     * module will try to pull the arm as close to this posture as possible
     * without affecting the primary Cartesian motion-force control task.
     */
    void setNullSpacePosture(const std::vector<double>& preferredPositions);

    /**
     * @brief [Non-blocking] Reset preferred joint positions to the ones
     * automatically recorded when entering the applicable control modes.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE.
     */
    void resetNullSpacePosture(void);

    /**
     * @brief [Blocking] Set force-controlled Cartesian axis(s) for the
     * Cartesian motion-force control modes. The axis(s) not enabled for force
     * control will be motion controlled. This function can only be called when
     * the robot is in IDLE mode.
     * @param[in] enabledAxis Flags to enable/disable force control for certain
     * Cartesian axis(s) in the force control reference frame (configured by
     * setForceControlFrame()). The corresponding order is \f$ [X, Y, Z, Rx, Ry,
     * Rz] \f$. By default, force control is disabled for all Cartesian axes.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if failed to execute the request.
     * @note Applicable control modes: IDLE.
     * @warning This setting will reset to all axes disabled upon disconnection.
     * @warning This function blocks until the request is successfully delivered
     * to the robot.
     */
    void setForceControlAxis(const std::vector<bool>& enabledAxis);

    /**
     * @brief [Blocking] Set force control reference frame for the Cartesian
     * motion-force control modes. This function can only be called when the
     * robot is in IDLE mode.
     * @param[in] referenceFrame The reference frame to use for force control.
     * Options are: "TCP" and "BASE". The target wrench and force control axis
     * should also be expressed in the selected reference frame. By default,
     * base frame is used for force control.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if failed to execute the request.
     * @note Applicable control modes: IDLE.
     * @warning This setting will reset to robot base frame upon disconnection.
     * @warning This function blocks until the request is successfully delivered
     * to the robot.
     * @par Force control reference frame
     * In Cartesian motion-force control modes, the reference frame of motion
     * control is always the robot base frame, but the reference frame of force
     * control can be either robot base frame or the robot's current TCP frame.
     * While the robot base frame is the commonly used global coordinate, the
     * current TCP frame is a dynamic local coordinate whose transformation with
     * regard to the robot base frame changes as the robot TCP moves. When using
     * robot base frame for force control, the force-controlled axis(s) and
     * motion-controlled axis(s) are guaranteed to be orthogonal. However, when
     * using current TCP frame for force control, the force-controlled axis(s)
     * and motion-controlled axis(s) are NOT guaranteed to be orthogonal because
     * different reference frames are used. In this case, it's recommended but
     * not required to set the target pose such that the actual robot motion
     * direction(s) are orthogonal to force direction(s). If they are not
     * orthogonal, the motion control's vector component(s) in the force
     * direction(s) will be eliminated.
     */
    void setForceControlFrame(const std::string& referenceFrame);

    /**
     * @brief [Blocking] Enable or disable passive force control for the
     * Cartesian motion-force control modes. When enabled, an open-loop force
     * controller will be used to feed forward the target wrench, i.e. passive
     * force control. When disabled, a closed-loop force controller will be used
     * to track the target wrench, i.e. active force control. This function can
     * only be called when the robot is in IDLE mode.
     * @param[in] isEnabled True: enable, false: disable. By default, passive
     * force control is disabled and active force control is used.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if failed to execute the request.
     * @note Applicable control modes: IDLE.
     * @warning This setting will reset to disabled upon disconnection.
     * @warning This function blocks until the request is successfully delivered
     * to the robot.
     * @par Difference between active and passive force control
     * Active force control uses a feedback loop to reduce the error between
     * target wrench and measured wrench. This method results in better force
     * tracking performance, but at the cost of additional Cartesian damping
     * which could potentially decrease motion tracking performance. On the
     * other hand, passive force control simply feeds forward the target wrench.
     * This methods results in worse force tracking performance, but is more
     * robust and does not introduce additional Cartesian damping. The choice of
     * active or passive force control depends on the actual application.
     */
    void setPassiveForceControl(bool isEnabled);

    //=============================== IO CONTROL ===============================
    /**
     * @brief [Blocking] Write to a single or multiple digital output port(s) on
     * the control box.
     * @param[in] portIdx Index of port(s) to write, can be a single port or
     * multiple ports. E.g. {0, 5, 7, 15} or {1, 3, 10} or {8}. Valid range of
     * the index number is [0–15].
     * @param[in] values Corresponding values to write to the specified ports.
     * True: set port high, false: set port low. Vector size must match the size
     * of portIdx.
     * @throw InputException if any index number in portIdx is not within
     * [0–15], or if the two input vectors have different sizes.
     * @throw ExecutionException if failed to execute the request.
     * @warning This function blocks until the request is successfully delivered
     * to the robot.
     */
    void writeDigitalOutput(const std::vector<unsigned int>& portIdx,
        const std::vector<bool>& values);

    /**
     * @brief [Non-blocking] Read all digital input ports on the control box.
     * @return Digital input readings array whose index corresponds to the
     * digital input port index. True: port high, false: port low.
     */
    std::array<bool, k_IOPorts> readDigitalInput(void);

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;

    friend class Model;
    friend class Gripper;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_ROBOT_HPP_ */
