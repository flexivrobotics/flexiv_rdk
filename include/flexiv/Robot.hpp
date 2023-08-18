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
     * @note True: released, false: pressed.
     */
    bool isEstopReleased(void) const;

    /**
     * @brief [Non-blocking] Check if the enabling button is pressed.
     * @note True: pressed, false: released.
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
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @throw InputException if index is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note isBusy() can be used to check if a plan task has finished.
     * @warning This function blocks until the request is successfully delivered
     * to the robot and fully processed.
     */
    void executePlan(unsigned int index);

    /**
     * @brief [Blocking] Execute a plan by specifying its name.
     * @param[in] name Name of the plan to execute, can be obtained via
     * getPlanNameList().
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @note isBusy() can be used to check if a plan task has finished.
     * @warning This function blocks until the request is successfully delivered
     * to the robot and fully processed.
     */
    void executePlan(const std::string& name);

    /**
     * @brief [Blocking] Pause or resume the execution of the current plan.
     * @param[in] pause True: pause plan, false: resume plan.
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
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
     * @note Applicable control mode: NRT_PRIMITIVE_EXECUTION.
     * @throw InputException if size of the input string is greater than 5kb.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
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
     * @throw CommException if there's no response from the robot.
     * @throw ExecutionException if error occurred during execution.
     * @warning This function blocks until the reply from the robot is received.
     */
    std::vector<std::string> getPrimitiveStates(void) const;

    /**
     * @brief [Blocking] Set global variables for the robot by specifying name
     * and value.
     * @param[in] globalVars Command to set global variables using the format:
     * globalVar1=value(s), globalVar2=value(s), ...
     * @note Applicable control mode: NRT_PLAN_EXECUTION.
     * @throw InputException if size of the input string is greater than 5kb.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
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
     * sending a new primitive command.
     * @warning This function blocks until the request is successfully delivered
     * to the robot.
     */
    void switchTcp(unsigned int index);

    /**
     * @brief [Blocking] Run automatic recovery to bring joints that are outside
     * the allowed position range back into allowed range.
     * @note Refer to user manual for more details.
     * @see isRecoveryState()
     * @throw ExecutionException if error occurred during execution.
     * @warning This function blocks until the automatic recovery process is
     * finished.
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
     * @note Applicable control mode: RT_JOINT_TORQUE.
     * @note Real-time (RT).
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
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
     * @note Applicable control mode: RT_JOINT_POSITION.
     * @note Real-time (RT).
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
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
     * @note Applicable control mode: NRT_JOINT_POSITION.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
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
     * command for the robot to track using its unified motion-force controller.
     * @param[in] pose Target TCP pose in base or TCP frame (depends on
     * control mode): \f$ {^{O}T_{TCP}}_{d} \f$ or \f$ {^{TCP}T_{TCP}}_{d} \in
     * \mathbb{R}^{7 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     * @param[in] wrench  Target TCP wrench in base or TCP frame (depends on
     * control mode): \f$ ^{0}F_d \f$ or \f$ ^{TCP}F_d \in \mathbb{R}^{6
     * \times 1} \f$. If TCP frame is used, unlike motion control, the reference
     * frame for force control is always the robot's current TCP frame. When the
     * target value of a direction is set to non-zero, this direction will
     * smoothly transit from motion control to force control, and the robot will
     * track the target force/moment in this direction using an explicit force
     * controller. When the target value is reset to 0, this direction will then
     * smoothly transit from force control back to motion control, and the robot
     * will gently move to the target motion point even if the set point is
     * distant. Calling with default parameter (all zeros) will result in pure
     * motion control in all directions. Consists of \f$ \mathbb{R}^{3 \times 1}
     * \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z,
     * m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
     * @note Applicable control modes: RT_CARTESIAN_MOTION_FORCE_BASE,
     * RT_CARTESIAN_MOTION_FORCE_TCP.
     * @note Real-time (RT).
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @warning Reference frame non-orthogonality between motion- and force-
     * controlled directions can happen when using the TCP frame mode
     * (RT_CARTESIAN_MOTION_FORCE_TCP). The reference frame for motion control
     * is defined as the robot TCP frame at the time point when the control
     * mode is switched into RT_CARTESIAN_MOTION_FORCE_TCP and is updated only
     * upon each mode entrance, since motion control requires a fixed reference
     * frame. The reference frame for force control is defined as the current
     * (latest) robot TCP frame, since force control does not require a fixed
     * reference frame. Such difference in frame definition means that, when
     * force control is enabled for one or more directions, the force-controlled
     * directions and motion-controlled directions are not guaranteed to stay
     * orthogonal to each other. When non-orthogonality happens, the affected
     * directions will see some control performance degradation. To avoid
     * reference frame non-orthogonality and retain maximum control performance,
     * it's recommended to keep the robot's Cartesian orientation unchanged when
     * running motion-force control in TCP frame mode. Note that the base frame
     * mode (RT_CARTESIAN_MOTION_FORCE_BASE) does not have such restriction.
     * @warning Always stream smooth and continuous commands to avoid sudden
     * movements.
     */
    void streamCartesianMotionForce(const std::vector<double>& pose,
        const std::vector<double>& wrench = std::vector<double>(6));

    /**
     * @brief [Non-blocking] Discretely send Cartesian motion and/or force
     * command for the robot to track using its unified motion-force controller.
     * The robot's internal motion generator will smoothen the discrete
     * commands.
     * @param[in] pose Target TCP pose in base or TCP frame (depends on
     * control mode): \f$ {^{O}T_{TCP}}_{d} \f$ or \f$ {^{TCP}T_{TCP}}_{d} \in
     * \mathbb{R}^{7 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     * @param[in] wrench  Target TCP wrench in base or TCP frame (depends on
     * control mode): \f$ ^{0}F_d \f$ or \f$ ^{TCP}F_d \in \mathbb{R}^{6
     * \times 1} \f$. If TCP frame is used, unlike motion control, the reference
     * frame for force control is always the robot's current TCP frame. When the
     * target value of a direction is set to non-zero, this direction will
     * smoothly transit from motion control to force control, and the robot will
     * track the target force/moment in this direction using an explicit force
     * controller. When the target value is reset to 0, this direction will then
     * smoothly transit from force control back to motion control, and the robot
     * will gently move to the target motion point even if the set point is
     * distant. Calling with default parameter (all zeros) will result in pure
     * motion control in all directions. Consists of \f$ \mathbb{R}^{3 \times 1}
     * \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z,
     * m_x, m_y, m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
     * @param[in] maxLinearVel Maximum Cartesian linear velocity when moving to
     * the target pose. Default maximum linear velocity is used when this
     * parameter is set to 0.
     * @param[in] maxAngularVel Maximum Cartesian angular velocity when moving
     * to the target pose. Default maximum angular velocity is used when this
     * parameter is set to 0.
     * @note Applicable control modes: NRT_CARTESIAN_MOTION_FORCE_BASE,
     * NRT_CARTESIAN_MOTION_FORCE_TCP.
     * @note Real-time (RT).
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @warning Reference frame non-orthogonality between motion- and force-
     * controlled directions can happen when using the TCP frame mode
     * (NRT_CARTESIAN_MOTION_FORCE_TCP). The reference frame for motion control
     * is defined as the robot TCP frame at the time point when the control
     * mode is switched into NRT_CARTESIAN_MOTION_FORCE_TCP and is updated only
     * upon each mode entrance, since motion control requires a fixed reference
     * frame. The reference frame for force control is defined as the current
     * (latest) robot TCP frame, since force control does not require a fixed
     * reference frame. Such difference in frame definition means that, when
     * force control is enabled for one or more directions, the force-controlled
     * directions and motion-controlled directions are not guaranteed to stay
     * orthogonal to each other. When non-orthogonality happens, the affected
     * directions will see some control performance degradation. To avoid
     * reference frame non-orthogonality and retain maximum control performance,
     * it's recommended to keep the robot's Cartesian orientation unchanged when
     * running motion-force control in TCP frame mode. Note that the base frame
     * mode (NRT_CARTESIAN_MOTION_FORCE_BASE) does not have such restriction.
     */
    void sendCartesianMotionForce(const std::vector<double>& pose,
        const std::vector<double>& wrench = std::vector<double>(6),
        double maxLinearVel = 0.0, double maxAngularVel = 0.0);

    /**
     * @brief [Non-blocking] Set motion stiffness for the Cartesian motion-force
     * control modes.
     * @param[in] stiffness Desired Cartesian motion stiffness: \f$ K_d \in
     * \mathbb{R}^{6 \times 1} \f$. Calling with default parameter (all zeros)
     * will reset to the robot's nominal stiffness. Consists of \f$
     * \mathbb{R}^{3 \times 1} \f$ linear stiffness and \f$ \mathbb{R}^{3 \times
     * 1} \f$ angular stiffness: \f$ [k_x, k_y, k_z, k_{Rx}, k_{Ry},
     * k_{Rz}]^T \f$. Unit: \f$ [N/m]~[Nm/rad] \f$.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE_BASE,
     * RT/NRT_CARTESIAN_MOTION_FORCE_TCP.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @warning The robot will automatically reset to its nominal stiffness upon
     * re-entering the applicable control modes.
     */
    void setCartesianStiffness(
        const std::vector<double>& stiffness = std::vector<double>(6));

    /**
     * @brief [Non-blocking] Set preferred joint positions for the null-space
     * posture control module used in the Cartesian motion-force control modes.
     * @param[in] preferredPositions Preferred joint positions for the
     * null-space posture control: \f$ q_{ns} \in \mathbb{R}^{DOF \times 1} \f$.
     * Calling with default parameter (all zeros) will reset to the robot's
     * nominal preferred joint positions, which is the home posture.
     * Unit: \f$ [rad] \f$.
     * @par Null-space posture control
     * Similar to human arm, a robotic arm with redundant degree(s) of
     * freedom (DOF > 6) can change its overall posture without affecting the
     * ongoing primary task. This is achieved through a technique called
     * "null-space control". After the preferred joint positions for a desired
     * robot posture is set using this function, the robot's null-space control
     * module will try to pull the arm as close to this posture as possible
     * without affecting the primary Cartesian motion-force control task.
     * @note Applicable control modes: RT/NRT_CARTESIAN_MOTION_FORCE_BASE,
     * RT/NRT_CARTESIAN_MOTION_FORCE_TCP.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct control mode.
     * @throw ExecutionException if error occurred during execution.
     * @warning The robot will automatically reset to its nominal preferred
     * joint positions upon re-entering the below applicable control modes.
     */
    void setNullSpacePosture(
        const std::vector<double>& preferredPositions = std::vector<double>(7));

    //=============================== IO CONTROL ===============================
    /**
     * @brief [Blocking] Write to single or multiple digital output port(s) on
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
