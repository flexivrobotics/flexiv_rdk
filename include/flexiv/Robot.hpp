/**
 * @file Robot.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_ROBOT_HPP_
#define FLEXIVRDK_ROBOT_HPP_

#include "StatesData.hpp"
#include "Mode.hpp"

#include <vector>
#include <memory>
#include <functional>

namespace flexiv {

/**
 * @class Robot
 * @brief Main interface with the robot, containing system, control, motion, and
 * IO methods. This interface is also responsible for communication.
 */
class Robot
{
public:
    /**
     * @brief Create a flexiv::Robot instance as the main robot interface. RDK
     * services will start and connection with robot server will be established.
     * @param[in] serverIP IP address of the robot server (remote).
     * @param[in] localIP IP address of the workstation PC (local).
     * @throw InitException if the instance failed to initialize.
     * @throw CompatibilityException if the RDK library version is incompatible
     * with robot server.
     * @throw CommException if the connection with robot server failed to
     * establish.
     */
    Robot(const std::string& serverIP, const std::string& localIP);
    virtual ~Robot();

    //============================= SYSTEM CONTROL =============================
    /**
     * @brief Enable the robot, if E-stop is released and there's no fault, the
     * robot will release brakes, and becomes operational a few seconds later.
     * @throw ExecutionException if error occurred during execution.
     * @throw CommException if connection with robot server is lost.
     */
    void enable(void);

    /**
     * @brief Stop the robot and transit robot mode to Idle.
     * @throw ExecutionException if error occurred during execution.
     */
    void stop(void);

    /**
     * @brief Check if the robot is normally operational, which requires the
     * following conditions to be met: enabled, brakes fully released, in auto
     * mode, no fault, and not in reduced state.
     * @warning The robot won't execute any user command until it becomes
     * normally operational.
     * @return True: operational, false: not operational.
     */
    bool isOperational(void) const;

    /**
     * @brief Check if the robot is currently executing a task. This includes
     * any user commanded operations that requires the robot to execute. For
     * example, plans, primitives, Cartesian and joint motions, etc.
     * @warning Some exceptions exist for primitives, see executePrimitive()
     * warning for more details.
     * @return True: busy, false: idle.
     */
    bool isBusy(void) const;

    /**
     * @brief Check if the connection with the robot server is established.
     * @return True: connected, false: disconnected.
     */
    bool isConnected(void) const;

    /**
     * @brief Check if the robot is in fault state.
     * @return True: robot has fault, false: robot normal.
     */
    bool isFault(void) const;

    /**
     * @brief Check if the Emergency Stop is released.
     * @note True: E-stop released, false: E-stop pressed
     */
    bool isEstopReleased(void) const;

    /**
     * @brief Check if the robot system is in recovery state.
     * @return True: in recovery state, false: not in recovery state.
     * @note Use startAutoRecovery() to carry out recovery operation.
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
     * @brief Try establishing connection with the robot server.
     * @throw CommException if failed to establish connection.
     */
    void connect(void);

    /**
     * @brief Disconnect with the robot server.
     * @throw ExecutionException if error occurred during execution.
     */
    void disconnect(void);

    /**
     * @brief Clear minor fault of the robot.
     * @throw ExecutionException if error occurred during execution.
     */
    void clearFault(void);

    /**
     * @brief Set robot mode, call it after initialization.
     * @param[in] mode Mode of the robot, see flexiv::Mode.
     * @warning To avoid unexpected behavior, it's recommended to call stop()
     * and then check if the robot has come to a complete stop using isStopped()
     * before switching mode.
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if requested mode is invalid.
     *
     */
    void setMode(Mode mode);

    /**
     * @brief Get the current robot mode.
     * @return Mode of the robot, see flexiv::Mode.
     */
    Mode getMode(void) const;

    //============================ ROBOT OPERATION =============================
    /**
     * @brief Get robot states like joint position, velocity, torque, TCP
     * pose, velocity, etc.
     * @param[out] output Reference of output data object.
     * @note Call this method periodically to get the latest states.
     */
    void getRobotStates(RobotStates& output);

    /**
     * @brief Execute a plan by specifying its index.
     * @param[in] index Index of the plan to execute, can be obtained via
     * getPlanNameList().
     * @throw InputException if index is invalid.
     * @throw LogicException if robot is not in the correct operation mode.
     * @throw ExecutionException if error occurred during execution.
     * @note isBusy() can be used to check if a plan task has finished.
     * @warning This method will block for 50 ms for the non-real-time command
     * to be transmitted and fully processed by the robot server.
     */
    void executePlanByIndex(unsigned int index);

    /**
     * @brief Execute a plan by specifying its name.
     * @param[in] name Name of the plan to execute, can be obtained via
     * getPlanNameList().
     * @throw LogicException if robot is not in the correct operation mode.
     * @throw ExecutionException if error occurred during execution.
     * @note isBusy() can be used to check if a plan task has finished.
     * @warning This method will block for 50 ms for the non-real-time command
     * to be transmitted and fully processed by the robot server.
     */
    void executePlanByName(const std::string& name);

    /**
     * @brief Get a list of all available plans.
     * @return Available plans in the format of a string list.
     * @throw CommException if there's no response from server.
     * @throw ExecutionException if error occurred during execution.
     * @warning This method will block until the request-reply operation with
     * the server is done. The blocking time varies by communication latency.
     */
    std::vector<std::string> getPlanNameList(void) const;

    /**
     * @brief Get detailed information about the currently running plan.
     * Contains information like plan name, primitive name, node name, node
     * path, node path time period, etc.
     * @param[out] output Reference of output data object.
     * @throw CommException if there's no response from server.
     * @throw ExecutionException if error occurred during execution.
     * @warning This method will block until the request-reply operation with
     * the server is done. The blocking time varies by communication latency.
     */
    void getPlanInfo(PlanInfo& output);

    /**
     * @brief Execute a primitive by specifying its name and parameters.
     * @param[in] ptCmd Primitive command with the format:
     * ptName(inputParam1=xxx, inputParam2=xxx, ...).
     * @throw InputException if size of the input string is greater than 5kb.
     * @throw LogicException if robot is not in the correct operation mode.
     * @throw ExecutionException if error occurred during execution.
     * @note A primitive won't terminate itself upon finish, thus isBusy()
     * cannot be used to check if a primitive task is finished, use primitive
     * states like "reachedTarget" instead.
     * @see getPrimitiveStates()
     * @warning The input parameters in ptCmd may not use SI units, please refer
     * to the detailed [Flexiv Primitives](https://www.flexiv.com/primitives/)
     * documentation.
     * @warning Some primitives may not terminate automatically and require
     * users to manually terminate them based on specific primitive states,
     * for example, most [Move] primitives. In such case, isBusy() will stay
     * true even if it seems everything is done for that primitive.
     * @warning This method will block for 50 ms for the non-real-time command
     * to be transmitted and fully processed by the robot server.
     */
    void executePrimitive(const std::string& ptCmd);

    /**
     * @brief Get feedback states of the executing primitive.
     * @return Primitive states in the format of a string list.
     * @throw CommException if there's no response from server.
     * @throw ExecutionException if error occurred during execution.
     * @warning This method will block until the request-reply operation with
     * the server is done. The blocking time varies by communication latency.
     */
    std::vector<std::string> getPrimitiveStates(void) const;

    /**
     * @brief Set global variables for the robot by specifying name and value.
     * @param[in] globalVars Command to set global variables using the format:
     * globalVar1=value(s), globalVar2=value(s), ...
     * @note Applicable mode: MODE_PLAN_EXECUTION.
     * @throw InputException if size of the input string is greater than 5kb.
     * @throw LogicException if robot is not in the correct operation mode.
     * @throw ExecutionException if error occurred during execution.
     * @warning The specified global variable(s) must have already been created
     * in the robot server, otherwise setting a nonexistent global variable will
     * have no effect. To check if a global variable is successfully set, use
     * getGlobalVariables().
     * @warning This method will block for 50 ms for the non-real-time command
     * to be transmitted and fully processed by the robot server.
     */
    void setGlobalVariables(const std::string& globalVars);

    /**
     * @brief Get available global variables from the robot.
     * @return Global variables in the format of a string list.
     * @throw CommException if there's no response from server.
     * @throw ExecutionException if error occurred during execution.
     * @warning This method will block until the request-reply operation with
     * the server is done. The blocking time varies by communication latency.
     */
    std::vector<std::string> getGlobalVariables(void) const;

    /**
     * @brief Check if the robot has come to a complete stop.
     * @return True: stopped, false: still moving.
     */
    bool isStopped(void) const;

    /**
     * @brief If the mounted tool has more than one TCP, switch the TCP being
     * used by the robot server. Default to the 1st one (index = 0).
     * @param[in] index Index of the TCP on the mounted tool to switch to.
     * @note No need to call this method if the mounted tool on the robot has
     * only one TCP, it'll be used by default.
     * @throw ExecutionException if error occurred during execution.
     */
    void switchTcp(unsigned int index);

    /**
     * @brief Start auto recovery to bring joints that are outside the allowed
     * position range back into allowed range.
     * @note Refer to user manual for more details.
     * @see isRecoveryState()
     * @throw ExecutionException if error occurred during execution.
     */
    void startAutoRecovery(void);

    //============================= MOTION CONTROL =============================
    /**
     * @brief Continuously send joint torque command to robot.
     * @param[in] torques \f$ \mathbb{R}^{Dof \times 1} \f$ target torques of
     * the joints: \f$ {\tau_J}_d~[Nm] \f$.
     * @param[in] enableGravityComp Enable/disable robot gravity compensation.
     * @param[in] enableSoftLimits Enable/disable soft limits to keep the
     * joints from moving outside the allowed position range, which will
     * trigger a safety fault that requires recovery operation.
     * @note Applicable mode: MODE_JOINT_TORQUE.
     * @note Real-time (RT).
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct operation mode.
     * @throw ExecutionException if error occurred during execution.
     * @warning Always send smooth and continuous commands to avoid sudden
     * movements.
     */
    void streamJointTorque(const std::vector<double>& torques,
        bool enableGravityComp = true, bool enableSoftLimits = true);

    /**
     * @brief Continuously send joint position, velocity, and acceleration
     * command.
     * @param[in] positions \f$ \mathbb{R}^{Dof \times 1} \f$ target positions
     * of the joints: \f$ q_d~[rad] \f$.
     * @param[in] velocities \f$ \mathbb{R}^{Dof \times 1} \f$ target velocities
     * of the joints: \f$ \dot{q}_d~[rad/s] \f$.
     * @param[in] accelerations \f$ \mathbb{R}^{Dof \times 1} \f$ target
     * accelerations of the joints: \f$ \ddot{q}_d~[rad/s^2] \f$.
     * @note Applicable mode: MODE_JOINT_POSITION.
     * @note Real-time (RT).
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct operation mode.
     * @throw ExecutionException if error occurred during execution.
     * @warning Always send smooth and continuous commands to avoid sudden
     * movements.
     */
    void streamJointPosition(const std::vector<double>& positions,
        const std::vector<double>& velocities,
        const std::vector<double>& accelerations);

    /**
     * @brief Discretely send joint position, velocity, and acceleration
     * command. The internal trajectory generator will interpolate between two
     * set points and make the motion smooth.
     * @param[in] positions \f$ \mathbb{R}^{Dof \times 1} \f$ target positions
     * of the joints: \f$ q_d~[rad] \f$.
     * @param[in] velocities \f$ \mathbb{R}^{Dof \times 1} \f$ target velocities
     * of the joints: \f$ \dot{q}_d~[rad/s] \f$.
     * @param[in] accelerations \f$ \mathbb{R}^{Dof \times 1} \f$ target
     * accelerations of the joints: \f$ \ddot{q}_d~[rad/s^2] \f$.
     * @param[in] maxVel \f$ \mathbb{R}^{Dof \times 1} \f$ maximum velocities
     * of the joints: \f$ \dot{q}_{max}~[rad/s] \f$.
     * @param[in] maxAcc \f$ \mathbb{R}^{Dof \times 1} \f$ maximum accelerations
     * of the joints: \f$ \ddot{q}_{max}~[rad/s^2] \f$.
     * @note Applicable mode: MODE_JOINT_POSITION_NRT.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct operation mode.
     * @throw ExecutionException if error occurred during execution.
     */
    void sendJointPosition(const std::vector<double>& positions,
        const std::vector<double>& velocities,
        const std::vector<double>& accelerations,
        const std::vector<double>& maxVel, const std::vector<double>& maxAcc);

    /**
     * @brief Continuously command target TCP pose for the robot to track using
     * its Cartesian impedance controller.
     * @param[in] pose \f$ \mathbb{R}^{7 \times 1} \f$ target TCP pose
     * in base frame, \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$
     * \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y,
     * q_z]^T~[m][] \f$.
     * @param[in] maxWrench \f$ \mathbb{R}^{6 \times 1} \f$ maximum
     * contact wrench in TCP coordinate. The controller will soften if needed to
     * keep the actual contact wrench under this value. Default value will be
     * used if not specified. \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y,
     * m_z]^T~[N][Nm] \f$.
     * @note Applicable mode: MODE_CARTESIAN_IMPEDANCE.
     * @note Real-time (RT).
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct operation mode.
     * @throw ExecutionException if error occurred during execution.
     * @warning Always send smooth and continuous commands to avoid sudden
     * movements.
     */
    void streamTcpPose(const std::vector<double>& pose,
        const std::vector<double>& maxWrench
        = {100.0, 100.0, 100.0, 30.0, 30.0, 30.0});

    /**
     * @brief Discretely command target TCP pose for the robot to track using
     * its Cartesian impedance controller. An internal motion generator will
     * smooth the discrete commands.
     * @param[in] pose \f$ \mathbb{R}^{7 \times 1} \f$ target TCP pose
     * in base frame, \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$
     * \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y,
     * q_z]^T~[m][] \f$.
     * @param[in] maxWrench \f$ \mathbb{R}^{6 \times 1} \f$ maximum
     * contact wrench in TCP coordinate. The controller will soften if needed to
     * keep the actual contact wrench under this value. Default value will be
     * used if not specified. \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y,
     * m_z]^T~[N][Nm] \f$.
     * @note Applicable mode: MODE_CARTESIAN_IMPEDANCE_NRT.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct operation mode.
     * @throw ExecutionException if error occurred during execution.
     */
    void sendTcpPose(const std::vector<double>& pose,
        const std::vector<double>& maxWrench
        = {100.0, 100.0, 100.0, 30.0, 30.0, 30.0});

    /**
     * @brief Set stiffness of Cartesian impedance controller.
     * @param[in] stiffness \f$ \mathbb{R}^{6 \times 1} \f$ diagonal elements of
     * the positive definite stiffness matrix. Maximum (nominal) stiffness is
     * provided as parameter default: \f$ [k_x, k_y, k_z, k_{Rx}, k_{Ry},
     * k_{Rz}]^T~[N/m][Nm/rad] \f$.
     * @note Applicable modes: MODE_CARTESIAN_IMPEDANCE,
     * MODE_CARTESIAN_IMPEDANCE_NRT.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct operation mode.
     * @throw ExecutionException if error occurred during execution.
     */
    void setCartesianStiffness(const std::vector<double>& stiffness
                               = {4000, 4000, 4000, 1900, 1900, 1900});

    /**
     * @brief Set preferred joint positions of Cartesian impedance controller,
     * for use in null-space posture control.
     * @param[in] preferredPositions \f$ \mathbb{R}^{Dof \times 1} \f$ preferred
     * joint positions for the null-space posture control. Nominal values (home
     * posture) are provided as parameter default: \f$ q_{nullspace}~[rad] \f$.
     * @par Null-space posture control
     * Similar to human arm, a robotic arm with redundant degree(s) of
     * freedom (DOF > 6) can change its overall posture without affecting the
     * ongoing primary task. This is achieved through a technique called
     * "null-space control". After the preferred joint positions for a desired
     * robot posture is set using this method, the robot's null-space controller
     * will try to pull the arm as close to this posture as possible without
     * affecting the primary TCP pose control task.
     * @note Applicable modes: MODE_CARTESIAN_IMPEDANCE,
     * MODE_CARTESIAN_IMPEDANCE_NRT.
     * @throw InputException if input is invalid.
     * @throw LogicException if robot is not in the correct operation mode.
     * @throw ExecutionException if error occurred during execution.
     */
    void setNullSpacePosture(const std::vector<double>& preferredPositions
                             = {0.0, -0.6981, 0.0, 1.5708, 0.0, 0.6981, 0.0});

    //=============================== IO CONTROL ===============================
    /**
     * @brief Set digital output on the control box.
     * @param[in] portNumber Port to set value to [0 ~ 15].
     * @param[in] value True: set high, false: set low.
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if input is invalid.
     */
    void writeDigitalOutput(unsigned int portNumber, bool value);

    /**
     * @brief Read digital input on the control box.
     * @param[in] portNumber Port to read value from [0 ~ 15].
     * @return True: port high, false: port low.
     * @throw CommException if there's no response from server.
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if input is invalid.
     */
    bool readDigitalInput(unsigned int portNumber);

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;

    friend class Model;
    friend class Gripper;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_ROBOT_HPP_ */
