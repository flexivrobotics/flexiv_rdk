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
 * @brief Main interface with the robot, including system, control, motion, and
 * IO methods. Also responsible for communication.
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

    //============================ SYSTEM METHODS =============================
    /**
     * @brief Enable the robot, if E-stop is released and there's no fault, the
     * robot will release brakes, and becomes operational a few seconds later.
     * @note Call init() before calling this method.
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
     * @brief Check if the robot is normally operational.
     * @return True: operational, false: not operational.
     */
    bool isOperational(void) const;

    /**
     * @brief Check if the connection with the robot server is established.
     * @return True: connected, false: disconnected.
     */
    bool isConnected(void) const;

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
     * @brief Check if the connection with the robot server has timeout,
     * according to the heartbeat singal monitoring.
     * @return True: timeout, false: not timeout.
     */
    bool isTimeout(void) const;

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
     * @brief Check if the robot is in fault state.
     * @return True: robot has fault, false: robot normal.
     */
    bool isFault(void) const;

    /**
     * @brief Clear minor fault of the robot.
     * @throw ExecutionException if error occurred during execution.
     */
    void clearFault(void);

    //=========================== CONTROL METHODS ==============================
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

    /**
     * @brief Get robot states like joint position, velocity, torque, TCP
     * pose, velocity, etc.
     * @param[out] output Pointer to output data.
     * @note Call this method periodically to get the latest states.
     * @throw InputException if output is null.
     */
    void getRobotStates(RobotStates* output);

    /**
     * @brief Get system status data like E-stop status, program execution
     * status, etc.
     * @param[out] output Pointer to output data.
     * @throw InputException if output is null.
     */
    void getSystemStatus(SystemStatus* output);

    /**
     * @brief Execute a plan by specifying its index.
     * @param[in] index Index of the plan to execute, can be obtained via
     * getPlanNameList().
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if index is invalid.
     * @warning This method will block for 50 ms.
     */
    void executePlanByIndex(unsigned int index);

    /**
     * @brief Execute a plan by specifying its name.
     * @param[in] name Name of the plan to execute, can be obtained via
     * getPlanNameList().
     * @throw ExecutionException if error occurred during execution.
     * @warning This method will block for 50 ms.
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
     * @param[out] output Pointer to output data.
     * @throw CommException if there's no response from server.
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if output is null.
     * @warning This method will block until the request-reply operation with
     * the server is done. The blocking time varies by communication latency.
     */
    void getPlanInfo(PlanInfo* output);

    /**
     * @brief Execute a primitive by specifying its name and parameters.
     * @param[in] ptCmd Primitive command with the format:
     * ptName(inputParam1=xxx, inputParam2=xxx, ...).
     * @throw ExecutionException if error occurred during execution.
     * @warning This method will block for 50 ms.
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

    /**
     * @brief Set stiffness of Cartesian impedance controller.
     * @param[in] stiffness \f$ \mathbb{R}^{6 \times 1} \f$ diagonal elements of
     * the positive definite stiffness matrix. Defaulted to the nominal
     * stiffness.
     * @note Applicable modes: MODE_CARTESIAN_IMPEDANCE,
     * MODE_CARTESIAN_IMPEDANCE_NRT.
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if input is invalid.
     */
    void setCartesianStiffness(const std::vector<double>& stiffness
                               = {4000, 4000, 4000, 300, 300, 300});

    /**
     * @brief During Cartesian impedance modes, set preferred elbow swivel
     * angle, which is the angle between the arm plane and the reference plane.
     * @param[in] angle Swivel angle \f$ {\phi}~[rad] \f$, valid range:
     * [-2.0944, 2.0944] rad, i.e. [-120, 120] deg. Defaulted to the nominal
     * swivel angle.
     * @par Geometry definitions
     * Arm plane: defined by the origin of 3 body frames: link2(shoulder),
     * link4(elbow), and link6(wrist).
     * Reference plane: defined by the origin of 3 body frames: base,
     * link2(shoulder), and link6(wrist).
     * Positive direction: defined as from link2 origin to link6 origin, right
     * hand rule.
     * @note Applicable modes: MODE_CARTESIAN_IMPEDANCE,
     * MODE_CARTESIAN_IMPEDANCE_NRT.
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if input is invalid.
     */
    void setSwivelAngle(double angle = 0);

    //=========================== MOTION METHODS ===============================
    /**
     * @brief [Real-time] Continuously send joint torque command to robot.
     * @param[in] torques \f$ \mathbb{R}^{Dof \times 1} \f$ target torques of
     * the joints, \f$ {\tau_J}_d~[Nm] \f$.
     * @param[in] enableGravityComp Enable/disable robot gravity compensation.
     * @param[in] enableSoftLimits Enable/disable soft limits to keep the
     * joints from moving outside the allowed position range, which will
     * trigger a safety fault that requires recovery operation.
     * @note Applicable mode: MODE_JOINT_TORQUE.
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if input is invalid.
     * @warning Always send smooth and continuous commands to avoid sudden
     * movements.
     * @warning C++ only.
     */
    void streamJointTorque(const std::vector<double>& torques,
        bool enableGravityComp = true, bool enableSoftLimits = true);

    /**
     * @brief [Real-time] Continuously send joint position, velocity and
     * acceleration command.
     * @param[in] positions \f$ \mathbb{R}^{Dof \times 1} \f$ target positions
     * of the joints, \f$ q_d~[rad] \f$.
     * @param[in] velocities \f$ \mathbb{R}^{Dof \times 1} \f$ target velocities
     * of the joints, \f$ \dot{q}_d~[rad/s] \f$.
     * @param[in] accelerations \f$ \mathbb{R}^{Dof \times 1} \f$ target
     * accelerations of the joints, \f$ \ddot{q}_d~[rad/s^2] \f$.
     * @note Applicable mode: MODE_JOINT_POSITION.
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if input is invalid.
     * @warning Always send smooth and continuous commands to avoid sudden
     * movements.
     * @warning C++ only.
     */
    void streamJointPosition(const std::vector<double>& positions,
        const std::vector<double>& velocities,
        const std::vector<double>& accelerations);

    /**
     * @brief [Non-real-time] Discretely send joint position, velocity and
     * acceleration command. The internal trajectory generator will interpolate
     * between two set points and make the motion smooth.
     * @param[in] positions \f$ \mathbb{R}^{Dof \times 1} \f$ target positions
     * of the joints, \f$ q_d~[rad] \f$.
     * @param[in] velocities \f$ \mathbb{R}^{Dof \times 1} \f$ target velocities
     * of the joints, \f$ \dot{q}_d~[rad/s] \f$.
     * @param[in] accelerations \f$ \mathbb{R}^{Dof \times 1} \f$ target
     * accelerations of the joints, \f$ \ddot{q}_d~[rad/s^2] \f$.
     * @param[in] maxVel \f$ \mathbb{R}^{Dof \times 1} \f$ maximum velocities
     * of the joints, \f$ \dot{q}_{max}~[rad/s] \f$.
     * @param[in] maxAcc \f$ \mathbb{R}^{Dof \times 1} \f$ maximum accelerations
     * of the joints, \f$ \ddot{q}_{max}~[rad/s^2] \f$.
     * @param[in] maxJerk \f$ \mathbb{R}^{Dof \times 1} \f$ maximum jerk
     * of the joints, \f$ \dddot{q}_{max}~[rad/s^3] \f$.
     * @note Applicable mode: MODE_JOINT_POSITION_NRT.
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if input is invalid.
     */
    void sendJointPosition(const std::vector<double>& positions,
        const std::vector<double>& velocities,
        const std::vector<double>& accelerations,
        const std::vector<double>& maxVel, const std::vector<double>& maxAcc,
        const std::vector<double>& maxJerk);

    /**
     * @brief [Real-time] Continuously send TCP pose, velocity and acceleration
     * command.
     * @param[in] pose \f$ \mathbb{R}^{7 \times 1} \f$ target TCP pose
     * in base frame, \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$
     * \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y,
     * q_z]^T~[m][] \f$.
     * @param[in] velocity \f$ \mathbb{R}^{6 \times 1} \f$ target TCP velocity
     * in base frame, \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular velocity \f$ [v_x, v_y, v_z,
     * \omega_x, \omega_y, \omega_z]^T~[m/s][rad/s] \f$.
     * @param[in] acceleration \f$ \mathbb{R}^{6 \times 1} \f$ target TCP
     * acceleration in base frame, \f$ \mathbb{R}^{3 \times 1} \f$ linear
     * acceleration and \f$ \mathbb{R}^{3 \times 1} \f$ angular acceleration \f$
     * [a_x, a_y, a_z, \alpha_x, \alpha_y, \alpha_z]^T~[m/s^2][rad/s^2] \f$.
     * @note Applicable mode: MODE_CARTESIAN_IMPEDANCE.
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if input is invalid.
     * @warning Always send smooth and continuous commands to avoid sudden
     * movements.
     * @warning C++ only.
     */
    void streamTcpPose(const std::vector<double>& pose,
        const std::vector<double>& velocity,
        const std::vector<double>& acceleration);

    /**
     * @brief [Non-real-time] Discretely send TCP pose command with contact
     * force constraints. The internal trajectory generator will interpolate
     * between two set points and make the motion smooth.
     * @param[in] pose \f$ \mathbb{R}^{7 \times 1} \f$ target TCP pose
     * in base frame, \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$
     * \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y,
     * q_z]^T~[m][] \f$.
     * @param[in] maxWrench \f$ \mathbb{R}^{6 \times 1} \f$ maximum contact
     * wrench in TCP coordinate, \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment \f$ [f_x, f_y, f_z, m_x, m_y,
     * m_z]^T~[N][Nm] \f$.
     * @note Applicable mode: MODE_CARTESIAN_IMPEDANCE_NRT.
     * @throw ExecutionException if error occurred during execution.
     * @throw InputException if input is invalid.
     */
    void sendTcpPose(
        const std::vector<double>& pose, const std::vector<double>& maxWrench);

    /**
     * @brief Check if the robot has come to a complete stop.
     * @return True: stopped, false: still moving.
     */
    bool isStopped(void) const;

    //============================= IO METHODS =================================
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
