/**
 * @file Robot.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_ROBOT_HPP_
#define FLEXIVRDK_ROBOT_HPP_

#include <vector>
#include <memory>
#include <inttypes.h>
#include <functional>

#include "RobotStates.hpp"
#include "Mode.hpp"
#include "Model.hpp"

namespace flexiv {

class RobotClientHandler;

/**
 * @class Robot
 * @brief Main interface with the robot, including system, control, motion, and
 * IO methods.
 */
class Robot
{
public:
    Robot();
    virtual ~Robot();

    //============================ SYSTEM METHODS =============================
    /**
     * @brief Initialize robot client and establish connection with server
     * @param[in] serverAddr The IP address of RDK server (remote)
     * @param[in] localAddr The IP address of RDK client (local)
     * @return True: success, false: failed
     */
    bool init(const std::string& serverAddr, const std::string& localAddr);

    /**
     * @brief Load the model library from the robot
     * @param[out] model Pointer to the flexiv::Model object that gets updated
     * @param[in] gravityEarth Earth's gravity vector in base frame. Default
     * to \f$ [0.0, 0.0, -9.81]^T~[m/s^2] \f$
     * @return True: success, false: failed
     * @note Must call this method before using the flexiv::Model library
     */
    bool loadModel(Model* model,
        const Eigen::Vector3d& gravityEarth = Eigen::Vector3d(0.0, 0.0, -9.81));

    /**
     * @brief Enable the robot, if E-stop is released and there's no fault, the
     * robot will release brakes, and becomes operational a few seconds later
     * @return True: success, false: failed
     * @note Call init() before calling this method
     * @warning Only call this method when needed, avoid calling it periodically
     */
    bool enable(void);

    /**
     * @brief Start running user-defined high-priority control task at 1000 Hz
     * @param[in] callback Callback function with user code
     * @return True: success, false: failed
     * @note This method will block the thread from which this function is
     * called, so user needs to spawn all other background threads, if any,
     * before calling this method
     */
    bool start(std::function<void(void)>&& callback);

    /**
     * @brief Stop the robot
     * @return True: attempt successful, false: attempt failed
     * @note This will transit the robot mode to Idle
     * @warning It is recommended to call this method before switching from one
     * robot mode to another
     */
    bool stop(void);

    /**
     * @brief Check if the robot is normally operational
     * @return True: operational, false: not operational
     */
    bool isOperational(void) const;

    /**
     * @brief Check status of the connection with the robot server
     * @return True: connected, false: disconnected
     */
    bool isConnected(void) const;

    /**
     * @brief Check if the robot system is in recovery state
     * @return True: in recovery state, false: not in recovery state
     * @note The robot system will enter recovery state if it needs to recover
     * from joint position limit violation (a critical system fault that
     * requires a recovery operation, during which the joints that moved outside
     * the allowed position range will need to move very slowly back into the
     * allowed range). Refer to user manual for more details about system
     * recovery state.
     * @note Use startAutoRecovery() to carry out recovery operation
     */
    bool isRecoveryState(void) const;

    /**
     * @brief Check if the heartbeat signal between server and client has
     * timeout
     * @return True: timeout, false: not timeout
     */
    bool timeout(void) const;

    /**
     * @brief Try establishing connection with the server
     * @return True: success, false: failed
     */
    bool connect(void);

    /**
     * @brief Disconnect with the server
     */
    void disconnect(void);

    /**
     * @brief Check if the robot is in fault state
     * @return True: robot has fault, false: robot normal
     */
    bool isFault(void) const;

    /**
     * @brief Clear minor fault of the robot
     * @return True: success, false: failed
     */
    bool clearFault(void);

    //=========================== CONTROL METHODS ==============================
    /**
     * @brief Set robot mode, call it after initialization
     * @param[in] mode Mode of the robot, see flexiv::Mode
     * @return True: success, false: failed
     * @warning Only call this method when needed, avoid periodic calling. To
     * avoid unexpected behavior, it's recommended to call stop() and then check
     * if the robot has come to a complete stop using isStopped() before
     * switching robot's control mode
     */
    bool setMode(Mode mode);

    /**
     * @brief Get the current robot mode
     * @return Mode of the robot, see flexiv::Mode
     */
    Mode getMode(void) const;

    /**
     * @brief Get robot states like joint position, velocity, torque, TCP
     * pose, velocity, etc.
     * @param[out] robotStates RobotStates data struct
     * @return True: success, false: failed
     * @note Call this method in the periodic loop
     */
    bool getRobotStates(RobotStates* robotStates);

    /**
     * @brief Get system status data like E-stop status, program execution
     * status, etc.
     * @param[out] systemStatus SystemStatus data struct
     * @return True: success, false: failed
     */
    bool getSystemStatus(SystemStatus* systemStatus);

    /**
     * @brief Select the plan to execute using its index
     * @param[in] index Index of the plan to execute, can be obtained via
     * getPlanNameList()
     * @return True: success, false: failed
     */
    bool executePlanByIndex(int32_t index);

    /**
     * @brief Select the plan to execute using its name
     * @param[in] name Name of the plan to execute, can be obtained via
     * getPlanNameList()
     * @return True: success, false: failed
     */
    bool executePlanByName(const std::string& name);

    /**
     * @brief Get a list of all available plans
     * @param[out] planNameList String list of available plans
     * @return True: success, false: failed
     */
    bool getPlanNameList(std::vector<std::string>* planNameList);

    /**
     * @brief Get detailed information about the currently running plan
     * @param[out] planInfo PlanInfo struct
     * @return True: success, false: failed
     * @note Contains stuff like plan name, primitive name, node name, node
     * path, node path time period, etc.
     */
    bool getPlanInfo(PlanInfo* planInfo);

    /**
     * @brief Select a primitive to execute using its name and parameters
     * @param[in] ptCmd Primitive command with the format:
     * ptName(inputParam1=xxx, inputParam2=xxx, ...)
     * @return True: success, false: failed
     */
    bool executePrimitive(const std::string& ptCmd);

    /**
     * @brief If the mounted tool has more than one TCP, switch the TCP being
     * used by the robot server. Default to the 1st one (index = 0)
     * @param[in] index Index of the TCP on the mounted tool to switch to
     * @return True: success, false: failed
     * @note No need to call this method if the mounted tool on the robot has
     * only one TCP, it'll be used by default
     */
    bool switchTcp(int32_t index);

    /**
     * @brief Start auto recovery to bring joints that are outside the allowed
     * position range back into allowed range
     * @return True: success, false: failed
     * @note Refer to isRecoveryState() and user manual for more details.
     */
    bool startAutoRecovery(void);

    //=========================== MOTION METHODS ===============================
    /**
     * @brief [Realtime] Continuously send joint torque command to robot
     * @param[in] torques \f$ \mathbb{R}^{Dof \times 1} \f$ target torques of
     * all joints, \f$ {\tau_J}_d~[Nm] \f$
     * @param[in] enableGravityComp Enable/disable robot gravity compensation
     * @param[in] enableJointSoftLimits Enable/disable soft limits to keep the
     * joints from moving outside the allowed position range, which will
     * trigger a safety fault that requires recovery operation
     * @return True: success, false: failed
     * @note Set mode to MODE_JOINT_TORQUE, see flexiv::Mode
     * @warning Always send smooth and continuous commands to avoid sudden
     * movements
     */
    bool streamJointTorque(const std::vector<double>& torques,
        bool enableGravityComp = true, bool enableJointSoftLimits = true);

    /**
     * @brief [Realtime] Continuously send joint position, velocity and
     * acceleration command
     * @param[in] positions \f$ \mathbb{R}^{Dof \times 1} \f$ target positions
     * of all joints, \f$ q_d~[rad] \f$
     * @param[in] velocities \f$ \mathbb{R}^{Dof \times 1} \f$ target velocities
     * of all joints, \f$ \dot{q}_d~[rad/s] \f$
     * @param[in] accelerations \f$ \mathbb{R}^{Dof \times 1} \f$ target
     * accelerations of all joints, \f$ \ddot{q}_d~[rad/s^2] \f$
     * @return True: success, false: failed
     * @note Set mode to MODE_JOINT_POSITION, see flexiv::Mode
     * @warning Always send smooth and continuous commands to avoid sudden
     * movements
     */
    bool streamJointPosition(const std::vector<double>& positions,
        const std::vector<double>& velocities,
        const std::vector<double>& accelerations);

    /**
     * @brief [Realtime] Continuously send TCP pose, velocity and acceleration
     * command
     * @param[in] pose \f$ \mathbb{R}^{7 \times 1} \f$ target TCP pose
     * in base frame, \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$
     * \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y,
     * q_z]^T \f$
     * @param[in] velocity \f$ \mathbb{R}^{6 \times 1} \f$ target TCP velocity
     * in base frame, \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular velocity \f$ [v_x, v_y, v_z,
     * \omega_x, \omega_y, \omega_z]^T \f$
     * @param[in] acceleration \f$ \mathbb{R}^{6 \times 1} \f$ target TCP
     * acceleration in base frame, \f$ \mathbb{R}^{3 \times 1} \f$ linear
     * acceleration and \f$ \mathbb{R}^{3 \times 1} \f$ angular acceleration \f$
     * [a_x, a_y, a_z, \alpha_x, \alpha_y, \alpha_z]^T \f$
     * @return True: success, false: failed
     * @note Set mode to flexiv::Mode::MODE_CARTESIAN_IMPEDANCE, see
     * flexiv::Mode
     * @warning Always send smooth and continuous commands to avoid sudden
     * movements
     */
    bool streamTcpPose(const std::vector<double>& pose,
        const std::vector<double>& velocity,
        const std::vector<double>& acceleration);

    /**
     * @brief Check if the robot has come to a complete stop
     * @return True: stopped, false: still moving
     */
    bool isStopped(void) const;

    //============================= IO METHODS =================================
    /**
     * @brief Set digital output on the control box
     * @param[in] portNumber Port to set value to [0 ~ 15]
     * @param[in] value True: set high, false: set low
     * @return True: success, false: failed
     */
    bool writeDigitalOutput(int32_t portNumber, bool value);

    /**
     * @brief Read digital input on the control box
     * @param[in] portNumber Port to read value from [0 ~ 15]
     * @param[out] value True: port high, false: port low
     * @return True: success, false: failed
     */
    bool readDigitalInput(int32_t portNumber, bool* value);

private:
    std::unique_ptr<RobotClientHandler> m_handler;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_ROBOT_HPP_ */
