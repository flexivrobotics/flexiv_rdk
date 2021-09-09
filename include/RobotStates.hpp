/**
 * @file RobotStates.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_ROBOTSTATES_HPP_
#define FLEXIVRDK_ROBOTSTATES_HPP_

#include <vector>
#include <string>

namespace flexiv {

/**
 * @struct SystemStatus
 * @brief Status of the system
 */
struct SystemStatus
{
    /**
     * @brief Emergency stop state
     * @note True: E-stop released, false: E-stop pressed
     */
    bool m_emergencyStop;

    /**
     * @brief External active state
     * @note True: robot executing user commands, false: robot waiting
     * for user commands
     */
    bool m_externalActive;

    /**
     * @brief If user can send program request
     * @note True: server can take new plan/primitive request from the client
     */
    bool m_programRequest;

    /**
     * @brief If a plan or primitive is currently running
     */
    bool m_programRunning;

    /**
     * @brief If the robot has reached the ready pose
     */
    bool m_reachedReady;

    /**
     * @brief If the robot has reached target pose
     */
    bool m_reachedTarget;

    /**
     * @brief Percentage of the last 1000 motion commands that were successfully
     * received by the server, for example, 91.2
     */
    double m_motionCmdSuccessRate;

    /**
     * @brief String containing error message from the server
     */
    std::string m_errorMsg;

    /**
     * @brief If joint limit is triggered
     */
    bool m_jntLimitTriggered;

    /**
     * @brief Primitive states information
     */
    std::vector<std::string> m_ptStates;
};

/**
 * @struct RobotStates
 * @brief Joint and cartesian-level robot states
 */
struct RobotStates
{
    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ measured link-side positions \f$
     * q~[rad] \f$
     */
    std::vector<double> m_linkPosition;

    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ measured link-side velocities
     * \f$ \dot{q}~[rad/s] \f$
     */
    std::vector<double> m_linkVelocity;

    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ measured link-side torques \f$
     * \tau_{J}~[Nm] \f$
     */
    std::vector<double> m_linkTorque;

    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ estimated external link-side
     * torques \f$ \hat \tau_{ext}~[Nm] \f$
     */
    std::vector<double> m_linkTorqueExt;

    /**
     * @brief \f$ \mathbb{R}^{7 \times 1} \f$ current TCP pose in base frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$
     */
    std::vector<double> m_tcpPose;

    /**
     * @brief \f$ \mathbb{R}^{6 \times 1} \f$ current TCP velocity in base frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular velocity \f$ [v_x, v_y, v_z,
     * \omega_x, \omega_y, \omega_z]^T \f$
     */
    std::vector<double> m_tcpVel;

    /**
     * @brief \f$ \mathbb{R}^{6 \times 1} \f$ current TCP acceleration in base
     * frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ linear acceleration and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular acceleration \f$ [a_x, a_y, a_z,
     * \alpha_x, \alpha_y, \alpha_z]^T \f$
     * @note To be deprecated
     */
    std::vector<double> m_tcpAcc;

    /**
     * @brief \f$ \mathbb{R}^{7 \times 1} \f$ current camera pose in base frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$
     */
    std::vector<double> m_camPose;

    /**
     * @brief \f$ \mathbb{R}^{7 \times 1} \f$ current flange pose in base frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$
     */
    std::vector<double> m_flangePose;

    /**
     * @brief \f$ \mathbb{R}^{7 \times 1} \f$ current pose of the last link
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$
     */
    std::vector<double> m_endLinkPose;

    /**
     * @brief \f$ \mathbb{R}^{6 \times 1} \f$ current external wrench applied on
     * tool in TCP frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times
     * 1} \f$ moment \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$
     */
    std::vector<double> m_tcpWrench;
};

/**
 * @struct PlanInfo
 * @brief Information on the execution of a primitive/plan
 */
struct PlanInfo
{
    /**
     * @brief Current primitive name
     */
    std::string m_ptName;

    /**
     * @brief Current node name
     */
    std::string m_nodeName;

    /**
     * @brief Current node path
     */
    std::string m_nodePath;

    /**
     * @brief Current node path time period
     */
    std::string m_nodePathTimePeriod;

    /**
     * @brief Current node path number
     */
    std::string m_nodePathNumber;

    /**
     * @brief Assigned plan name
     */
    std::string m_assignedPlanName;

    /**
     * @brief Velocity scale
     */
    double m_velocityScale;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_ROBOTSTATES_HPP_ */