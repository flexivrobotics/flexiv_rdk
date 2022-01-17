/**
 * @file RobotStates.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_ROBOTSTATES_HPP_
#define FLEXIVRDK_ROBOTSTATES_HPP_

#include <vector>
#include <string>
#include <ostream>

namespace flexiv {

/**
 * @struct RobotStates
 * @brief Data struct containing the joint- and Cartesian-space robot states.
 */
struct RobotStates
{
    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ measured link-side joint
     * positions \f$ q~[rad] \f$
     */
    std::vector<double> m_q;

    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ measured motor-side joint
     * positions \f$ \theta~[rad] \f$
     */
    std::vector<double> m_theta;

    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ measured link-side joint
     * velocities \f$ \dot{q}~[rad/s] \f$
     */
    std::vector<double> m_dq;

    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ measured motor-side joint
     * velocities \f$ \dot{\theta}~[rad/s] \f$
     */
    std::vector<double> m_dtheta;

    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ measured joint torques
     * \f$ \tau~[Nm] \f$
     */
    std::vector<double> m_tau;

    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ desired joint torques
     * \f$ \tau_{d}~[Nm] \f$
     */
    std::vector<double> m_tauDes;

    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ numerical derivative of joint
     * torques \f$ \dot{\tau}~[Nm/s] \f$
     */
    std::vector<double> m_tauDot;

    /**
     * @brief \f$ \mathbb{R}^{Dof \times 1} \f$ estimated external joint torques
     * \f$ \hat \tau_{ext}~[Nm] \f$
     */
    std::vector<double> m_tauExt;

    /**
     * @brief \f$ \mathbb{R}^{7 \times 1} \f$ measured TCP pose in base frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$
     */
    std::vector<double> m_tcpPose;

    /**
     * @brief \f$ \mathbb{R}^{7 \times 1} \f$ desired TCP pose in base frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$
     */
    std::vector<double> m_tcpPoseDes;

    /**
     * @brief \f$ \mathbb{R}^{6 \times 1} \f$ measured TCP velocity in base
     * frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$
     * \mathbb{R}^{3 \times 1} \f$ angular velocity \f$ [v_x, v_y, v_z,
     * \omega_x, \omega_y, \omega_z]^T \f$
     */
    std::vector<double> m_tcpVel;

    /**
     * @brief \f$ \mathbb{R}^{7 \times 1} \f$ measured camera pose in base frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$
     */
    std::vector<double> m_camPose;

    /**
     * @brief \f$ \mathbb{R}^{7 \times 1} \f$ measured flange pose in base frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$
     */
    std::vector<double> m_flangePose;

    /**
     * @brief \f$ \mathbb{R}^{7 \times 1} \f$ measured pose of the last link in
     * base frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4
     * \times 1} \f$ quaternion \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$
     */
    std::vector<double> m_endLinkPose;

    /**
     * @brief \f$ \mathbb{R}^{6 \times 1} \f$ estimated external force vector
     * applied on TCP in TCP frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times
     * 1} \f$ moment \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$
     */
    std::vector<double> m_extForceInTcpFrame;

    /**
     * @brief \f$ \mathbb{R}^{6 \times 1} \f$ estimated external force vector
     * applied on TCP in the base frame
     * @note \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times
     * 1} \f$ moment \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$
     */
    std::vector<double> m_extForceInBaseFrame;
};

/**
 * @brief Operator overloading to out stream all robot states in JSON format:
 * {"state_1": [val1,val2,val3,...], "state_2": [val1,val2,val3,...], ...}
 * @param[in] ostream Ostream instance
 * @param[in] robotStates RobotStates to out stream
 * @return Updated ostream instance
 */
std::ostream& operator<<(
    std::ostream& ostream, const flexiv::RobotStates& robotStates);

/**
 * @struct SystemStatus
 * @brief Data struct containing status of the robot system.
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
     * @brief If a plan is currently running
     */
    bool m_programRunning;

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
    bool m_jointLimitTriggered;

    /**
     * @brief Primitive states information
     */
    std::vector<std::string> m_ptStates;
};

/**
 * @struct PlanInfo
 * @brief Data struct containing information of the on-going primitive/plan.
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
