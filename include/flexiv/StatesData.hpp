/**
 * @file StatesData.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_STATESDATA_HPP_
#define FLEXIVRDK_STATESDATA_HPP_

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
     * Measured link-side joint positions \f$ q~[rad] \f$.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> m_q = {};

    /**
     * Measured motor-side joint positions \f$ \theta~[rad] \f$.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> m_theta = {};

    /**
     * Measured link-side joint velocities \f$ \dot{q}~[rad/s] \f$.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> m_dq = {};

    /**
     * Measured motor-side joint velocities \f$ \dot{\theta}~[rad/s] \f$.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> m_dtheta = {};

    /**
     * Measured joint torques \f$ \tau~[Nm] \f$.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> m_tau = {};

    /**
     * Desired joint torques \f$ \tau_{d}~[Nm] \f$.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> m_tauDes = {};

    /**
     * Numerical derivative of joint torques \f$ \dot{\tau}~[Nm/s] \f$.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> m_tauDot = {};

    /**
     * Estimated external joint torques \f$ \hat \tau_{ext}~[Nm] \f$.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> m_tauExt = {};

    /**
     * Measured TCP pose in base frame \f$ ^{O}T_{TCP}~[m][] \f$.
     *
     * Size: \f$ \mathbb{R}^{7 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$.
     */
    std::vector<double> m_tcpPose = {};

    /**
     * Desired TCP pose in base frame \f$ {^{O}T_{TCP}}_{d}~[m][] \f$.
     *
     * Size: \f$ \mathbb{R}^{7 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$.
     */
    std::vector<double> m_tcpPoseDes = {};

    /**
     * Measured TCP velocity in base frame \f$ ^{O}\dot{X}~[m/s][rad/s] \f$.
     *
     * Size: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * linear velocity and \f$ \mathbb{R}^{3 \times 1} \f$ angular velocity \f$
     * [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$.
     */
    std::vector<double> m_tcpVel = {};

    /**
     * Measured camera pose in base frame \f$ ^{O}T_{cam}~[m][] \f$.
     *
     * Size: \f$ \mathbb{R}^{7 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$.
     */
    std::vector<double> m_camPose = {};

    /**
     * Measured flange pose in base frame \f$ ^{O}T_{fl}~[m][] \f$.
     *
     * Size: \f$ \mathbb{R}^{7 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$.
     */
    std::vector<double> m_flangePose = {};

    /**
     * Measured end link pose in base frame \f$ ^{O}T_{el}~[m][] \f$.
     *
     * Size: \f$ \mathbb{R}^{7 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$.
     */
    std::vector<double> m_endLinkPose = {};

    /**
     * Estimated external force applied on TCP in TCP frame
     * \f$ ^{TCP}F_{ext}~[N][Nm] \f$.
     *
     * Size: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * force and \f$ \mathbb{R}^{3 \times 1} \f$ moment \f$ [f_x, f_y, f_z, m_x,
     * m_y, m_z]^T \f$.
     */
    std::vector<double> m_extForceInTcpFrame = {};

    /**
     * Estimated external force vector applied on TCP in the base frame
     * \f$ ^{0}F_{ext}~[N][Nm] \f$.
     *
     * Size: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * force and \f$ \mathbb{R}^{3 \times 1} \f$ moment \f$ [f_x, f_y, f_z, m_x,
     * m_y, m_z]^T \f$.
     */
    std::vector<double> m_extForceInBaseFrame = {};
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
     * Emergency stop state
     * @note True: E-stop released, false: E-stop pressed
     */
    bool m_emergencyStop = {};

    /**
     * External active state
     * @note True: robot executing user commands, false: robot waiting
     * for user commands
     */
    bool m_externalActive = {};

    /**
     * If user can send program request
     * @note True: server can take new plan/primitive request from the client
     */
    bool m_programRequest = {};

    /**
     * If a plan is currently running
     */
    bool m_programRunning = {};

    /**
     * If the robot has reached target pose
     */
    bool m_reachedTarget = {};

    /**
     * If joint limit is triggered
     */
    bool m_jointLimitTriggered = {};
};

/**
 * @struct PlanInfo
 * @brief Data struct containing information of the on-going primitive/plan.
 */
struct PlanInfo
{
    /**
     * Current primitive name
     */
    std::string m_ptName = {};

    /**
     * Current node name
     */
    std::string m_nodeName = {};

    /**
     * Current node path
     */
    std::string m_nodePath = {};

    /**
     * Current node path time period
     */
    std::string m_nodePathTimePeriod = {};

    /**
     * Current node path number
     */
    std::string m_nodePathNumber = {};

    /**
     * Assigned plan name
     */
    std::string m_assignedPlanName = {};

    /**
     * Velocity scale
     */
    double m_velocityScale = {};
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_STATESDATA_HPP_ */
