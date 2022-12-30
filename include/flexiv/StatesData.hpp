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
     * \f$ q~[rad] \f$: measured joint positions using link-side encoder. This
     * is the direct measurement of joint positions, preferred for most cases.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> q = {};

    /**
     * \f$ \theta~[rad] \f$: measured joint positions using motor-side encoder.
     * This is the indirect measurement of joint positions. \f$ \theta = q +
     * \Delta \f$, where \f$ \Delta \f$ is the joint's internal deflection
     * between motor and link.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> theta = {};

    /**
     * \f$ \dot{q}~[rad/s] \f$: measured joint velocities using link-side
     * encoder. This is the direct but more noisy measurement of joint
     * velocities.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> dq = {};

    /**
     * \f$ \dot{\theta}~[rad/s] \f$: measured joint velocities using motor-side
     * encoder. This is the indirect but less noisy measurement of joint
     * velocities, preferred for most cases.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> dtheta = {};

    /**
     * \f$ \tau~[Nm] \f$: measured joint torques.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> tau = {};

    /**
     * \f$ \tau_{d}~[Nm] \f$: desired joint torques excluding nonlinearity
     * compensation (gravity, centrifugal, and Coriolis).
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> tauDes = {};

    /**
     * \f$ \dot{\tau}~[Nm/s] \f$: numerical derivative of measured joint
     * torques.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> tauDot = {};

    /**
     * \f$ \hat \tau_{ext}~[Nm] \f$: estimated external joint torques, produced
     * by any external contact (with robot body or TCP) that does not belong to
     * the known robot model.
     *
     * Size: \f$ \mathbb{R}^{Dof \times 1} \f$.
     */
    std::vector<double> tauExt = {};

    /**
     * \f$ ^{O}T_{TCP}~[m][] \f$: measured TCP pose expressed in base frame,
     * consists of translation and rotation.
     *
     * Size: \f$ \mathbb{R}^{7 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$.
     */
    std::vector<double> tcpPose = {};

    /**
     * \f$ {^{O}T_{TCP}}_{d}~[m][] \f$: desired TCP pose expressed in base
     * frame, consists of translation and rotation.
     *
     * Size: \f$ \mathbb{R}^{7 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$.
     */
    std::vector<double> tcpPoseDes = {};

    /**
     * \f$ ^{O}\dot{X}~[m/s][rad/s] \f$: measured TCP velocity expressed in base
     * frame, consists of linear and angular velocity.
     *
     * Size: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * linear velocity and \f$ \mathbb{R}^{3 \times 1} \f$ angular velocity \f$
     * [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$.
     */
    std::vector<double> tcpVel = {};

    /**
     * \f$ ^{O}T_{cam}~[m][] \f$: measured camera pose expressed in base frame,
     * consists of translation and rotation.
     *
     * Size: \f$ \mathbb{R}^{7 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$.
     */
    std::vector<double> camPose = {};

    /**
     * \f$ ^{O}T_{fl}~[m][] \f$: measured flange pose expressed in base frame,
     * consists of translation and rotation.
     *
     * Size: \f$ \mathbb{R}^{7 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion \f$ [x, y, z,
     * q_w, q_x, q_y, q_z]^T \f$.
     */
    std::vector<double> flangePose = {};

    /**
     * \f$ ^{flange}F_{raw}~[N][Nm] \f$: force-torque (FT) sensor raw reading in
     * flange frame, consists of force (linear) and moment (angular). The value
     * is 0 if no FT sensor is installed.
     *
     * Size: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * force and \f$ \mathbb{R}^{3 \times 1} \f$ moment \f$ [f_x, f_y, f_z, m_x,
     * m_y, m_z]^T \f$.
     */
    std::vector<double> ftSensorRaw = {};

    /**
     * \f$ ^{TCP}F_{ext}~[N][Nm] \f$: estimated external wrench applied on TCP
     * and expressed in TCP frame, consists of force (linear) and moment
     * (angular).
     *
     * Size: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * force and \f$ \mathbb{R}^{3 \times 1} \f$ moment \f$ [f_x, f_y, f_z, m_x,
     * m_y, m_z]^T \f$.
     */
    std::vector<double> extWrenchInTcp = {};

    /**
     * \f$ ^{0}F_{ext}~[N][Nm] \f$: estimated external wrench applied on TCP and
     * expressed in base frame, consists of force (linear) and moment (angular).
     *
     * Size: \f$ \mathbb{R}^{6 \times 1} \f$ = \f$ \mathbb{R}^{3 \times 1} \f$
     * force and \f$ \mathbb{R}^{3 \times 1} \f$ moment \f$ [f_x, f_y, f_z, m_x,
     * m_y, m_z]^T \f$.
     */
    std::vector<double> extWrenchInBase = {};
};

/**
 * @brief Operator overloading to out stream all robot states in JSON format:
 * {"state_1": [val1,val2,val3,...], "state_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] robotStates RobotStates data struct to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(
    std::ostream& ostream, const flexiv::RobotStates& robotStates);

/**
 * @struct PlanInfo
 * @brief Data struct containing information of the on-going primitive/plan.
 */
struct PlanInfo
{
    /** Current primitive name */
    std::string ptName = {};

    /** Current node name */
    std::string nodeName = {};

    /** Current node path */
    std::string nodePath = {};

    /** Current node path time period */
    std::string nodePathTimePeriod = {};

    /** Current node path number */
    std::string nodePathNumber = {};

    /** Assigned plan name */
    std::string assignedPlanName = {};

    /** Velocity scale */
    double velocityScale = {};
};

/**
 * @brief Operator overloading to out stream all plan info in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] planInfo PlanInfo data struct to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(
    std::ostream& ostream, const flexiv::PlanInfo& planInfo);

/**
 * @struct GripperStates
 * @brief Data struct containing the gripper states.
 */
struct GripperStates
{
    /** Measured finger opening width [m] */
    double width = {};

    /** Measured finger force. Positive: opening force, negative: closing force.
     * 0 if the mounted gripper has no force sensing capability [N] */
    double force = {};

    /** Maximum finger opening width of the mounted gripper [m] */
    double maxWidth = {};

    /** Whether the fingers are moving */
    bool isMoving = {};
};

/**
 * @brief Operator overloading to out stream all gripper states in JSON format:
 * {"state_1": [val1,val2,val3,...], "state_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] gripperStates GripperStates data struct to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(
    std::ostream& ostream, const flexiv::GripperStates& gripperStates);

} /* namespace flexiv */

#endif /* FLEXIVRDK_STATESDATA_HPP_ */
