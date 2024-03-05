/**
 * @file Data.hpp
 * @brief Header file containing various data structs.
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_DATA_HPP_
#define FLEXIVRDK_DATA_HPP_

#include <array>
#include <string>
#include <ostream>

namespace flexiv {

/** Robot joint-space degrees of freedom \f$ n \f$ */
constexpr size_t k_jointDOF = 7;

/** Robot Cartesian-space degrees of freedom \f$ m \f$ */
constexpr size_t k_cartDOF = 6;

/** Size of pose array (3 position + 4 quaternion) */
constexpr size_t k_poseSize = 7;

/** Number of digital IO ports */
constexpr size_t k_IOPorts = 16;

/**
 * @struct RobotInfo
 * @brief General information of the connected robot.
 */
struct RobotInfo
{
    /** Robot serial number */
    std::string serialNum = {};

    /** Robot software version */
    std::string softwareVer = {};

    /**
     * Nominal motion stiffness of the Cartesian motion-force control modes: \f$ K_{nom} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear stiffness and
     * \f$ \mathbb{R}^{3 \times 1} \f$ angular stiffness: \f$ [k_x, k_y, k_z, k_{Rx}, k_{Ry},
     * k_{Rz}]^T \f$. Unit: \f$ [N/m]~[Nm/rad] \f$.
     */
    std::array<double, k_cartDOF> nominalK = {};

    /**
     * Lower limits of joint positions: \f$ q_{min} \in \mathbb{R}^{n \times 1} \f$.
     * Unit: \f$ [rad] \f$.
     */
    std::array<double, k_jointDOF> qMin = {};

    /**
     * Upper limits of joint positions: \f$ q_{max} \in \mathbb{R}^{n \times 1} \f$.
     * Unit: \f$ [rad] \f$.
     */
    std::array<double, k_jointDOF> qMax = {};

    /**
     * Upper limits of joint velocities: \f$ \dot{q}_{max} \in \mathbb{R}^{n \times 1} \f$.
     * Unit: \f$ [rad/s] \f$.
     */
    std::array<double, k_jointDOF> dqMax = {};

    /**
     * Upper limits of joint torques: \f$ \tau_{max} \in \mathbb{R}^{n \times 1} \f$.
     * Unit: \f$ [Nm] \f$.
     */
    std::array<double, k_jointDOF> tauMax = {};
};

/**
 * @struct RobotStates
 * @brief Data struct containing the joint- and Cartesian-space robot states.
 */
struct RobotStates
{
    /**
     * Measured joint positions using link-side encoder: \f$ q \in \mathbb{R}^{n \times 1} \f$.
     * This is the direct measurement of joint positions, preferred for most cases.
     * Unit: \f$ [rad] \f$.
     */
    std::array<double, k_jointDOF> q = {};

    /**
     * Measured joint positions using motor-side encoder: \f$ \theta \in \mathbb{R}^{n \times 1}
     * \f$. This is the indirect measurement of joint positions. \f$ \theta = q + \Delta \f$, where
     * \f$ \Delta \f$ is the joint's internal deflection between motor and link.
     * Unit: \f$ [rad] \f$.
     */
    std::array<double, k_jointDOF> theta = {};

    /**
     * Measured joint velocities using link-side encoder: \f$ \dot{q} \in \mathbb{R}^{n \times 1}
     * \f$. This is the direct but more noisy measurement of joint velocities.
     * Unit: \f$ [rad/s] \f$.
     */
    std::array<double, k_jointDOF> dq = {};

    /**
     * Measured joint velocities using motor-side encoder: \f$ \dot{\theta} \in \mathbb{R}^{n \times
     * 1} \f$. This is the indirect but less noisy measurement of joint velocities, preferred for
     * most cases. Unit: \f$ [rad/s] \f$.
     */
    std::array<double, k_jointDOF> dtheta = {};

    /**
     * Measured joint torques: \f$ \tau \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm] \f$.
     */
    std::array<double, k_jointDOF> tau = {};

    /**
     * Desired joint torques: \f$ \tau_{d} \in \mathbb{R}^{n \times 1} \f$. Compensation of
     * nonlinear dynamics (gravity, centrifugal, and Coriolis) is excluded. Unit: \f$ [Nm] \f$.
     */
    std::array<double, k_jointDOF> tauDes = {};

    /**
     * Numerical derivative of measured joint torques: \f$ \dot{\tau} \in \mathbb{R}^{n \times 1}
     * \f$. Unit: \f$ [Nm/s] \f$.
     */
    std::array<double, k_jointDOF> tauDot = {};

    /**
     * Estimated external joint torques: \f$ \hat \tau_{ext} \in \mathbb{R}^{n \times 1} \f$.
     * Produced by any external contact (with robot body or end-effector) that does not belong to
     * the known robot model. Unit: \f$ [Nm] \f$.
     */
    std::array<double, k_jointDOF> tauExt = {};

    /**
     * Measured TCP pose expressed in world frame: \f$ ^{O}T_{TCP} \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     */
    std::array<double, k_poseSize> tcpPose = {};

    /**
     * Desired TCP pose expressed in world frame: \f$ {^{O}T_{TCP}}_{d} \in \mathbb{R}^{7 \times 1}
     * \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     */
    std::array<double, k_poseSize> tcpPoseDes = {};

    /**
     * Measured TCP velocity expressed in world frame: \f$ ^{O}\dot{X} \in \mathbb{R}^{6 \times 1}
     * \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$ \mathbb{R}^{3 \times
     * 1} \f$ angular velocity: \f$ [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$.
     * Unit: \f$ [m/s]~[rad/s] \f$.
     */
    std::array<double, k_cartDOF> tcpVel = {};

    /**
     * Measured flange pose expressed in world frame: \f$ ^{O}T_{flange} \in \mathbb{R}^{7 \times 1}
     * \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     */
    std::array<double, k_poseSize> flangePose = {};

    /**
     * Force-torque (FT) sensor raw reading in flange frame: \f$ ^{flange}F_{raw} \in \mathbb{R}^{6
     * \times 1} \f$. The value is 0 if no FT sensor is installed. Consists of \f$ \mathbb{R}^{3
     * \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y,
     * m_z]^T \f$. Unit: \f$ [N]~[Nm] \f$.
     */
    std::array<double, k_cartDOF> ftSensorRaw = {};

    /**
     * Estimated external wrench applied on TCP and expressed in TCP frame: \f$ ^{TCP}F_{ext} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]~[Nm] \f$.
     */
    std::array<double, k_cartDOF> extWrenchInTcp = {};

    /**
     * Estimated external wrench applied on TCP and expressed in world frame: \f$ ^{0}F_{ext} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]~[Nm] \f$.
     */
    std::array<double, k_cartDOF> extWrenchInWorld = {};
};

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

    /** Waiting for user signal to step the breakpoint */
    bool waitingForStep = {};
};

/**
 * @struct GripperStates
 * @brief Data struct containing the gripper states.
 */
struct GripperStates
{
    /** Measured finger opening width [m] */
    double width = {};

    /** Measured finger force. Positive: opening force, negative: closing force.
     * Reads 0 if the mounted gripper has no force sensing capability [N] */
    double force = {};

    /** Maximum finger opening width of the mounted gripper [m] */
    double maxWidth = {};
};

/**
 * @struct ToolParams
 * @brief Data struct containing robot tool parameters.
 */
struct ToolParams
{
    /** Total mass. Unit: \f$ [kg] \f$ */
    double mass = 0.0;

    /** Center of mass in robot flange frame: \f$ [x, y, z] \f$. Unit: \f$ [m] \f$ */
    std::array<double, 3> CoM = {};

    /** Inertia at center of mass: \f$ [Ixx, Iyy, Izz, Ixy, Ixz, Iyz] \f$. Unit: \f$ [kg m^2] \f$ */
    std::array<double, 6> inertia = {};

    /** Position and orientation of the tool center point (TCP) in flange frame. Consists of \f$
     * \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x,
     * y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$ */
    std::array<double, k_poseSize> tcpLocation = {};
};

/**
 * @brief Operator overloading to out stream all robot info in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] robotInfo RobotInfo data struct to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const flexiv::RobotInfo& robotInfo);

/**
 * @brief Operator overloading to out stream all robot states in JSON format:
 * {"state_1": [val1,val2,val3,...], "state_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] robotStates RobotStates data struct to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const flexiv::RobotStates& robotStates);

/**
 * @brief Operator overloading to out stream all plan info in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] planInfo PlanInfo data struct to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const flexiv::PlanInfo& planInfo);

/**
 * @brief Operator overloading to out stream all gripper states in JSON format:
 * {"state_1": [val1,val2,val3,...], "state_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] gripperStates GripperStates data struct to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const flexiv::GripperStates& gripperStates);

} /* namespace flexiv */

#endif /* FLEXIVRDK_DATA_HPP_ */
