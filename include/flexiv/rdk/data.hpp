/**
 * @file data.hpp
 * @brief Header file containing various constant expressions, data structs, and enums.
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_DATA_HPP_
#define FLEXIV_RDK_DATA_HPP_

#include <array>
#include <vector>
#include <string>
#include <ostream>
#include <variant>

namespace flexiv {
namespace rdk {
/** Cartesian-space degrees of freedom */
constexpr size_t kCartDoF = 6;

/** Joint-space degrees of freedom of Flexiv's serial robots */
constexpr size_t kSerialJointDoF = 7;

/** Size of pose array (3 position + 4 quaternion) */
constexpr size_t kPoseSize = 7;

/** Number of digital IO ports */
constexpr size_t kIOPorts = 16;

/** Maximum number of external axes */
constexpr size_t kMaxExtAxes = 6;

/**
 * @brief Operational status of the robot. Except for the first two, the other enumerators
 * indicate the cause of the robot being not ready to operate.
 * @see Robot::operational_status().
 */
enum class OperationalStatus
{
    UNKNOWN = 0,        ///< Unkown status.
    READY,              ///< Ready to be operated.
    BOOTING,            ///< System still booting, please wait.
    ESTOP_NOT_RELEASED, ///< E-Stop is not released.
    NOT_ENABLED,        ///< Not enabled, call Enable() to send the signal.
    RELEASING_BRAKE,    ///< Brake release in progress, please wait.
    MINOR_FAULT,        ///< Minor fault occurred, call ClearFault() to try clearing it.
    CRITICAL_FAULT,     ///< Critical fault occurred, call ClearFault() to try clearing it.
    IN_REDUCED_STATE,   ///< In reduced state, see reduced().
    IN_RECOVERY_STATE,  ///< In recovery state, see recovery().
    IN_MANUAL_MODE,     ///< In Manual mode, need to switch to Auto (Remote) mode.
    IN_AUTO_MODE,       ///< In regular Auto mode, need to switch to Auto (Remote) mode.
};


/**
 * @struct RobotInfo
 * @brief General information about the connected robot.
 * @see Robot::info().
 */
struct RobotInfo
{
    /** Robot serial number. */
    std::string serial_num = {};

    /** Robot software version. */
    std::string software_ver = {};

    /** Robot model name, e.g. Rizon4, Rizon10, Moonlight, etc. */
    std::string model_name = {};

    /** Type of license */
    std::string license_type = {};

    /** Joint-space degrees of freedom: \f$ n \f$. */
    size_t DoF = {};

    /**
     * Nominal motion stiffness of the Cartesian motion-force control modes: \f$ K_x^{nom} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear stiffness and
     * \f$ \mathbb{R}^{3 \times 1} \f$ angular stiffness: \f$ [k_x, k_y, k_z, k_{Rx}, k_{Ry},
     * k_{Rz}]^T \f$. Unit: \f$ [N/m]:[Nm/rad] \f$.
     */
    std::array<double, kCartDoF> K_x_nom = {};

    /**
     * Nominal motion stiffness of the joint impedance control modes: \f$ K_q^{nom} \in
     * \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm/rad] \f$.
     */
    std::vector<double> K_q_nom = {};

    /**
     * Lower limits of joint positions: \f$ q_{min} \in \mathbb{R}^{n \times 1} \f$.
     * Unit: \f$ [rad] \f$.
     */
    std::vector<double> q_min = {};

    /**
     * Upper limits of joint positions: \f$ q_{max} \in \mathbb{R}^{n \times 1} \f$.
     * Unit: \f$ [rad] \f$.
     */
    std::vector<double> q_max = {};

    /**
     * Upper limits of joint velocities: \f$ \dot{q}_{max} \in \mathbb{R}^{n \times 1} \f$.
     * Unit: \f$ [rad/s] \f$.
     */
    std::vector<double> dq_max = {};

    /**
     * Upper limits of joint torques: \f$ \tau_{max} \in \mathbb{R}^{n \times 1} \f$.
     * Unit: \f$ [Nm] \f$.
     */
    std::vector<double> tau_max = {};
};

/**
 * @struct RobotStates
 * @brief Data structure containing the joint- and Cartesian-space robot states.
 * @see Robot::states().
 */
struct RobotStates
{
    /**
     * Measured joint positions of the arm using link-side encoder: \f$ q \in \mathbb{R}^{n \times
     * 1} \f$. This is the direct measurement of joint positions, preferred for most cases. Unit:
     * \f$ [rad] \f$.
     */
    std::vector<double> q = {};

    /**
     * Measured joint positions of the arm using motor-side encoder: \f$ \theta \in \mathbb{R}^{n
     * \times 1} \f$. This is the indirect measurement of joint positions. \f$ \theta = q + \Delta
     * \f$, where \f$ \Delta \f$ is the joint's internal deflection between motor and link. Unit:
     * \f$ [rad] \f$.
     */
    std::vector<double> theta = {};

    /**
     * Measured joint velocities of the arm using link-side encoder: \f$ \dot{q} \in \mathbb{R}^{n
     * \times 1} \f$. This is the direct but more noisy measurement of joint velocities. Unit: \f$
     * [rad/s] \f$.
     */
    std::vector<double> dq = {};

    /**
     * Measured joint velocities of the arm using motor-side encoder: \f$ \dot{\theta} \in
     * \mathbb{R}^{n \times 1} \f$. This is the indirect but less noisy measurement of joint
     * velocities, preferred for most cases. Unit: \f$ [rad/s] \f$.
     */
    std::vector<double> dtheta = {};

    /**
     * Measured joint torques of the arm: \f$ \tau \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm]
     * \f$.
     */
    std::vector<double> tau = {};

    /**
     * Desired joint torques of the arm: \f$ \tau_{d} \in \mathbb{R}^{n \times 1} \f$. Compensation
     * of nonlinear dynamics (gravity, centrifugal, and Coriolis) is excluded. Unit: \f$ [Nm] \f$.
     */
    std::vector<double> tau_des = {};

    /**
     * Numerical derivative of measured joint torques of the arm: \f$ \dot{\tau} \in \mathbb{R}^{n
     * \times 1} \f$. Unit: \f$ [Nm/s] \f$.
     */
    std::vector<double> tau_dot = {};

    /**
     * Estimated external joint torques of the arm: \f$ \hat \tau_{ext} \in \mathbb{R}^{n \times 1}
     * \f$. Produced by any external contact (with robot body or end-effector) that does not belong
     * to the known robot model. Unit: \f$ [Nm] \f$.
     */
    std::vector<double> tau_ext = {};

    /**
     * Measured joint positions of the external axes (if any): \f$ q_e \in \mathbb{R}^{n_e \times 1}
     * \f$. Unit: \f$ [rad] \f$.
     */
    std::vector<double> q_e = {};

    /**
     * Measured joint velocities of the external axes (if any): \f$ \dot{q}_e \in \mathbb{R}^{n_e
     * \times 1} \f$. Unit: \f$ [rad/s] \f$.
     */
    std::vector<double> dq_e = {};

    /**
     * Measured joint torques of the external axes (if any): \f$ \tau_e \in \mathbb{R}^{n_e \times
     * 1} \f$. Unit: \f$ [Nm] \f$.
     */
    std::vector<double> tau_e = {};

    /**
     * Measured TCP pose expressed in world frame: \f$ ^{O}T_{TCP} \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     */
    std::array<double, kPoseSize> tcp_pose = {};

    /**
     * Desired TCP pose expressed in world frame: \f$ {^{O}T_{TCP}}_{d} \in \mathbb{R}^{7 \times 1}
     * \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     */
    std::array<double, kPoseSize> tcp_pose_des = {};

    /**
     * Measured TCP velocity expressed in world frame: \f$ ^{O}\dot{X} \in \mathbb{R}^{6 \times 1}
     * \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$ \mathbb{R}^{3 \times
     * 1} \f$ angular velocity: \f$ [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$.
     * Unit: \f$ [m/s]:[rad/s] \f$.
     */
    std::array<double, kCartDoF> tcp_vel = {};

    /**
     * Measured flange pose expressed in world frame: \f$ ^{O}T_{flange} \in \mathbb{R}^{7 \times 1}
     * \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     */
    std::array<double, kPoseSize> flange_pose = {};

    /**
     * Force-torque (FT) sensor raw reading in flange frame: \f$ ^{flange}F_{raw} \in \mathbb{R}^{6
     * \times 1} \f$. The value is 0 if no FT sensor is installed. Consists of \f$ \mathbb{R}^{3
     * \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y,
     * m_z]^T \f$. Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> ft_sensor_raw = {};

    /**
     * Estimated external wrench applied on TCP and expressed in TCP frame: \f$ ^{TCP}F_{ext} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> ext_wrench_in_tcp = {};

    /**
     * Estimated external wrench applied on TCP and expressed in world frame: \f$ ^{0}F_{ext} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> ext_wrench_in_world = {};

    /**
     * Unfiltered version of ext_wrench_in_tcp. The data is more noisy but has no filter latency.
     */
    std::array<double, kCartDoF> ext_wrench_in_tcp_raw = {};

    /**
     * Unfiltered version of ext_wrench_in_world The data is more noisy but has no filter latency.
     */
    std::array<double, kCartDoF> ext_wrench_in_world_raw = {};
};

/**
 * @struct PlanInfo
 * @brief Data structure containing information of the on-going primitive/plan.
 * @see Robot::plan_info().
 */
struct PlanInfo
{
    /** Current primitive name */
    std::string pt_name = {};

    /** Current node name */
    std::string node_name = {};

    /** Current node path */
    std::string node_path = {};

    /** Current node path time period */
    std::string node_path_time_period = {};

    /** Current node path number */
    std::string node_path_number = {};

    /** Assigned plan name */
    std::string assigned_plan_name = {};

    /** Velocity scale */
    double velocity_scale = {};

    /** Waiting for user signal to step the breakpoint */
    bool waiting_for_step = {};
};

/**
 * @struct GripperStates
 * @brief Data structure containing the gripper states.
 * @see Gripper::states().
 */
struct GripperStates
{
    /** Measured finger opening width [m] */
    double width = {};

    /** Measured finger force. Positive: opening force, negative: closing force.
     * Reads 0 if the mounted gripper has no force sensing capability [N] */
    double force = {};

    /** Maximum finger opening width of the mounted gripper [m] */
    double max_width = {};
};

/**
 * @struct ToolParams
 * @brief Data structure containing robot tool parameters.
 * @see Tool::params().
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
     * y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$ */
    std::array<double, kPoseSize> tcp_location = {};
};

/**
 * @struct Coord
 * @brief Data structure representing the customized data type "COORD" in Flexiv Elements.
 * @warning Here [m] is used as the unit of length, whereas [mm] is used in Flexiv Elements. The
 * conversion is automatically done when exchanging "COORD" data type with the robot via functions
 * like Robot::ExecutePrimitive(), Robot::SetGlobalVariables(), Robot::global_variables(), etc.
 */
struct Coord
{
    /**
     * @brief Construct an instance of Coord.
     * @param[in] _position Sets the [position] member.
     * @param[in] _orientation Sets the [orientation] member.
     * @param[in] _ref_frame Sets the [ref_frame] member.
     * @param[in] _ref_joint_positions Sets the [ref_joint_positions] member. Leave empty to use
     * default values.
     * @param[in] _ext_axis_positions Sets the [ext_axis_positions] member. Leave empty if there's
     * no external axis.
     */
    Coord(const std::array<double, kCartDoF / 2>& _position,
        const std::array<double, kCartDoF / 2>& _orientation,
        const std::array<std::string, 2>& _ref_frame,
        const std::array<double, kSerialJointDoF>& _ref_joint_positions = {},
        const std::array<double, kMaxExtAxes>& _ext_axis_positions = {})
    : position(_position)
    , orientation(_orientation)
    , ref_frame(_ref_frame)
    , ref_joint_positions(_ref_joint_positions)
    , ext_axis_positions(_ext_axis_positions)
    {
    }
    Coord() = default;

    /** Position in [ref_frame]. Unit: [m] */
    std::array<double, kCartDoF / 2> position = {};

    /** Orientation in terms of Euler angles in [ref_frame]. Unit: [degree] */
    std::array<double, kCartDoF / 2> orientation = {};

    /** Name of the reference frame "root::branch" represented as {"root", "branch"}.
     *  Refer to Flexiv Elements for available options. Some common ones are:
     * - World origin: {"WORLD", "WORLD_ORIGIN"}
     * - Current pose: {"TRAJ", "START"}
     * - A work coordinate: {"WORK", "WorkCoord0"}
     * - A global variable: {"GVAR", "MyCoord0"}
     */
    std::array<std::string, 2> ref_frame = {};

    /** Reference joint positions for robot with a redundant degree of freedom. Unit: [degree]
     * @note Leave empty to use default values. However, this array cannot be empty if
     * [ext_axis_positions] has values */
    std::array<double, kSerialJointDoF> ref_joint_positions = {};

    /** Linear or angular positions of the external axes. Unit: [m] or [degree]
     * @note Leave empty if there's no external axis. */
    std::array<double, kMaxExtAxes> ext_axis_positions = {};

    /** String representation of all data in the struct, separated by space */
    std::string str() const;
};

/** Alias of the variant that holds all possible types of data exchanged with Flexiv robots */
using FlexivDataTypes = std::variant<int, double, std::string, rdk::Coord, std::vector<int>,
    std::vector<double>, std::vector<std::string>, std::vector<rdk::Coord>>;

/**
 * @brief Operator overloading to out stream all robot info in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] robot_info RobotInfo data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const RobotInfo& robot_info);

/**
 * @brief Operator overloading to out stream all robot states in JSON format:
 * {"state_1": [val1,val2,val3,...], "state_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] robot_states RobotStates data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const RobotStates& robot_states);

/**
 * @brief Operator overloading to out stream all plan info in JSON format:
 * {"info_1": [val1,val2,val3,...], "info_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] plan_info PlanInfo data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const PlanInfo& plan_info);

/**
 * @brief Operator overloading to out stream all gripper states in JSON format:
 * {"state_1": [val1,val2,val3,...], "state_2": [val1,val2,val3,...], ...}.
 * @param[in] ostream Ostream instance.
 * @param[in] gripper_states GripperStates data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const GripperStates& gripper_states);

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_DATA_HPP_ */
