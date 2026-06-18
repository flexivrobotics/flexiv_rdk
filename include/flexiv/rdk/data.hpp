/**
 * @file data.hpp
 * @brief Header file containing various constant expressions, data structures, and enums.
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_DATA_HPP_
#define FLEXIV_RDK_DATA_HPP_

#include <array>
#include <vector>
#include <string>
#include <ostream>
#include <variant>
#include <chrono>
#include <map>

namespace flexiv::rdk {
/** Cartesian-space degrees of freedom */
constexpr size_t kCartDoF = 6;

/** Joint-space degrees of freedom of Flexiv's serial robots */
constexpr size_t kSerialJointDoF = 7;

/** Size of pose array (3 position + 4 quaternion) */
constexpr size_t kPoseSize = 7;

/** Number of digital IO ports: 16 on the control box + 2 in each wrist connector * maximum 2 wrists
 */
constexpr size_t kIOPorts = 16 + 2 * 2;

/** Maximum number of external axes */
constexpr size_t kMaxExtAxes = 6;

/**
 * @brief All supported product models of the robot.
 */
enum class ProductModel
{
    UNKNOWN = 0,
    Enlight_L,  /// Enlight-L standard version
    Enlight_LL, /// Enlight-LL: Dual Enlight-L with customizable mounting poses
    MICO_Core,  /// MICO-Core: Dual Enlight-L with fixed-mounting upper body
    MICO_Plus,  /// MICO-Plus: MICO-Core with pan-tilt torso
    MICO_Ultra, /// MICO-Ultra: MICO-Plus with mobile base
};

/** Map ProductModel enums to strings */
inline const std::map<ProductModel, std::string> kProductModelNames
    = {{ProductModel::UNKNOWN, "UNKNOWN"}, {ProductModel::Enlight_L, "Enlight-L"},
        {ProductModel::Enlight_LL, "Enlight-LL"}, {ProductModel::MICO_Core, "MICO-Core"},
        {ProductModel::MICO_Plus, "MICO-Plus"}, {ProductModel::MICO_Ultra, "MICO-Ultra"}};

/**
 * @brief All possible joint groups of the robot.
 */
enum class JointGroup
{
    UNKNOWN = -1, ///< Unknown group
    ALL = 0,      ///< The full system, including all actuated joints
    ARMS = 2,     ///< The dual arms as a whole, only applicable to dual-arm robots
    ARM_1 = 3,    ///< The 1st single arm in a dual-arm robot or the only arm in a single-arm robot
    ARM_2 = 4,    ///< The 2nd single arm in a dual-arm robot, not applicable to single-arm robots
    EXT_AXIS = 5, ///< External axis(es) for workspace extension

    FIRST = ALL,
    LAST = EXT_AXIS,
};

/** Map JointGroup enums to strings */
inline const std::map<JointGroup, std::string> kJointGroupNames {
    {JointGroup::UNKNOWN, "UNKNOWN"},
    {JointGroup::ALL, "ALL"},
    {JointGroup::ARMS, "ARMS"},
    {JointGroup::ARM_1, "ARM_1"},
    {JointGroup::ARM_2, "ARM_2"},
    {JointGroup::EXT_AXIS, "EXT_AXIS"},
};

/**
 * @brief All possible operational statuses of the robot. Except for the first two, the other
 * enumerators indicate the cause of the robot being not ready to operate.
 * @see Robot::operational_status().
 */
enum class OperationalStatus
{
    UNKNOWN = 0,        ///< Unknown status.
    READY,              ///< Ready to be operated.
    BOOTING,            ///< System still booting, please wait.
    ESTOP_NOT_RELEASED, ///< E-Stop is not released.
    NOT_SERVO_ON,       ///< Not servo on, call ServoOn() to send the signal.
    RELEASING_BRAKE,    ///< Brake release in progress, please wait.
    MINOR_FAULT,        ///< Minor fault occurred, call ClearFault() to try clearing it.
    CRITICAL_FAULT,     ///< Critical fault occurred, call ClearFault() to try clearing it.
    IN_REDUCED_STATE,   ///< In reduced state, see reduced().
    IN_RECOVERY_STATE,  ///< In recovery state, see recovery().
    IN_MANUAL_MODE,     ///< In Manual mode, need to switch to Auto (Remote) mode.
    IN_AUTO_MODE,       ///< In regular Auto mode, need to switch to Auto (Remote) mode.
};

/** Map OperationalStatus enums to strings */
inline const std::map<OperationalStatus, std::string> kOpStatusNames {
    {OperationalStatus::UNKNOWN, "Unknown status"},
    {OperationalStatus::READY, "Ready"},
    {OperationalStatus::BOOTING, "System booting"},
    {OperationalStatus::ESTOP_NOT_RELEASED, "E-Stop not released"},
    {OperationalStatus::NOT_SERVO_ON, "Not servo on"},
    {OperationalStatus::RELEASING_BRAKE, "Releasing brakes"},
    {OperationalStatus::MINOR_FAULT, "Minor fault occurred"},
    {OperationalStatus::CRITICAL_FAULT, "Critical fault occurred"},
    {OperationalStatus::IN_REDUCED_STATE, "In reduced state"},
    {OperationalStatus::IN_RECOVERY_STATE, "In recovery state"},
    {OperationalStatus::IN_MANUAL_MODE, "In Manual mode"},
    {OperationalStatus::IN_AUTO_MODE, "In regular Auto mode"},
};

/**
 * @brief Type of commonly-used reference coordinates.
 */
enum class CoordType
{
    WORLD, ///< World frame (fixed).
    TCP,   ///< TCP frame (move with the robot's end effector).
};

/**
 * @struct RobotEvent
 * @brief Information about a robot event.
 * @see Robot::event_log().
 */
struct RobotEvent
{
    enum Level
    {
        UNKNOWN = 0, ///< Not set
        INFO,        ///< Informational event
        WARNING,     ///< Warning event
        ERROR,       ///< Error event
        CRITICAL,    ///< Critical error event
    };

    /** Level of the event */
    Level level = UNKNOWN;

    /** Unique ID of the event */
    int id = 0;

    /** Brief description of the event */
    std::string description = "";

    /** Consequences caused by the event */
    std::string consequences = "";

    /** Probable causes of the event */
    std::string probable_causes = "";

    /** Recommended actions after the event */
    std::string recommended_actions = "";

    /** Timestamp (since epoch) of the event */
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};

/**
 * @struct RobotInfo
 * @brief General information about the connected robot.
 * @see Robot::info().
 */
struct RobotInfo
{
    /** Product model of the robot */
    ProductModel product_model = {};

    /** Serial number of the robot */
    std::string serial_num = {};

    /** Software version of the robot */
    std::string software_ver = {};

    /** Type of license */
    std::string license_type = {};

    /** All available joint groups in the robot and their names */
    std::map<JointGroup, std::string> all_groups = {};

    /** Available single-arm joint groups in the robot and their names */
    std::map<JointGroup, std::string> single_arm_groups = {};

    /** Joint-space degrees of freedom for each joint group */
    std::map<JointGroup, size_t> DoF = {};

    /** Nominal Cartesian motion stiffness of all available single-arm joint groups when in
     * Cartesian motion-force control modes: \f$ K_x^{nom} \in \mathbb{R}^{6 \times 1} \f$. Consists
     * of \f$ \mathbb{R}^{3 \times 1} \f$ linear stiffness and \f$ \mathbb{R}^{3 \times 1} \f$
     * angular stiffness: \f$ [k_x, k_y, k_z, k_{Rx}, k_{Ry}, k_{Rz}]^T \f$.
     * Unit: \f$ [N/m]:[Nm/rad] \f$. */
    std::map<JointGroup, std::array<double, kCartDoF>> K_x_nom = {};

    /** Nominal joint motion stiffness of all available single-arm joint groups when in joint
     * impedance control modes: \f$ K_q^{nom} \in \mathbb{R}^{n \times 1} \f$.
     * Unit: \f$ [Nm/rad] \f$. */
    std::map<JointGroup, std::vector<double>> K_q_nom = {};

    /** Lower software limits of joint positions of all available joint groups: \f$ q_{min} \in
     * \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$. */
    std::map<JointGroup, std::vector<double>> q_min = {};

    /** Upper software limits of joint positions of all available joint groups: \f$ q_{max} \in
     * \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$. */
    std::map<JointGroup, std::vector<double>> q_max = {};

    /** Upper software limits of joint velocities of all available joint groups: \f$ \dot{q}_{max}
     * \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s] \f$. */
    std::map<JointGroup, std::vector<double>> dq_max = {};

    /** Upper software limits of joint torques of all available joint groups: \f$ \tau_{max} \in
     * \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm] \f$. */
    std::map<JointGroup, std::vector<double>> tau_max = {};
};

/**
 * @struct RobotStates
 * @brief Robot states data in joint and Cartesian space for a joint group.
 */
struct RobotStates
{
    /** Current time since epoch of the robot system. The pair consists of {seconds since epoch,
     * nanoseconds since last full second} */
    std::pair<int, int> timestamp = {};

    /**
     * Measured joint positions from the link-side encoder: \f$ q \in \mathbb{R}^{n \times 1} \f$.
     * This is the direct measurement of joint positions. Unit: \f$ [rad] or [m] \f$.
     * @note If a joint has only one encoder, then \f$ \theta = q \f$.
     */
    std::vector<double> q = {};

    /**
     * Measured joint positions from the motor-side encoder: \f$ \theta \in \mathbb{R}^{n \times 1}
     * \f$. This is the indirect measurement of joint positions. \f$ \theta = q + \Delta \f$, where
     * \f$ \Delta \f$ is the joint's internal deflection between motor and link. Unit: \f$ [rad] or
     * [m] \f$.
     * @note If a joint has only one encoder, then \f$ \theta = q \f$.
     */
    std::vector<double> theta = {};

    /**
     * Measured joint velocities from the link-side encoder: \f$ \dot{q} \in \mathbb{R}^{n \times
     * 1} \f$. This is the direct but more noisy measurement of joint velocities. Unit: \f$ [rad/s]
     * or [m/s] \f$.
     * @note If a joint has only one encoder, then \f$ \dot{\theta} = \dot{q} \f$.
     */
    std::vector<double> dq = {};

    /**
     * Measured joint velocities from the motor-side encoder: \f$ \dot{\theta} \in \mathbb{R}^{n
     * \times 1} \f$. This is the indirect but less noisy measurement of joint velocities. Unit: \f$
     * [rad/s] or [m/s] \f$.
     * @note If a joint has only one encoder, then \f$ \dot{\theta} = \dot{q} \f$.
     */
    std::vector<double> dtheta = {};

    /**
     * Measured joint torques: \f$ \tau \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm] \f$.
     * @note If a joint has no torque measurement, the corresponding value will be 0.
     */
    std::vector<double> tau = {};

    /**
     * Numerical derivative of measured joint torques: \f$ \dot{\tau} \in \mathbb{R}^{n \times 1}
     * \f$. Unit: \f$ [Nm/s] \f$.
     * @note If a joint has no torque measurement, the corresponding value will be 0.
     */
    std::vector<double> tau_dot = {};

    /**
     * Estimated external joint torques: \f$ \hat \tau_{ext} \in \mathbb{R}^{n \times 1} \f$.
     * Produced by any external contact (with robot body or end-effector) that does not belong to
     * the known robot model. Unit: \f$ [Nm] \f$.
     * @note If a joint has no torque measurement, the corresponding value will be 0.
     */
    std::vector<double> tau_ext = {};

    /**
     * Estimated interaction joint torques: \f$ \hat \tau_{int} \in \mathbb{R}^{n \times 1} \f$.
     * Produced by any interaction forces at the TCP. Unit: \f$ [Nm] \f$.
     * @note If a joint has no torque measurement, the corresponding value will be 0.
     */
    std::vector<double> tau_interact = {};

    /**
     * Measured joint temperatures: \f$ temp \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [°C] \f$.
     * @note If a joint has no temperature measurement, the corresponding value will be 0.
     */
    std::vector<double> temperature = {};

    /**
     * Measured flange pose w.r.t. world frame: \f$ ^{O}T_{flange} \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     */
    std::array<double, kPoseSize> flange_pose = {};

    /**
     * Measured TCP pose w.r.t. world frame: \f$ ^{O}T_{TCP} \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     */
    std::array<double, kPoseSize> tcp_pose = {};

    /**
     * Measured TCP twist w.r.t. world frame: \f$ ^{O}\dot{X} \in \mathbb{R}^{6 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$ \mathbb{R}^{3 \times 1}
     * \f$ angular velocity: \f$ [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$. Unit: \f$
     * [m/s]:[rad/s] \f$.
     */
    std::array<double, kCartDoF> tcp_twist = {};

    /**
     * Measured or estimated external wrench applied on TCP w.r.t. world frame: \f$ ^{O}F_{ext} \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> tcp_wrench = {};

    /**
     * Measured or estimated external wrench applied on TCP w.r.t. local frame: \f$ ^{TCP}F_{ext}
     * \in \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$.
     * Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> tcp_wrench_local = {};

    /**
     * Unfiltered tcp_wrench. The data is more noisy but has no filter latency.
     */
    std::array<double, kCartDoF> raw_tcp_wrench = {};

    /**
     * Unfiltered tcp_wrench_local. The data is more noisy but has no filter latency.
     */
    std::array<double, kCartDoF> raw_tcp_wrench_local = {};

    /**
     * Raw reading from the force-torque (FT) sensor w.r.t. flange frame: \f$ ^{flange}F_{raw}
     * \in \mathbb{R}^{6 \times 1} \f$. The value is 0 if no FT sensor is installed. Consists of
     * \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$
     * [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$ [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> raw_ft_sensor = {};
};

/**
 * @struct RobotActions
 * @brief Robot actions data in joint and Cartesian space for a joint group.
 */
struct RobotActions
{
    /** Current time since epoch of the robot system. The pair consists of {seconds since epoch,
     * nanoseconds since last full second} */
    std::pair<int, int> timestamp = {};

    /**
     * Desired joint positions: \f$ q_d \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] or [m] \f$.
     */
    std::vector<double> q_d = {};

    /**
     * Desired joint velocities: \f$ \dot{q}_d \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s] or
     * [m/s] \f$.
     */
    std::vector<double> dq_d = {};

    /**
     * Desired joint torques excluding the compensation of nonlinear dynamics: \f$ \tau_d \in
     * \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm] \f$.
     * @note Nonlinear dynamics include: gravity, centrifugal, and Coriolis. If the robot is static,
     * tau_d will be zeros.
     * @note If a joint has no torque control capability, the corresponding value will be 0.
     */
    std::vector<double> tau_d = {};

    /**
     * Desired TCP pose w.r.t. world frame: \f$ {^{O}T_{TCP}}_d \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]~[] \f$.
     */
    std::array<double, kPoseSize> tcp_pose_d = {};

    /**
     * Desired TCP twist w.r.t. world frame: \f$ {^{O}\dot{X}}_d \in \mathbb{R}^{6 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ linear velocity and \f$ \mathbb{R}^{3 \times 1}
     * \f$ angular velocity: \f$ [v_x, v_y, v_z, \omega_x, \omega_y, \omega_z]^T \f$. Unit: \f$
     * [m/s]:[rad/s] \f$.
     */
    std::array<double, kCartDoF> tcp_twist_d = {};

    /**
     * Desired TCP wrench w.r.t. the current force control frame: \f$ {^{ctrl}F_{ext}}_d \in
     * \mathbb{R}^{6 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ force and \f$
     * \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T \f$. Unit: \f$
     * [N]:[Nm] \f$.
     */
    std::array<double, kCartDoF> tcp_wrench_d = {};
};

/**
 * @struct PlanInfo
 * @brief Information of the on-going primitive/plan.
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
 * @struct JPos
 * @brief Data structure representing the customized data type "JPOS" in Flexiv Elements.
 * @warning Here [m] is used as the unit of length, whereas [mm] is used in Flexiv Elements. The
 * conversion is automatically done when exchanging "JPOS" data type with the robot via functions
 * like Robot::ExecutePrimitive(), Robot::SetGlobalVariables(), etc.
 */
struct JPos
{
    /** Default constructor */
    JPos() = default;

    /**
     * @brief Custom constructor.
     * @param[in] q_m Sets struct member [q_m].
     * @param[in] q_e Sets struct member [q_e]. Leave empty if there's no external axis.
     */
    JPos(const std::array<double, kSerialJointDoF>& q_m,
        const std::array<double, kMaxExtAxes>& q_e = {})
    : q_m(q_m)
    , q_e(q_e)
    {
    }

    /** Joint positions of the robot manipulator. Unit: [degree] */
    std::array<double, kSerialJointDoF> q_m = {};

    /** Joint positions (linear or angular) of the external axes. Unit: [m] or [degree]
     * @note If the number of external axes \f$ n_e < kMaxExtAxes \f$, set the first \f$ n_e \f$
     * elements and leave the rest 0. Leave the whole array empty if there's no external axis. */
    std::array<double, kMaxExtAxes> q_e = {};

    /** String representation of all data in the struct, separated by space */
    std::string str() const;
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
    /** Default constructor */
    Coord() = default;

    /**
     * @brief Custom constructor.
     * @param[in] position Sets struct member [position].
     * @param[in] orientation Sets struct member [orientation].
     * @param[in] ref_frame Sets struct member [ref_frame].
     * @param[in] ref_q_m Sets struct member [ref_q_m]. Leave empty to use default values.
     * @param[in] ref_q_e Sets struct member [ref_q_e]. Leave empty if there's no external axis.
     */
    Coord(const std::array<double, kCartDoF / 2>& position,
        const std::array<double, kCartDoF / 2>& orientation,
        const std::array<std::string, 2>& ref_frame,
        const std::array<double, kSerialJointDoF>& ref_q_m = {},
        const std::array<double, kMaxExtAxes>& ref_q_e = {})
    : position(position)
    , orientation(orientation)
    , ref_frame(ref_frame)
    , ref_q_m(ref_q_m)
    , ref_q_e(ref_q_e)
    {
    }

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

    /** Reference joint positions of the robot manipulator. Only effective on robots with redundant
     * degrees of freedom. Unit: [degree]
     * @note Leave empty to use default values. However, this array cannot be empty if [ref_q_e] has
     * values */
    std::array<double, kSerialJointDoF> ref_q_m = {};

    /** Reference joint positions (linear or angular) of the external axes. Only effective on
     * robots with redundant degrees of freedom and external axes. Unit: [m] or [degree]
     * @note If the number of external axes \f$ n_e < kMaxExtAxes \f$, set the first \f$ n_e \f$
     * elements and leave the rest 0. Leave the whole array empty if there's no external axis. */
    std::array<double, kMaxExtAxes> ref_q_e = {};

    /** String representation of all data in the struct, separated by space */
    std::string str() const;
};

/**
 * @brief Modes for synchronous motions.
 * @see PrimitiveArgs::sync_motion_mode
 */
enum class SyncMotionMode
{
    DISABLE = 0,    ///< Don't sync with any target
    ARM1_TCP = 1,   ///< Sync with arm1 tcp
    ARM2_TCP = 2,   ///< sync with arm2 tcp
    POSITIONER = 3, ///< sync with positioner
};

/** Alias of the variant that holds all possible types of data exchanged with Flexiv robots */
using FlexivDataTypes = std::variant<int, double, std::string, rdk::JPos, rdk::Coord,
    std::vector<int>, std::vector<double>, std::vector<std::string>, std::vector<rdk::JPos>,
    std::vector<rdk::Coord>>;

/**
 * @struct PrimitiveArgs
 * @brief Arguments of a primitive command.
 * @see Robot::ExecutePrimitive().
 * @see [Flexiv Primitives documentation](https://www.flexiv.com/primitives/)
 */
struct PrimitiveArgs
{
    /** Default constructor */
    PrimitiveArgs() = default;

    /** Custom constructor */
    PrimitiveArgs(
        const std::string& pt_name, const std::map<std::string, FlexivDataTypes>& input_params)
    : pt_name(pt_name)
    , input_params(input_params)
    {
    }

    /** Name of the primitive to execute. For example, "Home", "MoveL", "ZeroFTSensor", etc. */
    std::string pt_name = {};

    /** Input parameter names and values of this primitive. Use int 1 and 0 to represent booleans.
     * E.g. {{"target", rdk::Coord({0.65, -0.3, 0.2}, {180, 0, 180}, {"WORLD", "WORLD_ORIGIN"})},
     * {"vel", 0.6}, {"zoneRadius", "Z50"}}.
     * @warning The primitive input parameters may not use SI units, please refer to the Flexiv
     * Primitives documentation for exact unit definition. */
    std::map<std::string, FlexivDataTypes> input_params = {};

    /** True: The external axis motion is actively controlled by this primitive. When this primitive
     * is executed, the external axis will move to the position defined in target points or
     * waypoints. False: The external axis motion is not actively controlled by this primitive.
     * @note This setting is only effective when the external axis exists. */
    bool external_axis_control = true;

    /** Mode for synchronous motions between the TCP of this primitive and the selected arm or
     * positioner. Once configured, the motion of the TCP will be automatically adjusted in
     * accordance with the movement of the selected arm or positioner.
     * @note Only certain primitives support this feature, this setting will be ignored if this
     * primitive does not support it. */
    SyncMotionMode sync_motion_mode = SyncMotionMode::DISABLE;
};

/**
 * @struct PrimitiveStates
 * @brief States data of a primitive.
 * @see Robot::primitive_states().
 */
struct PrimitiveStates
{
    /** Name of the currently running primitive */
    std::string pt_name = {};

    /** Names and corresponding values of this primitive's states. Booleans are represented by int 1
     * and 0. For example:
     * {{"reachedTarget", 1}, {"timePeriod", 5.6}, {"forceOffset", {0.1, 0.2, -1.3}}}.
     */
    std::map<std::string, FlexivDataTypes> names_and_values = {};
};

/**
 * @struct RtJointTorqueCmd
 * @brief Commands data for real-time joint torque control.
 * @see Robot::StreamJointTorque().
 */
struct RtJointTorqueCmd
{
    /** Default constructor */
    RtJointTorqueCmd() = default;

    /** Custom constructor */
    RtJointTorqueCmd(const std::vector<double>& tau_d, bool enable_gravity_comp = true,
        bool enable_soft_limits = true, double friction_comp_scale = 100.0)
    : tau_d(tau_d)
    , enable_gravity_comp(enable_gravity_comp)
    , enable_soft_limits(enable_soft_limits)
    , friction_comp_scale(friction_comp_scale)
    {
    }

    /** Target joint torques: \f$ \tau_d \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm] \f$ */
    std::vector<double> tau_d = {};

    /** Enable/disable robot gravity compensation */
    bool enable_gravity_comp = true;

    /** Enable/disable soft limits to keep the joints from moving outside allowed position range,
     * which will trigger a safety fault that requires recovery operation */
    bool enable_soft_limits = true;

    /** Percentage of joint friction to be compensated. Valid range: [0, 100]. Setting to 100 means
     * to compensate all joint friction, and 0 means no friction compensation at all.
     * Under-compensation increases natural damping of the joints, which can be useful in some
     * cases, e.g. zero-torque floating. */
    double friction_comp_scale = 100.0;
};

/**
 * @struct RtJointPositionCmd
 * @brief Commands data for real-time joint position control.
 * @see Robot::StreamJointPosition().
 */
struct RtJointPositionCmd
{
    /** Default constructor */
    RtJointPositionCmd() = default;

    /** Custom constructor */
    RtJointPositionCmd(const std::vector<double>& q_d, const std::vector<double>& dq_d,
        const std::vector<double>& ddq_d)
    : q_d(q_d)
    , dq_d(dq_d)
    , ddq_d(ddq_d)
    {
    }

    /** Target joint positions: \f$ q_d \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$ */
    std::vector<double> q_d = {};

    /** Target joint velocities: \f$ \dot{q}_d \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s]
     * \f$ */
    std::vector<double> dq_d = {};

    /** Target joint accelerations: \f$ \ddot{q}_d \in \mathbb{R}^{n \times 1} \f$. Unit: \f$
     * [rad/s^2] \f$ */
    std::vector<double> ddq_d = {};
};

/**
 * @struct NrtJointPositionCmd
 * @brief Commands data for non-real-time joint position control.
 * @see Robot::SendJointPosition().
 */
struct NrtJointPositionCmd
{
    /** Default constructor */
    NrtJointPositionCmd() = default;

    /** Custom constructor */
    NrtJointPositionCmd(const std::vector<double>& q_d, const std::vector<double>& dq_d,
        const std::vector<double>& dq_max, const std::vector<double>& ddq_max)
    : q_d(q_d)
    , dq_d(dq_d)
    , dq_max(dq_max)
    , ddq_max(ddq_max)
    {
    }

    /** Target joint positions: \f$ q_d \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$ */
    std::vector<double> q_d = {};

    /** Target joint velocities: \f$ \dot{q}_d \in \mathbb{R}^{n \times 1} \f$. Each joint will
     * maintain this amount of velocity when it reaches the target position. Unit: \f$ [rad/s] \f$
     */
    std::vector<double> dq_d = {};

    /** Maximum joint velocities for the planned trajectory: \f$ \dot{q}_{max} \in \mathbb{R}^{n
     * \times 1} \f$. Unit: \f$ [rad/s] \f$ */
    std::vector<double> dq_max = {};

    /** Maximum joint accelerations for the planned trajectory: \f$ \ddot{q}_{max} \in \mathbb{R}^{n
     * \times 1} \f$. Unit: \f$ [rad/s^2] \f$ */
    std::vector<double> ddq_max = {};
};

/**
 * @struct RtCartesianCmd
 * @brief Commands data for real-time Cartesian motion-force control.
 * @see Robot::StreamCartesianMotionForce().
 */
struct RtCartesianCmd
{
    /** Default constructor */
    RtCartesianCmd() = default;

    /** Custom constructor */
    RtCartesianCmd(const std::array<double, kPoseSize>& pose_d,
        const std::array<double, kCartDoF>& wrench_d = {},
        const std::array<double, kCartDoF>& twist_d = {},
        const std::array<double, kCartDoF>& acc_d = {})
    : pose_d(pose_d)
    , wrench_d(wrench_d)
    , twist_d(twist_d)
    , acc_d(acc_d)
    {
    }

    /** Target TCP pose in world frame: \f$ {^{O}T_{TCP}}_{d} \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$ */
    std::array<double, kPoseSize> pose_d = {};

    /** Target TCP wrench in the force control reference frame (configured by
     * SetForceControlFrame()): \f$ ^{O}F_d \in \mathbb{R}^{6 \times 1} \f$. The robot will track
     * the target wrench using an explicit force controller. Consists of \f$ \mathbb{R}^{3 \times 1}
     * \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T
     * \f$. Unit: \f$ [N]:[Nm] \f$ */
    std::array<double, kCartDoF> wrench_d = {};

    /** Target TCP twist in world frame: \f$ ^{O}\dot{x}_d \in \mathbb{R}^{6 \times 1} \f$.
     * Providing properly calculated target twist can improve the robot's overall tracking
     * performance at the cost of reduced robustness. Leaving this input 0 can maximize robustness
     * at the cost of reduced tracking performance. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
     * linear and \f$ \mathbb{R}^{3 \times 1} \f$ angular velocity. Unit: \f$ [m/s]:[rad/s] \f$ */
    std::array<double, kCartDoF> twist_d = {};

    /** Target TCP acceleration in world frame: \f$ ^{O}\ddot{x}_d \in \mathbb{R}^{6 \times 1} \f$.
     * Feeding forward target acceleration can improve the robot's tracking performance for highly
     * dynamic motions, but it's also okay to leave this input 0. Consists of \f$ \mathbb{R}^{3
     * \times 1} \f$ linear and \f$ \mathbb{R}^{3 \times 1} \f$ angular acceleration. Unit: \f$
     * [m/s^2]:[rad/s^2] \f$ */
    std::array<double, kCartDoF> acc_d = {};
};

/**
 * @struct NrtCartesianCmd
 * @brief Commands data for non-real-time Cartesian motion-force control.
 * @see Robot::SendCartesianMotionForce().
 */
struct NrtCartesianCmd
{
    /** Default constructor */
    NrtCartesianCmd() = default;

    /** Custom constructor */
    NrtCartesianCmd(const std::array<double, kPoseSize>& pose_d,
        const std::array<double, kCartDoF>& wrench_d = {},
        const std::array<double, kCartDoF>& twist_d = {}, double max_linear_vel = 0.5,
        double max_angular_vel = 1.0, double max_linear_acc = 2.0, double max_angular_acc = 5.0)
    : pose_d(pose_d)
    , wrench_d(wrench_d)
    , twist_d(twist_d)
    , max_linear_vel(max_linear_vel)
    , max_angular_vel(max_angular_vel)
    , max_linear_acc(max_linear_acc)
    , max_angular_acc(max_angular_acc)
    {
    }

    /** Target TCP pose in world frame: \f$ {^{O}T_{TCP}}_{d} \in \mathbb{R}^{7 \times 1} \f$.
     * Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1} \f$
     * quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$ */
    std::array<double, kPoseSize> pose_d = {};

    /** Target TCP wrench in the force control reference frame (configured by
     * SetForceControlFrame()): \f$ ^{O}F_d \in \mathbb{R}^{6 \times 1} \f$. The robot will track
     * the target wrench using an explicit force controller. Consists of \f$ \mathbb{R}^{3 \times 1}
     * \f$ force and \f$ \mathbb{R}^{3 \times 1} \f$ moment: \f$ [f_x, f_y, f_z, m_x, m_y, m_z]^T
     * \f$. Unit: \f$ [N]:[Nm] \f$ */
    std::array<double, kCartDoF> wrench_d = {};

    /** Target TCP twist in world frame: \f$ ^{O}\dot{x}_d \in \mathbb{R}^{6 \times 1} \f$.
     * Providing properly calculated target twist can improve the robot's overall tracking
     * performance at the cost of reduced robustness. Leaving this input 0 can maximize robustness
     * at the cost of reduced tracking performance. Consists of \f$ \mathbb{R}^{3 \times 1} \f$
     * linear and \f$ \mathbb{R}^{3 \times 1} \f$ angular velocity. Unit: \f$ [m/s]:[rad/s] \f$ */
    std::array<double, kCartDoF> twist_d = {};

    /** Maximum Cartesian linear velocity when moving to the target pose. A safe value is provided
     * as default. Unit: \f$ [m/s] \f$ */
    double max_linear_vel = 0.5;

    /** Maximum Cartesian angular velocity when moving to the target pose. A safe value is provided
     * as default. Unit: \f$ [rad/s] \f$ */
    double max_angular_vel = 1.0;

    /** Maximum Cartesian linear acceleration when moving to the target pose. A safe value is
     * provided as default. Unit: \f$ [m/s^2] \f$ */
    double max_linear_acc = 2.0;

    /** Maximum Cartesian angular acceleration when moving to the target pose. A safe value is
     * provided as default. Unit: \f$ [rad/s^2] \f$ */
    double max_angular_acc = 5.0;
};

/**
 * @struct NrtCartesianMultiWaypointCmd
 * @brief Commands data for non-real-time Cartesian multi-waypoint motion-force control.
 * @see Robot::SendCartesianMotionForceMultiWaypoint().
 */
struct NrtCartesianMultiWaypointCmd
{
    /** Default constructor */
    NrtCartesianMultiWaypointCmd() = default;

    /** Custom constructor */
    NrtCartesianMultiWaypointCmd(const std::vector<NrtCartesianCmd>& waypoints)
    : waypoints(waypoints)
    {
    }

    /** Sequence of Cartesian waypoints to execute. Each waypoint uses the same data layout as
     * NrtCartesianCmd. */
    std::vector<NrtCartesianCmd> waypoints = {};
};

/**
 * @brief Operator overloading to out stream all members of RobotEvent in JSON format.
 * @param[in] ostream Ostream instance.
 * @param[in] robot_event RobotEvent data structure to out stream.
 * @return Updated ostream instance.
 * @note The event timestamp is converted to local timezone when printed.
 */
std::ostream& operator<<(std::ostream& ostream, const RobotEvent& robot_event);

/**
 * @brief Operator overloading to out stream all members of RobotInfo in JSON format.
 * @param[in] ostream Ostream instance.
 * @param[in] robot_info RobotInfo data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const RobotInfo& robot_info);

/**
 * @brief Operator overloading to out stream all members of RobotStates in JSON format.
 * @param[in] ostream Ostream instance.
 * @param[in] robot_states RobotStates data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const RobotStates& robot_states);

/**
 * @brief Operator overloading to out stream all members of RobotActions in JSON format.
 * @param[in] ostream Ostream instance.
 * @param[in] robot_actions RobotActions data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const RobotActions& robot_actions);

/**
 * @brief Operator overloading to out stream all members of PlanInfo in JSON format.
 * @param[in] ostream Ostream instance.
 * @param[in] plan_info PlanInfo data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const PlanInfo& plan_info);

} /* namespace flexiv::rdk */

#endif /* FLEXIV_RDK_DATA_HPP_ */
