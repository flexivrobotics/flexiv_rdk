/**
 * @file safety.hpp
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_SAFETY_HPP_
#define FLEXIV_RDK_SAFETY_HPP_

#include "robot.hpp"

namespace flexiv::rdk {

/** Number of safety IO ports */
constexpr size_t kSafetyIOPorts = 8;

/**
 * @struct SafetyLimits
 * @brief Data structure containing configurable robot safety limits, for each single-arm joint
 * group of the robot. They are the ones under Settings -> Safety Configuration in Flexiv Elements:
 * the joint limits under Joint Limits, and the TCP, power and momentum limits under General Limits.
 * @note A TCP, power or momentum limit that the robot does not enforce, i.e. whose enabling status
 * is "Disabled" in Flexiv Elements, is infinity. Some robot models do not enforce the power,
 * momentum, TCP force and TCP torque limits by default.
 */
struct SafetyLimits
{
    /** Lower safety limits of joint positions for each joint group: \f$ q_{min} \in
     * \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$. */
    std::map<JointGroup, std::vector<double>> q_min = {};

    /** Upper safety limits of joint positions for each joint group: \f$ q_{max} \in
     * \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad] \f$. */
    std::map<JointGroup, std::vector<double>> q_max = {};

    /** Upper safety limits of joint velocities for each joint group when the robot is in normal
     * state: \f$ \dot{q}^{normal}_{max} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s] \f$. */
    std::map<JointGroup, std::vector<double>> dq_max_normal = {};

    /** Upper safety limits of joint velocities for each joint group when the robot is in reduced
     * state: \f$ \dot{q}^{reduced}_{max} \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [rad/s] \f$.
     * @see Robot::reduced() */
    std::map<JointGroup, std::vector<double>> dq_max_reduced = {};

    /** Upper safety limit of the TCP linear velocity of each arm when the robot is in normal state.
     * Unit: \f$ [m/s] \f$. */
    std::map<JointGroup, double> tcp_linear_vel_max_normal = {};

    /** Upper safety limit of the TCP linear velocity of each arm when the robot is in reduced
     * state. Unit: \f$ [m/s] \f$.
     * @see Robot::reduced() */
    std::map<JointGroup, double> tcp_linear_vel_max_reduced = {};

    /** Upper safety limit of the TCP angular velocity of each arm when the robot is in normal
     * state. Unit: \f$ [rad/s] \f$. */
    std::map<JointGroup, double> tcp_angular_vel_max_normal = {};

    /** Upper safety limit of the TCP angular velocity of each arm when the robot is in reduced
     * state. Unit: \f$ [rad/s] \f$.
     * @see Robot::reduced() */
    std::map<JointGroup, double> tcp_angular_vel_max_reduced = {};

    /** Upper safety limit of the power of each arm when the robot is in normal state. Unit:
     * \f$ [W] \f$. */
    std::map<JointGroup, double> power_max_normal = {};

    /** Upper safety limit of the power of each arm when the robot is in reduced state. Unit:
     * \f$ [W] \f$.
     * @see Robot::reduced() */
    std::map<JointGroup, double> power_max_reduced = {};

    /** Upper safety limit of the momentum of each arm when the robot is in normal state. Unit:
     * \f$ [kg \cdot m/s] \f$. */
    std::map<JointGroup, double> momentum_max_normal = {};

    /** Upper safety limit of the momentum of each arm when the robot is in reduced state. Unit:
     * \f$ [kg \cdot m/s] \f$.
     * @see Robot::reduced() */
    std::map<JointGroup, double> momentum_max_reduced = {};

    /** Upper safety limit of the force at the TCP of each arm when the robot is in normal state.
     * Unit: \f$ [N] \f$. */
    std::map<JointGroup, double> tcp_force_max_normal = {};

    /** Upper safety limit of the force at the TCP of each arm when the robot is in reduced state.
     * Unit: \f$ [N] \f$.
     * @see Robot::reduced() */
    std::map<JointGroup, double> tcp_force_max_reduced = {};

    /** Upper safety limit of the torque at the TCP of each arm when the robot is in normal state.
     * Unit: \f$ [Nm] \f$. */
    std::map<JointGroup, double> tcp_torque_max_normal = {};

    /** Upper safety limit of the torque at the TCP of each arm when the robot is in reduced state.
     * Unit: \f$ [Nm] \f$.
     * @see Robot::reduced() */
    std::map<JointGroup, double> tcp_torque_max_reduced = {};
};

/**
 * @class Safety
 * @brief Interface to manage safety settings of the robot. Reading the safety settings needs no
 * password, while changing them requires the administrator password of the robot.
 * @note As mentioned in the function doc, certain safety settings are only applicable to the arms
 * but not the external axis.
 * @note Thread safety: all functions of this class are thread-safe and can be called concurrently
 * from multiple threads.
 */
class RDK_API Safety
{
public:
    /**
     * @brief [Blocking] Instantiate the safety settings interface to only read the safety settings,
     * which needs no password.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @throw std::runtime_error if the initialization sequence failed, e.g. the default or current
     * safety limits could not be obtained from the robot.
     * @note This function blocks until the robot has replied.
     * @note Functions that change the safety settings throw std::logic_error on this instance.
     * Instantiate with the password to change them.
     */
    explicit Safety(const Robot& robot);

    /**
     * @brief [Blocking] Instantiate the safety settings interface to read and change the safety
     * settings.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @param[in] password Password to authorize making changes to the robot's safety settings,
     * which is the administrator password of the robot's user authority, as set in Flexiv
     * Elements. The robot verifies it, here and again on every request to change a setting.
     * @throw std::invalid_argument if the provided password is incorrect, or the robot could not
     * verify it (e.g. its user authority service is unavailable, or it is locked out, see below).
     * @throw std::runtime_error if the initialization sequence failed, e.g. the default or current
     * safety limits could not be obtained from the robot.
     * @note This function blocks until the robot has replied.
     * @note To protect the password against guessing, the robot locks out every password, the
     * correct one included, for 1 minute after 5 incorrect ones in a row. After that, each further
     * incorrect password locks them out again, twice as long each time, up to 15 minutes. The
     * correct password, or 1 hour without an incorrect one, starts the count over. The lockout
     * applies to all RDK clients together, and each lockout is recorded in the robot's log.
     * @warning The connection to the robot is not encrypted yet, so the password can be read by
     * any host on the same network. Keep the robot on an isolated network.
     */
    Safety(const Robot& robot, const std::string& password);
    virtual ~Safety();

    /**
     * @brief [Non-blocking] Default values of the safety limits of the connected robot, which are
     * the factory settings of its exact model and variant. Any safety limit to set must stay within
     * them.
     * @return SafetyLimits value copy.
     * @note The values are obtained from the robot when this instance is created.
     */
    SafetyLimits default_limits() const;

    /**
     * @brief [Non-blocking] Current values of the safety limits of the connected robot, for both
     * normal and reduced state.
     * @return SafetyLimits value copy.
     * @note The values are obtained from the robot when this instance is created. They are the ones
     * the robot has been running with since it booted: changes made since then, with this class or
     * in Flexiv Elements, take effect after a reboot.
     */
    SafetyLimits current_limits() const;

    /**
     * @brief [Non-blocking] Current reading from all safety input ports.
     * @return A boolean array whose index corresponds to that of the safety input ports.
     * True: port high; false: port low.
     */
    std::array<bool, kSafetyIOPorts> safety_inputs() const;

    /**
     * @brief [Blocking] Set safety limits on the joint positions of the arm, which will
     * honor this setting when making movements.
     * @param[in] min_positions Minimum joint positions: \f$ q_{min} \in \mathbb{R}^{n \times 1}
     * \f$. Valid range: [default_min_joint_positions, default_max_joint_positions]. Unit: \f$ [rad]
     * \f$.
     * @param[in] max_positions Maximum joint positions: \f$ q_{max} \in \mathbb{R}^{n \times 1}
     * \f$. Valid range: [default_min_joint_positions, default_max_joint_positions]. Unit: \f$ [rad]
     * \f$.
     * @param[in] group Joint group to apply the limits to.
     * @throw std::invalid_argument if [group] is not an existing single-arm joint group in the
     * connected robot, if [min_positions] or [max_positions] contains any value outside the valid
     * range, or if size of any input vector does not match arm DoF.
     * @throw std::logic_error if robot is not in the correct control mode, or if this instance
     * was created without a password.
     * @throw std::runtime_error if the request failed, e.g. it could not be delivered to the
     * connected robot, or the robot rejected the password, see Safety(const Robot&, const
     * std::string&).
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     * @warning A reboot is required for the updated safety settings to take effect. After reboot,
     * make sure to use current_limits() to double check the updated safety limits are correct.
     */
    void SetJointPositionLimits(JointGroup group, const std::vector<double>& min_positions,
        const std::vector<double>& max_positions);

    /**
     * @brief [Blocking] Set safety limits on the joint velocities of the arm, which will
     * honor this setting when making movements under the normal state.
     * @param[in] max_velocities Maximum joint velocities for normal state: \f$ \dot{q}_{max} \in
     * \mathbb{R}^{n \times 1} \f$. Valid range: [0.8727, joint_velocity_normal_limits]. Unit: \f$
     * [rad/s] \f$.
     * @param[in] group Joint group to apply the limits to.
     * @throw std::invalid_argument if [group] is not an existing single-arm joint group in the
     * connected robot, if [max_velocities] contains any value outside the valid range, or if its
     * size does not match arm DoF.
     * @throw std::logic_error if robot is not in the correct control mode, or if this instance
     * was created without a password.
     * @throw std::runtime_error if the request failed, e.g. it could not be delivered to the
     * connected robot, or the robot rejected the password, see Safety(const Robot&, const
     * std::string&).
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     * @warning A reboot is required for the updated safety settings to take effect. After reboot,
     * make sure to use current_limits() to double check the updated safety limits are correct.
     */
    void SetJointVelocityNormalLimits(JointGroup group, const std::vector<double>& max_velocities);

    /**
     * @brief [Blocking] Set safety limits on the joint velocities of the arm, which will
     * honor this setting when making movements under the reduced state.
     * @param[in] max_velocities Maximum joint velocities for reduced state: \f$ \dot{q}_{max} \in
     * \mathbb{R}^{n \times 1} \f$. Valid range: [0.8727, joint_velocity_normal_limits]. Unit: \f$
     * [rad/s] \f$.
     * @param[in] group Joint group to apply the limits to.
     * @throw std::invalid_argument if [group] is not an existing single-arm joint group in the
     * connected robot, if [max_velocities] contains any value outside the valid range, or if its
     * size does not match arm DoF.
     * @throw std::logic_error if robot is not in the correct control mode, or if this instance
     * was created without a password.
     * @throw std::runtime_error if the request failed, e.g. it could not be delivered to the
     * connected robot, or the robot rejected the password, see Safety(const Robot&, const
     * std::string&).
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     * @warning A reboot is required for the updated safety settings to take effect. After reboot,
     * make sure to use current_limits() to double check the updated safety limits are correct.
     */
    void SetJointVelocityReducedLimits(JointGroup group, const std::vector<double>& max_velocities);

    /**
     * @brief [Blocking] Change settings of the regulator on the joint output torques of the
     * arm and external axes. The regulator will limit the joints' total output torques so
     * that the measured torques are less likely to exceed safety limits under large payload or
     * fast/abrupt motions.
     * @param[in] limiting_factor Factor to limit the total output torques: \f$ \tau^{o}_{max} =
     * \tau_{max} \times factor \f$.
     * @param[in] error_threshold If the unregulated output torque of any joint exceeds the
     * configured regulator limit for more than [error_threshold] cycles consecutively, the robot
     * will trigger a minor fault and stop.
     * @throw std::invalid_argument if [limiting_factor] or [error_threshold] is not positive.
     * @throw std::logic_error if robot is not in the correct control mode, or if this instance
     * was created without a password.
     * @throw std::runtime_error if the request failed, e.g. it could not be delivered to the
     * connected robot, or the robot rejected the password, see Safety(const Robot&, const
     * std::string&).
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     * @note This setting takes effect immediately and doesn't require a reboot.
     * @warning Setting the limiting factor too small can cause performance degradation.
     */
    void SetJointOutputTorqueRegulator(
        double limiting_factor = 1.3, unsigned int error_threshold = 50);

private:
    class RDK_HIDDEN Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace flexiv::rdk */

#endif /* FLEXIV_RDK_SAFETY_HPP_ */
