/**
 * @file maintenance.hpp
 * @copyright Copyright (C) 2016-2025 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_MAINTENANCE_HPP_
#define FLEXIV_RDK_MAINTENANCE_HPP_

#include "robot.hpp"

namespace flexiv {
namespace rdk {

/**
 * @enum NetworkPort
 * @brief Network ports of the control box, named after the labels printed next to them on the
 * control box itself.
 */
enum class NetworkPort
{
    USER_1 = 0, ///< Port labeled "User 1"
    USER_2,     ///< Port labeled "User 2"
    GENERAL,    ///< Port labeled "General"
};

/**
 * @struct PtpConfig
 * @brief Configuration of the PTP (Precision Time Protocol) clock synchronization client running
 * on the control box.
 * @see Maintenance::SetPtpConfig(), Maintenance::ptp_config().
 */
struct PtpConfig
{
    /** Whether the control box runs a PTP client to synchronize its system clock with a PTP
     * grandmaster clock on the network */
    bool enabled = false;

    /** Network port of the control box that the PTP client runs over */
    NetworkPort network_port = NetworkPort::USER_1;

    /** POSIX time zone of the control box system clock, e.g. "UTC-8" for UTC+8. Must not contain
     * any whitespace */
    std::string time_zone = "UTC-8";

    /** Whether to periodically write the synchronized system clock to the hardware RTC clock, so
     * that the synchronized time is preserved across a control box reboot */
    bool sync_rtc_clock = true;
};

/**
 * @class Maintenance
 * @brief Interface to run maintenance operations on the robot.
 * @note Thread safety: all functions of this class are thread-safe and can be called concurrently
 * from multiple threads.
 */
class RDK_API Maintenance
{
public:
    /**
     * @brief [Non-blocking] Instantiate the robot maintenance interface.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    Maintenance(const Robot& robot);
    virtual ~Maintenance();

    /**
     * @brief [Blocking] Calibrate all joint torque sensors. The robot will first move to a proper
     * calibration posture, then start the low-level calibration of all joint torque sensors.
     * Trigger this calibration if the sensed joint torques have noticeable deviations from true
     * values. See below for more details.
     * @param[in] cali_posture Joint positions to move to before starting the calibration: \f$
     * q_cali \in \mathbb{R}^{n \times 1} \f$. If left empty, the robot will use the recommended
     * upright posture for calibration. Otherwise the specified posture will be used, which is NOT
     * recommended. Valid range: [RobotInfo::q_min, RobotInfo::q_max]. Unit: \f$ [rad] \f$.
     * @throw std::invalid_argument if [cali_posture] contains any value outside the valid range, or
     * its size does not match robot DoF.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if fault occurred during the calibration or failed to save the
     * calibration result.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the calibration is finished.
     * @warning The robot needs to be rebooted for the calibration result to take effect.
     * @par How to determine when this calibration is needed?
     * 1. When the robot is static and there's no payload or external force exerted on it, if
     * RobotStates::ext_wrench_in_tcp still gives greater than 5N reading, then this calibration
     * should be triggered once.
     * 2. When running the "intermediate4_realtime_joint_floating.cpp" example, if the joints drift
     * swiftly toward one direction, then this calibration should be triggered once.
     */
    void CalibrateJointTorqueSensors(const std::vector<double>& cali_posture = {});

    /**
     * @brief [Blocking] Update the PTP (Precision Time Protocol) clock synchronization
     * configuration of the control box. When enabled, the control box runs a PTP client that
     * synchronizes its system clock with a PTP grandmaster clock on the network.
     * @param[in] config New PTP configuration. All fields are written, so read the current
     * configuration with ptp_config() first if only some of the fields are to be changed.
     * @throw std::invalid_argument if [config] contains an invalid network port or time zone.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: all.
     * @note This function blocks until the request is successfully delivered.
     * @warning The control box needs to be rebooted for the new configuration to take effect.
     */
    void SetPtpConfig(const PtpConfig& config);

    /**
     * @brief [Blocking] Get the PTP (Precision Time Protocol) clock synchronization configuration
     * currently stored on the control box.
     * @return PtpConfig data structure.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note Applicable control modes: all.
     * @note This function blocks until a reply is received.
     * @warning The returned configuration is the one stored on the control box, which is only
     * applied after the control box is rebooted.
     */
    PtpConfig ptp_config() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_MAINTENANCE_HPP_ */
