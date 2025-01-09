/**
 * @file maintenance.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_MAINTENANCE_HPP_
#define FLEXIV_RDK_MAINTENANCE_HPP_

#include "robot.hpp"

namespace flexiv {
namespace rdk {

/**
 * @class Maintenance
 * @brief Interface to carry out robot maintenance operations. The robot must be in IDLE mode when
 * triggering any operations.
 */
class Maintenance
{
public:
    /**
     * @brief [Non-blocking] Create an instance and initialize the interface.
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
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
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

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_MAINTENANCE_HPP_ */
