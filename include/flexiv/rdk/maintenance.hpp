/**
 * @file maintenance.hpp
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_MAINTENANCE_HPP_
#define FLEXIV_RDK_MAINTENANCE_HPP_

#include "robot.hpp"

namespace flexiv::rdk {

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
     * @brief [Blocking] Calibrate all joint torque sensors of the robot, which will move to a
     * proper calibration posture first, then start the calibration of its joint torque sensors.
     * Trigger this calibration if the sensed joint torques have noticeable deviations from true
     * values. See below for more details.
     * @param[in] cali_posture Joint positions to move the robot to before starting the calibration,
     * keyed by the joint group to apply them to: \f$ q_cali \in \mathbb{R}^{n \times 1} \f$. The
     * valid keys are the single-arm joint groups of the connected robot, plus JointGroup::EXT_AXIS
     * if it has external axes. Any joint group left out of the map, including all of them when the
     * map is empty, moves to its recommended posture, which is upright for an arm and zero for the
     * external axes. However, calibrating with a custom posture is NOT recommended. Valid range:
     * [RobotInfo::q_min, RobotInfo::q_max]. Unit: \f$ [rad] \f$ for an arm joint or an angular
     * external axis, \f$ [m] \f$ for a linear external axis.
     * @throw std::invalid_argument if [cali_posture] is keyed by a joint group that the connected
     * robot does not have, if the size of an entry does not match the DoF of its joint group, or if
     * an entry contains a value outside the valid range.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if the calibration did not finish within the timeout or a fault
     * occurred during it.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the calibration is finished.
     * @par How to determine when this calibration is needed?
     * 1. When the robot is static and there's no payload or external force exerted on it, if
     * RobotStates::tcp_wrench_local still gives greater than 5N reading, then this calibration
     * should be triggered once.
     * 2. When running the "intermediate4_realtime_joint_floating.cpp" example, if the joints drift
     * swiftly toward one direction, then this calibration should be triggered once.
     */
    void CalibrateJointTorqueSensors(
        const std::map<JointGroup, std::vector<double>>& cali_posture = {});

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace flexiv::rdk */

#endif /* FLEXIV_RDK_MAINTENANCE_HPP_ */
