/**
 * @file gripper.hpp
 * @copyright Copyright (C) 2016-2026 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_GRIPPER_HPP_
#define FLEXIV_RDK_GRIPPER_HPP_

#include "robot.hpp"
#include <map>
#include <memory>

namespace flexiv::rdk {

/**
 * @struct GripperParams
 * @brief Data structure containing the gripper parameters.
 * @see Gripper::params().
 */
struct GripperParams
{
    /** Gripper name */
    std::string name = {};

    /** Minimum finger opening width [m] */
    double min_width = {};

    /** Maximum finger opening width [m] */
    double max_width = {};

    /** Minimum finger moving velocity [m/s] */
    double min_vel = {};

    /** Maximum finger moving velocity [m/s] */
    double max_vel = {};

    /** Minimum grasping force [N] */
    double min_force = {};

    /** Maximum grasping force [N] */
    double max_force = {};
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
     * Reads 0 if the enabled gripper has no force sensing capability [N] */
    double force = {};

    /** Whether the gripper fingers are moving */
    bool is_moving = {};
};

/**
 * @brief Operator overloading to out stream all members of GripperStates in JSON format.
 * @param[in] ostream Ostream instance.
 * @param[in] gripper_states GripperStates data structure to out stream.
 * @return Updated ostream instance.
 */
std::ostream& operator<<(std::ostream& ostream, const GripperStates& gripper_states);

/**
 * @class Gripper
 * @brief Interface to control the gripper installed on the robot. Because gripper is also a type of
 * robot device, this API uses the same underlying infrastructure as rdk::Device, but with functions
 * tailored specifically for gripper controls.
 */
class Gripper
{
public:
    /**
     * @brief [Non-blocking] Instantiate the gripper control interface.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    Gripper(const Robot& robot);
    virtual ~Gripper();

    /**
     * @brief [Blocking] Enable the specified gripper as a device used by the specified joint group.
     * @param[in] group Joint group that uses this gripper. Only existing single-arm joint groups
     * like ARM_1 and ARM_2 are accepted.
     * @param[in] name Name of the gripper to enable.
     * @throw std::invalid_argument if [group] is not an existing single-arm joint group in the
     * connected robot, or if the specified gripper does not exist.
     * @throw std::runtime_error if failed to deliver the request to the connected robot or failed
     * to sync gripper parameters.
     * @note This function blocks until the request is successfully delivered.
     * @warning There's no enforced check on whether the enabled device is a gripper or not. Using
     * this function to enable a non-gripper device will likely lead to undefined behaviors.
     */
    void Enable(JointGroup group, const std::string& name);

    /**
     * @brief [Blocking] Disable the gripper currently used by the specified joint group.
     * @param[in] group Joint group whose gripper to be disabled.
     * @throw std::logic_error if no gripper is enabled for the specified joint group.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Disable(JointGroup group);

    /**
     * @brief [Blocking] Manually trigger the initialization of the enabled gripper used by the
     * specified joint group. This step is not needed for grippers that automatically initialize
     * upon power-on.
     * @param[in] group Joint group whose gripper is to be initialized.
     * @throw std::logic_error if no gripper is enabled for the specified joint group.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     * @warning This function does not wait for the initialization sequence to finish, the user may
     * need to implement wait after calling this function before commanding the gripper.
     */
    void Init(JointGroup group);

    /**
     * @brief [Blocking] Command the gripper used by the specified joint group to grasp with direct
     * force control. This function requires the enabled gripper to support direct force control.
     * @param[in] group Joint group whose gripper to send this command to.
     * @param[in] force Target gripping force. Positive: closing force, negative: opening force.
     * Valid range: [GripperParams::min_force, GripperParams::max_force]. Unit: \f$ [N] \f$.
     * @throw std::invalid_argument if [force] is outside the valid range.
     * @throw std::logic_error if no gripper is enabled for the specified joint group.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Grasp(JointGroup group, double force);

    /**
     * @brief [Blocking] Command the gripper used by the specified joint group to move the fingers
     * with position control.
     * @param[in] group Joint group whose gripper to send this command to.
     * @param[in] width Target opening width. Valid range: [GripperParams::min_width,
     * GripperParams::max_width]. Unit: \f$ [m] \f$.
     * @param[in] velocity Closing/opening velocity, cannot be 0. Valid range:
     * [GripperParams::min_vel, GripperParams::max_vel]. Unit: \f$ [m/s] \f$.
     * @param[in] force_limit Maximum contact force during movement. Valid range:
     * [GripperParams::min_force, GripperParams::max_force]. Unit: \f$ [N] \f$.
     * @throw std::invalid_argument if any input parameter is outside its valid range.
     * @throw std::logic_error if no gripper is enabled for the specified joint group.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Move(JointGroup group, double width, double velocity, double force_limit);

    /**
     * @brief [Blocking] Stop and hold the gripper used by the specified joint group.
     * @param[in] group Joint group whose gripper to send this command to.
     * @throw std::logic_error if no gripper is enabled for the specified joint group.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note This function blocks until the request is successfully delivered.
     */
    void Stop(JointGroup group);

    /**
     * @brief [Non-blocking] Parameters of all enabled grippers.
     * @return Gripper parameters mapped by joint group.
     */
    std::map<JointGroup, GripperParams> params() const;

    /**
     * @brief [Non-blocking] Current states data of all enabled grippers.
     * @return Gripper states data mapped by joint group.
     */
    std::map<JointGroup, GripperStates> states() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace flexiv::rdk */

#endif /* FLEXIV_RDK_GRIPPER_HPP_ */
