/**
 * @file Gripper.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_GRIPPER_HPP_
#define FLEXIVRDK_GRIPPER_HPP_

#include "Robot.hpp"
#include <memory>

namespace flexiv {

/**
 * @class Gripper
 * @brief Interface with grippers that use the communication protocol template
 * provided by Flexiv.
 */
class Gripper
{
public:
    /**
     * @brief Create a flexiv::Gripper instance for gripper control.
     * @param[in] robot Pointer to the instance of flexiv::Robot main interface.
     * @throw InitException if the instance failed to initialize.
     */
    Gripper(Robot* robot);
    virtual ~Gripper();

    /**
     * @brief Grasp with direct force control.
     * @param[in] force Target gripping force. Positive direction: closing [N].
     * @return True: success, false: failed.
     * @note Applicable modes: all modes except MODE_IDLE.
     * @warning Target inputs outside the valid range (specified in gripper's
     * configuration file) will be saturated.
     * @throw ExecutionException if error occurred during execution.
     */
    void grasp(double force);

    /**
     * @brief Move the gripper fingers with position control.
     * @param[in] width Target opening width [m].
     * @param[in] velocity Closing/opening velocity, cannot be 0 [m/s].
     * @return True: success, false: failed.
     * @note Applicable modes: all modes except MODE_IDLE.
     * @warning Target inputs outside the valid range (specified in gripper's
     * configuration file) will be saturated.
     * @throw ExecutionException if error occurred during execution.
     */
    void move(double width, double velocity);

    /**
     * @brief Stop the gripper.
     * @return True: success, false: failed.
     * @note Applicable modes: all modes.
     * @throw ExecutionException if error occurred during execution.
     */
    void stop(void);

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_GRIPPER_HPP_ */
