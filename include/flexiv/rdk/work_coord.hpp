/**
 * @file work_coord.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_WORK_COORD_HPP_
#define FLEXIV_RDK_WORK_COORD_HPP_

#include "robot.hpp"

namespace flexiv {
namespace rdk {

/**
 * @class WorkCoord
 * @brief Interface to online update and interact with the robot's work coordinates. All updates
 * will take effect immediately without a power cycle. However, the robot must be in IDLE mode when
 * applying changes.
 */
class WorkCoord
{
public:
    /**
     * @brief [Non-blocking] Create an instance and initialize the interface.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @throw std::runtime_error if the initialization sequence failed.
     */
    WorkCoord(const Robot& robot);
    virtual ~WorkCoord();

    /**
     * @brief [Blocking] Get a name list of all configured work coordinates.
     * @return Work coordinate names as a string list.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    const std::vector<std::string> list() const;

    /**
     * @brief [Blocking] Whether the specified work coordinate already exists.
     * @param[in] name Name of the tool to check.
     * @return True if the specified tool exists.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    bool exist(const std::string& name) const;

    /**
     * @brief [Blocking] Get pose of an existing work coordinate.
     * @param[in] name Name of the work coordinate to get pose for, must be an existing one.
     * @return Pose of the work coordinate in world frame: \f$ ^{O}T_{work} \in \mathbb{R}^{7 \times
     * 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$ \mathbb{R}^{4 \times 1}
     * \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$ [m]:[] \f$.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    const std::array<double, kPoseSize> pose(const std::string& name) const;

    /**
     * @brief [Blocking] Add a new work coordinate with user-specified parameters.
     * @param[in] name Name of the new work coordinate, must be unique.
     * @param[in] pose Pose of the new work coordinate in world frame: \f$ ^{O}T_{work} \in
     * \mathbb{R}^{7 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$
     * \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$
     * [m]:[] \f$.
     * @throw std::invalid_argument if the specified work coordinate already exists.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     */
    void Add(const std::string& name, const std::array<double, kPoseSize>& pose);

    /**
     * @brief [Blocking] Update the pose of an existing work coordinate.
     * @param[in] name Name of the work coordinate to update, must be an existing one.
     * @param[in] pose New pose for the specified work coordinate in world frame: \f$ ^{O}T_{work}
     * \in \mathbb{R}^{7 \times 1} \f$. Consists of \f$ \mathbb{R}^{3 \times 1} \f$ position and \f$
     * \mathbb{R}^{4 \times 1} \f$ quaternion: \f$ [x, y, z, q_w, q_x, q_y, q_z]^T \f$. Unit: \f$
     * [m]:[] \f$.
     * @throw std::invalid_argument if the specified work coordinate does not exist.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     */
    void Update(const std::string& name, const std::array<double, kPoseSize>& pose);

    /**
     * @brief [Blocking] Remove an existing work coordinate.
     * @param[in] name Name of the work coordinate to remove, must be an existing one.
     * @throw std::invalid_argument if the specified work coordinate does not exist.
     * @throw std::logic_error if robot is not in the correct control mode.
     * @throw std::runtime_error if failed to deliver the request to the connected robot.
     * @note Applicable control modes: IDLE.
     * @note This function blocks until the request is successfully delivered.
     */
    void Remove(const std::string& name);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_WORK_COORD_HPP_ */
