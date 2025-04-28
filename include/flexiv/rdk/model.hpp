/**
 * @file model.hpp
 * @copyright Copyright (C) 2016-2024 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIV_RDK_MODEL_HPP_
#define FLEXIV_RDK_MODEL_HPP_

#include "robot.hpp"
#include <Eigen/Eigen>
#include <memory>

namespace flexiv {
namespace rdk {

/**
 * @class Model
 * @brief Interface to obtain certain model data of the robot, including kinematics and dynamics.
 */
class Model
{
public:
    /**
     * @brief [Non-blocking] Instantiate the robot model interface.
     * @param[in] robot Reference to the instance of flexiv::rdk::Robot.
     * @param[in] gravity_vector Earth's gravity vector in world frame. Default to \f$ [0.0, 0.0,
     * -9.81]^T \f$. Unit: \f$ [m/s^2] \f$.
     * @throw std::runtime_error if the initialization sequence failed.
     * @throw std::logic_error if the connected robot does not have an RDK professional license; or
     * the parsed robot model is not supported.
     */
    Model(const Robot& robot,
        const Eigen::Vector3d& gravity_vector = Eigen::Vector3d(0.0, 0.0, -9.81));
    virtual ~Model();

    //========================================== DYNAMICS ==========================================
    /**
     * @brief [Blocking] Reload (refresh) parameters of the robot model stored locally in this class
     * using the latest data synced from the connected robot. Tool model is also synced.
     * @throw std::runtime_error if failed to sync model data.
     * @throw std::logic_error if the synced robot model contains invalid data.
     * @note This function blocks until the model parameters are synced and reloaded.
     * @note This function does not affect the kinematics functions.
     * @warning Parameters of the locally-stored robot model must be manually refreshed using this
     * function whenever a physical change is made to the connected robot (e.g. a tool is added or
     * changed). Otherwise all dynamics functions will return incorrect results.
     */
    void Reload();

    /**
     * @brief [Non-blocking] Update the configuration (posture) of the locally-stored robot model
     * so that the dynamics functions return results based on the updated configuration.
     * @param[in] positions Current joint positions: \f$ q \in \mathbb{R}^{n \times 1} \f$. Unit:
     * \f$ [rad] \f$.
     * @param[in] velocities Current joint velocities: \f$ \dot{q} \in \mathbb{R}^{n \times 1}
     * \f$. Unit: \f$ [rad/s] \f$.
     * @throw std::invalid_argument if size of any input vector does not match robot DoF.
     * @note This function does not affect the kinematics functions.
     */
    void Update(const std::vector<double>& positions, const std::vector<double>& velocities);

    /**
     * @brief [Non-blocking] Compute and get the Jacobian matrix at the frame of the specified link
     * \f$ i \f$, expressed in world frame.
     * @param[in] link_name Name of the link to get Jacobian for.
     * @return Jacobian matrix: \f$ ^{0}J_i \in \mathbb{R}^{m \times n} \f$.
     * @throw std::out_of_range if the specified link_name does not exist.
     * @note Call Update() before this function.
     * @note Available links can be found in the provided URDF. They are {"base_link", "link1",
     * "link2", "link3", "link4", "link5", "link6", "link7", "flange"}, plus "tool" if any flange
     * tool is mounted.
     */
    Eigen::MatrixXd J(const std::string& link_name);

    /**
     * @brief [Non-blocking] Compute and get the time derivative of Jacobian matrix at the frame of
     * the specified link \f$ i \f$, expressed in world frame.
     * @param[in] link_name Name of the link to get Jacobian derivative for.
     * @return Time derivative of the Jacobian matrix: \f$ ^{0}\dot{J_i} \in \mathbb{R}^{m \times n}
     * \f$.
     * @throw std::out_of_range if the specified link_name does not exist.
     * @note Call Update() before this function.
     * @note Available links can be found in the provided URDF. They are {"base_link", "link1",
     * "link2", "link3", "link4", "link5", "link6", "link7", "flange"}, plus "tool" if any flange
     * tool is mounted.
     */
    Eigen::MatrixXd dJ(const std::string& link_name);

    /**
     * @brief [Non-blocking] Compute and get the mass matrix for the generalized coordinates, i.e.
     * joint space.
     * @return Symmetric positive definite mass matrix: \f$ M(q) \in \mathbb{S}^{n \times n}_{++}
     * \f$. Unit: \f$ [kgm^2] \f$.
     * @note Call Update() before this function.
     */
    Eigen::MatrixXd M();

    /**
     * @brief [Non-blocking] Compute and get the Coriolis/centripetal matrix for the generalized
     * coordinates, i.e. joint space.
     * @return Coriolis/centripetal matrix: \f$ C(q,\dot{q}) \in \mathbb{R}^{n \times n} \f$.
     * @note Call Update() before this function.
     */
    Eigen::MatrixXd C();

    /**
     * @brief [Non-blocking] Compute and get the gravity force vector for the generalized
     * coordinates, i.e. joint space.
     * @return Gravity force vector: \f$ g(q) \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm] \f$.
     * @note Call Update() before this function.
     */
    Eigen::VectorXd g();

    /**
     * @brief [Non-blocking] Compute and get the Coriolis force vector for the generalized
     * coordinates, i.e. joint space.
     * @return Coriolis force vector: \f$ c(q,\dot{q}) \in \mathbb{R}^{n \times 1} \f$. Unit: \f$
     * [Nm] \f$.
     * @note Call Update() before this function.
     */
    Eigen::VectorXd c();

    //========================================= KINEMATICS =========================================
    /**
     * @brief [Blocking] Sync the actual kinematic parameters of the connected robot into the
     * template URDF.
     * @param[in] template_urdf_path Path to the template URDF in [flexiv_rdk/resources] directory.
     * This template URDF will be updated when the sync is finished.
     * @throw std::invalid_argument if failed to load the template URDF.
     * @throw std::runtime_error if failed to sync the URDF.
     * @note This function blocks until the URDF syncing is finished.
     * @par Why is this function needed?
     * The URDFs in [flexiv_rdk/resources] directory contain kinematic parameters of the latest
     * robot hardware version, which might be different from older versions. This function is
     * therefore provided to sync the actual kinematic parameters of the connected robot into the
     * template URDF.
     */
    void SyncURDF(const std::string& template_urdf_path);

    /**
     * @brief [Blocking] Check if a Cartesian pose is reachable. If yes, also return an IK solution
     * of the corresponding joint positions.
     * @param[in] pose Cartesian pose to be checked.
     * @param[in] seed_positions Joint positions to be used as the seed for solving IK.
     * @param[in] free_orientation Only constrain position and allow orientation to move freely.
     * @return A pair of {is_reachable, ik_solution}.
     * @throw std::invalid_argument if size of [seed_positions] does not match robot DoF.
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     */
    std::pair<bool, std::vector<double>> reachable(const std::array<double, kPoseSize>& pose,
        const std::vector<double>& seed_positions, bool free_orientation) const;

    /**
     * @brief [Blocking] Score of the robot's current configuration (posture), calculated from the
     * manipulability measurements.
     * @return A pair of {translation_score, orientation_score}. The quality of configuration based
     * on score is mapped as: poor = [0, 20), medium = [20, 40), good = [40, 100].
     * @throw std::runtime_error if failed to get a reply from the connected robot.
     * @note This function blocks until a reply is received.
     * @warning A poor configuration score means the robot is near or at singularity, which can lead
     * to degraded Cartesian performance. Use configuration with high scores for better
     * manipulability and task results.
     */
    std::pair<double, double> configuration_score() const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};

} /* namespace rdk */
} /* namespace flexiv */

#endif /* FLEXIV_RDK_MODEL_HPP_ */
