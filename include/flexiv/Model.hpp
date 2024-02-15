/**
 * @file Model.hpp
 * @copyright Copyright (C) 2016-2023 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_MODEL_HPP_
#define FLEXIVRDK_MODEL_HPP_

#include "Robot.hpp"
#include <Eigen/Eigen>
#include <memory>

namespace flexiv {

/**
 * @class Model
 * @brief Robot model with integrated dynamics engine.
 */
class Model
{
public:
    /**
     * @brief [Non-blocking] Create an instance and initialize the integrated dynamics engine.
     * @param[in] robot Reference to the instance of flexiv::Robot.
     * @param[in] gravityEarth Earth's gravity vector in base frame. Default to \f$ [0.0, 0.0,
     * -9.81]^T \f$. Unit: \f$ [m/s^2] \f$.
     * @throw std::runtime_error if the initialization sequence failed.
     * @throw std::logic_error if the connected robot does not have an RDK professional license; or
     * the parsed robot model is not supported.
     */
    Model(
        const Robot& robot, const Eigen::Vector3d& gravityEarth = Eigen::Vector3d(0.0, 0.0, -9.81));
    virtual ~Model();

    /**
     * @brief [Blocking] Reload robot model using the latest data synced from the connected robot.
     * Tool model is also synced.
     * @throw std::runtime_error if failed to sync model data.
     * @throw std::logic_error if the synced robot model contains invalid data.
     * @note This function blocks until the model data is synced and the reloading is finished.
     * @note Call this function if the robot tool has changed.
     */
    void reload();

    /**
     * @brief [Non-blocking] Update robot model using new joint states data.
     * @param[in] positions Current joint positions: \f$ q \in \mathbb{R}^{n \times 1} \f$. Unit:
     * \f$ [rad] \f$.
     * @param[in] velocities Current joint velocities: \f$ \dot{q} \in \mathbb{R}^{n \times 1}
     * \f$. Unit: \f$ [rad/s] \f$.
     */
    void update(const std::array<double, k_jointDOF>& positions,
        const std::array<double, k_jointDOF>& velocities);

    /**
     * @brief [Non-blocking] Compute and get the Jacobian matrix at the frame of the specified link
     * \f$ i \f$, expressed in base frame.
     * @param[in] linkName Name of the link to get Jacobian for.
     * @return Jacobian matrix: \f$ ^{0}J_i \in \mathbb{R}^{m \times n} \f$.
     * @note Call update() before this method.
     * @note Available links can be found in the provided URDF. They are {"base_link", "link1",
     * "link2", "link3", "link4", "link5", "link6", "link7", "flange"}, plus "tool" if any flange
     * tool is mounted.
     * @throw std::out_of_range if the specified linkName does not exist.
     */
    const Eigen::MatrixXd J(const std::string& linkName);
    [[deprecated("Use J() instead")]] const Eigen::MatrixXd getJacobian(
        const std::string& linkName);

    /**
     * @brief [Non-blocking] Compute and get the time derivative of Jacobian matrix at the frame of
     * the specified link \f$ i \f$, expressed in base frame.
     * @param[in] linkName Name of the link to get Jacobian derivative for.
     * @return Time derivative of Jacobian matrix: \f$ ^{0}\dot{J_i} \in \mathbb{R}^{m \times n}
     * \f$.
     * @note Call update() before this method.
     * @note Available links can be found in the provided URDF. They are {"base_link", "link1",
     * "link2", "link3", "link4", "link5", "link6", "link7", "flange"}, plus "tool" if any flange
     * tool is mounted.
     * @throw std::out_of_range if the specified linkName does not exist.
     */
    const Eigen::MatrixXd dJ(const std::string& linkName);
    [[deprecated("Use dJ() instead")]] const Eigen::MatrixXd getJacobianDot(
        const std::string& linkName);

    /**
     * @brief [Non-blocking] Compute and get the mass matrix for the generalized coordinates, i.e.
     * joint space.
     * @return Symmetric positive definite mass matrix: \f$ M(q) \in \mathbb{S}^{n \times n}_{++}
     * \f$. Unit: \f$ [kgm^2] \f$.
     * @note Call update() before this method.
     */
    const Eigen::MatrixXd M();
    [[deprecated("Use M() instead")]] const Eigen::MatrixXd getMassMatrix();

    /**
     * @brief [Non-blocking] Compute and get the Coriolis/centripetal matrix for the generalized
     * coordinates, i.e. joint space.
     * @return Coriolis/centripetal matrix: \f$ C(q,\dot{q}) \in \mathbb{R}^{n \times n} \f$.
     * @note Call update() before this method.
     */
    const Eigen::MatrixXd C();
    [[deprecated("Use C() instead")]] const Eigen::MatrixXd getCoriolisMatrix();

    /**
     * @brief [Non-blocking] Compute and get the gravity force vector for the generalized
     * coordinates, i.e. joint space.
     * @return Gravity force vector: \f$ g(q) \in \mathbb{R}^{n \times 1} \f$. Unit: \f$ [Nm] \f$.
     * @note Call update() before this method.
     */
    const Eigen::VectorXd g();
    [[deprecated("Use g() instead")]] const Eigen::VectorXd getGravityForce();

    /**
     * @brief [Non-blocking] Compute and get the Coriolis force vector for the generalized
     * coordinates, i.e. joint space.
     * @return Coriolis force vector: \f$ c(q,\dot{q}) \in \mathbb{R}^{n \times 1} \f$. Unit: \f$
     * [Nm] \f$.
     * @note Call update() before this method.
     */
    const Eigen::VectorXd c();
    [[deprecated("Use c() instead")]] const Eigen::VectorXd getCoriolisForce();

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_MODEL_HPP_ */
