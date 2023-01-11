/**
 * @file Model.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_MODEL_HPP_
#define FLEXIVRDK_MODEL_HPP_

#include "Robot.hpp"

#include <Eigen/Eigen>
#include <memory>
#include <vector>

namespace flexiv {

/**
 * @class Model
 * @brief Robot model with integrated dynamics engine.
 */
class Model
{
public:
    /**
     * @brief Create a flexiv::Model instance to get robot dynamics data.
     * @param[in] robot Reference to the instance of flexiv::Robot.
     * @param[in] gravityEarth Earth's gravity vector in base frame. Default
     * to \f$ [0.0, 0.0, -9.81]^T~[m/s^2] \f$.
     * @throw InitException if the instance failed to initialize.
     */
    Model(const Robot& robot,
        const Eigen::Vector3d& gravityEarth = Eigen::Vector3d(0.0, 0.0, -9.81));
    virtual ~Model();

    /**
     * @brief Update robot model using new joint states data.
     * @param[in] positions \f$ \mathbb{R}^{Dof \times 1} \f$ new link positions
     * \f$ q~[rad] \f$.
     * @param[in] velocities \f$ \mathbb{R}^{Dof \times 1} \f$ new link
     * velocities \f$ \dot{q}~[rad/s] \f$.
     * @throw InputException if the input vector is of wrong size.
     */
    void updateModel(const std::vector<double>& positions,
        const std::vector<double>& velocities);

    /**
     * @brief Compute and get the Jacobian matrix at the frame of the specified
     * link \f$ i \f$, expressed in the base frame.
     * @param[in] linkName Name of the link to get Jacobian for.
     * @return \f$ \mathbb{R}^{6 \times Dof} \f$ Jacobian matrix,
     * \f$ ^0 J_i \f$.
     * @note Call updateModel() before this method.
     * @note Available links can be found in the provided URDF. They are
     * {"base_link", "link1", "link2", "link3", "link4", "link5", "link6",
     * "link7", "flange"}, plus "tool" if any flange tool is mounted.
     * @throw InputException if the specified linkName does not exist.
     */
    const Eigen::MatrixXd getJacobian(const std::string& linkName);

    /**
     * @brief Compute and get the time derivative of Jacobian matrix at the
     * frame of the specified link \f$ i \f$, expressed in the base frame.
     * @param[in] linkName Name of the link to get Jacobian derivative for.
     * @return \f$ \mathbb{R}^{6 \times Dof} \f$ Time derivative of Jacobian
     * matrix, \f$ ^0 \dot{J_i} \f$.
     * @note Call updateModel() before this method.
     * @note Available links can be found in the provided URDF. They are
     * {"base_link", "link1", "link2", "link3", "link4", "link5", "link6",
     * "link7", "flange"}, plus "tool" if any flange tool is mounted.
     * @throw InputException if the specified linkName does not exist.
     */
    const Eigen::MatrixXd getJacobianDot(const std::string& linkName);

    /**
     * @brief Compute and get the mass matrix for the generalized coordinates,
     * i.e. joint space.
     * @return \f$ \mathbb{S}^{Dof \times Dof}_{++} \f$ Symmetric positive
     * definite mass matrix \f$ M(q)~[kgm^2] \f$.
     * @note Call updateModel() before this method.
     */
    const Eigen::MatrixXd getMassMatrix();

    /**
     * @brief Compute and get the Coriolis/centripetal matrix for the
     * generalized coordinates, i.e. joint space.
     * @return \f$ \mathbb{R}^{Dof \times Dof} \f$ Coriolis/centripetal matrix
     * \f$ C(q,\dot{q}) \f$.
     * @note Call updateModel() before this method.
     */
    const Eigen::MatrixXd getCoriolisMatrix();

    /**
     * @brief Compute and get the gravity force vector for the generalized
     * coordinates, i.e. joint space.
     * @return \f$ \mathbb{R}^{Dof \times 1} \f$ gravity force vector \f$
     * g(q)~[Nm] \f$.
     * @note Call updateModel() before this method.
     */
    const Eigen::VectorXd getGravityForce();

    /**
     * @brief Compute and get the Coriolis force vector for the generalized
     * coordinates, i.e. joint space.
     * @return \f$ \mathbb{R}^{Dof \times 1} \f$ Coriolis force vector \f$
     * c(q,\dot{q})~[Nm] \f$.
     * @note Call updateModel() before this method.
     */
    const Eigen::VectorXd getCoriolisForce();

private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_MODEL_HPP_ */
