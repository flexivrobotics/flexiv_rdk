/**
 * @file Model.hpp
 * @copyright Copyright (C) 2016-2021 Flexiv Ltd. All Rights Reserved.
 */

#ifndef FLEXIVRDK_MODEL_HPP_
#define FLEXIVRDK_MODEL_HPP_

#include <Eigen/Eigen>
#include <memory>
#include <vector>

namespace flexiv {

class ModelHandler;

/**
 * @class Model
 * @brief Integrated dynamics engine with robot model and dynamics.
 */
class Model
{
public:
    Model();
    virtual ~Model();

    /**
     * @brief Update robot model using new joint states data
     * @param[in] positions \f$ \mathbb{R}^{Dof \times 1} \f$ new link positions
     * \f$ q~[rad] \f$
     * @param[in] velocities \f$ \mathbb{R}^{Dof \times 1} \f$ new link
     * velocities \f$ \dot{q}~[rad/s] \f$
     * @return True: success, false: failed
     */
    bool updateModel(const std::vector<double>& positions,
        const std::vector<double>& velocities);

    /**
     * @brief Set tool configuration and add to robot model. The tool is
     * installed on the flange
     * @param[in] mass Total mass of the tool \f$ [kg] \f$
     * @param[in] inertiaAtCom \f$ \mathbb{R}^{3 \times 3} \f$ inertia matrix of
     * the tool at COM \f$ [kg \cdot m^2] \f$
     * @param[in] comInTcp \f$ \mathbb{R}^{3 \times 1} \f$ tool COM position in
     * TCP frame \f$ [m] \f$
     * @param[in] tcpInFlange \f$ \mathbb{R}^{3 \times 1} \f$ TCP position in
     * flange frame \f$ [m] \f$
     * @return True: success, false: failed
     */
    bool setTool(double mass, const Eigen::Matrix3d& inertiaAtCom,
        const Eigen::Vector3d& comInTcp, const Eigen::Vector3d& tcpInFlange);

    /**
     * @brief Compute and get the Jacobian matrix at the frame of the specified
     * link \f$ i \f$, expressed in the base frame.
     * @param[in] linkName Name of the link to get Jacobian for
     * @return \f$ \mathbb{R}^{6 \times Dof} \f$ Jacobian matrix, \f$ ^0 J_i \f$
     * @note Use updateModel() to update robot states first before calling
     * this function
     * @note Available links can be found in the provided URDF. They are
     * {"base_link", "link1", "link2", "link3", "link4", "link5", "link6",
     * "link7", "flange"}, plus "tool" after setTool() is called
     */
    const Eigen::MatrixXd getJacobian(const std::string& linkName);

    /**
     * @brief Compute and get the time derivative of Jacobian matrix at the
     * frame of the specified link \f$ i \f$, expressed in the base frame.
     * @param[in] linkName Name of the link to get Jacobian derivative for
     * @return \f$ \mathbb{R}^{6 \times Dof} \f$ Time derivative of Jacobian
     * matrix, \f$ ^0 \dot{J_i} \f$
     * @note Use updateModel() to update robot states first before calling
     * this function
     * @note Available links can be found in the provided URDF. They are
     * {"base_link", "link1", "link2", "link3", "link4", "link5", "link6",
     * "link7", "flange"}, plus "tool" after setTool() is called
     */
    const Eigen::MatrixXd getJacobianDot(const std::string& linkName);

    /**
     * @brief Compute and get the mass matrix for the generalized coordinates,
     * i.e. joint space
     * @return \f$ \mathbb{S}^{Dof \times Dof}_{++} \f$ Symmetric positive
     * definite mass matrix \f$ M(q)~[kgm^2] \f$
     * @note Use updateModel() to update robot states first before calling
     * this function
     */
    const Eigen::MatrixXd getMassMatrix();

    /**
     * @brief Compute and get the Coriolis/centripetal matrix for the
     * generalized coordinates, i.e. joint space
     * @return \f$ \mathbb{R}^{Dof \times Dof} \f$ Coriolis/centripetal matrix
     * \f$ C(q,\dot{q}) \f$
     * @note Use updateModel() to update robot states first before calling
     * this function
     * @par Coriolis matrix factorization
     * The factorization of the Coriolis matrix C is not unique, and this API
     * is using the factorization method found in "A new Coriolis matrix
     * factorization", 2012 by M. Bjerkend and K. Pettersen
     */
    const Eigen::MatrixXd getCoriolisMatrix();

    /**
     * @brief Compute and get the gravity force vector for the generalized
     * coordinates, i.e. joint space
     * @return \f$ \mathbb{R}^{Dof \times 1} \f$ gravity force vector \f$
     * g(q)~[Nm] \f$
     * @note Use updateModel() to update robot states first before calling
     * this function
     */
    const Eigen::VectorXd getGravityForce();

    /**
     * @brief Compute and get the Coriolis force vector for the generalized
     * coordinates, i.e. joint space
     * @return \f$ \mathbb{R}^{Dof \times 1} \f$ Coriolis force vector \f$
     * c(q,\dot{q})~[Nm] \f$
     * @note Use updateModel() to update robot states first before calling
     * this function
     */
    const Eigen::VectorXd getCoriolisForce();

    friend class RobotClientHandler;

private:
    std::unique_ptr<ModelHandler> m_handler;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_MODEL_HPP_ */
