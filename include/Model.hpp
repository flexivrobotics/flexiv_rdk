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

    /** Body frames of the robot */
    enum Frame
    {
        BASE,
        JOINT_1,
        JOINT_2,
        JOINT_3,
        JOINT_4,
        JOINT_5,
        JOINT_6,
        JOINT_7,
        FLANGE,
        TCP,
    };

    /**
     * @brief Update robot model using new joint states data
     * @param[in] positions \f$ \mathbb{R}^{Dof \times 1} \f$ new positions
     * of all joints, \f$ q~[rad] \f$
     * @param[in] velocities \f$ \mathbb{R}^{Dof \times 1} \f$ new velocities
     * of all joints, \f$ \dot{q}~[rad/s] \f$
     * @return True: success, false: failed
     */
    bool updateModel(const std::vector<double>& positions,
        const std::vector<double>& velocities);

    /**
     * @brief Set tool configuration and add to robot model, append right after
     * flange
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
     * @brief Compute and get the Jacobian matrix for the specified frame
     * relative to the base frame, using the latest robot states
     * @param[in] frame The specified frame to get the Jacobian of
     * @return \f$ \mathbb{R}^{6 \times Dof} \f$ Jacobian matrix relative to the
     * base frame
     * @note Use updateModel() to update robot states first before calling this
     */
    const Eigen::MatrixXd getJacobian(const Model::Frame& frame);

    /**
     * @brief Compute and get the time derivative of Jacobian matrix for the
     * specified frame relative to the base frame, using the latest robot states
     * @param[in] frame The specified frame to get Jacobian derivative for
     * @return \f$ \mathbb{R}^{6 \times Dof} \f$ Jacobian matrix derivative
     * relative to the base frame
     * @note Use updateModel() to update robot states first before calling
     * this
     */
    const Eigen::MatrixXd getJacobianDot(const Model::Frame& frame);

    /**
     * @brief Compute and get the mass matrix for the generalized coordinates,
     * i.e. joint space
     * @return \f$ \mathbb{S}^{Dof \times Dof}_{++} \f$ Symmetric positive
     * definite mass matrix \f$ [kgm^2] \f$
     * @note The latest robot states are used in the computation, so need to
     * call updateModel() to update robot states first before calling this
     */
    const Eigen::MatrixXd getMassMatrix();

    /**
     * @brief Compute and get the gravity force vector for the generalized
     * coordinates, i.e. joint space
     * @return \f$ \mathbb{R}^{Dof \times 1} \f$ gravity force vector \f$ [Nm]
     * \f$
     * @note The latest robot states are used in the computation, so need to
     * call updateModel() to update robot states first before calling this
     */
    const Eigen::VectorXd getGravityForce();

    /**
     * @brief Compute and get the Coriolis force vector for the generalized
     * coordinates, i.e. joint space
     * @return \f$ \mathbb{R}^{Dof \times 1} \f$ Coriolis force vector \f$ [Nm]
     * \f$
     * @note The latest robot states are used in the computation, so need to
     * call updateModel() to update robot states first before calling this
     */
    const Eigen::VectorXd getCoriolisForce();

    friend class RobotClientHandler;

private:
    std::unique_ptr<ModelHandler> m_handler;
};

} /* namespace flexiv */

#endif /* FLEXIVRDK_MODEL_HPP_ */
