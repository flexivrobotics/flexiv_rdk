// cppimport
#include <Log.hpp>
#include <Robot.hpp>
#include <Visualization.hpp>

#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/functional.h>
#include <pybind11/stl.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;
using namespace pybind11::literals;

PYBIND11_MODULE(pylibflexiv, m) {
    m.doc() = R"pbdoc(
        Pylibflexiv: a Python binding for libflexiv
        -----------------------

        .. currentmodule:: pylibflexiv

        .. autosummary::
           :toctree: _generate

    )pbdoc";
    
    // Log.hpp
    py::class_<flexiv::Log>(m, "Log")
        .def(py::init<>())
        .def("info", &flexiv::Log::info)
        .def("warn", &flexiv::Log::warn)
        .def("error", &flexiv::Log::error);

    // Mode.hpp
    py::enum_<flexiv::Mode>(m, "Mode")
        .value("MODE_UNKNOWN", flexiv::Mode::MODE_UNKNOWN)
        .value("MODE_IDLE", flexiv::Mode::MODE_IDLE)
        .value("MODE_JOINT_POSITION", flexiv::Mode::MODE_JOINT_POSITION)
        .value("MODE_JOINT_TORQUE", flexiv::Mode::MODE_JOINT_TORQUE)
        .value("MODE_PLAN_EXECUTION", flexiv::Mode::MODE_PLAN_EXECUTION)
        .value("MODE_PRIMITIVE_EXECUTION", flexiv::Mode::MODE_PRIMITIVE_EXECUTION)
        .value("MODE_CARTESIAN_IMPEDANCE", flexiv::Mode::MODE_CARTESIAN_IMPEDANCE);

    // Moddl.hpp
    py::class_<flexiv::Model>(m, "Model")
        .def(py::init<>())
        .def("updateModel", &flexiv::Model::updateModel)
        .def("setTool", &flexiv::Model::setTool)
        .def("getJacobian", &flexiv::Model::getJacobian)
        .def("getJacobianDot", &flexiv::Model::getJacobianDot)
        .def("getMassMatrix", &flexiv::Model::getMassMatrix)
        .def("getCoriolisMatrix", &flexiv::Model::getCoriolisMatrix)
        .def("getGravityForce", &flexiv::Model::getGravityForce)
        .def("getCoriolisForce", &flexiv::Model::getCoriolisForce);

    // Robot.hpp
    py::class_<flexiv::Robot>(m, "Robot")
        .def(py::init<>())
        .def("init", &flexiv::Robot::init)
        .def("loadModel", &flexiv::Robot::loadModel)
        .def("enable", &flexiv::Robot::enable)
        .def("start", &flexiv::Robot::start)
        .def("stop", &flexiv::Robot::stop)
        .def("isOperational", &flexiv::Robot::isOperational)
        .def("isConnected", &flexiv::Robot::isConnected)
        .def("isRecoveryState", &flexiv::Robot::isRecoveryState)
        .def("timeout", &flexiv::Robot::timeout)
        .def("connect", &flexiv::Robot::connect)
        .def("disconnect", &flexiv::Robot::disconnect)
        .def("isFault", &flexiv::Robot::isFault)
        .def("clearFault", &flexiv::Robot::clearFault)
        .def("setMode", &flexiv::Robot::setMode)
        .def("getMode", &flexiv::Robot::getMode)
        .def("getRobotStates", &flexiv::Robot::getRobotStates)
        .def("getSystemStatus", &flexiv::Robot::getSystemStatus)
        .def("executePlanByIndex", &flexiv::Robot::executePlanByIndex)
        .def("executePlanByName", &flexiv::Robot::executePlanByName)
        .def("getPlanNameList", &flexiv::Robot::getPlanNameList)
        .def("getPlanInfo", &flexiv::Robot::getPlanInfo)
        .def("executePrimitive", &flexiv::Robot::executePrimitive)
        .def("switchTcp", &flexiv::Robot::switchTcp)
        .def("startAutoRecovery", &flexiv::Robot::startAutoRecovery)
        .def("streamJointTorque", &flexiv::Robot::streamJointTorque)
        .def("streamJointPosition", &flexiv::Robot::streamJointPosition)
        .def("streamTcpPose", &flexiv::Robot::streamTcpPose)
        .def("isStopped", &flexiv::Robot::isStopped)
        .def("writeDigitalOutput", &flexiv::Robot::writeDigitalOutput)
        .def("readDigitalInput", &flexiv::Robot::readDigitalInput);

    // RobotStates.hpp
    py::class_<flexiv::RobotStates>(m, "RobotStates")
        .def_readwrite("m_q", &flexiv::RobotStates::m_q)
        .def_readwrite("m_theta", &flexiv::RobotStates::m_theta)
        .def_readwrite("m_dq", &flexiv::RobotStates::m_dq)
        .def_readwrite("m_dtheta", &flexiv::RobotStates::m_dtheta)
        .def_readwrite("m_tau", &flexiv::RobotStates::m_tau)
        .def_readwrite("m_tauDes", &flexiv::RobotStates::m_tauDes)
        .def_readwrite("m_tauDot", &flexiv::RobotStates::m_tauDot)
        .def_readwrite("m_tauExt", &flexiv::RobotStates::m_tauExt)
        .def_readwrite("m_tcpPose", &flexiv::RobotStates::m_tcpPose)
        .def_readwrite("m_tcpPoseDes", &flexiv::RobotStates::m_tcpPoseDes)
        .def_readwrite("m_camPose", &flexiv::RobotStates::m_camPose)
        .def_readwrite("m_flangePose", &flexiv::RobotStates::m_flangePose)
        .def_readwrite("m_endLinkPose", &flexiv::RobotStates::m_endLinkPose)
        .def_readwrite("m_extForceInTcpFrame", &flexiv::RobotStates::m_extForceInTcpFrame)
        .def_readwrite("m_extForceInBaseFrame", &flexiv::RobotStates::m_extForceInBaseFrame);

    py::class_<flexiv::SystemStatus>(m, "SystemStatus")
        .def_readwrite("m_emergencyStop", &flexiv::SystemStatus::m_emergencyStop)
        .def_readwrite("m_externalActive", &flexiv::SystemStatus::m_externalActive)
        .def_readwrite("m_programRequest", &flexiv::SystemStatus::m_programRequest)
        .def_readwrite("m_programRunning", &flexiv::SystemStatus::m_programRunning)
        .def_readwrite("m_reachedTarget", &flexiv::SystemStatus::m_reachedTarget)
        .def_readwrite("m_motionCmdSuccessRate", &flexiv::SystemStatus::m_motionCmdSuccessRate)
        .def_readwrite("m_errorMsg", &flexiv::SystemStatus::m_errorMsg)
        .def_readwrite("m_jntLimitTriggered", &flexiv::SystemStatus::m_jntLimitTriggered)
        .def_readwrite("m_ptStates", &flexiv::SystemStatus::m_ptStates);

    py::class_<flexiv::PlanInfo>(m, "PlanInfo")
        .def_readwrite("m_ptName", &flexiv::PlanInfo::m_ptName)
        .def_readwrite("m_nodeName", &flexiv::PlanInfo::m_nodeName)
        .def_readwrite("m_nodePath", &flexiv::PlanInfo::m_nodePath)
        .def_readwrite("m_nodePathTimePeriod", &flexiv::PlanInfo::m_nodePathTimePeriod)
        .def_readwrite("m_nodePathNumber", &flexiv::PlanInfo::m_nodePathNumber)
        .def_readwrite("m_assignedPlanName", &flexiv::PlanInfo::m_assignedPlanName)
        .def_readwrite("m_velocityScale", &flexiv::PlanInfo::m_velocityScale);

    // Visualization.hpp
    py::class_<flexiv::Visualization>(m, "Visualization")
        .def(py::init<>())
        .def("init", py::overload_cast<const std::string &>(&flexiv::Visualization::init))
        .def("init", py::overload_cast<const std::string &, const std::string &>(&flexiv::Visualization::init))
        .def("update", &flexiv::Visualization::update);

#ifdef VERSION_INFO
    m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
    m.attr("__version__") = "dev";
#endif
}

<%
cfg['compiler_args'] = ['-std=c++14']
cfg['libraries'] = ['FlexivRdk']
setup_pybind11(cfg)
%>
