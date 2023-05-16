// Standard library
#include <cstdio>
#include <string>

// NumPy: https://numpy.org/
#include <numpy/arrayobject.h>

// pybind: https://pybind11.readthedocs.io/en/stable/
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

// KUKA FRI-Client-SDK_Cpp (using version hosted at:
// https://github.com/cmower/FRI-Client-SDK_Cpp)
#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"
#include "fri_config.h"

#include <memory>

// Make LBRClient a Python abstract class
class PyLBRClient : public KUKA::FRI::LBRClient {

  using KUKA::FRI::LBRClient::LBRClient;

public:
  void onStateChange(KUKA::FRI::ESessionState oldState,
                     KUKA::FRI::ESessionState newState) override {
    PYBIND11_OVERRIDE_PURE(void, LBRClient, onStateChange, oldState, newState);
  }

  void monitor() override { PYBIND11_OVERRIDE_PURE(void, LBRClient, monitor); }

  void waitForCommand() override {
    PYBIND11_OVERRIDE_PURE(void, LBRClient, waitForCommand);
  }

  void command() override { PYBIND11_OVERRIDE_PURE(void, LBRClient, command); }
};

// Wrapper for ClientApplication (does not make sense for the user to
// instantiate UdpConnection on the Python side).
class PyClientApplication {

public:
  PyClientApplication(PyLBRClient &client) {
    _app = std::make_unique<KUKA::FRI::ClientApplication>(_connection, client);
  }

  bool connect(const int port, char *const remoteHost = NULL) {
    return _app->connect(port, remoteHost);
  }

  void disconnect() { _app->disconnect(); }

  bool step() { return _app->step(); }

private:
  KUKA::FRI::UdpConnection _connection;
  std::unique_ptr<KUKA::FRI::ClientApplication> _app;
};

// Python bindings
namespace py = pybind11;

PYBIND11_MODULE(pyFRIClient, m) {
  m.doc() = "Python bindings for the KUKA FRI Client SDK. THIS IS NOT A KUKA "
            "PRODUCT.";

  m.attr("FRI_VERSION_MAJOR") = FRI_VERSION_MAJOR;
  m.attr("FRI_VERSION_MINOR") = FRI_VERSION_MINOR;
  m.attr("FRI_VERSION") = std::to_string(FRI_VERSION_MAJOR) + "." +
                          std::to_string(FRI_VERSION_MINOR);

  py::enum_<KUKA::FRI::ESessionState>(m, "ESessionState")
      .value("IDLE", KUKA::FRI::ESessionState::IDLE)
      .value("MONITORING_WAIT", KUKA::FRI::ESessionState::MONITORING_WAIT)
      .value("MONITORING_READY", KUKA::FRI::ESessionState::MONITORING_READY)
      .value("COMMANDING_WAIT", KUKA::FRI::ESessionState::COMMANDING_WAIT)
      .value("COMMANDING_ACTIVE", KUKA::FRI::ESessionState::COMMANDING_ACTIVE)
      .export_values();

  py::enum_<KUKA::FRI::EConnectionQuality>(m, "EConnectionQuality")
      .value("POOR", KUKA::FRI::EConnectionQuality::POOR)
      .value("FAIR", KUKA::FRI::EConnectionQuality::FAIR)
      .value("GOOD", KUKA::FRI::EConnectionQuality::GOOD)
      .value("EXCELLENT", KUKA::FRI::EConnectionQuality::EXCELLENT)
      .export_values();

  py::enum_<KUKA::FRI::ESafetyState>(m, "ESafetyState")
      .value("NORMAL_OPERATION", KUKA::FRI::ESafetyState::NORMAL_OPERATION)
      .value("SAFETY_STOP_LEVEL_0",
             KUKA::FRI::ESafetyState::SAFETY_STOP_LEVEL_0)
      .value("SAFETY_STOP_LEVEL_1",
             KUKA::FRI::ESafetyState::SAFETY_STOP_LEVEL_1)
      .value("SAFETY_STOP_LEVEL_2",
             KUKA::FRI::ESafetyState::SAFETY_STOP_LEVEL_2)
      .export_values();

  py::enum_<KUKA::FRI::EOperationMode>(m, "EOperationMode")
      .value("TEST_MODE_1", KUKA::FRI::EOperationMode::TEST_MODE_1)
      .value("TEST_MODE_2", KUKA::FRI::EOperationMode::TEST_MODE_2)
      .value("AUTOMATIC_MODE", KUKA::FRI::EOperationMode::AUTOMATIC_MODE)
      .export_values();

  py::enum_<KUKA::FRI::EDriveState>(m, "EDriveState")
      .value("OFF", KUKA::FRI::EDriveState::OFF)
      .value("TRANSITIONING", KUKA::FRI::EDriveState::TRANSITIONING)
      .value("ACTIVE", KUKA::FRI::EDriveState::ACTIVE)
      .export_values();

  py::enum_<KUKA::FRI::EControlMode>(m, "EControlMode")
      .value("POSITION_CONTROL_MODE",
             KUKA::FRI::EControlMode::POSITION_CONTROL_MODE)
      .value("CART_IMP_CONTROL_MODE",
             KUKA::FRI::EControlMode::CART_IMP_CONTROL_MODE)
      .value("JOINT_IMP_CONTROL_MODE",
             KUKA::FRI::EControlMode::JOINT_IMP_CONTROL_MODE)
      .value("NO_CONTROL", KUKA::FRI::EControlMode::NO_CONTROL)
      .export_values();

  py::enum_<KUKA::FRI::EClientCommandMode>(m, "EClientCommandMode")
      .value("NO_COMMAND_MODE", KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE)
      .value("WRENCH", KUKA::FRI::EClientCommandMode::WRENCH)
      .value("TORQUE", KUKA::FRI::EClientCommandMode::TORQUE)
#if FRI_VERSION_MAJOR == 1
      .value("POSITION", KUKA::FRI::EClientCommandMode::POSITION)
#elif FRI_VERSION_MAJOR == 2
      .value("JOINT_POSITION", KUKA::FRI::EClientCommandMode::JOINT_POSITION)
      .value("CARTESIAN_POSE", KUKA::FRI::EClientCommandMode::CARTESIAN_POSE)
#endif
      .export_values();

  py::enum_<KUKA::FRI::EOverlayType>(m, "EOverlayType")
      .value("NO_OVERLAY", KUKA::FRI::EOverlayType::NO_OVERLAY)
      .value("JOINT", KUKA::FRI::EOverlayType::JOINT)
      .value("CARTESIAN", KUKA::FRI::EOverlayType::CARTESIAN)
      .export_values();

#if FRI_VERSION_MAJOR == 2
  py::enum_<KUKA::FRI::ERedundancyStrategy>(m, "ERedundancyStrategy")
      .value("E1", KUKA::FRI::ERedundancyStrategy::E1)
      .value("NO_STRATEGY", KUKA::FRI::ERedundancyStrategy::NO_STRATEGY)
      .export_values();
#endif

  py::class_<KUKA::FRI::LBRState>(m, "LBRState")
      .def(py::init<>())
      .def_property_readonly_static("NUMBER_OF_JOINTS",
                                    [](py::object) {
                                      int num =
                                          KUKA::FRI::LBRState::NUMBER_OF_JOINTS;
                                      return num;
                                    })
      .def("getSampleTime", &KUKA::FRI::LBRState::getSampleTime)
      .def("getSessionState", &KUKA::FRI::LBRState::getSessionState)
      .def("getConnectionQuality", &KUKA::FRI::LBRState::getConnectionQuality)
      .def("getSafetyState", &KUKA::FRI::LBRState::getSafetyState)
      .def("getOperationMode", &KUKA::FRI::LBRState::getOperationMode)
      .def("getDriveState", &KUKA::FRI::LBRState::getDriveState)
      .def("getClientCommandMode", &KUKA::FRI::LBRState::getClientCommandMode)
      .def("getOverlayType", &KUKA::FRI::LBRState::getOverlayType)
      .def("getControlMode", &KUKA::FRI::LBRState::getControlMode)
      .def("getTimestampSec", &KUKA::FRI::LBRState::getTimestampSec)
      .def("getTimestampNanoSec", &KUKA::FRI::LBRState::getTimestampNanoSec)
      .def("getMeasuredJointPosition",
           [](const KUKA::FRI::LBRState &self) {
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             memcpy(data, self.getMeasuredJointPosition(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));
             return py::array_t<double>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                        data);
           })
      .def("getCommandedJointPosition",
           [](const KUKA::FRI::LBRState &self) {
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             memcpy(data, self.getCommandedJointPosition(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));
             return py::array_t<double>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                        data);
           })
      .def("getMeasuredTorque",
           [](const KUKA::FRI::LBRState &self) {
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             memcpy(data, self.getMeasuredTorque(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));
             return py::array_t<double>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                        data);
           })
      .def("getCommandedTorque",
           [](const KUKA::FRI::LBRState &self) {
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             memcpy(data, self.getCommandedTorque(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));
             return py::array_t<double>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                        data);
           })
      .def("getExternalTorque",
           [](const KUKA::FRI::LBRState &self) {
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             memcpy(data, self.getExternalTorque(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));
             return py::array_t<double>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                        data);
           })
      .def("getIpoJointPosition",
           [](const KUKA::FRI::LBRState &self) {
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             memcpy(data, self.getIpoJointPosition(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));
             return py::array_t<double>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                        data);
           })
      .def("getTrackingPerformance",
           &KUKA::FRI::LBRState::getTrackingPerformance)
      .def("getBooleanIOValue", &KUKA::FRI::LBRState::getBooleanIOValue)
      .def("getDigitalIOValue", &KUKA::FRI::LBRState::getDigitalIOValue)
      .def("getAnalogIOValue", &KUKA::FRI::LBRState::getAnalogIOValue)
#if FRI_VERSION_MAJOR == 2
      .def("getMeasuredCartesianPose",
           [](const KUKA::FRI::LBRState &self) {
             double data[3][4];
             memcpy(data, self.getMeasuredCartesianPose(),
                    3 * 4 * sizeof(double));
             py::array_t<double> np_array({3, 4}, &data[0][0]);
             return np_array;
           })
      .def("getMeasuredCartesianPoseAsMatrix",
           [](const KUKA::FRI::LBRState &self) {
             py::array_t<double> result({3, 4});
             auto ptr = result.mutable_data();
             self.getMeasuredCartesianPoseAsMatrix(
                 reinterpret_cast<double(&)[3][4]>(ptr));
             return result;
           })
      .def("getIpoCartesianPose",
           [](const KUKA::FRI::LBRState &self) {
             double data[3][4];
             memcpy(data, self.getIpoCartesianPose(), 3 * 4 * sizeof(double));
             py::array_t<double> np_array({3, 4}, &data[0][0]);
             return np_array;
           })
      .def("getIpoCartesianPoseAsMatrix",
           [](const KUKA::FRI::LBRState &self) {
             py::array_t<double> result({3, 4});
             auto ptr = result.mutable_data();
             self.getIpoCartesianPoseAsMatrix(
                 reinterpret_cast<double(&)[3][4]>(ptr));
             return result;
           })
      .def("getMeasuredRedundancyValue",
           &KUKA::FRI::LBRState::getMeasuredRedundancyValue)
      .def("getIpoRedundancyValue", &KUKA::FRI::LBRState::getIpoRedundancyValue)
      .def("getRedundancyStrategy",
           &KUKA::FRI::LBRState::getRedundancyStrategy)
#endif
      ; // NOTE: this completes LBRState

  py::class_<KUKA::FRI::LBRCommand>(m, "LBRCommand")
      .def(py::init<>())
      .def("setJointPosition",
           [](KUKA::FRI::LBRCommand &self, py::array_t<double> values) {
             if (values.ndim() != 1 ||
                 PyArray_DIMS(values.ptr())[0] !=
                     KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
               throw std::runtime_error(
                   "Input array must have shape (" +
                   std::to_string(KUKA::FRI::LBRState::NUMBER_OF_JOINTS) +
                   ",)!");
             }
             auto buf = values.request();
             const double *data = static_cast<double *>(buf.ptr);
             self.setJointPosition(data);
           })
      // .def("setWrench", &KUKA::FRI::LBRCommand::setWrench)   // TODO
      .def("setTorque",
           [](KUKA::FRI::LBRCommand &self, py::array_t<double> values) {
             if (values.ndim() != 1 ||
                 PyArray_DIMS(values.ptr())[0] !=
                     KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
               throw std::runtime_error(
                   "Input array must have shape (" +
                   std::to_string(KUKA::FRI::LBRState::NUMBER_OF_JOINTS) +
                   ",)!");
             }
             auto buf = values.request();
             const double *data = static_cast<double *>(buf.ptr);
             self.setTorque(data);
           })
      // .def("setCartesianPose") // TODO
      // .def("setCartesianPoseAsMatrix")  // TODO
      .def("setBooleanIOValue", &KUKA::FRI::LBRCommand::setBooleanIOValue)
      .def("setDigitalIOValue", &KUKA::FRI::LBRCommand::setDigitalIOValue)
      .def("setAnalogIOValue", &KUKA::FRI::LBRCommand::setAnalogIOValue);

  py::class_<KUKA::FRI::LBRClient, PyLBRClient>(m, "LBRClient")
      .def(py::init<>())
      .def("onStateChange", &KUKA::FRI::LBRClient::onStateChange)
      .def("monitor", &KUKA::FRI::LBRClient::monitor)
      .def("waitForCommand", &KUKA::FRI::LBRClient::waitForCommand)
      .def("command", &KUKA::FRI::LBRClient::command)
      .def("robotState", &KUKA::FRI::LBRClient::robotState)
      .def("robotCommand", &KUKA::FRI::LBRClient::robotCommand);

  py::class_<PyClientApplication>(m, "ClientApplication")
      .def(py::init<PyLBRClient &>())
      .def("connect", &PyClientApplication::connect)
      .def("disconnect", &PyClientApplication::disconnect)
      .def("step", &PyClientApplication::step);
}
