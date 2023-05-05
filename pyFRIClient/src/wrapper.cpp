// Standard library
#include <cstdio>

// pybind: https://pybind11.readthedocs.io/en/stable/
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

// KUKA FRI-Client-SDK_Cpp (using version hosted at: https://github.com/cmower/FRI-Client-SDK_Cpp)
#include "friLBRClient.h"
#include "friUdpConnection.h"
#include "friClientApplication.h"

// Make LBRClient a Python abstract class
class PyLBRClient : public KUKA::FRI::LBRClient {

  using KUKA::FRI::LBRClient::LBRClient;

public:

  void onStateChange(KUKA::FRI::ESessionState oldState, KUKA::FRI::ESessionState newState) override {
    PYBIND11_OVERRIDE_PURE(void, LBRClient, onStateChange, oldState, newState);
  }

  void monitor() override {
    PYBIND11_OVERRIDE_PURE(void, LBRClient, monitor);
  }

  void waitForCommand() override {
    PYBIND11_OVERRIDE_PURE(void, LBRClient, waitForCommand);
  }

  void command() override {
    PYBIND11_OVERRIDE_PURE(void, LBRClient, command);
  }

};

// Wrapper for ClientApplication (does not make sense for the user to
// instantiate UdpConnection on the Python side).
class PyClientApplication {

public:

  PyClientApplication(PyLBRClient& client) {
    _app = new KUKA::FRI::ClientApplication(_connection, client);
  }

  bool connect(const int port, char* const remoteHost) {
    return _app->connect(port, remoteHost);
  }

  void disconnect() {
    _app->disconnect();
  }

  bool step() {
    return _app->step();
  }

private:
  KUKA::FRI::UdpConnection _connection;
  KUKA::FRI::ClientApplication* _app;

};

// Python bindings
namespace py = pybind11;

PYBIND11_MODULE(pyFRIClient, m) {
    m.doc() = "Python bindings for the KUKA FRI Client SDK. THIS IS NOT A KUKA PRODUCT.";


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
      .value("SAFETY_STOP_LEVEL_0", KUKA::FRI::ESafetyState::SAFETY_STOP_LEVEL_0)
      .value("SAFETY_STOP_LEVEL_1", KUKA::FRI::ESafetyState::SAFETY_STOP_LEVEL_1)
      .value("SAFETY_STOP_LEVEL_2", KUKA::FRI::ESafetyState::SAFETY_STOP_LEVEL_2)
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
      .value("POSITION_CONTROL_MODE", KUKA::FRI::EControlMode::POSITION_CONTROL_MODE)
      .value("CART_IMP_CONTROL_MODE", KUKA::FRI::EControlMode::CART_IMP_CONTROL_MODE)
      .value("JOINT_IMP_CONTROL_MODE", KUKA::FRI::EControlMode::JOINT_IMP_CONTROL_MODE)
      .value("NO_CONTROL", KUKA::FRI::EControlMode::NO_CONTROL)
      .export_values();


    py::enum_<KUKA::FRI::EClientCommandMode>(m, "EClientCommandMode")
      .value("NO_COMMAND_MODE", KUKA::FRI::EClientCommandMode::NO_COMMAND_MODE)
      .value("JOINT_POSITION", KUKA::FRI::EClientCommandMode::JOINT_POSITION)
      .value("WRENCH", KUKA::FRI::EClientCommandMode::WRENCH)
      .value("TORQUE", KUKA::FRI::EClientCommandMode::TORQUE)
      .value("CARTESIAN_POSE", KUKA::FRI::EClientCommandMode::CARTESIAN_POSE)
      .export_values();


    py::enum_<KUKA::FRI::EOverlayType>(m, "EOverlayType")
      .value("NO_OVERLAY", KUKA::FRI::EOverlayType::NO_OVERLAY)
      .value("JOINT", KUKA::FRI::EOverlayType::JOINT)
      .value("CARTESIAN", KUKA::FRI::EOverlayType::CARTESIAN)
      .export_values();


    py::enum_<KUKA::FRI::ERedundancyStrategy>(m, "ERedundancyStrategy")
      .value("E1", KUKA::FRI::ERedundancyStrategy::E1)
      .value("NO_STRATEGY", KUKA::FRI::ERedundancyStrategy::NO_STRATEGY)
      .export_values();


    py::class_<KUKA::FRI::LBRState>(m, "LBRState")
      .def(py::init<>())
      .def_property_readonly_static("NUMBER_OF_JOINTS", [](py::object /* self */) {int num = KUKA::FRI::LBRState::NUMBER_OF_JOINTS; return num;})
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
      .def("getMeasuredJointPosition", &KUKA::FRI::LBRState::getMeasuredJointPosition)
      .def("getMeasuredTorque", &KUKA::FRI::LBRState::getMeasuredTorque)
      .def("getCommandedTorque", &KUKA::FRI::LBRState::getCommandedTorque)
      .def("getExternalTorque", &KUKA::FRI::LBRState::getExternalTorque)
      .def("getIpoJointPosition", [](const KUKA::FRI::LBRState& self) {
				    double position[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
				    memcpy(position, self.getIpoJointPosition(), KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));
				    return py::array_t<double>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS}, position);
				  })
      .def("getTrackingPerformance", &KUKA::FRI::LBRState::getTrackingPerformance)
      .def("getBooleanIOValue", &KUKA::FRI::LBRState::getBooleanIOValue)
      .def("getDigitalIOValue", &KUKA::FRI::LBRState::getDigitalIOValue)
      .def("getAnalogIOValue", &KUKA::FRI::LBRState::getAnalogIOValue)
      .def("getMeasuredCartesianPose", &KUKA::FRI::LBRState::getMeasuredCartesianPose)
      .def("getMeasuredCartesianPoseAsMatrix", [](const KUKA::FRI::LBRState& self) {
						 py::array_t<double> result({3, 4});
						 auto ptr = result.mutable_data();
						 self.getMeasuredCartesianPoseAsMatrix(reinterpret_cast<double(&)[3][4]>(ptr));
						 return result;})
      .def("getIpoCartesianPose", &KUKA::FRI::LBRState::getIpoCartesianPose)
      .def("getIpoCartesianPoseAsMatrix", [](const KUKA::FRI::LBRState& self) {
					    py::array_t<double> result({3, 4});
					    auto ptr = result.mutable_data();
					    self.getIpoCartesianPoseAsMatrix(reinterpret_cast<double(&)[3][4]>(ptr));
					    return result;})
      .def("getMeasuredRedundancyValue", &KUKA::FRI::LBRState::getMeasuredRedundancyValue)
      .def("getIpoRedundancyValue", &KUKA::FRI::LBRState::getIpoRedundancyValue)
      .def("getRedundancyStrategy", &KUKA::FRI::LBRState::getRedundancyStrategy);


    py::class_<KUKA::FRI::LBRCommand>(m, "LBRCommand")
      .def(py::init<>())
      // .def("setJointPosition", &KUKA::FRI::LBRCommand::setJointPosition)
      .def("setJointPosition", [] (KUKA::FRI::LBRCommand& self, py::array_t<double> values) {
	    auto buf = values.request();
            const double* data = static_cast<double*>(buf.ptr);
            self.setJointPosition(data);
        })
      .def("setWrench", &KUKA::FRI::LBRCommand::setWrench)
      .def("setTorque", [] (KUKA::FRI::LBRCommand& self, py::array_t<double> values) {
	    auto buf = values.request();
            const double* data = static_cast<double*>(buf.ptr);

	    // for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i)
	      // std::cout << "data" << i << "=" << data[i] << "\n";

            self.setTorque(data);
        })
      .def("setCartesianPose", &KUKA::FRI::LBRCommand::setCartesianPose)
      // .def("setCartesianPoseAsMatrix", &KUKA::FRI::LBRCommand::setCartesianPoseAsMatrix)  // TODO
      .def("setBooleanIOValue", &KUKA::FRI::LBRCommand::setBooleanIOValue)
      .def("setDigitalIOValue", &KUKA::FRI::LBRCommand::setDigitalIOValue)
      .def("setAnalogIOValue",  &KUKA::FRI::LBRCommand::setAnalogIOValue);


    py::class_<KUKA::FRI::LBRClient, PyLBRClient>(m, "LBRClient")
      .def(py::init<>())
      .def("onStateChange", &KUKA::FRI::LBRClient::onStateChange)
      .def("monitor", &KUKA::FRI::LBRClient::monitor)
      .def("waitForCommand", &KUKA::FRI::LBRClient::waitForCommand)
      .def("command", &KUKA::FRI::LBRClient::command)
      .def("robotState", &KUKA::FRI::LBRClient::robotState)
      .def("robotCommand", &KUKA::FRI::LBRClient::robotCommand);


    py::class_<PyClientApplication>(m, "ClientApplication")
      .def(py::init<PyLBRClient&>())
      .def("connect", &PyClientApplication::connect)
      .def("disconnect", &PyClientApplication::disconnect)
      .def("step", &PyClientApplication::step);


}
