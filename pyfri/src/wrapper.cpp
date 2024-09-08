// Standard library
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

// pybind: https://pybind11.readthedocs.io/en/stable/
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

// KUKA FRI-Client-SDK_Cpp (using version hosted at:
// https://github.com/cmower/FRI-Client-SDK_Cpp)
#include "friClientApplication.h"
#include "friClientVersion.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

// Function for returning the current time
long long getCurrentTimeInNanoseconds() {
  using namespace std::chrono;

  // Use the high-resolution clock for the most accurate time measurement
  high_resolution_clock::time_point currentTime = high_resolution_clock::now();

  // Get the time duration since the epoch in nanoseconds
  auto duration = duration_cast<nanoseconds>(currentTime.time_since_epoch());

  // Convert the duration to a long long value representing nanoseconds
  return duration.count();
}

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
  PyClientApplication(PyLBRClient &client)
      : _record_index(0), _collect_data(false), _client(client) {
    _app = std::make_unique<KUKA::FRI::ClientApplication>(_connection, client);
  }

  void collect_data(std::string file_name) {

    _collect_data = true;
    _time = 0.0;

    // Ensure file name ends with .csv extension
    std::string extension = ".csv";

    if (file_name.length() < extension.length() ||
        file_name.compare(file_name.length() - extension.length(),
                          extension.length(), extension) != 0) {
      // File name doesn't end with ".csv", so append the extension.
      file_name += extension;
    }

    _file_name = file_name;

    // Open data file
    _data_file.open(_file_name);

    // Write header
    _data_file << "index"
               << ",";
    _data_file << "time"
               << ",";
    _data_file << "record_time_nsec"
               << ",";
    _data_file << "tsec"
               << ",";
    _data_file << "tnsec"
               << ",";

    for (unsigned int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i)
      _data_file << "mp" << i + 1 << ",";

    for (unsigned int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i)
      _data_file << "ip" << i + 1 << ",";

    for (unsigned int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i)
      _data_file << "mt" << i + 1 << ",";

    for (unsigned int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i)
      _data_file << "et" << i + 1 << ",";

    _data_file << "dt"
               << "\n";
  }

  bool connect(const int port, char *const remoteHost = NULL) {
    return _app->connect(port, remoteHost);
  }

  void disconnect() {
    _app->disconnect();
    if (_collect_data) {
      _data_file.close();
      std::cout << "Saved:" << _file_name << "\n";
    }
  }

  bool step() {

    // Step FRI
    if (!_app->step())
      return false;

    // Optionally record data
    KUKA::FRI::ESessionState currentState =
        _client.robotState().getSessionState();
    if (_collect_data &&
        (currentState == KUKA::FRI::ESessionState::COMMANDING_WAIT ||
         currentState == KUKA::FRI::ESessionState::COMMANDING_ACTIVE)) {
      _record_data();
    }

    return true;
  }

private:
  unsigned long long _record_index;
  long double _time;
  bool _collect_data;
  std::string _file_name;
  std::ofstream _data_file;
  PyLBRClient &_client;
  KUKA::FRI::UdpConnection _connection;
  std::unique_ptr<KUKA::FRI::ClientApplication> _app;

  void _record_data() {

    // Get state data
    double mposition[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
    memcpy(mposition, _client.robotState().getMeasuredJointPosition(),
           KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));

    double ipoposition[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
    memcpy(ipoposition, _client.robotState().getIpoJointPosition(),
           KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));

    double mtorque[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
    memcpy(mtorque, _client.robotState().getMeasuredTorque(),
           KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));

    double ext_torque[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
    memcpy(ext_torque, _client.robotState().getExternalTorque(),
           KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));

    // Record data
    _data_file << _record_index << ",";
    _data_file << _time << ",";
    _data_file << getCurrentTimeInNanoseconds() << ",";
    _data_file << _client.robotState().getTimestampSec() << ",";
    _data_file << _client.robotState().getTimestampNanoSec() << ",";

    for (unsigned int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i)
      _data_file << mposition[i] << ",";

    for (unsigned int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i)
      _data_file << ipoposition[i] << ",";

    for (unsigned int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i)
      _data_file << mtorque[i] << ",";

    for (unsigned int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; ++i)
      _data_file << ext_torque[i] << ",";

    double sample_time = _client.robotState().getSampleTime();
    _data_file << sample_time << "\n";

    // Increment _record_index and _time
    _record_index++;
    _time += sample_time;
  }
};

// Python bindings
namespace py = pybind11;

PYBIND11_MODULE(_pyfri, m) {
  m.doc() = "Python bindings for the KUKA FRI Client SDK. THIS IS NOT A KUKA "
            "PRODUCT.";

  m.attr("FRI_CLIENT_VERSION_MAJOR") = FRI_CLIENT_VERSION_MAJOR;
  m.attr("FRI_CLIENT_VERSION_MINOR") = FRI_CLIENT_VERSION_MINOR;
  m.attr("FRI_CLIENT_VERSION") = std::to_string(FRI_CLIENT_VERSION_MAJOR) +
                                 "." + std::to_string(FRI_CLIENT_VERSION_MINOR);

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
#if FRI_CLIENT_VERSION_MAJOR == 1
      .value("POSITION", KUKA::FRI::EClientCommandMode::POSITION)
#elif FRI_CLIENT_VERSION_MAJOR == 2
      .value("JOINT_POSITION", KUKA::FRI::EClientCommandMode::JOINT_POSITION)
      .value("CARTESIAN_POSE", KUKA::FRI::EClientCommandMode::CARTESIAN_POSE)
#endif
      .export_values();

  py::enum_<KUKA::FRI::EOverlayType>(m, "EOverlayType")
      .value("NO_OVERLAY", KUKA::FRI::EOverlayType::NO_OVERLAY)
      .value("JOINT", KUKA::FRI::EOverlayType::JOINT)
      .value("CARTESIAN", KUKA::FRI::EOverlayType::CARTESIAN)
      .export_values();

#if FRI_CLIENT_VERSION_MAJOR == 2
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
             // Declare variables
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             float dataf[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];

             // Retrieve state
             memcpy(data, self.getMeasuredJointPosition(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));

             // Parse: double -> float
             for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++)
               dataf[i] = (float)data[i];

             return py::array_t<float>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                       dataf);
           })
      .def("getMeasuredTorque",
           [](const KUKA::FRI::LBRState &self) {
             // Declare variables
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             float dataf[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];

             // Retrieve state
             memcpy(data, self.getMeasuredTorque(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));

             // Parse: double -> float
             for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++)
               dataf[i] = (float)data[i];

             return py::array_t<float>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                       dataf);
           })
      .def("getCommandedTorque",
           [](const KUKA::FRI::LBRState &self) {
             // Declare variables
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             float dataf[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];

             // Retrieve state
             memcpy(data, self.getCommandedTorque(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));

             // Parse: double -> float
             for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++)
               dataf[i] = (float)data[i];

             return py::array_t<float>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                       dataf);
           })
      .def("getExternalTorque",
           [](const KUKA::FRI::LBRState &self) {
             // Declare variables
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             float dataf[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];

             // Retrieve state
             memcpy(data, self.getExternalTorque(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));

             // Parse: double -> float
             for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++)
               dataf[i] = (float)data[i];

             return py::array_t<float>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                       dataf);
           })
      .def("getIpoJointPosition",
           [](const KUKA::FRI::LBRState &self) {
             // Declare variables
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             float dataf[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];

             // Retrieve state
             memcpy(data, self.getIpoJointPosition(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));

             // Parse: double -> float
             for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++)
               dataf[i] = (float)data[i];

             return py::array_t<float>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                       dataf);
           })
      .def("getTrackingPerformance",
           &KUKA::FRI::LBRState::getTrackingPerformance)
      .def("getBooleanIOValue", &KUKA::FRI::LBRState::getBooleanIOValue)
      .def("getDigitalIOValue", &KUKA::FRI::LBRState::getDigitalIOValue)
      .def("getAnalogIOValue", &KUKA::FRI::LBRState::getAnalogIOValue)
#if FRI_CLIENT_VERSION_MAJOR == 1
      .def("getCommandedJointPosition",
           [](const KUKA::FRI::LBRState &self) {
             // Declare variables
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             float dataf[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];

             // Retrieve state
             memcpy(data, self.getCommandedJointPosition(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));

             // Parse: double -> float
             for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++)
               dataf[i] = (float)data[i];

             return py::array_t<float>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                       dataf);
           })
#elif FRI_CLIENT_VERSION_MAJOR == 2
    .def("getMeasuredCartesianPose",
	 [](const KUKA::FRI::LBRState &self) {

             // Declare variables
             double data[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];
             float dataf[KUKA::FRI::LBRState::NUMBER_OF_JOINTS];

             // Retrieve state
             memcpy(data, self.getMeasuredCartesianPose(),
                    KUKA::FRI::LBRState::NUMBER_OF_JOINTS * sizeof(double));

             // Parse: double -> float
             for (int i = 0; i < KUKA::FRI::LBRState::NUMBER_OF_JOINTS; i++)
               dataf[i] = (float)data[i];

             return py::array_t<float>({KUKA::FRI::LBRState::NUMBER_OF_JOINTS},
                                       dataf);
           })
      .def("getMeasuredCartesianPoseAsMatrix",
           [](const KUKA::FRI::LBRState &self) {
	     // TODO
	     throw std::runtime_error("getMeasuredCartesianPoseAsMatrix is not yet exposed (use .getMeasuredCartesianPose instead).");
           })
      .def("getIpoCartesianPose",
           [](const KUKA::FRI::LBRState &self) {
	     // TODO
	     // Currently, FRI Cartesian Overlay is not supported by FRI-Client-SDK_Python.
	     // IPO Cartesian Pose not available when FRI Cartesian Overlay is not active.
	     throw std::runtime_error("getIpoCartesianPose is not yet exposed.");
           })
      .def("getIpoCartesianPoseAsMatrix",
           [](const KUKA::FRI::LBRState &self) {
	     // TODO
	     // Currently, FRI Cartesian Overlay is not supported by FRI-Client-SDK_Python.
	     // IPO Cartesian Pose not available when FRI Cartesian Overlay is not active.
	     throw std::runtime_error("getIpoCartesianPoseAsMatrix is not yet exposed.");
           })
      .def("getMeasuredRedundancyValue",
           &KUKA::FRI::LBRState::getMeasuredRedundancyValue)
      .def("getIpoRedundancyValue",
	   [](KUKA::FRI::LBRState &self) {
	     // TODO
	     // Currently, FRI Cartesian Overlay is not supported by FRI-Client-SDK_Python.
	     // IPO redundancy value not available when FRI Cartesian Overlay is not active.
	     throw std::runtime_error("getIpoRedundancyValue is not yet exposed.");
	   })
      .def("getRedundancyStrategy",
           &KUKA::FRI::LBRState::getRedundancyStrategy)
#endif
      ; // NOTE: this completes LBRState

  py::class_<KUKA::FRI::LBRCommand>(m, "LBRCommand")
      .def(py::init<>())
      .def("setJointPosition",
           [](KUKA::FRI::LBRCommand &self, py::array_t<double> values) {
             if (values.ndim() != 1 ||
                 values.shape(0) != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
               throw std::runtime_error(
                   "Input array must have shape (" +
                   std::to_string(KUKA::FRI::LBRState::NUMBER_OF_JOINTS) +
                   ",)!");
             }
             auto buf = values.request();
             const double *data = static_cast<double *>(buf.ptr);
             self.setJointPosition(data);
           })
      .def("setWrench",
           [](KUKA::FRI::LBRCommand &self, py::array_t<double> values) {
             if (values.ndim() != 1 ||
                 values.shape(0) != 6 // [F_x, F_y, F_z, tau_A, tau_B, tau_C]
             ) {
               throw std::runtime_error(
                   "Input array must have shape (" +
                   std::to_string(KUKA::FRI::LBRState::NUMBER_OF_JOINTS) +
                   ",)!");
             }
             auto buf = values.request();
             const double *data = static_cast<double *>(buf.ptr);
             self.setWrench(data);
           })
      .def("setTorque",
           [](KUKA::FRI::LBRCommand &self, py::array_t<double> values) {
             if (values.ndim() != 1 ||
                 values.shape(0) != KUKA::FRI::LBRState::NUMBER_OF_JOINTS) {
               throw std::runtime_error(
                   "Input array must have shape (" +
                   std::to_string(KUKA::FRI::LBRState::NUMBER_OF_JOINTS) +
                   ",)!");
             }
             auto buf = values.request();
             const double *data = static_cast<double *>(buf.ptr);
             self.setTorque(data);
           })
      .def("setCartesianPose",
           [](KUKA::FRI::LBRCommand &self, py::array_t<double> values) {
             // TODO
             // Currently, FRI Cartesian Overlay is not supported by
             // FRI-Client-SDK_Python.
             throw std::runtime_error("setCartesianPose is not yet exposed.");
           })
      .def("setCartesianPoseAsMatrix",
           [](KUKA::FRI::LBRCommand &self, py::array_t<double> values) {
             // TODO
             // Currently, FRI Cartesian Overlay is not supported by
             // FRI-Client-SDK_Python.
             throw std::runtime_error(
                 "setCartesianPoseAsMatrix is not yet exposed.");
           })
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
      .def("collect_data", &PyClientApplication::collect_data)
      .def("disconnect", &PyClientApplication::disconnect)
      .def("step", &PyClientApplication::step);
}
