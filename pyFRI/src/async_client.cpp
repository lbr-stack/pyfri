// Standard library
#include <iostream>
#include <string>

// KUKA FRI-Client-SDK_Cpp (using version hosted at:
// https://github.com/cmower/FRI-Client-SDK_Cpp)
#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

// PID implementation: https://github.com/cmower/pid
#include "pid.hpp"

const unsigned int NCART = 6; // number of dimensions in cartesian vector
const unsigned int NDOF =
    KUKA::FRI::LBRState::NUMBER_OF_JOINTS; // number of degrees of freedom

class AsyncLBRClient : public KUKA::FRI::LBRClient {

private:
  bool _ready;

  KUKA::FRI::EClientCommandMode _ccmode;

  double _dt;

  double _set_position[NDOF];
  double _set_wrench[NCART];
  double _set_torque[NDOF];

  double _pv_position[NDOF];
  double _pv_wrench[NCART];
  double _pv_torque[NDOF];

  PIDControl::PID _pid_position[NDOF];
  PIDControl::PID _pid_wrench[NCART];
  PIDControl::PID _pid_torque[NDOF];

  bool _is_position_pid_ready() {
    for (auto &pid : _pid_position) {
      if (!pid.is_ready())
        return false;
    }
    return true;
  }

  bool _is_wrench_pid_ready() {
    for (auto &pid : _pid_wrench) {
      if (!pid.is_ready())
        return false;
    }
    return true;
  }

  bool _is_torque_pid_ready() {
    for (auto &pid : _pid_torque) {
      if (!pid.is_ready())
        return false;
    }
    return true;
  }

  void _command() {

    // Command position
    robotCommand().setJointPosition(_pv_position);

    // Command wrench/torque
    switch (_ccmode) {

    case KUKA::FRI::EClientCommandMode::WRENCH: {
      robotCommand().setWrench(_pv_wrench);
      break;
    }

    case KUKA::FRI::EClientCommandMode::TORQUE: {
      robotCommand().setTorque(_pv_torque);
      break;
    }
    }
  }

  void _update() {

    // When not ready simply return
    if (!_ready)
      return;

    // Update PID
    for (unsigned int i = 0; i < NDOF; ++i)
      _pv_position[i] =
          _pid_position[i].next(_set_position[i], _pv_position[i], _dt);

    switch (_ccmode) {

    case KUKA::FRI::EClientCommandMode::WRENCH: {
      for (unsigned int i = 0; i < NCART; ++i)
        _pv_wrench[i] = _pid_wrench[i].next(_set_wrench[i], _pv_wrench[i], _dt);
      break;
    }

    case KUKA::FRI::EClientCommandMode::TORQUE: {
      for (unsigned int i = 0; i < NDOF; ++i)
        _pv_torque[i] = _pid_torque[i].next(_set_torque[i], _pv_torque[i], _dt);
      break;
    }
    }
  }

public:
  AsyncLBRClient() : _ready(false) {

    // Fill set/pv position and torque with zeros
    for (unsigned i = 0; i < NDOF; ++i) {
      _set_position[i] = 0.0;
      _pv_position[i] = 0.0;
      _set_torque[i] = 0.0;
      _pv_torque[i] = 0.0;
    }

    // Fill set/pv wrench with zeros
    for (unsigned i = 0; i < NCART; ++i) {
      _set_wrench[i] = 0.0;
      _pv_wrench[i] = 0.0;
    }
  }

  ~AsyncLBRClient() {}

  void set_pid_position_gains(double Kp[NDOF], double Ki[NDOF],
                              double Kd[NDOF]) {
    for (unsigned int i = 0; i < NDOF; ++i)
      _pid_position[i].set_gains(Kp[i], Ki[i], Kd[i]);
  }

  void set_pid_wrench_gains(double Kp[NCART], double Ki[NCART],
                            double Kd[NCART]) {
    for (unsigned int i = 0; i < NCART; ++i)
      _pid_wrench[i].set_gains(Kp[i], Ki[i], Kd[i]);
  }

  void set_pid_torque_gains(double Kp[NDOF], double Ki[NDOF], double Kd[NDOF]) {
    for (unsigned int i = 0; i < NDOF; ++i)
      _pid_torque[i].set_gains(Kp[i], Ki[i], Kd[i]);
  }

  void set_position(double position[NDOF]) {
    for (unsigned int i = 0; i < NDOF; ++i) {
      _set_position[i] = position[i];
    }
  }

  void set_wrench(double wrench[NCART]) {
    for (unsigned int i = 0; i < NCART; ++i) {
      _set_wrench[i] = wrench[i];
    }
  }

  void set_torque(double torque[NDOF]) {
    for (unsigned int i = 0; i < NDOF; ++i) {
      _set_torque[i] = torque[i];
    }
  }

  void onStateChange(KUKA::FRI::ESessionState oldState,
                     KUKA::FRI::ESessionState newState) {

    // Report state change
    KUKA::FRI::LBRClient::onStateChange(oldState, newState);

    // Set set/pv position
    if (newState == KUKA::FRI::ESessionState::MONITORING_READY) {

      // Get sample time
      _dt = robotState().getSampleTime();

      // Get client command mode
      _ccmode = robotState().getClientCommandMode();

      // Retrieve current joint position
      memcpy(_pv_position, robotState().getIpoJointPosition(),
             NDOF * sizeof(double));

      // Initialize set position
      for (unsigned int i = 0; i < NDOF; ++i) {
        _set_position[i] = _pv_position[i];

        // Reset position/torque PID
        _pid_position[i].reset();
        _pid_torque[i].reset();
      }

      // Reset wrench PID
      for (unsigned int i = 0; i < NCART; ++i)
        _pid_wrench[i].reset();

      // Ensure gains are set for position PID controller
      if (!_is_position_pid_ready()) {
        std::cout << "Error: you must set gains for PID position controller.\n";
        return;
      }

      // Ensure gains are set for wrench/torque PID controllers
      switch (_ccmode) {

      case KUKA::FRI::EClientCommandMode::WRENCH: {
        if (!_is_wrench_pid_ready()) {
          std::cout << "Error: you must set gains for PID wrench controller.\n";
          return;
        }
      }

      case KUKA::FRI::EClientCommandMode::TORQUE: {
        if (!_is_torque_pid_ready()) {
          std::cout << "Error: you must set gains for PID torque controller.\n";
          return;
        }
      }
      }

      // Above checks passed -> client is ready
      _ready = true;
    }
  }

  void monitor() { KUKA::FRI::LBRClient::monitor(); }

  void waitForCommand() { _command(); }

  void command() {
    _update();
    _command();
  }
};
