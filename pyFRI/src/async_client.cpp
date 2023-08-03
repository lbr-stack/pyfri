// Standard library
#include <iostream>

// KUKA FRI-Client-SDK_Cpp (using version hosted at:
// https://github.com/cmower/FRI-Client-SDK_Cpp)
#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

// PID implementation: https://github.com/cmower/pid
#include "pid.hpp"

const unsigned int NUM_CART_VEC = 6;

class AsyncLBRClient : public KUKA::FRI::LBRClient {

private:
  std::vector<double> _set_position;
  std::vector<double> _set_wrench;
  std::vector<double> _set_torque;

  std::vector<double> _proc_position;
  std::vector<double> _proc_wrench;
  std::vector<double> _proc_torque;

  double _dt;

  bool _pid_position_ready;
  bool _pid_wrench_ready;
  bool _pid_torque_ready;

  std::unique_ptr<PID::PIDArray> _pid_position;
  std::unique_ptr<PID::PIDArray> _pid_wrench;
  std::unique_ptr<PID::PIDArray> _pid_torque;

public:
  AsyncLBRClient()
      : _pid_position_ready(false), _pid_wrench_ready(false),
        _pid_torque_ready(false) {}

  ~AsyncLBRClient() {}

  void init_pid_position(std::vector<double> Kp, std::vector<double> Ki,
                         std::vector<double> Kd) {
    _pid_position_ready = true;
    _pid_position = std::make_unique<PID::PIDArray>(
        static_cast<unsigned int>(KUKA::FRI::LBRState::NUMBER_OF_JOINTS), Kp,
        Ki, Kd);
  }

  void init_pid_wrench(std::vector<double> Kp, std::vector<double> Ki,
                       std::vector<double> Kd) {
    _pid_wrench_ready = true;
    _pid_wrench = std::make_unique<PID::PIDArray>(NUM_CART_VEC, Kp, Ki, Kd);
  }

  void init_pid_torque(std::vector<double> Kp, std::vector<double> Ki,
                       std::vector<double> Kd) {
    _pid_torque_ready = true;
    _pid_torque = std::make_unique<PID::PIDArray>(
        static_cast<unsigned int>(KUKA::FRI::LBRState::NUMBER_OF_JOINTS), Kp,
        Ki, Kd);
  }

  void onStateChange(KUKA::FRI::ESessionState oldState,
                     KUKA::FRI::ESessionState newState) {
    KUKA::FRI::LBRClient::onStateChange(oldState, newState);
  }

  void monitor() { KUKA::FRI::LBRClient::monitor(); }

  void waitForCommand() {

    _dt = robotState().getSampleTime();

    const double *p = robotState().getIpoJointPosition();
    _proc_position =
        std::vector<double>(p, p + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
    _set_position =
        std::vector<double>(p, p + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);

    robotCommand().setJointPosition(_proc_position.data());

    switch (robotState().getSessionState()) {

    case KUKA::FRI::EClientCommandMode::WRENCH: {
      _proc_wrench = std::vector<double>(NUM_CART_VEC, 0.0);
      _set_wrench = std::vector<double>(NUM_CART_VEC, 0.0);
      robotCommand().setWrench(_proc_wrench.data());
      break;
    }

    case KUKA::FRI::EClientCommandMode::TORQUE: {
      _proc_torque =
          std::vector<double>(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0);
      _set_torque =
          std::vector<double>(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0);
      robotCommand().setTorque(_proc_torque.data());
      break;
    }
    }
  }

  void command() {

    if (_pid_position_ready) {
      _proc_position = _pid_position->next(_set_position, _proc_position, _dt);
    } else {
      std::cout << "Error: PID not setup for position.\n";
    }
    robotCommand().setJointPosition(_proc_position.data());

    switch (robotState().getSessionState()) {

    case KUKA::FRI::EClientCommandMode::WRENCH: {
      if (_pid_wrench_ready) {
        _proc_wrench = _pid_wrench->next(_set_wrench, _proc_wrench, _dt);
      } else {
        std::cout << "Error: PID not setup for wrench.\n";
      }

      robotCommand().setWrench(_proc_wrench.data());
      break;
    }

    case KUKA::FRI::EClientCommandMode::TORQUE: {
      if (_pid_torque_ready) {
        _proc_torque = _pid_torque->next(_set_torque, _proc_torque, _dt);
      } else {
        std::cout << "Error: PID not setup for torque.\n";
      }

      robotCommand().setTorque(_proc_torque.data());
      break;
    }
    }
  }

  std::vector<double> get_proc_position() { return _proc_position; }

  std::vector<double> get_proc_wrench() { return _proc_wrench; }

  std::vector<double> get_proc_torque() { return _proc_torque; }

  std::vector<double> get_set_position() { return _set_position; }

  std::vector<double> get_set_wrench() { return _set_wrench; }

  std::vector<double> get_set_torque() { return _set_torque; }

  void set_position(std::vector<double> position) { _set_position = position; }

  void set_wrench(std::vector<double> wrench) { _set_wrench = wrench; }

  void set_torque(std::vector<double> torque) { _set_torque = torque; }
};
