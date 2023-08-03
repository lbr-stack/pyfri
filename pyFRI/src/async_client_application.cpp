// Standard library
#include <thread>

// KUKA FRI-Client-SDK_Cpp (using version hosted at:
// https://github.com/cmower/FRI-Client-SDK_Cpp)
#include "friClientApplication.h"
#include "friLBRClient.h"
#include "friUdpConnection.h"

// Local
#include "async_client.cpp"

// Asynchronous client application implementation
class AsyncClientApplication {

private:
  bool _success;
  bool _fri_loop_continue;
  bool _spinning;
  std::thread _fri_loop_thread;
  AsyncLBRClient &_client;
  KUKA::FRI::UdpConnection _connection;
  KUKA::FRI::ClientApplication &_app;

  void _spin_fri() {
    while (_success && _fri_loop_continue) {
      _success = _app.step();

      if (_success)
        _spinning = true;

      if (_client.robotState().getSessionState() ==
          KUKA::FRI::ESessionState::IDLE) {
        _success = false;
        _spinning = false;
        break;
      }
    }
  }

public:
  AsyncClientApplication()
      : _client(*new AsyncLBRClient),
        _app(*new KUKA::FRI::ClientApplication(_connection, _client)),
        _fri_loop_continue(false), _spinning(false) {

    std::vector<double> Kp_position = {1., 1., 1., 1., 1., 1., 1.};
    std::vector<double> Ki_position =
        std::vector<double>(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0);
    std::vector<double> Kd_position =
        std::vector<double>(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0);
    _client.init_pid_position(Kp_position, Ki_position, Kd_position);

    std::vector<double> Kp_wrench = {1., 1., 1., 1., 1., 1.};
    std::vector<double> Ki_wrench = std::vector<double>(NUM_CART_VEC, 0.0);
    std::vector<double> Kd_wrench = std::vector<double>(NUM_CART_VEC, 0.0);
    _client.init_pid_wrench(Kp_wrench, Ki_wrench, Kd_wrench);

    std::vector<double> Kp_torque = {1., 1., 1., 1., 1., 1., 1.};
    std::vector<double> Ki_torque =
        std::vector<double>(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0);
    std::vector<double> Kd_torque =
        std::vector<double>(KUKA::FRI::LBRState::NUMBER_OF_JOINTS, 0.0);
    _client.init_pid_torque(Kp_torque, Ki_torque, Kd_torque);
  }

  bool connect(const int port, char *const remoteHost = NULL) {

    // Connect to controller
    _success = _app.connect(port, remoteHost);

    // When successfull, start the fri loop
    if (_success)
      _fri_loop_continue = true;
    _fri_loop_thread = std::thread(&AsyncClientApplication::_spin_fri, this);

    return _success;
  }

  bool is_spinning() { return _spinning; }

  bool is_ok() { return _success; }

  std::vector<double> get_ipo_position() const {
    const double *p = _client.robotState().getIpoJointPosition();
    return std::vector<double>(p, p + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  }

  std::vector<double> get_measured_position() const {
    const double *p = _client.robotState().getMeasuredJointPosition();
    return std::vector<double>(p, p + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  }

  std::vector<double> get_measured_torque() const {
    const double *t = _client.robotState().getMeasuredTorque();
    return std::vector<double>(t, t + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  }

  std::vector<double> get_external_torque() const {
    const double *t = _client.robotState().getExternalTorque();
    return std::vector<double>(t, t + KUKA::FRI::LBRState::NUMBER_OF_JOINTS);
  }

  std::vector<double> get_proc_position() const {
    return _client.get_proc_position();
  }

  std::vector<double> get_proc_wrench() const {
    return _client.get_proc_wrench();
  }

  std::vector<double> get_proc_torque() const {
    return _client.get_proc_torque();
  }

  std::vector<double> get_set_position() const {
    return _client.get_set_position();
  }

  std::vector<double> get_set_wrench() const {
    return _client.get_set_wrench();
  }

  std::vector<double> get_set_torque() const {
    return _client.get_set_torque();
  }

  void set_position(std::vector<double> position) {
    _client.set_position(position);
  }

  void set_wrench(std::vector<double> wrench) { _client.set_wrench(wrench); }

  void set_torque(std::vector<double> torque) { _client.set_torque(torque); }

  void disconnect() {
    _app.disconnect();
    _fri_loop_continue = false;
  }
};
