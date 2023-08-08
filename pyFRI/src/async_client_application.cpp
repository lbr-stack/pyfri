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
  bool _connected;
  bool _fri_spinning;

  std::thread _fri_loop_thread;

  KUKA::FRI::UdpConnection _connection;
  AsyncLBRClient &_client;
  KUKA::FRI::ClientApplication &_app;

  void _spin_fri() {

    while (_connected) {

      // Step the application
      _connected = _app.step();

      // Update _fri_spinning variable
      if (_connected)
        _fri_spinning = true;
      else
        _fri_spinning = false;

      // Get session state
      KUKA::FRI::ESessionState state = _client.robotState().getSessionState();

      // Check session state
      if (state == KUKA::FRI::ESessionState::IDLE) {
        _connected = false;
        _fri_spinning = false;
      }
    }
  }

public:
  AsyncClientApplication()
      : _client(*new AsyncLBRClient),
        _app(*new KUKA::FRI::ClientApplication(_connection, _client)),
        _connected(false), _fri_spinning(false) {}

  bool connect(const int port, char *const remoteHost = NULL) {

    // Connect to controller
    _connected = _app.connect(port, remoteHost);

    // When successfull, start the fri loop
    if (_connected)
      _fri_loop_thread = std::thread(&AsyncClientApplication::_spin_fri, this);

    return _connected;
  }

  bool is_ok() { return _connected && _fri_spinning; }

  void disconnect() {
    _app.disconnect();
    _connected = false;
    _fri_spinning = false;
  }

  AsyncLBRClient client() { return _client; }

  void wait() {
    while (!_fri_spinning) {
      std::this_thread::sleep_for(
          std::chrono::milliseconds(10)); // Sleep for 10 milliseconds
    }
  }
};
