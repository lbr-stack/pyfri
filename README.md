# FRI-Client-SDK_Python

KUKA Fast Robot Interface Python SDK.
The code in this repository, provides Python bindings for the FRI Client SDK C++.
The interface has been designed to be as similar as possible to the documentation provided by KUKA.

There is one difference users of the Python bindings should be aware.
When instantiating the client application, in C++ this is performed as follows.

```cpp
// ..setup client..

// create new udp connection
UdpConnection connection;


// pass connection and client to a new FRI client application
ClientApplication app(connection, client);
```

In Python, the equivalent code is as follows.

```python
import pyFRI as fri

# ..setup client..

app = fri.ClientApplication(client)
```

Since UDP is the only supported connection type and the connection object is not actually required by the user after declaring the variable, the `UdpConnection` object is created internally to the `fri.ClientApplication` class object.

See the [examples](examples/).

# Important notice

**THIS IS NOT A KUKA PRODUCT.**

[@cmower](https://github.com/cmower) is not affiliated with KUKA.

# Support

The following versions of FRI are currently supported:
* 1.15
* 2.5

Note, whilst FRI version 2.5 is supported some functionality is not.
Currently, FRI Cartesian Overlay is not supported by FRI-Client-SDK_Python.

If you have a different version, please consider [forking](https://github.com/cmower/FRI-Client-SDK_Cpp/fork) and [submitting a pull request](https://github.com/cmower/FRI-Client-SDK_Cpp/pulls).

# Install

1. Clone repository: `$ git clone --recursive git@github.com:cmower/FRI-Client-SDK_Python.git` (make sure you include `--recursive`)
2. Change directory: `$ cd FRI-Client-SDK_Python`
3. Modify `fri_config.py`: uncomment the line corresponding to your version of FRI.
4. Install: `$ pip install .`

## Upgrading/switching between FRI Versions

If you upgrade your FRI version or want to switch between them, you need to manually remove the `FRI-Client-SDK_Python/build` directory before running `pip install`.

# Usage

## Java application

A flexible Java application is provided [here](https://github.com/cmower/LBR-Java-app).
This must be installed on the KUKA Sunrise controller.

## Data types

You can pass NumPy arrays to the "set" methods (e.g. `setJointPosition`) in order to command the robot.
However, you **must** ensure the format of the array is correct.

Arrays that have a `dtype` of `np.float32` are the only ones that can be accepted.
See how the commands are set in the the examples.

# Examples

First, ensure the corresponding Java applications for each example are installed (these were supplied with KUKA Sunrise).
Then turn on the robot, connect your laptop via ethernet, and follow these steps on your laptop.

1. Change directory: `cd /path/to/FRI-Client-SDK_Python/examples`
2. Run examples
   - `$ python LBRJointSineOverlay.py # based on examples provided by KUKA`
   - `$ python LBRTorqueSineOverlay.py # based on examples provided by KUKA`
   - `$ python LBRWrenchSineOverlay.py # based on examples provided by KUKA`
   - `$ python joint_teleop.py`
   - `$ python task_teleop.py`
   - `$ python move_to.py`
