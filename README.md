# pyfri

[![Build](https://github.com/lbr-stack/pyfri/actions/workflows/build.yaml/badge.svg)](https://github.com/lbr-stack/pyfri/actions/workflows/build.yaml)
[![License](https://img.shields.io/github/license/lbr-stack/pyfri)](https://github.com/lbr-stack/pyfri/tree/main?tab=Apache-2.0-1-ov-file#readme)
[![JOSS](https://joss.theoj.org/papers/c43c82bed833c02503dd47f2637192ef/status.svg)](https://joss.theoj.org/papers/c43c82bed833c02503dd47f2637192ef)
[![Code Style: Black](https://img.shields.io/badge/code%20style-black-000000.svg)](https://github.com/psf/black)

KUKA Fast Robot Interface Python SDK.
The code in this repository, provides Python bindings for the FRI Client SDK C++ through [pybind11](https://github.com/pybind/pybind11).
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
import pyfri as fri

# ..setup client..

app = fri.ClientApplication(client)
```

Since UDP is the only supported connection type and the connection object is not actually required by the user after declaring the variable, the `UdpConnection` object is created internally to the `fri.ClientApplication` class object.

See the [examples](examples/).

## Quickstart

1. Clone repository (make sure you include `--recursive`):
   ```shell
   git clone --recursive https://github.com/lbr-stack/pyfri.git
   ```
2. Change directory:
   ```shell
   cd pyfri
   ```
3. Install:
   ```shell
   export FRI_CLIENT_VERSION=1.15
   pip3 install .
   ```

> [!NOTE]
> FRI client is fetched from [fri](https://github.com/lbr-stack/fri) and must be available as branch, refer [README](https://github.com/lbr-stack/fri?tab=readme-ov-file#contributing).

4. Setup the hardware, see [Hardware Setup](https://lbr-stack.readthedocs.io/en/latest/lbr_fri_ros2_stack/lbr_fri_ros2_stack/doc/hardware_setup.html).

5. Checkout the [Example Applications](https://lbr-stack.readthedocs.io/en/latest/pyfri/doc/example_applications.html#example-applications).

## Citation
If you enjoyed using this repository for your work, we would really appreciate ❤️ if you could leave a ⭐ and / or cite it, as it helps us to continue offering support.

```
@misc{huber2023lbrstack,
      title={LBR-Stack: ROS 2 and Python Integration of KUKA FRI for Med and IIWA Robots}, 
      author={Martin Huber and Christopher E. Mower and Sebastien Ourselin and Tom Vercauteren and Christos Bergeles},
      year={2023},
      eprint={2311.12709},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
