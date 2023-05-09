# FRI-Client-SDK_Python

KUKA Fast Robot Interface Python SDK.

# Important notice

**THIS IS NOT A KUKA PRODUCT.**

[@cmower](https://github.com/cmower) is not affiliated with KUKA.

# Support

The following versions of FRI are currently supported:
* 1.15
* 2.5

If you have a different version, please consider [forking](https://github.com/cmower/FRI-Client-SDK_Cpp/fork) and [submitting a pull request](https://github.com/cmower/FRI-Client-SDK_Cpp/pulls).

# Install

1. Clone repository: `$ git clone --recursive git@github.com:cmower/FRI-Client-SDK_Python.git` (make sure you include `--recursive`)
2. Change directory; `$ cd FRI-Client-SDK_Python`
3. Modify `project.toml`: uncomment the line corresponding to your version of FRI.
4. Install: `$ pip install .`

## Upgrading/switching between FRI Versions

If you upgrade your FRI version or want to switch between them, you need to manually remove the `FRI-Client-SDK_Python/build` directory before running `pip install`.

# Examples

First, ensure the corresponding Java applications for each example are installed (these were supplied with KUKA Sunrise).
Then turn on the robot, connect your laptop via ethernet, and follow these steps on your laptop.

1. Change directory: `cd /path/to/FRI-Client-SDK_Python/python_examples`
2. Run examples
   - `$ python LBRJointSineOverlay.py`
   - `$ python LBRTorqueSineOverlay.py`
