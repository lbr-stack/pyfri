
# Usage

## Data types

You can pass NumPy arrays to the "set" methods (e.g. `setJointPosition`) in order to command the robot.
However, you **must** ensure the format of the array is correct.

Arrays that have a `dtype` of `np.float32` are the only ones that can be accepted.
See how the commands are set in the the examples.

## Collecting data from the robot

We provide additional functionality to the LBR client application class that enables data collection to a text file.
In Python, simply add the following

```python
app = fri.ClientApplication(client)
app.collect_data(file_name)
```

The string `file_name` should contain the file name for the data file.
We use the comma-separated values format for the data file, and so the file name should end with the extension `.csv` - *note*, if this isn't the case then `.csv` is automatically appended to the given file name.

The columns in the recorded csv file are as follows:
* `index`: the index of the recorded data, starts at 0, and is incremented by 1 at each call to the `step` method.
* `time`: the time of the command, starts at 0.0, then is incremented by the sample time at each call to the `step` method.
* `record_time_nsec`: the epoch time collected when the data in the current time step is recorded.
* `tsec`: controller time as specified by the FRI in seconds. See FRI documentation for `getTimestampSec`.
* `tnsec`: controller time as specified by the FRI in nanoseconds. See FRI documentation for `getTimestampNanoSec`.
* `mp1, ..., mp7`: The measured joint position for the robot.
* `ip1, ..., ip7`: The ipo joint position for the robot.
* `mt1, ..., mt7`: The measured torque for the robot.
* `et1, ..., et7`: The external torque for the robot.
* `dt`: The sample time specified on the KUKA controller.

See the [examples/LBRJointSineOverlay.py](examples/LBRJointSineOverlay.py) example that demonstrates how to easily collect data from the robot.

# Examples

First, ensure the corresponding Java applications for each example are installed (these were supplied with KUKA Sunrise).
Then turn on the robot, connect your laptop via ethernet, and follow these steps on your laptop.

1. Change directory: `cd /path/to/FRI-Client-SDK_Python/examples`
2. Run examples
   - `$ python3 LBRJointSineOverlay.py # based on examples provided by KUKA`
   - `$ python3 LBRTorqueSineOverlay.py # based on examples provided by KUKA`
   - `$ python3 LBRWrenchSineOverlay.py # based on examples provided by KUKA`
   - `$ python3 joint_teleop.py`
   - `$ python3 task_teleop.py`

