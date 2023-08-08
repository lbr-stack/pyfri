import time
import math
import argparse
import numpy as np
import pyFRI as fri

np.set_printoptions(precision=3, suppress=True, linewidth=1000)


def get_arguments():
    def cvt_joint_mask(value):
        int_value = int(value)
        if 0 <= int_value < 7:
            return int_value
        else:
            raise argparse.ArgumentTypeError(f"{value} is not in the range [0, 7).")

    parser = argparse.ArgumentParser(description="LRBJointSineOverlay example.")
    parser.add_argument(
        "--hostname",
        dest="hostname",
        default=None,
        help="The hostname used to communicate with the KUKA Sunrise Controller.",
    )
    parser.add_argument(
        "--port",
        dest="port",
        type=int,
        default=30200,
        help="The port number used to communicate with the KUKA Sunrise Controller.",
    )
    parser.add_argument(
        "--joint-mask",
        dest="joint_mask",
        type=cvt_joint_mask,
        default=3,
        help="The joint to move.",
    )

    return parser.parse_args()


def main():
    print("Running FRI Version:", fri.FRI_VERSION)

    # Get arguments and initialize client application
    args = get_arguments()
    app = fri.AsyncClientApplication()

    # Set PID position gains
    pos_Kp = np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    pos_Ki = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    pos_Kd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    app.set_pid_position_gains(pos_Kp, pos_Ki, pos_Kd)

    # Connect to controller
    if app.connect(args.port, args.hostname):
        print("Connected to KUKA Sunrise controller.")
    else:
        print("Connection to KUKA Sunrise controller failed.")
        return

    # Wait for FRI loop to start
    app.wait()
    print("FRI Loop started")

    # Setup for Python loop
    hz = 10
    dt = 1.0 / float(hz)
    rate = fri.Rate(hz)
    q = app.robotState().getIpoJointPosition()

    try:
        t = 0.0
        while app.is_ok():
            q[args.joint_mask] += math.radians(20) * math.sin(t * 0.01)
            app.set_position(q.astype(np.float32))
            rate.sleep()
            t += time_step
    except KeyboardInterrupt:
        pass
    finally:
        app.disconnect()


if __name__ == "__main__":
    main()
