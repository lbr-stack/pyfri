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
    parser.add_argument(
        "--freq-hz",
        dest="freq_hz",
        type=float,
        default=0.25,
        help="The frequency of the sine wave.",
    )
    parser.add_argument(
        "--ampl-rad",
        dest="ampl_rad",
        type=float,
        default=0.04,
        help="Applitude of the sine wave.",
    )
    parser.add_argument(
        "--filter-coeff",
        dest="filter_coeff",
        type=float,
        default=0.99,
        help="Exponential smoothing coeficient.",
    )

    return parser.parse_args()


def main():
    print("Running FRI Version:", fri.FRI_VERSION)

    args = get_arguments()

    app = fri.AsyncClientApplication()
    if app.connect(args.port, args.hostname):
        print("Connected to KUKA Sunrise controller.")
    else:
        print("Connection to KUKA Sunrise controller failed.")
        return

    # Wait for FRI loop to start spinning
    # try:
    rate_wait = fri.Rate(1)
    counter = 1
    max_counter = 5
    spinning = app.is_spinning()
    print("spinning=", spinning)
    while not spinning:
        print("Waiting for FRI loop to start, attempt", counter, "of", max_counter)
        counter += 1
        if counter == max_counter + 1:
            print("FRI loop did not start, quitting")
            return
        rate_wait.sleep()
        spinning = app.is_spinning()
    print("spinning=", spinning)
    time.sleep(10.)
    # except KeyboardInterrupt:
    #     pass
    # finally:
    #     app.disconnect()
    #     print("Goodbye for now")
    #     return

    print("FRI Loop started")

    hz = 10
    time_step = 1.0 / float(hz)

    q0 = app.get_proc_position()
    offset = 0.0
    phi = 0.0
    step_width = 2 * math.pi * args.freq_hz * time_step

    try:
        rate = fri.Rate(hz)
        t = 0.
        while app.is_ok():
            new_offset = args.ampl_rad * math.sin(phi)
            offset = (offset * args.filter_coeff) + (
                new_offset * (1.0 - args.filter_coeff)
            )
            phi += step_width
            if phi >= (2 * math.pi):
                phi -= 2 * math.pi
            # q[args.joint_mask] += offset
            q = q0.copy()
            q[args.joint_mask] += math.radians(20)*math.sin(t*0.)
            # print(q)
            app.set_joint_position(q.astype(np.float32))
            qm = app.get_measured_position()
            print(np.linalg.norm(qm - q))
            rate.sleep()
            t += time_step
    except KeyboardInterrupt:
        pass
    finally:
        app.disconnect()


if __name__ == "__main__":
    main()
