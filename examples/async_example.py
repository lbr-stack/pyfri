import time
import math
import argparse
import pyFRI as fri


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
    parser.add_argument(
        "--save-data",
        dest="save_data",
        action="store_true",
        default=False,
        help="Set this flag to save the data.",
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
    try:
        rate_wait = fri.Rate(1)
        counter = 1
        max_counter = 5
        while not app.is_spinning():
            print("Waiting for FRI loop to start, attempt", counter, "of", max_counter)
            counter += 1
            if counter == max_counter + 1:
                print("FRI loop did not start, quitting")
                return
            rate_wait.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        app.disconnect()
        print("Goodbye")
        return

    hz = 50
    time_step = 1.0 / float(hz)

    q = app.get_proc_position()
    offset = 0.0
    phi = 0.0
    step_width = 2 * math.pi * args.freq_hz * time_step

    try:
        rate = fri.Rate(hz)
        while app.is_ok():
            new_offset = args.ampl_rad * math.sin(phi)
            offset = (offset * args.filter_coeff) + (
                new_offset * (1.0 - args.filter_coeff)
            )
            phi += step_width
            if phi >= (2 * math.pi):
                phi -= 2 * math.pi
            q[args.joint_mask] += offset
            app.set_position(q.astype(np.float32))
            rate.sleep()
    except KeyboardInterrupt:
        pass
    finally:
        app.disconnect()


if __name__ == "__main__":
    main()
