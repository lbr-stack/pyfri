import os
import time
import yaml
import argparse
import pybullet as p

DEFAULT_SAMPLE_FREQUENCY = 100
ACCELERATION_DUE_TO_GRAVITY = -9.81


class LBRSimulation:
    def __init__(self, lbr_model, config_file_name=None):
        # Load configuration
        if isinstance(config_file_name, str):
            with open(config_file_name, "r") as f:
                self._config = yaml.load(f, Loader=yaml.FullLoader)
        else:
            self._config = {}

        # Connect pybullet
        self._client_id = p.connect(p.GUI_SERVER)
        assert self._client_id != -1, "Failed to connect to the PyBullet server!"

        # Set gravity
        p.setGravity(gravX=0.0, gravY=0.0, gravZ=ACCELERATION_DUE_TO_GRAVITY)

        # Set time step
        hz = self._config.get("sample_frequency", DEFAULT_SAMPLE_FREQUENCY)
        self._time_step = 1.0 / float(hz)
        p.setTimeStep(self._time_step)
        p.setRealTimeSimulation(1, self._client_id)

        # Load model
        path = os.path.join(
            os.path.dirname(os.path.realpath(__file__)), "robots", f"{lbr_model}.urdf"
        )
        self._lbr_id = p.loadURDF(path, useFixedBase=1)

    def spin(self):
        while True:
            time.sleep(self._time_step)

    def disconnect(self):
        p.disconnect(self._client_id)


def main():
    # Handle command line arguments
    parser = argparse.ArgumentParser(description="KUKA LBR Simulator")
    parser.add_argument(
        "--lbr-model",
        dest="lbr_model",
        required=True,
        type=str,
        choices=["med7", "med14"],
        help="The LBR model, either 'med7' or 'med14'.",
    )
    parser.add_argument(
        "--config",
        dest="config_file_name",
        type=str,
        default=None,
        help="Path to YAML configuration file.",
    )
    args = parser.parse_args()

    # Initialize simulation
    sim = LBRSimulation(args.lbr_model, config_file_name=args.config_file_name)

    # Start simulation
    try:
        sim.spin()
    except KeyboardInterrupt:
        pass
    finally:
        sim.disconnect()


if __name__ == "__main__":
    main()
