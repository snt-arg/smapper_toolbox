import typer

from smapper_toolbox.cli import kalibr_app
from smapper_toolbox.logger import logger


def main():
    """
    A toolbox to help automate the calibration process using dockerised kalibr.

    This tool helps calibrate cameras and IMUs using the Kalibr toolbox in a Docker container.
    It supports camera calibration, IMU calibration, and camera-IMU calibration.
    """
    logger.info("Hello from calib-toolbox!")

    app = typer.Typer()
    app.add_typer(kalibr_app, name="kalibr")
    app()


if __name__ == "__main__":
    # typer.run(main)
    main()
