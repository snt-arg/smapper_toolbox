import typer

from smapper_toolbox.config import Config
from smapper_toolbox.logger import logger
from smapper_toolbox.calibration import Calibrators
from smapper_toolbox.rosbags import RosbagsConverter

app = typer.Typer()
config = Config()
rosbags_converter = RosbagsConverter(config.rosbags_dir)
calibrators = Calibrators(config)


def setup():
    if not rosbags_converter.convert():
        logger.error("Something went wrong when converting rosbags")
        exit(1)

    calibrators.setup()


@app.command()
def cameras():
    logger.info("Camera calibration")
    setup()
    calibrators.calibrate_cameras()


@app.command()
def imu():
    logger.info("IMU calibration")
    setup()
    calibrators.calibrate_imu()


@app.command()
def imu_cam():
    logger.info("IMU Camera calibration")
    setup()
    calibrators.calibrate_imu_camera()


@app.command()
def all():
    logger.info("All calibration")
    setup()
    calibrators.calibrate_cameras()
    calibrators.calibrate_imu()
    calibrators.calibrate_imu_camera()
