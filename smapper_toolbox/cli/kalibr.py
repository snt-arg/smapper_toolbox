from typing import Annotated

import typer

from smapper_toolbox.calibration.kalibr import Calibrators
from smapper_toolbox.config import ConfigManager
from smapper_toolbox.logger import logger
from smapper_toolbox.rosbags import RosbagsConverter

app = typer.Typer()
config_manager = ConfigManager()
config = config_manager.config
calibrators = Calibrators(config)


@app.callback()
def setup():
    rosbags_converter = RosbagsConverter(
        config.workspace.rosbags_dir,
        parallel_jobs=config.performance.parallel_conversions,
    )

    if not rosbags_converter.convert():
        logger.error("Something went wrong when converting rosbags")
        exit(1)

    calibrators.setup()


@app.command()
def cams(
    workspace: Annotated[
        str, typer.Option(help="Location to smapper repository.")
    ] = config.workspace.base_dir,
    calib_dir: Annotated[
        str,
        typer.Option(
            help="Path where calibration files are and where it will be saved."
        ),
    ] = config.workspace.calibration_dir,
    rosbags_dir: Annotated[
        str, typer.Option(help="Path where ros2 bags are located")
    ] = config.workspace.rosbags_dir,
    camera_model: Annotated[
        str, typer.Option(help="Camera model (pinhole-radtan/pinhole-equi)")
    ] = config.calibration.camera.camera_model,
    save_dir: Annotated[
        str, typer.Option(help="Save directory (relative to calibration_dir)")
    ] = config.calibration.camera.save_dir,
    parallel: Annotated[
        int, typer.Option(help="Number of parallel calibrations")
    ] = config.performance.parallel_calibrations,
    target: Annotated[
        str, typer.Option(help="Calibration target (apriltag/checkerboard)")
    ] = config.calibration.camera.target,
    imu: Annotated[
        bool, typer.Option(help="Also run the IMU noise calibration")
    ] = False,
    cam_imu: Annotated[
        bool,
        typer.Option(
            help="Also run the camera imu calibration (if --imu is set, this will only run after.)"
        ),
    ] = False,
):
    """Calibrate cameras."""

    config.workspace.base_dir = workspace

    config.workspace.calibration_dir = calib_dir
    config.workspace.rosbags_dir = rosbags_dir
    config.performance.parallel_calibrations = parallel
    config.calibration.camera.camera_model = camera_model
    config.calibration.camera.save_dir = save_dir
    config.calibration.camera.target = target

    calibrators.calibrate_cameras()

    if imu:
        calibrators.calibrate_imu()

    if cam_imu:
        calibrators.calibrate_cam_imu()


@app.command()
def imu(
    workspace: Annotated[
        str, typer.Option(help="Location to smapper repository.")
    ] = config.workspace.base_dir,
    calib_dir: Annotated[
        str,
        typer.Option(
            help="Path where calibration files are and where it will be saved."
        ),
    ] = config.workspace.calibration_dir,
    rosbags_dir: Annotated[
        str, typer.Option(help="Path where ros2 bags are located")
    ] = config.workspace.rosbags_dir,
    random_walk_multiplier: Annotated[
        int, typer.Option(help="Random Walk multiplier value to be applied to results")
    ] = config.calibration.imu.random_walk_multiplier,
    white_noise_multiplier: Annotated[
        int, typer.Option(help="White Noise multiplier value to be applied to results")
    ] = config.calibration.imu.white_noise_multiplier,
    save_dir: Annotated[
        str, typer.Option(help="Save directory (relative to calibration_dir)")
    ] = config.calibration.camera.save_dir,
    min_duration_hours: Annotated[
        float, typer.Option(help="Calibration target (apriltag/checkerboard)")
    ] = config.calibration.imu.validation.min_duration_hours,
):
    """Obtain IMU noise model(s)."""

    config.workspace.base_dir = workspace

    config.workspace.calibration_dir = calib_dir
    config.workspace.rosbags_dir = rosbags_dir
    config.calibration.imu.random_walk_multiplier = random_walk_multiplier
    config.calibration.imu.white_noise_multiplier = white_noise_multiplier
    config.calibration.imu.save_dir = save_dir
    config.calibration.imu.validation.min_duration_hours = min_duration_hours

    calibrators.calibrate_cameras()
    calibrators.calibrate_imu()


@app.command()
def cam_imu(
    workspace: Annotated[
        str, typer.Option(help="Location to smapper repository.")
    ] = config.workspace.base_dir,
    calib_dir: Annotated[
        str,
        typer.Option(
            help="Path where calibration files are and where it will be saved."
        ),
    ] = config.workspace.calibration_dir,
    rosbags_dir: Annotated[
        str, typer.Option(help="Path where ros2 bags are located")
    ] = config.workspace.rosbags_dir,
    save_dir: Annotated[
        str, typer.Option(help="Save directory (relative to calibration_dir)")
    ] = config.calibration.camera.save_dir,
    reprojection_sigma: Annotated[
        float, typer.Option(help="Estimated reprojection sigma to be used.")
    ] = config.calibration.camera_imu.reprojection_sigma,
    target: Annotated[
        str, typer.Option(help="Calibration target (apriltag/checkerboard)")
    ] = config.calibration.camera_imu.target,
):
    """Calibrate camera to IMU."""

    config.workspace.base_dir = workspace

    config.workspace.calibration_dir = calib_dir
    config.workspace.rosbags_dir = rosbags_dir
    config.calibration.camera_imu.reprojection_sigma = reprojection_sigma
    config.calibration.camera_imu.save_dir = save_dir
    config.calibration.camera_imu.target = target
    calibrators.calibrate_cam_imu()


@app.command()
def all(
    workspace: Annotated[
        str, typer.Option(help="Location to smapper repository.")
    ] = config.workspace.base_dir,
    calib_dir: Annotated[
        str,
        typer.Option(
            help="Path where calibration files are and where it will be saved."
        ),
    ] = config.workspace.calibration_dir,
    rosbags_dir: Annotated[
        str, typer.Option(help="Path where ros2 bags are located")
    ] = config.workspace.rosbags_dir,
    camera_model: Annotated[
        str, typer.Option(help="Camera model (pinhole-radtan/pinhole-equi)")
    ] = config.calibration.camera.camera_model,
    camera_save_dir: Annotated[
        str, typer.Option(help="Camera save directory (relative to calibration_dir)")
    ] = config.calibration.camera.save_dir,
    parallel: Annotated[
        int, typer.Option(help="Number of parallel calibrations")
    ] = config.performance.parallel_calibrations,
    target: Annotated[
        str, typer.Option(help="Calibration target (apriltag/checkerboard)")
    ] = config.calibration.camera.target,
    random_walk_multiplier: Annotated[
        int, typer.Option(help="IMU random walk multiplier")
    ] = config.calibration.imu.random_walk_multiplier,
    white_noise_multiplier: Annotated[
        int, typer.Option(help="IMU white noise multiplier")
    ] = config.calibration.imu.white_noise_multiplier,
    imu_save_dir: Annotated[
        str, typer.Option(help="IMU save directory (relative to calibration_dir)")
    ] = config.calibration.imu.save_dir,
    min_duration_hours: Annotated[
        float, typer.Option(help="Minimum duration in hours for IMU validation")
    ] = config.calibration.imu.validation.min_duration_hours,
    reprojection_sigma: Annotated[
        float, typer.Option(help="Estimated reprojection sigma")
    ] = config.calibration.camera_imu.reprojection_sigma,
    cam_imu_target: Annotated[
        str, typer.Option(help="Camera-IMU calibration target")
    ] = config.calibration.camera_imu.target,
    cam_imu_save_dir: Annotated[
        str,
        typer.Option(help="Camera-IMU save directory (relative to calibration_dir)"),
    ] = config.calibration.camera_imu.save_dir,
):
    """Run all calibrations. It starts with camera, then IMU and finally camera to IMU."""

    # Workspace
    config.workspace.base_dir = workspace
    config.workspace.calibration_dir = calib_dir
    config.workspace.rosbags_dir = rosbags_dir
    config.performance.parallel_calibrations = parallel

    # Camera calibration
    config.calibration.camera.camera_model = camera_model
    config.calibration.camera.save_dir = camera_save_dir
    config.calibration.camera.target = target

    # IMU calibration
    config.calibration.imu.random_walk_multiplier = random_walk_multiplier
    config.calibration.imu.white_noise_multiplier = white_noise_multiplier
    config.calibration.imu.save_dir = imu_save_dir
    config.calibration.imu.validation.min_duration_hours = min_duration_hours

    # Camera-IMU calibration
    config.calibration.camera_imu.reprojection_sigma = reprojection_sigma
    config.calibration.camera_imu.target = cam_imu_target
    config.calibration.camera_imu.save_dir = cam_imu_save_dir

    logger.info("Running all calibrations with overridden configuration")

    calibrators.calibrate_cameras()
    calibrators.calibrate_imu()
    calibrators.calibrate_cam_imu()
