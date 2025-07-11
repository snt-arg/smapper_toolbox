from typing import Annotated, Optional

import typer

from smapper_toolbox.calibration.kalibr import Calibrators
from smapper_toolbox.logger import logger
from smapper_toolbox.rosbags import RosbagsConverter

app = typer.Typer()


# Helper to conditionally override config
def _set_if_not_none(attr, value):
    if value is not None:
        return value
    return attr


def setup(ctx: typer.Context):
    config = ctx.obj["config"]

    print(config.workspace.rosbags_dir)

    rosbags_converter = RosbagsConverter(
        config.workspace.rosbags_dir,
        parallel_jobs=config.performance.parallel_conversions,
    )

    if not rosbags_converter.convert():
        logger.error("Something went wrong when converting rosbags")
        raise typer.Exit(1)

    calibrators = Calibrators(config)
    calibrators.setup()

    ctx.obj["calibrators"] = calibrators


@app.command()
def cams(
    ctx: typer.Context,
    workspace: Annotated[
        Optional[str], typer.Option(help="Location to smapper repository.")
    ] = None,
    calib_dir: Annotated[
        Optional[str],
        typer.Option(
            help="Path where calibration files are and where it will be saved."
        ),
    ] = None,
    rosbags_dir: Annotated[
        Optional[str], typer.Option(help="Path where ros2 bags are located")
    ] = None,
    camera_model: Annotated[
        Optional[str], typer.Option(help="Camera model (pinhole-radtan/pinhole-equi)")
    ] = None,
    save_dir: Annotated[
        Optional[str], typer.Option(help="Save directory (relative to calibration_dir)")
    ] = None,
    parallel: Annotated[
        Optional[int], typer.Option(help="Number of parallel calibrations")
    ] = None,
    target: Annotated[
        Optional[str], typer.Option(help="Calibration target (apriltag/checkerboard)")
    ] = None,
):
    """Calibrate cameras."""
    config = ctx.obj["config"]

    config.workspace.base_dir = _set_if_not_none(config.workspace.base_dir, workspace)
    config.workspace.calibration_dir = _set_if_not_none(
        config.workspace.calibration_dir, calib_dir
    )
    config.workspace.rosbags_dir = _set_if_not_none(
        config.workspace.rosbags_dir, rosbags_dir
    )
    config.performance.parallel_calibrations = _set_if_not_none(
        config.performance.parallel_calibrations, parallel
    )

    config.calibration.camera.camera_model = _set_if_not_none(
        config.calibration.camera.camera_model, camera_model
    )
    config.calibration.camera.save_dir = _set_if_not_none(
        config.calibration.camera.save_dir, save_dir
    )
    config.calibration.camera.target = _set_if_not_none(
        config.calibration.camera.target, target
    )

    setup(ctx)
    calibrators = ctx.obj["calibrators"]
    calibrators.calibrate_cameras()


@app.command()
def imu(
    ctx: typer.Context,
    workspace: Annotated[
        Optional[str], typer.Option(help="Location to smapper repository.")
    ] = None,
    calib_dir: Annotated[
        Optional[str],
        typer.Option(
            help="Path where calibration files are and where it will be saved."
        ),
    ] = None,
    rosbags_dir: Annotated[
        Optional[str], typer.Option(help="Path where ros2 bags are located")
    ] = None,
    random_walk_multiplier: Annotated[
        Optional[int], typer.Option(help="Random Walk multiplier")
    ] = None,
    white_noise_multiplier: Annotated[
        Optional[int], typer.Option(help="White Noise multiplier")
    ] = None,
    save_dir: Annotated[Optional[str], typer.Option(help="IMU save directory")] = None,
    min_duration_hours: Annotated[
        Optional[float], typer.Option(help="Minimum duration in hours")
    ] = None,
):
    """Obtain IMU noise model(s)."""
    config = ctx.obj["config"]

    config.workspace.base_dir = _set_if_not_none(config.workspace.base_dir, workspace)
    config.workspace.calibration_dir = _set_if_not_none(
        config.workspace.calibration_dir, calib_dir
    )
    config.workspace.rosbags_dir = _set_if_not_none(
        config.workspace.rosbags_dir, rosbags_dir
    )

    config.calibration.imu.random_walk_multiplier = _set_if_not_none(
        config.calibration.imu.random_walk_multiplier, random_walk_multiplier
    )
    config.calibration.imu.white_noise_multiplier = _set_if_not_none(
        config.calibration.imu.white_noise_multiplier, white_noise_multiplier
    )
    config.calibration.imu.save_dir = _set_if_not_none(
        config.calibration.imu.save_dir, save_dir
    )
    config.calibration.imu.validation.min_duration_hours = _set_if_not_none(
        config.calibration.imu.validation.min_duration_hours, min_duration_hours
    )

    setup(ctx)
    calibrators = ctx.obj["calibrators"]
    calibrators.calibrate_imu()


@app.command()
def cam_imu(
    ctx: typer.Context,
    workspace: Annotated[
        Optional[str], typer.Option(help="Location to smapper repository.")
    ] = None,
    calib_dir: Annotated[
        Optional[str],
        typer.Option(
            help="Path where calibration files are and where it will be saved."
        ),
    ] = None,
    rosbags_dir: Annotated[
        Optional[str], typer.Option(help="Path where ros2 bags are located")
    ] = None,
    save_dir: Annotated[Optional[str], typer.Option(help="Save directory")] = None,
    reprojection_sigma: Annotated[
        Optional[float], typer.Option(help="Estimated reprojection sigma")
    ] = None,
    target: Annotated[Optional[str], typer.Option(help="Calibration target")] = None,
):
    """Calibrate camera to IMU."""
    config = ctx.obj["config"]

    config.workspace.base_dir = _set_if_not_none(config.workspace.base_dir, workspace)
    config.workspace.calibration_dir = _set_if_not_none(
        config.workspace.calibration_dir, calib_dir
    )
    config.workspace.rosbags_dir = _set_if_not_none(
        config.workspace.rosbags_dir, rosbags_dir
    )

    config.calibration.camera_imu.save_dir = _set_if_not_none(
        config.calibration.camera_imu.save_dir, save_dir
    )
    config.calibration.camera_imu.reprojection_sigma = _set_if_not_none(
        config.calibration.camera_imu.reprojection_sigma, reprojection_sigma
    )
    config.calibration.camera_imu.target = _set_if_not_none(
        config.calibration.camera_imu.target, target
    )

    setup(ctx)

    calibrators = ctx.obj["calibrators"]
    calibrators.calibrate_cam_imu()


@app.command()
def all(
    ctx: typer.Context,
    workspace: Annotated[
        Optional[str], typer.Option(help="Location to smapper repository.")
    ] = None,
    calib_dir: Annotated[
        Optional[str],
        typer.Option(
            help="Path where calibration files are and where it will be saved."
        ),
    ] = None,
    rosbags_dir: Annotated[
        Optional[str], typer.Option(help="Path where ros2 bags are located")
    ] = None,
    camera_model: Annotated[Optional[str], typer.Option(help="Camera model")] = None,
    camera_save_dir: Annotated[
        Optional[str], typer.Option(help="Camera save dir")
    ] = None,
    parallel: Annotated[
        Optional[int], typer.Option(help="Parallel calibrations")
    ] = None,
    target: Annotated[
        Optional[str], typer.Option(help="Camera calibration target")
    ] = None,
    random_walk_multiplier: Annotated[
        Optional[int], typer.Option(help="IMU random walk multiplier")
    ] = None,
    white_noise_multiplier: Annotated[
        Optional[int], typer.Option(help="IMU white noise multiplier")
    ] = None,
    imu_save_dir: Annotated[Optional[str], typer.Option(help="IMU save dir")] = None,
    min_duration_hours: Annotated[
        Optional[float], typer.Option(help="IMU min duration")
    ] = None,
    reprojection_sigma: Annotated[
        Optional[float], typer.Option(help="Reprojection sigma")
    ] = None,
    cam_imu_target: Annotated[
        Optional[str], typer.Option(help="Camera-IMU target")
    ] = None,
    cam_imu_save_dir: Annotated[
        Optional[str], typer.Option(help="Camera-IMU save dir")
    ] = None,
):
    """Run all calibrations (camera, IMU, camera-IMU)."""
    config = ctx.obj["config"]

    config.workspace.base_dir = _set_if_not_none(config.workspace.base_dir, workspace)
    config.workspace.calibration_dir = _set_if_not_none(
        config.workspace.calibration_dir, calib_dir
    )
    config.workspace.rosbags_dir = _set_if_not_none(
        config.workspace.rosbags_dir, rosbags_dir
    )
    config.performance.parallel_calibrations = _set_if_not_none(
        config.performance.parallel_calibrations, parallel
    )

    config.calibration.camera.camera_model = _set_if_not_none(
        config.calibration.camera.camera_model, camera_model
    )
    config.calibration.camera.save_dir = _set_if_not_none(
        config.calibration.camera.save_dir, camera_save_dir
    )
    config.calibration.camera.target = _set_if_not_none(
        config.calibration.camera.target, target
    )

    config.calibration.imu.random_walk_multiplier = _set_if_not_none(
        config.calibration.imu.random_walk_multiplier, random_walk_multiplier
    )
    config.calibration.imu.white_noise_multiplier = _set_if_not_none(
        config.calibration.imu.white_noise_multiplier, white_noise_multiplier
    )
    config.calibration.imu.save_dir = _set_if_not_none(
        config.calibration.imu.save_dir, imu_save_dir
    )
    config.calibration.imu.validation.min_duration_hours = _set_if_not_none(
        config.calibration.imu.validation.min_duration_hours, min_duration_hours
    )

    config.calibration.camera_imu.reprojection_sigma = _set_if_not_none(
        config.calibration.camera_imu.reprojection_sigma, reprojection_sigma
    )
    config.calibration.camera_imu.target = _set_if_not_none(
        config.calibration.camera_imu.target, cam_imu_target
    )
    config.calibration.camera_imu.save_dir = _set_if_not_none(
        config.calibration.camera_imu.save_dir, cam_imu_save_dir
    )

    logger.info("Running all calibrations with overridden configuration")

    setup(ctx)

    calibrators = ctx.obj["calibrators"]
    calibrators.calibrate_cameras()
    calibrators.calibrate_imu()
    calibrators.calibrate_cam_imu()
