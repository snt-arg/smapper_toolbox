from typing import Annotated

import typer

from smapper_toolbox.calibration.cam_info_generator import kalibr_to_ros2_yaml

app = typer.Typer()


@app.command()
def generate(
    input: Annotated[
        str,
        typer.Option(
            help="Path to Kalibr camera YAML file (e.g. camchain.yaml or cam0.yaml)"
        ),
    ],
    output: Annotated[
        str, typer.Option(help="Path to output ROS 2-compatible YAML file")
    ],
    cam_name: Annotated[
        str,
        typer.Option(help="Camera name for ROS"),
    ] = "camera",
):
    kalibr_to_ros2_yaml(input, output, cam_name)
