from typing import Annotated

import typer

from smapper_toolbox.calibration.tf_generator import TFTreeGenerator
from smapper_toolbox.logger import logger

app = typer.Typer()


@app.command()
def generate(
    ctx: typer.Context,
    input: Annotated[str, typer.Option(help="Summary config file genereated.")],
    output: Annotated[str, typer.Option(help="Output file")],
):
    logger.info("Generating TFs")
    config = ctx.obj["config"]

    print(config)

    generator = TFTreeGenerator()
    config_data = generator.load_config_from_yaml(input)
    generator.calculate_transforms_from_config(config_data)
    generator.print_tf_tree_structure()
    generator.print_transform_summary()
    generator.generate_yaml_config(output)
    generator.generate_launch_file()
