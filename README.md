# SMapper Toolbox

The **SMapper Toolbox** is a command-line interface (CLI) built with Python to simplify the calibration process for the SMapper device. It automates complex procedures, reducing them to a handful of simple commands.

## Features

- **Automated Calibration:** Executes the [Kalibr](http://wiki.ros.org/kalibr) toolbox within Docker containers to handle camera intrinsics, IMU noise models, and camera-IMU extrinsics.
- **ROS Bag Conversion:** Automatically converts ROS2 bags to the ROS1 format required by Kalibr.
- **TF Tree Generation:** Creates a complete ROS transformation (TF) tree from the calibration results.
- **Configuration Conversion:** Transforms Kalibr's output into standard ROS `camera_info` configuration files.

## How It Works

The toolbox streamlines calibration by spawning Docker containers with the Kalibr environment. Since Kalibr requires ROS1, the toolbox first initiates a conversion process to transform your ROS2 rosbags into the compatible ROS1 format before running the calibration tasks.

## Prerequisites

Before using the toolbox, ensure you have met the following requirements.

### 1. UV Package Manager

The `smapper_toolbox` uses **UV**, an extremely fast Python package manager from Astral.

- **Installation (Linux/macOS):**
    ```bash
    curl -LsSf [https://astral.sh/uv/install.sh](https://astral.sh/uv/install.sh) | sh
    ```
- You can find more information about UV on its [official website](https://astral.sh/uv) and [GitHub repository](https://github.com/astral-sh/uv).

### 2. Rosbag Folder Structure

For the calibration commands, your rosbags **must** be organized in the following directory structure. The toolbox looks for a root folder containing a `ros2` subdirectory where the bags are located.

```
.
└── ros2
    ├── bag1
    ├── bag2
```

## Getting Started

### Configuration

The primary configuration file is located at `config/config.yaml`. Here, you can define settings for the calibration pipeline, such as workspace parameters.

Alternatively, many settings can be overridden directly with command-line options. For example:
`toolbox kalibr [subcommand] --option-name value`

Some commands, like `cam_info`, require input and output files to be specified via options:
`toolbox cam_info generate --input <input_file> --output <output_file>`

### Usage

1.  Navigate to the root directory of the `smapper_toolbox` project.
2.  Run the help command. On the first run, UV will automatically download and install all required dependencies into a virtual environment.

    ```bash
    uv run ./toolbox --help
    ```

3.  This will display the main help menu, showing all available commands and options:

    ```text
    ❯ uv run ./toolbox --help
    INFO:     Hello from calib-toolbox!

     Usage: toolbox [OPTIONS] COMMAND [ARGS]...

     A toolbox to help automate the calibration process using dockerised kalibr.

     This tool helps calibrate cameras and IMUs using the Kalibr toolbox in a Docker container. It supports
     camera calibration, IMU calibration, and camera-IMU calibration.

    ╭─ Options ───────────────────────────────────────────────────────────────────────────────────────────────╮
    │ --install-completion           Install completion for the current shell.                                │
    │ --show-completion              Show completion for the current shell, to copy it or customize the       │
    │                                installation.                                                            │
    │ --help                         Show this message and exit.                                              │
    ╰─────────────────────────────────────────────────────────────────────────────────────────────────────────╯
    ╭─ Commands ──────────────────────────────────────────────────────────────────────────────────────────────╮
    │ kalibr                                                                                                  │
    │ transforms                                                                                              │
    │ cam_info                                                                                                │
    ╰─────────────────────────────────────────────────────────────────────────────────────────────────────────╯
    ```
