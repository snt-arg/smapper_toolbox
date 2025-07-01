"""
Hybrid configuration management using ConfigManager logic with Pydantic models.

This combines the flexible multi-source loading from ConfigManager with
Pydantic's validation and type safety.
"""

import os
from pathlib import Path
from typing import Any, Dict, List, Optional

import toml
import yaml
from pydantic import BaseModel, Field, field_validator, model_validator

from smapper_toolbox.logger import logger


class WorkspaceConfig(BaseModel):
    """Workspace directory configuration."""

    base_dir: str = Field(default="~/Work/smapper")
    calibration_dir: str = Field(default="~/Work/smapper/calibration/kalibr")
    rosbags_dir: str = Field(default="~/rosbags")

    @field_validator("base_dir", "calibration_dir", "rosbags_dir", mode="before")
    @classmethod
    def expand_path(cls, v: str) -> str:
        """Expand environment variables and user paths."""
        return os.path.expandvars(os.path.expanduser(v)) if isinstance(v, str) else v


class DockerConfig(BaseModel):
    """Docker container configuration."""

    image_tag: str = "kalibr"
    dockerfile_path: Optional[str] = None

    @field_validator("dockerfile_path", mode="before")
    @classmethod
    def expand_dockerfile_path(cls, v: Optional[str]) -> Optional[str]:
        """Expand dockerfile path if provided."""
        return os.path.expandvars(os.path.expanduser(v)) if v else v


class TargetConfig(BaseModel):
    """Target configuration for calibration."""

    type: str
    filename: str
    path: Optional[str] = None

    @field_validator("path", mode="before")
    @classmethod
    def expand_path(cls, v: Optional[str]) -> Optional[str]:
        """Expand target path if provided."""
        return os.path.expandvars(os.path.expanduser(v)) if v else v


class PerformanceConfig(BaseModel):
    """Performance and parallelization settings."""

    parallel_conversions: int = 4
    parallel_calibrations: int = 4


class ValidationConfig(BaseModel):
    """Base validation configuration."""

    pass


class CameraValidationConfig(ValidationConfig):
    """Camera calibration validation settings."""

    min_images: int = 50
    max_reprojection_error: float = 2.0


class IMUValidationConfig(ValidationConfig):
    """IMU calibration validation settings."""

    min_duration_hours: float = 3.0
    max_bias_drift: float = 0.01


class CameraIMUValidationConfig(ValidationConfig):
    """Camera-IMU calibration validation settings."""

    min_images: int = 100
    max_reprojection_error: float = 1.5


class CameraCalibratorConfig(BaseModel):
    """Camera calibration configuration."""

    save_dir: str = "static/camera"
    target: str = "default"
    camera_model: str = "pinhole-radtan"
    output_formats: List[str] = Field(default_factory=lambda: ["yaml", "pdf", "txt"])
    validation: CameraValidationConfig = Field(default_factory=CameraValidationConfig)


class IMUCalibratorConfig(BaseModel):
    """IMU calibration configuration."""

    save_dir: str = "static/imu"
    validation: IMUValidationConfig = Field(default_factory=IMUValidationConfig)
    random_walk_multiplier: int = 10
    white_noise_multiplier: int = 5


class CameraIMUCalibratorConfig(BaseModel):
    """Camera-IMU calibration configuration."""

    save_dir: str = "dynamic"
    target: str = "default"
    reprojection_sigma: float = 1.0
    output_formats: List[str] = Field(default_factory=lambda: ["yaml", "pdf", "txt"])
    validation: CameraIMUValidationConfig = Field(
        default_factory=CameraIMUValidationConfig
    )


class CalibrationConfig(BaseModel):
    """Calibration modes configuration."""

    camera: CameraCalibratorConfig = Field(default_factory=CameraCalibratorConfig)
    imu: IMUCalibratorConfig = Field(default_factory=IMUCalibratorConfig)
    camera_imu: CameraIMUCalibratorConfig = Field(
        default_factory=CameraIMUCalibratorConfig
    )


class Config(BaseModel):
    """Main configuration class using Pydantic for validation."""

    workspace: WorkspaceConfig = Field(default_factory=WorkspaceConfig)
    docker: DockerConfig = Field(default_factory=DockerConfig)
    targets: Dict[str, TargetConfig] = Field(
        default_factory=lambda: {
            "default": TargetConfig(type="apriltag", filename="april_6x6_80x80cm.yaml"),
            "checkerboard": TargetConfig(
                type="checkerboard", filename="checkerboard_9x6_0.05m.yaml"
            ),
        }
    )
    performance: PerformanceConfig = Field(default_factory=PerformanceConfig)
    calibration: CalibrationConfig = Field(default_factory=CalibrationConfig)

    @model_validator(mode="after")
    def resolve_references(self) -> "Config":
        """Resolve references between configuration sections."""
        # Set target paths relative to calibration_dir if not specified
        for target_name, target_config in self.targets.items():
            if target_config.path is None:
                target_config.path = str(
                    Path(self.workspace.calibration_dir) / "targets"
                )

        # Set docker dockerfile path if not specified
        if self.docker.dockerfile_path is None:
            self.docker.dockerfile_path = str(
                Path(self.workspace.base_dir) / "docker" / "kalibr" / "Dockerfile"
            )

        return self

    def get_target_config(self, target_name: str) -> Optional[TargetConfig]:
        """Get target configuration by name."""
        return self.targets.get(target_name)

    def get_target_path(self, target_name: str) -> Optional[str]:
        """Get full path to target file."""
        target = self.get_target_config(target_name)
        if target and target.path:
            return str(Path(target.path) / target.filename)
        return None


class ConfigManager:
    """Configuration manager that loads config and validates with Pydantic."""

    def __init__(self):
        self.config_paths = [
            Path.home() / ".config" / "kalibr-toolbox" / "config.yaml",  # User config
            Path.cwd() / ".kalibr-config.yaml",  # Project config
            Path.cwd() / "kalibr.yaml",  # Legacy support
            Path.cwd() / "config" / "config.yaml",  # Default config location
        ]
        self.config = self.load_config()

    def load_config(self) -> Config:
        """Load configuration from multiple sources with precedence and validate with Pydantic."""
        config_dict = self.get_default_config()

        # Load from files (later files override earlier ones)
        for config_path in self.config_paths:
            if config_path.exists():
                try:
                    with open(config_path) as f:
                        if config_path.suffix in [".yaml", ".yml"]:
                            file_config = yaml.safe_load(f)
                        elif config_path.suffix == ".toml":
                            file_config = toml.load(f)
                        else:
                            continue

                    if file_config:
                        config_dict = self.deep_merge(config_dict, file_config)
                        logger.info(f"Loaded config from {config_path}")
                except Exception as e:
                    logger.error(f"Warning: Failed to load {config_path}: {e}")

        # Validate and create Pydantic model
        return Config.model_validate(config_dict)

    def get_default_config(self) -> Dict[str, Any]:
        """Get sensible default configuration as dict."""
        return {
            "workspace": {
                "base_dir": str(Path.home() / "Work" / "smapper"),
                "calibration_dir": str(
                    Path.home() / "Work" / "smapper" / "calibration" / "kalibr"
                ),
                "rosbags_dir": str(Path.home() / "rosbags"),
            },
            "docker": {
                "image_tag": "kalibr",
            },
            "performance": {
                "parallel_conversions": 4,
                "parallel_calibrations": 4,
            },
            "targets": {
                "default": {
                    "type": "apriltag",
                    "filename": "april_6x6_80x80cm.yaml",
                },
            },
            "calibration": {
                "camera": {
                    "save_dir": "static/camera",
                    "target": "default",
                    "camera_model": "pinhole-radtan",
                    "output_formats": ["yaml", "pdf", "txt"],
                    "validation": {
                        "min_images": 50,
                        "max_reprojection_error": 2.0,
                    },
                },
                "imu": {
                    "save_dir": "static/imu",
                    "random_walk_multiplier": 10,
                    "white_noise_multiplier": 5,
                    "validation": {
                        "min_duration_hours": 3.0,
                        "max_bias_drift": 0.01,
                    },
                },
                "camera_imu": {
                    "save_dir": "dynamic",
                    "target": "default",
                    "reprojection_sigma": 1.0,
                    "output_formats": ["yaml", "pdf", "txt"],
                    "validation": {
                        "min_images": 100,
                        "max_reprojection_error": 1.5,
                    },
                },
            },
        }

    def deep_merge(self, base: Dict, override: Dict) -> Dict:
        """Deep merge two dictionaries."""
        result = base.copy()
        for key, value in override.items():
            if (
                key in result
                and isinstance(result[key], dict)
                and isinstance(value, dict)
            ):
                result[key] = self.deep_merge(result[key], value)
            else:
                result[key] = value
        return result

    def set_nested_value(self, config: Dict, path: str, value: Any):
        """Set a nested configuration value using dot notation."""
        keys = path.split(".")
        current = config
        for key in keys[:-1]:
            if key not in current:
                current[key] = {}
            current = current[key]

        # Type conversion for common values
        if isinstance(value, str):
            if value.lower() in ["true", "false"]:
                value = value.lower() == "true"
            elif value.isdigit():
                value = int(value)
            elif value.replace(".", "").isdigit():
                value = float(value)

        current[keys[-1]] = value

    def override_with_cli_args(self, **kwargs) -> Config:
        """Override configuration with CLI arguments and return new Config instance."""
        overrides = {}

        # Map CLI args to config paths
        cli_mappings = {
            "workspace": "workspace.base_dir",
            "calibration_dir": "workspace.calibration_dir",
            "rosbags_dir": "workspace.rosbags_dir",
            "parallel": "performance.parallel_calibrations",
            "target": "calibration.camera.target",
            "camera_model": "calibration.camera.camera_model",
            "reprojection_sigma": "calibration.camera_imu.reprojection_sigma",
        }

        for cli_arg, config_path in cli_mappings.items():
            if cli_arg in kwargs and kwargs[cli_arg] is not None:
                self.set_nested_value(overrides, config_path, kwargs[cli_arg])

        # Apply overrides to current config
        current_dict = self.config.model_dump()
        merged_dict = self.deep_merge(current_dict, overrides)

        # Return new Config instance
        return Config.model_validate(merged_dict)


# Usage example
if __name__ == "__main__":
    # Load configuration
    config_manager = ConfigManager()
    config = config_manager.config

    # Override with CLI args
    cli_config = config_manager.override_with_cli_args(
        workspace="~/custom/workspace", parallel=8, target="checkerboard"
    )

    print(f"Workspace: {cli_config.workspace.base_dir}")
    print(f"Target: {cli_config.get_target_path(cli_config.calibration.camera.target)}")
    print(f"Parallel jobs: {cli_config.performance.parallel_calibrations}")

    # Access nested config with type safety
    print(f"Camera model: {cli_config.calibration.camera.camera_model}")
    print(f"Min images: {cli_config.calibration.camera.validation.min_images}")
