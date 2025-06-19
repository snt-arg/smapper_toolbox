"""
Job and process execution utilities for the SMapper toolbox.

This module provides classes and functions to manage, execute, and monitor jobs in parallel, including support for Docker-based jobs.
"""

import signal
import subprocess
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

from docker.errors import NotFound
from rich.progress import (
    BarColumn,
    Progress,
    SpinnerColumn,
    TaskProgressColumn,
    TextColumn,
    TimeElapsedColumn,
)

from smapper_toolbox.logger import logger
from smapper_toolbox.utils.docker import DockerRunner


class JobStatus(Enum):
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class JobResult:
    """Stores the result of a job execution"""

    returncode: int
    stdout: Optional[str]
    stderr: Optional[str]
    status: JobStatus


@dataclass
class DockerJobConfig:
    img_tag: str
    command: List[str]
    env_var: Optional[Dict[str, str]]
    volumes: Optional[List[str]]


class DockerJob:
    """Represents a single Docker job in the execution pool"""

    def __init__(
        self, docker_runner: DockerRunner, job_id: int, job_config: DockerJobConfig
    ):
        self.docker_runner = docker_runner
        self.img_tag = job_config.img_tag
        self.command = job_config.command
        self.id = job_id
        self.env_var = job_config.env_var
        self.volumes = job_config.volumes
        self.status = JobStatus.PENDING
        self.result: Optional[JobResult] = None
        self.container = None
        self.container_id = None

    def start(self) -> None:
        """Start the Docker container"""
        try:
            config = self.docker_runner._prepare_container_config(
                self.env_var, self.volumes
            )
            self.container = self.docker_runner.client.containers.run(
                self.img_tag,
                self.command,
                **config,
                detach=True,
            )
            self.container_id = self.container.id
            self.status = JobStatus.RUNNING
        except Exception as e:
            logger.error(f"Failed to start Docker job {self.id}: {str(e)}")
            self.status = JobStatus.FAILED
            self.result = JobResult(1, None, str(e), JobStatus.FAILED)

    def check_status(self) -> JobStatus:
        """Check the current status of the Docker container"""
        if not self.container_id:
            return self.status

        try:
            # Try to get the container by ID
            self.container = self.docker_runner.client.containers.get(self.container_id)
            self.container.reload()

            if self.container.status == "exited":
                logs = self.container.logs().decode()
                returncode = self.container.attrs["State"]["ExitCode"]
                self.status = (
                    JobStatus.COMPLETED if returncode == 0 else JobStatus.FAILED
                )
                self.result = JobResult(returncode, logs, None, self.status)

                if self.status == JobStatus.FAILED:
                    logger.error(
                        f"Docker job {self.id} failed with return code {returncode}"
                    )
                    if logs:
                        logger.error(f"Container logs: {logs}")

                # Clean up the container if it still exists
                try:
                    self.container.remove()
                except Exception as e:
                    logger.debug(
                        f"Container {self.container_id} was already removed: {str(e)}"
                    )

                self.container = None
                self.container_id = None

        except NotFound:
            # Container was removed (likely due to --rm flag)
            # We can consider this as completed since the container finished its execution
            self.status = JobStatus.COMPLETED
            self.result = JobResult(0, None, None, JobStatus.COMPLETED)
            self.container = None
            self.container_id = None
        except Exception as e:
            logger.error(f"Error checking Docker job {self.id} status: {str(e)}")
            self.status = JobStatus.FAILED
            self.result = JobResult(1, None, str(e), JobStatus.FAILED)
            self.container = None
            self.container_id = None

        return self.status


class Job:
    """Represents a single job in the execution pool"""

    def __init__(self, cmd: List[str], job_id: int):
        self.cmd = cmd
        self.id = job_id
        self.process: Optional[subprocess.Popen] = None
        self.status = JobStatus.PENDING
        self.result: Optional[JobResult] = None

    def start(self) -> None:
        """Start the job process"""
        try:
            self.process = subprocess.Popen(
                self.cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN),
            )
            self.status = JobStatus.RUNNING
        except Exception as e:
            logger.error(f"Failed to start job {self.id}: {str(e)}")
            self.status = JobStatus.FAILED
            self.result = JobResult(1, None, str(e), JobStatus.FAILED)

    def check_status(self) -> JobStatus:
        """Check the current status of the job"""
        if not self.process:
            return self.status

        returncode = self.process.poll()
        if returncode is not None:
            stdout, stderr = self.process.communicate()
            self.status = JobStatus.COMPLETED if returncode == 0 else JobStatus.FAILED
            self.result = JobResult(returncode, stdout, stderr, self.status)

            if self.status == JobStatus.FAILED:
                logger.error(f"Job {self.id} failed with return code {returncode}")
                if stderr:
                    logger.error(f"Error output: {stderr}")

        return self.status


class JobPool:
    """Manages a pool of jobs with parallel execution"""

    def __init__(self, max_parallel: int):
        self.max_parallel = max_parallel
        self.jobs: List[Any] = []  # Can be either Job or DockerJob
        self.active_jobs: List[Any] = []
        self.completed_jobs: List[Any] = []

    def add_job(self, job: Any) -> None:
        """Add a new job to the pool"""
        job.id = len(self.jobs)
        self.jobs.append(job)

    def run_jobs(self, description: str) -> bool:
        """Run all jobs in the pool with progress tracking"""
        with Progress(
            SpinnerColumn(),
            TextColumn("[bold blue]{task.description}"),
            BarColumn(),
            TaskProgressColumn(),
            TimeElapsedColumn(),
            transient=True,
        ) as progress:
            task = progress.add_task(description, total=len(self.jobs))

            try:
                while self.jobs or self.active_jobs:
                    # Start new jobs if possible
                    while self.jobs and len(self.active_jobs) < self.max_parallel:
                        job = self.jobs.pop(0)
                        job.start()
                        if job.status != JobStatus.FAILED:
                            self.active_jobs.append(job)
                        else:
                            progress.update(task, advance=1)

                    # Check active jobs
                    for job in self.active_jobs[:]:
                        status = job.check_status()
                        if status in [JobStatus.COMPLETED, JobStatus.FAILED]:
                            self.active_jobs.remove(job)
                            self.completed_jobs.append(job)
                            progress.update(task, advance=1)

                    time.sleep(0.1)

                # Check if any jobs failed
                failed_jobs = [
                    job for job in self.completed_jobs if job.status == JobStatus.FAILED
                ]
                if failed_jobs:
                    logger.error(f"{len(failed_jobs)} jobs failed")
                    return False

                return True

            except KeyboardInterrupt:
                logger.warning("Received interrupt, cleaning up jobs...")
                self._cleanup_jobs()
                return False

    def _cleanup_jobs(self) -> None:
        """Clean up all running jobs"""
        for job in self.active_jobs:
            if isinstance(job, Job) and job.process:
                try:
                    job.process.terminate()
                    job.process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    job.process.kill()
            elif isinstance(job, DockerJob) and job.container:
                try:
                    job.container.stop()
                    job.container.remove()
                except Exception as e:
                    logger.error(f"Error cleaning up Docker container: {str(e)}")


def execute_pool(
    cmds: List[List[str]], description: str, max_parallel_jobs: int = 1
) -> bool:
    """
    Execute a pool of commands in parallel with improved error handling and job management.

    Args:
        cmds: List of commands to execute
        description: Description for the progress bar
        max_parallel_jobs: Maximum number of parallel jobs

    Returns:
        bool: True if all jobs completed successfully, False otherwise
    """
    pool = JobPool(max_parallel_jobs)
    for cmd in cmds:
        pool.add_job(Job(cmd, len(pool.jobs)))
    return pool.run_jobs(description)


def execute_docker_pool(
    docker_runner: DockerRunner,
    jobs: List[DockerJobConfig],
    description: str,
    max_parallel_jobs: int = 1,
) -> bool:
    """
    Execute a pool of Docker jobs in parallel with improved error handling and job management.

    Args:
        docker_runner: DockerRunner instance
        jobs: List of job configurations, each containing:
            - img_tag: Docker image tag
            - command: Command to run in container
            - env_var: Optional environment variables
            - volumes: Optional volume mounts
        description: Description for the progress bar
        max_parallel_jobs: Maximum number of parallel jobs

    Returns:
        bool: True if all jobs completed successfully, False otherwise
    """
    pool = JobPool(max_parallel_jobs)
    for job_config in jobs:
        pool.add_job(DockerJob(docker_runner, len(pool.jobs), job_config))
    return pool.run_jobs(description)
