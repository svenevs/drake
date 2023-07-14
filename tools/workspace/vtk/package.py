"""Build and package the .tar.gz archives for VTK."""
from __future__ import annotations

import argparse
import atexit
import os
import platform
import subprocess
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from bazel_tools.tools.python.runfiles import runfiles

# NOTE: doing from image.vtk_common confuses mypy, there is no other reason to
# do this sys.path modification.
# from image.vtk_common import vtk_package_tree
sys.path.insert(0, str(Path(__file__).parent.absolute() / "image"))
from vtk_common import vtk_package_tree  # noqa: E402


@dataclass
class DockerPlatform:
    """Map human friendly names to docker base image tags to build with."""

    codename: str
    """The common name, e.g., ``jammy``."""

    docker_tag: str
    """The ubuntu docker tag to use, e.g., ``ubuntu:22.04``."""


# Artifacts that need to be cleaned up. DO NOT MODIFY outside of this file.
_images_to_remove: list[str] = []


@atexit.register
def _cleanup():
    """Remove any artifacts (docker images) on exit."""
    if len(_images_to_remove):
        subprocess.check_call(
            [
                "docker",
                "image",
                "rm",
                *_images_to_remove,
            ],
        )


def _rlocation(relative_path: str) -> Path:
    """Return the real path to ``tools/workspace/vtk/{relative_path}``."""
    manifest = runfiles.Create()
    resource_path = f"drake/tools/workspace/vtk/{relative_path}"
    resolved_path = manifest.Rlocation(resource_path)
    assert resolved_path, f"Missing {resource_path}"
    return Path(resolved_path).resolve()


def build_linux(output_dir: Path, platforms: list[DockerPlatform], keep: bool):
    dockerfile_path = _rlocation("Dockerfile")
    tools_workspace_vtk_dir = dockerfile_path.parent.resolve()
    env = os.environ.copy()
    env["DOCKER_BUILDKIT"] = "1"
    for docker_platform in platforms:
        # Generate a unique identifier for this build.
        salt = os.urandom(8).hex()
        time = datetime.now(timezone.utc).strftime("%Y%m%d%H%M%S")
        tag = f"vtk:{time}-{salt}-{docker_platform.codename}"

        # Build and tag the vtk image.
        print(f"==> Building: {tag}")
        subprocess.check_call(
            [
                "docker",
                "build",
                "--force-rm",
                "--tag",
                tag,
                "--build-arg",
                f"PLATFORM={docker_platform.docker_tag}",
                str(tools_workspace_vtk_dir),
            ],
            env=env,
        )

        # Only add after it has successfully built.
        if not keep:
            _images_to_remove.append(tag)

        # Extract the .tar.gz archive.
        container_name = f"{tag}.extract".replace(":", ".")
        package_tree = vtk_package_tree()
        ls_proc = subprocess.run(
            [
                "docker",
                "run",
                f"--name={container_name}",
                "bash",
                "-c",
                f"ls {package_tree.root}/vtk-*.tar.gz{{,.sha256}}",
            ],
            env=env,
            stdout=subprocess.PIPE,
        )

        try:
            vtk_paths = ls_proc.stdout.decode("utf-8").splitlines()
            # There should be 1 vtk-*.tar.gz, and 1 vtk-*.tar.gz.sha256.
            assert len(vtk_paths) == 2, f"Expected two files in {vtk_paths}."
            for container_path in vtk_paths:
                basename = Path(container_path).name
                subprocess.check_call(
                    [
                        "docker",
                        "cp",
                        f"{container_name}:{container_path}",
                        str(output_dir / basename),
                    ]
                )
        finally:
            subprocess.check_call(["docker", "rm", container_name])


def build_macos(output_dir: Path, keep: bool):
    pass


def main() -> None:
    # Work around `bazel run` changing the working directory; this is to allow
    # the user to pass in relative paths in a sane manner.
    real_cwd = os.environ.get("BUILD_WORKING_DIRECTORY")
    if real_cwd is not None:
        os.chdir(real_cwd)

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "-o",
        "--output-dir",
        metavar="DIR",
        type=Path,
        default=Path(".").resolve(),
        help="directory to put the final .tar.gz in.  Default: %(default)s",
    )

    # On Linux we have docker, on macOS we just build in a dedicated location.
    system_is_linux = platform.system() == "Linux"
    if system_is_linux:
        keep_help = "permanently tag docker container"
        keep_help = "keep the VTK build directory"
    else:
        parser.error(f"Unsupported operating system: {platform.system()}")
    parser.add_argument("-k", "--keep", action="store_true", help=keep_help)

    # On Linux we build using docker and can build both, or one / the other.
    # On macOS we cannot build a "fat-binary" with both x86_64 and arm64
    # because of the VTK dependencies.
    if system_is_linux:
        focal = DockerPlatform("focal", "ubuntu:20.04")
        jammy = DockerPlatform("jammy", "ubuntu:22.04")
        all_platforms = [focal, jammy]  # used after parsing args too
        parser.add_argument(
            "-p",
            "--platform",
            type=str,
            choices=["all"] + [p.codename for p in all_platforms],
            default="all",
            help="which distribution(s) to build.  Default: %(default)s",
        )

    args = parser.parse_args()

    if system_is_linux:
        if args.platform == "all":
            platforms = all_platforms
        else:
            platforms = [
                p for p in all_platforms if args.platform == p.codename
            ]
        build_linux(args.output_dir, platforms, args.keep)
    else:
        build_macos(args.output_dir, args.keep)


if __name__ == "__main__":
    main()
