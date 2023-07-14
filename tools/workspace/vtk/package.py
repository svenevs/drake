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

sys.path.insert(0, str(Path(__file__).parent.resolve()))
from image.vtk_common import architecture, _rlocation, vtk_package_tree, system_is_linux, system_is_macos, _rlocation, vtk_git_ref
from image.clone_vtk import clone_vtk
from image.vtk_cmake_args import cmake_configure_args

@dataclass
class DockerPlatform:
    """Map human friendly names to docker base image tags to build with."""

    codename: str
    """The common name, e.g., ``jammy``."""

    docker_tag: str
    """The ubuntu docker tag to use, e.g., ``ubuntu:22.04``."""


# Artifacts that need to be cleaned up. DO NOT MODIFY outside of this file.
_images_to_remove: list[str] = []
_paths_to_remove: list[Path] = []


@atexit.register
def _cleanup():
    """Remove any artifacts (docker images) on exit."""
    # TODO(svenevs): what should we clean up on macOS? (-k arg,paths_to_remove)
    if len(_images_to_remove):
        subprocess.check_call(
            [
                "docker",
                "image",
                "rm",
                *_images_to_remove,
            ],
        )


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
    # Gather the explicit information needed for directly configuring the CMake
    # C and C++ compilers, and configuring the Xcode toolchain.
    xcrun_proc = subprocess.run(
        [
            "xcrun", "-sdk", "macosx", "-show-sdk-path"
        ],
        check=True,
        stdout=subprocess.PIPE,
    )
    sysroot = xcrun_proc.stdout.decode("utf-8").strip()

    xcrun_proc = subprocess.run(
        [
            "xcrun", "-sdk", "macosx", "-find", "gcc",
        ],
        check=True,
        stdout=subprocess.PIPE,
    )
    xcode_c_compiler = xcrun_proc.stdout.decode("utf-8").strip()

    xcrun_proc = subprocess.run(
        [
            "xcrun", "-sdk", "macosx", "-find", "g++",
        ],
        check=True,
        stdout=subprocess.PIPE,
    )
    xcode_cxx_compiler = xcrun_proc.stdout.decode("utf-8").strip()

    print(f"=> Using Xcode toolchain:")
    print(f"   Sysroot:      {sysroot}")
    print(f"   C Compiler:   {xcode_c_compiler}")
    print(f"   C++ Compiler: {xcode_cxx_compiler}")

    package_tree = vtk_package_tree()
    # TODO(svenevs): what are we wanting to do to save macOS dev time vs clean up.
    if package_tree.root.exists():
        raise RuntimeError(
            f"ERROR: {package_tree.root} already exists.  This directory is retained "
            "when the --keep argument is given, refusing to delete."
        )
    print("=> Creating the following directories:")
    print(f"  Containment folder: {package_tree.root}")
    print(f"  VTK source code:    {package_tree.source_dir}")
    print(f"  VTK build tree:     {package_tree.build_dir}")
    print(f"  VTK install tree:   {package_tree.install_dir}")

    package_tree.root.mkdir(parents=True, exist_ok=False)
    clone_vtk(vtk_git_ref(), package_tree.source_dir)

    subprocess.check_call(
        [
            "cmake",
            # TODO(svenevs): yes, this should be used, you need to revisit the
            # dependencies again.  Seems the externals on macOS are different.
            # "-Werror",
            "-G", "Ninja",
            f"-DCMAKE_INSTALL_PREFIX:PATH={package_tree.install_dir}",
            "-DCMAKE_BUILD_TYPE:STRING=Release",
            f"-DCMAKE_OSX_ARCHITECTURES:STRING={architecture()}",
            "-DCMAKE_INSTALL_LIBDIR=lib",
            # TODO(svenevs): check NEVER
            "-DCMAKE_FIND_FRAMEWORK=LAST",
            f"-DCMAKE_OSX_SYSROOT={sysroot}",
            f"-DCMAKE_C_COMPILER={xcode_c_compiler}",
            f"-DCMAKE_CXX_COMPILER={xcode_cxx_compiler}",
            *cmake_configure_args(),
            "-B", str(package_tree.build_dir),
            "-S", str(package_tree.source_dir),
        ]
    )



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
    # Verify Linux or macOS once, everything else assumes !Linux => macOS.
    if system_is_linux():
        keep_help = "permanently tag docker container"
    elif system_is_macos():
        package_tree = vtk_package_tree()
        keep_help = (
            "retain the VTK checkout, build, and installation trees "
            f"contained in {package_tree.root}"
        )
    else:
        parser.error(f"Unsupported operating system: {platform.system()}")
    parser.add_argument("-k", "--keep", action="store_true", help=keep_help)

    # On Linux we build using docker and can build both, or one / the other.
    # On macOS we cannot build a "fat-binary" with both x86_64 and arm64
    # because of the VTK dependencies.
    if system_is_linux():
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

    if system_is_linux():
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
