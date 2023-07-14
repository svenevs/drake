import hashlib
import shutil
import subprocess
import tarfile

from clone_vtk import clone_vtk

from vtk_cmake_args import cmake_configure_args

from vtk_common import (
    PackageTree,
    system_is_linux,
    vtk_archive_name,
    vtk_git_ref,
    vtk_package_tree,
)


def build_vtk(package_tree: PackageTree):
    # Always start with a clean CMake build tree.
    if package_tree.build_dir.is_dir():
        shutil.rmtree(package_tree.build_dir)
    elif package_tree.build_dir.is_file():
        package_tree.build_dir.unlink()
    package_tree.build_dir.mkdir(parents=True, exist_ok=False)
    subprocess.check_call(
        [
            "cmake",
            "-Werror",
            "-G",
            "Ninja",
            f"-DCMAKE_INSTALL_PREFIX:PATH={package_tree.install_dir}",
            "-DCMAKE_BUILD_TYPE:STRING=Release",
            *cmake_configure_args(),
            "-B",
            str(package_tree.build_dir),
            "-S",
            str(package_tree.source_dir),
        ]
    )
    subprocess.check_call(
        [
            "cmake",
            "--build",
            str(package_tree.build_dir),
            "--target",
            "install",
        ]  # this comment stops black from making the line too long.
    )

    archive_name = vtk_archive_name(package_tree.source_dir)
    tarball_path = package_tree.root / archive_name
    # Creating the .tgz can take a little while, display something to the user
    # so it's clear the script isn't stuck.
    print(f"==> Compressing {package_tree.install_dir} to {tarball_path}.")
    with tarfile.open(tarball_path, "w:gz") as tgz:
        # NOTE: the second argument to `add` is what to rename things as, by
        # using the empty string it means that all components are stripped.
        # This way, when extracting, the bin, lib, etc directories are created.
        tgz.add(package_tree.install_dir, "")

    # Create the accompanying sha256sum verification file.
    sha256_path = tarball_path.parent / f"{tarball_path.name}.sha256"
    print(f"==> Computing sha256sum of {tarball_path} into {sha256_path}.")
    sha256 = hashlib.sha256()
    with open(tarball_path, "rb", buffering=0) as tgz:
        while True:
            data = tgz.read(sha256.block_size)
            if not data:
                break
            sha256.update(data)
    with open(sha256_path) as f:
        f.write(f"{sha256.hexdigest()} {tarball_path.name}\n")


def main() -> None:
    package_tree = vtk_package_tree()
    package_tree.root.mkdir(parents=True, exist_ok=True)
    # On Linux the Dockerfile should have already staged our clone, assert that
    # it exists.  On macOS, perform the clone only if the folder does not
    # already exist (it can take some time to fully clone VTK).
    if system_is_linux():
        assert (
            package_tree.source_dir.is_dir()
        ), f"Expected {package_tree.source_dir} to be a directory."
    else:
        if not package_tree.source_dir.is_dir():
            clone_vtk(vtk_git_ref(), package_tree.source_dir)
    build_vtk(package_tree)


if __name__ == "__main__":
    main()
