import platform
import subprocess
from dataclasses import dataclass
from pathlib import Path

from bazel_tools.tools.python.runfiles import runfiles

@dataclass
class PackageTree:
    root: Path
    source_dir: Path
    build_dir: Path
    install_dir: Path


def architecture() -> str:
    machine = platform.machine()
    assert machine in {
        "x86_64",
        "arm64",
    }, f"Unsupported build architecture {machine}"
    return machine


def build_number() -> int:
    arch = architecture()
    if system_is_linux():
        if arch == "x86_64":
            return 1  # Linux x86_64
        elif arch == "arm64":
            return 1  # Linux arm64
    if system_is_macos():
        if arch == "x86_64":
            return 1  # macOS x86_64
        elif arch == "arm64":
            return 1  # macOS arm64

    raise NotImplementedError("Unrecognized platform.")


def codename() -> str:
    if platform.system() == "Linux":
        import lsb_release

        return lsb_release.get_os_release()["CODENAME"]

    return "mac"


def _rlocation(relative_path: str) -> Path:
    """Return the real path to ``tools/workspace/vtk/{relative_path}``."""
    manifest = runfiles.Create()
    resource_path = f"drake/tools/workspace/vtk/{relative_path}"
    resolved_path = manifest.Rlocation(resource_path)
    assert resolved_path, f"Missing {resource_path}"
    return Path(resolved_path).resolve()


def system_is_linux():
    return platform.system() == "Linux"


def system_is_macos():
    return platform.system() == "Darwin"


def vtk_git_ref() -> str:
    return "d706250a1422ae1e7ece0fa09a510186769a5fec"


def vtk_package_tree() -> PackageTree:
    if system_is_linux():
        root = Path("/") / "vtk"
    else:
        # We are in tools/workspace/vtk/image/, build in vtk/mac_binary_build.
        this_file_real_path = _rlocation("image/vtk_common.py")
        tools_workspace_vtk_path = this_file_real_path.parent.parent.resolve()
        root = tools_workspace_vtk_path / "mac_binary_build"
    source_dir = root / "source"
    build_dir = root / "build"
    install_dir = root / "install"
    # TODO(svenevs): can remove this.?
    # if system_is_macos():
    #     install_dir = root / "install"
    # else:
    #     install_dir = Path("/") / "opt" / "vtk"
    return PackageTree(
        root=root,
        source_dir=source_dir,
        build_dir=build_dir,
        install_dir=install_dir,
    )


def vtk_version(source_dir: Path) -> str:
    proc = subprocess.run(
        ["git", "describe"], check=True, cwd=source_dir, stdout=subprocess.PIPE
    )
    return proc.stdout.decode("utf-8").strip()


def vtk_archive_name(source_dir: Path) -> str:
    version = vtk_version(source_dir)
    code = codename()
    arch = architecture()
    build = build_number()
    return f"vtk-{version}-{code}-{arch}-{build}.tar.gz"
