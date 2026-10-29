"""6DoF ArUco plug-hole localization and 50Hz ONNX insertion control package."""

from pathlib import Path


def package_file(relative: str) -> Path:
    source = Path(__file__).resolve().parent.parent / relative
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory

    installed = Path(get_package_share_directory('upper_insert')) / relative
    if not installed.is_file():
        raise FileNotFoundError(installed)
    return installed
