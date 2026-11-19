"""Assemble the static SuspensionGeometryLab site for GitHub Pages."""

import argparse
from fnmatch import fnmatch
from pathlib import Path
from shutil import copy2, copytree, ignore_patterns


DESKTOP_MODULES = (
    "__main__.py",
    "chart_base.py",
    "charts.py",
    "gui.py",
    "model_notes.py",
    "widgets.py",
    "*_dialog.py",
    "tk_*.py",
    "tire_*.py",
)


def build(destination: Path) -> None:
    web = Path(__file__).resolve().parent
    root = web.parent
    package = root / "suspension_geometry_lab"
    destination = destination.resolve()
    if destination == root or destination.is_relative_to(web) or destination.is_relative_to(package):
        raise ValueError("Choose a build directory outside docs and suspension_geometry_lab.")

    copytree(web, destination, dirs_exist_ok=True, ignore=ignore_patterns("build.py", "__pycache__", "*.pyc"))
    for source in package.rglob("*.py"):
        relative = source.relative_to(package)
        if "_pacejka" in relative.parts or any(fnmatch(source.name, pattern) for pattern in DESKTOP_MODULES):
            continue
        target = destination / package.name / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        copy2(source, target)

    copy2(package / "default_project.json", destination / package.name / "default_project.json")
    copy2(root / "LICENSE", destination / "LICENSE")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("destination", type=Path)
    build(parser.parse_args().destination)
