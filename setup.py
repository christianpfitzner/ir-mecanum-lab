"""Install the mecanum lab as a ROS 2 package (ament_python): `colcon build --paths . --symlink-install`.

The lab and its launch files run without this step, straight from the source tree. What the build adds is
`ros2 launch mecanum_lab <file>` from any directory — and `data_files` is the part to read: an installed
package has no `config/` next to its module, the data lives in `<prefix>/share/mecanum_lab/`, and
`types.data_root()` is what finds it there.
"""
import os

from setuptools import setup

PACKAGE = "mecanum_lab"
ROOT = os.path.dirname(os.path.abspath(__file__))
LEFTOVERS = (".aux", ".log", ".fls", ".toc", ".out", ".lof", ".lot", ".bbl", ".blg",
         ".fdb_latexmk", ".synctex.gz", ".pyc", ".pyo")     # build by-products


def _clean(rel):
    """Drop cache folders (__pycache__, .pytest_cache), dot files and LaTeX leftovers."""
    if any(part.startswith(".") or part == "__pycache__" for part in rel.split(os.sep)):
        return False
    return not rel.endswith(LEFTOVERS)


def data_files_in(folder):
    """Every file under `folder` into share/mecanum_lab/… — folder structure preserved.

    colcon demands relative sources (otherwise "'data_files' must be relative") and
    setuptools does not copy folders — hence paths relative to ROOT, without caches.
    """
    groups = {}
    for walk_root, subdirs, names in os.walk(os.path.join(ROOT, folder)):
        subdirs[:] = [u for u in subdirs if _clean(u)]
        rel = os.path.relpath(walk_root, ROOT)
        files = sorted(os.path.join(rel, n) for n in names if _clean(os.path.join(rel, n)))
        if files:
            groups[os.path.join("share", PACKAGE, rel)] = files
    return sorted(groups.items())


setup(
    name=PACKAGE,
    version="0.1.0",
    description="2D Mecanum simulator (Pygame) with ROS 2 binding, experiments 1 and 2",
    author="Praktikum Intelligente Robotik",
    license="MIT",
    python_requires=">=3.10",
    packages=[PACKAGE],
    # The ament resource marker first: `ros2 pkg list` reads that index, and until now the entry there
    # came only from colcon-ros adding it as a side effect. (package_data would be redundant.)
    data_files=([(os.path.join("share", "ament_index", "resource_index", "packages"),
                  ["resource/" + PACKAGE]),
                 (os.path.join("share", PACKAGE), ["package.xml"])]
                + [g for folder in ("config", "worlds", "launch", "student", "docs")
                   for g in data_files_in(folder)]),
    install_requires=["setuptools"],
    extras_require={"gui": ["pygame>=2.5"]},
    entry_points={"console_scripts": ["mecanum-lab = mecanum_lab.node:main"]},
    zip_safe=True,
)
