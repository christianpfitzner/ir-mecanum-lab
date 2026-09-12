"""Install the mecanum lab as a ROS 2 package (ament_python).

The lab runs without this step straight from the source tree via `./lab`. Only build it
if you want `ros2 run`/`ros2 pkg`:

    colcon build --paths mecanum-lab --symlink-install

The launch files in the source tree work without a build (PYTHONPATH is set).
"""
import os

from setuptools import setup

PAKET = "mecanum_lab"
ROOT = os.path.dirname(os.path.abspath(__file__))
RESTE = (".aux", ".log", ".fls", ".toc", ".out", ".lof", ".lot", ".bbl", ".blg",
         ".fdb_latexmk", ".synctex.gz", ".pyc", ".pyo")     # build by-products


def _sauber(rel):
    """Drop cache folders (__pycache__, .pytest_cache), dot files and LaTeX leftovers."""
    if any(teil.startswith(".") or teil == "__pycache__" for teil in rel.split(os.sep)):
        return False
    return not rel.endswith(RESTE)


def daten(ordner):
    """Every file under `ordner` into share/mecanum_lab/… — folder structure preserved.

    colcon demands relative sources (otherwise "'data_files' must be relative") and
    setuptools does not copy folders — hence paths relative to ROOT, without caches.
    """
    gruppen = {}
    for basis, unter, namen in os.walk(os.path.join(ROOT, ordner)):
        unter[:] = [u for u in unter if _sauber(u)]
        rel = os.path.relpath(basis, ROOT)
        dateien = sorted(os.path.join(rel, n) for n in namen if _sauber(os.path.join(rel, n)))
        if dateien:
            gruppen[os.path.join("share", PAKET, rel)] = dateien
    return sorted(gruppen.items())


setup(
    name=PAKET,
    version="0.1.0",
    description="2D Mecanum simulator (Pygame) with ROS 2 binding, experiments 1 and 2",
    author="Praktikum Intelligente Robotik",
    license="MIT",
    python_requires=">=3.10",
    packages=[PAKET],
    # package_data would be redundant: data_files ships config/ and worlds/.
    data_files=([(os.path.join("share", PAKET), ["package.xml"])]
                + [g for ordner in ("config", "worlds", "launch", "student", "docs")
                   for g in daten(ordner)]),
    install_requires=["setuptools"],
    extras_require={"gui": ["pygame>=2.5"]},
    entry_points={"console_scripts": ["mecanum-lab = mecanum_lab.node:main"]},
    zip_safe=True,
)
