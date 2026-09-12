"""Installiert den Mecanum-Versuch als ROS-2-Paket (ament_python).

Der Versuch laeuft auch ohne diesen Schritt direkt im Quellbaum ueber `./lab`.
Gebaut wird nur, wer `ros2 run`/`ros2 pkg` benutzen will:

    colcon build --paths mecanum-lab --symlink-install

Die Launch-Dateien im Quellbaum funktionieren ohne Build (PYTHONPATH wird gesetzt).
"""
import os

from setuptools import setup

PAKET = "mecanum_lab"
ROOT = os.path.dirname(os.path.abspath(__file__))
RESTE = (".aux", ".log", ".fls", ".toc", ".out", ".lof", ".lot", ".bbl", ".blg",
         ".fdb_latexmk", ".synctex.gz", ".pyc", ".pyo")     # Nebenprodukte beim Bauen


def _sauber(rel):
    """Cacheordner (__pycache__, .pytest_cache), versteckte Dateien und LaTeX-Reste raus."""
    if any(teil.startswith(".") or teil == "__pycache__" for teil in rel.split(os.sep)):
        return False
    return not rel.endswith(RESTE)


def daten(ordner):
    """Alle Dateien unter `ordner` nach share/mecanum_lab/… — Ordnerstruktur bleibt.

    colcon verlangt relative Quellen (sonst „'data_files' must be relative“) und
    setuptools kopiert keine Ordner — deshalb Dateien relativ zu ROOT, ohne Cache.
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
    description="2D-Mecanum-Simulator (Pygame) mit ROS-2-Anbindung, Versuch 1",
    author="Praktikum Intelligente Robotik",
    license="MIT",
    python_requires=">=3.10",
    packages=[PAKET],
    # package_data waere ueberfluessig: config/ und worlds/ liefert data_files.
    data_files=([(os.path.join("share", PAKET), ["package.xml"])]
                + [g for ordner in ("config", "worlds", "launch", "student", "docs")
                   for g in daten(ordner)]),
    install_requires=["setuptools"],
    extras_require={"gui": ["pygame>=2.5"]},
    entry_points={"console_scripts": ["mecanum-lab = mecanum_lab.node:main"]},
    zip_safe=True,
)
