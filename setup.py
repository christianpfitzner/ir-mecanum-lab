"""Installiert den Mecanum-Versuch als ROS-2-Paket (ament_python).

Der Versuch laeuft auch ohne diesen Schritt direkt im Quellbaum ueber `./lab`.
Gebaut wird nur, wer `ros2 run`/`ros2 pkg` benutzen will:

    colcon build --paths mecanum-lab --symlink-install

Die Launch-Dateien im Quellbaum funktionieren ohne Build (PYTHONPATH wird gesetzt).
"""
import glob
import os

from setuptools import setup

PAKET = "mecanum_lab"
ROOT = os.path.dirname(os.path.abspath(__file__))


def daten(ordner):
    """Alle Dateien eines Ordners nach share/mecanum_lab/<ordner> — leerer Ordner: nichts."""
    dateien = sorted(glob.glob(os.path.join(ROOT, ordner, "*")))
    return [(os.path.join("share", PAKET, ordner), dateien)] if dateien else []


setup(
    name=PAKET,
    version="0.1.0",
    description="2D-Mecanum-Simulator (Pygame) mit ROS-2-Anbindung, Versuch 1",
    author="Praktikum Intelligente Robotik",
    license="MIT",
    python_requires=">=3.10",
    packages=[PAKET],
    package_data={PAKET: ["../worlds/*.txt", "../config/*.json"]},
    data_files=([(os.path.join("share", PAKET), ["package.xml"])]
                + [d for ordner in ("config", "worlds", "launch", "student", "docs")
                   for d in daten(ordner)]),
    install_requires=["setuptools"],
    extras_require={"gui": ["pygame>=2.5"]},
    entry_points={"console_scripts": ["mecanum-lab = mecanum_lab.node:main"]},
    zip_safe=True,
)
