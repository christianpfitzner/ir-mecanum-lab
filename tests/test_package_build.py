"""The ament_python build must not break on `data_files`.

colcon aborts with "'data_files' must be relative" as soon as a source is absolute
(earlier: a glob over the absolute package path). setuptools can also only copy
files, not directories — `docs/praktikum/` or `__pycache__/` as a source would only
fail at install time. This test covers both, without ROS, colcon or network.
"""
import os
import runpy
from unittest.mock import patch

import setuptools

WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _data_files():
    """Run setup.py but capture only the arguments passed to setup()."""
    gefangen = {}

    def fange(**kwargs):
        gefangen.update(kwargs)

    with patch.object(setuptools, "setup", fange):
        runpy.run_path(os.path.join(WURZEL, "setup.py"), run_name="__setup__")
    return gefangen["data_files"]


def test_quellen_sind_relativ():
    for ziel, quellen in _data_files():
        assert not os.path.isabs(ziel), f"target is absolute: {ziel}"
        for quelle in quellen:
            assert not os.path.isabs(quelle), f"source is absolute: {quelle}"


def test_quellen_sind_dateien():
    for ziel, quellen in _data_files():
        for quelle in quellen:
            assert os.path.isfile(os.path.join(WURZEL, quelle)), f"not a file: {quelle}"


def test_laufzeitdaten_landen_im_share_baum():
    """node/launch need config, worlds and launch under share/mecanum_lab/."""
    anteile = {ziel: quellen for ziel, quellen in _data_files()}
    for ordner, datei in (("config", "default.json"), ("worlds", "arena.txt"),
                          ("launch", "lab.launch.py"), ("student", "solution.py")):
        quellen = anteile.get(os.path.join("share", "mecanum_lab", ordner), [])
        assert any(os.path.basename(q) == datei for q in quellen), f"{ordner}/{datei} missing"
    assert any(q == "package.xml" for q in anteile["share/mecanum_lab"])


def test_kein_cache_oder_latex_muell():
    for _, quellen in _data_files():
        for quelle in quellen:
            teile = quelle.split(os.sep)
            assert "__pycache__" not in teile, quelle
            assert not any(t.startswith(".") for t in teile), quelle
            assert not quelle.endswith((".aux", ".log", ".fls", ".toc", ".out")), quelle
