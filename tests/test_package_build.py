"""Der ament_python-Bau darf nicht an `data_files` zerbrechen.

colcon bricht mit „'data_files' must be relative“ ab, sobald eine Quelle absolut ist
(früher: glob über den absoluten Paketpfad). Ausserdem kann setuptools nur Dateien
kopieren, keine Ordner — `docs/praktikum/` oder `__pycache__/` als Quelle würden erst
beim Installieren knallen. Dieser Test deckt beides ab, ohne ROS, colcon oder Netz.
"""
import os
import runpy
from unittest.mock import patch

import setuptools

WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _data_files():
    """setup.py ausführen, aber nur die Argumente von setup() abgreifen."""
    gefangen = {}

    def fange(**kwargs):
        gefangen.update(kwargs)

    with patch.object(setuptools, "setup", fange):
        runpy.run_path(os.path.join(WURZEL, "setup.py"), run_name="__setup__")
    return gefangen["data_files"]


def test_quellen_sind_relativ():
    for ziel, quellen in _data_files():
        assert not os.path.isabs(ziel), f"Ziel absolut: {ziel}"
        for quelle in quellen:
            assert not os.path.isabs(quelle), f"Quelle absolut: {quelle}"


def test_quellen_sind_dateien():
    for ziel, quellen in _data_files():
        for quelle in quellen:
            assert os.path.isfile(os.path.join(WURZEL, quelle)), f"kein File: {quelle}"


def test_laufzeitdaten_landen_im_share_baum():
    """node/launch brauchen config, worlds und launch unter share/mecanum_lab/."""
    anteile = {ziel: quellen for ziel, quellen in _data_files()}
    for ordner, datei in (("config", "default.json"), ("worlds", "arena.txt"),
                          ("launch", "lab.launch.py"), ("student", "solution.py")):
        quellen = anteile.get(os.path.join("share", "mecanum_lab", ordner), [])
        assert any(os.path.basename(q) == datei for q in quellen), f"{ordner}/{datei} fehlt"
    assert any(q == "package.xml" for q in anteile["share/mecanum_lab"])


def test_kein_cache_oder_latex_muell():
    for _, quellen in _data_files():
        for quelle in quellen:
            teile = quelle.split(os.sep)
            assert "__pycache__" not in teile, quelle
            assert not any(t.startswith(".") for t in teile), quelle
            assert not quelle.endswith((".aux", ".log", ".fls", ".toc", ".out")), quelle
