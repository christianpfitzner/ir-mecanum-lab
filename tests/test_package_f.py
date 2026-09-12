"""Checks the ROS packaging — no ROS, no build, no window (agent F).

What is written here is deliberately static (XML/AST/text): a broken package.xml or a
.srv with renamed fields only surfaces in practice after a colcon build, and then
finding it takes forever. These tests catch it beforehand.
"""
import ast
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest

import mecanum_lab  # only for the root path
from mecanum_lab.types import topic

WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(mecanum_lab.__file__)))
PKT = os.path.join(WURZEL, "interfaces", "mecanum_lab_interfaces")

ANFORDERUNG = {                                        # CONTRACT §4, order does not matter
    "req": {"name": "string", "variant": "string"},
    # variant in the response is the extra field B expects (which variant it actually
    # became) — CONTRACT §4 does not list it, and it does no harm to any other field.
    "resp": {"success": "bool", "message": "string", "index": "uint8", "color": "string",
             "marker": "string", "variant": "string", "x": "float64", "y": "float64",
             "theta": "float64"},
}


def liest(pfad):
    with open(pfad, encoding="utf-8") as fh:
        return fh.read()


def zeilen(pfad):
    return len(liest(pfad).splitlines())


# ------------------------------------------------------------------- SpawnRobot.srv


def test_srv_datei_und_grenzen():
    assert os.path.exists(os.path.join(PKT, "srv", "SpawnRobot.srv"))
    assert os.path.exists(os.path.join(PKT, "CMakeLists.txt"))
    assert os.path.exists(os.path.join(PKT, "package.xml"))
    assert zeilen(os.path.join(PKT, "CMakeLists.txt")) <= 30


def test_srv_felder_wie_vertrag():
    text = liest(os.path.join(PKT, "srv", "SpawnRobot.srv"))
    kopf, _, schwanz = text.partition("---")
    for teil, soll in ((kopf, ANFORDERUNG["req"]), (schwanz, ANFORDERUNG["resp"])):
        gefunde = {}
        for zeile in teil.splitlines():
            zeile = zeile.split("#")[0].strip()
            if zeile:
                typ, feld = zeile.split()[0], zeile.split()[1]
                gefunde[feld] = typ
        assert gefunde == soll, f"service fields deviate: {gefunde}"


def test_srv_ohne_zusatzliche_ordner():
    assert not os.path.exists(os.path.join(PKT, "msg")), "Experiment 1 needs no messages"


# ---------------------------------------------------------------------- package.xml


@pytest.mark.parametrize("pfad,name,bauform,max_zeilen", [
    (os.path.join(WURZEL, "package.xml"), "mecanum_lab", "ament_python", 30),
    (os.path.join(PKT, "package.xml"), "mecanum_lab_interfaces", "ament_cmake", 30),
])
def test_package_xml_wohlgeformt_und_vollstaendig(pfad, name, bauform, max_zeilen):
    wurzel = ET.parse(pfad).getroot()                     # raises on malformed XML
    assert wurzel.findtext("name") == name
    assert wurzel.findtext("export/build_type") == bauform
    assert wurzel.findtext("license"), "license missing"
    assert wurzel.findtext("maintainer"), "maintainer missing"
    assert zeilen(pfad) <= max_zeilen


def test_hauptpaket_deklariert_was_es_importiert():
    wurzel = ET.parse(os.path.join(WURZEL, "package.xml")).getroot()
    abhaengigkeiten = {e.text for e in wurzel.findall("exec_depend")}
    for_needed = {"rclpy", "std_msgs", "geometry_msgs", "nav_msgs", "sensor_msgs",
                  "rosgraph_msgs"}
    assert for_needed <= abhaengigkeiten, f"missing exec_depend: {for_needed - abhaengigkeiten}"
    assert "std_srvs" in abhaengigkeiten                    # /sim/reset uses Trigger


def test_interfacepaket_ist_rosidl_paket():
    wurzel = ET.parse(os.path.join(PKT, "package.xml")).getroot()
    assert "rosidl_interface_packages" in {e.text for e in wurzel.findall("member_of_group")}
    cmake = liest(os.path.join(PKT, "CMakeLists.txt"))
    assert "rosidl_generate_interfaces" in cmake and "srv/SpawnRobot.srv" in cmake


# ---------------------------------------------------------------------------- setup.py


def setup_ast():
    return ast.parse(liest(os.path.join(WURZEL, "setup.py")))


def test_setup_py_grenzen_und_paketname():
    assert zeilen(os.path.join(WURZEL, "setup.py")) <= 60
    text = liest(os.path.join(WURZEL, "setup.py"))
    assert 'name=PAKET' in text or 'name="mecanum_lab"' in text or "name='mecanum_lab'" in text
    assert "mecanum_lab" in text


def test_console_script_zeigt_auf_main():
    text = liest(os.path.join(WURZEL, "setup.py")).replace("'", '"')
    assert "mecanum-lab = mecanum_lab.node:main" in text
    assert "entry_points" in text and "console_scripts" in text


def test_node_main_ist_importierbar():
    from mecanum_lab.node import main                    # the entry-point target really exists
    assert callable(main)


def test_requirements_nur_pygame():
    zeilen_ = [l.strip() for l in liest(os.path.join(WURZEL, "requirements.txt")).splitlines()
               if l.strip() and not l.strip().startswith("#")]
    assert len(zeilen_) == 1 and zeilen_[0].startswith("pygame"), zeilen_


# ---------------------------------------------------------------------- launch files


@pytest.mark.parametrize("datei,args", [
    ("sim.launch.py", {"world", "robots", "task", "seconds", "headless", "config",
                       "use_sim_time"}),
    ("student.launch.py", {"robot", "controller", "config", "use_sim_time"}),
    ("lab.launch.py", {"world", "robot", "robots", "controller", "task", "grade", "seconds",
                       "headless", "use_sim_time"}),
])
def test_launch_datei_klein_und_mit_argumenten(datei, args):
    pfad = os.path.join(WURZEL, "launch", datei)
    text = liest(pfad)
    assert zeilen(pfad) < 60, f"{datei} over 60 lines"
    assert "generate_launch_description" in text
    ast.parse(text)
    for arg in args:
        assert f'"{arg}"' in text, f"{datei}: argument {arg} missing"
    assert "DeclareLaunchArgument" in text


def test_headless_setzt_sdl_treiber():
    for datei in ("sim.launch.py", "lab.launch.py"):
        text = liest(os.path.join(WURZEL, "launch", datei))
        assert 'SDL_VIDEODRIVER' in text and '"dummy"' in text, datei


def test_launch_startt_node_modul_und_setzt_pythonpath():
    for datei in ("sim.launch.py", "student.launch.py", "lab.launch.py"):
        text = liest(os.path.join(WURZEL, "launch", datei)).replace("'", '"')
        assert '"-m", "mecanum_lab.node"' in text, f"{datei}: does not start the node module"
        assert "PYTHONPATH" in text, datei
        assert "os.path.dirname" in text, f"{datei}: paths must resolve relative to the file"


def test_launch_dateien_bauen_wirklich_eine_description():
    """Check the way `ros2 launch` later will: load the file by path, and the source
    tree must NOT be on sys.path while that happens — our `launch/` directory would
    shadow the ROS package `launch` as a namespace package. The test therefore runs
    in a subprocess without PYTHONPATH (and skips when ROS is missing)."""
    if os.environ.get("MECANUM_ROS", "").lower() == "stub":
        pytest.skip("deliberately run without ROS")
    code = """
import importlib.util, sys
for pfad in sys.argv[1:]:
    spec = importlib.util.spec_from_file_location("lf", pfad)
    modul = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modul)
    beschreibung = modul.generate_launch_description()
    namen = sorted(a.name for a in beschreibung.entities
                   if type(a).__name__ == "DeclareLaunchArgument")
    print(len(beschreibung.entities), namen)
"""
    pfad = os.environ.get("PYTHONPATH", "")          # keep the ROS paths, drop our source tree
    rest = [e for e in pfad.split(os.pathsep) if e and os.path.abspath(e) != WURZEL]
    umgebung = dict(os.environ, PYTHONPATH=os.pathsep.join(rest))
    pfade = [os.path.join(WURZEL, "launch", d) for d in
             ("sim.launch.py", "student.launch.py", "lab.launch.py")]
    lauf = subprocess.run([sys.executable, "-c", code, *pfade], cwd="/tmp", env=umgebung,
                          capture_output=True, text=True, timeout=60)
    if "No module named" in lauf.stderr and "launch" in lauf.stderr:
        pytest.skip("ROS 2 package 'launch' not sourced")
    assert lauf.returncode == 0, lauf.stderr[-600:]
    assert "world" in lauf.stdout and "controller" in lauf.stdout and "use_sim_time" in lauf.stdout


def test_thema_namen_stehen_in_der_doku_des_node():
    assert topic("wheels", "alice") == "/alice/wheel_speeds"
    assert topic("spawn") == "/sim/spawn_robot"
