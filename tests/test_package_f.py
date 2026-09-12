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


def reads(path):
    with open(path, encoding="utf-8") as fh:
        return fh.read()


def lines(path):
    return len(reads(path).splitlines())


# ------------------------------------------------------------------- SpawnRobot.srv


def test_srv_file_and_limits():
    assert os.path.exists(os.path.join(PKT, "srv", "SpawnRobot.srv"))
    assert os.path.exists(os.path.join(PKT, "CMakeLists.txt"))
    assert os.path.exists(os.path.join(PKT, "package.xml"))
    assert lines(os.path.join(PKT, "CMakeLists.txt")) <= 30


def test_srv_fields_as_in_the_contract():
    text = reads(os.path.join(PKT, "srv", "SpawnRobot.srv"))
    kopf, _, schwanz = text.partition("---")
    for teil, expected in ((kopf, ANFORDERUNG["req"]), (schwanz, ANFORDERUNG["resp"])):
        gefunde = {}
        for row_rect in teil.splitlines():
            row_rect = row_rect.split("#")[0].strip()
            if row_rect:
                typ, field = row_rect.split()[0], row_rect.split()[1]
                gefunde[field] = typ
        assert gefunde == expected, f"service fields deviate: {gefunde}"


def test_srv_without_extra_folders():
    assert not os.path.exists(os.path.join(PKT, "msg")), "Experiment 1 needs no messages"


# ---------------------------------------------------------------------- package.xml


@pytest.mark.parametrize("path,name,shape,max_lines", [
    (os.path.join(WURZEL, "package.xml"), "mecanum_lab", "ament_python", 30),
    (os.path.join(PKT, "package.xml"), "mecanum_lab_interfaces", "ament_cmake", 30),
])
def test_package_xml_wellformed_and_complete(path, name, shape, max_lines):
    repo = ET.parse(path).getroot()                     # raises on malformed XML
    assert repo.findtext("name") == name
    assert repo.findtext("export/build_type") == shape
    assert repo.findtext("license"), "license missing"
    assert repo.findtext("maintainer"), "maintainer missing"
    assert lines(path) <= max_lines


def test_main_package_declares_what_it_imports():
    repo = ET.parse(os.path.join(WURZEL, "package.xml")).getroot()
    abhaengigkeiten = {e.text for e in repo.findall("exec_depend")}
    for_needed = {"rclpy", "std_msgs", "geometry_msgs", "nav_msgs", "sensor_msgs",
                  "rosgraph_msgs"}
    assert for_needed <= abhaengigkeiten, f"missing exec_depend: {for_needed - abhaengigkeiten}"
    assert "std_srvs" in abhaengigkeiten                    # /sim/reset uses Trigger


def test_interface_package_is_an_rosidl_package():
    repo = ET.parse(os.path.join(PKT, "package.xml")).getroot()
    assert "rosidl_interface_packages" in {e.text for e in repo.findall("member_of_group")}
    cmake = reads(os.path.join(PKT, "CMakeLists.txt"))
    assert "rosidl_generate_interfaces" in cmake and "srv/SpawnRobot.srv" in cmake


# ---------------------------------------------------------------------------- setup.py


def setup_ast():
    return ast.parse(reads(os.path.join(WURZEL, "setup.py")))


def test_setup_py_limits_and_package_name():
    assert lines(os.path.join(WURZEL, "setup.py")) <= 60
    text = reads(os.path.join(WURZEL, "setup.py"))
    assert 'name=PACKAGE' in text or 'name="mecanum_lab"' in text or "name='mecanum_lab'" in text
    assert "mecanum_lab" in text


def test_console_script_points_at_main():
    text = reads(os.path.join(WURZEL, "setup.py")).replace("'", '"')
    assert "mecanum-lab = mecanum_lab.node:main" in text
    assert "entry_points" in text and "console_scripts" in text


def test_node_main_is_importable():
    from mecanum_lab.node import main                    # the entry-point target really exists
    assert callable(main)


def test_requirements_only_pygame():
    lines = [l.strip() for l in reads(os.path.join(WURZEL, "requirements.txt")).splitlines()
               if l.strip() and not l.strip().startswith("#")]
    assert len(lines) == 1 and lines[0].startswith("pygame"), lines


# ---------------------------------------------------------------------- launch files


@pytest.mark.parametrize("file_name,args", [
    ("sim.launch.py", {"world", "robots", "task", "seconds", "headless", "config",
                       "use_sim_time"}),
    ("student.launch.py", {"robot", "controller", "config", "use_sim_time"}),
    ("lab.launch.py", {"world", "robot", "robots", "controller", "task", "grade", "seconds",
                       "headless", "use_sim_time"}),
])
def test_launch_file_small_and_with_arguments(file_name, args):
    path = os.path.join(WURZEL, "launch", file_name)
    text = reads(path)
    assert lines(path) < 60, f"{file_name} over 60 lines"
    assert "generate_launch_description" in text
    ast.parse(text)
    for arg in args:
        assert f'"{arg}"' in text, f"{file_name}: argument {arg} missing"
    assert "DeclareLaunchArgument" in text


def test_headless_sets_the_sdl_driver():
    for file_name in ("sim.launch.py", "lab.launch.py"):
        text = reads(os.path.join(WURZEL, "launch", file_name))
        assert 'SDL_VIDEODRIVER' in text and '"dummy"' in text, file_name


def test_launch_starts_the_node_module_and_sets_pythonpath():
    for file_name in ("sim.launch.py", "student.launch.py", "lab.launch.py"):
        text = reads(os.path.join(WURZEL, "launch", file_name)).replace("'", '"')
        assert '"-m", "mecanum_lab.node"' in text, f"{file_name}: does not start the node module"
        assert "PYTHONPATH" in text, file_name
        assert "os.path.dirname" in text, f"{file_name}: paths must resolve relative to the file"


def test_launch_files_really_build_a_description():
    """Check the way `ros2 launch` later will: load the file by path, and the source
    tree must NOT be on sys.path while that happens — our `launch/` directory would
    shadow the ROS package `launch` as a namespace package. The test therefore runs
    in a subprocess without PYTHONPATH (and skips when ROS is missing)."""
    if os.environ.get("MECANUM_ROS", "").lower() == "stub":
        pytest.skip("deliberately run without ROS")
    code = """
import importlib.util, sys
for path in sys.argv[1:]:
    spec = importlib.util.spec_from_file_location("lf", path)
    modul = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modul)
    description = modul.generate_launch_description()
    names = sorted(a.name for a in description.entities
                   if type(a).__name__ == "DeclareLaunchArgument")
    print(len(description.entities), names)
"""
    path = os.environ.get("PYTHONPATH", "")          # keep the ROS paths, drop our source tree
    rest = [e for e in path.split(os.pathsep) if e and os.path.abspath(e) != WURZEL]
    env = dict(os.environ, PYTHONPATH=os.pathsep.join(rest))
    paths = [os.path.join(WURZEL, "launch", d) for d in
             ("sim.launch.py", "student.launch.py", "lab.launch.py")]
    run = subprocess.run([sys.executable, "-c", code, *paths], cwd="/tmp", env=env,
                          capture_output=True, text=True, timeout=60)
    if "No module named" in run.stderr and "launch" in run.stderr:
        pytest.skip("ROS 2 package 'launch' not sourced")
    assert run.returncode == 0, run.stderr[-600:]
    assert "world" in run.stdout and "controller" in run.stdout and "use_sim_time" in run.stdout


def test_topic_names_are_in_the_node_documentation():
    assert topic("wheels", "alice") == "/alice/wheel_speeds"
    assert topic("spawn") == "/sim/spawn_robot"
