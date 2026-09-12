"""Versuch 2 (Zustandsschätzung) — alles Wichtige im Launch-File konfigurieren.

    ros2 launch launch/kf.launch.py                                  # K1 mit Musterlösung
    ros2 launch launch/kf.launch.py aufgabe:=kf_fusion \
        controller:=student/kf_template.py gps_luecke:=[14,8] gps_sigma:=1.2
    ros2 launch launch/kf.launch.py bewerten:=kf_alle headless:=true \
        controller:=student/kf_solution.py protokoll:=messung.csv
    ros2 launch launch/kf.launch.py aufgabe:=kf_dynamik imu_rate:=400 \
        imu_accel_bias:=0.12 rviz:=true

Jedes Sensor-Argument ist dieselbe Einstellung, die man ohne ROS so angibt:
`./lab sim --set gps.sigma_xy=1.2 --set imu.rate=400` (Tabelle unten in EINSTELLUNGEN,
auch in der Versuchsanleitung). Ein Argument, das leer bleibt, wird nicht gesetzt — dann
gewinnt das Prüfprofil des Auftrags aus config/tasks.json; ein gesetztes Argument gewinnt
immer. `wahrheit:=true` (Standard) veröffentlicht die exakte Pose auf /<robot>/truth:
die braucht der Bewerter, ohne sie ist keine Bewertung möglich.

Gestartet werden zwei Prozesse: Simulator (mit optionalem Bewerter) und dein Knoten;
dazu optional `ros2 bag record` für die Auswertung zu Hause und `rviz2`.
"""
import os
import sys

from launch import LaunchDescription
import launch.actions as L
from launch.substitutions import LaunchConfiguration as Halt

WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WAHR = ("true", "1", "yes", "on")

# Launch-Argument -> Pfad in der Simulator-Konfiguration (types.DEFAULT_CONFIG); leer = nicht setzen
EINSTELLUNGEN = [
    ("gps_rate", "gps.rate", "GPS-Messungen pro Sekunde"),
    ("gps_sigma", "gps.sigma_xy", "GPS-Streuung je Achse in m"),
    ("gps_sigma_theta", "gps.sigma_theta", "GPS-Streuung des Kurses in rad"),
    ("gps_bias", "gps.bias_xy", "fester GPS-Versatz als [dx,dy] in m"),
    ("gps_luecke", "gps.gap", "Funkloch als [start, dauer] in s nach Auftragsbeginn"),
    ("gps_bias_step", "gps.bias_step", "springender Bias [start, dauer, dx, dy]"),
    ("imu_rate", "imu.rate", "IMU-Stichproben pro Sekunde"),
    ("imu_gyro_noise", "imu.gyro_noise", "Drehrat-Rauschendichte rad/s/√Hz"),
    ("imu_gyro_bias", "imu.gyro_bias", "Drehraten-Bias 1σ in rad/s"),
    ("imu_gyro_walk", "imu.gyro_bias_walk", "Bias-Random-Walk rad/s/√s"),
    ("imu_accel_noise", "imu.accel_noise", "Beschleunigungs-Rauschendichte m/s²/√Hz"),
    ("imu_accel_bias", "imu.accel_bias", "Beschleunigungs-Bias 1σ in m/s²"),
    ("imu_accel_walk", "imu.accel_bias_walk", "Bias-Random-Walk m/s²/√s"),
    ("imu_skala", "imu.accel_scale", "relativer Skalenfehler der Beschleunigung"),
    ("imu_neigung", "imu.tilt_sigma", "Neigung der IMU 1σ in rad (Schwerkraft sickert rein)"),
    ("imu_neigung_tau", "imu.tilt_tau", "Korrelationszeit der Neigung in s"),
    ("imu_vibration", "imu.vibration", "Amplitude der Fahrwerksschwingung in m/s²"),
    ("imu_vibration_hz", "imu.vibration_hz", "Frequenz dieser Schwingung in Hz"),
    ("imu_einschwingen", "imu.startup", "Einschwingzeit nach dem Einschalten in s"),
    ("imubias_start", "imu.startup_bias", "zusätzlicher Bias während des Einschwingens m/s²"),
    ("odom_sigma_rad", "odom.sigma_wheel", "Rauschen je gemessener Radgeschwindigkeit"),
    ("odom_bias_omega", "odom.bias_omega", "systematischer Drehratenfehler rad/s"),
    ("odom_sigma_xy", "odom.sigma_xy", "Messrauschen der ausgegebenen Odometrie-Pose in m"),
    ("truth_rate", "truth.rate", "Rate der exakten Pose in Hz (Bewertung)"),
    ("physik_rate", "rate", "Physikschritte pro Sekunde"),
    ("gui_rate", "gui_rate", "Bildpunkte pro Sekunde"),
    ("kf_rate", "kf.rate", "Startempfehlung für deine Filterrate (nur Hinweis)"),
    ("kf_q", "kf.q_acc", "Startempfehlung für dein Q in m²/s³ (nur Hinweis)"),
    ("kf_q_gier", "kf.q_turn", "Startempfehlung für dein Gier-Q (nur Hinweis)"),
]

GRUNDLEGENDES = [
    ("welt", "arena", "Umgebung — für die KF-Aufträge ist arena gedacht"),
    ("robot", "alice", "dein Robotername (eine Person, ein Roboter)"),
    ("robots", "", "Roboter, die beim Start gespawnt werden (leer = nur dein Roboter)"),
    ("controller", "student/kf_template.py", "dein Filterknoten (Datei relativ zum Quellbaum)"),
    ("aufgabe", "kf_gps", "kf_gps | kf_fusion | kf_kovarianz | kf_dynamik (leer = frei fahren)"),
    ("bewerten", "", "Auftrag oder Gruppe zum Bewerten, z. B. kf_alle (leer = nicht bewerten)"),
    ("sekunden", "0", "nach N Sekunden Simulationszeit enden (0 = bis q/Strg-C)"),
    ("headless", "false", "ohne Pygame-Fenster (setzt SDL_VIDEODRIVER=dummy)"),
    ("wahrheit", "true", "exakte Pose auf /<robot>/truth — für die Bewertung nötig"),
    ("seed", "1", "Rausch-Seed: gleiches Seed, gleiche Messreihe (Nachfahrbarkeit)"),
    ("config", "", "zusätzliche JSON-Config (überschreibt config/default.json)"),
    ("protokoll", "", "CSV-Messprotokoll, z. B. messung.csv — damit arbeitet tools/kfplot.py"),
    ("protokoll_intervall", "0.05", "Abstand der Protokollzeilen in s Simulationszeit"),
    ("aufzeichnung", "", "ros2-bag-Name, z. B. kf_versuch (leer = keine Aufzeichnung)"),
    ("rviz", "false", "rviz2 mit rviz/kf.rviz dazustarten, falls vorhanden"),
    ("log_stufe", "info", "info | debug | warning"),
    ("use_sim_time", "true", "Simulationszeit (/clock) für Zeitstempel verwenden"),
]


def _pfad(angabe: str) -> str:
    """Relativ zum Quellbaum, absolut bleibt absolut — die Datei darf von überall kommen."""
    return angabe if os.path.isabs(angabe) else os.path.join(WURZEL, angabe)


def aufbau(context, *args, **kwargs):
    """Aus den Argumenten zwei (bis vier) Prozesse bauen."""
    hol = lambda name: Halt(name).perform(context)                        # noqa: E731
    kopflos = hol("headless").lower() in WAHR
    umgebung = {"PYTHONPATH": os.pathsep.join([WURZEL, os.environ.get("PYTHONPATH", "")]),
                "MECANUM_LOG": hol("log_stufe"),
                "MECANUM_USE_SIM_TIME": "1" if hol("use_sim_time").lower() in WAHR else "0"}
    if kopflos:
        umgebung["SDL_VIDEODRIVER"] = "dummy"

    sim = [sys.executable, "-m", "mecanum_lab.node", "sim",
           "--world", hol("welt"), "--robots", hol("robots") or hol("robot"),
           "--seconds", hol("sekunden"), "--seed", hol("seed")]
    if hol("aufgabe"):
        sim += ["--task", hol("aufgabe")]
    if hol("config"):
        sim += ["--config", _pfad(hol("config"))]
    for name, pfad, _beschreibung in EINSTELLUNGEN:
        if hol(name):
            sim += ["--set", f"{pfad}={hol(name)}"]
    if hol("wahrheit").lower() in WAHR:
        sim.append("--truth")
    if hol("protokoll"):
        sim += ["--log", _pfad(hol("protokoll")), "--log-intervall",
                hol("protokoll_intervall")]
    if hol("bewerten"):
        sim += ["--grade", hol("bewerten"), "--robot", hol("robot")]
    if kopflos:
        sim.append("--headless")

    knoten = [sys.executable, "-m", "mecanum_lab.node", "controller",
              "--robot", hol("robot"), "--controller", _pfad(hol("controller"))]

    teile = [L.ExecuteProcess(cmd=sim, additional_env=umgebung, output="screen",
                              name="mecanum_sim", emulate_tty=True),
             L.ExecuteProcess(cmd=knoten, additional_env=umgebung, output="screen",
                              name=f"knoten_{hol('robot')}", emulate_tty=True)]
    if hol("aufzeichnung"):
        themen = [f"/{hol('robot')}/{ende}" for ende in
                  ("odom", "gps", "imu", "kf/pose", "truth")] + ["/sim/robots", "/sim/config"]
        teile.append(L.ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", _pfad(hol("aufzeichnung")), *themen],
            additional_env=umgebung, output="screen", name="aufzeichnung"))
    rviz_config = os.path.join(WURZEL, "rviz", "kf.rviz")
    if hol("rviz").lower() in WAHR and os.path.exists(rviz_config):
        teile.append(L.ExecuteProcess(cmd=["rviz2", "-d", rviz_config],
                                      additional_env=umgebung, output="screen", name="rviz"))
    sensorik = "  ".join(f"{name}={hol(name)}" for name, _, _ in EINSTELLUNGEN if hol(name))
    teile.insert(0, L.LogInfo(msg=f"[kf] Auftrag: {hol('aufgabe') or '—'} · Welt: {hol('welt')} · "
                                  f"Roboter: {hol('robots') or hol('robot')} · Knoten: "
                                  f"{hol('controller')} · Sensorik: {sensorik or 'Prüfprofil'}"))
    return teile


def generate_launch_description():
    argumente = [L.DeclareLaunchArgument(name, default_value=default, description=text)
                 for name, default, text in GRUNDLEGENDES]
    argumente += [L.DeclareLaunchArgument(name, default_value="", description=text)
                  for name, _, text in EINSTELLUNGEN]
    return LaunchDescription(argumente + [L.OpaqueFunction(function=aufbau)])
