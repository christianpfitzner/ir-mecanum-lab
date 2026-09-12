#!/usr/bin/env bash
# Qualitaetsbarriere des Betreuers: alles, was gruen sein muss, bevor der Versuch
# freigegeben wird. Laeuft ohne ROS und ohne Fenster.
#
#   tools/check.sh            Basis-Checks (ohne ROS)
#   tools/check.sh --ros      zusaetzlich echte ROS-2-Nachweise (source wird erwartet)
set -uo pipefail
cd "$(dirname "$0")/.."; export SDL_VIDEODRIVER=dummy MECANUM_LOG=warning PYGAME_HIDE_SUPPORT_PROMPT=1
fail=0
step() { printf '\n\033[1m== %s\033[0m\n' "$1"; }
run() { if eval "$1"; then echo "  ok: $1"; else echo "  FEHLER: $1"; fail=1; fi; }

step "Statische Pruefung (Syntax aller Module)"
run "python3 -m compileall -q mecanum_lab student tools"
step "Unit-Tests (ohne ROS, ohne Fenster)"
run "python3 -m pytest tests -q"
step "LOC-Budget"
run "python3 tools/loc.py"
step "Kurzer Simulationslauf stub, Welt maze, mit Musterloesung"
run "./lab run --world maze --robot muster --controller student/solution.py --headless --seconds 6"
step "Bewertung der Musterloesung gegen allen Aufgaben (Versuch 1)"
# --controller ist kein Detail: ohne Knoten fährt niemand, und T2..T4 werden mit "kaum
# bewegt" bewertet — ein Prüfstand, der ohne die Musterloesung laeuft, prueft nichts.
run "./lab grade --robot muster --task alle --controller student/solution.py --json /tmp/grade.json --seconds 150"
step "Teleop-Pfad (pass-through) ohne Studierendenknoten"
run "./lab sim --world track --robots a,b --headless --seconds 3"

step "Versuch 2: arena-Welt, Installation, Messprotokoll-Werkzeug"
run "python3 tools/worldcheck.py --welt arena"
run "bash -n install.sh"
run "./install.sh --check"
if [[ -f student/kf_solution.py ]]; then
  step "Versuch 2: Bewertung der Musterloesung (kf_alle)"
  run "./lab grade --robot kf --task kf_alle --controller student/kf_solution.py --json /tmp/kf.json --seconds 230"
  step "Versuch 2: Messprotokoll und Auswertung"
  run "./lab grade --robot kf --task kf_gps --controller student/kf_solution.py --log /tmp/messung.csv --seconds 60"
  run "python3 tools/kfplot.py /tmp/messung.csv"
else
  echo "  uebersprungen: student/kf_solution.py fehlt noch"
fi
if [[ -f student/kf_template.py ]]; then
  step "Versuch 2: Template laeuft, ohne abzustuerzen"
  run "./lab run --world arena --task kf_gps --robot tpl --controller student/kf_template.py --headless --seconds 10 --truth"
else
  echo "  uebersprungen: student/kf_template.py fehlt noch"
fi

if [[ "${1:-}" == "--ros" ]]; then
  step "ROS 2: Topics und Services gegen laufende Simulation"
  # set +u ist Pflicht: die ROS-Setup-Skripte lesen AMENT_*-Variablen, die vor dem ersten
  # Source noch nicht gesetzt sind — mit `set -u` bricht der Source ab (wie in ./lab).
  set +u
  source /opt/ros/${ROS_DISTRO:-kilted}/setup.bash || { echo "ROS nicht sourced"; exit 1; }
  set -u
  # Die Sim muss laenger laufen als alle ROS-Nachweise darunter zusammen (Topics, Services,
  # drei Launch-Aufe); sonst wartet ein `ros2 topic hz` danach fuer immer auf Nachrichten.
  ./lab sim --world track --robots ros_test --headless --seconds 300 & sim=$!
  sleep 6
  run "ros2 topic list | grep -q /ros_test/odom"
  run "timeout 25 ros2 topic echo /ros_test/odom --once"
  run "timeout 25 ros2 topic echo /ros_test/scan --once | head -20"
  # Ein Rate-Test endet nie von selbst: der Timeout ist der Abbruch, nicht der Fehlerfall.
  # Deshalb in eine Datei messen und deren Inhalt pruefen — die Pipe wuerde pipefail den
  # Timeout als Fehler der Simulation melden lassen.
  run "timeout 25 ros2 topic hz --window 10 /ros_test/scan >/tmp/hz_check.txt 2>/dev/null; grep -q average\\ rate /tmp/hz_check.txt"
  # Der typisierte Spawn-Dienst braucht die gebauten Interfaces (mecanum_lab_interfaces);
  # ohne sie weicht der Bus auf den JSON-Handshake aus, und `ros2 service call` findet dann
  # keinen Dienst dieses Typs. Also nur pruefen, wenn das Interface importierbar ist.
  if python3 -c 'from mecanum_lab_interfaces.srv import SpawnRobot' >/dev/null 2>&1; then
    run "./lab spawn --name gast"
  else
    echo "  uebersprungen: Interfaces nicht gebaut — ./install.sh baut sie, bis dahin Spawn als JSON-Handshake"
    run "./lab robots"
  fi
  run "timeout 25 ros2 service call /sim/reset std_srvs/srv/Trigger || true"
  run "timeout 25 ros2 topic echo /sim/robots --once"
  step "ROS 2: Launch-Dateien"
  run "timeout 25 ros2 launch launch/sim.launch.py headless:=true seconds:=15"
  step "ROS 2: eigener Studierendenknoten gegen echte Sim"
  timeout 40 ros2 launch launch/lab.launch.py headless:=true seconds:=35 controller:=student/solution.py || true
  if [[ -f student/kf_template.py ]]; then
    step "ROS 2: Versuch 2 ueber kf.launch.py (KF-Themen in der echten Sim)"
    run "timeout 45 ros2 launch launch/kf.launch.py headless:=true sekunden:=25 aufgabe:=kf_gps controller:=student/kf_template.py"
  fi
  kill $sim 2>/dev/null
fi

step "Ergebnis"
[[ $fail == 0 ]] && echo "ALLE CHECKS GRUEN" || echo "MINDESTENS EIN CHECK FEHLERHAFT"
exit $fail
