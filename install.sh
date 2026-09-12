#!/usr/bin/env bash
# Ein-Kommando-Installation für das Praktikum (Versuch 1 Kinematik, Versuch 2 Zustandsschätzung).
#
#   ./install.sh               Umgebung prüfen, fehlendes pygame besorgen, Interfaces bauen
#   ./install.sh --check       nur prüfen und erklären, nichts installieren
#   ./install.sh --without-ros ROS-Zweig bewusst auslassen (der Simulator braucht kein ROS)
#   ./install.sh --user        pygame/pytest per `pip --user` statt über das Systempaket
#   ./install.sh --mit-tests   danach Selbsttest: kurze Simulation + Unit-Tests (headless)
#
# Kein sudo, kein Internet nötig: ohne beides sagt das Skript, was fehlt und wer es einsetzen
# muss — es scheitert laut, nicht still. In Dateien im Home-Verzeichnis schreibt es nichts.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYGAME_HIDE_SUPPORT_PROMPT=1        # pygame bedankt sich sonst laut bei jedem Import
cd "$here"

check_nur=0; ohne_ros=0; mit_pip_user=0; mit_tests=0
for a in "$@"; do
  case "$a" in
    --check) check_nur=1 ;;
    --without-ros) ohne_ros=1 ;;
    --user) mit_pip_user=1 ;;
    --mit-tests) mit_tests=1 ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "unbekannte Option: $a (mit: --check --without-ros --user --mit-tests)" >&2; exit 2 ;;
  esac
done

fehler=0; hinweise=()
ok()   { printf '  \033[32mok\033[0m     %s\n' "$1"; }
wann () { printf '  \033[33mACHTUNG\033[0m %s\n' "$1"; }
fehlt(){ printf '  \033[31mFEHLT\033[0m   %s\n' "$1"; fehler=1; }

echo "Praktikum Mecanum-Simulator — Umgebung"
echo "------------------------------------------------------------------"

# ---------------------------------------------------------------- Python (Pflicht, 3.10+)
if python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3,10) else 1)' 2>/dev/null; then
  ok "python3 $(python3 -c 'import platform;print(platform.python_version())')"
else
  fehlt "python3 mindestens 3.10 nötig (dataclasses mit slots, Pattern matching)"
fi

# ---------------------------------------------------------------- pygame (Pflicht für das Fenster)
if python3 -c 'import pygame' >/dev/null 2>&1; then
  ok "pygame $(python3 -c 'import pygame;print(pygame.ver)' 2>/dev/null | tail -1)"
else
  fehlt "pygame fehlt — ohne pygame läuft der Simulator nur mit --headless"
  if [[ $check_nur == 0 ]]; then
    if [[ $mit_pip_user == 1 ]]; then
      echo "        versuche: pip3 install --user pygame"
      python3 -m pip install --user pygame \
        || python3 -m pip install --user --break-system-packages pygame \
        || echo "        keine Netzwerkverbindung? Dann: sudo apt install python3-pygame"
    else
      hinweise+=("pygame einmal installieren:  sudo apt install python3-pygame   (oder: ./install.sh --user)")
    fi
    python3 -c 'import pygame' >/dev/null 2>&1 && fehler=0 && ok "pygame jetzt vorhanden"
  fi
fi

# ---------------------------------------------------------------- Tests (nur für die Qualitätsschranke)
if python3 -m pytest --version >/dev/null 2>&1; then
  ok "pytest $(python3 -m pytest --version 2>/dev/null | head -1 | awk '{print $2}')"
else
  wann "pytest fehlt — nur tools/check.sh und die Betreuer-Tests brauchen das"
  if [[ $check_nur == 0 && $mit_pip_user == 1 ]]; then
    python3 -m pip install --user pytest \
      || python3 -m pip install --user --break-system-packages pytest || true
  else
    hinweise+=("für die Unit-Tests:  sudo apt install python3-pytest   (oder: ./install.sh --user)")
  fi
fi

# ---------------------------------------------------------------- ROS 2 (optional, für ros2 launch)
ros_setup=""
if [[ $ohne_ros == 0 ]]; then
  for f in "${ROS_SETUP:-}" "/opt/ros/${ROS_DISTRO:-kilted}/setup.bash" \
           /opt/ros/humble/setup.bash /opt/ros/jazzy/setup.bash /opt/ros/rolling/setup.bash; do
    [[ -n "$f" && -f "$f" ]] && { ros_setup="$f"; break; }
  done
  ros_name="$(basename "$(dirname "$ros_setup")" 2>/dev/null || echo ros)"
  if [[ -n "$ros_setup" ]]; then
    # set +u ist Pflicht: die ROS-Setup-Skripte lesen AMENT_*-Variablen, die beim ersten
    # Source noch ungesetzt sind — mit `set -u` stirbt das Skript hier lautlos.
    set +u; source "$ros_setup" 2>/dev/null || true; set -u
    if python3 -c 'import rclpy' >/dev/null 2>&1; then
      ok "ROS 2 (${ROS_DISTRO:-$ros_name}) — ros2 launch und ros2 topic funktionieren"
      if python3 -c 'import mecanum_lab_interfaces' >/dev/null 2>&1; then
        ok "mecanum_lab_interfaces gebaut (Spawn-Service mit eigenem Typ)"
      elif [[ $check_nur == 1 ]]; then
        wann "interfaces nicht gebaut — im Praktikumssaal: ./install.sh (dauert ~20 s)"
      elif command -v colcon >/dev/null 2>&1; then
        echo "  baue mecanum_lab_interfaces (colcon, --symlink-install) …"
        ( cd "$here" && colcon build --paths interfaces/mecanum_lab_interfaces --symlink-install ) \
          && ok "interfaces gebaut — für neue Shells: source install/setup.bash" \
          || wann "colcon build fehlgeschlagen — ohne die Interfaces läuft alles über den JSON-Handshake"
      else
        wann "colcon fehlt — ohne Gebauten Interfaces antwortet die Sim auf den JSON-Handshake (siehe docs/CONTRACT.md §4)"
      fi
    else
      wann "ROS 2 ist installiert, aber rclpy nicht importierbar — ROS-Anteile übersprungen"
    fi
  else
    wann "kein ROS 2 gefunden — der Simulator läuft trotzdem (./lab run, ./lab grade, ./lab sim --stub)"
    hinweise+=("ROS 2 würde ros2 launch und RViz bringen: rosinstall nach docs.ros.org, danach neu starten")
  fi
else
  hinweise+=("dieser Lauf hat ROS 2 absichtlich ausgelassen (--without-ros)")
fi

# ---------------------------------------------------------------- Schluss
echo "------------------------------------------------------------------"
for h in "${hinweise[@]:-}"; do [[ -n "$h" ]] && echo "  Hinweis: $h"; done

if [[ $mit_tests == 1 ]]; then
  echo "Selbsttest (ohne Fenster, ohne ROS):"
  SDL_VIDEODRIVER=dummy MECANUM_ROS=stub ./lab sim --headless --seconds 3 --robot test \
    && ok "Simulation läuft" || fehlt "Simulation läuft nicht"
  SDL_VIDEODRIVER=dummy MECANUM_ROS=stub python3 -m pytest tests -q \
    && ok "Unit-Tests grün" || fehlt "Unit-Tests nicht grün"
fi

if [[ $fehler == 1 ]]; then
  echo "Ergebnis: Umgebung unvollständig — siehe FEHLT-Zeilen oben."
  exit 1
fi
echo "Ergebnis: bereit. Weiter im Verzeichnis $here mit"
echo "  ./lab run --robot alice --controller student/controller_template.py"
echo "  ./lab grade --task kf_alle --controller student/kf_template.py --log messung.csv"
echo "  ros2 launch launch/kf.launch.py            (mit ROS 2)"
[[ -n "$ros_setup" ]] && echo "Vor jedem neuen Terminal, falls ROS nicht automatisch kommt:"
echo "  source ${ROS_SETUP:-$ros_setup}"
exit 0
