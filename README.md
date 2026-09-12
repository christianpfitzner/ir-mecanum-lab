# Mecanum-Labor — zwei Versuche in einem 2D-Simulator

Ein Python-Simulator für das Praktikum: **Versuch 1** lehrt Mecanum-Kinematik (der
Studierende fährt Aufgaben), **Versuch 2** lehrt Zustandsschätzung (der Simulator fährt, der
Studierende schätzt mit Kalman-Filter). Beides läuft headless, ohne ROS 2 und ohne numpy —
ROS 2 ist eine optionale Schicht, keine Voraussetzung.

## Installation in einem Befehl

```bash
./install.sh              # prüft python/pygame/pytest/ROS und bietet an, was fehlt
./install.sh --check      # nur berichten, nichts installieren
./install.sh --mit-tests  # dazu ein kurzer Selbsttest (Simulation + Unit-Tests, headless)
```

`./install.sh` schreibt nichts ins Home-Verzeichnis und braucht kein sudo. Ohne
Internetzugang: `sudo apt install python3-pygame` (oder `./install.sh --user`).

## Erste Schritte — Versuch 1 (Kinematik)

```bash
./lab run --robot alice --controller student/controller_template.py   # Fenster, selbst fahren
./lab grade --task v1 --controller student/solution.py               # alle Aufgaben bewerten
```

## Erste Schritte — Versuch 2 (Zustandsschätzung)

Die Simulation fährt; der Knoten misst nur und meldet seine Schätzung auf
`/<robot>/kf/pose` — Position **und** die eigene 1σ, denn die Note hängt an der Unsicherheit.

```bash
./lab run --task kf_gps --robot alice --controller student/kf_template.py --truth
./lab grade --task kf_alle --controller student/kf_template.py --log messung.csv
python3 tools/kfplot.py messung.csv       # Zahlen + ASCII-Diagramm, ohne matplotlib
```

Vier Aufgaben: `kf_gps` (CV-Modell, nur GPS) → `kf_fusion` (GPS + Odometrie + IMU durch ein
8-s-Funkloch) → `kf_kovarianz` (die angegebene σ muss zum Fehler passen, NEES) →
`kf_dynamik` (schnell, 200 Hz IMU). Arbeitsblatt: `docs/praktikum/kalman.tex` (`make kalman`).

## Mit ROS 2 (Kilted oder neuer)

```bash
source /opt/ros/kilted/setup.bash
./lab sim --robots alice,bob                      # Simulator als echter ROS-Knoten
./lab spawn --name carlo                          # Roboter zu einer laufenden Sim hinzugesetzt
ros2 launch launch/kf.launch.py                   # Versuch 2, alles konfigurierbar
ros2 launch launch/kf.launch.py bewerten:=kf_alle controller:=student/kf_solution.py
ros2 launch launch/sim.launch.py robots:=alice,bob  # Versuch 1
ros2 topic echo /alice/imu --once                 # az in Ruhe ≈ +9,81 — das ist richtig
```

Wer den In-Prozess-Bus auch mit ROS will: `MECANUM_ROS=stub ./lab sim --headless`.

## Auftrag und Halle gehören zusammen

`--task` nimmt Auftrags-IDs, Gruppen oder eine Kommaliste: `v1` und `alle` (Versuch 1),
`kf_alle`/`v2` (Versuch 2), `beide` (wirklich alle). Jede Aufgabe nennt in
`config/tasks.json` ihre Halle (`"welt"`); `./lab` und `tools/fastgrade.py` folgen dieser
Angabe, `--world` überschreibt sie. Wer in der falschen Halle bewertet, bekommt
Wandkontakte statt Noten.

## Wo konfiguriert wird

| Schicht | Datei / Option | Hinweis |
|---|---|---|
| Vorgaben | `mecanum_lab/types.py` → `DEFAULT_CONFIG` | jede Sensorzahl beider Versuche |
| Standort | `config/default.json` | überschreibt die Vorgaben |
| Auftragsprofil | `config/tasks.json` → `sim` | die gemessene Sensorik je Aufgabe (GPS-Rate, σ, Funkloch, IMU) |
| Kommandozeile | `--set gps.sigma_xy=1.2 --set imu.rate=400 --set gps.gap='[14,8]'` | gewinnt immer |
| Launch-Datei | `ros2 launch launch/kf.launch.py --show-args` | 46 Argumente, alle auf `--set` abgebildet |

Hallen: `arena` (offen, Versuch 2), `production`, `maze`, `track` — oder eine eigene
`worlds/name.txt` (`python3 tools/worldcheck.py --welt name` prüft sie).

## Nützliche Befehle

```bash
./lab docs                # Themen, Aufgaben, Beispiele
./lab robots              # wer fährt gerade?
./lab grade --task kf_gps --controller student/kf_solution.py --json bericht.json
tools/check.sh            # Barriere der Betreuung (Tests, Bewertung, Budgets)
python3 tools/fastgrade.py --task kf_alle --controller student/kf_solution.py --speed 25
```

## Wenn etwas nicht läuft

* **`pygame fehlt`** → `./install.sh`, oder alles mit `--headless` laufen lassen.
* **Kein Display / SSH** → `--headless` (oder `SDL_VIDEODRIVER=dummy`).
* **`ros2: command not found`** → `/opt/ros/<distro>/setup.bash` sourcen; alles außer
  `ros2 …` geht auch ohne.
* **Bewertung meldet „keine Messpaare"** → der Knoten muss `/<robot>/kf/pose` senden, und
  die Sim muss die Wahrheit senden (`--truth`, bei `./lab grade` automatisch).
* **Schätzung hängt hinter den Messungen her** → Wanduhr benutzt. Es zählen die
  Messstempel (`fix.t`, `odom.t`), nie `time.time()`.
* **Zweiter Auftrag startet weit weg** → Messungen des vorigen Auftrags verwerfen
  (`fix.t >= Missionsbeginn`), siehe Arbeitsblatt, „Drei Regeln".
* **Listen** → `./lab docs`.

## Regeln dieses Codebases

Stdlib + pygame (kein numpy/scipy/yaml im Simulator und in den Studierendendateien),
Kommentare und Oberfläche deutsch, Bezeichner englisch, deterministisch aus `--seed`,
headless-fähig. Details: `docs/CONTRACT.md` (Versuch 1), `docs/CONTRACT-KF.md` (Versuch 2).
