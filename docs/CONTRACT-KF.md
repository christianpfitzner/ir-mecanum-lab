# CONTRACT-KF.md — Technik-Vertrag Versuch 2 „Zustandsschätzung“ (nicht für Studierende)

Ergänzt `docs/CONTRACT.md` (Versuch 1) und **widerspricht ihm nicht**: alles Bestehende
bleibt, Versuch 2 setzt darauf auf. Wer eine Datei ändert, die auch Versuch 1 nutzt,
hält die Abwärtskompatibilität — `./lab grade --task alle` (Versuch 1) muss unverändert
grün bleiben.

---

## 1. Lernziel und Prüfidee

| | Versuch 1 | Versuch 2 |
|---|---|---|
| Gegenstand | Kinematik + Regelung | Zustandsschätzung (Kalman-Filter) |
| Stellpfad | Studierende → `wheel_speeds` | **Bewerter** → `cmd_vel` (pass-through) |
| Messpfad | Odometrie/LIDAR/GPS → Studierende | Sensoren → Filter → `kf/pose` → Bewerter |
| Urteil | Verhalten (angekommen?) | **Genauigkeit** (RMSE gegen `truth`) |

Der Bewerter fährt den Roboter über eine in `config/tasks.json` definierte
Kommandofahrt, zeichnet gleichzeitig `truth` (exakt) und die studentische Schätzung
`kf/pose` auf und urteilt über RMSE, Verbesserungsverhältnis gegenüber dem Rohsensor,
Maximalfehler in einer GPS-Lücke und Konsistenz der angegebenen Standardabweichung
(NEES). Der Studierende steuert **nicht**.

Damit das funktioniert, bleibt der Roboter im `pass-through`-Modus: der Knoten für
Versuch 2 sendet niemals `wheel_speeds` und definiert kein `inverse_kinematics`.

## 2. Neue Themen

| Thema | Typ | Richtung | Inhalt |
|---|---|---|---|
| `/<robot>/imu` | `sensor_msgs/msg/Imu` | Sim → alle | 100 Hz (konfigurierbar), Achsen im Körperframe, `linear_acceleration.z ≈ +g` |
| `/<robot>/kf/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Studierende → Sim/Bewerter | eigene Schätzung inkl. 1σ (Diagonale) |
| `/<robot>/kf/info` | `std_msgs/msg/String` | Studierende → alle | JSON-Diagnose (Laufzeit, Raten, Q/R) — Auswerter ignoriert sie |
| `/<robot>/truth` | `geometry_msgs/msg/PoseStamped` | Sim → alle | wie Versuch 1, jetzt mit einstellbarer Rate (`truth.rate`), nur mit `debug_truth` |
| `/sim/config` | `std_msgs/msg/String` | Sim → alle | JSON der Sensor-Konfiguration (Bewerter prüft das Prüfprofil) |

`types.MSG_SPECS` trägt die neuen Schlüssel `imu`, `kf`, `kfinfo`; `topic("kf","alice")`
→ `/alice/kf/pose`.

## 3. IMU-Modell (`sensors.ImuSensor`) — Absicht und Werte

Realistisches MEMS-Modell, bewusst mit den Effekten, die in der Praxis wehtun:

1. **Weißes Rauschen** als *Dichte* (`*_noise` in Einheit/√Hz) → Streuung pro Stichprobe
   `dichte · √(rate/2)`.
2. **Bias** — fester Startwert pro Roboter und Achse (Ziehung aus `Noise`, also seed-fest),
   dazu **Bias-Random-Walk** (`*_bias_walk` in Einheit/√s). Der Bias dominiert alles:
   ein Beschleunigungs-Bias von 0,05 m/s² ergibt nach 10 s Doppelintegration 2,5 m.
3. **Skalenfehler** (`*_scale`, relativ, je Roboter fest) — mit Rauschen nicht wegzuwerten.
4. **Neigung + Vibration**: ein kleiner OU-getriebener Roll/Pitch (σ `tilt_sigma`,
   Korrelationszeit `tilt_tau`) lässt die Schwerkraft in dieHorizontalachsen sickern
   (`g·sin φ`), plus Fahrwerksschwingung (`vibration` bei `vibration_hz`).
5. **Einschwingen**: die ersten `startup` Sekunden laufen mit `startup_bias` Versatz —
   wie ein IMU-Treiber, der sich erst einschießt.

`az` enthält die Specific Force (+g, wenn flach), `gx/gy` sind durch die Neigung
mitbedingt — exakt wie bei einer echten 6-DOF-IMU.

**Determinismus:** alle Zufallszüge pro Roboter aus dem einen `Noise`-Strom der Engine.

## 4. Konfiguration (Versuch 2)

Neu in `DEFAULT_CONFIG` (`types.py`), überschreibbar wie gehabt über
`config/default.json` → `--config datei.json` → `--set section.schlüssel=wert`:

```
truth:  {rate: 20.0}
imu:    {rate, gyro_noise, gyro_bias, gyro_bias_walk, gyro_scale,
         accel_noise, accel_bias, accel_bias_walk, accel_scale,
         tilt_sigma, tilt_tau, vibration, vibration_hz, gravity,
         startup, startup_bias}
gps:    {rate, sigma_xy, sigma_theta, bias_xy, gap: [t0, dauer], bias_step: [t0, dauer, dx, dy]}
kf:     {rate, q_acc, q_turn, gps_delay}    # reine Empfehlung an die Studierenden,
                                            # die Simulation benutzt diesen Block nicht
```

`--set` (neu in `node.py`) nimmt `pfad.unter.pfad=Wert`, Wert als JSON geparst
(Zahl/Bool/String/Liste). Damit ist **alles** über das Launch-File erreichbar, ohne
JSON-Dateien zu schreiben.

**Prüfprofil:** jeder KF-Auftrag in `config/tasks.json` trägt einen `"sim"`-Block mit dem
Sensorprofil, gegen das bewertet wird. `mach_engine()` (node.py) merge
`DEFAULT < config/default.json <- --config <- tasksim <- --set`. So ist eine Bewertung
unabhängig davon, was gerade in `config/default.json` steht.

## 5. Fahrplan für Versuch 2 (`config/tasks.json`, `"versuch": 2`)

| ID | P | Kern | Profil | Schwellen |
|---|---|---|---|---|
| `kf_gps` | 30 | KF mit CV-Modell, nur GPS | σ=0,5 m, 5 Hz | RMSE ≤ 0,42 m, Verbesserung ≥ 1,6, Maxfehler ≤ 1,25 m, Rate ≥ 5 Hz |
| `kf_fusion` | 30 | GPS + Odometrie + IMU, GPS-Lücke | σ=0,8 m, 1 Hz, Lücke 15…23 s nach Auftragsbeginn, Gyro-Bias 0,002 rad/s | RMSE ≤ 0,80 m, Verbesserung ≥ 2,5, Lückenfehler ≤ 1,8 m |
| `kf_kovarianz` | 20 | konsistente 1σ (NEES) | σ=0,5 m, 5 Hz | mittl. NEES in [0,15 … 3,5], RMSE ≤ 0,42 m |
| `kf_dynamik` | 10 | schnell + treuer Folgefehler | σ=0,6 m, 5 Hz, IMU 200 Hz | RMSE ≤ 0,35 m, Verbesserung ≥ 1,8, Maxfehler ≤ 0,9 m, Rate ≥ 10 Hz |

`--task kf_alle` wählt alle Aufträge mit `"versuch": 2` (`tasks.resolve` kann das,
ebenso `kf_gps,kf_fusion`). Welt für alle: `arena` (freie Fläche, Umfangswände).

Bewertungsformeln (`grade.py`), gemessen nur nach `einlauf` Sekunden:

```
rmse(schätzung)   = sqrt(mean(dx²+dy²))            gegen truth, Next-Neighbor ≤ 0,25 s
verbesserung      = rmse(gps) / rmse(kf)
nees              = mean((dx²/sx² + dy²/sy²) / 2)   # 2 Freiheitsgrade, erwarteter Wert 1
luecke            = max Fehler im Zeitfenster der GPS-Lücke
```

Zwei Randbedingungen, die der Versuch 2 mit sich bringt (beide in `engine.py`/`node.py`):

* **Der Bewerter fährt blind.** Bei einem Auftrag mit `"art": "kf"` wird der Roboter vor dem
  Start an der Spawn-Pose abgesetzt (`Engine.reset_robot`) — die Kommandofolge in `fahrt` wird
  nicht zurückgemeldet, also muss sie dort beginnen, wo die Welt den Roboter abstellt. Die
  Simulationszeit läuft dabei weiter; `gps.gap` bleibt **auftragsrelativ** (`set_task`).
  `--world` darf dann ausgelassen werden: KF-Aufträge holen sich ihre Halle (`arena`) selbst.
* **Funkloch und Sensoruhren laufen über Messstempel, nicht über die Wanduhr.** Odometrie und
  IMU zählen ab ihrer Entstehung; die Engine hängt sie mit `_zeitbezug()` an die
  Simulationszeit, damit ihr Stempel nach Respawn/Profilwechsel nicht bei 0 startet, während
  das GPS weiter Simulationszeit meldet. `tools/fastgrade.py` rechnet 25 Simulationssekunden
  pro Sekunde — ein wanduhr-getakteter Test sähe weder das Funkloch noch einen falschen `dt`.

**Warum K2 relativ und K3 mit lockerer unterer Grenze prüft** (Kalibrierung über Seeds 1–4,
Musterlösung): bei 1 Hz GPS mit σ = 0,8 m ist der *absolute* Fehler rauschbegrenzt — dieselbe
Implementierung misst je Rauschrealisierung 0,27…0,60 m RMSE. Eine absolute Grenze von 0,45 m
würde damit zur Lotterie, also prüft `verbesserung_min` (stabil: 3,4…6,2) und die
Lückengrenze. Beim NEES streut dieselbe Lösung zwischen 0,27 und 1,04; die untere Grenze bei
0,15 bestraft nur deutlich überhöhende Kovarianz, die obere (3,5) jeden übermüteten Filter.

## 6. Datei-Besitz Versuch 2

```
mecanum_lab/types.py        Imu, Kf, MSG_SPECS, Config-Blöcke            [Integrator]
mecanum_lab/sensors.py      ImuSensor, GpsSensor (gap/bias_step)         [Integrator]
mecanum_lab/engine.py       IMU-Takt, truth-Rate, set_kf, /sim/config    [Integrator]
mecanum_lab/ros_bridge.py   Imu- und PoseWithCovarianceStamped-Konvert.   [Integrator]
mecanum_lab/robot_io.py     imu() truth() kf() send_kf()                  [Integrator]
mecanum_lab/tasks.py        Versuchs-Auswahl, sim-profil()                [Integrator]
mecanum_lab/grade.py        KF-Art „kf_fahrt“ + Messung                   [Integrator]
mecanum_lab/node.py         --set, --truth, --log, Profil-Merge, tap      [Integrator]
mecanum_lab/render.py       Überlagerung truth/gps/kf + Kovarianzellipse  [Integrator]
config/tasks.json           vier KF-Aufträge                             [Integrator]
worlds/arena.txt            offene Fläche                                [Integrator]
launch/kf.launch.py         Hauptlaunch: alles als Argument              [Integrator]
student/kf_template.py      Abgabe-Template mit TODOs                     [D]
student/kf_solution.py      Musterlösung (besteht kf_alle)                [D]
docs/praktikum/kalman.tex   Arbeitsblatt Versuch 2                        [E]
tools/kfplot.py             CSV -> matplotlib oder ASCII-Diagramm         [F]
install.sh, README.md       Ein-Kommando-Installation, Schnellstart       [F]
rviz/kf.rviz                Ansicht für odom + gps + kf/pose              [F]
docs/praktikum/kalman.tex   Arbeitsblatt Versuch 2                        [E]
install.sh + README.md      Drei-Zeilen-Installation                      [E]
tests/test_sensors_imu_b.py Rauschmodell, Determinismus, Lücke            [A]
tests/test_kf_grade_e.py    Bewertung mit eingebautem Ideal-Filter        [D]
```

Budgets (Neuzugänge, `tools/loc.py`): `sensors.py` 260, `grade.py` 300, `node.py` 240,
`types.py` 290, `engine.py` 210, `ros_bridge.py` 290, `robot_io.py` 215, `render.py` 230,
`launch/kf.launch.py` 130, `student/kf_template.py` 150, `student/kf_solution.py` 260,
`tools/kfplot.py` 120.

## 7. Randbedingungen (gelten weiter)

Stdlib + pygame, `rclpy` optional, alles headless und ohne ROS lauffähig,
Determinismus über Seed, Deutsch als Kommentar-/Anleitungssprache, kein `print` im
Simulationspfad. Die Musterlösung darf **kein** numpy benutzen — sie ist Lese-Vorbild
für den Abgabe-Code der Studierenden.

## 8. Arbeitsanweisung für die Agenten (Versuch 2)

### 8.1 Was der Studierenden-Knoten tun darf und muss

Der Knoten läuft im Runner `robot_io.serve(modul)`; die Simulation **fährt** (pass-through
über `cmd_vel` vom Bewerter). Deshalb: **keine** `inverse_kinematics` definieren und **nie**
`send_wheels()`/`publish_cmd_vel()` aufrufen — sonst konkurriert der Knoten mit dem Bewerter.

`mission(rob, task)` wird vom Runner genau einmal pro Auftrag aufgerufen und soll loop en,
bis sich der Auftrag ändert:

```python
while rob.running() and rob.task() == task:
    rob.spin(0.005)
    ...
```

Der Auftrag endet für den *Bewerter* durch Ablauf der Kommandosequenz, nicht durch `done`.
`mission()` darf also bis zum Task-Wechsel laufen; der Runner meldet danach `done`.

**Zeit ist Simulationszeit.** `mess.t` (Odom, Gps, Scan, Imu, Kf, truth) ist Sekundenzeit der
Simulation; der Runner stempelt jede outgoing `Kf`-Meldung mit der aktuellen Simulationszeit.
Kein `time.time()`, kein `time.sleep()` als Taktgeber — `tools/fastgrade.py` entkoppelt
Simulations- und Wandzeit, und ein wanduhr-getakteter Filter läuft dann in die falsche Richtung.
Der Filter taktet über die Stempel der hereinkommenden Messungen.

**Neuer Auftrag = neue Messungen.** Bei KF-Aufträgen setzt die Simulation den Roboter je
Auftrag an der Spawn-Pose ab (`Engine.reset_robot`), aber sie zieht eine längst
veröffentlichte Messung nicht zurück: Der Bus hält die letzte Meldung bereit, und die gehört
zum *vorigen* Auftrag. Ein Filter, der damit startet, steht Sekunden lang meterweit daneben.
Deshalb gehört in jede Lösung ein Missionsbeginn (`KF.start`, gesetzt beim ersten Odometrie-
Stempel des Auftrags) und die Rückfrage `fix.t >= f.start`. Das Arbeitsblatt nennt es unter
„Drei Regeln, die den Lauf retten" — es ist der Fall, der in der Betreuung die meisten Punkte
kostet.

### 8.2 API (nur diese Methoden benutzen)

| Aufruf | liefert |
|---|---|
| `rob.gps()`, `rob.odom()`, `rob.imu()`, `rob.scan()` | letzte Messung oder None (Dataclass aus `types.py`) |
| `rob.age("gps")` | Alter der letzten Messung in s (1e9 = nie) |
| `rob.sensor_profil()` | dict der aktiven Sensorik (`gps`, `imu`, `odom`, `truth`, `rate`, `debug_truth`, `seed`) |
| `rob.config("gps.sigma_xy", 0.5)` | einzelner Sensorik-Wert, ohne Dict-Gequickle |
| `rob.task()` | aktiver Auftragstitel, z. B. `kf_fusion` |
| `rob.send_kf(x, y, theta, sx, sy, sth, info=None)` | Schätzung inkl. 1σ auf `/<robot>/kf/pose` |
| `rob.truth()` | exakte Pose — **nur** Selbstkontrolle, nie im Filter |
| `rob.running()`, `rob.spin(dt)`, `rob.sleep(s)` | Lebenszyklus |

`types.Imu`: `t, ax, ay, az, gx, gy, gz, roll, pitch` (`az` ≈ +g im Stand).
`types.Odom`: `t, x, y, theta, vx, vy, omega`. `types.Gps`: `t, x, y, theta`.

### 8.3 Bewertung (grade.py, pro Auftrag in `config/tasks.json`)

Nach `einlauf` Sekunden werden bei jedem Takt die letzte Wahrheit, die letzte Schätzung und
der letzte Rohsensor gemessen; gemeldet werden `rmse`, `rmse_<sensor>`, `verbesserung`,
`max_fehler`, `rate_hz`, `nees`, `luecke_max`/`luecke_dauer`. Bestanden = alle Schwellen
erfüllt. Die Schwellen stehen in der JSON — **nicht** im Code nachjustieren, sondern die
Musterlösung besser machen.

Gültig für die Musterlösung: `python3 tools/fastgrade.py --task kf_alle
--controller student/kf_solution.py --speed 20` muss Exit 0 liefern, und
`./lab grade --task kf_alle --controller student/kf_solution.py` ebenfalls.

### 8.4 Randbedingungen für alle Dateien

Stdlib + pygame, **kein numpy** (die Musterlösung ist Lesevorbild), Deutsch als
Kommentarsprache, keine Banner-Kommentare, LOC-Budgets aus §6 einhalten. Tests heißen
`tests/test_<thema>_<agent>.py` und laufen ohne ROS, ohne Fenster, ohne Netzwerk
(`SDL_VIDEODRIVER=dummy MECANUM_ROS=stub python3 -m pytest tests -q`).
