# CONTRACT.md — Interner Technik-Vertrag (nicht für Studierende)

Dieser File ist die **einzige Wahrheit** über Namensgebung, Schnittstellen und
Datei-Besitz. Alle Module halten sich daran; Abweichungen nur nach Rücksprache mit
dem Integrator (der auch integriert und testet).

Ziel des Projekts: 2D-Mecanum-Simulator in Pygame mit ROS-2-Anbindung für einen
Hochschulversuch. Randbedingungen: **sehr wenig Code**, **sehr wenige
Abhängigkeiten**, **sofort startbar**, von Studierenden lesbar und änderbar.

---

## 1. Grundsätze (harte Anforderungen)

| # | Regel |
|---|-------|
| 1 | **Abhängigkeiten:** Python-Stdlib + `pygame`. `rclpy` ist *optional* (läuft auch ohne ROS). Kein numpy, kein yaml, kein scipy, keine ROS-Message-Generierung als Pflicht. |
| 2 | **Python:** 3.10+, nur Stdlib-Typen (`dataclasses`, `math`, `json`, `random`, `argparse`, `threading`). Keine Typ-Konzerte ohne Wert — Kommentare schlagen Typannotations, wo sie Lesbarkeit kosten. |
| 3 | **Kommentare und UI-Texte Deutsch**, Variablen/Functions Englisch. |
| 4 | **Keine Klasse, die nur weiterreicht.** Wenn eine Funktion < 4 Zeilen hat und einmal benutzt wird, inline. |
| 5 | **LOC-Budgets einhalten** (siehe §7). Am Ende jeder Datei kein leerer Zeilen-Müll, keine Banner-Kommentare. |
| 6 | **Determinismus:** Weltphysik hängt nur von `dt` und Seeds ab, nie von Wandzeit oder Thread-Reihenfolge. |
| 7 | Alles läuft unter `SDL_VIDEODRIVER=dummy` (headless, CI) und ohne ROS. |
| 8 | Kein `print` für Debugging im Simulationspfad — `logging` über `mecanum_lab.log`. |

## 2. Verzeichnis & Datei-Besitz

```
mecanum-lab/
├── lab                       bash, ONLY ENTRY POINT für Studierende   [B]
├── README.md                 Quickstart                               [E]
├── requirements.txt          pygame                                   [B]
├── config/
│   ├── default.json          Sim-Konfiguration                        [MINE]
│   └── tasks.json            Bewertungsschwellen                      [D]
├── worlds/*.txt              3 Umgebungen als ASCII-Grids             [A]
├── mecanum_lab/
│   ├── __init__.py           log-Helper, Version                      [MINE]
│   ├── types.py              Datentypen, Themen, Config               [MINE]
│   ├── stub.py               In-Prozess-Bus (läuft ohne ROS)           [MINE]
│   ├── engine.py             SimEngine: Robotern, Schritt, Sensoren   [MINE]
│   ├── worlds.py             ASCII-Grid -> World                      [A]
│   ├── physics.py            Mecanum-IK/FK, Chassis, Kollision        [A]
│   ├── sensors.py            Odometrie / LIDAR / GPS + Rauschen       [A]
│   ├── render.py             Pygame-Renderer + Tastatur               [C]
│   ├── ros_bridge.py         rclpy-Backend + Stub-Bus + Services      [B]
│   ├── node.py               CLI: sim/run/teleop/spawn/grade/robots   [B]
│   ├── robot_io.py            client für Studiumsknoten  [D]
│   ├── tasks.py              Auftrag-Definitionen       [D]
│   └── grade.py              Bewertungslauf             [D]
├── student/
│   ├── controller_template.py  Abgabe-Template mit TODOs              [D]
│   └── solution.py             Musterlösung                           [D]
├── launch/*.launch.py        ROS-2-Launch                             [B]
├── interfaces/mecanum_lab_interfaces/   eigenes Spawn-Service         [B]
├── tests/                    pytest, keine ROS-Abhängigkeit     [A,B,D]
└── docs/praktikum/anleitung.tex  Anleitung (LaTeX)                    [E]
```

**Regel:** Jede Datei gehört genau einem Agenten. `git`-Konfliktfreiheit schlägt
Eleganz. Wenn Agent X eine Änderung an einer fremden Datei braucht: in
`docs/notes/<agent>.md` — ein File pro Agent, niemals teilen — eintragen und **nicht
selbst editieren**. Dort auch: LOC-Bilanz, Abweichungen vom Vertrag, Testkommandos.

## 3. Themen, Frames, Typen

Namensschema: `/<robot>/<topic>` für Roboterkanten, `/sim/<...>` für die Simulation.
Frames: `map` (Welt), `odom` (Odometrie-Ursprung), `base_link` (Roboter), `laser`.

| Thema | Typ (ROS 2) | Richtung | Inhalt |
|---|---|---|---|
| `/<robot>/cmd_vel` | `geometry_msgs/msg/Twist` | studentischer Knoten → Sim | Körpergeschw. (vx, vy, omega); vy nach **links** positiv |
| `/<robot>/wheel_speeds` | `std_msgs/msg/Float64MultiArray` (4) | Studierende → Sim | Radgeschw. [FL, FR, RL, RR] in rad/s — **der eigentliche Übungsteil** |
| `/<robot>/odom` | `nav_msgs/msg/Odometry` | Sim → alle | Dead-Reckoning aus *gemessenen* Radgeschwindigkeiten, verrauscht |
| `/<robot>/scan` | `sensor_msgs/msg/LaserScan` | Sim → alle | 360 Strahlen, 0…2π, Reichweite `range_max` |
| `/<robot>/gps` | `geometry_msgs/msg/PoseStamped` | Sim → alle | verrauschte globale Position ("UWB/MoCap") |
| `/<robot>/truth` | `geometry_msgs/msg/PoseStamped` | Sim → alle | exakte Pose, nur wenn `debug_truth: true` |
| `/<robot>/mission_state` | `std_msgs/msg/String` | Studierende → Sim/Bewerter | `"idle"`, `"running"`, `"done"`, `"failed:<grund>"` |
| `/sim/robots` | `std_msgs/msg/String` | Sim → alle | JSON: Liste der Roboter inkl. Farbe/Marker/Modus |
| `/sim/task` | `std_msgs/msg/String` | Bewerter → Sim → alle | aktiver Auftrag (`kinematik`, `quadrat`, …) |
| `/sim/spawn_robot` | *Service*, siehe §4 | Student/Betreuer → Sim | Roboter hinzufügen |
| `/sim/despawn_robot` | *Service*, siehe §4 | dito | Roboter entfernen |
| `/sim/reset` | `std_srvs/srv/Trigger` | dito | Welt zurücksetzen |
| `/clock` | `rosgraph_msgs/msg/Clock` | Sim → alle | Simulationszeit (Sekunden, `use_sim_time`) |

`wheel_speeds`-Reihenfolge ist heilig: **Index 0=VL(FL), 1=VR(FR), 2=HL(RL), 3=HR(RR)**.

**Fallback-Modus (Pflicht, sonst kein Schnelleinstieg):** hat ein Roboter *nie*
`wheel_speeds` gesendet und kommt `cmd_vel`, dann übernimmt die Sim die Twist
direkt ("pass-through"). Sobald die erste `wheel_speeds`-Meldung eintrifft, ist der
Roboter für immer im `wheels`-Modus. Modus steht in `/sim/robots` und im HUD.

## 4. Spawn-Service (ROS 2 hat kein gebautes String-Service)

Transport wird zur Laufzeit erkannt, die Aufrufer-Seite (`robot_io`, `tools/lab`)
merkt davon nichts:

1. **Bevorzugt:** eigenes Interface `mecanum_lab_interfaces/srv/SpawnRobot`
   (liegt in `interfaces/`, `colcon build --packages-select mecanum_lab_interfaces`):
   ```
   string name            # eindeutig, [a-z0-9_]{2,24}
   string variant         # "" = automatisch (slow|fast|agile)
   ---
   bool   success
   string message         # Fehlergrund oder "ok"
   uint8  index
   string color           # "red", "blue", ...
   string marker          # "triangle", "square", ...
   float64 x
   float64 y
   float64 theta
   ```
2. **Fallback ohne Interface-Build:** Topic-Handshake
   `/sim/spawn_robot/request` (`std_msgs/String`, JSON `{"name": "...", "variant": ""}`)
   → Antwort auf `/sim/spawn_robot/result` (JSON desselben Formats). Timeout 2 s.
3. **Stub-Modus (kein ROS):** direkter Methodenaufruf `SimEngine.spawn(name, variant)`.

Regeln für Namen: `^[a-z][a-z0-9_]{1,23}$`; Duplikat → `success=false`,
`message="name vergeben"`; Limit `spawn_limit` → `message="Roboterlimit erreicht"`.

## 5. Physik-Konventionen (Agent A und D müssen identisch rechnen)

Rechterhandtes System, x nach vorn, y nach **links**, theta gegen den Uhrzeigersinn
(CCW) positiv. Räder: `FL` (vorne links), `FR` (vorne rechts), `RL`, `RR`.
Mecanum in **X-Anordnung** (Rollachsen zeigen nach vorn-innen), wie bei den
gängigen Bausätzen.

```
a = lx + ly                      # Hebelarm für Gier
w_FL = ( vx - vy - a*omega) / r      # inverse Kinematik
w_FR = ( vx + vy + a*omega) / r
w_RL = ( vx + vy - a*omega) / r
w_RR = ( vx - vy + a*omega) / r
```

Vorwärts Kinematik (Pseudo-Inverse, Schlupf wird ignoriert) ist die exakte Umkehrung:

```
vx    =  r/4 * (w_FL + w_FR + w_RL + w_RR)
vy    =  r/4 * (-w_FL + w_FR + w_RL - w_RR)
omega =  r/(4*a) * (-w_FL + w_FR - w_RL + w_RR)
```

`types.GEOM` liefert `lx, ly, r` — alle Implementierungen verwenden **dasselbe**
Vorzeichen. Unit-Test `tests/test_kinematics.py` prüft `fk(ik(v)) == v` für
Zufallswerte *und* die obigen Vorzeichen explizit (y-Pfeil nach links!).

## 6. Schnittstellen der Modul-Ecken (exakt so implementieren)

### 6.1 `mecanum_lab/types.py` [MINE, bereits geschrieben — nur lesen]
`Pose(x,y,theta)`, `Twist(vx,vy,omega)`, `Rect(x0,y0,x1,y1)`, `World(name,cell,walls,spawns,goal,markings,size)`,
`RobotSpec(name,index,color,variant,pose)`, `Robot(...)`, `Odom`, `Scan`, `Gps`,
`PALETTE`, `MARKERS`, `CONFIG` (dict), `load_config()`, `topic(kind, robot=None)`,
`sanitize_name()`, `log`.

### 6.2 `mecanum_lab/worlds.py` [A]
```python
def load_world(name: str, path: str | None = None) -> World      # cached; pfad: worlds/<name>.txt
def parse_grid(text: str, cell: float = 0.5, name: str = "?") -> World
def list_worlds() -> list[str]
```
ASCII-Grid: Zeichenblock `cell` Meter groß.
`#` Wand, `.`/` ` frei, `S` Startpose 1, `2`..`6` weitere Startposes, `G` Ziel,
`-`/`|` Markierungslinie (nur sichtbare Deko, keine Kollision).
Robotermitte liegt in der Zellmitte; theta aus Reihenfolge: 1=`0°`, 2=`90°`, 3=`180°`, 4=`270°`, dann wieder 0°.
Benachbarte Wandzellen zu großen Rechtecken verschmelzen (2-Pass, reduziert Strahltests).

### 6.3 `mecanum_lab/physics.py` [A]
```python
@dataclass
class Geometry: lx, ly, r, max_speed, max_accel, tau, footprint_r
def make_geometry(cfg: dict, variant: str = "stock") -> Geometry   # variant: stock|slow|fast|agile
def inverse_kinematics(g, vx, vy, omega) -> list[float]            # [FL,FR,RL,RR] rad/s
def forward_kinematics(g, w) -> tuple[float, float, float]         # vx, vy, omega
class Chassis:
    def __init__(self, g: Geometry, pose: Pose, seed: int | None = None)
    wheels: list[float]        # gemessene Ist-Radgeschwindigkeit rad/s
    pose: Pose                 # exakte (Wahrheit-)Pose
    twist: Twist               # Körpergeschwindigkeit aus Rädern
    contacts: int              # Anzahl Wandberührungen (steigend)
    def set_wheels(self, w: Sequence[float]) -> None    # begrenzt auf max_speed
    def step(self, dt: float, walls: list[Rect]) -> None  # Motor-Trägheit, FK, Integrate, Kollision
```
Motor: Verzögerung 1. Ordnung mit `tau` und Drehbeschleunigungs-Begrenzung
`max_accel`. Kollision: Kreis `footprint_r` gegen Rechtecke, Position wird längs der
kleinsten Eindringtiefe herausgeschoben, betroffene Radgeschwindigkeit auf 0
gesetzt, `contacts += 1` (nur bei neuem Kontakt). Kein Schlupf-Modell — dokumentieren.

### 6.4 `mecanum_lab/sensors.py` [A]
```python
class Noise:  __init__(self, seed); gauss(sigma); uniform(a)
class OdometrySensor: __init__(self, g, noise, cfg); reset(pose); update(self, wheels, dt) -> Odom
class Lidar: __init__(self, world, noise, cfg); scan(self, pose) -> Scan
class GpsSensor: __init__(self, noise, cfg); fix(self, pose) -> Gps
```
**Namensregel (Integrator, nach Kollisionsfund):** Sensor-Klassen heissen
`OdometrySensor`, `Lidar`, `GpsSensor`; `types.Odom/Scan/Gps` sind die Nachrichten. Ein
Modul darf nicht beide Namen führen — `sensors.Gps` hätte den Import von `types.Gps`
verdeckt und lieferte `TypeError: unexpected keyword 't'`.
Odometrie **rechnet nur mit den übergebenen Radgeschwindigkeiten** (Ist-Werte,
inkl. Rauschen), nicht mit der Wahrheitss-Pose → Drift entsteht naturally und ist
Absicht von Versuch 1. LIDAR: Strahl/AABB-Slab-Test, Trefferkante auf `range_max`
begrenzt, kein Treffer = `inf` → in `LaserScan.ranges` als `range_max` (und
`inf` nur intern). Treffer gegen andere Roboter: nein (Versuch 1).

### 6.5 `mecanum_lab/engine.py` [MINE, bereits geschrieben — nur lesen]
```python
SimEngine(world, cfg=None, seed=None)
  .spawn(name, variant="", pose=None) -> Robot        # raises SpawnError
  .despawn(name) -> bool ; .reset() ; .set_task(name) ; .task
  .set_cmd_vel(name, Twist) ; .set_wheel_speeds(name, list4) ; .set_mission(name, str)
  .step(dt) -> None      # zerlegt in feste Physikschritte, Sensor-Raten intern
  .drain() -> list[(kind, robot|None, payload)]        # von der Bridge abzuarbeiten
  .robots: dict[str, Robot] ; .world ; .t ; .robots_info() ; .world_json()
```
Der Engine ruft Physik/Sensoren **genau so** auf (Agent A muss das treffen):
`physics.make_geometry(cfg["robot"], variant)`,
`physics.Chassis(geom, pose, seed=…)` mit **öffentlichen Attributen**
`geom, pose, wheels, twist, contacts` sowie `set_wheels(list)`, `step(dt, walls)`;
`sensors.Noise(seed)`, `sensors.OdometrySensor(geom, noise, cfg["odom"])` mit
`reset(pose)` und `update(wheels, dt) -> Odom` (wird *jeden* Physikschritt aufgerufen),
`sensors.Lidar(world, noise, cfg["lidar"]).scan(pose) -> Scan`,
`sensors.Gps(noise, cfg["gps"]).fix(pose) -> Gps`.


### 6.6 `mecanum_lab/render.py` [C]
```python
class Renderer:
    def __init__(self, engine, cfg: dict)
    def draw(self) -> None                 # ein Frame, inkl. HUD
    def poll(self) -> dict                 # gedrückte Tasten/Events, s.u.
    def close(self) -> None
    @property
    def ok(self) -> bool                   # False wenn Fenster geschlossen
```
`poll()` liefert u. a. `{"quit": bool, "pause": bool, "toggle_lidar": bool,
"toggle_trail": bool, "camera": int|None, "key": str}`. Tastatur-Teleop wird **nicht**
in render.py implementiert (Aufgabe von node.py, damit renderer ROS-frei bleibt).
Farben kommen aus `Robot.spec.color` (0..1-Floating-RGB), Marker aus `spec.marker`.
Der Renderer liest nur `engine.robots[*].{pose,scan,odom,wheels,...}` und `engine.world`.

### 6.7 `mecanum_lab/ros_bridge.py` [B]
```python
class Bus:                       # gemeinsame Oberfläche für rclpy und stub (stub.py [MINE])
    def pub(self, kind: str, robot: str | None = None) -> Callable[[payload], None]
    def publish(self, topic_name: str, payload) -> None
    def sub(self, kind: str, robot: str | None, cb) -> None       # cb(payload)
    def sub_topic(self, topic_name: str, cb) -> None              # für JSON-/Debug-Themen
    def last(self, kind, robot=None) -> tuple[payload | None, float]   # Wert + Alter in s
    def service(self, name: str, handler) -> None                 # handler(dict) -> dict
    def call(self, name: str, req: dict, timeout: float = 2.0) -> dict
    def spin(self, timeout: float = 0.01) -> None
    def ok(self) -> bool
def make_bus(kind="auto", node_name="mecanum_sim") -> Bus        # auto: rclpy -> stub.py
```
`stub.StubBus` ist vorgegeben — `ros_bridge.RclpyBus` muss diese surface **wortgleich**
bieten, sonst ist `robot_io` nicht portabel. `kind`-Werte sind die Schlüssel von
`types.MSG_SPECS`. `payload` ist *immer* ein Datenobjekt aus `types.py` (`Twist`,
`Scan`, `Odom`, `Gps`, `str`, `list[float]`, `Pose` für `truth`); die Konvertierung in
echte ROS-Meldungen passiert **nur** in `ros_bridge.py`. **Keinen** zweiten Stub-Bus in
ros_bridge bauen — `from . import stub` verwenden.

### 6.8 `mecanum_lab/robot_io.py` [D]
```python
class RobotIO:
    def __init__(self, name: str, role="controller")     # verbindet mit Bus (rclpy oder stub)
    def spin(self, timeout=0.01) ; def running(self) -> bool ; def sleep(self, sec)
    def cmd_vel(self) -> Twist          # letzte Twist, sonst None
    def odom(self) -> Odom | None ; def gps(self) -> Gps | None ; def scan(self) -> Scan | None
    def age(self, kind) -> float        # Alter der letzten Messung in s (für Watchdogs)
    def send_wheels(self, w: Sequence[float]) -> None
    def publish_cmd_vel(self, vx, vy, omega) -> None
    def set_mission_state(self, s: str) -> None
    def task(self) -> str
    def spawn(self, name, variant="") -> dict ; def robots(self) -> list[dict]
def run_loop(node: RobotIO, on_tick, hz: float = 50) -> None
```

### 6.9 CLI `python -m mecanum_lab.node <befehl>` [B]
```
run     Sim + ein/mehrere Controller in EINEM Prozess (stub, kein ROS)  -> Schnelleinstieg
        --world maze --robot alice --controller student/controller_template.py [--headless]
sim     Sim solo (rclpy falls ROS vorhanden, sonst stub + Meldung)
        --world maze --gui --robots alice,bob --task kinematik
controller  nur ein Studiumsknoten (setzt ROS oder laufende stub-Bus voraus) --robot alice
teleop  Tastatur-/Twist-Fernsteuerung auf cmd_vel          --robot alice
spawn   --name alice ; despawn --name alice ; robots ; reset ; task --name quadrat
grade   Bewertungslauf  --robot alice [--task kinematik|quadrat|korridor|alle] [--json pfad]
docs    zeigt Themen aller Roboter
```
`./lab <befehl>` (bash) = `python3 -m mecanum_lab.node <befehl>` mit korrektem
`PYTHONPATH` und `source /opt/ros/$ROS_DISTRO/setup.bash`, falls vorhanden.

## 7. LOC-Budgets (Ziel, keine K.-o.-Grenze — Abweichung > 25 % begründen)

| Modul | LOC |
|---|---|
| types.py | 190 |
| stub.py | 100 |
| engine.py | 170 |
| worlds.py | 90 |
| physics.py | 130 |
| sensors.py | 150 |
| render.py | 170 |
| ros_bridge.py | 230 |
| node.py | 170 |
| robot_io.py + tasks.py + grade.py | 120 / 110 / 160 |
| student/controller_template.py | 90 |
| student/solution.py | 200 |
| **Simulator gesamt (ohne Docs/Tests)** | **< 1400** |

Schon geschrieben: types.py 245, stub.py 105, engine.py 165 LOC. Budgets sind
Richtwerte, der Gesamtrahmen ist die Grenze.

## 8. Bewertaufträge (Versuch 1) — Details in `config/tasks.json` [D]

| ID | Name | Prüft | Erfolgskriterium (Default) |
|---|---|---|---|
| `kinematik` | T1 | IK-Vorzeichen/-Zuordnung | 3 Phasen je 3 s (vx=0.3 / vy=0.3 / ω=0.6): Δx>+0.35, \|Δy\|<0.12, \|Δθ\|<0.18 rad usw. |
| `quadrat` | T2 | Regelungskreis + Odometrie | 1-m-Seiten, 90°-Kurven, wieder < 0.20 m / 15° vom Start, Zeit < 90 s |
| `korridor` | T3 | LIDAR-Einblick | Ziel ohne Wandberührung (`contacts == 0`) erreichen, seitl. Abstand 0.25–0.8 m |
| `gps_anfahrt` | T4 (Bonus) | GPS statt Odometrie | Ziel aus `world.goal` mit GPS-Rückführung, \|Fehler\| < 0.25 m |

Bewertung läuft **über die Themen**, nie durch Code-Analyse: der Studierende kann
implementieren wie er will, gemessen wird Verhalten. Reihenfolge der Prüfung in T1 ist
vx, vy, omega; der Bewerter setzt dazu `/sim/task = kinematik`, damit `mission()`
nicht gegeneinander arbeitet.

## 9. Qualitätsbarriere (vor der Abgabe selbst laufen lassen)

```bash
cd mecanum-lab
python3 -m pytest tests -q                     # muss grün sein, ohne ROS, ohne Fenster
SDL_VIDEODRIVER=dummy ./lab run --robot test --controller student/solution.py --headless
./lab grade --robot test --task alle           # Musterlösung besteht alle
```
Testbarkeit ist Teil der Aufgabe: jede Agentin liefert ihre Tests in `tests/` mit,
Dateiname `test_<modul>_<agent>.py`, damit nichts überschrieben wird.
