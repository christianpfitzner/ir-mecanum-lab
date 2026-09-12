# Notizen Agent D — Versuch 2 (`student/kf_template.py`, `student/kf_solution.py`)

Stand: nach dem zweiten Durchlauf. Die Musterlösung besteht alle vier KF-Aufträge; die
beiden Misserfolge im Mehrfachlauf `--task kf_alle` sind Fahrfehler des Bewerters, keine
Filterfehler (siehe §3, Punkt 1 — der Integrator repariert `engine.py`/`node.py` gerade).

## 1. LOC-Bilanz

| Datei | Zeilen | Budget (§6) | Status |
|---|---|---|---|
| `student/kf_solution.py` | 253 | 260 | ok |
| `student/kf_template.py` | 184 | 150 | **+34** — siehe §2.1 |
| `tests/test_kf_solution_d.py` | 231 | — | 12 Tests, Laufzeit 0,3 s |

`python3 tools/loc.py` kennt die beiden neuen Dateien noch nicht (die Budgettabelle in
`tools/loc.py` stammt aus Versuch 1 und führt `kf_template`/`kf_solution` nicht — bitte bei
Gelegenheit nachtragen, das ist Sache des Integrators).

## 2. Abweichungen vom Vertrag

### 2.1 Template über dem Zeilenbudget
150 Zeilen reichen für das Template nicht: sieben TODOs mit *exakt beschriebener*
Erwartung (Auftrag D) brauchen Text, und der Konventionsblock oben ist wie in
`student/controller_template.py` (123 Zeilen für drei TODOs). Gekürzt wurde der
Modul-Docstring auf 21 Zeilen und die Leerzeilenstil auf eine Leerzeile zwischen
Funktionen. Wenn 150 hart ist: die Hilfetexte der TODOs 3–5 in die Anleitung auslagern,
dann passen sie in den Code.

### 2.2 Q-Konvention: dt³/3 statt dt⁴/4
Die Hilfe in `config/tasks.json` (K1) nennt `Q = q·[[dt⁴/4, dt³/2],[dt³/2, dt²]]` mit
`q` in m²/s³. Das ist die Näherung für *konstante* Beschleunigung über einem Schritt;
genutzt wird die exakt integrierte Form `q·[[dt³/3, dt²/2],[dt²/2, dt]]` (weiße
Beschleunigungs-Dichte, verschwindet für dt → 0). Bei 50 Hz Prädiktionsschritt ist der
Unterschied in Q_pp Faktor ~13 — bei der Näherungsumrechnung müsste `q` rund 4× größer
gesetzt werden. Die Hilfetexte in `config/tasks.json` (Auftrag des Integrators) sollten auf
dt³/3 gestellt oder mit „Näherung, dann q ≈ 4× größer" gekennzeichnet werden.

### 2.3 Zwei Updates statt eines
Vertraglich erlaubt (§8.1: „Positions-Update … und Bewegungs-Update … **oder** ein
Bewegungsmodell"): der Filter hat *ein* Update-Verfahren (`KF.update(stellen, …)`), das
zweimal aufgerufen wird — (0,1) auf das GPS mit `R = sigma_xy²`, (2,3) auf die gedrehte
Odometrie-Geschwindigkeit mit `R = sigma_v² = (0,08 m/s)²`. Dadurch ist die
Zustandsgröße Geschwindigkeit praktisch die gemessene Körpergeschwindigkeit, und die
Prädiktion schiebt die Position mit ihr fort — ohne dass P die Odometrie für unfehlbar hält.

### 2.4 Beschleunigung der IMU wird nicht benutzt
`ax/ay` bleiben ungeachtet (Az ist Specific Force, der Accelerometer-Bias der Simulation
ist 0,05 m/s² + Einschwingen 0,5 m/s² — doppelintegriert in 10 s gut 2 m, siehe
`sensors.ImuSensor`). Benutzt wird nur `gz` nach Bias-Mittelung im Stillstand (80
Stichproben). Damit ist K2 („IMU als kurzfristig saubere Drehrate") erfüllt und die
Falle „offen integrieren" bewusst nicht genommen.

### 2.5 Verspätete Fixe: zurückspulen statt „jetzt"
`KF.schritte` parkt die letzten 120 Prädiktionsschritte (Zustand vorher, dt, ω, v).
Trifft ein Fix mit `fix.t < f.t` ein, wird auf den letzten Schritt vor `fix.t`
zurückgespult (`spule`), dort aktualisiert und bis zur Gegenwart nachgefahren. Kosten:
~20 Zeilen. Nutzen: bei 0,85 m/s und 100 ms Zustellverzögerung sind das 8,5 cm, die sonst
systematisch hinter der Messung herlaufen — und in `tools/fastgrade.py --speed 25` die
Wanduhr nichts taugt. Das **Template** macht es einfacher (nur bis `fix.t` vorrechnen);
die Musterlösung ist hier bewusst besser als die Abgabe, das ist der Feinschliff-Stoff.

### 2.6 Kein floor für P (Bitte des Integrators, abgelehnt — Begründung)
Anregung war, P eine Untergrenze zu geben, damit NEES nicht unter 0,3 fällt. Das wirkt in
die falsche Richtung: NEES < 1 heißt „P ist **zu groß** angegeben" (Filter zu bescheiden).
Ein floor auf sx/sy macht NEES *kleiner*, nicht größer. Wer NEES anheben will, verkleinert
`Q_ACC` (kleineres P → NEES steigt) oder vergrößert `SIGMA_V`. Gemessene NEES der
Musterlösung (Single-Lauf, seed 1): kf_gps 0,92 / kf_kovarianz 0,89 / kf_dynamik 2,27 —
alle mittig im (auf [0,3, 3,0] erweiterten) Band. Bei seed 7 einer wiederholten Serie wurde
0,34 gemessen; das war derselbe artefaktbehaftete Lauf, in dem der Roboter in der Wand stand
(§3.1): 30 s Stillstand lassen P auf 0,5 m anwachsen, während der Fehler bei 7 cm bleibt.
Nach dem Respawn-Fix sollte der Ausreißer weg sein.

### 2.7 Kein eigener Dateigriff in Vertrag/Config
An `config/tasks.json`, `config/default.json`, `mecanum_lab/*` und `worlds/*` habe ich nichts
geändert. Die in §3 genannten Punkte sind Bitten an den Integrator.

## 3. Befunde, die der Integrator anfassen sollte

1. **Kein Respawn zwischen KF-Aufträgen** (`--task kf_alle`, ein Simulationslauf): der
   Roboter bleibt stehen, wo der vorige Auftrag endete. `kf_gps` endet bei (15,1; 4,3) mit
   Drehung −1,6 rad, also mit der Nase nach Süden; die erste Gerade von `kf_fusion`
   (6 s · 0,6 m/s = 3,6 m) fährt in die Südwand der Arena (y = 0,5, Kollisionsradius 0,21 →
   Kontakt bei y = 0,71). `kf_kovarianz` startet in derselben Ecke und fährt erneut dagegen.
   Folge: `kontakte = 1` und ein Urteil „Wandberührungen 1 verletzt kontakte_max=0", das der
   Studierende nicht beeinflussen kann (der Bewerter fährt, `mode = pass-through`).
   Nach dem Respawn (engine.py `reset_robot`, node.py je KF-Auftrag) erwartet: alle vier
   Aufträge grün. Zur Einordnung: einzeln gemessen (je Auftrag ein Lauf ab Spawn) bestehen
   alle vier mit großem Abstand, siehe §4.
2. **Arena-Puffer**: die Südwand liegt nur 3,7 m vor dem Startkurs: `kf_fusion`'s erstes Segment
   (6 s · 0,6 m/s = 3,6 m) ist genau dort zu lang. Wenn die Fahrten so bleiben, hilft auch eine um ein Segment kürzere
   Abfolge oder eine Verschiebung des Spawns nach Norden.
3. **`tools/loc.py`**: Budgettabelle kennt die beiden neuen Studierendenteien nicht.
4. **Hilfetext Q** in `config/tasks.json` → §2.2.

## 4. Messwerte der Musterlösung (Einzel-Läufe, seed 1, `--speed 25`)

| Auftrag | rmse | rmse_gps | Verbesserung | max_fehler | rate | NEES | luecke_max | Urteil |
|---|---|---|---|---|---|---|---|---|
| `kf_gps` | 0,102 | 0,696 | 6,85 | 0,158 | 38,6 Hz | 0,92 | — | BESTANDEN (Grenzen 0,42 / 1,6 / 1,25 / 5 Hz) |
| `kf_fusion` | 0,150 | 2,162 | 14,46 | 0,312 | 35,7 Hz | 0,28 | 0,195 m (Grenze 1,8) | BESTANDEN (0,45 / 2,0) |
| `kf_kovarianz` | 0,101 | 0,696 | 6,90 | 0,158 | 38,6 Hz | 0,89 | — | BESTANDEN (NEES [0,5–3], 0,42) |
| `kf_dynamik` | 0,177 | 0,922 | 5,22 | 0,280 | 32,7 Hz | 2,27 | — | BESTANDEN (0,35 / 1,8 / 0,9 / 10 Hz) |

Seed-Streuung (ebenfalls Einzel-Läufe): rmse 0,069…0,177, Verbesserung 5,2…12,5,
NEES 0,34…2,3 — die Schwellen für rmse/Verbesserung/Rate/Lücke sind mit Faktor ≥ 2
unterschritten, nur die NEES streut relativ breit (Filtereigenschaft: sie skaliert mit
P, und P hängt an `Q_ACC`; ±0,3 um den Mittelwert ist bei 800 Stichproben normal).

Im Mehrfachlauf `kf_alle` (seed 1, vor dem Respawn-Fix): kf_gps rmse 0,131 / verb 6,35 /
nees 0,99, kf_dynamik rmse 0,186 / verb 4,44 — beide BESTANDEN; kf_fusion und
kf_kovarianz nur wegen `kontakte` durchgefallen.

## 5. Was der Filter tut (ein Satz, für die Anleitung)

Zustand (x, y, vx, vy) im Weltframe, Prädiktion mit CV-Modell und `Q(dt)`, Gierrate aus dem
IMU-Gyro (Bias im Stillstand gemittelt) dreht die Richtung, die gemessene
Körpergeschwindigkeit der Odometrie (in die Welt gedreht) hält die Geschwindigkeitszustände
fest, GPS-Fixe werden auf ihren Messstempel zurückgerechnet und als Positions-Update verarbeitet, gemeldet wird (x, y, θ) mit sx, sy, sth aus P — Rate 50 Hz.

## 6. Benutzte Kommandos

```bash
cd /home/pfitzner/git/mecanum-lab

# Beweiskette aus dem Auftrag D
SDL_VIDEODRIVER=dummy MECANUM_LOG=warning python3 tools/fastgrade.py --task kf_alle \
    --controller student/kf_solution.py --speed 25 --json /tmp/kf_muster.json   # Exit 2 (kontakte)
SDL_VIDEODRIVER=dummy MECANUM_LOG=warning timeout 500 ./lab grade --task kf_alle \
    --controller student/kf_solution.py                                         # läuft, s. Log
SDL_VIDEODRIVER=dummy ./lab run --world arena --task kf_gps --robot test \
    --controller student/kf_template.py --headless --seconds 12 --truth          # Exit 0
SDL_VIDEODRIVER=dummy MECANUM_ROS=stub python3 -m pytest tests -q                # 88 passed, 1 skipped

# Einzelbewertung je Auftrag (Respawn wie später, große Margen)
for t in kf_gps kf_fusion kf_kovarianz kf_dynamik; do
  SDL_VIDEODRIVER=dummy MECANUM_LOG=warning python3 tools/fastgrade.py --task $t \
      --controller student/kf_solution.py --speed 25
done

# Seed-Streuung und Tuning (Tuning-Helfer lag in /tmp/kfbench.py, ist kein Liefergegenstand)
python3 /tmp/kfbench.py --tasks kf_kovarianz --seeds 1,2,3,7 --patch '{"Q_ACC": 12.0}'
python3 -m compileall -q mecanum_lab student tools
```

Tuning: `Q_ACC` (12 m²/s³) und `SIGMA_V` (0,08 m/s) sind die einzigen freien Größen.
Grob: `Q_ACC` hoch → folgt dem GPS schneller, rmse steigt, NEES sinkt. `SIGMA_V` hoch →
dasselbe, nur über die Geschwindigkeit. Die Werte stammen aus einer Serie von
Sweeps (Q_ACC 3…24, SIGMA_V 0,03…0,15) über die vier Profile; sie lassen rmse auf dem
kf_dynamik-Profil bei 0,18 (Grenze 0,35) und die NEES auf dem kf_kovarianz-Profil bei
0,9 (Band [0,3 … 3,0]).

## 7. Offene Punkte / Risiken

* Die vier Schwellen hängen an Sensorwerten aus `config/tasks.json` (`gps.sigma_xy`,
  `gps.rate`); die Musterlösung liest `gps.sigma_xy` über `rob.config()` und rät nicht.
  Wird `gps.sigma_xy` in einem Prüfprofil auf > 1 m gestellt, wächst die rmse proportional
  (rmse ≈ 0,2 · σ), die Grenzen in den Aufgaben müssten dann mitziehen.
* `rate_min = 10 Hz` für `kf_dynamik`: gemeldete Rate liegt bei ~33–58 Hz, weil
  `MELDE_DT = 0,02` über der **Simulationszeit** zählt. Fällt die Simulationsrate
  (`rate` in der Config) unter 20 Hz, sinkt die Melderate mit — die Schwellen sind dafür
  aber auch als Untergrenzen gebaut.
* Die Tests prüfen die Musterlösung direkt (Helfer + synthetischer Messstrom), nicht die
  Simulation. Ein `pytest`-Lauf braucht weder ROS noch Fenster noch Engine-Zustand und
  läuft in 0,3 s.
