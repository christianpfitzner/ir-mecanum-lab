# Agent G — Umgebungsdaten + Welten-Prüfer

## Abnahme (Ist-Zustand, wiederholbar)

    $ python3 tools/worldcheck.py --frei 0.3 --startseite 1.0        # Exit 0
      ok   maze: 12.5 x 9.5 m, 20 Rechtecke, 4 Starts, Ziel ja
             breitester Weg Start->Ziel: engste Stelle 0.75 m frei (nötig 0.51 m)
      ok   production: 20.0 x 12.0 m, 10 Rechtecke, 4 Starts, Ziel ja
             breitester Weg Start->Ziel: engste Stelle 1.25 m frei (nötig 0.51 m)
      ok   track: 18.0 x 11.0 m, 5 Rechtecke, 4 Starts, Ziel ja
             breitester Weg Start->Ziel: engste Stelle 0.75 m frei (nötig 0.51 m)
      0 Welten mit Problemen
    $ SDL_VIDEODRIVER=dummy python3 -m pytest tests -q          -> 52 passed, 1 skipped
    $ ./lab sim --world {maze,track,production} --robots a1,b2 --headless --seconds 2
                                                               -> keine Meldung
    load_world: maze 20 / track 5 / production 10 Wandrechtecke, je 4 Starts (Orientierung
                0, 90, 180, 270 Grad), Ziele maze (10.75, 7.75), track (9.25, 9.75),
                production (15.25, 6.75)

## Geändert

* `worlds/maze.txt`, `worlds/track.txt`, `worlds/production.txt` — neu gebaut (Generator,
  Freiräume als Rechtecke, Korridorbreite ist eine Zahl im Code):
  maze: 4x3 Räume à 5 Zellen, 1 Zelle Wände, 3-Zellen-Tore mit Zufallsversatz -> Stummel
  und Sackgassen, Anlieferhalle unten links, Zielraum oben rechts.
  track: geschlossener Ring, Fahrbahn 4 Zellen (2,0 m), Geraden 5 Zellen, untere
  Zielgerade 7 Zeilen hoch als Lade-/Startbereich, Infield, `|` Fahrbahnrand, `-` Startlinie.
  production: Halle mit Anlieferzone (13 x 7 Zellen), drei Tischreihen oben, zwei
  Regalblöcken, Ladesäule unten rechts, Ziel in der mittleren Fahrstraße.
  Vorher: maze-Korridore teils 0,5 m, track-Start 0,25 m an der Wand, production-Inseln
  4 cm in der T2-Quadratbahn (das war die Ursache der fünf Wandberührungen in T2).
* `tools/worldcheck.py` — jetzt 182 Zeilen. Neu: Option `--startseite <m>` (Standard 1,0).

## Zwei Korrekturen am Prüfer, ohne die die Acceptance nicht ehrlich war

1. „engste Stelle" maß auf dem **kürzesten** Zellweg; dieser knickt an Wänden entlang und
   jede Zelle an einer Wand hat per Definition 0,25 m Freiheit — das Labyrinth flog mit
   0,35 m raus, obwohl seine Korridore 1,5 m breit sind. Jetzt **breitester Weg**
   (maximin über die Zellfreiheit, Dijkstra mit `heapq`). Reichweite-BFS, Meldungstexte,
   Format sonst unverändert.
2. `abstand()` clamppte den Punkt nicht ins Rechteck (`max(min(x, x0), x0)` ist immer
   `x0`) und maß damit überall Quatsch. Jetzt korrekt, und zwar über die verschmolzenen
   `world.walls` (schneller und an Ecken exakt).

## Kriterium: Freiheit nach allen Seiten, nicht nur in +x/+y

`--startseite` prüft zuerst nur das Quadrat in +x/+y. Das war ein falsches Grün: die
Startorientierung wechselt je Teilnehmer (0/90/180/270 Grad, Regel in `worlds.py`), T2
fährt sein Quadrat aber in Startrichtung — für die Teilnehmer 2..4 hätte es in eine Wand
gezeigt. Geprüft wird jetzt die Hülle über alle vier Richtungen, also **1,26 m Freiheit
in jede Himmelsrichtung um jeden Startplatz** (Startseite 1,0 + Fußabdruck 0,21 + 0,05
Spiel). Das ist der Grund, warum die Zonen 7 Zeilen (3,5 m) hoch sein müssen.

## Empfehlung an den Integrator (wurde nicht beauftragt, deshalb nicht gebaut)

Das Labyrinth ist dadurch eine Halle mit Regalgassen geworden: **T2 diktiert über
`--startseite` die Geometrie aller drei Welten, obwohl T2 laut `config/tasks.json` nur in
`production` läuft.** zwei saubere Wege:

* `--startseite` nur für die Welten verlangen, in denen ein Quadrat-Auftrag läuft
  (z. B. Option `--startseite-nur production`), dann darf `maze` wieder 2-Zellen-Korridore
  und damit echten Irrgarten-Charakter haben; oder
* T2 auf eine vierte, flache Welt „kalibrierung" legen und `maze`/`track` eng bauen.

Zweite Erkenntnis für die Anleitung: bei 0,5 m Zellen, 0,42 m Roboter und 0,51 m
verlangter Freiheit ist ein enger Irrgarten geometrisch unmöglich — Korridore brauchen
3 Zellen, und ein 1-Zellen-Tor hat mittig nur 0,25 m, also sind auch Tore 3 Zellen breit.
Echtes Mäander bräuchte `"worlds": {"cell": 0.25}` (doppelte Auflösung, selber Prüfer).
Nebeneffekt: `--frei 0.3` schließt 2-Zellen-Korridore (0,5 m) wegen 1 cm aus; mit
`--frei 0.25` gelten sie als passing.

## Nicht anfasse-Regel beachtet, deshalb offen

1. **Generator sichern:** die drei Welten kommen aus `/tmp/gen.py` (~130 LOC, deterministisch,
   Seed 20260912). `/tmp` ist flüchtig, und ASCII-Grids können keine Kommentare tragen
   (Zeile mit `#` = Wand), ohne `parse_grid` anzufassen. Wenn die Welten je nachgebaut
   oder in 0,25-m-Auflösung erzeugt werden sollen: `cp /tmp/gen.py tools/genworlds.py`
   — das wäre deine Datei, nicht meine. Sonst sind `worlds/*.txt` ab jetzt Handarbeit.
2. `worlds.cell` (0,5 m) steckt nur in `parse_grid(text, cell=...)` und `World.cell`; die
   Sim liest die Zellgröße nicht aus der Config. Für die 0,25-m-Idee müsste
   `load_world(name, cell=...)` durchgereicht werden — eine Zeile in `worlds.py` (fremde
   Datei, nicht angefasst).
3. `student/solution.py` kommentiert die Seitenstabilisierung mit „1-m-Korridor"; die
   track-Fahrbahn ist jetzt 2,0 m. Nur ein Kommentar, aber er erzählt Studierenden eine
   Zahl, die nicht mehr zu ihrer Welt passt.
4. T4-Ziel ist `spawns[-1]` (Ladeplatz, unten links, 1,25 m bis zur nächsten Wand). Die
   Ladesäule unten rechts hat damit nichts zu tun — falls die Anleitung/Welt-Erklärung
   „Ladesäule = T4-Ziel" suggeriert, ist das die Stelle, die man umbenennen sollte
   (z. B. Palette/Regal), statt die Aufgabe zu entschärfen.

LOC: worldcheck.py 182 Zeilen, worlds 19/22/24 Zeilen, notes = diese Datei. Kein git, nichts
installiert, headless getestet, keine Zeile in `mecanum_lab/` oder `student/`.
