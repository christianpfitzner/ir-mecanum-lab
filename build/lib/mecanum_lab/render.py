"""Pygame-Ansicht des Simulators: liest engine.world und engine.robots, aendert nichts.

Die Welt passt mit Rand ins Fenster; 1..9 springt zu einem Roboter, 0 zeigt wieder
alles, +/- zoomt. Pause, Scan, Spur, Zoom und Fokus leben im Renderer, poll() meldet
nur die Tastendrucke seit dem letzten Aufruf — Teleop und Auftragslogik bleiben beim
Aufrufer (node.py). Physik und ROS werden dabei nie angefasst.
"""
import math

import pygame

from .types import cfg_get

MARGIN = 40                                        # Pixelrand, den die Welt nie ueberlaeuft
HELP = "q Ende | SPACE Pause | l Scan | t Spur | k Schätzung | 0 alles | 1..9 Roboter | +/- Zoom"
GREY = (210, 212, 218)
GPS_FARBE = (250, 210, 90)                         # Messung: flach und eckig
KF_FARBE = (120, 240, 170)                         # Schätzung: leuchtend und rund
KF_SICHERHEIT = 2.0                                # wie viele σ die Ellipse in der GUI zeigt
CORNERS = ((1, 1), (1, -1), (-1, 1), (-1, -1))     # FL, FR, RL, RR im Koerperrahmen
ROLLERS = ((1, -1), (1, 1), (1, 1), (1, -1))       # Rollachsen der X-Anordnung
# Kennzeichnung je Roboter als Liste von (radius, winkel in Grad); radius 1 = Fussabdruck.
SHAPES = {
    "triangle": [(1, -90), (1, 30), (1, 150)],
    "square": [(1.2, a) for a in (45, 135, 225, 315)],
    "diamond": [(1.25, a) for a in (0, 90, 180, 270)],
    "circle": [(1, a) for a in range(0, 360, 30)],
    "pentagon": [(1.15, a) for a in range(90, 450, 72)],
    "hexagon": [(1.15, a) for a in range(0, 360, 60)],
    "star": [v for a in range(0, 360, 36) for v in ((1.35, a), (.55, a + 18))],
    "cross": [v for a in range(0, 360, 45) for v in ((1.3, a), (.5, a + 22.5))],
}
# Taste -> (umgeschaltetes Attribut, Name der Flanke im poll()-Worterbuch)
KEYS = {"space": ("paused", "pause"), "l": ("show_scan", "toggle_lidar"),
        "t": ("show_trails", "toggle_trail"), "k": ("show_kf", "toggle_kf")}


def body(theta: float, dx: float, dy: float) -> tuple:
    """Vektor aus dem Koerperrahmen (x vorn, y links) in die Welt drehen."""
    c, s = math.cos(theta), math.sin(theta)
    return (dx * c - dy * s, dx * s + dy * c)


def rgb(color, factor: float = 1.0) -> tuple:
    """Farbe aus types.PALETTE (0..1) -> pygame (0..255), optional abgedunkelt."""
    return tuple(min(255, int(255 * v * factor)) for v in color)


def mix(a: tuple, b: tuple, part: float = .5) -> tuple:
    """Zwei Farben mischen; Teil 0..1 zugunsten von b."""
    return tuple(int((1 - part) * x + part * y) for x, y in zip(rgb(a), rgb(b)))


def _add(p: tuple, d: tuple) -> tuple:
    return (p[0] + d[0], p[1] + d[1])


def shape(name: str, centre: tuple, radius: float, theta: float) -> list:
    """Eckpunkte einer Kennzeichnungsform in Pixeln, unbekannte Namen als Kreis.

    Das Minus vor theta gleicht die nach unten zeigende Bildschirmy aus, damit sich die
    Form mit dem Roboter mitdreht statt spiegelverkehrt zu stehen.
    """
    return [(centre[0] + radius * rt * math.cos(math.radians(ang) - theta),
             centre[1] + radius * rt * math.sin(math.radians(ang) - theta))
            for rt, ang in SHAPES.get(name) or SHAPES["circle"]]


class Renderer:
    """Ein Frame pro draw(), Tastendrucke pro poll(); Zustand hier, Entscheidungen dort."""

    def __init__(self, engine, cfg: dict):
        style = cfg_get(cfg, "gui_style") or {}
        self.engine, self.cfg, self.size = engine, cfg, (int(cfg_get(cfg, "width", 1120)),
                                                         int(cfg_get(cfg, "height", 700)))
        self.col_wall = rgb(style.get("wall", (.34, .36, .42)))
        self.col_floor = rgb(style.get("floor", (.13, .14, .17)))
        self.scan_dim = 1 - int(style.get("lidar_alpha", 70)) / 255      # helle, blasse Punkte
        self.trail_len = int(style.get("trail_len", 400))
        self.show_scan, self.show_trails, self.paused = True, True, False
        self.show_kf = True
        self.kf_spur = {}                                      # letzte Schätzungen je Name
        self.zoom, self.focus = 1.0, None
        try:                                                    # bewusst ohne pygame.init():
            pygame.display.init()                               # Audio und Joystick braucht
            pygame.font.init()                                  # die Ansicht nicht
        except pygame.error as exc:
            raise RuntimeError("Kein Display — fuer Headless SDL_VIDEODRIVER=dummy setzen."
                               ) from exc
        self.screen = pygame.display.set_mode(self.size)
        pygame.display.set_caption(f"mecanum_lab — {engine.world.name}")
        pygame.event.get()                                      # Reste eines Vorfensters weg
        self.font, self.big = pygame.font.Font(None, 17), pygame.font.Font(None, 21)
        self.clock = pygame.time.Clock()
        self.trails, self.phase = {}, {}                        # Pfad und Radverdrehung je Name
        self.s, self.ox, self.oy = 100.0, 0.0, 0.0              # Pixel je Meter, Bildschirmnull
        self._t, self._closed, self._alive = float(engine.t), False, True

    def _cam(self) -> None:
        """Pixel je Meter und Bildschirm-Ursprung bestimmen (einmal pro Frame)."""
        wx, wy = [max(v, .5) for v in (self.engine.world.size or (10, 10))]
        s = min((self.size[0] - 2 * MARGIN) / wx, (self.size[1] - 2 * MARGIN) / wy)
        s *= min(max(self.zoom, 1.0), 8.0)
        bots = list(self.engine.robots.values())
        rx, ry = (bots[self.focus].pose.x, bots[self.focus].pose.y) if self.focus is not None \
            and self.focus < len(bots) else (wx / 2, wy / 2)
        hvx, hvy = self.size[0] / (2 * s), self.size[1] / (2 * s)
        rx = wx / 2 if hvx >= wx / 2 else min(max(rx, hvx), wx - hvx)   # Welt bleibt sichtbar
        ry = wy / 2 if hvy >= wy / 2 else min(max(ry, hvy), wy - hvy)
        self.s, self.ox, self.oy = s, self.size[0] / 2 - rx * s, self.size[1] / 2 + ry * s

    def px(self, x: float, y: float) -> tuple:
        """Weltmeter -> Pixel: Welty zeigt nach oben, Bildschirmy nach unten."""
        return (self.ox + x * self.s, self.oy - y * self.s)

    def _text(self, txt, x, y, color, big=False, center=False) -> int:
        img = (self.big if big else self.font).render(txt, True, color)
        self.screen.blit(img, img.get_rect(center=(x, y)) if center else (x, y))
        return img.get_width()

    def draw(self, cap: bool = True) -> None:
        """Ein kompletter Frame. cap=False ohne Frameraten-Bremse (fuer Messungen)."""
        eng = self.engine
        dt = max(0.0, min(float(eng.t) - self._t, .5))          # nur Simzeit treibt die Rollen
        self._t = float(eng.t)
        self._cam()
        self.screen.fill(self.col_floor)
        self._world()
        self.trails = {k: v for k, v in self.trails.items() if k in eng.robots}
        self.kf_spur = {k: v for k, v in self.kf_spur.items() if k in eng.robots}
        self.phase = {k: v for k, v in self.phase.items() if k in eng.robots}
        for robot in eng.robots.values():
            self._trail(robot)
            if self.show_scan:
                self._scan_dots(robot)
            if self.show_kf:
                self._schaetzung(robot)
        for robot in eng.robots.values():
            self._robot(robot, dt)
        self._hud()
        if self.paused:
            self._text("PAUSE", self.size[0] / 2, 24, (250, 220, 120), True, True)
        pygame.display.flip()
        if cap:
            self.clock.tick(int(cfg_get(self.cfg, "gui_rate", 30) or 30))

    def _world(self) -> None:
        """Deko-Linien, Waende und Ziel."""
        w, sc = self.engine.world, self.screen
        for x0, y0, x1, y1 in w.markings or []:
            pygame.draw.line(sc, mix(self.col_floor, (1, 1, 1), .45), self.px(x0, y0),
                             self.px(x1, y1), 2)
        for wall in w.walls or []:
            rect = pygame.Rect(self.px(wall.x0, wall.y1),
                               (max(1, int((wall.x1 - wall.x0) * self.s)),
                                max(1, int((wall.y1 - wall.y0) * self.s))))
            pygame.draw.rect(sc, self.col_wall, rect)
            pygame.draw.rect(sc, mix(self.col_wall, (1, 1, 1), .3), rect, 1)
        if w.goal:
            centre = self.px(w.goal.x, w.goal.y)
            for step, rad in enumerate((16, 10, 4)):            # Zielscheibe
                pygame.draw.circle(sc, mix((1, .85, .3), (1, 1, 1), step / 3), centre, rad,
                                   2 if step else 0)
            self._text("Ziel", centre[0] - 12, centre[1] - 32, (1, .85, .3))

    def _scan_dots(self, robot) -> None:
        scan = robot.scan
        if not scan or not scan.ranges:
            return
        color = mix(rgb(robot.spec.rgb), self.col_floor, self.scan_dim)
        for i in range(0, len(scan.ranges), max(1, len(scan.ranges) // 180)):
            rng = scan.ranges[i]
            if not math.isfinite(rng) or rng <= scan.range_min:     # inf/nan = kein Treffer
                continue
            wx, wy = _add((robot.pose.x, robot.pose.y),
                          body(robot.pose.theta + scan.angle_min + i * scan.angle_increment,
                               rng, 0))
            pygame.draw.rect(self.screen, color, (self.px(wx, wy), (3, 3)))

    def _schaetzung(self, robot) -> None:
        """Versuch 2: Rohmessung (Kreuz), eigene Schätzung (Raute + σ-Ellipse), Fehlerstrich.

        Die Ellipse ist keine Deko: wer eine zu kleine σ angibt, sieht sofort einen Filter,
        der neben der Wahrheit herläuft, aber einen winzigen Streukreis malt — genau der
        Selbstbetrug, den Auftrag K3 bestraft.
        """
        sc, name = self.screen, robot.spec.name
        if robot.gps:
            px = self.px(robot.gps.x, robot.gps.y)
            pygame.draw.line(sc, GPS_FARBE, (px[0] - 5, px[1]), (px[0] + 5, px[1]), 2)
            pygame.draw.line(sc, GPS_FARBE, (px[0], px[1] - 5), (px[0], px[1] + 5), 2)
        kf = robot.kf
        if not kf:
            return
        spur = self.kf_spur.setdefault(name, [])
        if not spur or math.dist((kf.x, kf.y), spur[-1]) > .02:
            spur.append((kf.x, kf.y))
            del spur[:-max(1, self.trail_len)]
        if len(spur) > 1:
            pygame.draw.lines(sc, KF_FARBE, False, [self.px(x, y) for x, y in spur], 2)
        mittig, radius = self.px(kf.x, kf.y), max(4, KF_SICHERHEIT * self.s * max(kf.sx, 1e-4))
        hoehe = max(4, KF_SICHERHEIT * self.s * max(kf.sy, 1e-4))
        pygame.draw.ellipse(sc, KF_FARBE, (mittig[0] - radius, mittig[1] - hoehe,
                                          2 * radius, 2 * hoehe), 1)
        pygame.draw.polygon(sc, KF_FARBE, [(mittig[0], mittig[1] - 6), (mittig[0] + 6, mittig[1]),
                                           (mittig[0], mittig[1] + 6), (mittig[0] - 6, mittig[1])])
        pygame.draw.line(sc, (240, 120, 120), mittig, self.px(robot.pose.x, robot.pose.y), 1)

    def _trail(self, robot) -> None:
        pts = self.trails.setdefault(robot.spec.name, [])
        here = (robot.pose.x, robot.pose.y)
        if not pts or math.dist(here, pts[-1]) > .03:
            pts.append(here)
            del pts[:-max(1, self.trail_len)]
        if self.show_trails and len(pts) > 1:
            pygame.draw.lines(self.screen, rgb(robot.spec.rgb, .6), False,
                              [self.px(x, y) for x, y in pts], 2)

    def _robot(self, robot, dt: float) -> None:
        """Fussabdruck, Kennzeichnung, Richtungspfeil, vier Mecanum-Raeder, Namenslabel."""
        pose, col = robot.pose, rgb(robot.spec.rgb)
        lx, ly, wr, fp_m = self._geom(robot)
        centre, fp = self.px(pose.x, pose.y), max(4, fp_m * self.s)
        pygame.draw.circle(self.screen, mix(col, self.col_floor, .55), centre, fp, 1)
        pygame.draw.polygon(self.screen, col, shape(robot.spec.marker, centre, fp * .55,
                                                    pose.theta))
        pygame.draw.line(self.screen, col, centre,
                         self.px(*_add((pose.x, pose.y), body(pose.theta, fp_m, 0))), 3)
        for i, (sx, sy) in enumerate(CORNERS):
            self._wheel(robot.spec.name,
                        self.px(*_add((pose.x, pose.y), body(pose.theta, sx * lx, sy * ly))),
                        pose.theta, robot.wheels[i] if i < len(robot.wheels) else 0.0,
                        i, ROLLERS[i], col, max(3, wr * self.s), dt)
        speed = math.hypot(robot.twist.vx, robot.twist.vy)
        if speed > .02:                                         # Geschwindigkeit als Strahl
            heading = pose.theta + math.atan2(robot.twist.vy, abs(robot.twist.vx) or 1e-6)
            pygame.draw.line(self.screen, mix(col, (1, 1, 1), .5), centre,
                             self.px(*_add((pose.x, pose.y), body(heading, speed, 0))), 1)
        self._text(robot.spec.name, centre[0], centre[1] - fp - 12, col, center=True)

    def _wheel(self, name, centre, theta, spin, index, roller, col, size, dt) -> None:
        """Rad als Linie in Fahrtrichtung, drei Rollen als mitlaufende Diagonalstriche.

        Die Rollenbewegung ist auf 0.25 geeicht und nur zur Anschauung — bei 12 rad/s
        und 30 fps wuerde sie sonst nur flackern.
        """
        phase = self.phase.setdefault(name, [0.0] * 4)
        phase[index] = (phase[index] + spin * dt * .25) % math.tau
        u = body(-theta, 1, 0)                    # minus: die Bildschirmy zeigt nach unten
        pygame.draw.line(self.screen, col, _add(centre, (-u[0] * size, -u[1] * size)),
                         _add(centre, (u[0] * size, u[1] * size)), 2)
        roll = body(-theta, roller[0] / 1.4143, roller[1] / 1.4143)
        for k in range(3):
            off = (phase[index] + k * math.tau / 3) % math.tau / math.tau * 2 - 1
            mid = _add(centre, (u[0] * size * off, u[1] * size * off))
            pygame.draw.line(self.screen, mix(col, (1, 1, 1), .45), mid,
                             _add(mid, (roll[0] * size * .8, roll[1] * size * .8)), 1)

    def _hud(self) -> None:
        """Kopfzeile plus eine Zeile je Roboter; ab 5 Robotern zwei Spalten."""
        eng = self.engine
        self._text(f"{eng.world.name}  t={eng.t:6.1f}s  {self.clock.get_fps():4.0f} fps  "
                   f"Roboter: {len(eng.robots)}  Auftrag: {eng.task or '-'}  {HELP}",
                   8, 6, (235, 235, 240), big=True)
        width = self.size[0] // 2 if len(eng.robots) > 4 else self.size[0]
        for i, r in enumerate(eng.robots.values()):
            o = (f"x={r.odom.x:+.2f} y={r.odom.y:+.2f} th={math.degrees(r.odom.theta):+.0f}"
                 if r.odom else "keine Odometrie")
            x = 8 + (i // 10) * width
            y = 30 + (i % 10) * 17
            pygame.draw.rect(self.screen, rgb(r.spec.rgb), (x, y + 3, 9, 9))
            for label in [(r.spec.name, rgb(r.spec.rgb)), (r.spec.variant, GREY),
                          (r.mode, GREY),
                          (f"|v|={math.hypot(r.twist.vx, r.twist.vy):.2f} m/s", GREY),
                          (f"w={r.twist.omega:+.2f}", GREY), ("odom " + o, GREY),
                          ("gps " + (f"x={r.gps.x:+.2f} y={r.gps.y:+.2f}" if r.gps
                                     else "kein fix"), GREY),
                          ("kf " + (f"x={r.kf.x:+.2f} y={r.kf.y:+.2f} "
                                    f"σ=({r.kf.sx:.2f},{r.kf.sy:.2f}) "
                                    f"Δ={r.kf_err:.2f} m" if r.kf else "-"), KF_FARBE),
                          (f"Weg={r.distance:.1f}m", GREY), (f"Kontakt={r.contacts}", GREY),
                          (r.mission_state, GREY)]:
                x += self._text(label[0], x + 13, y, label[1]) + 8

    def _geom(self, robot) -> list:
        """lx, ly, radradius, fussabdruck — aus der Physik, sonst aus der cfg."""
        g = getattr(robot.chassis, "geom", None)
        return [getattr(g, key, cfg_get(self.cfg, "robot." + key, dflt))
                for key, dflt in (("lx", .14), ("ly", .13), ("r", .05), ("footprint_r", .21))]

    def poll(self) -> dict:
        """Tastendrucke seit dem letzten Aufruf; der Zustand wechselt hier, Flanke wird gemeldet."""
        flags = {"quit": False, "pause": False, "toggle_lidar": False,
                 "toggle_trail": False, "camera": None, "key": ""}
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self._alive, flags["quit"] = False, True
            elif ev.type == pygame.KEYDOWN:
                key = pygame.key.name(ev.key)
                flags["key"] = key
                if key in KEYS:
                    attr, flank = KEYS[key]
                    setattr(self, attr, not getattr(self, attr))
                    flags[flank] = True
                elif key in ("q", "escape"):
                    self._alive, flags["quit"] = False, True
                elif key == "0":
                    self.focus, flags["camera"] = None, 0
                elif key in "123456789":
                    self.focus = int(key) - 1
                    flags["camera"] = self.focus
                elif key in ("+", "="):
                    self.zoom = min(self.zoom * 1.25, 8.0)
                elif key in ("-", "_"):
                    self.zoom = max(self.zoom / 1.25, 1.0)
        return flags

    @property
    def ok(self) -> bool:
        return bool(self._alive and not self._closed and pygame.display.get_init())

    def close(self) -> None:
        if not self._closed:                                      # pygame.quit nur einmal
            self._closed, self._alive = True, False
            pygame.quit()
