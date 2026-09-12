"""Camera of the view: world metres <-> screen pixels, zoom to cursor, drag pan, resize.

The camera owns scale and centre and nothing else — it never reads the engine and never
decides what is drawn. Two things matter while teaching:

* `px_per_meter_min` keeps a 15 m hall from shrinking the robot to a few pixels. The whole
  world is therefore not always in view by default — `f` or zooming out shows all of it.
* zooming happens around the cursor, so the spot you are looking at stays on screen.
"""
MIN_PX = 240                                       # smallest window, below this it is useless


class Camera:
    """Scale `s` (pixels per metre) and the world point `cx, cy` shown at the centre."""

    def __init__(self, size, world_size=(10.0, 10.0), zoom=1.0, margin=16,
                 px_per_meter_min=50.0):
        self.margin, self.px_min = margin, float(px_per_meter_min)
        self.size, self.zoom = _solid(size), float(zoom)
        self.fit, self.fit_all = 50.0, 50.0
        self.world = tuple(world_size)
        self.cx, self.cy, self.s = 0.0, 0.0, 50.0
        self.update(self.world)

    # ----------------------------------------------------------------------- Kalibrierung

    def resize(self, size) -> None:
        self.size = _solid(size)
        self.update(self.world)

    def update(self, world_size) -> None:
        """Fit scale, pixels per metre and a centre that keeps the world in reach."""
        self.world = tuple(world_size)
        wx, wy = [max(v, 0.5) for v in self.world]
        whole = min((self.size[0] - 2 * self.margin) / wx,
                   (self.size[1] - 2 * self.margin) / wy)
        self.fit_all = whole
        self.fit = max(whole, self.px_min)
        self.s = self.fit * min(max(self.zoom, 0.25), 20.0)
        self.clamp()

    def clamp(self) -> None:
        """Let the centre wander over the world plus half a view — the border stays on screen."""
        hvx, hvy = self.size[0] / (2 * self.s), self.size[1] / (2 * self.s)
        self.cx = self.world[0] / 2 if hvx >= self.world[0] / 2 \
            else min(max(self.cx, hvx * 0.35), self.world[0] - hvx * 0.35)
        self.cy = self.world[1] / 2 if hvy >= self.world[1] / 2 \
            else min(max(self.cy, hvy * 0.35), self.world[1] - hvy * 0.35)

    # ---------------------------------------------------------------------------- Bedienung

    def center(self) -> None:
        """`f`: whole world in view and back in the middle, however small that makes it."""
        self.update(self.world)
        self.cx, self.cy = self.world[0] / 2, self.world[1] / 2
        self.zoom = min(max(self.fit_all / self.fit, 0.25), 20.0)
        self.update(self.world)

    def center_on(self, x: float, y: float) -> None:
        self.cx, self.cy = float(x), float(y)
        self.update(self.world)

    def zoom_to(self, factor: float) -> None:
        """Keyboard zoom: keep the centre, only change the scale."""
        self.zoom_to_at((self.size[0] / 2, self.size[1] / 2), factor)

    def zoom_to_at(self, pixel, factor: float) -> None:
        """Wheel zoom: the world point under the cursor stays under the cursor."""
        wx, wy = self.wx(*pixel)
        self.zoom = min(max(self.zoom * factor, 0.25), 20.0)
        self.update(self.world)
        self.cx, self.cy = wx + (self.size[0] / 2 - pixel[0]) / self.s, \
            wy + (pixel[1] - self.size[1] / 2) / self.s
        self.clamp()

    def pan(self, dpx: float, dpy: float) -> None:
        """Drag: the world moves with the mouse, not against it."""
        self.cx, self.cy = self.cx - dpx / self.s, self.cy + dpy / self.s
        self.clamp()

    # --------------------------------------------------------------------------- Rechenweg

    def px(self, x: float, y: float) -> tuple:
        """World metres -> pixels. World y points up, screen y points down."""
        return (self.size[0] / 2 + (x - self.cx) * self.s,
                self.size[1] / 2 - (y - self.cy) * self.s)

    def wx(self, sx: float, sy: float) -> tuple:
        """Pixels -> world metres (for zoom around the cursor)."""
        return (self.cx + (sx - self.size[0] / 2) / self.s,
                self.cy - (sy - self.size[1] / 2) / self.s)

    @property
    def rect(self) -> tuple:
        """Screen rectangle of the world — everything outside it is void."""
        top, bottom = self.px(0.0, self.world[1]), self.px(self.world[0], 0.0)
        return (int(top[0]), int(top[1]), max(1, int(bottom[0] - top[0])),
                max(1, int(bottom[1] - top[1])))

    @property
    def meters_wide(self) -> float:
        return self.size[0] / self.s


def _solid(size) -> tuple:
    return (max(MIN_PX, int(size[0])), max(MIN_PX // 2 + 1, int(size[1])))
