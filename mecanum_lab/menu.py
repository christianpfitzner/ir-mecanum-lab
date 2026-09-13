"""Small click menu for the view: which sensor layers are drawn — and which are not.

Only the drawing is switched off here. The simulator publishes odom, scan, gps, imu and the
estimate on the bus at full rate regardless of what this panel shows, so hiding the lidar
dots does not hide a single message on `/<robot>/scan`. That is the point of the panel: students
see which layer belongs to which topic and can look at the ROS side without a full screen.

The panel is translucent and sits in the top right corner, so the map underneath stays
readable; `m` hides the panel itself.
"""
import pygame

# attribute on the renderer -> (label in the panel, key that also toggles it)
LAYERS = (("show_scan", "lidar scan", "l"), ("show_trails", "odometry trail", "t"),
          ("show_gps", "gps fix", "g"), ("show_kf", "estimate + σ ellipse", "k"),
          ("show_wheels", "wheels", "w"), ("show_velocity", "velocity vector", "v"),
          ("show_markers", "floor markings", "d"), ("show_goal", "goal", "z"),
          ("show_hud", "readout lines", "h"), ("show_zones", "gps shadow zones", "s"),
          ("show_ghost", "odometry ghost + drift", "o"),
          ("show_pois", "radiation source + field", "p"),
          ("show_network", "radio link + access point", "n"))
HEAD = "view layers"
FUSS = "m shows and hides · l t g k w v d z h s o p n"
WIDTH, ROW, PAD = 210, 19, 6


class Panel:
    """A column of checkmarks; `handle` turns a click into a layer name.

    Starts closed: a panel over the map would hide exactly the corner a goal is often placed
    in. `m` (or the HUD line that says so) opens it, clicking a row toggles a layer.
    """

    def __init__(self, font, big):
        self.font, self.big = font, big
        self.open, self.hot = False, -1
        self.rect = pygame.Rect(0, 0, WIDTH, ROW * len(LAYERS) + 52)

    def toggle(self) -> bool:
        self.open = not self.open
        return self.open

    def row_rect(self, index: int) -> pygame.Rect:
        """Hit area of one row, in screen coordinates."""
        return pygame.Rect(self.rect.x, self.rect.y + 24 + index * ROW, WIDTH - 2, ROW)

    def handle(self, ev) -> str:
        """One event, one attribute name if a row was clicked or hovered ('' otherwise)."""
        if not self.open:
            return ""
        if ev.type == pygame.MOUSEMOTION:
            self.hot = next((i for i in range(len(LAYERS)) if self.row_rect(i).collidepoint(ev.pos)),
                            -1)
            return ""
        if ev.type != pygame.MOUSEBUTTONDOWN or ev.button != 1:
            return ""
        for i, (attribut, _text, _taste) in enumerate(LAYERS):
            if self.row_rect(i).collidepoint(ev.pos):
                return attribut
        return ""

    def inside(self, pos) -> bool:
        """True while the mouse is on the panel — a drag there must not pan the map."""
        return bool(self.open and self.rect.collidepoint(pos))

    def draw(self, screen, xy, state) -> None:
        if not self.open:
            return
        self.rect.x, self.rect.y = int(xy[0] - WIDTH - 8), int(xy[1])
        sheet = pygame.Surface((WIDTH, self.rect.height), pygame.SRCALPHA)
        sheet.fill((14, 16, 22, 205))
        sheet.blit(self.big.render(HEAD, True, (235, 235, 240)), (8, 5))
        for i, (attribut, text, taste) in enumerate(LAYERS):
            an, top = getattr(state, attribut, False), 24 + i * ROW
            if i == self.hot:
                pygame.draw.rect(sheet, (58, 64, 82, 235),
                                 pygame.Rect(1, top - 1, WIDTH - 3, ROW))
            checkbox = pygame.Rect(7, top + 3, 13, 13)
            pygame.draw.rect(sheet, (90, 200, 140, 255) if an else (70, 74, 88, 200), checkbox)
            color = (226, 228, 236) if an else (150, 154, 166)
            sheet.blit(self.font.render(text, True, color), (checkbox.right + 7, top + 2))
            sheet.blit(self.font.render(taste, True, (126, 130, 144)), (WIDTH - 20, top + 2))
        sheet.blit(self.font.render(FUSS, True, (150, 154, 168)), (8, self.rect.height - 19))
        pygame.draw.rect(sheet, (96, 102, 120, 255), sheet.get_rect(), 1)
        screen.blit(sheet, self.rect)
