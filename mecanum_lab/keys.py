"""Every key of the window in one table, plus the check that proves no key means two things.

Four places used to write bindings down, and they drifted. The letters that switched a view layer
lived in `render.KEYS`, the rows and key hints of the panel in `menu.LAYERS`, the help line in the
header was a string no test ever compared with the code, and the keys that drove were read a fourth
time in `node.teleop_keys()` — polled next to the event loop that had already read the same
keyboard. Three real bugs came out of that, and all three stood between a student and the first
minute of driving:

  * `q` turned right and `e` turned left, the opposite of where the two keys lie;
  * `q` also quit, decided by a mode nobody reads;
  * `w`, `s` and `d` switched view layers, so they could not be the driving letters that a WASD
    layout promises — and the help line still named keys whose code was gone.

One rule now, and it is a runnable one: **a key means one thing, and the letters that drive are
never layer switches.** `check_bindings()` below proves it for the table as it stands and is a step
of `tools/check.sh`, because a rule that only lives in a comment is a wish.

    while a key can drive (teleop on)   w / up     forward            s / down   backward
                                       a / left   strafe left        d / right  strafe right
                                       q          turn left          e          turn right
                                       , / .      turn left / right  (the same pair, for the right hand)
                                       SHIFT      both speeds doubled, for as long as it is held
    in every mode                      SPACE pause · ESC quit · m the layer panel · f the whole world
                                       0 all robots · 1…9 follow one · = / - zoom · wheel zooms to the cursor
    the layers (both modes)            l t g k r v c z h x o p n     — see LAYERS

`q` is the one key with two meanings, and they are the two it always had: while teleop is on it
turns, when teleop is off it ends the run — with no teleop nothing is driven, so it is stolen from
nobody. `ESC` and the close button of the window end a run in every mode.

Two layers had to give up a driving letter and kept their meaning instead of their key: the wheels
are switched by `r` (the rollers) and the GPS shadow by `x` (the place where a fix is lost). The
third of them, the painted floor on `c`, is not coming back: a line of paint is neither an obstacle
nor a measurement, so the layer was deleted and `c` now switches the radio coverage map.

The letters then moved once more, and the reason is the chassis: `a`/`d` strafe and `q`/`e` turn,
because the one degree of freedom a car does not have and a mecanum base does is sideways. Turning
was on `a`/`d` while sideways sat on the arrow keys, which taught the layout of a car and left the
interesting axis on the keys nobody reaches for. `SHIFT` is a modifier rather than a binding: it is
never alone in `DRIVING`, it cannot switch a layer, and it scales the axes `driving_twist()` returns
instead of adding a fourth kind of command — the same `Twist` on the same topic, only larger.

Names are the pygame names — what `pygame.key.name()` reports for a press and what
`pygame.key.key_code()` accepts for the polling side. Lowercase, ASCII, like everywhere else here.
"""

# ------------------------------------------------------------------------------------- driving
# key name -> (forward, sideways left, counter-clockwise), one axis per key, as unit values;
# TELEOP_SPEED and TELEOP_YAW below turn a held key into a body velocity. The signs follow
# CONTRACT section 5 (y to the left, theta counter-clockwise), so "turn left" is positive omega
# here and everywhere else in this package.
#
# W/S/A/D are the layout everybody knows from a game, and here the letters sit on the two axes a
# mecanum chassis actually has: w/s along the body, a/d across it. Turning is the third axis and got
# the two keys beside them, q/e. The arrow keys repeat the same physics — up/down drive, left/right
# **strafe** — because the arrows lie where the two axes of the body frame are. Both sets publish one
# Twist on /<robot>/cmd_vel and nothing else (CONTRACT section 6.9), so they cannot disagree about
# physics, only about which finger is nearer.
DRIVING = {
    "w": (1.0, 0.0, 0.0), "up": (1.0, 0.0, 0.0),
    "s": (-1.0, 0.0, 0.0), "down": (-1.0, 0.0, 0.0),
    "a": (0.0, 1.0, 0.0), "left": (0.0, 1.0, 0.0),
    "d": (0.0, -1.0, 0.0), "right": (0.0, -1.0, 0.0),
    "q": (0.0, 0.0, 1.0), ",": (0.0, 0.0, 1.0),
    "e": (0.0, 0.0, -1.0), ".": (0.0, 0.0, -1.0),
}
TELEOP_SPEED = 0.35          # m/s of body speed while a drive key is held — walking pace, on purpose
TELEOP_YAW = 0.9             # rad/s while a turn key is held
BOOST_FACTOR = 2.0           # while SHIFT is held: both at once, so a curve keeps its shape
# The shift keys are the one pair on the keyboard that the *polled array cannot answer for*. Measured
# on pygame 2.6.1: the names do not resolve either way — `key_code("lshift")` and `key_code("leftshift")`
# both raise ValueError — and `key.get_pressed()[K_LSHIFT]` answers False while the key is held down,
# because that array is indexed by scancode and a keysym above 0x40000000 has none to look one up
# from. So a name in BOOST_KEYS is a label of this file and never something to ask pygame about, and a
# held shift is read from `key.get_mods()`: the modifier word, which SDL keeps current from the
# keyboard's own state while events are pumped. Nothing else in this program has needed that word, and
# this is the one place where getting it wrong costs a student forty seconds per lane.
BOOST_KEYS = ("lshift", "rshift")
BOOST_LABEL = "shift"          # what the window, the panel and the handout call the gesture
# A shift on either side is the same gesture, so the set of held keys carries the label rather than a
# claim about which hand pressed. Both spellings arrive here: the label from `pressed_names()`, the
# two key names from whoever types them — one gesture in, one boost out.
BOOST_NAMES = BOOST_KEYS + (BOOST_LABEL,)
FORWARD_KEYS = ("w", "s", "up", "down")
STRAFE_KEYS = ("a", "d", "left", "right")
TURN_KEYS = ("q", "e", ",", ".")

# ---------------------------------------------------------------------------- the view layers
# key name -> (attribute on the renderer, label in the panel, edge the run loop reports). The four
# driving letters are not here — that is the rule, and `check_bindings()` is the proof.
LAYERS = (
    ("l", "show_scan", "lidar scan", "toggle_lidar"),
    ("t", "show_trails", "odometry trail", "toggle_trail"),
    ("g", "show_gps", "gps fix", ""),
    ("k", "show_kf", "estimate + sigma ellipse", ""),
    ("r", "show_wheels", "wheels", ""),
    ("v", "show_velocity", "velocity vector", ""),
    ("z", "show_goal", "goal", ""),
    ("h", "show_hud", "readout lines", ""),
    ("x", "show_zones", "gps shadow zones", ""),
    ("o", "show_ghost", "odometry ghost + drift", ""),
    ("p", "show_pois", "radiation source + field", ""),
    ("i", "show_dose", "radiation dose map", ""),
    ("n", "show_network", "radio link + access point", ""),
    ("c", "show_coverage", "radio coverage map", ""),
)
LAYER_ATTRIBUTES = tuple(row[1] for row in LAYERS)

# ---------------------------------------------------------------------------- the rest, both modes
MENU, FIT, PAUSE, QUIT, ALL_ROBOTS = "m", "f", "space", "escape", "0"
PAUSE_EDGE = "pause"                       # the name the run loop hears when a frame stands still
ZOOM_IN, ZOOM_OUT = ("=", "+"), ("-", "_")        # `+` is the unshifted `=` on a PC keyboard
ROBOTS = tuple(str(digit) for digit in range(1, 10))         # 1..9: follow that robot


# ------------------------------------------------------------------------------------ the lookups
def layer_table() -> dict:
    """name -> (attribute, edge name): the dict a key event handler works through."""
    return {key: (attribute, edge) for key, attribute, _label, edge in LAYERS}


def menu_rows() -> tuple:
    """(attribute, label, key) for the panel: the same rows, columns turned by 90 degrees."""
    return tuple((attribute, label, key) for key, attribute, label, _edge in LAYERS)


def key_of(attribute: str) -> str:
    """The key that switches one renderer attribute, '' when only the panel offers it."""
    return next((key for key, attr, _label, _edge in LAYERS if attr == attribute), "")


def help_line(teleop: bool) -> str:
    """The header line of the window — built from this table, so it cannot name a dead key.

    Worth the string building: it is the only documentation a student reads with the window open,
    and for a long time it was a literal that nothing compared with the code behind it.
    """
    control = (f"w/s drive · a/d strafe · q/e turn · {BOOST_LABEL} ×2 · SPACE pause · ESC quit · "
               if teleop else "SPACE pause · q quit · ")
    return (control + "m layers · f whole world · 0 all · 1..9 follow · =/- zoom · "
                      f"wheel zooms to cursor · drag pans · {layer_hint()}")


def layer_hint() -> str:
    """All layer keys in panel order — part of the header line, and the panel says it too."""
    return "layers: " + " ".join(key for key, _attr, _label, _edge in LAYERS)


# ------------------------------------------------------------------------------ the driving side
def code(name: str) -> int:
    """pygame key code of a name in this table."""
    import pygame
    return pygame.key.key_code(name)


def pressed_names(state, mods: int = 0) -> set:
    """The polled array and the modifier word -> the names of the keys that matter, that are down.

    Asks for the code of every driving name instead of walking all 512 slots of the state: the names
    above are then the whole contract, and a key a keyboard does not have reads as not pressed
    instead of raising in the middle of a run. SHIFT comes in through `mods`, because it is not a slot
    in that array — see the measurement at `BOOST_KEYS`. `driving_twist()` reads the boost out of the
    same set, so a caller cannot forget it and the loop cannot ask for one without the other.
    """
    held = {name for name in DRIVING if _down(state, name)}
    if boost_down(mods):
        held.add(BOOST_LABEL)
    return held


def boost_down(mods: int) -> bool:
    """Is a shift held — asked of the one word that knows, `key.get_mods()`."""
    import pygame
    return bool(mods & pygame.KMOD_SHIFT)


def held_mods() -> int:
    """The modifier word of this moment, and 0 where the event system cannot answer.

    `key.get_mods()` belongs to the video system: asked before a window exists it raises rather than
    answering 0. A held modifier has never been a reason to end a run, so the answer taken here is the
    one that costs a student nothing — the same judgement `_down()` makes for a key this keyboard
    happens not to have. Every real teleop run has a window; this is the path where something ran the
    loop without one, and it should drive at walking pace instead of dying at the first round.
    """
    import pygame
    try:
        return pygame.key.get_mods()
    except pygame.error:
        return 0


def _down(state, name: str) -> bool:
    try:
        return bool(state[code(name)])
    except (KeyError, IndexError, ValueError):
        return False


def driving_twist(names: set) -> tuple:
    """The keys that are down -> (vx, vy, omega): the one place a key becomes a body velocity.

    One axis is the sum of its keys, clamped: `w` and `s` cancel, `a` and `d` cancel, and `q` held
    together with `,` is still one turn at `TELEOP_YAW` and not two. The axes still combine, so
    `w` + `d` is a curve — which is the only thing a differential base cannot do anyway.

    SHIFT multiplies the result, after the clamping and not before it: a held shift may double the
    walking pace to a jogging one, but it cannot make a two-key combination exceed what one key per
    axis means. 0.7 m/s and 1.8 rad/s are still inside what the model drives without slipping, which
    is the reason the factor is a flat two and not a slider.
    """
    axes = [0.0, 0.0, 0.0]
    for key, vector in DRIVING.items():
        if key in names:
            axes = [total + part for total, part in zip(axes, vector)]
    speed, strafe, yaw = (max(-1.0, min(1.0, axis)) for axis in axes)
    boost = BOOST_FACTOR if names & set(BOOST_NAMES) else 1.0
    return (boost * speed * TELEOP_SPEED, boost * strafe * TELEOP_SPEED, boost * yaw * TELEOP_YAW)


# --------------------------------------------------------------------------------- the guarantee
def check_bindings(layers: dict | None = None) -> list:
    """Every double binding of this table as one sentence each; [] means the table is clean.

    Each rule below is a mistake this file's four predecessors actually contained. A key that means
    two things at once is not cosmetic: nobody reports it, they drive badly and blame themselves,
    which in a lab course is the most expensive kind of bug there is.
    """
    layers = layer_table() if layers is None else layers
    problems = []
    for key, (attribute, _edge) in layers.items():
        if key in DRIVING:
            problems.append(f"'{key}' drives and switches the layer '{attribute}'")
    for label, block in (("driving", tuple(DRIVING)), ("layer", tuple(layers))):
        for key in sorted({name for name in block if block.count(name) > 1}):
            problems.append(f"'{key}' is bound twice inside the {label} block")
    for field, values in (("layer key", [row[0] for row in LAYERS]),
                          ("renderer attribute", [row[1] for row in LAYERS]),
                          ("edge name", [row[3] for row in LAYERS if row[3]])):
        for name in sorted({value for value in values if values.count(value) > 1}):
            problems.append(f"the {field} '{name}' appears twice in LAYERS")
    for view_key in (MENU, FIT, PAUSE, QUIT, ALL_ROBOTS) + ROBOTS:
        if view_key in DRIVING or view_key in layers:
            problems.append(f"'{view_key}' is a camera or view key and something else as well")
    for key, attribute, _label, _edge in LAYERS:
        if key != key_of(attribute):
            problems.append(f"the layer '{attribute}' is offered twice: '{key}' and "
                            f"'{key_of(attribute)}'")
    for axis, keys in (("forward", FORWARD_KEYS), ("strafe", STRAFE_KEYS), ("turn", TURN_KEYS)):
        for key in keys:
            if key not in DRIVING:
                problems.append(f"'{key}' is counted on the {axis} axis but binds nothing")
    for key in BOOST_KEYS:
        if key in DRIVING or key in layers or key in (MENU, FIT, PAUSE, QUIT, ALL_ROBOTS):
            problems.append(f"'{key}' is a modifier and something else as well")
    if set(ZOOM_IN) & set(ZOOM_OUT):
        problems.append("zooming in and zooming out share a key")
    return problems
