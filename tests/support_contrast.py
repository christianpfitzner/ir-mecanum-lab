"""The one number a label has to reach: its contrast against whatever it is drawn on.

Three labels in the window were painted a colour a hair away from the floor under them. Two because
pygame truncates a colour written 0..1 — `(1, .85, .3)`, the amber of the goal, arrives at the font
as `(1, 0, 0)` — and one because a label wore the colour of its own hatch. Nobody caught it in a
screenshot, because "barely visible" and "invisible" look identical to anyone who already knows what
is supposed to stand there.

So the check is the number WCAG defines rather than a glance: 1.0 is no contrast at all, 4.5 is what
small text needs. The reference is the dark edge `render._text()` draws behind every label — no
colour of `types.PALETTE` reaches 4.5:1 on a wall (86, 91, 107), so the map under a label is not what
decides whether it can be read; the edge under it is. A label in a colour too dark for that edge is
still unreadable, which is what these assertions are for.
"""


def leucht(color) -> float:
    """Relative luminance of one colour, the way WCAG defines it."""
    def kurve(v: float) -> float:
        v /= 255
        return v / 12.92 if v <= 0.03928 else ((v + 0.055) / 1.055) ** 2.4
    return sum(g * kurve(c) for g, c in zip((0.2126, 0.7152, 0.0722), color[:3]))


def ratio(a, b) -> float:
    """Contrast ratio of two colours: 1.0 is invisible, 4.5 is what small text has to reach."""
    hell, dunkel = sorted([leucht(a), leucht(b)], reverse=True)
    return (hell + 0.05) / (dunkel + 0.05)
