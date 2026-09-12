"""mecanum_lab — small 2D Mecanum simulator with a ROS 2 binding, for experiments 1 and 2."""
import logging
import os
import sys

__version__ = "0.1"


def setup_logging(level: str | None = None) -> None:
    """Einmaliger Logging-Aufbau; MECANUM_LOG=debug kann ueberschreiben."""
    lvl = os.environ.get("MECANUM_LOG", level or "info").upper()
    logging.basicConfig(level=getattr(logging, lvl, logging.INFO),
                        format="%(levelname).1s %(name)-14s %(message)s",
                        stream=sys.stderr)
