"""Collecting log records in a way that survives whatever else did to the logging machinery.

`caplog` hangs a handler on the **root** logger and relies on propagation. That is fine until a
test in the same process initialises ROS: after an `rclpy` round trip the records of
`mecanum.*` no longer arrive at the root, and four tests that assert on a warning started failing
*only* in a shell where ROS is importable — green in the stub, red in `tools/check.sh --ros`. The
product was right (verified: the warning is emitted and propagates), so the fix belongs in the
tests: attach the collector to the logger that logs, and do not depend on propagation at all.

Use it as a context manager:

    from support_logging import logged

    with logged("mecanum.tasks") as records:
        tasks.load_tasks(path)
    assert any("deprecated key 'titel'" in r.getMessage() for r in records)
"""
import logging


class _Collector(logging.Handler):
    def __init__(self):
        super().__init__(level=logging.NOTSET)
        self.records = []

    def emit(self, record):
        self.records.append(record)


class logged:
    """Every record raised on any of `names` while inside the block.

    One handler per name, and the records are deduplicated: whether a logger forwards to its
    parent is not ours to depend on — under a sourced ROS shell `mecanum.engine` keeps its records
    to itself, which is precisely why `caplog` (a handler on the root logger) saw nothing at all.
    Asking for the names that log, instead of for their common ancestor, is the portable version.
    """

    def __init__(self, *names, level=logging.DEBUG):
        self.names = names or ("mecanum",)
        self.level = level
        self.handler = _Collector()
        self.saved = {}

    def __enter__(self):
        for name in self.names:
            logger = logging.getLogger(name)
            self.saved[name] = (logger.level, logger.propagate, list(logger.handlers))
            logger.setLevel(self.level)
            logger.addHandler(self.handler)
        return self.handler.records

    def __exit__(self, *exc_info):
        for name, (level, propagate, handlers) in self.saved.items():
            logger = logging.getLogger(name)
            for handler in list(logger.handlers):
                if handler is self.handler:
                    logger.removeHandler(handler)
            logger.setLevel(level)
            logger.propagate = propagate
            logger.handlers[:] = handlers
        return False


def messages(records):
    """The text of every collected record — the assertion reads like a sentence."""
    return [record.getMessage() for record in records]
