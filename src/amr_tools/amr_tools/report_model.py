"""ROS-free helpers for summarizing AMR health.

Pulled out of ``health_report`` so the summarization logic (worst diagnostic,
topic liveness, scan summary) can be unit tested without a running ROS graph.
Diagnostic levels follow ``diagnostic_msgs/DiagnosticStatus``:
OK=0, WARN=1, ERROR=2, STALE=3.
"""
from __future__ import annotations

from typing import Dict, Iterable, Optional, Sequence, Tuple

OK = 0
WARN = 1
ERROR = 2
STALE = 3

_LEVEL_NAMES = {OK: "OK", WARN: "WARN", ERROR: "ERROR", STALE: "STALE"}


def level_name(level: int) -> str:
    return _LEVEL_NAMES.get(level, str(level))


def worst_diagnostic(statuses: Iterable[Tuple[int, str]]) -> Tuple[int, str]:
    """Return the highest-severity ``(level, name)``; ``(OK, "none")`` if all OK.

    Ties keep the first status seen, so the result is deterministic. STALE (3)
    ranks above ERROR, matching the diagnostic aggregator convention that absent
    data is the most serious condition.
    """
    worst_level = OK
    worst_name = "none"
    for level, name in statuses:
        if level > worst_level:
            worst_level = level
            worst_name = name
    return worst_level, worst_name


def topic_liveness(age_s: Optional[float], timeout_s: float) -> str:
    """Classify a topic as ``missing`` (never seen), ``stale``, or ``ok``."""
    if age_s is None:
        return "missing"
    if age_s > timeout_s:
        return "stale"
    return "ok"


def scan_summary(
    ranges: Sequence[float],
    range_min: float,
    range_max: float,
) -> Dict[str, Optional[float]]:
    """Summarize a laser scan: beam count, valid count, and nearest return."""
    valid = [r for r in ranges if range_min <= r <= range_max]
    nearest = min(valid) if valid else None
    return {
        "beams": len(ranges),
        "valid": len(valid),
        "nearest_m": nearest,
    }
