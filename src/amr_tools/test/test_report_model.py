"""Unit tests for the ROS-free health summarization helpers."""
from amr_tools import report_model
from amr_tools.report_model import ERROR, OK, STALE, WARN


def test_worst_diagnostic_picks_highest_level():
    level, name = report_model.worst_diagnostic(
        [(OK, "a"), (ERROR, "motor"), (WARN, "battery")]
    )
    assert level == ERROR
    assert name == "motor"


def test_worst_diagnostic_ranks_stale_above_error():
    level, name = report_model.worst_diagnostic([(ERROR, "motor"), (STALE, "lidar")])
    assert level == STALE
    assert name == "lidar"


def test_worst_diagnostic_all_ok():
    assert report_model.worst_diagnostic([(OK, "a"), (OK, "b")]) == (OK, "none")


def test_worst_diagnostic_empty():
    assert report_model.worst_diagnostic([]) == (OK, "none")


def test_topic_liveness_states():
    assert report_model.topic_liveness(None, 1.5) == "missing"
    assert report_model.topic_liveness(3.0, 1.5) == "stale"
    assert report_model.topic_liveness(0.5, 1.5) == "ok"


def test_scan_summary_counts_and_nearest():
    summary = report_model.scan_summary([0.05, 1.0, 2.0, 99.0], 0.1, 8.0)
    assert summary["beams"] == 4
    assert summary["valid"] == 2  # 0.05 and 99.0 are out of range
    assert summary["nearest_m"] == 1.0


def test_scan_summary_no_valid_returns_none():
    summary = report_model.scan_summary([100.0, 200.0], 0.1, 8.0)
    assert summary["valid"] == 0
    assert summary["nearest_m"] is None


def test_level_name():
    assert report_model.level_name(WARN) == "WARN"
    assert report_model.level_name(99) == "99"
