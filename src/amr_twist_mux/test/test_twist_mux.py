"""Unit tests for the ROS-free twist mux selection."""
from amr_twist_mux.twist_mux import TwistInput, select_command


def _inp(name, priority, timeout=0.5, lin=0.0, ang=0.0, stamp=None):
    return TwistInput(name, priority, timeout, lin, ang, stamp)


def test_no_inputs_stops():
    assert select_command([], now_s=10.0) == ("none", 0.0, 0.0)


def test_never_published_is_ignored():
    out = select_command([_inp("dock", 10, stamp=None)], now_s=10.0)
    assert out == ("none", 0.0, 0.0)


def test_single_fresh_source_wins():
    out = select_command([_inp("nav", 5, lin=0.3, ang=0.1, stamp=9.9)], now_s=10.0)
    assert out == ("nav", 0.3, 0.1)


def test_higher_priority_wins_among_fresh():
    inputs = [
        _inp("nav", 5, lin=0.3, stamp=9.9),
        _inp("dock", 10, lin=0.1, stamp=9.95),
        _inp("teleop", 20, lin=0.0, ang=0.5, stamp=9.95),
    ]
    name, lin, ang = select_command(inputs, now_s=10.0)
    assert name == "teleop"
    assert ang == 0.5


def test_stale_high_priority_yields_to_fresh_lower():
    inputs = [
        _inp("teleop", 20, lin=1.0, stamp=5.0),   # stale (timeout 0.5)
        _inp("nav", 5, lin=0.3, stamp=9.9),        # fresh
    ]
    name, lin, _ang = select_command(inputs, now_s=10.0)
    assert name == "nav"
    assert lin == 0.3


def test_timeout_boundary_is_inclusive():
    # Exactly at the timeout is still eligible.
    out = select_command([_inp("nav", 5, timeout=0.5, lin=0.2, stamp=9.5)], now_s=10.0)
    assert out[0] == "nav"


def test_priority_tie_resolves_to_first():
    inputs = [
        _inp("a", 10, lin=0.1, stamp=9.9),
        _inp("b", 10, lin=0.2, stamp=9.9),
    ]
    assert select_command(inputs, now_s=10.0)[0] == "a"
