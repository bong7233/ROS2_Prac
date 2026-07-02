"""Unit tests for the ROS-free mission state machine."""
from amr_mission.mission_coordinator import (
    CHARGING,
    PATROL,
    RETURN_TO_DOCK,
    MissionConfig,
    decide,
)

CONFIG = MissionConfig()


def test_patrol_stays_while_battery_ok():
    d = decide(PATROL, battery_pct=0.5, docked=False, config=CONFIG)
    assert d.state == PATROL
    assert d.enable_navigation and not d.enable_docking


def test_patrol_returns_to_dock_when_low():
    d = decide(PATROL, battery_pct=0.25, docked=False, config=CONFIG)
    assert d.state == RETURN_TO_DOCK
    assert d.enable_docking and not d.enable_navigation


def test_return_to_dock_continues_until_docked():
    d = decide(RETURN_TO_DOCK, battery_pct=0.2, docked=False, config=CONFIG)
    assert d.state == RETURN_TO_DOCK
    assert d.enable_docking and not d.enable_navigation


def test_return_to_dock_becomes_charging_when_docked():
    d = decide(RETURN_TO_DOCK, battery_pct=0.2, docked=True, config=CONFIG)
    assert d.state == CHARGING
    assert not d.enable_navigation and not d.enable_docking


def test_charging_holds_until_full():
    d = decide(CHARGING, battery_pct=0.5, docked=True, config=CONFIG)
    assert d.state == CHARGING
    assert not d.enable_navigation and not d.enable_docking


def test_charging_resumes_patrol_when_full():
    d = decide(CHARGING, battery_pct=0.95, docked=True, config=CONFIG)
    assert d.state == PATROL
    assert d.enable_navigation and not d.enable_docking


def test_unknown_state_defaults_to_patrol():
    d = decide("BOGUS", battery_pct=0.5, docked=False, config=CONFIG)
    assert d.state == PATROL


def test_low_battery_threshold_is_inclusive():
    d = decide(PATROL, battery_pct=0.30, docked=False, config=CONFIG)
    assert d.state == RETURN_TO_DOCK
