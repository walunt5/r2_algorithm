import pytest

from r2_nav_action_server.r2_nav_action_server_node import resolve_control_mode


def test_empty_control_mode_defaults_to_x_then_y():
    assert resolve_control_mode("") == "x_then_y"
    assert resolve_control_mode("   ") == "x_then_y"


def test_supported_control_modes_are_preserved():
    assert resolve_control_mode("x_then_y") == "x_then_y"
    assert resolve_control_mode("fixed_map") == "fixed_map"


def test_control_mode_is_trimmed():
    assert resolve_control_mode("  fixed_map  ") == "fixed_map"


def test_unsupported_control_mode_is_rejected():
    with pytest.raises(ValueError):
        resolve_control_mode("live_error")
