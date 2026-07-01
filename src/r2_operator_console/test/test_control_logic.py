from r2_operator_console.control_logic import (
    SpeedSettings,
    motion_from_key,
    percent_text,
    speed_scale_from_key,
    velocity_from_motion,
)


def test_official_motion_keys():
    assert motion_from_key('i') == (1, 0, 0, 0)
    assert motion_from_key(',') == (-1, 0, 0, 0)
    assert motion_from_key('j') == (0, 0, 0, 1)
    assert motion_from_key('l') == (0, 0, 0, -1)
    assert motion_from_key('t') == (0, 0, 1, 0)
    assert motion_from_key('b') == (0, 0, -1, 0)


def test_shift_holonomic_keys():
    assert motion_from_key('U') == (1, 1, 0, 0)
    assert motion_from_key('O') == (1, -1, 0, 0)
    assert motion_from_key('M') == (-1, 1, 0, 0)
    assert motion_from_key('>') == (-1, -1, 0, 0)
    assert motion_from_key('J') == (0, 1, 0, 0)
    assert motion_from_key('L') == (0, -1, 0, 0)


def test_speed_scaling_keys():
    assert speed_scale_from_key('q') == (1.1, 1.1)
    assert speed_scale_from_key('z') == (0.9, 0.9)
    assert speed_scale_from_key('w') == (1.1, 1.0)
    assert speed_scale_from_key('x') == (0.9, 1.0)
    assert speed_scale_from_key('e') == (1.0, 1.1)
    assert speed_scale_from_key('c') == (1.0, 0.9)


def test_speed_clamps_to_limits():
    speeds = SpeedSettings(vx=2.45, vy=2.45, wz=1.15, max_vx=2.5, max_vy=2.5, max_wz=1.2)
    scaled = speeds.scaled(1.1, 1.1)
    assert scaled.vx == 2.5
    assert scaled.vy == 2.5
    assert scaled.wz == 1.2


def test_velocity_from_motion():
    speeds = SpeedSettings(vx=0.5, vy=0.4, wz=1.0, linear_z=0.2)
    assert velocity_from_motion((1, -1, 0, 1), speeds) == (0.5, -0.4, 0.0, 1.0)
    assert velocity_from_motion((0, 0, -1, 0), speeds) == (0.0, 0.0, -0.2, 0.0)


def test_percent_text():
    assert percent_text(0.5, 2.5) == '20%'
    assert percent_text(2.7, 2.5) == '100%'
