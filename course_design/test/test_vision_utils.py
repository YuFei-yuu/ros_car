from types import SimpleNamespace

from course_design.vision_utils import alignment_command, clamp_symmetric


def test_clamp_symmetric_limits_both_directions():
    assert clamp_symmetric(2.0, 0.3) == 0.3
    assert clamp_symmetric(-2.0, 0.3) == -0.3
    assert clamp_symmetric(0.2, 0.3) == 0.2


def test_alignment_command_stops_inside_tolerance():
    config = {
        'target_pixel': {'x': 320, 'y': 388},
        'x_tolerance_px': 10,
        'y_tolerance_px': 10,
        'linear_gain': 0.01,
        'angular_gain': 0.01,
        'max_linear_speed': 0.03,
        'max_angular_speed': 0.12,
    }
    command, aligned, error_x, error_y = alignment_command(
        SimpleNamespace(x=325, y=380), config)

    assert aligned is True
    assert error_x == -5.0
    assert error_y == 8.0
    assert command.linear.x == 0.0
    assert command.angular.z == 0.0


def test_alignment_command_applies_configured_speed_limits():
    config = {
        'target_pixel': {'x': 320, 'y': 388},
        'x_tolerance_px': 1,
        'y_tolerance_px': 1,
        'linear_gain': 0.01,
        'angular_gain': 0.01,
        'max_linear_speed': 0.03,
        'max_angular_speed': 0.12,
    }
    command, aligned, _, _ = alignment_command(
        SimpleNamespace(x=0, y=0), config)

    assert aligned is False
    assert command.linear.x == 0.03
    assert command.angular.z == 0.12
