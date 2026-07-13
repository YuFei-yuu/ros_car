from geometry_msgs.msg import Twist


def clamp_symmetric(value, limit):
    limit = abs(float(limit))
    return max(-limit, min(float(value), limit))


def alignment_command(detection, vision_config):
    target = vision_config.get('target_pixel', {})
    error_x = float(target.get('x', 0)) - float(detection.x)
    error_y = float(target.get('y', 0)) - float(detection.y)
    aligned = (
        abs(error_x) <= float(vision_config.get('x_tolerance_px', 10))
        and abs(error_y) <= float(vision_config.get('y_tolerance_px', 10))
    )

    command = Twist()
    if not aligned:
        command.linear.x = clamp_symmetric(
            error_y * float(vision_config.get('linear_gain', 0.0)),
            vision_config.get('max_linear_speed', 0.0),
        )
        command.angular.z = clamp_symmetric(
            error_x * float(vision_config.get('angular_gain', 0.0)),
            vision_config.get('max_angular_speed', 0.0),
        )
    return command, aligned, error_x, error_y
