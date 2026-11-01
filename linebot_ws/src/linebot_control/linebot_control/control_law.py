"""Pure steering / speed law shared by the ROS node and the offline simulator."""


def command_from_error(pid, error, far_error, cfg, dt):
    """Return (linear_x, angular_z).

    error > 0  => line is right of centre => turn right => angular_z < 0,
    hence angular_z = -PID(error).
    Speed drops as the line gets off-centre or curves away (look-ahead error).
    """
    w = -pid.update(float(error), dt)
    curve = min(1.0, max(abs(error), abs(far_error)))
    v = cfg['max_speed'] * (1.0 - cfg['slow_gain'] * curve)
    v = min(cfg['max_speed'], max(min(cfg['min_speed'], cfg['max_speed']), v))
    return v, w
