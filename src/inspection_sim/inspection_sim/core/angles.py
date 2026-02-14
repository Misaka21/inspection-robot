import math


def normalize_angle_rad(a: float) -> float:
    """Normalize angle to [-pi, pi]."""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def quat_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    """Return (x,y,z,w) quaternion for yaw around +Z."""
    half = 0.5 * float(yaw_rad)
    return (0.0, 0.0, math.sin(half), math.cos(half))

