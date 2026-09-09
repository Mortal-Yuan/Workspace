"""Custom low pickup pose captured on 2026-09-09.

This module only builds command strings. Importing it does not drive servos.
"""


# Exact readback after the arm was positioned by hand while torque was released.
CAPTURED_POSE = (1496, 788, 1816, 1265, 1496, 1235)

# Repeatable pickup endpoint: same arm pose, with the gripper fully open.
PICKUP_OPEN_POSE = (1496, 788, 1816, 1265, 1496, 1200)

GRIPPER_CLOSED = 1700


def pose_command(pose=PICKUP_OPEN_POSE, duration_ms=3000):
    """Return one six-servo command; this function does not send it."""
    if len(pose) != 6:
        raise ValueError("pose must contain six servo pulse values")
    return "{" + "".join(
        "#%03dP%04dT%04d!" % (servo_id, pulse, duration_ms)
        for servo_id, pulse in enumerate(pose)
    ) + "}"


def close_gripper_command(duration_ms=1000):
    """Return the gripper-close command; this function does not send it."""
    return "#005P%04dT%04d!" % (GRIPPER_CLOSED, duration_ms)


CAPTURED_POSE_COMMAND = pose_command(CAPTURED_POSE)
PICKUP_OPEN_COMMAND = pose_command(PICKUP_OPEN_POSE)
PICKUP_CLOSE_COMMAND = close_gripper_command()

