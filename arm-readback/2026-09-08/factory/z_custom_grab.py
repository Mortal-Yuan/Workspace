"""Custom low pickup pose captured on 2026-09-09.

This module only builds command strings. Importing it does not drive servos.
"""


# Exact readback after the arm was positioned by hand while torque was released.
CAPTURED_POSE = (1496, 788, 1816, 1265, 1496, 1235)

# Repeatable pickup endpoint: same arm pose, with the gripper fully open.
PICKUP_OPEN_POSE = (1496, 788, 1816, 1265, 1496, 1200)

GRIPPER_CLOSED = 1700

# User-selected post-grab holding position, captured on 2026-09-10.
# Readback records the physical loaded pose, not a fresh set of servo targets.
POST_GRAB_HOLD_POSE = (1486, 1082, 2026, 1346, 1496, 1506)

# Last commands that produced the selected pose. Keep these separately because
# the loaded shoulder has a substantial tracking offset. Axes 000-004 only;
# ID005 must retain its existing P1700 grasp, not receive readback P1506.
POST_GRAB_HOLD_TARGETS = (1496, 1123, 2019, 1355, 1495)

# Historical candidate, superseded by the user-selected POST_GRAB_HOLD pose.
# Candidate post-grab lift: approximately +30 mm from PICKUP_OPEN_POSE
# using the factory 105/75/185 mm linkage model, preserving tip pitch/reach.
# 2026-09-10 loaded readback was 873/1930/1261; physical height is unverified.
# Only axes 001-003 are commanded: retain the existing gripper holding target.
POST_GRAB_LIFT_TARGETS = (906, 1927, 1258)


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


def post_grab_lift_command(duration_ms=3000):
    """Build the historical lift candidate; not the selected holding pose."""
    return "{" + "".join(
        "#%03dP%04dT%04d!" % (servo_id, pulse, duration_ms)
        for servo_id, pulse in enumerate(POST_GRAB_LIFT_TARGETS, 1)
    ) + "}"


def post_grab_hold_command(duration_ms=4000):
    """Build holding targets for axes 000-004, preserving the existing grasp.

    Does not transmit. The path from pickup to this endpoint still needs its
    own validation; the user selected the endpoint reached during manual tests.
    """
    return "{" + "".join(
        "#%03dP%04dT%04d!" % (servo_id, pulse, duration_ms)
        for servo_id, pulse in enumerate(POST_GRAB_HOLD_TARGETS)
    ) + "}"


CAPTURED_POSE_COMMAND = pose_command(CAPTURED_POSE)
PICKUP_OPEN_COMMAND = pose_command(PICKUP_OPEN_POSE)
PICKUP_CLOSE_COMMAND = close_gripper_command()
POST_GRAB_LIFT_COMMAND = post_grab_lift_command()
POST_GRAB_HOLD_COMMAND = post_grab_hold_command()
