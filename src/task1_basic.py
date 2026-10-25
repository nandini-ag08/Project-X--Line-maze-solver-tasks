"""
Task 1 - Basic Coding
Two small functions used later by the line-follower (Task 9).

1. line_position(): weighted centroid of 5 IR sensors
2. clamp_motors():  limit left/right motor commands, keeping their sign
"""

WEIGHTS = [-2, -1, 0, 1, 2]  # sensor positions: leftmost ... rightmost


def line_position(readings, last_position=0.0):
    """
    Estimate where the line is under the sensor array.

    position = sum(weight * reading) / sum(reading)

    Returns (position, line_found).
      position   : -2.0 (line under leftmost sensor) ... +2.0 (rightmost)
      line_found : False if every sensor reads 0 (line lost)

    All-zero case: the denominator would be 0 (division by zero), so we
    check first and return the last known position instead. The caller
    can use line_found=False to trigger lost-line recovery.
    """
    if len(readings) != len(WEIGHTS):
        raise ValueError("expected 5 sensor readings")
    if any(r < 0 for r in readings):
        raise ValueError("readings must be non-negative")

    total = sum(readings)
    if total == 0:                       # explicit all-zero handling
        return last_position, False

    weighted_sum = sum(w * r for w, r in zip(WEIGHTS, readings))
    return weighted_sum / total, True


def clamp_motors(left, right, max_mag):
    """
    Limit each motor command to [-max_mag, +max_mag].
    Sign is preserved, so a reverse command stays a reverse command.
    """
    if max_mag < 0:
        raise ValueError("max_mag must be non-negative")

    def clamp(x):
        return max(-max_mag, min(max_mag, x))

    return clamp(left), clamp(right)


if __name__ == "__main__":
    # ---- line_position examples ----
    print(line_position([0, 0, 1000, 0, 0]))        # centred -> (0.0, True)
    print(line_position([0, 0, 500, 500, 0]))       # between S3,S4 -> (0.5, True)
    print(line_position([1000, 500, 0, 0, 0]))      # far left -> (-1.67, True)
    print(line_position([0, 0, 0, 0, 1000]))        # far right -> (2.0, True)
    print(line_position([0, 0, 0, 0, 0], last_position=1.2))  # lost -> (1.2, False)

    # ---- clamp_motors examples ----
    print(clamp_motors(300, -50, 255))              # (255, -50)
    print(clamp_motors(-400, 400, 255))             # (-255, 255)
    print(clamp_motors(100, 200, 255))              # (100, 200) unchanged

    # ---- simple self-checks ----
    assert line_position([0, 0, 1000, 0, 0]) == (0.0, True)
    assert line_position([0, 0, 500, 500, 0]) == (0.5, True)
    assert line_position([0, 0, 0, 0, 0], 1.2) == (1.2, False)
    assert clamp_motors(300, -50, 255) == (255, -50)
    assert clamp_motors(-400, 400, 255) == (-255, 255)
    print("all tests passed")

    
