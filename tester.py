"""
Pytest suite for the sivoli quaternion / vector geometry library.

Converted from an interactive script (tester.py) into automated pytest
tests, with no decorators (no @pytest.mark.parametrize, no @pytest.fixture).
All `input("Press Enter to continue...")` prompts and print-based
pass/fail reporting have been replaced with plain `assert` statements, so
the suite can run unattended via:

    pytest tester.py -v

A fixed random seed is set once at import time so failures are
reproducible; remove/change the seed if you want fresh random vectors on
every run.
"""

import math
import random

import sivoli as sv


TOL = 1e-6

random.seed(12345)


def assert_close(a, b, tol=TOL, msg=""):
    """Assert two equal-length sequences (or scalars) are close."""
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        assert abs(a - b) <= tol, f"{msg}: {a} != {b}"
        return
    assert len(a) == len(b), f"{msg}: length mismatch {a} vs {b}"
    for i, (x, y) in enumerate(zip(a, b)):
        assert abs(x - y) <= tol, f"{msg}: element {i} differs: {a} != {b}"


def random_vector_and_angle():
    """Mirror the vector/angle generation used throughout tester.py."""
    v = sv.tolength(
        [random.random() * 10 - 5, random.random() * 10 - 5, random.random() * 10 - 5],
        random.random() * 5000,
    )
    alpha = (random.random() - 0.5) * math.pi * 77
    return v, alpha


def quaternion_from_axis_angle(v, alpha):
    vu = sv.unit(v)
    q = math.cos(alpha / 2)
    qx = vu[0] * math.sin(alpha / 2)
    qy = vu[1] * math.sin(alpha / 2)
    qz = vu[2] * math.sin(alpha / 2)
    return [q, qx, qy, qz]


# ---------------------------------------------------------------------------
# Test 1: quaternion built from axis/angle should match vecs2quat's result
# ---------------------------------------------------------------------------

def test_quaternion_axis_angle_matches_vecs2quat():
    for i in range(10):
        v, alpha = random_vector_and_angle()
        quaternion_in = quaternion_from_axis_angle(v, alpha)

        # two perpendicular reference vectors, rotated by the same axis/angle
        ux = [444.9, 0, 0]
        uy = [0, 1.423, 0]

        urx = sv.rot(ux, v, alpha)
        ury = sv.rot(uy, v, alpha)

        quaternion_out = sv.vecs2quat(urx, ury)

        quaternion_out_flip = quaternion_out[0]

        if quaternion_out_flip[0] < 0 and quaternion_in[0] > 0 or quaternion_out_flip[0] > 0 and quaternion_in[0] < 0:
            quaternion_out_flip = [-x for x in quaternion_out_flip]

        assert_close(
            quaternion_in,
            quaternion_out_flip,
            msg=f"[i={i}] quaternion built from axis/angle does not match vecs2quat output",
        )


# ---------------------------------------------------------------------------
# Test 2: sys2sys2quat should recover the rotation quaternion between two
# rotated coordinate systems
# ---------------------------------------------------------------------------

def test_sys2sys2quat_recovers_rotation():
    for i in range(500):
        # build a rotated coordinate system (ax, ay, az) from a random axis/angle
        v1, alpha1 = random_vector_and_angle()

        ax = [444.9, 0, 0]
        ay = [0, 1.423, 0]

        arx = sv.rot(ax, v1, alpha1)
        ary = sv.rot(ay, v1, alpha1)

        # second random axis/angle (integer-degree angle, like tester.py)
        v2 = sv.tolength(
            [random.random() * 10 - 5, random.random() * 10 - 5, random.random() * 10 - 5],
            random.random() * 5000,
        )
        alpha_deg = random.randint(-180, 180)
        alpha2 = alpha_deg * math.pi / 180

        quaternion_in = quaternion_from_axis_angle(v2, alpha2)

        brx = sv.rot(arx, v2, alpha2)
        bry = sv.rot(ary, v2, alpha2)

        quaternion_out, tmp  = sv.sys2sys2quat(arx, ary, brx, bry)

        quaternion_flip = quaternion_out

        # if quaternion_out[0] < 0 and quaternion_in[0] > 0 or quaternion_out[0] > 0 and quaternion_in[0] < 0:
        #     quaternion_flip = [-x for x in quaternion_out]

        assert_close(
            quaternion_in,
            quaternion_flip,
            msg=f"[i={i}] sys2sys2quat did not recover the expected rotation quaternion",
        )


# ---------------------------------------------------------------------------
# Test 3: rotating a vector and then rotating back by -alpha returns the
# original vector
# ---------------------------------------------------------------------------

def test_vector_rotate_and_unrotate_round_trip():
    a = [-1, 1, 0]
    r = [1.0, 0.0, 0.0]
    alpha = 0.25 * math.pi

    rotated = sv.rot(a, r, alpha)
    unrotated = sv.rot(rotated, r, -alpha)

    assert_close(a, unrotated, msg="rotating then un-rotating did not return the original vector")


# ---------------------------------------------------------------------------
# Test 4: ray/plane intersection against a hand-computed expected point
# ---------------------------------------------------------------------------

def test_ray_plane_intersect():
    normal = [15555, 0, 0]
    planepoint = [0, 6660, -5]
    ray = [-10000, 0, -5000]
    raypoint = [2, 0, 0]

    intersect = sv.rayplanex(normal, planepoint, ray, raypoint)

    # normal points along x, so the plane is x == 0.
    # raypoint + t*ray hits x == 0 at t = 2 / 10000 = 0.0002
    # => point = (0, 0, -1)
    expected = [0.0, 0.0, -1.0]

    assert_close(intersect, expected, msg="ray/plane intersection point is incorrect")


# ---------------------------------------------------------------------------
# Test 5: sys2sys2quat with fixed, known vectors (135 degree rotation about z)
# ---------------------------------------------------------------------------

def test_sys2sys2quat_known_rotation():
    a1 = [0, -1, 0]
    b1 = [1, 0, 0]

    a2 = [-0.5, 0.5, 0]
    b2 = [-0.5, -0.5, 0]

    result = sv.sys2sys2quat(a2, b2, a1, b1)[1]

    # 135 degree rotation around the z axis
    expected = [2.356194490192345, 0.0, 0.0, 1.0]

    assert_close(result, expected, msg="sys2sys2quat did not match the known 135deg z-rotation")


# ---------------------------------------------------------------------------
# Test 6: vecs2ypr with fixed, known vectors (-90 degree yaw only)
# ---------------------------------------------------------------------------

def test_vecs2ypr_known_rotation():
    a = [0, -1, 0]
    b = [1, 0, 0]

    ypr = sv.vecs2ypr(a, b)

    expected = [-math.pi / 2, 0.0, 0.0]

    assert_close(ypr, expected, msg="vecs2ypr did not match the expected -90deg yaw-only result")

