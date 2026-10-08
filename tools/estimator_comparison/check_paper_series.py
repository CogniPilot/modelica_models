#!/usr/bin/env python3
"""Independent exact-arithmetic check of the paper's right-flow Magnus grades.

This is a finite set of algebraic sanity checks, not a proof or a floating-point
error certificate. It uses the ODE coefficient recurrence, not project code.
"""

from fractions import Fraction as F
import numpy as np
import math


def bracket(a, b):
    return a @ b - b @ a


def check(n0, n1):
    size = len(n0)
    zero = np.zeros((size, size), dtype=object)
    eye = np.eye(size, dtype=object)
    q = [eye, n0]
    for k in range(1, 5):
        q.append((q[k] @ n0 + q[k - 1] @ n1) * F(1, k + 1))
    p = [zero, n0, n1 * F(1, 2), bracket(n0, n1) * F(1, 12), zero, zero]
    exp = [eye] + [zero.copy() for _ in range(5)]
    power = [eye] + [zero.copy() for _ in range(5)]
    for degree in range(1, 6):
        power = [
            sum((power[j] @ p[k - j] for j in range(k + 1)), zero.copy())
            for k in range(6)
        ]
        factorial = math.factorial(degree)
        exp = [x + y * F(1, factorial) for x, y in zip(exp, power)]
    residual = [x - y for x, y in zip(q, exp)]
    for k in range(5):
        assert (residual[k] == zero).all(), (k, residual[k])
    xi5 = -F(1, 240) * bracket(n1, bracket(n0, n1)) - F(1, 720) * bracket(
        n0, bracket(n0, bracket(n0, n1))
    )
    assert (residual[5] == xi5).all(), residual[5] - xi5


if __name__ == "__main__":
    rng = np.random.default_rng(2027)
    for size in (2, 3, 5):
        for _ in range(4):
            check(
                rng.integers(-3, 4, (size, size)).astype(object),
                rng.integers(-3, 4, (size, size)).astype(object),
            )
    print(
        "PASS: 12 exact-rational matrix cases; grades 0..4 vanish and grade 5 matches Xi5."
    )
