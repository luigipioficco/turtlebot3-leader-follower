# Pure geometric helpers used by the nodes.
#
# They live here, outside the nodes, for two reasons: they are the only parts
# of the system where a mistake produces no crash and no error message, only
# silently wrong behaviour, and keeping them ROS-free makes them testable
# without starting rclpy.

import math

import numpy as np


def estimate_velocity(history, min_samples=3, min_span=1e-3):
    # Target velocity, as the slope of a least-squares fit over the window.
    #
    # A finite difference over the last two samples is unbiased but its
    # variance is huge: it subtracts two noisy numbers and divides by a small
    # interval, which amplifies the noise instead of averaging it out. Fitting
    # a line through the whole window lets the errors cancel against each other.
    if len(history) < min_samples:
        return (0.0, 0.0)
    arr = np.asarray(history, dtype=float)
    t = arr[:, 0] - arr[0, 0]
    if t[-1] - t[0] < min_span:
        return (0.0, 0.0)
    return (float(np.polyfit(t, arr[:, 1], 1)[0]),
            float(np.polyfit(t, arr[:, 2], 1)[0]))


def predict_position(history, velocity, now, min_speed=0.03):
    # Constant-velocity extrapolation of the target position.
    #
    # The simplest classical tracking model, and the one whose assumptions are
    # least likely to be violated here. A model that also estimated the turn
    # rate would in principle do better, but the turn rate has to come from the
    # same short, noisy window, and it becomes unobservable at exactly the
    # moment the target leaves the field of view: when it would be needed.
    if not history:
        return None
    t_last, x_last, y_last = history[-1]
    vx, vy = velocity
    # On a stationary target the residual measurement noise produces a small
    # apparent velocity. Without this threshold the follower would extrapolate
    # that noise and drift away from a target that has not moved at all.
    if math.hypot(vx, vy) < min_speed:
        vx = vy = 0.0
    dt = now - t_last
    return (x_last + vx * dt, y_last + vy * dt)
