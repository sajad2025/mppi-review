"""A planar point mass steering around a circular obstacle with MPPI (Chapters 9 and 11)."""
import numpy as np
from mppi import MPPI

DT = 0.1
GOAL = np.array([4.0, 0.0])
OBSTACLE, RADIUS = np.array([2.0, 0.15]), 0.8


def dynamics(x, u):
    """x = (px, py, vx, vy), u = acceleration (ax, ay)."""
    v = x[:, 2:] + DT * u
    return np.concatenate([x[:, :2] + DT * v, v], axis=1)


def running_cost(x):
    dist = np.linalg.norm(x[:, :2] - GOAL, axis=1)
    hit = np.linalg.norm(x[:, :2] - OBSTACLE, axis=1) < RADIUS
    return dist ** 2 + 0.05 * np.sum(x[:, 2:] ** 2, axis=1) + 1000.0 * hit


def terminal_cost(x):
    return 20.0 * np.sum((x[:, :2] - GOAL) ** 2, axis=1) + 2.0 * np.sum(x[:, 2:] ** 2, axis=1)


def make_controller(num_samples=1000, lam=60.0, seed=0):
    return MPPI(dynamics, running_cost, terminal_cost, nu=2, horizon=30, num_samples=num_samples,
                sigma=6.0 * np.eye(2), lam=lam, u_min=-3.0, u_max=3.0, seed=seed)


def run(steps=100, **kw):
    ctrl = make_controller(**kw)
    x = np.zeros(4); path = [x.copy()]; first = None
    for k in range(steps):
        u = ctrl.command(x)
        if k == 0:
            first = dict(ctrl.last)
        x = dynamics(x[None, :], u[None, :])[0]; path.append(x.copy())
    return np.array(path), first


if __name__ == "__main__":
    path, first = run()
    clear = np.min(np.linalg.norm(path[:, :2] - OBSTACLE, axis=1)) - RADIUS
    print("final distance to goal: %.3f, smallest clearance from the obstacle: %.3f, ESS at the first step: %.1f"
          % (np.linalg.norm(path[-1, :2] - GOAL), clear, first['ess']))
