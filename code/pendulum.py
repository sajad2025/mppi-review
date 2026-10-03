"""Pendulum swing-up with MPPI. The angle theta is measured from the upright position."""
import numpy as np
from mppi import MPPI

G, LEN, MASS, DAMP, DT, U_MAX = 9.81, 1.0, 1.0, 0.1, 0.05, 5.0


def wrap(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


def dynamics(x, u):
    """Batched Euler step. x: (K, 2) = (theta, theta_dot), u: (K, 1)."""
    th, om = x[:, 0], x[:, 1]
    acc = (G / LEN) * np.sin(th) - DAMP * om + u[:, 0] / (MASS * LEN ** 2)
    om_next = om + DT * acc
    return np.stack([th + DT * om_next, om_next], axis=1)


def running_cost(x):
    return wrap(x[:, 0]) ** 2 + 0.1 * x[:, 1] ** 2


def terminal_cost(x):
    return 10.0 * (wrap(x[:, 0]) ** 2 + 0.1 * x[:, 1] ** 2)


def run(num_samples=1000, lam=1.0, noise_var=4.0, horizon=25, steps=160, seed=0):
    ctrl = MPPI(dynamics, running_cost, terminal_cost, nu=1, horizon=horizon,
                num_samples=num_samples, sigma=[[noise_var]], lam=lam,
                u_min=-U_MAX, u_max=U_MAX, seed=seed)
    x = np.array([np.pi, 0.0])                                      # hanging straight down
    xs, us, ess = [x.copy()], [], []
    for _ in range(steps):
        u = ctrl.command(x)
        x = dynamics(x[None, :], u[None, :])[0]
        xs.append(x.copy()); us.append(u.copy()); ess.append(ctrl.last['ess'])
    return np.array(xs), np.array(us), np.array(ess)


if __name__ == "__main__":
    xs, us, ess = run()
    print("final angle from upright [rad]: %.4f" % wrap(xs[-1, 0]))
    print("final angular velocity [rad/s]: %.4f" % xs[-1, 1])
    print("mean effective sample size    : %.1f of 1000" % ess.mean())
