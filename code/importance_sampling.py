"""Importance sampling with a shifted Gaussian proposal (Chapter 4).

Target p = N(0, 1), proposal q = N(delta, 1). The weights are w(x) = p(x)/q(x) and
E_q[w^2] = exp(delta^2), so the effective sample size is about K * exp(-delta^2)."""
import numpy as np


def ess_fraction(delta, K=2000, reps=200, seed=0):
    rng = np.random.default_rng(seed)
    out = []
    for _ in range(reps):
        x = delta + rng.standard_normal(K)
        logw = -0.5 * x ** 2 + 0.5 * (x - delta) ** 2
        w = np.exp(logw - logw.max()); w /= w.sum()
        out.append(1.0 / np.sum(w ** 2) / K)
    return float(np.mean(out))


if __name__ == "__main__":
    for d in [0.0, 0.5, 1.0, 1.5, 2.0, 3.0]:
        print("shift %.1f: ESS/K simulated %.4f, exp(-delta^2) = %.4f" % (d, ess_fraction(d), np.exp(-d ** 2)))
