"""A minimal reference implementation of Model Predictive Path Integral control (MPPI).

It follows the information-theoretic formulation of Williams et al. (IEEE T-RO, 2018):

    dynamics      x_{t+1} = F(x_t, v_t),      v_t = u_t + eps_t,   eps_t ~ N(0, Sigma)
    state cost    S(V) = phi(x_T) + sum_t q(x_t)
    weights       w_k  proportional to  exp(-(1/lam) * [S_k + lam * sum_t u_t' Sigma^{-1} eps_t^k])
    update        u_t <- u_t + sum_k w_k eps_t^k

Only NumPy is required. All K rollouts are propagated together, so `dynamics`, `running_cost`
and `terminal_cost` must accept a batch: states of shape (K, n) and controls of shape (K, m).
"""
import numpy as np


class MPPI:
    def __init__(self, dynamics, running_cost, terminal_cost, nu, horizon, num_samples,
                 sigma, lam, u_min=None, u_max=None, control_cost=True, seed=0):
        """
        dynamics(x, v)      -> next state, shapes (K, n), (K, m) -> (K, n)
        running_cost(x)     -> state cost q(x), shape (K,)
        terminal_cost(x)    -> terminal cost phi(x), shape (K,)
        nu                  control dimension m
        horizon             number of time steps T
        num_samples         number of rollouts K
        sigma               (m, m) covariance of the control noise
        lam                 temperature lambda > 0
        u_min, u_max        optional elementwise control limits
        control_cost        if True, include the term lam * u' Sigma^{-1} eps (implicit quadratic
                            control cost with R = lam * Sigma^{-1}); if False, weights use S only
        """
        self.F, self.q, self.phi = dynamics, running_cost, terminal_cost
        self.m, self.T, self.K = nu, horizon, num_samples
        self.Sigma = np.atleast_2d(np.asarray(sigma, dtype=float))
        self.Sigma_inv = np.linalg.inv(self.Sigma)
        self.L = np.linalg.cholesky(self.Sigma)
        self.lam = float(lam)
        self.u_min, self.u_max = u_min, u_max
        self.control_cost = control_cost
        self.rng = np.random.default_rng(seed)
        self.U = np.zeros((self.T, self.m))          # nominal control sequence u_0, ..., u_{T-1}
        self.last = {}                               # diagnostics of the most recent call

    def command(self, x0):
        """One MPPI iteration from the current state x0. Returns the control to apply now."""
        K, T, m = self.K, self.T, self.m
        eps = self.rng.standard_normal((K, T, m)) @ self.L.T        # eps_t^k ~ N(0, Sigma)
        V = self.U[None, :, :] + eps                                # perturbed controls v_t^k
        if self.u_min is not None or self.u_max is not None:
            V = np.clip(V, self.u_min, self.u_max)
            eps = V - self.U[None, :, :]                            # noise actually applied
        x = np.tile(np.asarray(x0, dtype=float), (K, 1))
        S = np.zeros(K)
        states = np.empty((K, T + 1, x.shape[1])); states[:, 0] = x
        for t in range(T):
            x = self.F(x, V[:, t])
            S += self.q(x)
            if self.control_cost:
                S += self.lam * np.einsum('i,ij,kj->k', self.U[t], self.Sigma_inv, eps[:, t])
            states[:, t + 1] = x
        S += self.phi(x)
        beta = S.min()                                              # subtract the minimum for
        w = np.exp(-(S - beta) / self.lam)                          # numerical stability
        w /= w.sum()
        self.U = self.U + np.einsum('k,ktm->tm', w, eps)            # weighted average of the noise
        if self.u_min is not None or self.u_max is not None:
            self.U = np.clip(self.U, self.u_min, self.u_max)
        u0 = self.U[0].copy()
        self.last = dict(weights=w, costs=S, states=states, ess=1.0 / np.sum(w ** 2))
        self.U = np.roll(self.U, -1, axis=0)                        # receding horizon: shift
        self.U[-1] = 0.0                                            # and re-initialise the tail
        return u0

    def reset(self):
        self.U[:] = 0.0
