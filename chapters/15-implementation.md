# 15. A reference implementation

The folder `code/` contains a complete implementation in about seventy lines of NumPy, written to be read alongside Chapter 9. It is not fast; its purpose is to make every step visible. This chapter walks through it and ends with exercises.

## 15.1 Files

| File | Contents |
|---|---|
| `mppi.py` | the controller |
| `pendulum.py` | swing-up of the pendulum (Sections 1.5 and 9.6) |
| `point_mass.py` | planar point mass avoiding an obstacle (Section 9.4) |
| `lq_example.py` | the linear-quadratic checks of Chapter 10 |
| `importance_sampling.py` | effective sample size versus proposal shift (Section 4.4) |
| `make_figures.py` | regenerates every figure in the book |

Requirements: Python 3.9 or later, NumPy, and Matplotlib for the figures.

```text
cd code
python pendulum.py
python point_mass.py
python lq_example.py
python make_figures.py
```

## 15.2 The interface

The controller is given three functions, each operating on a batch of $K$ rollouts at once:

```python
dynamics(x, v)        # (K, n), (K, m) -> (K, n)    next states
running_cost(x)       # (K, n)         -> (K,)      q(x)
terminal_cost(x)      # (K, n)         -> (K,)      phi(x)
```

Batching is what makes the method fast in practice: the $K$ rollouts advance together, one array operation per time step. On a graphics processor the same code structure applies with the arrays held on the device.

```python
ctrl = MPPI(dynamics, running_cost, terminal_cost, nu=1, horizon=25,
            num_samples=1000, sigma=[[4.0]], lam=1.0, u_min=-5.0, u_max=5.0)
u = ctrl.command(x)      # one MPPI step from state x; returns the control to apply
```

## 15.3 The update, line by line

The body of `command` follows the listing of Section 9.1.

**Sampling.** Noise for all rollouts and all time steps is drawn at once, using the Cholesky factor `L` of $\Sigma$:

```python
eps = self.rng.standard_normal((K, T, m)) @ self.L.T
V = self.U[None, :, :] + eps
```

**Control limits.** The perturbed controls are clipped, and the noise is redefined as what was actually applied:

```python
V = np.clip(V, self.u_min, self.u_max)
eps = V - self.U[None, :, :]
```

**Rollouts and costs.** The loop over time is sequential; each iteration advances all $K$ states:

```python
for t in range(T):
    x = self.F(x, V[:, t])
    S += self.q(x)
    if self.control_cost:
        S += self.lam * np.einsum('i,ij,kj->k', self.U[t], self.Sigma_inv, eps[:, t])
S += self.phi(x)
```

The `einsum` computes $\lambda  u_t^\top \Sigma^{-1} \varepsilon_t^k$ for every $k$. Setting `control_cost=False` gives the variant of Section 10.4, in which any control penalty must be included in the cost functions.

**Weights.** The minimum is subtracted before exponentiating:

```python
beta = S.min()
w = np.exp(-(S - beta) / self.lam)
w /= w.sum()
```

**Update, output and shift.**

```python
self.U = self.U + np.einsum('k,ktm->tm', w, eps)
u0 = self.U[0].copy()
self.U = np.roll(self.U, -1, axis=0)
self.U[-1] = 0.0
```

After each call, `ctrl.last` holds the weights, the costs, the sampled state trajectories and the effective sample size $1/\sum_k w_k^2$ of that step.

## 15.4 Using it on your own system

1. Write `dynamics` so that it accepts arrays of shape `(K, n)` and `(K, m)`. Avoid Python loops over samples.
2. Write the costs. Start with a smooth cost that decreases towards the goal.
3. Choose the parameters by the procedure of Section 11.10, watching `ctrl.last['ess']`.
4. Simulate the closed loop with a different random seed for the controller and, if the real system is noisy, with noise in the simulated plant.

## 15.5 Exercises

**1. Monte Carlo rate.** Estimate $\mathbb{E}[X^2]$ for $X \sim \mathcal{N}(0,1)$ with $K = 10, 100, \dots, 10^6$ samples, repeating each 100 times. Plot the root-mean-square error against $K$ on logarithmic axes and confirm the slope $-1/2$.

**2. Weight degeneracy in dimension.** Repeat the experiment of Section 4.4 in $d$ dimensions with a shift of 0.5 in every coordinate. Plot $K_{\mathrm{eff}}/K$ against $d$ and compare with $e^{-d/4}$.

**3. Free energy limits.** For $S$ uniformly distributed on $[0, 1]$, compute $-\lambda \log \mathbb{E}[e^{-S/\lambda}]$ in closed form and verify the limits $\lambda \to 0$ and $\lambda \to \infty$ of Section 5.2.

**4. Gaussian relative entropy.** Derive $\mathrm{KL}(\mathcal{N}(u, \Sigma) \parallel \mathcal{N}(0, \Sigma)) = \tfrac12 u^\top \Sigma^{-1} u$ directly from the definition.

**5. The two variants.** In `lq_example.py`, run several iterations of the likelihood-ratio variant from a starting sequence far from the optimum, with $K = 1000$. Explain why the first iteration is poor and later ones are better, using Section 4.4.

**6. Temperature and control cost.** In `pendulum.py`, increase $\lambda$ to 3 and the noise variance to 12, keeping their ratio fixed. Does the swing-up succeed? Relate the outcome to $R = \lambda \Sigma^{-1}$.

**7. Smoothing.** Add a moving-average filter over the time axis to the nominal sequence after the update. Measure the effect on the chatter and on the final error for the pendulum.

**8. Predictive sampling.** Replace the weighted average by the single best sample, and include the unperturbed nominal sequence among the candidates. Compare with MPPI on the pendulum for $K = 100$ and $K = 1000$.

**9. Cross-entropy method.** Replace the weights by $1/K_e$ on the $K_e = K/10$ best samples and refit a diagonal covariance to them at each step. Compare.

**10. Disturbances.** Add a constant torque disturbance to the simulated pendulum but not to the model. How large can it be before balancing fails? What does this suggest about the methods of Section 13.1?

**11. The multimodal case.** In `point_mass.py`, place the obstacle exactly on the line between start and goal. Run with several seeds and record which side the controller chooses and whether it ever collides.

**12. A terminal cost from LQR.** Linearise the pendulum about the upright position, solve the Riccati equation for the same running cost, and use $x^\top P x$ as the terminal cost near upright. How short can the horizon be made?
