"""Regenerates every figure in chapters/figures. Run from the code/ directory."""
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pendulum, point_mass, lq_example, importance_sampling

OUT = "../chapters/figures/"
INK, BLUE, GREY, RED = "#22262b", "#2c5f7c", "#9aa3ab", "#b4532a"
plt.rcParams.update({"font.family": "serif", "font.size": 10.5, "axes.spines.top": False,
                     "axes.spines.right": False, "axes.edgecolor": INK, "axes.labelcolor": INK,
                     "xtick.color": INK, "ytick.color": INK, "figure.dpi": 160, "savefig.bbox": "tight"})

# 1. importance sampling: effective sample size versus proposal shift
d = np.linspace(0, 3, 13)
sim = [importance_sampling.ess_fraction(x, K=2000, reps=100) for x in d]
fig, ax = plt.subplots(figsize=(5.2, 3.2))
ax.semilogy(d, np.exp(-d ** 2), color=BLUE, lw=2, label=r"$e^{-\delta^2}$ (large-$K$ limit)")
ax.semilogy(d, sim, "o", color=INK, ms=4, label="simulated, $K = 2000$")
ax.axhline(1 / 2000, color=GREY, lw=1, ls=":"); ax.text(0.05, 1 / 2000 * 1.25, "one sample", color=GREY, fontsize=9)
ax.set_xlabel(r"shift $\delta$ between proposal and target mean"); ax.set_ylabel("effective sample size / $K$")
ax.legend(frameon=False); fig.savefig(OUT + "ess-vs-shift.png"); plt.close(fig)

# 2. sampled rollouts, weights and the weighted average (point mass)
path, first = point_mass.run()
w, st = first["weights"], first["states"]
fig, ax = plt.subplots(figsize=(6.0, 3.6))
ax.add_patch(plt.Circle(point_mass.OBSTACLE, point_mass.RADIUS, color="#d9d3c7", zorder=1))
order = np.argsort(w)[::2]
for k in order:
    a = 0.06 + 0.9 * (w[k] / w.max())
    ax.plot(st[k, :, 0], st[k, :, 1], color=BLUE if w[k] > 0.2 * w.max() else GREY, lw=0.7, alpha=min(a, 1), zorder=2)
ax.plot(path[:, 0], path[:, 1], color=RED, lw=2.2, zorder=4, label="closed-loop path")
ax.plot(*point_mass.GOAL, marker="*", color=INK, ms=11, zorder=5); ax.plot(0, 0, "o", color=INK, ms=5, zorder=5)
ax.text(0.0, -0.33, "start", ha="center", fontsize=9); ax.text(4.0, -0.38, "goal", ha="center", fontsize=9)
ax.set_aspect("equal"); ax.set_xlim(-0.5, 4.8); ax.set_ylim(-1.9, 1.9); ax.set_xlabel("$p_x$"); ax.set_ylabel("$p_y$")
ax.legend(frameon=False, loc="upper left"); fig.savefig(OUT + "rollouts.png"); plt.close(fig)

# 3. pendulum swing-up
xs, us, ess = pendulum.run()
t = np.arange(len(us)) * pendulum.DT
fig, axs = plt.subplots(3, 1, figsize=(5.6, 5.2), sharex=True)
axs[0].plot(t, np.abs(pendulum.wrap(xs[:-1, 0])), color=BLUE, lw=1.8); axs[0].set_ylabel(r"$|\theta|$ from upright [rad]")
axs[1].plot(t, us[:, 0], color=INK, lw=1.2); axs[1].set_ylabel("torque $u$")
axs[2].semilogy(t, ess, color=RED, lw=1.2); axs[2].set_ylabel("effective samples"); axs[2].set_xlabel("time [s]")
fig.align_ylabels(axs); fig.savefig(OUT + "pendulum.png"); plt.close(fig)

# 4. linear-quadratic example: measured and predicted error per iteration
T, lam = 8, 1.0
M, bfun = lq_example.build(T); b = bfun(np.array([1.0, 0.0])); Sigma = 4.0 * np.eye(T)
Rbar = 0.25 * np.eye(T); Aj = M + Rbar; Uj = -np.linalg.solve(Aj, b)
C = np.linalg.inv(np.eye(T) + Sigma @ Aj / lam)
fig, ax = plt.subplots(figsize=(5.2, 3.3))
pred = [np.linalg.norm(np.linalg.matrix_power(C, i) @ (-Uj)) for i in range(11)]
ax.semilogy(range(11), pred, color=BLUE, lw=2, label="infinite-sample prediction")
for K, mk in [(100, "s"), (1000, "^"), (100000, "o")]:
    rng = np.random.default_rng(1); U = np.zeros(T); err = [np.linalg.norm(U - Uj)]
    for it in range(10):
        U, _ = lq_example.mppi_iteration(U, M, b, Sigma, lam, K, rng, 2, Rbar); err.append(np.linalg.norm(U - Uj))
    ax.semilogy(range(11), err, mk, color=INK, ms=4, mfc="none" if K < 1e5 else INK, label="$K = %d$" % K)
ax.set_xlabel("iteration"); ax.set_ylabel(r"$\Vert U - U^\circ \Vert$"); ax.legend(frameon=False)
fig.savefig(OUT + "lq-contraction.png"); plt.close(fig)

# 5. temperature sweep on the pendulum
lams = [0.03, 0.1, 0.3, 1.0, 3.0, 10.0]; err, es = [], []
for lam in lams:
    r = [pendulum.run(lam=lam, seed=s) for s in range(3)]
    err.append(np.mean([np.abs(pendulum.wrap(x[-40:, 0])).mean() for x, _, _ in r]))
    es.append(np.mean([e.mean() for _, _, e in r]))
fig, ax = plt.subplots(figsize=(5.2, 3.3)); ax2 = ax.twinx(); ax2.spines["right"].set_visible(True)
ax.loglog(lams, err, "o-", color=BLUE, lw=1.8); ax2.loglog(lams, es, "s--", color=RED, lw=1.4)
ax.set_xlabel(r"temperature $\lambda$"); ax.set_ylabel("final angle error [rad]", color=BLUE)
ax2.set_ylabel("mean effective samples", color=RED)
fig.savefig(OUT + "temperature.png"); plt.close(fig)
print("temperature sweep:", [(l, round(e, 4), round(s, 1)) for l, e, s in zip(lams, err, es)])
