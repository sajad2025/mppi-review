# 5. Relative entropy and free energy

This chapter proves one inequality. It relates three quantities: the expected cost under a distribution, the distance of that distribution from a reference, and a "soft minimum" of the cost called the free energy. The inequality is the bridge between optimal control and sampling, and the derivation of MPPI in Chapter 8 is a direct application of it.

## 5.1 Relative entropy

Let $\mathbb{P}$ and $\mathbb{Q}$ be probability distributions with densities $p$ and $q$, where $q(x) = 0$ wherever $p(x) = 0$. The relative entropy, or Kullback–Leibler divergence, of $\mathbb{Q}$ from $\mathbb{P}$ is

$$
\mathrm{KL}(\mathbb{Q} \parallel \mathbb{P}) = \mathbb{E}_{\mathbb{Q}}\Bigl[ \log \frac{q(X)}{p(X)} \Bigr] = \int q(x) \log \frac{q(x)}{p(x)}  dx .
$$

Its two essential properties (Cover and Thomas, Chapter 2) are:

- $\mathrm{KL}(\mathbb{Q} \parallel \mathbb{P}) \ge 0$, with equality if and only if $\mathbb{Q} = \mathbb{P}$. This is Gibbs' inequality, a consequence of Jensen's inequality applied to the concave function $\log$.
- It is not symmetric: in general $\mathrm{KL}(\mathbb{Q} \parallel \mathbb{P}) \neq \mathrm{KL}(\mathbb{P} \parallel \mathbb{Q})$. It is a measure of discrepancy, not a distance.

**Gaussians with equal covariance.** For $\mathbb{Q} = \mathcal{N}(u, \Sigma)$ and $\mathbb{P} = \mathcal{N}(0, \Sigma)$, using the likelihood ratio from Section 4.2,

$$
\log \frac{q(x)}{p(x)} = u^\top \Sigma^{-1} x - \tfrac12 u^\top \Sigma^{-1} u ,
$$

and taking the expectation under $\mathbb{Q}$, where $\mathbb{E}[x] = u$,

$$
\mathrm{KL}(\mathbb{Q} \parallel \mathbb{P}) = \tfrac12  u^\top \Sigma^{-1} u .
$$

The relative entropy between a shifted Gaussian and the unshifted one is a quadratic function of the shift. For independent shifts $u_0, \dots, u_{T-1}$ at successive time steps the divergences add:

$$
\mathrm{KL}(\mathbb{Q} \parallel \mathbb{P}) = \tfrac12 \sum_{t=0}^{T-1} u_t^\top \Sigma^{-1} u_t .
$$

This has the form of a quadratic control cost. That observation is what allows a control problem to be read as a problem about distributions.

## 5.2 Free energy

Let $S(x)$ be a cost function, $\mathbb{P}$ a reference distribution, and $\lambda \gt 0$. The free energy of $S$ with respect to $\mathbb{P}$ at temperature $\lambda$ is

$$
\mathcal{F}(S, \mathbb{P}, \lambda) = -\lambda \log \mathbb{E}_{\mathbb{P}}\Bigl[ \exp\bigl( -S(X)/\lambda \bigr) \Bigr] .
$$

The names come from statistical physics. The free energy is a soft minimum of $S$ over the support of $\mathbb{P}$:

- As $\lambda \to 0$, the expectation is dominated by the smallest values of $S$, and $\mathcal{F} \to \min S$ (the essential infimum of $S$ under $\mathbb{P}$).
- As $\lambda \to \infty$, expanding the exponential gives $\mathcal{F} \to \mathbb{E}_{\mathbb{P}}[S]$, the plain average.
- For every $\lambda$, $\min S \le \mathcal{F} \le \mathbb{E}_{\mathbb{P}}[S]$. The upper bound is Jensen's inequality.

The temperature therefore interpolates between caring only about the best outcome and caring equally about all outcomes.

## 5.3 The variational inequality

**Theorem.** For every distribution $\mathbb{Q}$ that is absolutely continuous with respect to $\mathbb{P}$,

$$
\mathcal{F}(S, \mathbb{P}, \lambda)  \le  \mathbb{E}_{\mathbb{Q}}[S(X)] + \lambda  \mathrm{KL}(\mathbb{Q} \parallel \mathbb{P}) ,
$$

with equality if and only if $\mathbb{Q} = \mathbb{Q}^{\star}$, where

$$
q^{\star}(x) = \frac{1}{\eta}  \exp\bigl( -S(x)/\lambda \bigr)  p(x), \qquad \eta = \mathbb{E}_{\mathbb{P}}\bigl[ \exp(-S(X)/\lambda) \bigr] .
$$

**Proof.** By the definition of $q^{\star}$,

$$
\log \frac{q(x)}{q^{\star}(x)} = \log \frac{q(x)}{p(x)} + \frac{S(x)}{\lambda} + \log \eta .
$$

Take the expectation under $\mathbb{Q}$ and multiply by $\lambda$:

$$
\lambda  \mathrm{KL}(\mathbb{Q} \parallel \mathbb{Q}^{\star}) = \lambda  \mathrm{KL}(\mathbb{Q} \parallel \mathbb{P}) + \mathbb{E}_{\mathbb{Q}}[S] + \lambda \log \eta .
$$

Since $\lambda \log \eta = -\mathcal{F}$, this rearranges to

$$
\mathbb{E}_{\mathbb{Q}}[S] + \lambda  \mathrm{KL}(\mathbb{Q} \parallel \mathbb{P}) = \mathcal{F} + \lambda  \mathrm{KL}(\mathbb{Q} \parallel \mathbb{Q}^{\star}) .
$$

The last term is nonnegative and vanishes exactly when $\mathbb{Q} = \mathbb{Q}^{\star}$. $\blacksquare$

The result is known as the Gibbs variational principle, or the Donsker–Varadhan formula. The identity in the last line of the proof says more than the inequality: the gap between the objective and its minimum is $\lambda$ times the relative entropy from $\mathbb{Q}$ to the optimal distribution. Chapter 8 uses precisely this.

## 5.4 Reading the inequality as a control problem

Consider the right-hand side as an objective to be minimised over $\mathbb{Q}$:

$$
\min_{\mathbb{Q}}    \underbrace{\mathbb{E}_{\mathbb{Q}}[S]}_{\text{expected cost}}   +   \lambda \underbrace{\mathrm{KL}(\mathbb{Q} \parallel \mathbb{P})}_{\text{price of deviating from } \mathbb{P}} .
$$

The first term rewards distributions concentrated on low-cost outcomes. The second penalises moving away from the reference. If $\mathbb{P}$ is the distribution of trajectories when no control is applied and $\mathbb{Q}$ is the distribution under some control, then by Section 5.1 the second term is a quadratic control cost, and the whole objective is a stochastic optimal control problem.

The theorem says three things about this problem:

1. Its optimal value is the free energy, which is an expectation under the uncontrolled distribution $\mathbb{P}$ and can be estimated by simulating the uncontrolled system.
2. Its optimal solution is known explicitly: reweight $\mathbb{P}$ by $\exp(-S/\lambda)$.
3. No optimisation was needed to obtain either.

The optimal distribution $\mathbb{Q}^{\star}$ is a Gibbs, or Boltzmann, distribution. Low-cost outcomes are exponentially more likely under it than high-cost ones, and $\lambda$ sets how sharp the preference is. The weights $\exp(-S_k/\lambda)$ that MPPI assigns to its rollouts are samples of the density ratio $q^{\star}/p$, up to normalisation.

## 5.5 Summary

- Relative entropy measures how far $\mathbb{Q}$ is from $\mathbb{P}$; between Gaussians that differ by a shift of the mean it is quadratic in the shift.
- The free energy $-\lambda \log \mathbb{E}_{\mathbb{P}}[e^{-S/\lambda}]$ is a soft minimum of the cost.
- Expected cost plus $\lambda$ times relative entropy is bounded below by the free energy, with equality for the Gibbs distribution $q^{\star} \propto e^{-S/\lambda} p$.
- This turns a class of optimal control problems into the computation of an expectation.
