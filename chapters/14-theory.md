# 14. What is proved about MPPI

MPPI works well in practice and was adopted long before its behaviour was understood mathematically. This chapter summarises what has been established and what has not, as of 2026. It separates three questions that are often run together.

## 14.1 Three questions

**Is the target correct?** With infinitely many samples the update returns the mean of the optimal distribution, $\mathbb{E}_{\mathbb{Q}^{\star}}[V]$. Is that the right control?

**How accurate is one update?** With $K$ samples, how far is the computed sequence from that mean?

**Is the closed loop stable?** When the update is applied repeatedly in a receding horizon with finitely many samples, does the system behave well?

## 14.2 The target

For the path integral class of Chapter 7, with control-affine dynamics, noise in the control channel and $R = \lambda \Sigma^{-1}$, the answer is exact in continuous time: the weighted average of the noise is the optimal feedback control at the current state.

In the discrete-time setting of Chapter 8, the free energy is a lower bound on the control objective, attained by a distribution that the controller generally cannot realise, and the control sequence is obtained by projecting that distribution onto the Gaussian family. For linear dynamics and quadratic cost the projection loses nothing and the target is the optimal open-loop sequence (Chapter 10). For general problems it is an approximation whose quality depends on how close the optimal distribution is to a Gaussian of covariance $\Sigma$.

For the variant with an explicit control cost, Homburger et al. (2025) characterise how far the infinite-sample result is from the solution of the deterministic and stochastic optimal control problems, in terms of the temperature and the noise covariance; the discrepancy vanishes as these tend to zero and does not vanish as the number of samples grows.

## 14.3 One update with finitely many samples

The update is a self-normalised importance sampling estimate, so the general theory of Chapter 4 applies: the error is of order $K^{-1/2}$, with a constant governed by the second moment of the weights.

Yoon et al. (2022) bound the variance of the path integral estimate and derive the corresponding sample complexity for trajectory optimisation.

Yi et al. (2024) analyse the update as an optimisation step. For quadratic costs they prove that the expected iterate contracts towards the optimum at a linear rate, the rate of Section 10.4, and they use the result to derive an optimal sampling covariance. They extend the analysis to costs that are close to quadratic.

Two limitations of results of this kind should be kept in mind. The constants hide the dependence on dimension, which Sections 4.4 and 10.6 showed to be exponential in unfavourable cases. And the bounds describe the regime in which the effective sample size is large, whereas working controllers often run with a small one, in which the estimate behaves more like a selection of the best sample than like an average.

## 14.4 Closed-loop stability

Classical MPC theory proves stability by showing that the optimal cost decreases along closed-loop trajectories (Section 3.3). The theory extends to inexact optimisation: Scokaert, Mayne and Rawlings (1999) showed that it suffices for each step to produce a feasible sequence whose cost is lower than that of the shifted previous one, without being optimal. Derivative-based real-time schemes are analysed in this way.

A sampling-based update does not guarantee a cost decrease at each step. It is random, and with some probability a step makes the plan worse. A stability statement for MPPI therefore has to be probabilistic: stability in expectation, or with high probability over a finite time.

Results of this kind are recent. For Robust MPPI, Gandhi et al. (2021) bound the growth of the free energy, which gives a performance guarantee in terms of the tracking controller and the sampling error. In two preprints, Yoon and Kim (2026) treat finite-sample MPPI as a random perturbation of an ideal controller: for linear systems with quadratic cost and a Riccati terminal cost, where the ideal controller is the linear-quadratic regulator, and for nonlinear systems, assuming the ideal MPC law has a control Lyapunov function and a contraction property. They prove practical stability, meaning convergence to a neighbourhood of the origin, in expectation, with an error that separates into a Monte Carlo part decreasing in $K$ and a temperature part that does not. The sample counts that their theorems require are, by the authors' own account, extremely conservative compared with what works in simulation. These preprints had not been peer reviewed at the time of writing.

A tutorial survey by Honda (2025) reviews the probabilistic-inference view of MPC and describes closed-loop stability guarantees for path integral MPC as largely open.

## 14.5 Open problems

- **Guarantees in the practical regime.** Bounds that hold for the sample counts actually used, where the weights are concentrated on a few samples.
- **Tight constants.** Existing sufficient conditions are far from necessary.
- **Nonlinear and constrained problems** without assuming that the ideal controller already has a stability certificate.
- **The best-sample limit.** Predictive sampling and the cross-entropy method select instead of averaging, and their error does not follow the $K^{-1/2}$ law.
- **Choice of parameters with guarantees.** Rules for $\lambda$, $\Sigma$, $T$ and $K$ that come with a statement about closed-loop behaviour.

## 14.6 Summary

- The MPPI target is exactly optimal for the path integral class and for linear-quadratic problems, and an approximation otherwise.
- One update has error of order $K^{-1/2}$, with constants that can be very large.
- Closed-loop stability results with finitely many samples began to appear only recently and are conservative.
