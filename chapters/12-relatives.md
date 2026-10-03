# 12. MPPI as an optimiser, and its relatives

MPPI is one member of a family of methods that improve a control sequence by sampling perturbations of it. This chapter places it in that family. The comparison also gives a second way to understand the update, as a gradient step on a smoothed cost.

## 12.1 The update as a gradient step

Consider the variant with weights $\exp(-S(U + \mathcal{E})/\lambda)$ and no likelihood-ratio term (Section 10.4, with any control cost included in $S$). Define the smoothed cost

$$
J_\lambda(U) = -\lambda \log \mathbb{E}_{\mathcal{E}}\Bigl[ \exp\bigl( -S(U + \mathcal{E})/\lambda \bigr) \Bigr], \qquad \mathcal{E} \sim \mathcal{N}(0, \bar\Sigma).
$$

This is the free energy of Chapter 5 with the reference distribution centred at $U$: a soft minimum of the cost over a Gaussian neighbourhood of $U$. Write the expectation as an integral over $V = U + \mathcal{E}$ against the density $\mathcal{N}(V; U, \bar\Sigma)$ and differentiate with respect to $U$. Since $\nabla_U  \mathcal{N}(V; U, \bar\Sigma) = \bar\Sigma^{-1} (V - U)  \mathcal{N}(V; U, \bar\Sigma)$,

$$
\nabla J_\lambda(U) = -\lambda  \bar\Sigma^{-1}  \frac{\mathbb{E}\bigl[ \mathcal{E}  e^{-S(U + \mathcal{E})/\lambda} \bigr]}{\mathbb{E}\bigl[ e^{-S(U + \mathcal{E})/\lambda} \bigr]} .
$$

The ratio on the right is the infinite-sample MPPI update. Therefore

$$
U_{\mathrm{new}} = U - \frac{1}{\lambda}  \bar\Sigma  \nabla J_\lambda(U).
$$

The MPPI update is a gradient descent step on the smoothed cost $J_\lambda$, with step size $1/\lambda$ and preconditioner $\bar\Sigma$. Three consequences:

- **No derivatives of $S$ are needed, and $S$ need not have any.** The Gaussian smoothing makes $J_\lambda$ differentiable even when $S$ is discontinuous, and the gradient of the smoothed function is computed from function values alone.
- **The method is local.** It follows the gradient of a smoothed landscape. The smoothing radius is set by $\Sigma$, which determines how wide a valley has to be to be noticed.
- **The contraction of Section 10.4 is recovered.** For a quadratic cost with Hessian $A_J$, the smoothed cost is quadratic with Hessian $A_J (I + \bar\Sigma A_J/\lambda)^{-1}$, and the gradient step above reproduces the factor $(I + \bar\Sigma A_J/\lambda)^{-1}$.

Views of MPPI as a first-order method are developed by Wagener et al. (2019), who derive it and several relatives from online mirror descent.

## 12.2 The two limits of the temperature

- As $\lambda \to 0$, all weight goes to the single lowest-cost sample, and the update replaces the nominal sequence by that sample.
- As $\lambda \to \infty$, the weights become equal, the update is the plain average of the noise, which tends to zero, and the nominal sequence does not move.

MPPI interpolates between "take the best" and "take them all".

## 12.3 Predictive sampling

The $\lambda \to 0$ limit is a method in its own right: sample perturbations of the nominal sequence, simulate, keep the best one. It is sometimes called random shooting; in the form used in the MuJoCo MPC software, with the nominal sequence represented by a spline and included among the candidates, it is called predictive sampling (Howell et al., 2022). Because the unperturbed nominal is a candidate, the cost of the chosen plan never increases for a fixed problem. It is simple and a useful baseline. It discards the information in all samples but one.

## 12.4 The cross-entropy method

The cross-entropy method (Rubinstein, 1999; de Boer et al., 2005) maintains a Gaussian over control sequences with mean $\mu$ and covariance $C$, and iterates:

1. Draw $K$ sequences from $\mathcal{N}(\mu, C)$ and evaluate their costs.
2. Select the $K_e$ lowest-cost samples, the elites.
3. Set $\mu$ and $C$ to the mean and covariance of the elites.

In the language of weights, the cross-entropy method gives weight $1/K_e$ to each elite and zero to the rest: a hard threshold on rank, where MPPI uses a soft function of cost. Two differences matter in practice. The cross-entropy method adapts the covariance, shrinking it as the samples agree, while standard MPPI keeps $\Sigma$ fixed. And because it uses ranks, it is unaffected by the scale of the cost and has no temperature to tune; the elite fraction plays that role. It is the usual planner in model-based reinforcement learning (Chua et al., 2018), and refinements for real-time use include time-correlated noise and reuse of elites across steps (Pinneri et al., 2020).

## 12.5 Evolution strategies

The covariance matrix adaptation evolution strategy, CMA-ES (Hansen, 2016), is the most developed method of this kind for general black-box optimisation. It combines rank-based weighted recombination of the best samples with an update of the full covariance matrix that accumulates information over iterations, and it adapts the overall step size separately. It needs more iterations than a control loop usually allows, but the ideas, particularly covariance adaptation, recur in the MPPI variants of Chapter 13.

## 12.6 One template

All of these methods fit a common pattern:

1. Maintain a distribution over control sequences.
2. Sample from it and evaluate each sample by a rollout.
3. Assign weights to the samples as a function of their costs.
4. Update the distribution towards the weighted samples.

| Method | Weights | Mean update | Covariance |
|---|---|---|---|
| Predictive sampling | 1 for the best sample | replace by best | fixed |
| Cross-entropy method | equal on the elites | mean of elites | refit to elites |
| MPPI | proportional to $\exp(-S/\lambda)$ | weighted mean | fixed |
| CMA-ES | decreasing in rank | weighted mean of best | adapted with memory |

Several authors have made the template precise. Wagener et al. (2019) obtain these methods as instances of online mirror descent with different utility functions. Okada and Taniguchi (2019) and Lambert et al. (2020) treat control as Bayesian inference over sequences, with MPPI as the case of a Gaussian approximation with fixed covariance.

## 12.7 Against derivative-based methods

| | Sampling (MPPI and relatives) | Derivative-based (iLQR, SQP) |
|---|---|---|
| Needs from the model | evaluation only | derivatives |
| Non-smooth costs and dynamics | handled | problematic |
| Accuracy near a solution | limited by Monte Carlo error | high |
| Scaling with $m \times T$ | sample count grows | polynomial |
| Parallelism | across samples | limited |
| Constraints | penalties, clipping | handled directly |

The two families are complementary. Sampling is strong where derivatives are missing or misleading and the control dimension is moderate. Derivative-based methods are strong where the problem is smooth and precision matters. Hybrids that use a sampled plan to initialise a derivative-based refinement, or a derivative-based feedback law to shape the samples, are an active area.

## 12.8 Summary

- Without the likelihood-ratio term, the MPPI update is a preconditioned gradient step on the Gaussian-smoothed cost.
- Low temperature gives predictive sampling; the cross-entropy method replaces soft weights by a hard elite threshold and adapts the covariance.
- These methods share a template of sampling, weighting and refitting.
