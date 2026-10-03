# 8. The information-theoretic derivation

This chapter derives the MPPI update law in discrete time, for arbitrary dynamics, using only the inequality of Chapter 5 and importance sampling from Chapter 4. It follows Williams et al. (2018). No stochastic calculus is needed.

## 8.1 Setting

The system is

$$
x_{t+1} = F(x_t, v_t), \qquad v_t \sim \mathcal{N}(u_t, \Sigma), \qquad t = 0, \dots, T-1 ,
$$

with $F$ any function that can be evaluated. The controller chooses the means $u_t$; the system receives $v_t = u_t + \varepsilon_t$ with $\varepsilon_t \sim \mathcal{N}(0, \Sigma)$ independent across time. Stack the sequences as $U$, $V$ and $\mathcal{E}$.

Given the initial state $x_0$, the input sequence $V$ determines the trajectory, so the state cost is a function of $V$:

$$
S(V) = \phi(x_T) + \sum_{t=1}^{T-1} q(x_t).
$$

Two distributions over input sequences are needed.

- The **base distribution** $\mathbb{P}$: no control, $v_t \sim \mathcal{N}(0, \Sigma)$. Its density is $p(V) \propto \exp\bigl(-\tfrac12 \sum_t v_t^\top \Sigma^{-1} v_t\bigr)$.
- The **controlled distribution** $\mathbb{Q}_U$: $v_t \sim \mathcal{N}(u_t, \Sigma)$, with density $q(V \mid U) \propto \exp\bigl(-\tfrac12 \sum_t (v_t - u_t)^\top \Sigma^{-1} (v_t - u_t)\bigr)$.

## 8.2 The control problem behind the free energy

Apply the variational inequality of Section 5.3 with $\mathbb{Q} = \mathbb{Q}_U$, and use the Gaussian formula of Section 5.1 for the relative entropy:

$$
-\lambda \log \mathbb{E}_{\mathbb{P}}\bigl[ e^{-S(V)/\lambda} \bigr]  \le  \mathbb{E}_{\mathbb{Q}_U}\bigl[ S(V) \bigr] + \frac{\lambda}{2} \sum_{t=0}^{T-1} u_t^\top \Sigma^{-1} u_t .
$$

The right-hand side is the objective of a stochastic optimal control problem: expected state cost plus a quadratic control cost with weight matrix

$$
R = \lambda  \Sigma^{-1}.
$$

This is the same relation between control cost and noise that Chapter 7 had to assume. Here it is not an assumption; it is what the relative entropy between shifted Gaussians happens to be. The left-hand side is a lower bound on the cost achievable by any control sequence.

## 8.3 The optimal distribution

The lower bound is attained by the Gibbs distribution

$$
q^{\star}(V) = \frac{1}{\eta} \exp\Bigl( -\frac{1}{\lambda} S(V) \Bigr)  p(V), \qquad \eta = \mathbb{E}_{\mathbb{P}}\bigl[ e^{-S(V)/\lambda} \bigr].
$$

In general $\mathbb{Q}^{\star}$ is not one of the distributions $\mathbb{Q}_U$. It need not be Gaussian, its covariance is not $\Sigma$, and it may have several modes, one for each distinct way of doing the task well. The controller can only shift the mean of a Gaussian, so it cannot produce $\mathbb{Q}^{\star}$ exactly.

The approach is to choose the control sequence whose distribution is as close as possible to the optimal one:

$$
U^{\star} = \arg\min_U   \mathrm{KL}\bigl( \mathbb{Q}^{\star} \parallel \mathbb{Q}_U \bigr).
$$

## 8.4 Solving for the control

Write out the divergence and drop everything that does not depend on $U$:

$$
\mathrm{KL}\bigl( \mathbb{Q}^{\star} \parallel \mathbb{Q}_U \bigr)
= \mathbb{E}_{\mathbb{Q}^{\star}}\bigl[ \log q^{\star}(V) \bigr] - \mathbb{E}_{\mathbb{Q}^{\star}}\bigl[ \log q(V \mid U) \bigr]
= \text{const} + \tfrac12  \mathbb{E}_{\mathbb{Q}^{\star}}\Bigl[ \sum_{t} (v_t - u_t)^\top \Sigma^{-1} (v_t - u_t) \Bigr].
$$

This is a convex quadratic in each $u_t$. Setting the gradient to zero,

$$
u_t^{\star} = \mathbb{E}_{\mathbb{Q}^{\star}}[ v_t ] .
$$

The best control sequence is the mean of the optimal distribution. This is the general fact that the Gaussian of fixed covariance closest to a given distribution, in this direction of the divergence, is the one with the same mean.

**A remark on the direction of the divergence.** By the identity in the proof of Section 5.3, minimising the control objective of Section 8.2 over $U$ is the same as minimising $\mathrm{KL}(\mathbb{Q}_U \parallel \mathbb{Q}^{\star})$, with the arguments in the other order. MPPI minimises $\mathrm{KL}(\mathbb{Q}^{\star} \parallel \mathbb{Q}_U)$ instead, because that problem has the closed-form solution just found. The two coincide whenever $\mathbb{Q}^{\star}$ is Gaussian, as in the linear-quadratic case of Chapter 10, and differ otherwise. The choice made by MPPI averages over all the modes of $\mathbb{Q}^{\star}$. When the modes are far apart, for example passing an obstacle on the left or on the right, the average can fall between them, which is a known failure mode discussed in Chapter 11.

## 8.5 Importance sampling around the current sequence

The expectation $\mathbb{E}_{\mathbb{Q}^{\star}}[v_t]$ cannot be computed by sampling from $\mathbb{Q}^{\star}$ directly. What the controller can sample from is $\mathbb{Q}_{\hat U}$, the Gaussian centred at its current control sequence $\hat U$. By the importance sampling identity,

$$
u_t^{\star} = \mathbb{E}_{\mathbb{Q}_{\hat U}}\bigl[ w(V)  v_t \bigr], \qquad
w(V) = \frac{q^{\star}(V)}{q(V \mid \hat U)} = \frac{1}{\eta}  e^{-S(V)/\lambda}  \frac{p(V)}{q(V \mid \hat U)} .
$$

The last factor is the Gaussian likelihood ratio of Section 4.2, applied at every time step. Writing $v_t = \hat u_t + \varepsilon_t$,

$$
\frac{p(V)}{q(V \mid \hat U)} = \exp\Bigl( -\sum_{t=0}^{T-1} \bigl( \hat u_t^\top \Sigma^{-1} \varepsilon_t + \tfrac12 \hat u_t^\top \Sigma^{-1} \hat u_t \bigr) \Bigr).
$$

Therefore

$$
w(\mathcal{E}) = \frac{1}{\eta} \exp\Bigl( -\frac{1}{\lambda} \Bigl[ S(\hat U + \mathcal{E}) + \lambda \sum_{t=0}^{T-1} \hat u_t^\top \Sigma^{-1} \varepsilon_t + \frac{\lambda}{2} \sum_{t=0}^{T-1} \hat u_t^\top \Sigma^{-1} \hat u_t \Bigr] \Bigr).
$$

Since $\mathbb{E}_{\mathbb{Q}_{\hat U}}[w] = 1$ and $v_t = \hat u_t + \varepsilon_t$,

$$
u_t^{\star} = \hat u_t + \mathbb{E}_{\mathbb{Q}_{\hat U}}\bigl[ w(\mathcal{E})  \varepsilon_t \bigr].
$$

This is the MPPI update law in its exact form: the new control is the old control plus the weighted average of the noise.

## 8.6 The Monte Carlo estimate

Draw $K$ noise sequences $\mathcal{E}^1, \dots, \mathcal{E}^K$, simulate the system under $\hat U + \mathcal{E}^k$, and record the cost

$$
\tilde S_k = S(\hat U + \mathcal{E}^k) + \lambda \sum_{t=0}^{T-1} \hat u_t^\top \Sigma^{-1} \varepsilon_t^k .
$$

The term $\tfrac{\lambda}{2}\sum_t \hat u_t^\top \Sigma^{-1} \hat u_t$ is the same for every sample, and the constant $\eta$ is unknown, so both are absorbed by normalising the weights to sum to one, as in Section 4.3:

$$
w_k = \frac{\exp\bigl( -\tilde S_k / \lambda \bigr)}{\sum_{j=1}^{K} \exp\bigl( -\tilde S_j / \lambda \bigr)}, \qquad
u_t \leftarrow \hat u_t + \sum_{k=1}^{K} w_k  \varepsilon_t^k .
$$

## 8.7 Remarks

**Nothing was assumed about the dynamics.** $F$ may be nonlinear, non-smooth, a neural network, or a physics simulator. It only has to be evaluated.

**The control cost is implicit.** The term $\lambda  \hat u_t^\top \Sigma^{-1} \varepsilon_t$ comes from the likelihood ratio, and its effect is that of a quadratic control penalty with $R = \lambda \Sigma^{-1}$. A sample whose noise points in the same direction as the current control, increasing its magnitude, is penalised. Many implementations instead put an explicit control cost in $S$ and drop this term. That is a different algorithm with a different fixed point; Chapter 10 compares the two.

**Agreement with Chapter 7.** With $\hat U = 0$ the correction term vanishes, the samples come from the uncontrolled system, and the update is $u_t = \sum_k w_k \varepsilon_t^k$ with weights $\exp(-S_k/\lambda)$. This is the path integral formula of Section 7.4.

**One update per time step.** With infinitely many samples the update returns $\mathbb{E}_{\mathbb{Q}^{\star}}[v_t]$ whatever the starting sequence $\hat U$. With finitely many, its accuracy depends on how well $\mathbb{Q}_{\hat U}$ covers the region where $\mathbb{Q}^{\star}$ has its mass. Model predictive control supplies a good $\hat U$ for free: the shifted solution from the previous time step.

## 8.8 Summary

- Shifting the mean of Gaussian control noise costs $\tfrac{\lambda}{2} u^\top \Sigma^{-1} u$ in relative entropy, so the free energy bounds a control problem with $R = \lambda \Sigma^{-1}$.
- The optimal distribution over input sequences is $q^{\star} \propto e^{-S/\lambda} p$.
- The control sequence closest to it is its mean, $u_t^{\star} = \mathbb{E}_{\mathbb{Q}^{\star}}[v_t]$.
- Importance sampling around the current sequence gives $u_t \leftarrow \hat u_t + \sum_k w_k \varepsilon_t^k$.
