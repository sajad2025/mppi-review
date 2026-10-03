# 7. Path integral control

The Hamilton–Jacobi–Bellman equation of Chapter 6 is nonlinear. For one class of problems a logarithmic change of variables makes it linear, and a linear equation of this type has a solution that can be written as an expectation over random trajectories. The optimal control is then a weighted average over those trajectories. This is path integral control (Kappen, 2005), the origin of the "PI" in MPPI.

## 7.1 The class of problems

Take the control-affine system of Section 6.4 and assume in addition that the noise enters through the same channel as the control:

$$
dx = f(x, t)  dt + G(x, t)  \bigl( u  dt + d\varepsilon \bigr),
$$

where $d\varepsilon$ is Brownian noise with covariance $\Sigma  dt$. In the notation of Chapter 6, $B = G L$ with $L L^\top = \Sigma$, so that $B B^\top = G \Sigma G^\top$. The cost rate is $q(x, t) + \tfrac12 u^\top R u$ as before. The Hamilton–Jacobi–Bellman equation is

$$
-\partial_t V = q + f^\top V_x - \tfrac12 V_x^\top G R^{-1} G^\top V_x + \tfrac12 \operatorname{tr}\bigl( G \Sigma G^\top V_{xx} \bigr), \qquad V(x, T) = \phi(x).
$$

## 7.2 The exponential transformation

Introduce a constant $\lambda \gt 0$ and define the desirability function $\Psi$ by

$$
V(x, t) = -\lambda \log \Psi(x, t).
$$

Then

$$
\partial_t V = -\lambda \frac{\partial_t \Psi}{\Psi}, \qquad
V_x = -\lambda \frac{\Psi_x}{\Psi}, \qquad
V_{xx} = -\lambda \frac{\Psi_{xx}}{\Psi} + \lambda \frac{\Psi_x \Psi_x^\top}{\Psi^2}.
$$

Substituting into the equation and collecting the two terms that are quadratic in $\Psi_x$:

$$
-\frac{\lambda^2}{2 \Psi^2}  \Psi_x^\top G R^{-1} G^\top \Psi_x  +  \frac{\lambda}{2 \Psi^2}  \Psi_x^\top G \Sigma G^\top \Psi_x .
$$

These cancel if and only if

$$
\lambda  R^{-1} = \Sigma .
$$

This is the central assumption of path integral control. It ties the control cost to the noise: directions in which the noise is large must be cheap to control, and directions with little noise must be expensive. The scalar $\lambda$ is the constant of proportionality. Under this assumption, what remains after multiplying through by $-\Psi/\lambda$ is

$$
\partial_t \Psi = \frac{q}{\lambda} \Psi - f^\top \Psi_x - \tfrac12 \operatorname{tr}\bigl( G \Sigma G^\top \Psi_{xx} \bigr), \qquad \Psi(x, T) = \exp\bigl( -\phi(x)/\lambda \bigr).
$$

The equation is linear in $\Psi$.

## 7.3 The Feynman–Kac formula

Linear equations of this form are solved by expectations. Let $\mathbb{P}$ denote the distribution of trajectories of the **uncontrolled** system,

$$
dx = f(x, t)  dt + G(x, t)  d\varepsilon ,
$$

started from $x$ at time $t$. The Feynman–Kac formula (Øksendal, Chapter 8) states that

$$
\Psi(x, t) = \mathbb{E}_{\mathbb{P}}\Bigl[ \exp\Bigl( -\frac{1}{\lambda} S(\tau) \Bigr) \Bigr], \qquad
S(\tau) = \phi(x_T) + \int_t^T q(x_s, s)  ds ,
$$

where $\tau$ denotes a trajectory and $S(\tau)$ its state cost. To see why, apply Itô's formula to $\exp(-\frac{1}{\lambda}\int_t^s q)  \Psi(x_s, s)$ along an uncontrolled trajectory: the linear equation makes the $ds$ terms vanish, so the process is a martingale, and equating its value at time $t$ with its expected value at time $T$ gives the formula.

Returning to the value function,

$$
V(x, t) = -\lambda \log \mathbb{E}_{\mathbb{P}}\Bigl[ \exp\Bigl( -\frac{1}{\lambda} S(\tau) \Bigr) \Bigr].
$$

The optimal cost-to-go is the free energy of Chapter 5, with the uncontrolled trajectory distribution as the reference. The value function of a nonlinear stochastic control problem has been expressed as an expectation that can be estimated by simulating the uncontrolled system and averaging $\exp(-S/\lambda)$. No grid over the state space is involved.

## 7.4 The optimal control as a weighted average of noise

The optimal control is $u^{\star} = -R^{-1} G^\top V_x$. With $V = -\lambda \log \Psi$ and $\lambda R^{-1} = \Sigma$,

$$
u^{\star}(x, t) = \Sigma  G^\top \frac{\Psi_x(x, t)}{\Psi(x, t)} .
$$

The gradient $\Psi_x$ can also be turned into an expectation. Consider the first small step of an uncontrolled trajectory, $x' = x + f  \Delta t + G  \Delta\varepsilon$ with $\Delta\varepsilon \sim \mathcal{N}(0, \Sigma \Delta t)$. Conditioning on $x'$,

$$
\mathbb{E}_{\mathbb{P}}\bigl[ e^{-S(\tau)/\lambda}  \Delta\varepsilon \bigr] = \mathbb{E}\bigl[ \Delta\varepsilon   e^{-q \Delta t/\lambda}  \Psi(x + f \Delta t + G \Delta\varepsilon,  t + \Delta t) \bigr].
$$

For a Gaussian vector $z \sim \mathcal{N}(0, C)$ and a smooth function $h$, integration by parts gives $\mathbb{E}[z  h(z)] = C  \mathbb{E}[\nabla h(z)]$ (Stein's lemma). Applying it with $z = \Delta\varepsilon$ and $C = \Sigma \Delta t$,

$$
\mathbb{E}_{\mathbb{P}}\bigl[ e^{-S(\tau)/\lambda}  \Delta\varepsilon \bigr] = \Sigma  \Delta t   G^\top \Psi_x(x, t) + o(\Delta t).
$$

Dividing by $\Psi(x,t) = \mathbb{E}_{\mathbb{P}}[e^{-S(\tau)/\lambda}]$ and comparing with the expression for $u^{\star}$:

$$
u^{\star}(x, t)  \Delta t = \frac{\mathbb{E}_{\mathbb{P}}\bigl[ e^{-S(\tau)/\lambda}  \Delta\varepsilon \bigr]}{\mathbb{E}_{\mathbb{P}}\bigl[ e^{-S(\tau)/\lambda} \bigr]} + o(\Delta t).
$$

This is the path integral formula for the optimal control. In words: simulate the uncontrolled system many times; weight each trajectory by $\exp(-S/\lambda)$; the optimal control effort over the next instant is the weighted average of the noise that the trajectories received in that instant. If the trajectories that happened to be pushed to the left did well, push left.

A Monte Carlo version with $K$ sampled trajectories is

$$
u^{\star}  \Delta t \approx \sum_{k=1}^{K} w_k  \Delta\varepsilon^k, \qquad
w_k = \frac{\exp(-S_k/\lambda)}{\sum_{j} \exp(-S_j/\lambda)} ,
$$

which is the self-normalised estimator of Section 4.3.

## 7.5 What is missing

Two things stand between this formula and a practical controller.

**Sampling efficiency.** The expectation is over the uncontrolled system. For a task such as swinging up a pendulum, almost no uncontrolled trajectory comes close to succeeding, so almost all weights are negligible and the estimate rests on one or two samples. The remedy is to sample around a control sequence that is already reasonable and to correct for the change of distribution with a likelihood ratio, as in Section 4.2. In continuous time the likelihood ratio between two diffusions is given by Girsanov's theorem. Williams, Aldrich and Theodorou (2017) carried this out, combined it with a receding horizon and warm starting, and ran the rollouts on a graphics processor; they called the result model predictive path integral control.

**Restrictive assumptions.** The derivation needs dynamics that are affine in the control, noise that enters only through the control channel, and the relation $\lambda R^{-1} = \Sigma$. Many models of interest, such as neural-network dynamics, are not control-affine. The next chapter gives a derivation that requires none of this structure of the dynamics and arrives at the same update.

## 7.6 Historical note

The linearisation of the Hamilton–Jacobi–Bellman equation by a logarithmic transformation goes back to work of Fleming in the 1970s. Its use for computing controls by sampling is due to Kappen (2005). Todorov (2006, 2009) developed the parallel discrete-state theory of linearly solvable Markov decision processes. Theodorou, Buchli and Schaal (2010) turned the path integral formula into the policy-improvement algorithm known as PI², widely used in robot learning. MPPI (Williams et al., 2016 and 2017) is the model predictive form.

## 7.7 Summary

- Under the assumption $\lambda R^{-1} = \Sigma$, the substitution $V = -\lambda \log \Psi$ makes the Hamilton–Jacobi–Bellman equation linear.
- By the Feynman–Kac formula, the value function is the free energy of the path cost under the uncontrolled dynamics.
- The optimal control is the average of the noise, weighted by $\exp(-S/\lambda)$.
- Sampling from the uncontrolled system is inefficient, which motivates importance sampling around a nominal control sequence.
