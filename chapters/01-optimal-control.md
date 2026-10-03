# 1. The optimal control problem

Control is the problem of choosing the inputs of a system so that it behaves as desired. Optimal control makes "as desired" precise by attaching a number, the cost, to every possible behaviour and asking for the inputs that make this number smallest. This chapter states the problem in the discrete-time form used in the rest of the book.

## 1.1 Systems

A discrete-time dynamical system is described by a state $x_t \in \mathbb{R}^n$, a control $u_t \in \mathbb{R}^m$, and a rule that produces the next state from the current state and control:

$$
x_{t+1} = F(x_t, u_t), \qquad t = 0, 1, 2, \dots
$$

The state is the information needed to predict the future given the future controls. For a pendulum it is the angle and the angular velocity; for a car it might be position, heading and speed. The function $F$ is called the dynamics, or the model. Most physical systems are described first in continuous time by a differential equation $\dot x = f(x, u)$; a discrete-time model is obtained by integrating over one sampling interval $\Delta t$, for example with the Euler rule $F(x, u) = x + \Delta t  f(x, u)$.

Given an initial state $x_0$ and a control sequence $U = (u_0, u_1, \dots, u_{T-1})$, the dynamics determine a state trajectory $x_1, \dots, x_T$. Computing that trajectory by applying $F$ repeatedly is called a rollout. Rollouts are the only way MPPI ever uses the model.

## 1.2 Cost

The quality of a trajectory is measured by a cost function

$$
J(x_0, U) = \phi(x_T) + \sum_{t=0}^{T-1} \ell(x_t, u_t).
$$

The function $\ell$ is the running cost (or stage cost), $\phi$ is the terminal cost, and $T$ is the horizon. Typical choices penalise distance from a target and control effort, for example

$$
\ell(x, u) = (x - x_{\mathrm{ref}})^\top Q  (x - x_{\mathrm{ref}}) + u^\top R  u ,
$$

with $Q$ positive semidefinite and $R$ positive definite. Nothing in the problem statement requires the cost to be quadratic, convex, or even continuous. A cost may contain an indicator such as "1000 if the state is inside an obstacle, 0 otherwise". Costs of that kind are awkward for methods that need derivatives and are one reason sampling methods exist.

The finite-horizon optimal control problem is

$$
\min_{U}   J(x_0, U) \quad \text{subject to} \quad x_{t+1} = F(x_t, u_t), \quad u_t \in \mathcal{U},
$$

where $\mathcal{U}$ is the set of admissible controls, for instance a box of actuator limits.

## 1.3 Open-loop sequences and feedback policies

There are two different objects one can optimise over.

An **open-loop** solution is a sequence $U$ computed once from $x_0$. It is a plan. If the model is exact and nothing disturbs the system, executing the plan is optimal.

A **closed-loop** solution, or feedback policy, is a sequence of functions $\pi = (\mu_0, \dots, \mu_{T-1})$ with $u_t = \mu_t(x_t)$. It prescribes what to do in every state the system might reach, not only in the states the plan expected.

For a deterministic system the two achieve the same optimal cost: the optimal policy evaluated along the optimal trajectory is the optimal sequence. Under uncertainty they differ, and feedback is better, because a policy can react to what actually happened. Chapter 3 shows how model predictive control obtains feedback out of repeated open-loop optimisation, which is the form in which MPPI is used.

## 1.4 Stochastic problems

Real systems are disturbed. A common model adds a random input $w_t$:

$$
x_{t+1} = F(x_t, u_t, w_t),
$$

with $w_0, w_1, \dots$ independent. The trajectory is then random, so the cost is random, and the objective becomes its expectation:

$$
\min_{\pi}   \mathbb{E}\Bigl[ \phi(x_T) + \sum_{t=0}^{T-1} \ell(x_t, \mu_t(x_t)) \Bigr].
$$

One special case matters for this book. Suppose the disturbance enters through the same channel as the control, so that the system receives $v_t = u_t + \varepsilon_t$ with $\varepsilon_t \sim \mathcal{N}(0, \Sigma)$:

$$
x_{t+1} = F(x_t, u_t + \varepsilon_t).
$$

MPPI is derived for exactly this structure. The noise plays two roles. In the theory it is a disturbance that the controller must cope with. In the algorithm it is the device by which the controller explores alternatives: each simulated rollout uses a different noise sequence, and the comparison between rollouts tells the controller which way to change $U$.

## 1.5 A running example

The pendulum is used throughout. Let $\theta$ be the angle measured from the upright position and $\omega = \dot\theta$. With mass $m$, length $l$, damping $b$ and torque $u$,

$$
\dot\theta = \omega, \qquad \dot\omega = \frac{g}{l} \sin\theta - b \omega + \frac{u}{m l^2}.
$$

The state is $x = (\theta, \omega)$. The task is to bring the pendulum from hanging down, $\theta = \pi$, to upright, $\theta = 0$, with a torque limit too small to lift it directly, so that the controller has to swing it back and forth to gain energy. A natural cost is $\ell(x,u) = \theta^2 + 0.1 \omega^2$ with the angle wrapped to $(-\pi, \pi]$. The problem is nonlinear, the cost is not convex in $U$, and the solution is not obvious, which makes it a fair small test.

## 1.6 Summary

- A model $x_{t+1} = F(x_t, u_t)$ turns a control sequence into a trajectory; this computation is a rollout.
- A cost $J$ assigns a number to each trajectory; optimal control minimises it.
- Open-loop solutions are plans; closed-loop solutions are policies. Under uncertainty, policies are better.
- MPPI assumes Gaussian noise entering through the control channel, and uses that same noise to explore.
