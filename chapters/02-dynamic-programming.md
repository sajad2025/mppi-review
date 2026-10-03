# 2. Dynamic programming and the linear-quadratic regulator

Dynamic programming is the general method for solving optimal control problems. It is exact, it produces a feedback policy, and for almost every problem of practical size it cannot be carried out. This chapter explains all three statements. The one large class for which it can be carried out, linear dynamics with quadratic cost, is used again in Chapter 10.

## 2.1 The principle of optimality

Consider the deterministic problem of Chapter 1 and define the optimal cost-to-go from state $x$ at time $t$:

$$
J_t(x) = \min_{u_t, \dots, u_{T-1}} \Bigl[ \phi(x_T) + \sum_{s=t}^{T-1} \ell(x_s, u_s) \Bigr], \qquad x_t = x .
$$

The principle of optimality (Bellman) states that the tail of an optimal sequence is optimal for the tail problem: if $(u_0^{\star}, \dots, u_{T-1}^{\star})$ is optimal from $x_0$ and passes through $x_t^{\star}$ at time $t$, then $(u_t^{\star}, \dots, u_{T-1}^{\star})$ is optimal from $x_t^{\star}$. If it were not, replacing the tail by a better one would lower the total cost, a contradiction.

The principle turns one optimisation over $T$ controls into $T$ optimisations over one control each.

## 2.2 The dynamic programming recursion

Start at the end, where there is nothing left to decide, and work backwards:

$$
J_T(x) = \phi(x), \qquad
J_t(x) = \min_{u \in \mathcal{U}} \bigl[ \ell(x, u) + J_{t+1}(F(x, u)) \bigr], \quad t = T-1, \dots, 0 .
$$

The minimising $u$ in the second equation, as a function of $x$, is the optimal policy $\mu_t^{\star}(x)$. The function $J_t$ is called the value function. For the stochastic problem with disturbance $w_t$ the recursion is the same with an expectation inside:

$$
J_t(x) = \min_{u \in \mathcal{U}}   \mathbb{E}_{w}\bigl[ \ell(x, u) + J_{t+1}(F(x, u, w)) \bigr].
$$

This is the Bellman equation. Its solution gives the optimal cost and the optimal feedback policy from every state.

## 2.3 Why it cannot usually be computed

The recursion asks for the function $J_{t+1}$ at every state before $J_t$ can be computed at any state. If the state space is discretised with 100 points per dimension, a system with $n$ state variables needs $100^n$ values per time step. For the pendulum ($n = 2$) this is ten thousand values and easy. For a quadrotor ($n = 12$) it is $10^{24}$. Bellman called this the curse of dimensionality.

Every practical method is a way around this obstacle. There are two broad strategies:

1. **Approximate the value function** with a parametrised family, which leads to approximate dynamic programming and reinforcement learning.
2. **Give up on the whole state space** and optimise only from the state the system is currently in, repeating as the state changes. This is model predictive control (Chapter 3), and MPPI belongs to it.

## 2.4 The linear-quadratic regulator

Dynamic programming can be carried out in closed form when the dynamics are linear and the cost is quadratic:

$$
x_{t+1} = A x_t + B u_t, \qquad
\ell(x, u) = x^\top Q x + u^\top R u, \qquad \phi(x) = x^\top Q_f  x ,
$$

with $Q, Q_f$ positive semidefinite and $R$ positive definite. Guess that the value function is quadratic, $J_{t+1}(x) = x^\top P_{t+1} x$. Substituting into the recursion gives

$$
J_t(x) = \min_u \bigl[ x^\top Q x + u^\top R u + (A x + B u)^\top P_{t+1} (A x + B u) \bigr].
$$

The expression in brackets is a convex quadratic in $u$. Setting its gradient to zero,

$$
u = -K_t x, \qquad K_t = (R + B^\top P_{t+1} B)^{-1} B^\top P_{t+1} A ,
$$

and substituting back shows that $J_t(x) = x^\top P_t x$ with

$$
P_t = Q + A^\top P_{t+1} A - A^\top P_{t+1} B  (R + B^\top P_{t+1} B)^{-1} B^\top P_{t+1} A, \qquad P_T = Q_f .
$$

This is the Riccati recursion. The guess was correct at $t = T$ and is preserved by each step, so it holds for all $t$. Three facts are worth recording.

- **The optimal policy is linear state feedback.** No state-space grid is needed; the value function is described by one $n \times n$ matrix per time step.
- **Infinite horizon.** As $T \to \infty$, under standard stabilisability and detectability conditions, $P_t$ converges to the solution $P$ of the discrete algebraic Riccati equation, and $u = -Kx$ with constant $K$ is optimal and stabilising.
- **Certainty equivalence.** If additive zero-mean noise is added, $x_{t+1} = A x_t + B u_t + w_t$, the optimal gains $K_t$ are unchanged. The noise only adds a constant to the cost. So for linear-quadratic problems, planning as if there were no noise is optimal.

The linear-quadratic regulator (LQR) is the reference point for everything else. It is what an exact method produces when an exact method is available, and a sampling-based controller applied to a linear-quadratic problem should reproduce it as the number of samples grows. Chapter 10 checks that MPPI does.

## 2.5 Summary

- The value function satisfies the Bellman recursion, and the minimiser is the optimal feedback policy.
- Computing the value function on a grid costs exponentially in the state dimension.
- For linear dynamics and quadratic cost, the value function is quadratic and the Riccati recursion gives the optimal linear feedback.
