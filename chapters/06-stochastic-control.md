# 6. Stochastic optimal control in continuous time

MPPI was first derived in continuous time, from the partial differential equation that the value function of a stochastic control problem satisfies. This chapter derives that equation. Chapter 7 shows how, for a particular class of problems, it can be solved by an expectation. Readers who prefer to stay in discrete time can go directly to Chapter 8, which reaches the same algorithm by a different route.

## 6.1 Stochastic differential equations

A controlled system driven by noise is written

$$
dx = f(x, u, t)  dt + B(x, t)  dw ,
$$

where $w$ is a Brownian motion: a continuous random process whose increments over disjoint intervals are independent, with $w(t + \Delta t) - w(t) \sim \mathcal{N}(0, \Delta t  I)$. The equation is shorthand for its integrated form, and its simplest discretisation (Euler–Maruyama) is

$$
x_{t + \Delta t} = x_t + f(x_t, u_t, t)  \Delta t + B(x_t, t)  \sqrt{\Delta t}  \xi_t, \qquad \xi_t \sim \mathcal{N}(0, I).
$$

The noise increment has standard deviation of order $\sqrt{\Delta t}$, not $\Delta t$. Its square is therefore of order $\Delta t$ and cannot be neglected in a first-order expansion. This is the one place where stochastic calculus differs from ordinary calculus, and it is expressed by Itô's formula: for a smooth function $V(x, t)$,

$$
dV = \Bigl( \partial_t V + f^\top V_x + \tfrac12 \operatorname{tr}\bigl( B B^\top V_{xx} \bigr) \Bigr) dt + V_x^\top B  dw ,
$$

where $V_x$ is the gradient and $V_{xx}$ the Hessian with respect to $x$. Compared with the ordinary chain rule there is one extra term, containing second derivatives and the noise covariance $B B^\top$ (Øksendal, Chapter 4).

## 6.2 The control problem

The objective is the expected cost over the interval $[t, T]$,

$$
V(x, t) = \min_{u(\cdot)}   \mathbb{E}\Bigl[ \phi(x_T) + \int_t^T \ell(x_s, u_s, s)  ds  \Big|  x_t = x \Bigr],
$$

where the minimum is over feedback controls. $V$ is the value function, the continuous-time counterpart of $J_t$ in Chapter 2.

## 6.3 The Hamilton–Jacobi–Bellman equation

Apply the principle of optimality over a short interval $[t, t + \Delta t]$:

$$
V(x, t) = \min_u   \mathbb{E}\bigl[ \ell(x, u, t)  \Delta t + V(x_{t+\Delta t}, t + \Delta t) \bigr] + o(\Delta t).
$$

Expand $V(x_{t+\Delta t}, t+\Delta t)$ with Itô's formula and take the expectation. The term $V_x^\top B  dw$ has zero mean and drops out, leaving

$$
V(x,t) = \min_u \Bigl[ \ell  \Delta t + V + \bigl( \partial_t V + f^\top V_x + \tfrac12 \operatorname{tr}(B B^\top V_{xx}) \bigr) \Delta t \Bigr] + o(\Delta t).
$$

Cancel $V$, divide by $\Delta t$, and let $\Delta t \to 0$:

$$
-\partial_t V = \min_u \Bigl[ \ell(x, u, t) + f(x, u, t)^\top V_x + \tfrac12 \operatorname{tr}\bigl( B B^\top V_{xx} \bigr) \Bigr], \qquad V(x, T) = \phi(x).
$$

This is the Hamilton–Jacobi–Bellman (HJB) equation. It is a partial differential equation that runs backwards in time from the terminal condition. The argument above is heuristic; the rigorous statement, including the sense in which a non-smooth $V$ solves the equation, is in Fleming and Soner (Chapters 3 to 5).

## 6.4 Control-affine dynamics and quadratic control cost

The minimisation inside the HJB equation can be done explicitly for the class of problems that matters here. Suppose the dynamics are affine in the control and the cost is quadratic in it:

$$
f(x, u, t) = f(x, t) + G(x, t)  u, \qquad \ell(x, u, t) = q(x, t) + \tfrac12 u^\top R  u ,
$$

with $R$ positive definite. The bracket is then a convex quadratic in $u$, minimised at

$$
u^{\star}(x, t) = -R^{-1} G(x,t)^\top V_x(x, t).
$$

The optimal control pushes the state in the direction in which the value function decreases, through the input matrix, scaled by the inverse of the control cost. Substituting back,

$$
-\partial_t V = q + f^\top V_x - \tfrac12 V_x^\top G R^{-1} G^\top V_x + \tfrac12 \operatorname{tr}\bigl( B B^\top V_{xx} \bigr).
$$

The equation is now free of the minimisation, but it is nonlinear: the third term is quadratic in the unknown $V_x$. A nonlinear second-order partial differential equation in $n$ space dimensions is as hard to solve on a grid as the Bellman recursion was in Chapter 2, and for the same reason.

## 6.5 Where this leaves us

The HJB equation characterises the optimal controller completely: solve for $V$, differentiate, and $u^{\star} = -R^{-1} G^\top V_x$. The obstacle is computing $V$. The next chapter shows that under one additional structural assumption, relating the noise to the control cost, a change of variables removes the nonlinearity. The resulting linear equation has a solution in the form of an expectation over trajectories of the uncontrolled system, which Monte Carlo can estimate without a grid.

## 6.6 Summary

- Noise of size $\sqrt{\Delta t}$ adds a second-derivative term to the chain rule (Itô's formula).
- The value function of a stochastic control problem satisfies the HJB equation, a backward partial differential equation.
- For control-affine dynamics and quadratic control cost, the optimal control is $u^{\star} = -R^{-1} G^\top V_x$ and the HJB equation becomes a nonlinear equation in $V$ alone.
