# 3. Model predictive control

Model predictive control (MPC) replaces the problem "find the optimal action in every state" by the smaller problem "find a good action in this state, now", solved again at every time step. It is the framework inside which MPPI operates: MPPI is one particular way of solving the optimisation that MPC requires.

## 3.1 The receding horizon

At time step $k$ the controller measures the current state $x_k$ and solves a finite-horizon optimal control problem that starts there:

$$
\min_{U = (u_0, \dots, u_{T-1})}   \phi(x_T) + \sum_{t=0}^{T-1} \ell(x_t, u_t)
\quad \text{subject to} \quad x_0 = x_k,    x_{t+1} = F(x_t, u_t),    u_t \in \mathcal{U}.
$$

Here $t$ counts steps into the predicted future, not real time. Only the first element $u_0$ of the solution is applied to the system. At the next step the state $x_{k+1}$ is measured, the horizon slides forward by one step, and the problem is solved again. Because the window of prediction moves with time, this is called receding horizon control.

The loop is:

1. Measure (or estimate) the current state.
2. Optimise a control sequence over the next $T$ steps using the model.
3. Apply the first control of that sequence.
4. Shift the sequence by one step and return to 1.

## 3.2 Why this gives feedback

Each optimisation is open-loop: it produces a plan, not a policy. But the plan is recomputed from the measured state at every step, so the control applied is a function of the current state, $u_k = \kappa(x_k)$, where $\kappa$ maps a state to the first element of the solution of the problem above. That is a feedback law. It is defined implicitly, by an optimisation, and it is evaluated only at the states the system actually visits. This is how MPC sidesteps the curse of dimensionality: it never represents the policy anywhere else.

Disturbances and model errors are handled the same way. If the system does not go where the plan predicted, the next plan starts from where it went.

## 3.3 The terminal cost

A finite horizon is short-sighted. A plan that looks good for $T$ steps may leave the system in a state from which things go badly afterwards. The terminal cost $\phi$ is there to stand in for the cost of everything beyond the horizon. If $\phi$ equalled the true infinite-horizon cost-to-go, a horizon of one step would already be optimal. In practice $\phi$ is an approximation, and a longer horizon compensates for a poorer approximation.

The standard stability theory of MPC (Rawlings, Mayne and Diehl, Chapter 2) makes this precise: if $\phi$ is a control Lyapunov function on a terminal region, in the sense that some admissible control decreases $\phi$ by at least the running cost, then the optimal value of the MPC problem decreases along closed-loop trajectories and the origin is asymptotically stable. For a linear-quadratic problem the choice $\phi(x) = x^\top P x$, with $P$ from the algebraic Riccati equation, makes finite-horizon MPC coincide exactly with infinite-horizon LQR, for every horizon length.

## 3.4 Warm starting

Consecutive problems are nearly the same. The solution at step $k$, shifted by one position, is a good guess for the solution at step $k+1$:

$$
(u_1, u_2, \dots, u_{T-1}, u_{\mathrm{new}}) .
$$

The last entry has no predecessor and must be filled in, commonly with zero, with a copy of $u_{T-1}$, or with a nominal controller. Starting each optimisation from the shifted previous solution is called warm starting. It matters for every MPC method and is essential for MPPI, which improves the sequence by a single step per time step and relies on the improvements accumulating.

## 3.5 Solving the optimisation in real time

Everything above assumes that the optimisation can be solved within one sampling period, which may be a few milliseconds. The methods divide into two families.

**Derivative-based methods** linearise the dynamics and build a quadratic model of the cost along the current trajectory, then solve the resulting linear-quadratic subproblem: sequential quadratic programming, the iterative linear-quadratic regulator (iLQR), differential dynamic programming. They converge fast near a solution. They need derivatives of $F$ and $\ell$, they struggle when those derivatives are discontinuous or uninformative (contact, collisions, indicator costs), and they find the local minimum nearest the starting guess.

**Sampling-based methods** evaluate the cost of many candidate control sequences and combine the results without using derivatives. They need only the ability to simulate. Each candidate is independent of the others, so the work parallelises perfectly. Their weakness is the number of samples needed, which grows with the dimension of the control sequence.

MPPI is a sampling-based method. What distinguishes it from simply trying random sequences and keeping the best one is the rule by which the candidates are combined. That rule is an importance-sampling estimate of a specific expectation, and the next two chapters develop the tools needed to say which expectation and why.

## 3.6 Summary

- MPC solves a finite-horizon problem from the current state at every step and applies only the first control.
- Recomputation from the measured state makes it a feedback law.
- The terminal cost approximates the cost beyond the horizon and is the main tool for stability.
- Warm starting from the shifted previous solution carries information from one step to the next.
- The optimisation can be solved with derivatives or with samples; MPPI uses samples.
