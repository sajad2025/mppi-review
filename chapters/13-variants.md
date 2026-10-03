# 13. Variants and extensions

The basic algorithm has been modified in many directions. This chapter groups the main ones by the problem each addresses. The descriptions are brief; the references in Chapter 16 give the details.

## 13.1 Robustness to disturbances and model error

Basic MPPI samples around its nominal sequence and assumes the real system will follow the model. A disturbance that throws the state far from where the plan expected leaves the nominal sequence useless, and with it the samples.

**Tube-MPPI** (Williams et al., 2018, RSS) runs MPPI on a nominal, undisturbed copy of the system and adds an ancillary feedback controller, an iterative linear-quadratic regulator, that makes the real system track the nominal one. This is the structure of tube MPC with a sampling-based planner in the middle.

**Robust MPPI** (Gandhi et al., 2021) refines the idea. It chooses the nominal state at each step so as to limit how fast the cost can grow, applies the tracking feedback inside the rollouts, and derives a bound on the growth of the free energy in terms of the tracking controller's performance and the sampling error.

**Covariance steering** variants (Yin et al., 2022; Balci et al., 2022) control the covariance of the sampled trajectory distribution with feedback, so that samples stay where they are useful, which helps particularly for unstable systems, the case identified in Section 11.4.

## 13.2 Safety

A penalty in the cost discourages constraint violation but does not exclude it. **Shield-MPPI** (Yin et al., 2023) adds a control barrier function in two places: as a cost term that penalises rollouts leaving the safe set, and as a correction step applied to the final control. Other work projects samples onto a safe set before rollout or filters the output with a separate safety layer.

## 13.3 Better sampling

**Covariance design.** CoVO-MPC (Yi et al., 2024) analyses convergence for quadratic costs and derives the sampling covariance that maximises the contraction rate, which depends on the Hessian of the cost; the resulting method computes the covariance from a local quadratic model at each step.

**Different noise distributions.** log-MPPI (Mohamed et al., 2022) draws samples from a mixture of normal and log-normal distributions to explore more widely with fewer infeasible rollouts in cluttered environments.

**Annealing.** DIAL-MPC (Xue et al., 2025) performs several updates per control step with a covariance that decreases across iterations and along the horizon, motivated by a connection between the MPPI update and one step of a diffusion process. It was demonstrated on torque-level control of legged robots.

**Several modes.** Stein variational MPC (Lambert et al., 2020) represents the distribution over control sequences by a set of interacting particles, which can sit in different modes of a multimodal cost, addressing the averaging problem of Section 11.8.

**Informed proposals.** A recurring idea is to include among the samples the outputs of other controllers, such as a classical feedback law or a learned policy, so that at least some rollouts are good.

## 13.4 Smoothness

**Filtering.** The original implementations smooth the control sequence after the update (Section 9.5).

**Lifting the input.** Smooth MPPI (Kim et al., 2022) samples the derivative of the control and integrates, so that the control is smooth by construction and its rate can be penalised.

**Correlated noise.** Sampling noise that is correlated along the horizon, by filtering white noise or by parametrising the sequence with a few spline knots, produces smooth candidates and reduces the dimension of the search.

## 13.5 Learned models and value functions

Because MPPI needs only model evaluations, it combines directly with learned dynamics. Williams et al. (2017, ICRA) used a neural network model identified from driving data. In model-based reinforcement learning, TD-MPC (Hansen et al., 2022) plans with an MPPI-style procedure in a learned latent space and uses a learned value function as the terminal cost, which is the idea of Section 3.3 carried out by learning.

## 13.6 Software

Efficient implementations run the rollouts on a graphics processor. MPPI-Generic (Vlahov et al., 2024) is a C++/CUDA library from the group that introduced the method. STORM (Bhardwaj et al., 2021) applies sampling-based MPC of this kind to robot manipulators. MuJoCo MPC provides predictive sampling and related planners on top of the MuJoCo simulator, and several open-source Python implementations exist for PyTorch and JAX. Chapter 15 gives a minimal NumPy version intended for reading.

## 13.7 Summary

| Problem | Approach | Examples |
|---|---|---|
| Disturbances, model error | nominal system plus tracking feedback | Tube-MPPI, Robust MPPI |
| Diverging rollouts | feedback on the sampling distribution | covariance steering |
| Hard constraints | barrier functions, projection | Shield-MPPI |
| Poor sample efficiency | adapted or annealed covariance | CoVO-MPC, DIAL-MPC |
| Multimodal costs | particles in several modes | Stein variational MPC |
| Rough controls | filter, lift, or correlate | Smooth MPPI |
| No analytic model | learned dynamics and value | TD-MPC |
