"""MPPI on a linear-quadratic problem, where everything can be computed in closed form (Chapter 10).

Dynamics x_{t+1} = A x_t + B v_t, state cost S(V) = sum_t x_t' Q x_t + x_T' Qf x_T.
As a function of the stacked input V this is S(V) = 1/2 V' M V + b' V + const.

Variant 1 (information-theoretic weights, with the term lam * u' Sigma^{-1} eps):
    the infinite-sample update lands on  U_opt = -(M + lam * Sigma^{-1})^{-1} b  in one iteration.
Variant 2 (weights exp(-J/lam) with an explicit cost J(U) = S(U) + 1/2 U' Rbar U):
    the infinite-sample update contracts the error by  (I + Sigma * (M + Rbar) / lam)^{-1}.
"""
import numpy as np


def build(T=8, q_pos=1.0, q_vel=0.1, qf=5.0):
    A = np.array([[1.0, 0.1], [0.0, 1.0]]); B = np.array([[0.005], [0.1]])
    Q = np.diag([q_pos, q_vel]); Qf = qf * Q
    n, m = 2, 1
    Su = np.zeros((n * T, m * T)); Sx = np.zeros((n * T, n)); Qbar = np.zeros((n * T, n * T))
    for i in range(T):
        Sx[i*n:(i+1)*n] = np.linalg.matrix_power(A, i + 1)
        for j in range(i + 1):
            Su[i*n:(i+1)*n, j*m:(j+1)*m] = np.linalg.matrix_power(A, i - j) @ B
        Qbar[i*n:(i+1)*n, i*n:(i+1)*n] = Q if i < T - 1 else Qf
    M = 2.0 * Su.T @ Qbar @ Su                   # S(V) = 1/2 V' M V + b' V + const
    return M, (lambda x0: 2.0 * Su.T @ Qbar @ Sx @ x0)


def mppi_iteration(U, M, b, Sigma, lam, K, rng, variant, Rbar=None):
    L = np.linalg.cholesky(Sigma)
    eps = rng.standard_normal((K, len(U))) @ L.T
    V = U[None, :] + eps
    cost = 0.5 * np.einsum('ki,ij,kj->k', V, M, V) + V @ b
    if variant == 1:
        cost += lam * eps @ np.linalg.solve(Sigma, U)
    else:
        cost += 0.5 * np.einsum('ki,ij,kj->k', V, Rbar, V)
    w = np.exp(-(cost - cost.min()) / lam); w /= w.sum()
    return U + w @ eps, 1.0 / np.sum(w ** 2)


if __name__ == "__main__":
    T, lam = 8, 1.0
    M, bfun = build(T)
    b = bfun(np.array([1.0, 0.0])); Sigma = 4.0 * np.eye(T); rng = np.random.default_rng(0)

    U_opt = -np.linalg.solve(M + lam * np.linalg.inv(Sigma), b)
    print("Variant 1: distance to the optimum after one iteration from U = 0")
    for K in [100, 1000, 10000, 100000]:
        d = [np.linalg.norm(mppi_iteration(np.zeros(T), M, b, Sigma, lam, K, rng, 1)[0] - U_opt) for _ in range(50)]
        print("  K = %6d: mean error %.4f   (||U_opt|| = %.3f)" % (K, np.mean(d), np.linalg.norm(U_opt)))

    Rbar = 0.25 * np.eye(T); Aj = M + Rbar
    U_j = -np.linalg.solve(Aj, b)
    C = np.linalg.inv(np.eye(T) + Sigma @ Aj / lam)
    print("Variant 2: predicted contraction factors (eigenvalues of (I + Sigma A / lam)^-1):")
    print("  ", np.round(np.sort(np.linalg.eigvals(C).real), 4))
    U = np.zeros(T); K = 200000
    print("  iteration, measured error, predicted error (K = %d)" % K)
    e_pred = U - U_j
    for it in range(1, 7):
        U, ess = mppi_iteration(U, M, b, Sigma, lam, K, rng, 2, Rbar)
        e_pred = C @ e_pred
        print("   %d   %.5f   %.5f   (ESS %.0f)" % (it, np.linalg.norm(U - U_j), np.linalg.norm(e_pred), ess))
