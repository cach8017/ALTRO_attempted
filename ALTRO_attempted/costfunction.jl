function compute_augmented_cost(X, U, λ_vals, ρ, solver, ilqr_params, constraints)
    """
    Compute the augmented Lagrangian cost function.

    Arguments:
    - X: State trajectory.
    - U: Control trajectory.
    - λ_vals: Lagrange multipliers for constraints.
    - ρ: Penalty parameter for constraints.
    - solver: Solver object with parameters like dynamics.
    - ilqr_params: Dictionary of iLQR parameters (e.g., Q, R, Qf).
    - constraints: Dictionary of problem constraints.

    Returns:
    - J: Total augmented Lagrangian cost.
    """
    J = 0.0  # Initialize total cost
    N = length(U)  # Number of timesteps

    # Extract cost matrices
    Q = ilqr_params[:Q]
    R = ilqr_params[:R]
    Qf = ilqr_params[:Qf]

    # Loop through each timestep
    for k in 1:N

        
        # State and control deviations
        δx_k = X[k] - constraints[:X_desired][k]  # Deviation from desired state
        δu_k = U[k]

        # Constraint residuals
        g_k = compute_constraint_residuals_at_k(X, U, k, solver, constraints)

        println("State deviation dimensions at step $k: ", size(δx_k))
        println("Control deviation dimensions at step $k: ", size(δu_k))
        println("Constraint residual dimensions at step $k: ", size(g_k))
        println("Lagrange multiplier dimensions at step $k: ", size(λ_vals[k]))


        # Original cost terms
        J += 0.5 * δx_k' * Q * δx_k  # State cost
        J += 0.5 * δu_k' * R * δu_k  # Control cost

        println("Norm of constraint residual (g_k): ", norm(g_k))


        # Augmented Lagrangian terms
        J += λ_vals[k]' * g_k  # Lagrange multiplier term
        J += 0.5 * ρ * norm(g_k)^2  # Quadratic penalty term
    end

    # Add terminal cost
    δx_N = X[end] - constraints[:X_desired][end]
    J += 0.5 * δx_N' * Qf * δx_N  # Terminal state cost

    return J
end




