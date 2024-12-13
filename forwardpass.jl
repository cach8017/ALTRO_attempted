# Forward Pass Implementation
function forward_pass(X, U, K, d, ΔV, λ_vals, ρ, J_aug_prev, solver, ilqr_params, constraints)
    """
    Perform the forward pass for iLQR optimization with line search.

    Arguments:
    - X: Initial state trajectory.
    - U: Initial control trajectory.
    - K: Feedback gain matrices.
    - d: Feedforward terms.
    - ΔV: Expected cost reduction (terms scaled during line search).
    - λ_vals: Lagrange multipliers for constraints.
    - ρ: Penalty parameter.
    - J_aug_prev: Previous augmented cost value.
    - solver: Solver object containing problem data.
    - ilqr_params: Dictionary of iLQR hyperparameters.
    - constraints: Dictionary of constraints.

    Returns:
    - X_new: Updated state trajectory.
    - U_new: Updated control trajectory.
    - J_new: New augmented cost value.
    """

    # Line search parameters
    line_search_lb = ilqr_params[:line_search_lb]
    line_search_ub = ilqr_params[:line_search_ub]
    line_search_max_iters = ilqr_params[:line_search_max_iters]
    α = line_search_ub  # Start with the upper bound of line search

    # Initialize trajectories
    X_new = deepcopy(X)
    U_new = deepcopy(U)
    J_new = J_aug_prev

    println("Entering Forward Pass with Line Search...")
    for line_iter in 1:line_search_max_iters
        println("Line Search Iteration $line_iter: α = $α")

        # Simulate forward dynamics
        X_trial = [X[1]]  # Start with initial state
        U_trial = []

        for k in 1:solver.N
            δx_k = X_trial[k] - X[k]
            u_k = U[k] + K[k] * δx_k + α * d[k]  # Apply feedback and feedforward
            push!(U_trial, u_k)

            # Propagate dynamics
            x_k_next = vehicle_dynamics(X_trial[k], u_k, solver.Δt, solver.l)
            push!(X_trial, x_k_next)
        end

        # Compute cost for trial trajectories
        J_new = compute_augmented_cost(X_trial, U_trial, λ_vals, ρ, solver, ilqr_params, constraints)

        # Compute expected decrease in cost
        ΔV_actual = J_aug_prev - J_new
        ΔV_expected = α * ΔV[:linear] + 0.5 * α^2 * ΔV[:quadratic]
        z = ΔV_actual / ΔV_expected

        println("Actual vs Expected Cost Reduction: ΔV_actual = $ΔV_actual, ΔV_expected = $ΔV_expected, z = $z")

        # Line search acceptance criteria
        if z >= line_search_lb && z <= line_search_ub
            println("Line search successful: accepting trial trajectories.")
            X_new = X_trial
            U_new = U_trial
            break
        else
            println("Line search failed: reducing step size.")
            α *= 0.5  # Reduce step size
        end
    end

    if α < line_search_lb
        println("Line search failed to find a suitable step size.")
    end

    return X_new, U_new, J_new
end
