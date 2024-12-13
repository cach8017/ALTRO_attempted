# AL-iLQR Outer Loop Implementation
function AL_iLQR(X, U, S, λ_vals, ρ, tol, solver, ilqr_params, constraints, al_params)
    max_outer_iterations = al_params[:max_outer_iterations]
    constraint_tol = al_params[:constraint_tol]
    penalty_scaling = al_params[:penalty_scaling]
    penalty_max = al_params[:penalty_max]

    # Compute initial augmented cost
    J_aug_prev = compute_augmented_cost(X, U, λ_vals, ρ, solver, ilqr_params, constraints)
    println("Initial Augmented Cost: $J_aug_prev")

    for outer_iter in 1:max_outer_iterations
        println("\n--- Outer Iteration $outer_iter ---")

        # Inner Loop (iLQR)
        for iter in 1:ilqr_params[:max_iterations]
            println("\n--- Inner Iteration $iter ---")

            # Backward Pass
            println("Running Backward Pass...")
            success, K, d, ΔV = backward_pass(X, U, S, λ_vals, ρ, solver, ilqr_params, constraints)
            if !success
                println("Backward Pass Failed.")
                break
            end

            if !success
                println("Backward Pass Failed.")
                break
            end

            println("Before Forward Pass: J_aug_prev = $J_aug_prev")


            # Forward Pass
            println("Running Forward Pass...")
            X, U, J_aug_new = forward_pass(X, U, K, d, ΔV, λ_vals, ρ, J_aug_prev, solver, ilqr_params, constraints)
          

            println("After Forward Pass: J_aug_new = $J_aug_new")
            

            # Check iLQR Convergence
            ΔJ = abs(J_aug_new - J_aug_prev)
            println("ΔJ = $ΔJ")
            println("Inner Loop: Augmented Cost Difference: ΔJ = $ΔJ")
            if ΔJ < ilqr_params[:epsilon_cost]
                println("iLQR Converged.")
                break
            end

            J_aug_prev = J_aug_new
        end

        # Compute Constraint Residuals
        constraint_residuals = compute_constraint_residuals(X, U, solver, constraints)
        max_residual = maximum(abs.(constraint_residuals))
        println("Constraint Residuals (Max): $max_residual")

        # Check AL-iLQR Convergence
        if max_residual < constraint_tol
            println("AL-iLQR Converged: Constraints Satisfied.")
            return X, U, J_aug_new
        end

        # Update Lagrange Multipliers
        println("Updating Lagrange Multipliers...")
        for k in 1:solver.N
            g_k = compute_constraint_residuals_at_k(X, U, k, solver, constraints)
            λ_vals[k] .= λ_vals[k] + ρ * g_k
        end

        # Adjust Penalty Parameter
        println("Adjusting Penalty Scaling...")
        ρ *= penalty_scaling
        ρ = min(ρ, penalty_max)
        println("Updated Penalty Parameter: ρ = $ρ")
    end

    println("AL-iLQR Reached Max Outer Iterations Without Full Convergence.")
    return X, U, J_aug_prev
end

# Helper Functions
function compute_augmented_cost(X, U, λ_vals, ρ, solver, ilqr_params, constraints)
    J = 0.0
    N = length(U)

    for k in 1:N
        # Quadraticized Original Cost
        δx_k = X[k] - constraints[:X_desired][k]
        δu_k = U[k]
        Q, R = ilqr_params[:Q], ilqr_params[:R]
        J += 0.5 * δx_k' * Q * δx_k + 0.5 * δu_k' * R * δu_k

        # Augmented Cost Terms (Lagrange Multiplier and Penalty)
        g_k = compute_constraint_residuals_at_k(X, U, k, solver, constraints)
        J += λ_vals[k]' * g_k + 0.5 * ρ * norm(g_k)^2
    end

    # Terminal Cost
    δx_N = X[end] - constraints[:X_desired][end]
    Qf = ilqr_params[:Qf]
    J += 0.5 * δx_N' * Qf * δx_N

    return J
end

function compute_constraint_residuals(X, U, solver, constraints)
    residuals = []
    for k in 1:solver.N
        g_k = compute_constraint_residuals_at_k(X, U, k, solver, constraints)
        append!(residuals, g_k)
    end
    return residuals
end










