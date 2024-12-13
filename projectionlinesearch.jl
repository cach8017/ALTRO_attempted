using Zygote


# function projection_line_search(Y, S, H_inv, D, d, v0; α_init = 1.0, γ = 0.5, tol = 1e-8)
#     """
#     Perform a line search to update the trajectory during the projection step.

#     Arguments:
#     - Y: Current concatenated state and control vector.
#     - S: Schur complement matrix (D * H_inv * D').
#     - H_inv: Regularized Hessian inverse.
#     - D: Jacobian of active constraints.
#     - d: Current constraint residuals.
#     - v0: Initial infinity norm of d.
#     - α_init: Initial step size (default: 1.0).
#     - γ: Step size reduction factor (default: 0.5).
#     - tol: Minimum step size tolerance (default: 1e-8).

#     Returns:
#     - Y_new: Updated trajectory vector.
#     - v: Updated infinity norm of the constraint residuals.
#     """
#     α = α_init  # Initialize step size
#     Y_new = copy(Y)  # Copy of the current trajectory
#     v = v0  # Current constraint violation

#     println("Starting line search with initial residual norm (v0): $v0")

#     while v > v0 && α > tol
#         # Step 1: Compute δY_p (direction for updating Y)
#         if cond(S) > 1e8
#             println("Schur complement matrix S is poorly conditioned. Applying regularization.")
#             S += 1e-6 * I
#         end

#         δY_p = H_inv * D' * (S \ (S' \ d))

#         # Step 2: Update Y using δY_p
#         Y_new += α * δY_p

#         # Step 3: Update the constraints using the new Y
#         d_new = update_constraints(Y_new, solver)  # Compute updated constraint residuals

#         # Step 4: Compute the new infinity norm of d
#         v = maximum(abs.(d_new))

#         # Step 5: Print debug info and reduce step size
#         println("Updated residual norm (v): $v with step size (α): $α")
#         α *= γ  # Reduce step size
#     end

#     println("Line search completed with final residual norm (v): $v")
#     return Y_new, v
# end



# function update_constraints(Y, solver)
#     """
#     Update the constraint residuals for the given trajectory vector.

#     Arguments:
#     - Y: Concatenated state and control vector.
#     - solver: Solver object containing problem data.

#     Returns:
#     - d: Updated constraint residuals as a single vector.
#     """
#     N = solver.N  # Number of time steps
#     nx = length(solver.D[1].fx)  # State dimension
#     nu = length(solver.D[1].fu)  # Control dimension

#     println("Updating constraints...")
#     println("Decomposing Y into states (X) and controls (U)...")

#     # Decompose Y into states (X) and controls (U)
#     X = [Y[(i-1)*nx + 1:i*nx] for i in 1:N+1]
#     U = [Y[N*nx + (i-1)*nu + 1:N*nx + i*nu] for i in 1:N]

#     println("Size of decomposed X: ", [size(x) for x in X])
#     println("Size of decomposed U: ", [size(u) for u in U])

#     # Compute the constraint residuals
#     residuals = []

#     for k in 1:N
#         # Predicted next state based on dynamics
#         x_next_dynamics = vehicle_dynamics(X[k], U[k], solver.Δt, solver.l)

#         # Residual: Actual next state - Predicted next state
#         residual = X[k+1] - x_next_dynamics
#         push!(residuals, residual)
#     end

#     # Concatenate residuals into a single vector
#     d = vcat(residuals...)

#     println("Updated constraint residuals size: $(size(d))")
#     return d
# end


function projection_line_search(Y, S, H_inv, D, d, v0, nx, nu, solver)
    α = 1.0  # Initial step size
    γ = 0.5  # Step size reduction factor
    max_iters = 10  # Max iterations for line search
    iter_count = 0
    tol_improvement = 1e-8  # Improvement threshold for early stopping

    while iter_count < max_iters
        δY_p = H_inv * D' * (S \ (S' \ d))  # Newton step direction
        println("Line search step direction (δY_p): ", δY_p)

        Y_new = Y + α * δY_p
        d_new = UPDATECONSTRAINTS(Y_new, nx, nu, solver)  # Update constraints
        v_new = maximum(abs.(d_new))
        println("Residual norm after step (v_new): ", v_new)

        # Check if the new residual norm is sufficiently reduced
        if v_new < v0
            if abs(v_new - v0) < tol_improvement
                println("Early termination: residual improvement below threshold.")
                return Y_new, v_new  # Stop line search if improvement is negligible
            end
            return Y_new, v_new  # Successful step
        else
            α *= γ  # Reduce step size
        end
        iter_count += 1
    end
    println("Max line search iterations reached.")
    return Y, v0  # Return the previous state if no improvement
end


function UPDATECONSTRAINTS(Y, nx, nu, solver)
    """
    Update the constraint residuals based on the current optimization variable Y.

    Arguments:
    - Y: Concatenated vector of states and controls.
    - nx: State dimension.
    - nu: Control dimension.
    - solver: Solver object with problem dimensions and dynamics.

    Returns:
    - d: Updated constraint residuals.
    """
    N = solver.N       # Number of time steps
    Δt = solver.Δt     # Time step size
    l = solver.l       # Vehicle length

    # Decompose Y into states (X) and controls (U)
    X_vals = [Y[(i-1)*nx + 1:i*nx] for i in 1:N+1]
    U_vals = [Y[N*nx + (i-1)*nu + 1:N*nx + i*nu] for i in 1:N]

    # Compute residuals for the dynamics constraints
    d_list = []
    for k in 1:N
        X_k = X_vals[k]
        U_k = U_vals[k]
        X_kp1 = X_vals[k+1]

        # Compute the predicted next state using vehicle dynamics
        X_pred = vehicle_dynamics(X_k, U_k, Δt, l)

        # Compute the residual for time step k
        d_k = X_kp1 - X_pred
        push!(d_list, d_k)
    end

    # Concatenate residuals into a single vector
    d = vcat(d_list...)

    return d
end

