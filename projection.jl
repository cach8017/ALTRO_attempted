# function projection(X_vals, U_vals, λ_vals, tol, solver::iLQRSolver, ilqr_params::Dict{Symbol, Any})
#     """
#     Refines state and control trajectories by enforcing constraints.
#     Arguments:
#     - X_vals: Initial state trajectory.
#     - U_vals: Initial control trajectory.
#     - λ_vals: Lagrange multipliers (not used here but reserved for extensibility).
#     - tol: Residual norm tolerance.
#     - solver: Solver object containing problem dimensions and precomputed data.
#     - ilqr_params: Dictionary of hyperparameters.
#     """
#     println("Projection: Initializing...")

#     # Extract Hyperparameters
#     ϵ = ilqr_params[:rho_min]             # Regularization term for Hessian
#     conv_rate_tol = ilqr_params[:epsilon_grad]  # Convergence rate tolerance

#     # Step 1: Invert Hessian
#     println("Step 1: Inverting Hessian...")
#     H_inv_blocks = Matrix{Float64}[]
#     for k in 1:solver.N
#         Quu_reg_k = solver.Quu_reg[k]
#         if cond(Quu_reg_k) > 1e10
#             println("Regularizing ill-conditioned Hessian at step $k.")
#             push!(H_inv_blocks, inv(Quu_reg_k + ϵ * I))
#         else
#             push!(H_inv_blocks, inv(Quu_reg_k))
#         end
#     end
#     H_inv = create_blockdiag(H_inv_blocks)
#     println("Step 1 Complete: Hessian Inverted.")

#     # Flatten X and U into Y
#     Y = vcat(vcat(X_vals...), vcat(U_vals...))
#     Y_new = copy(Y)
#     println("Initial Y vector created. Length: $(length(Y))")

#     # Projection Loop
#     while true
#         println("Step 2: Linearizing constraints...")
#         d, D, D_controls = linearize_active_constraints(X_vals, U_vals, solver)
#         println("Constraint Jacobians linearized. Residual size: $(size(d))")

#         # Step 3: Compute Schur Complement
#         println("Step 3: Computing Schur complement...")
#         S = D_controls * H_inv * D_controls'
#         println("Schur complement dimensions: $(size(S))")

#         # Step 4: Check Residual Norm
#         v = maximum(abs.(d))
#         println("Residual norm (v): $v")
#         if v <= tol
#             println("Projection converged. Residual norm within tolerance.")
#             break
#         end

#         # Step 5: Line Search Loop
#         println("Step 5: Entering line search loop...")
#         r = Inf
#         max_iters = ilqr_params[:line_search_max_iters]  # Add line search limit
#         iter_count = 0
#         while v > tol && r > conv_rate_tol && iter_count < max_iters
#             println("Performing line search...")
#             Y_new, v_new = projection_line_search(Y, S, H_inv, D, d, v)

#             # Update convergence rate and norms
#             r = log(v_new / v)
#             v = v_new
#             Y = copy(Y_new)
#             iter_count += 1

#             println("Residual norm updated: $v, Convergence rate: $r")
#         end
#     end

#     # Decompose Y back into X and U
#     println("Decomposing Y into X and U...")
#     nx = size(X_vals[1], 1)
#     nu = size(U_vals[1], 1)
#     X_new = [Y[(i-1)*nx + 1:i*nx] for i in 1:solver.N+1]
#     U_new = [Y[solver.N*nx + (i-1)*nu + 1:solver.N*nx + i*nu] for i in 1:solver.N]

#     println("Projection completed successfully.")
#     return X_new, U_new
# end

# function linearize_active_constraints(X_vals, U_vals, solver)
#     N = solver.N
#     Δt = solver.Δt
#     l = solver.l

#     nx = size(X_vals[1], 1)
#     nu = size(U_vals[1], 1)

#     d_list = []
#     D_blocks = []
#     D_controls_blocks = []

#     for k in 1:N
#         X_k = X_vals[k]
#         U_k = U_vals[k]
#         X_kp1 = X_vals[k + 1]

#         A, B = linearize_dynamics(X_k, U_k, Δt, l)

#         println("Step $k:")
#         println("  A size: ", size(A))  # Should print (nx, nx)
#         println("  B size: ", size(B))  # Should print (nx, nu)

#         d_k = residuals_at_k(X_k, U_k, X_kp1, Δt, l)
#         println("  Residual (d_k) size: ", size(d_k))  # Should match (nx,)

#         push!(d_list, d_k)
#         push!(D_blocks, hcat(A, -I(nx)))
#         push!(D_controls_blocks, B)
#     end

#     d = vcat(d_list...)
#     D = vcat(D_blocks...)
#     D_controls = vcat(D_controls_blocks...)

#     println("Final sizes:")
#     println("  d size: ", size(d))
#     println("  D size: ", size(D))
#     println("  D_controls size: ", size(D_controls))

#     return d, D, D_controls
# end

# function projection(X_vals, U_vals, λ_vals, tol, solver::iLQRSolver, ilqr_params::Dict{Symbol, Any})
#     """
#     Refines state and control trajectories by enforcing constraints.
#     Arguments:
#     - X_vals: Initial state trajectory.
#     - U_vals: Initial control trajectory.
#     - λ_vals: Lagrange multipliers (not used here but reserved for extensibility).
#     - tol: Residual norm tolerance.
#     - solver: Solver object containing problem dimensions and precomputed data.
#     - ilqr_params: Dictionary of hyperparameters.
#     """
#     println("Projection: Initializing...")

#     # Extract Hyperparameters
#     ϵ = ilqr_params[:rho_min]             # Regularization term for Hessian
#     conv_rate_tol = ilqr_params[:epsilon_grad]  # Convergence rate tolerance

#     # Dimensions
#     nx = size(X_vals[1], 1)  # State dimension
#     nu = size(U_vals[1], 1)  # Control dimension

#     # Step 1: Invert the full Hessian (state + control)
#     println("Step 1: Inverting the full Hessian...")
#     H_inv_blocks = Matrix{Float64}[]  # Initialize explicitly as Vector of Float64 matrices
#     for k in 1:solver.N
#         Qxx_k = solver.Q_expansion[k].xx  # Hessian w.r.t. states
#         Quu_k = solver.Quu_reg[k]          # Hessian w.r.t. controls
#         Qux_k = solver.Q_expansion[k].ux  # Cross-term Hessian (states-controls)

#         # Combine into a full block Hessian
#         H_k = [Qxx_k Qux_k'; Qux_k Quu_k]

#         # Regularize and invert the block Hessian
#         if cond(H_k) > 1e10
#             println("Regularizing ill-conditioned Hessian at step $k.")
#             push!(H_inv_blocks, inv(H_k + ϵ * I))
#         else
#             push!(H_inv_blocks, inv(H_k))
#         end
#     end
#     # Add a zero block for the final state (no controls at the last timestep)
#     final_state_block = zeros(size(solver.Q_expansion[1].xx))  # Assume consistent state size
#     push!(H_inv_blocks, inv(final_state_block + ϵ * I))  # Regularize final state block if needed

#     # Create the full block-diagonal Hessian inverse
#     H_inv = create_blockdiag(H_inv_blocks)

#     println("The type of H_inv_blocks is: ", typeof(H_inv_blocks))
#     println("Step 1 Complete: Full Hessian Inverted.")
#     println("Hessian dimensions: ", size(H_inv))

    

#     # Flatten X and U into Y
#     Y = vcat(vcat(X_vals...), vcat(U_vals...))
#     Y_new = copy(Y)
#     println("Initial Y vector created. Length: $(length(Y))")

# # Projection Loop
#     prev_v = Inf  # To track the previous residual norm
#     stagnation_tolerance = 0.1  # Define the tolerance for stagnation
#     while true
#         println("Step 2: Linearizing constraints...")
#         d, D = linearize_active_constraints(X_vals, U_vals, solver)  # Updated: Single D returned
#         println("Constraint Jacobians linearized. Residual size: $(size(d))")

#         # Step 3: Compute Schur Complement
#         println("Step 3: Computing Schur complement...")
#         S = D * H_inv * D'  # Use updated D here
#         println("Schur complement dimensions: $(size(S))")

#         # Step 4: Check Residual Norm
#         v = maximum(abs.(d))
#         println("Residual norm (v): $v")
        
#         # New: Check for stagnation
#         if abs(v - prev_v) < stagnation_tolerance && v <= tol
#             println("Projection terminated due to residual stagnation. Residual norm: $v")
#             break
#         end

#         # Existing condition: Check for convergence
#         if v <= tol
#             println("Projection converged. Residual norm within tolerance.")
#             break
#         end

#         # Step 5: Line Search Loop
#         println("Step 5: Entering line search loop...")
#         r = Inf
#         max_iters = ilqr_params[:line_search_max_iters]  # Add line search limit
#         iter_count = 0
#         while v > tol && r > conv_rate_tol && iter_count < max_iters
#             println("Performing line search...")
#             Y_new, v_new = projection_line_search(Y, S, H_inv, D, d, v, nx, nu, solver)

#             println("Line search completed. Residual norm after step (v_new): ", v_new)
#             # Update convergence rate and norms
#             r = log(v_new / v)
#             v = v_new
#             Y = copy(Y_new)
#             iter_count += 1

#             println("Residual norm updated: $v, Convergence rate: $r")
#         end

#         # Update the previous residual norm
#         prev_v = v
#     end


#     # Decompose Y back into X and U
#     println("Decomposing Y into X and U...")

#     X_new = [Y[(i-1)*nx + 1:i*nx] for i in 1:solver.N+1]
#     U_new = [Y[solver.N*nx + (i-1)*nu + 1:solver.N*nx + i*nu] for i in 1:solver.N]

#     println("Projection completed successfully.")
#     return X_new, U_new
# end

function projection(X_vals, U_vals, λ_vals, tol, solver::iLQRSolver, ilqr_params::Dict{Symbol, Any})

    """
    Refines state and control trajectories by enforcing constraints.
    Arguments:
    - X_vals: Initial state trajectory.
    - U_vals: Initial control trajectory.
    - λ_vals: Lagrange multipliers (not used here but reserved for extensibility).
    - tol: Residual norm tolerance.
    - solver: Solver object containing problem dimensions and precomputed data.
    """
    println("Projection: Initializing...")

    # Hardcoded Parameters
    ϵ = 1e-6  # Regularization term for Hessian
    stagnation_tolerance = 1e-3  # Relaxed stagnation tolerance
    max_projection_iters = 50  # Maximum projection iterations
    max_line_search_iters = 20  # Maximum line search iterations
    conv_rate_tol = 1e-6  # Convergence rate tolerance for line search

    # Dimensions
    nx = size(X_vals[1], 1)  # State dimension
    nu = size(U_vals[1], 1)  # Control dimension

    # Step 1: Invert the full Hessian (state + control)
    println("Step 1: Inverting the full Hessian...")
    H_inv_blocks = Matrix{Float64}[]  # Initialize explicitly as Vector of Float64 matrices
    for k in 1:solver.N
        Qxx_k = solver.Q_expansion[k].xx  # Hessian w.r.t. states
        Quu_k = solver.Quu_reg[k]          # Hessian w.r.t. controls
        Qux_k = solver.Q_expansion[k].ux  # Cross-term Hessian (states-controls)

        # Combine into a full block Hessian
        H_k = [Qxx_k Qux_k'; Qux_k Quu_k]

        # Regularize and invert the block Hessian
        if cond(H_k) > 1e10
            println("Regularizing ill-conditioned Hessian at step $k.")
            push!(H_inv_blocks, inv(H_k + ϵ * I))
        else
            push!(H_inv_blocks, inv(H_k))
        end
    end
    # Add a zero block for the final state (no controls at the last timestep)
    final_state_block = zeros(size(solver.Q_expansion[1].xx))  # Assume consistent state size
    push!(H_inv_blocks, inv(final_state_block + ϵ * I))  # Regularize final state block if needed

    # Create the full block-diagonal Hessian inverse
    H_inv = create_blockdiag(H_inv_blocks)

    println("Step 1 Complete: Full Hessian Inverted.")
    println("Hessian dimensions: ", size(H_inv))

    # Flatten X and U into Y
    Y = vcat(vcat(X_vals...), vcat(U_vals...))
    println("Initial Y vector created. Length: $(length(Y))")

    # Projection Loop
    prev_v = Inf  # To track the previous residual norm
    iter_count = 0

    while iter_count < max_projection_iters
        iter_count += 1
        println("Projection Iteration: $iter_count")

        println("Step 2: Linearizing constraints...")
        d, D = linearize_active_constraints(X_vals, U_vals, solver)
        println("Constraint Jacobians linearized. Residual size: $(size(d))")

        # Step 3: Compute Schur Complement
        println("Step 3: Computing Schur complement...")
        S = D * H_inv * D'
        println("Schur complement dimensions: $(size(S))")

        # Step 4: Check Residual Norm
        v = maximum(abs.(d))
        println("Residual norm (v): $v")

        # Check for convergence and stagnation
        if v <= tol
            println("Projection converged. Residual norm within tolerance: $v")
            break
        elseif abs(v - prev_v) < stagnation_tolerance
            println("Projection terminated due to stagnation. Residual norm: $v")
            break
        end

        # Step 5: Line Search Loop
        println("Step 5: Entering line search loop...")
        line_search_iters = 0
        success = false

        while line_search_iters < max_line_search_iters
            line_search_iters += 1
            println("Line search iteration: $line_search_iters")

            Y_new, v_new = projection_line_search(Y, S, H_inv, D, d, v, nx, nu, solver)
            println("Line search completed. Residual norm after step (v_new): $v_new")

            if v_new < v
                println("Line search successful. Residual norm reduced.")
                v = v_new
                Y = copy(Y_new)
                success = true
                break
            else
                println("Line search failed. Trying again...")
            end
        end

        if !success
            println("Line search failed to reduce residual. Stopping projection.")
            break
        end

        # Update the previous residual norm
        prev_v = v
    end

    # Decompose Y back into X and U
    println("Decomposing Y into X and U...")
    X_new = [Y[(i-1)*nx + 1:i*nx] for i in 1:solver.N+1]
    U_new = [Y[solver.N*nx + (i-1)*nu + 1:solver.N*nx + i*nu] for i in 1:solver.N]

    println("Projection completed successfully after $iter_count iterations.")
    return X_new, U_new
end # End of projection function

function linearize_active_constraints(X_vals, U_vals, solver)
    """
    Linearize the active constraints for the given trajectory.
    - X_vals: State trajectory.
    - U_vals: Control trajectory.
    - solver: iLQR solver object with problem dimensions.
    Returns:
    - d: Concatenated residuals vector.
    - D: Concatenated Jacobian matrix w.r.t. all optimization variables (states + controls).
    """
    N = solver.N  # Number of time steps
    Δt = solver.Δt
    l = solver.l

    # Dimensions
    nx = size(X_vals[1], 1)  # State dimension
    nu = size(U_vals[1], 1)  # Control dimension
    total_vars = solver.N * (nx + nu) + nx  # Full trajectory dimensions

    # Initialize concatenated residuals and Jacobians
    d_list = []
    D_blocks = []

    for k in 1:N
        X_k = X_vals[k]
        U_k = U_vals[k]
        X_kp1 = X_vals[k + 1]

        # Compute Jacobians
        A, B = linearize_dynamics(X_k, U_k, Δt, l)

        # Compute residuals
        d_k = residuals_at_k(X_k, U_k, X_kp1, Δt, l)

        # Construct D_k with zero-padding for global dimensions
        D_k = zeros(nx, total_vars)
        state_start_idx = (k - 1) * nx + 1
        state_end_idx = k * nx
        control_start_idx = solver.N * nx + (k - 1) * nu + 1
        control_end_idx = control_start_idx + nu - 1
        next_state_idx = k * nx + 1

        # Place A, B, and -I in the correct global positions
        D_k[:, state_start_idx:state_end_idx] .= A
        D_k[:, control_start_idx:control_end_idx] .= B
        D_k[:, next_state_idx:next_state_idx + nx - 1] .= -I(nx)

        # Append to lists
        push!(d_list, d_k)
        push!(D_blocks, D_k)
    end

    # Concatenate residuals and Jacobians
    d = vcat(d_list...)
    D = vcat(D_blocks...)

    println("Final Residuals (d) size: ", size(d))
    println("Final Jacobian (D) size: ", size(D))

    return d, D
end




function residuals_at_k(X_k, U_k, X_kp1, Δt, l)
    """
    Compute the residual at time step k.
    - X_k: Current state.
    - U_k: Current control input.
    - X_kp1: Next state from trajectory.
    - Δt: Time step size.
    - l: Vehicle length.
    """
    X_pred = vehicle_dynamics(X_k, U_k, Δt, l)  # Predicted next state
    return X_kp1 - X_pred  # Residual
end


# Helper function to create a block-diagonal matrix
function create_blockdiag(blocks::Vector{Matrix{Float64}})
    total_size = sum(size(block)[1] for block in blocks)
    blockdiag_matrix = zeros(Float64, total_size, total_size)
    
    offset = 1
    for (i, block) in enumerate(blocks)
        block_size = size(block, 1)
        println("Placing block $i of size $block_size at offset $offset")
        blockdiag_matrix[offset:offset+block_size-1, offset:offset+block_size-1] .= block
        offset += block_size
    end
    
    return blockdiag_matrix
end




