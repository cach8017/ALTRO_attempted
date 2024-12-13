function backward_pass(X, U, S, λ_vals, ρ, solver, ilqr_params, constraints)
    N = length(U)
    K = Vector{Matrix{Float64}}(undef, N)  # Feedback gains
    d = Vector{Vector{Float64}}(undef, N)  # Feedforward terms

    Q = ilqr_params[:Q]
    Qf = ilqr_params[:Qf]
    R = ilqr_params[:R]

    # Terminal cost expansion
    δx_N = X[end] - constraints[:X_desired][end]
    V_x = Qf * δx_N  # Gradient of terminal cost
    V_xx = Qf        # Hessian of terminal cost

    # Initialize ΔV terms
    # These are the expected cost reductions from the backward pass
    ΔV = Dict(
        :linear => 0.0,
        :quadratic => 0.0
    )

    println("Computing terminal cost expansion...")
    println("V_x dimensions: ", size(V_x))
    println("V_xx dimensions: ", size(V_xx))

    for k in N:-1:1
        println("Computing residuals for constraints at step $k...")
        g_k = compute_constraint_residuals_at_k(X, U, k, solver, constraints)
        println("Constraint residuals dimensions at step $k: ", size(g_k))

        g_x, g_u = compute_constraint_jacobians(X[k], U[k], solver, constraints)
        println("Constraint Jacobians dimensions at step $k: ", size(g_x), size(g_u))

        A, B = linearize_dynamics(X[k], U[k], solver.Δt, solver.l)
        println("g_x dimensions: ", size(g_x))
        println("g_u dimensions: ", size(g_u))
        println("g_k dimensions: ", size(g_k))
        println("λ_vals[$k] dimensions: ", size(λ_vals[k]))
        println("V_x dimensions: ", size(V_x))
        println("A dimensions: ", size(A))
        println("B dimensions: ", size(B))
        println("Q dimensions: ", size(Q))
        println("R dimensions: ", size(R))
        println("Qf dimensions: ", size(Qf))
        println("V_xx dimensions: ", size(V_xx))

        # Quadratic expansions of the augmented cost
        Q_x = Q * (X[k] - constraints[:X_desired][k]) + A' * V_x + g_x' * λ_vals[k] + ρ * g_x' * g_k
        Q_u = R * U[k] + B' * V_x + g_u' * λ_vals[k] + ρ * g_u' * g_k
        Q_xx = Q + A' * V_xx * A + ρ * (g_x' * g_x)
        Q_uu = R + B' * V_xx * B + ρ * (g_u' * g_u)
        Q_ux = B' * V_xx * A + ρ * (g_u' * g_x)

        println("Q_x dimensions at step $k: ", size(Q_x))
        println("Q_u dimensions at step $k: ", size(Q_u))
        println("Q_xx dimensions: ", size(Q_xx))
        println("Q_uu dimensions: ", size(Q_uu))
        println("Q_ux dimensions: ", size(Q_ux))

        # Ensure that Quu is invertible
        if rank(Q_uu) < size(Q_uu, 1)
            println("Quu is singular at step $k, adding regularization.")
            Q_uu += ρ * I(size(Q_uu, 1))  # Add small regularization
        end

        # Compute feedback gains and feedforward terms
        K[k] = -Q_uu \ Q_ux
        d[k] = -Q_uu \ Q_u

        # Update value function
        V_x = Q_x + K[k]' * Q_uu * d[k]
        V_xx = Q_xx + K[k]' * Q_uu * K[k]

        # Compute contributions to ΔV

        ΔV[:linear] += d[k]' * Q_u
        ΔV[:quadratic] += d[k]' * Q_uu * d[k]
    end

    return true, K, d, ΔV
end


function compute_constraint_residuals_at_k(X, U, k, solver, constraints)
    println("Computing residuals for constraints at step $k...")

    # Extract current state, control, and next state
    x_k = X[k]
    u_k = U[k]
    x_next_k = X[k + 1]

    # Vehicle dynamics constraint residual
    x_dynamics = vehicle_dynamics(x_k, u_k, solver.Δt, solver.l)
    dynamics_residual = x_next_k - x_dynamics

    # Desired trajectory residual
    desired_residual = x_k - constraints[:X_desired][k]

    # Combine residuals
    residual = vcat(dynamics_residual, desired_residual)
    println("Residuals dimensions: ", size(residual))
    return residual
end

function compute_constraint_jacobians(x_k, u_k, solver, constraints)
    # Extract state and control dimensions
    nx = length(x_k)
    nu = length(u_k)

    # Initialize jacobians
    g_x = zeros(nx + nx, nx)  # Dynamics + Desired trajectory
    g_u = zeros(nx + nx, nu)  # Dynamics only

    # Dynamics Jacobians (∂f/∂x, ∂f/∂u)
    A, B = linearize_dynamics(x_k, u_k, solver.Δt, solver.l)

    # Add dynamics jacobians
    g_x[1:nx, :] .= A
    g_u[1:nx, :] .= B

    # Desired trajectory Jacobians (identity matrix for state deviation)
    g_x[nx+1:end, :] .= I(nx)

    return g_x, g_u
end




