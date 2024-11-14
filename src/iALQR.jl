function iLQR(vehicle_dynamics, cost_function, x0, U_init, max_iterations, tol)
    # Initialize state and control trajectories
    X = [x0]  # State trajectory, list of vectors for each time step
    U = U_init  # Control sequence, list of vectors for each time step
    T = length(U)  # Number of time steps

    # Initialize costs
    total_cost = cost_function(X, U)
   
    for iter in 1:max_iterations
        # Step 1: Forward pass to compute the current trajectory and cost
        X, U, current_cost = simulate_forward(X[1], U, vehicle_dynamics, cost_function)
       
        # Step 2: Linearize dynamics and approximate cost around the current trajectory
        A, B = linearize_dynamics(vehicle_dynamics, X, U)
        Q, R, q, r = quadratic_cost_approximation(cost_function, X, U)

        # Step 3: Backward pass to compute gains
        K, d = backward_pass(A, B, Q, R, q, r, T)

        # Step 4: Forward pass with updated control sequence
        X_new, U_new = forward_pass(X[1], K, d, X, U, vehicle_dynamics)

        # Step 5: Check for convergence
        new_cost = cost_function(X_new, U_new)
        if abs(new_cost - current_cost) < tol
            println("Converged after $iter iterations.")
            return X_new, U_new
        end
       
        # Update for next iteration
        X = X_new
        U = U_new
        current_cost = new_cost
    end
    println("Reached max iterations without full convergence.")
    return X, U
end

# Function Definitions (Simplified for Structure)

function simulate_forward(x0, U, vehicle_dynamics, cost_function)
    # Simulate system forward to get state trajectory and cost
    X = [x0]
    total_cost = 0
    for t in 1:length(U)
        x_next = vehicle_dynamics(X[end], U[t])  # Compute next state
        push!(X, x_next)
        total_cost += cost_function(X, U)  # Accumulate cost
    end
    return X, U, total_cost
end

function linearize_dynamics(vehicle_dynamics, X, U)
    # Compute A and B matrices (Jacobian of dynamics)
    A, B = [], []
    for t in 1:length(U)
        A_t = jacobian_x(vehicle_dynamics, X[t], U[t])  # State Jacobian
        B_t = jacobian_u(vehicle_dynamics, X[t], U[t])  # Control Jacobian
        push!(A, A_t)
        push!(B, B_t)
    end
    return A, B
end

function quadratic_cost_approximation(cost_function, X, U)
    # Approximate the cost function quadratically
    Q, R, q, r = [], [], [], []
    for t in 1:length(U)
        Q_t, R_t, q_t, r_t = compute_cost_terms(cost_function, X[t], U[t])
        push!(Q, Q_t)
        push!(R, R_t)
        push!(q, q_t)
        push!(r, r_t)
    end
    return Q, R, q, r
end

function backward_pass(A, B, Q, R, q, r, T)
    # Perform backward pass to calculate feedback gains
    K, d = [], []
    V_x, V_xx = q[T], Q[T]  # Initialize terminal cost-to-go
   
    for t in T-1:-1:1
        Q_x = q[t] + A[t]' * V_x
        Q_u = r[t] + B[t]' * V_x
        Q_xx = Q[t] + A[t]' * V_xx * A[t]
        Q_ux = B[t]' * V_xx * A[t]
        Q_uu = R[t] + B[t]' * V_xx * B[t]

        K_t = -inv(Q_uu) * Q_ux
        d_t = -inv(Q_uu) * Q_u

        V_x = Q_x + K_t' * Q_uu * d_t + K_t' * Q_u
        V_xx = Q_xx + K_t' * Q_uu * K_t + Q_ux' * K_t + K_t' * Q_ux

        push!(K, K_t)
        push!(d, d_t)
    end
    return reverse(K), reverse(d)
end

function forward_pass(x0, K, d, X, U, vehicle_dynamics)
    # Forward pass with updated controls
    X_new, U_new = [x0], []
    for t in 1:length(U)
        u_t = U[t] + K[t] * (X_new[end] - X[t]) + d[t]
        x_next = vehicle_dynamics(X_new[end], u_t)
        push!(X_new, x_next)
        push!(U_new, u_t)
    end
    return X_new, U_new
end

# Placeholder for Convergence Check
function has_converged(U, U_new, tol)
    # Check if control inputs have converged within tolerance
    max(norm(U_new[i] - U[i]) for i in 1:length(U)) < tol
end

