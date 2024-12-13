# Dynamics function
function vehicle_dynamics(x, u, Δt, l = 2.5)
    px, py, v, a, θ, φ = x
    η, ω = u  # Control inputs

    # Discretized dynamics
    px_next = px + Δt * v * cos(θ)
    py_next = py + Δt * v * sin(θ)
    v_next = v + Δt * a
    a_next = a + Δt * η
    θ_next = θ + Δt * (v * tan(φ) / l)
    φ_next = φ + Δt * ω

    return [px_next, py_next, v_next, a_next, θ_next, φ_next]
end

function modified_dynamics_with_slack(x, u, s, Δt, l)
    """
    Compute the next state with slack controls.

    Arguments:
    - x: Current state.
    - u: Current control input.
    - s: Slack control.
    - Δt: Time step duration.
    - l: Vehicle wheelbase.

    Returns:
    - x_next: Next state.
    """
    return vehicle_dynamics(x, u, Δt, l) + s
end


# Linearize dynamics
function linearize_dynamics(X, U, Δt, l)
    px, py, v, a, θ, φ = X
    η, ω = U

    # Ensure `A` is a matrix with dimensions (nx, nx)
    A = [
        1  0  Δt * cos(θ)  0  -Δt * v * sin(θ)  0;
        0  1  Δt * sin(θ)  0   Δt * v * cos(θ)  0;
        0  0  1            Δt  0               0;
        0  0  0            1   0               0;
        0  0  Δt / l * tan(φ)  0  1  Δt * v / (l * cos(φ)^2);
        0  0  0            0   0               1
    ]

    # Ensure `B` is a matrix with dimensions (nx, nu)
    B = [
        0  0;
        0  0;
        0  0;
        Δt 0;
        0  0;
        0  Δt
    ]

    return A, B
end



# Simulate helper function
function simulate(x0, U, Δt, l)
    X = [x0]
    for u in U
        x_next = vehicle_dynamics(X[end], u, Δt, l)
        push!(X, x_next)
    end
    return X
end
