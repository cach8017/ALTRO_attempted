using IterativeLQR
using LinearAlgebra
using Plots

# Parameters
Δt = 0.2  # Timestep
T = Int(2 / Δt) + 1  # Horizon (2 seconds into the future)
num_state = 2  # [x, y]
num_action = 1  # [u]

# Dynamics
function car_dynamics(x, u)
    x_next = [
        x[1] + 30 * Δt;
        x[2] + 30 * Δt * sin(u[1])
    ]
    return x_next
end

car = Dynamics(car_dynamics, num_state, num_action)
dynamics = [car for _ in 1:T-1]

# Initial state and target
x1 = [0.0; 0.0]  # Start at (x, y) = (0, 0)
xT = [30.0; 1.0]  # Goal at x = 30, y = 1

# Rollout
ū = [zeros(num_action) for _ in 1:T-1]
x̄ = rollout(dynamics, x1, ū)

# Objective: Minimize deviation from centerline
objective = [
    [Cost((x, u) -> (x[2]^2) + 1e-3 * dot(u, u), num_state, num_action) for _ in 1:T-1]...,
    Cost((x, u) -> 1e3 * (x[2] - xT[2])^2, num_state, 0)
]

# Constraints
constraints = [
    [Constraint((x, u) -> [
        ul[1] - u[1];  # Lower steering limit
        u[1] - uu[1];  # Upper steering limit
        -2 - x[2];     # Stay within the lower road boundary
        x[2] - 2;      # Stay within the upper road boundary
    ], num_state, num_action, indices_inequality=collect(1:4)) for _ in 1:T-1]...,
    Constraint((x, u) -> [
        x[2] - xT[2];  # Deviation at the goal
    ], num_state, 0, indices_inequality=collect(1:1))
]


# Solver
solver = Solver(dynamics, objective, constraints)
initialize_controls!(solver, ū)
initialize_states!(solver, x̄)

# Solve
solve!(solver)

# Extract solution
x_sol, u_sol = get_trajectory(solver)


# Extract x and y components from x_sol
x_vals = [x[1] for x in x_sol]  # Longitudinal position
y_vals = [x[2] for x in x_sol]  # Transverse position

# Plot x trajectory
p1 = plot(x_vals, title="Longitudinal Position (x)", xlabel="Time Step", ylabel="Position (x)", legend=false)
display(p1)  # Display x trajectory

# Plot y trajectory
p2 = plot(y_vals, title="Transverse Position (y)", xlabel="Time Step", ylabel="Position (y)", legend=false)
display(p2)  # Display y trajectory

# Extract control inputs
u_vals = [u[1] for u in u_sol]

# Plot control inputs
p3 = plot(u_vals, title="Control Inputs (Steering Angle)", xlabel="Time Step", ylabel="Steering Angle", linetype=:steppost, legend=false)
display(p3)  # Display control inputs

using BenchmarkTools
info = @benchmark solve!($solver, x̄, ū) setup=(x̄=deepcopy(x̄), ū=deepcopy(ū))