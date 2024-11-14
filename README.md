# Pseudocode for Multiobjective Optimization using iALQR

## Overview
This pseudocode outlines the modular flow of the iLQR optimization process for autonomous vehicle overtaking, using multiobjective constraints. Each module (e.g., vehicle dynamics, cost function, constraints) interacts with the main `iLQR` function to compute an optimized trajectory and control sequence.

---

## Main `iLQR` Algorithm

1. **Initialize State and Control Trajectories**
   - Set `X` as the initial state trajectory list with starting state `x0`.
   - Set `U` as the initial control sequence `U_init`.
   - Define `T` as the number of time steps based on `U`.

2. **Initialize Total Cost**
   - Calculate `total_cost` using the `cost_function(X, U)` function.

3. **Start Optimization Loop** (up to `max_iterations`)
   - For each iteration in the range `1:max_iterations`:

     1. **Step 1: Forward Pass - Simulate Trajectory and Cost**
        - Call `simulate_forward(X[1], U, vehicle_dynamics, cost_function)` to:
          - Compute the full state trajectory `X` based on `vehicle_dynamics`.
          - Calculate the `current_cost` using `cost_function`.

     2. **Step 2: Linearize Dynamics**
        - Call `linearize_dynamics(vehicle_dynamics, X, U)` to:
          - Compute linearized matrices `A`, `B` for the system dynamics.

     3. **Step 3: Quadratic Cost Approximation**
        - Call `quadratic_cost_approximation(cost_function, X, U)` to:
          - Obtain quadratic approximations `Q`, `R`, `q`, `r` for the cost function around the current trajectory.

     4. **Step 4: Backward Pass - Compute Gains**
        - Call `backward_pass(A, B, Q, R, q, r, T)` to:
          - Compute feedback gain matrices `K` and feedforward terms `d` using dynamic programming.

     5. **Step 5: Forward Pass with Updated Controls**
        - Call `forward_pass(X[1], K, d, X, U, vehicle_dynamics)` to:
          - Generate updated state `X_new` and control sequence `U_new`.

     6. **Step 6: Convergence Check**
        - Calculate `new_cost` using `cost_function(X_new, U_new)`.
        - If the absolute difference `|new_cost - current_cost| < tol`:
          - Print convergence message and return `X_new`, `U_new`.

     7. **Step 7: Update Trajectories for Next Iteration**
        - Update `X = X_new`, `U = U_new`, `current_cost = new_cost`.

   - **End Loop**: If maximum iterations reached without convergence, print a message and return `X`, `U`.

---

## Workflow Summary
1. **Initialize**: Define initial trajectories and parameters.
2. **Optimize Trajectory**: Iterate through the iLQR loop, adjusting controls and states until convergence.
3. **Constraint Checking**: Use `check_constraints` to ensure all generated trajectories are feasible.
4. **Final Output**: Return optimized trajectory `X` and control sequence `U`.

---
