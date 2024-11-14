# Multiobjective Optimization for Overtaking Manuever using iALQR
---

## Workflow Summary
1. **Initialize**: Define initial trajectories and parameters.
2. **Optimize Trajectory**: Iterate through the iLQR loop, adjusting controls and states until convergence.
3. **Constraint Checking**: Use `check_constraints` to ensure all generated trajectories are feasible.
4. **Final Output**: Return optimized trajectory `X` and control sequence `U`.

---
