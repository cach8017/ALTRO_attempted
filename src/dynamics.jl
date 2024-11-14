function vehicle_dynamics(x, u, params)
    # Extract state and control variables
    px, py, v, a, θ, φ = x
    η, ω = u

    # Define equations of motion
    dpx = v * cos(θ)
    dpy = v * sin(θ)
    dv = a
    da = η
    dθ = v * tan(φ) / params.l
    dφ = ω

    # Return the new state as a vector
    return [dpx, dpy, dv, da, dθ, dφ]
end
