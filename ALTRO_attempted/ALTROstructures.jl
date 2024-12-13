struct DynamicsExpansion
    fx::Matrix{Float64}  # Jacobian of dynamics w.r.t state
    fu::Matrix{Float64}  # Jacobian of dynamics w.r.t control
end

struct CostExpansion
    x::Vector{Float64}       # Gradient of cost w.r.t state
    u::Vector{Float64}       # Gradient of cost w.r.t control
    xx::Matrix{Float64}      # Hessian of cost w.r.t state
    uu::Matrix{Float64}      # Hessian of cost w.r.t control
    ux::Matrix{Float64}      # Cross-term Hessian
end

mutable struct CostToGo
    x::Vector{Float64}       # Gradient of cost-to-go
    xx::Matrix{Float64}      # Hessian of cost-to-go
end

mutable struct Regularization
    ρ::Float64  # Penalty parameter (e.g., for augmented Lagrangian)
end
# Add a custom constructor for keyword arguments
function Regularization(; ρ::Float64)
    Regularization(ρ)
end

struct SolverOptions
    bp_reg_type::Symbol      # Regularization type (:state or :control)
end

# Add a custom constructor for keyword arguments
function SolverOptions(; bp_reg_type::Symbol)
    SolverOptions(bp_reg_type)
end

struct QtmpStruct
    xx::Matrix{Float64} 
    ux::Matrix{Float64}
    u::Vector{Float64}
end

# Add a custom constructor for keyword arguments
function QtmpStruct(; xx::Matrix{Float64}, ux::Matrix{Float64}, u::Vector{Float64})
    QtmpStruct(xx, ux, u)
end

mutable struct iLQRSolver
    N::Int                                  # Number of time steps
    D::Vector{DynamicsExpansion}            # Dynamics expansions (Jacobian matrices)
    Eerr::Vector{CostExpansion}             # Cost expansions for error terms
    S::Vector{CostToGo}                     # Cost-to-go structures
    Slack::Vector{Vector{Float64}}          # Slack variables for constraint violations
    Q_expansion::Vector{CostExpansion}      # Quadratic approximations of the cost-to-go
    K::Vector{Matrix{Float64}}              # Feedback gains
    d::Vector{Vector{Float64}}              # Feedforward controls
    Quu_reg::Vector{Matrix{Float64}}        # Regularized control cost Hessian
    Qux_reg::Matrix{Float64}                # Cross-term for state-control Hessian
    ΔV::Vector{Float64}                     # Change in cost-to-go
    Qtmp::QtmpStruct                        # Temporary storage for quadratic cost terms
    reg::Regularization                     # Regularization parameters
    opts::SolverOptions                     # Solver options (e.g., regularization type)
    Q_cost::Diagonal{Float64, Vector{Float64}}  # State cost weight matrix
    R::Diagonal{Float64, Vector{Float64}}       # Control cost weight matrix
    H::Matrix{Float64}                          # Cross-term weight matrix
    Qf::Diagonal{Float64, Vector{Float64}}      # Terminal state cost weight matrix
    Rs::Diagonal{Float64, Vector{Float64}}      # Slack cost weight matrix
    q::Vector{Float64}                          # Linear state cost vector
    r::Vector{Float64}                          # Linear control cost vector
    Δt::Float64                                 # Time step
    l::Float64                                  # Vehicle wheelbase
end



# Add a custom constructor for keyword arguments
function iLQRSolver(; 
    N::Int,
    D::Vector{DynamicsExpansion},
    Eerr::Vector{CostExpansion},
    S::Vector{CostToGo},
    Slack::Vector{Vector{Float64}},  # Add Slack as a field
    Q_expansion::Vector{CostExpansion},
    K::Vector{Matrix{Float64}},
    d::Vector{Vector{Float64}},
    Quu_reg::Vector{Matrix{Float64}},
    Qux_reg::Matrix{Float64},
    ΔV::Vector{Float64},
    Qtmp::QtmpStruct,
    reg::Regularization,
    opts::SolverOptions,
    Q_cost::Diagonal{Float64, Vector{Float64}},
    R::Diagonal{Float64, Vector{Float64}},
    H::Matrix{Float64},
    Qf::Diagonal{Float64, Vector{Float64}},
    Rs::Diagonal{Float64, Vector{Float64}},
    q::Vector{Float64},
    r::Vector{Float64},
    Δt::Float64,
    l::Float64
)
    return iLQRSolver(
        N,
        D,
        Eerr,
        S,
        Slack,  # Add Slack initialization here
        Q_expansion,
        K,
        d,
        Quu_reg,
        Qux_reg,
        ΔV,
        Qtmp,
        reg,
        opts,
        Q_cost,
        R,
        H,
        Qf,
        Rs,
        q,
        r,
        Δt,
        l
    )
end




