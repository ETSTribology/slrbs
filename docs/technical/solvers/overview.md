# Solver Overview

SLRBS implements several numerical solvers for resolving constraints in rigid body simulations. This document provides an overview of the available solvers and their characteristics.

## Available Solvers

| Solver | Description | Best For |
|--------|-------------|----------|
| [PGS (Projected Gauss-Seidel)](pgs.md) | Standard iterative constraint solver | General purpose, robust |
| [Conjugate Gradient](conjugate-gradient.md) | Krylov subspace method | Smooth dynamics without contacts |
| [Conjugate Residual](conjugate-residual.md) | Variant of CG for non-symmetric systems | Non-symmetric constraints |
| [BPP (Block Principal Pivoting)](bpp.md) | Direct LCP solver | High-accuracy solutions |
| [PGSSM](pgssm.md) | PGS with subspace minimization | Fast convergence with contacts |

## Solver Selection

The choice of solver depends on the specific requirements of your simulation:

- **PGS**: Good all-around solver, handles contacts well
- **CG**: Fast for joint-only simulations, not suitable for contacts
- **CR**: Handles non-symmetric constraints better than CG
- **BPP**: Most accurate but computationally expensive
- **PGSSM**: Good convergence with mixed constraints

## Solver Parameters

Most solvers support the following parameters:

- **Iterations**: Number of iterations to perform
- **Tolerance**: Error tolerance for early termination
- **Warm Starting**: Whether to use previous solution as initial guess

## Constraint Formulation

All solvers in SLRBS operate on the following constraint formulation:

$$J \cdot v = b$$

Where:
- $J$ is the constraint Jacobian matrix
- $v$ is the velocity vector
- $b$ is the constraint right-hand side

For contact constraints, the complementarity condition is:

$$0 \leq \lambda \perp J \cdot v - b \geq 0$$

Where $\lambda$ is the Lagrange multiplier (constraint force).

## Solver Implementation

Each solver is implemented as a derived class of the base `Solver` class:

```cpp
class Solver {
public:
    virtual ~Solver() = default;
    
    // Main solve function
    virtual void solve(RigidBodySystem& system) = 0;
    
    // Set maximum iterations
    void setMaxIterations(int iterations) {
        maxIterations = iterations;
    }
    
    // Set error tolerance
    void setTolerance(float tol) {
        tolerance = tol;
    }
    
protected:
    int maxIterations = 100;
    float tolerance = 1e-6f;
};
```

## Performance Considerations

- **Memory Usage**: PGS uses less memory than CG/CR/BPP
- **Computational Cost**: BPP > PGSSM > PGS > CR > CG
- **Parallelization**: CG/CR parallelize better than PGS
- **Convergence**: BPP converges in a fixed number of steps, iterative solvers may require many iterations
