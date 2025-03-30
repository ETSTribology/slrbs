# Projected Gauss-Seidel (PGS) Solver

The Projected Gauss-Seidel (PGS) solver is a standard iterative method for solving the Mixed Linear Complementarity Problem (MLCP) that arises in rigid body simulation with contacts.

## Algorithm Overview

PGS is based on the Gauss-Seidel algorithm for solving linear systems but includes a projection step to handle inequality constraints. The solver iteratively updates the constraint forces until convergence or until a maximum number of iterations is reached.

### Pseudocode

