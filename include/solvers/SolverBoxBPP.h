#pragma once

#include "solvers/Solver.h"
#include <Eigen/Dense>
#include <string>

/**
 * Box-constrained Block Principal Pivoting (Box-BPP) solver for Linear Complementarity Problem.
 * 
 * This solver is especially effective for contact problems with friction, using
 * a box approximation of the friction cone and a pivoting strategy to efficiently
 * solve the resulting mixed linear complementarity problem (MLCP).
 */
class SolverBoxBPP : public Solver {
public:
    /**
     * Constructor
     * 
     * @param rigidBodySystem The rigid body system to solve constraints for
     */
    SolverBoxBPP(RigidBodySystem* rigidBodySystem);
    
    /**
     * Virtual destructor
     */
    virtual ~SolverBoxBPP() = default;
    
    /**
     * Get the name of this solver
     * 
     * @return Solver name
     */
    virtual const std::string& getName() const override;
    
    /**
     * Solve the constraints using the Box-BPP algorithm
     * 
     * @param h Time step size
     */
    virtual void solve(float h) override;
    
    /**
     * Enable or disable the smooth friction transition model
     * 
     * @param enable Whether to enable smooth friction
     */
    void enableSmoothFriction(bool enable) { m_useSmoothFriction = enable; }
    
    /**
     * Check if the smooth friction transition model is enabled
     * 
     * @return True if smooth friction is enabled
     */
    bool isSmoothFrictionEnabled() const { return m_useSmoothFriction; }
    
    /**
     * Set the velocity threshold for the smooth friction transition
     * 
     * @param threshold Velocity threshold (default: 0.1)
     */
    void setFrictionVelocityThreshold(float threshold) { m_frictionVelocityThreshold = threshold; }
    
    /**
     * Get the current friction velocity threshold
     * 
     * @return Current velocity threshold
     */
    float getFrictionVelocityThreshold() const { return m_frictionVelocityThreshold; }

private:
    /**
     * Index set types for the Block Principal Pivoting algorithm
     */
    enum IndexSet { kFree = 0, kLower, kUpper, kIgnore };
    
    /**
     * Pivot variables between free and tight sets based on the LCP conditions
     * 
     * @param idx Index set for variables
     * @param x Current solution vector
     * @param lower Lower bounds vector
     * @param upper Upper bounds vector
     * @param residual Residual vector (A*x - b)
     * @return Number of pivots performed
     */
    unsigned int pivotVariables(Eigen::VectorXi& idx, 
                              Eigen::VectorXf& x, 
                              const Eigen::VectorXf& lower, 
                              const Eigen::VectorXf& upper, 
                              const Eigen::VectorXf& residual);
    
    /**
     * Find the indices of tight variables (at a bound)
     * 
     * @param idx Index set
     * @param tightIdx Output vector of tight indices
     * @return Number of tight variables
     */
    unsigned int findTightIndices(const Eigen::VectorXi& idx, std::vector<int>& tightIdx) const;
    
    /**
     * Find the indices of free variables (not at a bound)
     * 
     * @param idx Index set
     * @param freeIdx Output vector of free indices
     * @return Number of free variables
     */
    unsigned int findFreeIndices(const Eigen::VectorXi& idx, std::vector<int>& freeIdx) const;
    
    /**
     * Solve the principal subproblem for the free variables
     * 
     * @param A System matrix
     * @param b RHS vector
     * @param idx Index set
     * @param lower Lower bounds
     * @param upper Upper bounds
     * @param x Solution vector (updated in-place)
     */
    void solvePrincipalSubproblem(const Eigen::MatrixXf& A, 
                                 const Eigen::VectorXf& b, 
                                 const Eigen::VectorXi& idx, 
                                 const Eigen::VectorXf& lower, 
                                 const Eigen::VectorXf& upper, 
                                 Eigen::VectorXf& x);

private:
    bool m_useSmoothFriction;           // Whether to use smooth friction transition
    float m_frictionVelocityThreshold;  // Velocity threshold for friction transition
    static const std::string m_name;    // Solver name
};