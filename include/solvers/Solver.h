#pragma once

#include "util/Types.h"
#include <string>
#include <chrono>

#ifdef USE_OPENMP
#include <omp.h>
#endif

class RigidBodySystem;

/**
 * Base class for all constraint solvers.
 * 
 * This class provides a common interface and shared functionality for all
 * constraint solvers in the system. Solvers compute constraint forces and impulses
 * that maintain joint limits and prevent bodies from interpenetrating.
 */
class Solver {
public:
    /**
     * Constructor.
     * 
     * @param rigidBodySystem The rigid body system to solve constraints for
     */
    Solver(RigidBodySystem* rigidBodySystem);
    
    /**
     * Virtual destructor for proper cleanup of derived classes.
     */
    virtual ~Solver() = default;
    
    /**
     * Sets the maximum number of iterations for the solver method.
     * 
     * @param maxIter Maximum number of iterations
     */
    void setMaxIter(int maxIter) { m_maxIter = maxIter; }
    
    /**
     * Returns the maximum number of iterations.
     * 
     * @return Maximum iterations
     */
    int getMaxIter() const { return m_maxIter; }
    
    /**
     * Set the number of threads to use when OpenMP is enabled.
     * Has no effect if OpenMP is not available.
     * 
     * @param numThreads Number of threads to use, 0 for auto-detection
     */
    void setNumThreads(int numThreads);
    
    /**
     * Get the current number of threads used for parallel execution.
     * 
     * @return Number of threads being used
     */
    int getNumThreads() const;
    
    /**
     * Enable or disable parallel computation.
     * 
     * @param enable Whether to enable parallel computation
     */
    void enableParallelComputation(bool enable) { m_parallelEnabled = enable; }
    
    /**
     * Check if parallel computation is enabled.
     * 
     * @return True if parallel computation is enabled
     */
    bool isParallelComputationEnabled() const { return m_parallelEnabled; }
    
    /**
     * Enable or disable performance measurement.
     * 
     * @param enable Whether to enable performance measurement
     */
    void enablePerformanceMeasurement(bool enable) { m_measurePerformance = enable; }
    
    /**
     * Get the time in milliseconds taken by the last solve operation.
     * 
     * @return Last solve time in milliseconds
     */
    double getLastSolveTime() const { return m_lastSolveTime; }
    
    /**
     * Get the name of this solver.
     * 
     * @return Solver name
     */
    virtual const std::string& getName() const = 0;
    
    /**
     * Solve the constraint forces in the rigid body system.
     * 
     * @param h Time step size
     */
    virtual void solve(float h) = 0;

protected:
    /**
     * Start performance timing.
     */
    void startTiming();
    
    /**
     * End performance timing and store result.
     */
    void endTiming();
    
    /**
     * Get the number of threads to use for parallel regions.
     * 
     * @return Number of threads to use
     */
    int getEffectiveThreadCount() const;

protected:
    RigidBodySystem* m_rigidBodySystem;    // The rigid body system to solve
    int m_maxIter;                         // Maximum number of iterations
    bool m_parallelEnabled;                // Whether parallel computation is enabled
    bool m_measurePerformance;             // Whether to measure performance
    double m_lastSolveTime;                // Last solve time in milliseconds
    int m_numThreads;                      // Number of threads to use (0 = auto)
    
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> m_startTime;
};