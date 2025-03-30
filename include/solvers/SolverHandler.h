#pragma once

#include "solvers/SolverRegistry.h"
#include "solvers/Solver.h"
#include <memory>
#include <string>
#include <unordered_map>

/**
 * Central handler for managing and accessing solver instances.
 * 
 * This class serves as the main entry point for creating and using solvers.
 * It caches solver instances to avoid repeated creation.
 */
class SolverHandler {
public:
    /**
     * Constructor.
     * 
     * @param rigidBodySystem The rigid body system to solve constraints for
     */
    explicit SolverHandler(RigidBodySystem* rigidBodySystem)
        : m_rigidBodySystem(rigidBodySystem), m_currentSolver(nullptr) {}
    
    /**
     * Get or create a solver by name.
     * 
     * @param name Name of the solver
     * @return Pointer to the solver, or nullptr if not found
     */
    Solver* getSolver(const std::string& name) {
        // Check if solver is already created
        auto it = m_solvers.find(name);
        if (it != m_solvers.end()) {
            m_currentSolver = it->second.get();
            return m_currentSolver;
        }
        
        // Create new solver
        auto solver = SolverRegistry::getInstance().createSolver(name, m_rigidBodySystem);
        if (!solver) {
            return nullptr; // Not registered
        }
        
        // Store and return
        m_currentSolver = solver.get();
        m_solvers[name] = std::move(solver);
        return m_currentSolver;
    }
    
    /**
     * Get the current active solver.
     * 
     * @return Pointer to the current solver, or nullptr if none is active
     */
    Solver* getCurrentSolver() const {
        return m_currentSolver;
    }
    
    /**
     * Set the current solver by name.
     * 
     * @param name Name of the solver to set as current
     * @return True if the solver was found and set as current
     */
    bool setCurrentSolver(const std::string& name) {
        Solver* solver = getSolver(name);
        if (!solver) {
            return false;
        }
        
        m_currentSolver = solver;
        return true;
    }
    
    /**
     * Solve using the current solver.
     * 
     * @param h Time step size
     * @return True if solving was successful
     */
    bool solve(float h) {
        if (!m_currentSolver) {
            return false;
        }
        
        m_currentSolver->solve(h);
        return true;
    }
    
    /**
     * Check if a solver with the specified name exists.
     * 
     * @param name Name of the solver to check
     * @return True if the solver exists
     */
    bool hasSolver(const std::string& name) const {
        return SolverRegistry::getInstance().isSolverRegistered(name);
    }
    
    /**
     * Get the names of all available solvers.
     * 
     * @return Vector of solver names
     */
    std::vector<std::string> getAvailableSolverNames() const {
        return SolverRegistry::getInstance().getRegisteredSolverNames();
    }
    
private:
    RigidBodySystem* m_rigidBodySystem;
    Solver* m_currentSolver;
    std::unordered_map<std::string, std::unique_ptr<Solver>> m_solvers;
};