#pragma once

#include "solvers/Solver.h"
#include <string>
#include <unordered_map>
#include <memory>
#include <functional>

/**
 * Registry for all available solver types.
 * 
 * This class implements the Registry pattern to maintain a collection of all available
 * solver types. Solvers can be registered by name and then instantiated on demand.
 */
class SolverRegistry {
public:
    // Type definition for solver factory functions
    using SolverFactoryFunc = std::function<std::unique_ptr<Solver>(RigidBodySystem*)>;
    
    /**
     * Get the singleton instance of the registry.
     * 
     * @return Reference to the singleton registry
     */
    static SolverRegistry& getInstance() {
        static SolverRegistry instance;
        return instance;
    }
    
    /**
     * Register a new solver type.
     * 
     * @param name Name of the solver
     * @param factoryFunc Factory function to create instances of this solver
     * @return True if registration was successful, false if the name was already registered
     */
    bool registerSolver(const std::string& name, SolverFactoryFunc factoryFunc) {
        if (m_solverFactories.find(name) != m_solverFactories.end()) {
            return false; // Already registered
        }
        
        m_solverFactories[name] = factoryFunc;
        return true;
    }
    
    /**
     * Check if a solver type is registered.
     * 
     * @param name Name of the solver
     * @return True if the solver is registered
     */
    bool isSolverRegistered(const std::string& name) const {
        return m_solverFactories.find(name) != m_solverFactories.end();
    }
    
    /**
     * Create a new instance of a registered solver.
     * 
     * @param name Name of the solver to create
     * @param rigidBodySystem The rigid body system to solve constraints for
     * @return Unique pointer to the created solver, or nullptr if not registered
     */
    std::unique_ptr<Solver> createSolver(const std::string& name, RigidBodySystem* rigidBodySystem) const {
        auto it = m_solverFactories.find(name);
        if (it == m_solverFactories.end()) {
            return nullptr; // Not registered
        }
        
        return it->second(rigidBodySystem);
    }
    
    /**
     * Get all registered solver names.
     * 
     * @return Vector of registered solver names
     */
    std::vector<std::string> getRegisteredSolverNames() const {
        std::vector<std::string> names;
        names.reserve(m_solverFactories.size());
        
        for (const auto& pair : m_solverFactories) {
            names.push_back(pair.first);
        }
        
        return names;
    }
    
private:
    // Private constructor for singleton
    SolverRegistry() = default;
    
    // Disable copy/move
    SolverRegistry(const SolverRegistry&) = delete;
    SolverRegistry& operator=(const SolverRegistry&) = delete;
    SolverRegistry(SolverRegistry&&) = delete;
    SolverRegistry& operator=(SolverRegistry&&) = delete;
    
    // Map of solver names to factory functions
    std::unordered_map<std::string, SolverFactoryFunc> m_solverFactories;
};