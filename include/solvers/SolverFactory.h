#pragma once

#include "solvers/SolverRegistry.h"
#include "solvers/Solver.h"
#include "solvers/SolverBoxBPP.h"
#include "solvers/SolverBoxPGS.h"
#include "solvers/SolverConjGradient.h"
#include "solvers/SolverConjResidual.h"
#include "solvers/SolverPGSSM.h"
#include "solvers/SolverProximal.h"
#include <memory>

/**
 * Factory for creating and registering solver instances.
 * 
 * This class implements the Factory pattern to create different types of solvers
 * and automatically register them with the SolverRegistry.
 */
class SolverFactory {
public:
    /**
     * Initialize the factory and register all available solvers.
     * 
     * This method should be called once at program startup.
     */
    static void initialize() {
        auto& registry = SolverRegistry::getInstance();
        
        // Register all available solver types
        registry.registerSolver("BoxBPP", [](RigidBodySystem* system) {
            return std::make_unique<SolverBoxBPP>(system);
        });
        
        registry.registerSolver("BoxPGS", [](RigidBodySystem* system) {
            return std::make_unique<SolverBoxPGS>(system);
        });
        
        registry.registerSolver("ConjugateGradient", [](RigidBodySystem* system) {
            return std::make_unique<SolverConjGradient>(system);
        });
        
        registry.registerSolver("ConjugateResidual", [](RigidBodySystem* system) {
            return std::make_unique<SolverConjResidual>(system);
        });
        
        registry.registerSolver("PGSSM", [](RigidBodySystem* system) {
            return std::make_unique<SolverPGSSM>(system);
        });
        
        registry.registerSolver("Proximal", [](RigidBodySystem* system) {
            return std::make_unique<SolverProximal>(system);
        });
    }
    
    /**
     * Create a new solver instance by name.
     * 
     * @param name Name of the solver to create
     * @param rigidBodySystem The rigid body system to solve constraints for
     * @return Unique pointer to the created solver, or nullptr if not registered
     */
    static std::unique_ptr<Solver> createSolver(const std::string& name, RigidBodySystem* rigidBodySystem) {
        return SolverRegistry::getInstance().createSolver(name, rigidBodySystem);
    }
};