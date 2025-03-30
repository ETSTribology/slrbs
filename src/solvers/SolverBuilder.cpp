#include "solvers/SolverBuilder.h"
#include "solvers/SolverBoxBPP.h"
#include "solvers/SolverBoxPGS.h"
#include "solvers/SolverProximal.h"
#include "friction/FrictionModels.h"

void SolverBuilder::configureSolverSpecific(Solver* solver) {
    // Apply friction strategy if specified
    std::unique_ptr<FrictionStrategy> frictionStrategy;
    
    if (m_customFrictionStrategy) {
        frictionStrategy = std::move(m_customFrictionStrategy);
    } else if (m_frictionStrategy) {
        frictionStrategy = FrictionRegistry::getInstance().createStrategy(*m_frictionStrategy);
    }
    
    // Configure smooth friction parameters if using SmoothFrictionTransition
    if (frictionStrategy && frictionStrategy->getName() == "SmoothTransition" && m_staticToKineticRatio) {
        auto* smoothStrategy = dynamic_cast<SmoothFrictionTransition*>(frictionStrategy.get());
        if (smoothStrategy) {
            smoothStrategy->setStaticToKineticRatio(*m_staticToKineticRatio);
        }
    }
    
    // Configure BoxBPP solver
    auto* boxBppSolver = dynamic_cast<SolverBoxBPP*>(solver);
    if (boxBppSolver) {
        if (m_useSmoothFriction) {
            boxBppSolver->enableSmoothFriction(*m_useSmoothFriction);
        }
        
        if (m_frictionVelocityThreshold) {
            boxBppSolver->setFrictionVelocityThreshold(*m_frictionVelocityThreshold);
        }
        
        if (frictionStrategy) {
            boxBppSolver->setFrictionStrategy(std::move(frictionStrategy));
        }
        
        if (m_tolerance) {
            boxBppSolver->setTolerance(*m_tolerance);
        }
        
        if (m_warmStartingFactor) {
            boxBppSolver->setWarmStartingFactor(*m_warmStartingFactor);
        }
        
        return;
    }
    
    // Configure BoxPGS solver
    auto* boxPgsSolver = dynamic_cast<SolverBoxPGS*>(solver);
    if (boxPgsSolver) {
        if (frictionStrategy) {
            boxPgsSolver->setFrictionStrategy(std::move(frictionStrategy));
        }
        
        if (m_relaxationApproach && m_relaxationOmega) {
            boxPgsSolver->setRelaxation(*m_relaxationApproach, *m_relaxationOmega);
        }
        
        if (m_useSequentialImpulses) {
            boxPgsSolver->setUseSequentialImpulses(*m_useSequentialImpulses);
        }
        
        if (m_warmStartingFactor) {
            boxPgsSolver->setWarmStartingFactor(*m_warmStartingFactor);
        }
        
        if (m_tolerance) {
            boxPgsSolver->setTolerance(*m_tolerance);
        }
        
        return;
    }
    
    // Configure Proximal solver
    auto* proximalSolver = dynamic_cast<SolverProximal*>(solver);
    if (proximalSolver) {
        if (m_exportEnabled) {
            proximalSolver->enableDataExport(*m_exportEnabled);
            
            if (m_exportPath) {
                proximalSolver->setExportPath(*m_exportPath);
            }
        }
        
        if (frictionStrategy) {
            proximalSolver->setFrictionStrategy(std::move(frictionStrategy));
        }
        
        if (m_tolerance) {
            proximalSolver->setTolerance(*m_tolerance);
        }
        
        return;
    }
    
    // Apply post-solve callback if specified
    if (m_postSolveCallback) {
        // We need to wrap the original solve method to add the callback
        // Save the original solve method
        auto originalSolve = solver->solve;
        
        // Replace with our wrapped version
        solver->solve = [originalSolve, callback = *m_postSolveCallback, solver](float h) {
            originalSolve(h);
            callback(solver, h);
        };
    }
}