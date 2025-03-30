#include "viewer/SimulationManager.h"
#include "viewer/SimViewer.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBodyState.h"
#include "viewer/RigidBodyRenderer.h"

#include <chrono>
#include <iostream>

namespace slrbs {

SimulationManager::SimulationManager(SimViewer* viewer)
    : viewer(viewer), timeScale(1.0f), timeStep(1.0f/60.0f), substeps(1),
      adaptiveTimesteps(false), alpha(0.01f), geometricStiffnessDamping(false),
      paused(true), stepOnce(false), dynamicsTime(0.0f), kineticEnergy(0.0f),
      constraintError(0.0f), frameCount(0) {
}

SimulationManager::~SimulationManager() {
}

void SimulationManager::initialize() {
    // Create the initial reset state
    resetState = std::make_unique<RigidBodySystemState>(viewer->getRigidBodySystem());
    
    // Set up preStep function for geometric stiffness damping
    viewer->getRigidBodySystem().setPreStepFunc([this](RigidBodySystem& system, float dt) {
        if (!geometricStiffnessDamping) return;
        
        auto& bodies = system.getBodies();
        auto& joints = system.getJoints();
        auto& contacts = system.getContacts();

        // Reset geometric stiffness damping values
        for (auto* body : bodies) {
            body->gsDamp.setZero();
        }

        // Compute geometric stiffness contributions
        for (auto* joint : joints) {
            joint->computeGeometricStiffness();
            joint->body0->gsSum += joint->G0;
            joint->body1->gsSum += joint->G1;
        }

        for (auto* contact : contacts) {
            contact->computeGeometricStiffness();
            contact->body0->gsSum += contact->G0;
            contact->body1->gsSum += contact->G1;
        }

        // Apply geometric stiffness damping
        for (auto* body : bodies) {
            if (body->fixed) continue;

            for (int c = 0; c < 3; c++) {
                const float m = body->I(c, c);
                const float k = 2.0f * body->gsSum.col(c + 3).norm();
                body->gsDamp(c) = std::max(0.0f, dt * dt * k - 4 * alpha * m);
            }
            
            // Update inertia matrix so solver has most recent gs damping information
            body->updateInertiaMatrix();
        }
    });
    
    std::cout << "SimulationManager initialized" << std::endl;
}

void SimulationManager::update(float deltaTime) {
    if (!paused || stepOnce) {
        auto start = std::chrono::high_resolution_clock::now();
        
        // Determine adaptive timesteps if enabled
        if (adaptiveTimesteps) {
            updateAdaptiveTimesteps();
        }
        
        // Apply time scaling
        float dt = timeStep * timeScale / static_cast<float>(substeps);
        
        // Step the simulation
        for (int i = 0; i < substeps; ++i) {
            viewer->getRigidBodySystem().step(dt);
        }
        
        // Calculate performance metrics
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        dynamicsTime = static_cast<float>(duration.count()) / 1000.0f;
        
        // Compute kinetic energy and constraint error
        computeKineticEnergy();
        computeConstraintError();
        
        // Increment frame counter
        ++frameCount;
        
        // Reset step once flag
        stepOnce = false;
    }
}

void SimulationManager::step() {
    stepOnce = true;
}

void SimulationManager::reset() {
    std::cout << "Resetting simulation..." << std::endl;
    
    // Reset simulation state
    resetState->restore(viewer->getRigidBodySystem());
    
    // Reset performance metrics
    dynamicsTime = 0.0f;
    frameCount = 0;
    kineticEnergy = 0.0f;
    constraintError = 0.0f;
    
    std::cout << "Simulation reset complete" << std::endl;
}

void SimulationManager::save() {
    std::cout << "Saving current simulation state..." << std::endl;
    resetState->save(viewer->getRigidBodySystem());
    std::cout << "Simulation state saved" << std::endl;
}

void SimulationManager::setSolverIterations(int iterations) {
    viewer->getRigidBodySystem().solverIter = iterations;
}

int SimulationManager::getSolverIterations() const {
    return viewer->getRigidBodySystem().solverIter;
}

void SimulationManager::setShowContacts(bool show) {
    viewer->getRenderer().setShowContacts(show);
}

bool SimulationManager::getShowContacts() const {
    // This would need to be stored in the renderer
    // For now, return a default value
    return true;
}

void SimulationManager::setShowJoints(bool show) {
    viewer->getRenderer().setShowJoints(show);
}

bool SimulationManager::getShowJoints() const {
    // This would need to be stored in the renderer
    // For now, return a default value
    return true;
}

void SimulationManager::setShowBoundingBoxes(bool show) {
    viewer->getConfig().rendering.showBoundingBoxes = show;
    viewer->getRenderer().setShowBoundingBoxes(show);
}

bool SimulationManager::getShowBoundingBoxes() const {
    return viewer->getConfig().rendering.showBoundingBoxes;
}

RigidBodyRenderer& SimulationManager::getRenderer() {
    return viewer->getRenderer();
}

void SimulationManager::updateAdaptiveTimesteps() {
    auto& system = viewer->getRigidBodySystem();
    auto& bodies = system.getBodies();
    auto& joints = system.getJoints();
    auto& contacts = system.getContacts();
    
    // Reset geometric stiffness contributions
    for (auto* body : bodies) {
        body->gsSum.setZero();
    }
    
    // Compute geometric stiffness contributions from joints
    for (auto* joint : joints) {
        joint->computeGeometricStiffness();
        joint->body0->gsSum += joint->G0;
        joint->body1->gsSum += joint->G1;
    }
    
    // Compute geometric stiffness contributions from contacts
    for (auto* contact : contacts) {
        contact->computeGeometricStiffness();
        contact->body0->gsSum += contact->G0;
        contact->body1->gsSum += contact->G1;
    }
    
    // Find maximum stiffness-to-mass ratio
    float maxK_M = 0.0f;
    for (auto* body : bodies) {
        if (body->fixed) continue;
        
        // Check translational DOFs
        for (int c = 0; c < 3; c++) {
            const float m = body->mass;
            const float k = 2.0f * body->gsSum.col(c).norm();
            maxK_M = std::max(k / m, maxK_M);
        }
        
        // Check rotational DOFs
        for (int c = 0; c < 3; c++) {
            const float m = body->I(c, c);
            const float k = 2.0f * body->gsSum.col(c + 3).norm();
            maxK_M = std::max(k / m, maxK_M);
        }
    }
    
    // Calculate stable timestep
    if (maxK_M > 0.0f) {
        const float dt = 2.0f * alpha * std::sqrt(1.0f / maxK_M);
        substeps = std::max(1, (int)ceil(timeStep / dt));
    } else {
        substeps = 1;
    }
}

void SimulationManager::computeKineticEnergy() {
    kineticEnergy = 0.0f;
    
    // Sum kinetic energy of all rigid bodies
    for (auto* body : viewer->getRigidBodySystem().getBodies()) {
        if (body->fixed) continue;
        
        // Translational kinetic energy: 1/2 * m * v^2
        kineticEnergy += 0.5f * body->mass * body->xdot.squaredNorm();
        
        // Rotational kinetic energy: 1/2 * ω^T * I * ω
        const auto I = body->q * body->Ibody * body->q.inverse();
        kineticEnergy += 0.5f * body->omega.dot(I * body->omega);
    }
}

void SimulationManager::computeConstraintError() {
    constraintError = 0.0f;
    
    // Sum constraint violations from all joints
    for (auto* joint : viewer->getRigidBodySystem().getJoints()) {
        constraintError += joint->phi.lpNorm<1>();
    }
}

void SimulationManager::setPreStepGeometricStiffnessDamping(bool enable) {
    geometricStiffnessDamping = enable;
    
    // Set up prestep function for geometric stiffness damping
    viewer->getRigidBodySystem().setPreStepFunc([this](RigidBodySystem& system, float h) {
        if (!geometricStiffnessDamping) return;
        
        auto& bodies = system.getBodies();
        auto& joints = system.getJoints();
        auto& contacts = system.getContacts();

        // Reset geometric stiffness damping values
        for (auto* body : bodies) {
            body->gsDamp.setZero();
        }

        // Compute geometric stiffness contributions
        for (auto* joint : joints) {
            joint->computeGeometricStiffness();
            joint->body0->gsSum += joint->G0;
            joint->body1->gsSum += joint->G1;
        }

        for (auto* contact : contacts) {
            contact->computeGeometricStiffness();
            contact->body0->gsSum += contact->G0;
            contact->body1->gsSum += contact->G1;
        }

        // Apply geometric stiffness damping
        for (auto* body : bodies) {
            if (body->fixed) continue;

            for (int c = 0; c < 3; c++) {
                const float m = body->I(c, c);
                const float k = 2.0f * body->gsSum.col(c + 3).norm();
                body->gsDamp(c) = std::max(0.0f, h * h * k - 4 * alpha * m);
            }
            
            // Update inertia matrix so solver has most recent gs damping information
            body->updateInertiaMatrix();
        }
    });
}

} // namespace slrbs
