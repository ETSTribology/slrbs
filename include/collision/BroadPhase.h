#pragma once

#include <vector>
#include <memory>
#include "collision/CollisionPair.h"

class RigidBody;
class RigidBodySystem;

/**
 * Base class for broad phase collision detection algorithms.
 * Broad phase quickly filters out pairs of bodies that cannot possibly collide.
 */
class BroadPhase {
public:
    virtual ~BroadPhase() = default;
    
    /**
     * Generate potential collision pairs from a set of bodies.
     * @param system The rigid body system containing the bodies
     * @return A vector of potential collision pairs
     */
    virtual std::vector<CollisionPair> generatePairs(const RigidBodySystem* system) = 0;
    
    /**
     * Update the broad phase structure with new body positions.
     * @param system The rigid body system containing the bodies
     */
    virtual void update(const RigidBodySystem* system) = 0;
};

/**
 * Simple sweep and prune (SAP) broad phase algorithm.
 * Works well for many typical game physics scenarios.
 */
class SweepAndPrune : public BroadPhase {
public:
    SweepAndPrune();
    ~SweepAndPrune() override = default;
    
    std::vector<CollisionPair> generatePairs(const RigidBodySystem* system) override;
    void update(const RigidBodySystem* system) override;

private:
    struct AxisEndpoint {
        uint32_t bodyIndex;     // Index of the body
        float value;            // Position along axis
        bool isMin;             // True if this is the min endpoint
        
        bool operator<(const AxisEndpoint& other) const {
            return value < other.value;
        }
    };
    
    std::vector<AxisEndpoint> m_xAxis;  // Endpoints along X axis
    std::vector<CollisionPair> m_pairs;  // Cached pairs to reduce allocations
};

/**
 * Factory for creating broad phase algorithms.
 */
class BroadPhaseFactory {
public:
    /**
     * Create a broad phase algorithm by name.
     * @param name Name of the algorithm (e.g., "sap" for Sweep and Prune)
     * @return A unique pointer to the created broad phase algorithm
     */
    static std::unique_ptr<BroadPhase> create(const std::string& name) {
        if (name == "sap") {
            return std::make_unique<SweepAndPrune>();
        }
        
        // Default to Sweep and Prune
        return std::make_unique<SweepAndPrune>();
    }
};