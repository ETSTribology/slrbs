#include "collision/BroadPhase.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBody.h"
#include "geometry/Geometry.h"

#include <algorithm>
#include <unordered_set>

// Helper function to get the bounding radius of a geometry
float getBoundingRadius(const Geometry* geometry) {
    switch (geometry->getType()) {
        case kSphere:
            return static_cast<const Sphere*>(geometry)->radius;
        case kBox: {
            const auto* box = static_cast<const Box*>(geometry);
            return 0.5f * box->dim.norm(); // Half diagonal
        }
        case kCylinder: {
            const auto* cylinder = static_cast<const Cylinder*>(geometry);
            float radiusSq = cylinder->radius * cylinder->radius;
            float heightSq = 0.25f * cylinder->height * cylinder->height;
            return std::sqrt(radiusSq + heightSq); // Radius of bounding sphere
        }
        case kPlane:
            return 0.0f; // Planes extend infinitely, handled specially
        default:
            return 0.0f; // Default fallback
    }
}

SweepAndPrune::SweepAndPrune() 
    : m_xAxis(), m_pairs()
{
}

std::vector<CollisionPair> SweepAndPrune::generatePairs(const RigidBodySystem* system) {
    // Clear previous pairs
    m_pairs.clear();
    
    // Get the bodies
    const auto& bodies = system->getBodies();
    
    // If we have no endpoints yet, initialize
    if (m_xAxis.empty()) {
        update(system);
    }
    
    // Use a hash set to avoid duplicate pairs
    std::unordered_set<uint64_t> pairSet;
    
    // Sort endpoints by position on axis
    std::sort(m_xAxis.begin(), m_xAxis.end());
    
    // Active set of bodies (indices)
    std::vector<uint32_t> active;
    
    // Sweep the axis
    for (const auto& endpoint : m_xAxis) {
        uint32_t bodyIndex = endpoint.bodyIndex;
        
        if (endpoint.isMin) {
            // Starting endpoint - check against all active bodies
            for (uint32_t activeIndex : active) {
                // Skip if both bodies are fixed
                if (bodies[bodyIndex]->fixed && bodies[activeIndex]->fixed) {
                    continue;
                }
                
                // Create a unique key for this pair
                uint32_t minIdx = std::min(bodyIndex, activeIndex);
                uint32_t maxIdx = std::max(bodyIndex, activeIndex);
                uint64_t pairKey = (static_cast<uint64_t>(minIdx) << 32) | maxIdx;
                
                // Add if not already present
                if (pairSet.insert(pairKey).second) {
                    // Compute distance for sorting
                    float distSq = (bodies[bodyIndex]->x - bodies[activeIndex]->x).squaredNorm();
                    m_pairs.emplace_back(bodyIndex, activeIndex, distSq);
                }
            }
            
            // Add to active set
            active.push_back(bodyIndex);
        } else {
            // Ending endpoint - remove from active set
            active.erase(std::remove(active.begin(), active.end(), bodyIndex), active.end());
        }
    }
    
    // Sort pairs by distance (nearest first)
    std::sort(m_pairs.begin(), m_pairs.end());
    
    return m_pairs;
}

void SweepAndPrune::update(const RigidBodySystem* system) {
    // Get the bodies
    const auto& bodies = system->getBodies();
    const size_t numBodies = bodies.size();
    
    // Resize and fill endpoint array
    m_xAxis.resize(numBodies * 2);
    
    for (size_t i = 0; i < numBodies; ++i) {
        const RigidBody* body = bodies[i];
        float radius = getBoundingRadius(body->geometry.get());
        
        // Min endpoint
        m_xAxis[i*2].bodyIndex = static_cast<uint32_t>(i);
        m_xAxis[i*2].value = body->x.x() - radius;
        m_xAxis[i*2].isMin = true;
        
        // Max endpoint
        m_xAxis[i*2+1].bodyIndex = static_cast<uint32_t>(i);
        m_xAxis[i*2+1].value = body->x.x() + radius;
        m_xAxis[i*2+1].isMin = false;
    }
}