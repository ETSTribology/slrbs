#pragma once

#include <cstdint>
#include <limits>
#include <algorithm>

class RigidBody;

/**
 * A collision pair represents two bodies that might collide.
 * This structure is optimized for cache efficiency and sorting.
 */
struct CollisionPair {
    uint32_t bodyA;       // Index of first body
    uint32_t bodyB;       // Index of second body
    float distanceSq;     // Squared distance between bounds (for broad phase sorting)
    
    /**
     * Default constructor. Creates an invalid pair.
     */
    CollisionPair()
        : bodyA(std::numeric_limits<uint32_t>::max())
        , bodyB(std::numeric_limits<uint32_t>::max())
        , distanceSq(std::numeric_limits<float>::max())
    {}
    
    /**
     * Create a collision pair with two body indices.
     * @param a First body index
     * @param b Second body index
     * @param distSq Squared distance between bounds (optional)
     */
    CollisionPair(uint32_t a, uint32_t b, float distSq = 0.0f)
        : bodyA(std::min(a, b))
        , bodyB(std::max(a, b))
        , distanceSq(distSq)
    {}
    
    /**
     * Check if this is a valid collision pair.
     */
    bool isValid() const {
        return bodyA != std::numeric_limits<uint32_t>::max() &&
               bodyB != std::numeric_limits<uint32_t>::max();
    }
    
    /**
     * Compare pairs for sorting (used in broad phase).
     */
    bool operator<(const CollisionPair& other) const {
        return distanceSq < other.distanceSq;
    }
    
    /**
     * Check if two pairs are the same.
     */
    bool operator==(const CollisionPair& other) const {
        return bodyA == other.bodyA && bodyB == other.bodyB;
    }
};