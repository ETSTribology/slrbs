#pragma once

#include "collision/CollisionHandler.h"

/**
 * Specialized collision handler for Sphere-Sphere collisions.
 */
class SphereSphereCollision : public CollisionHandler {
public:
    /**
     * Detect collision between two spheres.
     */
    void detectCollision(RigidBody* body0, RigidBody* body1, CollisionDetect* detector) override;
};