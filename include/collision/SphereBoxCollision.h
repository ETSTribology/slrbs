#pragma once

#include "collision/CollisionHandler.h"

/**
 * Specialized collision handler for Sphere-Box collisions.
 */
class SphereBoxCollision : public CollisionHandler {
public:
    /**
     * Detect collision between a sphere and a box.
     */
    void detectCollision(RigidBody* body0, RigidBody* body1, CollisionDetect* detector) override;
};