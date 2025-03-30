#pragma once

#include "collision/CollisionHandler.h"

/**
 * Specialized collision handler for Box-Box collisions.
 */
class BoxBoxCollision : public CollisionHandler {
public:
    /**
     * Detect collision between two box-shaped rigid bodies.
     */
    void detectCollision(RigidBody* body0, RigidBody* body1, CollisionDetect* detector) override;

private:
    /**
     * Helper function to normalize the final contact normal.
     */
    Eigen::Vector3f normalizeNormal(const Eigen::Vector3f& n, 
                                   const Eigen::Transform<float, 3, Eigen::Affine>& txA,
                                   const Eigen::Transform<float, 3, Eigen::Affine>& txB);
};