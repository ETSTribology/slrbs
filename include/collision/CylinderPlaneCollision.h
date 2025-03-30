#pragma once

#include "collision/CollisionHandler.h"

/**
 * Specialized collision handler for Cylinder-Plane collisions.
 */
class CylinderPlaneCollision : public CollisionHandler {
public:
    /**
     * Detect collision between a cylinder and a plane.
     */
    void detectCollision(RigidBody* body0, RigidBody* body1, CollisionDetect* detector) override;

private:
    /**
     * Calculate the distance from a point to a plane.
     */
    float distancePointPlane(const Eigen::Vector3f& p, 
                            const Eigen::Vector3f& plane_p, 
                            const Eigen::Vector3f& plane_n) const;
};