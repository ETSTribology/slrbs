#pragma once

#include <memory>

class RigidBody;
class CollisionDetect;

/**
 * Base class for all collision handlers.
 * Uses virtual inheritance to enable polymorphic behavior.
 */
class CollisionHandler {
public:
    virtual ~CollisionHandler() = default;
    
    /**
     * Virtual method to detect and resolve collisions between two rigid bodies.
     * 
     * @param body0 First rigid body
     * @param body1 Second rigid body
     * @param detector Collision detector to register contacts with
     */
    virtual void detectCollision(RigidBody* body0, RigidBody* body1, CollisionDetect* detector) = 0;
};