#pragma once

#include "collision/ImprovedCollisionRegistry.h"
#include "collision/CollisionHandler.h"
#include "geometry/GeometryType.h"

// Forward declarations of collision handlers
class BoxBoxCollision;
class SphereBoxCollision;
class SphereSphereCollision;
class CylinderPlaneCollision;
class SDFBoxCollision;

/**
 * Factory for creating and registering collision handlers.
 * This provides a simple interface for registering all default collision handlers.
 */
class CollisionFactory {
public:
    /**
     * Register all standard collision handlers with the registry.
     * @param registry The registry to register handlers with
     */
    static void registerStandardHandlers(ImprovedCollisionRegistry& registry) {
        registerBoxCollisions(registry);
        registerSphereCollisions(registry);
        registerCylinderCollisions(registry);
        registerSDFCollisions(registry);
    }

private:
    // Helper methods to register handlers for specific geometry types
    static void registerBoxCollisions(ImprovedCollisionRegistry& registry) {
        registry.registerHandler<BoxBoxCollision>(kBox, kBox);
        // Box-Plane is handled by primitive collision
    }
    
    static void registerSphereCollisions(ImprovedCollisionRegistry& registry) {
        registry.registerHandler<SphereSphereCollision>(kSphere, kSphere);
        registry.registerHandler<SphereBoxCollision>(kSphere, kBox);
        // Sphere-Plane is handled by primitive collision
    }
    
    static void registerCylinderCollisions(ImprovedCollisionRegistry& registry) {
        registry.registerHandler<CylinderPlaneCollision>(kCylinder, kPlane);
        // Other cylinder collisions could be added here
    }
    
    static void registerSDFCollisions(ImprovedCollisionRegistry& registry) {
        registry.registerHandler<SDFBoxCollision>(kSDF, kBox);
        // Other SDF collisions could be added here
    }
};