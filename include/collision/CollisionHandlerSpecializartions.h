#pragma once

#include "collision/CollisionRegistry.h"
#include "collision/CollisionDetect.h"
#include "collision/CollisionUtils.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "geometry/Geometry.h"

#include <Eigen/Dense>

/**
 * Template specialization for Sphere-Sphere collision detection.
 */
template<>
void CollisionHandlerFor<Sphere, Sphere>::detectCollisionImpl(
    RigidBody* body0, RigidBody* body1, 
    Sphere* sphere0, Sphere* sphere1,
    CollisionDetect* detector) 
{
    // Vector between sphere centers
    Eigen::Vector3f vec = body0->x - body1->x;

    // Sum of radii
    const float rsum = (sphere0->radius + sphere1->radius);
    
    // Distance between centers
    const float dist = vec.norm();
    
    // Check for collision (distance less than sum of radii)
    if (dist < (rsum + 1e-3f))
    {
        // Normalize the direction vector
        const Eigen::Vector3f n = vec / dist;
        
        // Compute contact point midway between sphere surfaces
        const Eigen::Vector3f p = 0.5f * ((body0->x - sphere0->radius * n) + 
                                          (body1->x + sphere1->radius * n));
        
        // Penetration depth (negative if penetrating)
        const float phi = std::min(0.0f, dist - rsum);

        // Create the contact
        detector->addContact(new Contact(body0, body1, p, n, phi));
    }
}

/**
 * Template specialization for Sphere-Box collision detection.
 */
template<>
void CollisionHandlerFor<Sphere, Box>::detectCollisionImpl(
    RigidBody* body0, RigidBody* body1, 
    Sphere* sphere, Box* box,
    CollisionDetect* detector) 
{
    // Transform sphere center to box's local space
    const Eigen::Vector3f clocal = body1->q.inverse() * (body0->x - body1->x);

    // Find closest point on box to sphere center
    Eigen::Vector3f q(0, 0, 0);
    for (unsigned int i = 0; i < 3; ++i)
    {
        // Clamp sphere center to box extents
        q[i] = clocal[i];
        if (q[i] < (-box->dim[i] / 2.0f)) q[i] = -box->dim[i] / 2.0f;
        else if (q[i] > (box->dim[i] / 2.0f)) q[i] = (box->dim[i] / 2.0f);
    }

    // Vector from closest point on box to sphere center
    const Eigen::Vector3f dx = clocal - q;
    const float dist = dx.norm();
    
    // Check for collision (distance less than sphere radius)
    if (dist < (sphere->radius + 1e-3f))
    {
        // Transform normal to world space and normalize
        const Eigen::Vector3f n = body1->q * (dx / dist);
        
        // Transform contact point to world space
        const Eigen::Vector3f p = body1->q * q + body1->x;
        
        // Penetration depth (negative if penetrating)
        const float phi = std::min(0.0f, dist - sphere->radius);

        // Create the contact
        detector->addContact(new Contact(body0, body1, p, n, phi));
    }
}

// More specializations would be added here for other collision pairs