#include "collision/SphereBoxCollision.h"
#include "collision/CollisionDetect.h"
#include "collision/CollisionUtils.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "geometry/Geometry.h"

#include <Eigen/Dense>

void SphereBoxCollision::detectCollision(RigidBody* body0, RigidBody* body1, CollisionDetect* detector) {
    // Ensure body0 is the sphere and body1 is the box
    Sphere* sphere = dynamic_cast<Sphere*>(body0->geometry.get());
    Box* box = dynamic_cast<Box*>(body1->geometry.get());

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