#include "collision/SphereSphereCollision.h"
#include "collision/CollisionDetect.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "geometry/Geometry.h"

#include <Eigen/Dense>

void SphereSphereCollision::detectCollision(RigidBody* body0, RigidBody* body1, CollisionDetect* detector) {
    Sphere* sphere0 = dynamic_cast<Sphere*>(body0->geometry.get());
    Sphere* sphere1 = dynamic_cast<Sphere*>(body1->geometry.get());

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