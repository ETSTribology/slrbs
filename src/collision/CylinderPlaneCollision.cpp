#include "collision/CylinderPlaneCollision.h"
#include "collision/CollisionDetect.h"
#include "collision/CollisionUtils.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "geometry/Geometry.h"

#include <Eigen/Dense>

void CylinderPlaneCollision::detectCollision(RigidBody* body0, RigidBody* body1, CollisionDetect* detector) {
    Cylinder* cyl = dynamic_cast<Cylinder*>(body0->geometry.get());
    Plane* plane = dynamic_cast<Plane*>(body1->geometry.get());

    // Get cylinder's principal axis (y-axis)
    const Eigen::Vector3f cyldir = body0->q * Eigen::Vector3f(0, 1, 0);
    
    // Get plane normal and point in world space
    const Eigen::Vector3f planen = body1->q * plane->n;
    const Eigen::Vector3f planep = body1->q * plane->p + body1->x;

    // Dot product between cylinder axis and plane normal
    const float dp = cyldir.dot(planen);

    // Case 1: Cylinder is nearly aligned with plane normal
    if (std::fabs(dp) > 0.995f) 
    {
        // Determine which end of cylinder to check
        Eigen::Vector3f w;
        if (dp < 0.0f) {
            w = cyldir;
        } else {
            w = -cyldir;
        }

        // Construct orthogonal basis for cylinder end
        Eigen::Vector3f u, v;
        if (std::fabs(w.dot(Eigen::Vector3f(1, 0, 0))) > 0.01f) {
            u = w.cross(Eigen::Vector3f(1, 0, 0));
        } else {
            u = w.cross(Eigen::Vector3f(0, 0, 1));
        }
        u.normalize();
        v = w.cross(u);
        v.normalize();

        // Check end cap of cylinder
        const Eigen::Vector3f a = body0->x + 0.5f * cyl->height * w;
        const float dist = distancePointPlane(a, planep, planen);
        
        if (dist < 1e-3f)
        {
            const Eigen::Vector3f n = planen;

            // Check four points on the end cap circle
            float phiA = distancePointPlane(a + cyl->radius * u, planep, planen);
            float phiB = distancePointPlane(a - cyl->radius * u, planep, planen);
            float phiC = distancePointPlane(a + cyl->radius * v, planep, planen);
            float phiD = distancePointPlane(a - cyl->radius * v, planep, planen);

            // Create contacts for each point that's penetrating
            if (phiA < 1e-3f) {
                detector->addContact(new Contact(body0, body1, a + cyl->radius * u, n, std::min(0.0f, phiA)));
            }
            if (phiB < 1e-3f) {
                detector->addContact(new Contact(body0, body1, a - cyl->radius * u, n, std::min(0.0f, phiB)));
            }
            if (phiC < 1e-3f) {
                detector->addContact(new Contact(body0, body1, a + cyl->radius * v, n, std::min(0.0f, phiC)));
            }
            if (phiD < 1e-3f) {
                detector->addContact(new Contact(body0, body1, a - cyl->radius * v, n, std::min(0.0f, phiD)));
            }
        }
    }
    // Case 2: Cylinder is nearly perpendicular to plane normal
    else if (std::fabs(dp) < 1e-2f)  
    {
        const Eigen::Vector3f w = cyldir;
        
        // Find direction from cylinder axis toward the plane
        Eigen::Vector3f u = (planen.cross(w)).cross(w);
        if (u.dot(planen) > 0.0f) {
            u = -u;
        }
        u.normalize();

        // Check cylinder's side against plane
        const Eigen::Vector3f cylpos = body0->x;
        const float dist = distancePointPlane(cylpos, planep, planen) - cyl->radius;
        
        if (dist < 0.01f)
        {
            const Eigen::Vector3f n = planen;

            // Check both ends of the cylinder
            float phiA = distancePointPlane(cylpos + float(0.5f) * cyl->height * cyldir + cyl->radius * u, planep, planen);
            float phiB = distancePointPlane(cylpos - float(0.5f) * cyl->height * cyldir + cyl->radius * u, planep, planen);
            
            // Create contacts for penetrating points
            if (phiA < 1e-3f) {
                detector->addContact(new Contact(body0, body1, 
                    cylpos + float(0.5f) * cyl->height * cyldir + cyl->radius * u, 
                    n, std::min(0.0f, phiA)));
            }
            if (phiB < 1e-3f) {
                detector->addContact(new Contact(body0, body1, 
                    cylpos - float(0.5f) * cyl->height * cyldir + cyl->radius * u, 
                    n, std::min(0.0f, phiB)));
            }
        }
    }
    // Case 3: General case - cylinder at an angle to plane
    else
    {
        // Determine which end of cylinder to check
        Eigen::Vector3f w;
        if (dp < 0.0f) {
            w = cyldir;
        } else {
            w = -cyldir;
        }
        w.normalize();

        // Find direction from cylinder axis toward the plane
        Eigen::Vector3f u = (planen.cross(w)).cross(w);   
        u.normalize();

        if (u.dot(planen) > 0.0f) {
            u = -u;
        }
        u.normalize();

        // Check the point most likely to penetrate
        const Eigen::Vector3f a = body0->x + float(0.5f) * cyl->height * w + cyl->radius * u;
        const float dist = distancePointPlane(a, planep, planen);
        
        if (dist < 1e-3f)
        {
            const Eigen::Vector3f n = planen;
            const Eigen::Vector3f p = a;
            const float phi = std::min(0.0f, dist);
            
            detector->addContact(new Contact(body0, body1, p, n, phi));
        }
    }
}

float CylinderPlaneCollision::distancePointPlane(const Eigen::Vector3f& p, 
                                              const Eigen::Vector3f& plane_p, 
                                              const Eigen::Vector3f& plane_n) const {
    return (p - plane_p).dot(plane_n);
}