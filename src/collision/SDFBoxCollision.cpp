#include "collision/SDFBoxCollision.h"
#include "collision/CollisionDetect.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "geometry/Geometry.h"

#include <Eigen/Dense>
#include <vector>

void SDFBoxCollision::detectCollision(RigidBody* body0, RigidBody* body1, CollisionDetect* detector) {
    // Make sure body0 is the SDF and body1 is the box
    SDFGeometry* sdf = dynamic_cast<SDFGeometry*>(body0->geometry.get());
    Box* box = dynamic_cast<Box*>(body1->geometry.get());
    
    if (!sdf || !box) {
        // Try the other way around
        sdf = dynamic_cast<SDFGeometry*>(body1->geometry.get());
        box = dynamic_cast<Box*>(body0->geometry.get());
        
        if (!sdf || !box) {
            // Not the right types
            return;
        }
        
        // Swap bodies
        std::swap(body0, body1);
    }
    
    // Transform for the box
    Eigen::Transform<float, 3, Eigen::Affine> boxTransform = 
        Eigen::Transform<float, 3, Eigen::Affine>::Identity();
    boxTransform.translate(body1->x);
    boxTransform.rotate(body1->q);
    
    // Generate sample points on the box
    std::vector<Eigen::Vector3f> samplePoints = 
        generateBoxSamplePoints(box, boxTransform);
    
    // Check each sample point against the SDF
    for (const auto& point : samplePoints) {
        // Transform point to SDF's local space
        Eigen::Vector3f localPoint = 
            body0->q.inverse() * (point - body0->x);
        
        // Sample the SDF
        float distance = sampleSDF(localPoint.cast<double>(), sdf);
        
        // If the point is inside the SDF, create a contact
        if (distance < 0.0f) {
            // Compute normal by sampling nearby points for gradient
            Eigen::Vector3f gradient(0, 0, 0);
            const float h = 0.01f; // Small delta for central difference
            
            // X direction
            Eigen::Vector3f pX = localPoint + Eigen::Vector3f(h, 0, 0);
            Eigen::Vector3f nX = localPoint - Eigen::Vector3f(h, 0, 0);
            gradient.x() = sampleSDF(pX.cast<double>(), sdf) - sampleSDF(nX.cast<double>(), sdf);
            
            // Y direction
            Eigen::Vector3f pY = localPoint + Eigen::Vector3f(0, h, 0);
            Eigen::Vector3f nY = localPoint - Eigen::Vector3f(0, h, 0);
            gradient.y() = sampleSDF(pY.cast<double>(), sdf) - sampleSDF(nY.cast<double>(), sdf);
            
            // Z direction
            Eigen::Vector3f pZ = localPoint + Eigen::Vector3f(0, 0, h);
            Eigen::Vector3f nZ = localPoint - Eigen::Vector3f(0, 0, h);
            gradient.z() = sampleSDF(pZ.cast<double>(), sdf) - sampleSDF(nZ.cast<double>(), sdf);
            
            // Normalize gradient to get normal (pointing outward from SDF)
            Eigen::Vector3f normal = -gradient.normalized();
            
            // Transform normal to world space
            normal = body0->q * normal;
            
            // Create contact with penetration depth
            detector->addContact(new Contact(body0, body1, point, normal, distance));
        }
    }
}

float SDFBoxCollision::sampleSDF(const Eigen::Vector3d& point, const SDFGeometry* sdf) const {
    // Sample the SDF at the given point
    return static_cast<float>(sdf->sdf->interpolate(0, point));
}

std::vector<Eigen::Vector3f> SDFBoxCollision::generateBoxSamplePoints(
    const Box* box, 
    const Eigen::Transform<float, 3, Eigen::Affine>& boxTransform,
    int samplesPerDimension)
{
    std::vector<Eigen::Vector3f> points;
    
    // Calculate step size
    Eigen::Vector3f step = box->dim / static_cast<float>(samplesPerDimension - 1);
    
    // Generate points on each face of the box
    for (int face = 0; face < 6; ++face) {
        int axis = face / 2;         // 0 for x, 1 for y, 2 for z
        float sign = (face % 2) ? 1.0f : -1.0f;  // +1 or -1
        
        for (int i = 0; i < samplesPerDimension; ++i) {
            for (int j = 0; j < samplesPerDimension; ++j) {
                // Create the point in local space
                Eigen::Vector3f point(-box->dim.x()/2.0f + (axis == 0 ? sign*box->dim.x()/2.0f : i*step.x()),
                                      -box->dim.y()/2.0f + (axis == 1 ? sign*box->dim.y()/2.0f : j*step.y()),
                                      -box->dim.z()/2.0f + (axis == 2 ? sign*box->dim.z()/2.0f : 
                                                          (axis == 0 ? j*step.z() : i*step.z())));
                
                // Transform to world space
                Eigen::Vector3f worldPoint = boxTransform * point;
                
                // Add to result
                points.push_back(worldPoint);
            }
        }
    }
    
    return points;
}