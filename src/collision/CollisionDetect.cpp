#include "collision/CollisionDetect.h"

#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "rigidbody/RigidBodySystem.h"


namespace
{
    // Compute the distance from a point to a plane defined by point and normal pair.
    // If the point is "inside" the plane, the returned distance is negative.
    static inline float distancePointPlane(const Eigen::Vector3f& p, const Eigen::Vector3f& plane_p, const Eigen::Vector3f& plane_n)
    {
        const Eigen::Vector3f v = (p - plane_p);
        const float d = v.dot(plane_n);
        return d;
    }

}


CollisionDetect::CollisionDetect(RigidBodySystem* rigidBodySystem) : m_rigidBodySystem(rigidBodySystem)
{

}

int CollisionDetect::findFaceForContact(RigidBody* body, const Eigen::Vector3f& worldPoint) {
    if (!body || !body->geometry) return -1;

    // A simple deterministic method to generate face indices without
    // needing direct access to mesh data

    // Convert world point to local coordinates
    Eigen::Vector3f localPoint = body->q.inverse() * (worldPoint - body->x);

    // Generate a face index based on position
    int faceIndex = 0;

    switch (body->geometry->getType()) {
        case kBox: {
            // For a box, determine which face was hit based on position
            Box* box = dynamic_cast<Box*>(body->geometry.get());
            Eigen::Vector3f halfDim = box->dim * 0.5f;
            Eigen::Vector3f normalizedPos = localPoint.cwiseQuotient(halfDim);

            // Which axis has the largest magnitude?
            int axisIndex = 0;
            float maxVal = std::abs(normalizedPos[0]);
            for (int i = 1; i < 3; i++) {
                if (std::abs(normalizedPos[i]) > maxVal) {
                    maxVal = std::abs(normalizedPos[i]);
                    axisIndex = i;
                }
            }

            // Face index: 0-1 for X, 2-3 for Y, 4-5 for Z
            // Even indices for negative direction, odd for positive
            faceIndex = axisIndex * 2 + (normalizedPos[axisIndex] > 0 ? 1 : 0);

            // Map to face triangles (each face has 2 triangles)
            faceIndex = faceIndex * 2;

            // Determine which of the two triangles on the face
            float u, v;
            switch (axisIndex) {
                case 0: // X axis - use Y,Z for subdivision
                    u = (normalizedPos.y() + 1.0f) * 0.5f;
                    v = (normalizedPos.z() + 1.0f) * 0.5f;
                    break;
                case 1: // Y axis - use X,Z for subdivision
                    u = (normalizedPos.x() + 1.0f) * 0.5f;
                    v = (normalizedPos.z() + 1.0f) * 0.5f;
                    break;
                case 2: // Z axis - use X,Y for subdivision
                    u = (normalizedPos.x() + 1.0f) * 0.5f;
                    v = (normalizedPos.y() + 1.0f) * 0.5f;
                    break;
            }

            // Determine which triangle of the face was hit
            if (u + v > 1.0f) {
                faceIndex += 1; // Second triangle
            }

            break;
        }
        case kSphere: {
            // For a sphere, create a mapping based on spherical coordinates
            Sphere* sphere = dynamic_cast<Sphere*>(body->geometry.get());

            // Normalize to unit sphere
            Eigen::Vector3f dir = localPoint.normalized();

            // Convert to spherical coordinates
            float theta = std::atan2(dir.y(), dir.x());
            float phi = std::acos(dir.z());

            // Map to a face index (assume 80 faces for a sphere)
            // Divide sphere into 8 sectors in theta and 5 in phi = 40 sectors
            // Each sector has 2 triangles
            int thetaIdx = static_cast<int>((theta + M_PI) / (2 * M_PI) * 8) % 8;
            int phiIdx = static_cast<int>(phi / M_PI * 5) % 5;

            faceIndex = (phiIdx * 8 + thetaIdx) * 2;

            // Determine which of the two triangles in the sector
            float thetaNorm = (theta + M_PI) / (2 * M_PI) * 8 - thetaIdx;
            float phiNorm = phi / M_PI * 5 - phiIdx;

            if (thetaNorm + phiNorm > 1.0f) {
                faceIndex += 1; // Second triangle
            }

            break;
        }
        default: {
            // For other shapes, just use a simple hash of the position
            float hash = localPoint.x() * 73.0f + localPoint.y() * 179.0f + localPoint.z() * 283.0f;
            faceIndex = static_cast<int>(std::abs(hash)) % 20; // Assume 20 faces
            break;
        }
    }

    return faceIndex;
}

void CollisionDetect::detectCollisions()
{
    // Next, loop over all pairs of bodies and test for contacts.
    //
    auto bodies = m_rigidBodySystem->getBodies();
    for (unsigned int i = 0; i < bodies.size(); ++i)
    {
        for (unsigned int j = i + 1; j < bodies.size(); ++j)
        {
            RigidBody* body0 = bodies[i];
            RigidBody* body1 = bodies[j];

            // Special case: skip tests for pairs of static bodies.
            //
            if (body0->fixed && body1->fixed)
                continue;

            // Test for sphere-sphere collision.
            if (body0->geometry->getType() == kSphere &&
                body1->geometry->getType() == kSphere)
            {
                collisionDetectSphereSphere(body0, body1);
            }
            // Test for sphere-box collision
            else if (body0->geometry->getType() == kSphere &&
                body1->geometry->getType() == kBox)
            {
                collisionDetectSphereBox(body0, body1);
            }
            // Test for box-sphere collision (order swap)
            else if (body1->geometry->getType() == kSphere &&
                body0->geometry->getType() == kBox)
            {
                collisionDetectSphereBox(body1, body0);
            }
            // Test for cylinder-plane collision
            else if (body0->geometry->getType() == kCylinder &&
                body1->geometry->getType() == kPlane)
            {
                collisionDetectCylinderPlane(body0, body1);
            }
            // Test for cylinder-plane collision
            else if (body1->geometry->getType() == kCylinder &&
                body0->geometry->getType() == kPlane)
            {
                collisionDetectCylinderPlane(body1, body0);
            }
            else if (body1->geometry->getType() == kBox &&
                body0->geometry->getType() == kBox)
            {
                collisionDetectBoxBox(body0, body1);
            }
        }
    }
}

void CollisionDetect::computeContactJacobians()
{
    for (auto c : m_contacts)
    {
        c->computeContactFrame();
        c->computeJacobian();
    }
}

void CollisionDetect::clear()
{
    for (auto c : m_contacts)
    {
        delete c;
    }
    m_contacts.clear();

    auto bodies = m_rigidBodySystem->getBodies();
    for (auto b : bodies)
    {
        b->contacts.clear();
    }
}


void CollisionDetect::collisionDetectSphereSphere(RigidBody* body0, RigidBody* body1)
{
    Sphere* sphere0 = dynamic_cast<Sphere*>(body0->geometry.get());
    Sphere* sphere1 = dynamic_cast<Sphere*>(body1->geometry.get());

    Eigen::Vector3f vec = body0->x - body1->x;

    const float rsum = (sphere0->radius + sphere1->radius);
    const float dist = vec.norm();
    if (dist < (rsum + 1e-3f))
    {
        const Eigen::Vector3f n = vec / dist;
        const Eigen::Vector3f p = 0.5f * ((body0->x - sphere0->radius * n) + (body1->x + sphere1->radius * n));
        const float phi = std::min(0.0f, dist - rsum);

        // Create contact first
        Contact* contact = new Contact(body0, body1, p, n, phi);
        // Set face indices separately (both -1 for spheres)
        contact->setFaceIndices(-1, -1);
        m_contacts.push_back(contact);
    }
}

void CollisionDetect::collisionDetectSphereBox(RigidBody* body0, RigidBody* body1)
{
    Sphere* sphere = dynamic_cast<Sphere*>(body0->geometry.get());
    Box* box = dynamic_cast<Box*>(body1->geometry.get());

    const Eigen::Vector3f clocal = body1->q.inverse() * (body0->x - body1->x);

    Eigen::Vector3f q(0, 0, 0);
    for (unsigned int i = 0; i < 3; ++i)
    {
        q[i] = clocal[i];
        if (q[i] < (-box->dim[i] / 2.0f)) q[i] = -box->dim[i] / 2.0f;
        else if (q[i] > (box->dim[i] / 2.0f)) q[i] = (box->dim[i] / 2.0f);
    }

    const Eigen::Vector3f dx = clocal - q;
    const float dist = dx.norm();
    if (dist < (sphere->radius + 1e-3f))
    {
        const Eigen::Vector3f n = body1->q * (dx / dist);
        const Eigen::Vector3f p = body1->q * q + body1->x;
        const float phi = std::min(0.0f, dist - sphere->radius);

        // Find the face index for the box
        int faceIndex = findFaceForContact(body1, p);

        // Create contact
        Contact* contact = new Contact(body0, body1, p, n, phi);
        // Set face indices (-1 for sphere, faceIndex for box)
        contact->setFaceIndices(-1, faceIndex);
        m_contacts.push_back(contact);
    }
}

void CollisionDetect::collisionDetectCylinderPlane(RigidBody* body0, RigidBody* body1)
{
    Cylinder* cyl = dynamic_cast<Cylinder*>(body0->geometry.get());
    Plane* plane = dynamic_cast<Plane*>(body1->geometry.get());

    // y-axis is the principal axis
    const Eigen::Vector3f cyldir = body0->q * Eigen::Vector3f(0, 1, 0);
    const Eigen::Vector3f planen = body1->q * plane->n;
    const Eigen::Vector3f planep = body1->q * plane->p + body1->x;

    const float dp = cyldir.dot(planen);

    if (std::fabs(dp) > 0.995f) // aligned with plane normal
    {
        Eigen::Vector3f w;
        if (dp < 0.0f)
        {
            w = cyldir;
        }
        else
        {
            w = -cyldir;
        }

        Eigen::Vector3f u, v;
        if (std::fabs(w.dot(Eigen::Vector3f(1, 0, 0))) > 0.01f)
        {
            u = w.cross(Eigen::Vector3f(1, 0, 0));
        }
        else
        {
            u = w.cross(Eigen::Vector3f(0, 0, 1));
        }
        u.normalize();
        v = w.cross(u);
        v.normalize();

        const Eigen::Vector3f a = body0->x + 0.5f * cyl->height * w;
        const float dist = distancePointPlane(a, planep, planen);
        if (dist < 1e-3f)
        {
            const Eigen::Vector3f n = planen;

            float phiA = distancePointPlane(a + cyl->radius * u, planep, planen);
            float phiB = distancePointPlane(a - cyl->radius * u, planep, planen);
            float phiC = distancePointPlane(a + cyl->radius * v, planep, planen);
            float phiD = distancePointPlane(a - cyl->radius * v, planep, planen);

            if (phiA < 1e-3f) {
                m_contacts.push_back(new Contact(body0, body1, a + cyl->radius * u, n, std::min(0.0f, phiA)));
            }
            if (phiB < 1e-3f) {
                m_contacts.push_back(new Contact(body0, body1, a - cyl->radius * u, n, std::min(0.0f, phiB)));
            }
            if (phiC < 1e-3f) {
                m_contacts.push_back(new Contact(body0, body1, a + cyl->radius * v, n, std::min(0.0f, phiC)));
            }
            if (phiD < 1e-3f) {
                m_contacts.push_back(new Contact(body0, body1, a - cyl->radius * v, n, std::min(0.0f, phiD)));
            }
        }

    }
    else if (std::fabs(dp) < 1e-2f)  // perpendicular to plane
    {
        const Eigen::Vector3f w = cyldir;
        Eigen::Vector3f u = (planen.cross(w)).cross(w);
        if (u.dot(planen) > 0.0f)
        {
            u = -u;
        }
        u.normalize();

        const Eigen::Vector3f cylpos = body0->x;
        const float dist = distancePointPlane(cylpos, planep, planen) - cyl->radius;
        if (dist < 0.01f)
        {
            const Eigen::Vector3f n = planen;

            float phiA = distancePointPlane(cylpos + float(0.5f) * cyl->height * cyldir + cyl->radius * u, planep, planen);
            float phiB = distancePointPlane(cylpos - float(0.5f) * cyl->height * cyldir + cyl->radius * u, planep, planen);
            if (phiA < 1e-3f) {
                m_contacts.push_back(new Contact(body0, body1, cylpos + float(0.5f) * cyl->height * cyldir + cyl->radius * u, n, std::min(0.0f, phiA)));
            }
            if (phiB < 1e-3f) {
                m_contacts.push_back(new Contact(body0, body1, cylpos - float(0.5f) * cyl->height * cyldir + cyl->radius * u, n, std::min(0.0f, phiB)));
            }
        }
    }
    else
    {
        Eigen::Vector3f w;
        if (dp < 0.0f)
        {
            w = cyldir;
        }
        else
        {
            w = -cyldir;
        }
        w.normalize();

        Eigen::Vector3f u = (planen.cross(w)).cross(w);   // u is orthogonal to v and is oriented toward the plane
        u.normalize();

        if (u.dot(planen) > 0.0f)
        {
            u = -u;
        }
        u.normalize();

        const Eigen::Vector3f a = body0->x + float(0.5f) * cyl->height * w + cyl->radius * u;
        const float dist = distancePointPlane(a, planep, planen);
        if (dist < 1e-3f)
        {
            const Eigen::Vector3f n = planen;
            const Eigen::Vector3f p = a;
            const float phi = std::min(0.0f, dist);

            Contact* contact = new Contact(body0, body1, p, n, phi);
            contact->setFaceIndices(-1, 0);
            m_contacts.push_back(contact);
        }
    }
}
