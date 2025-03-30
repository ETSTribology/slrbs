#include "collision/BoxBoxCollision.h"
#include "collision/CollisionDetect.h"
#include "collision/CollisionUtils.h"
#include "contact/Contact.h"
#include "rigidbody/RigidBody.h"
#include "geometry/Geometry.h"

#include <Eigen/Dense>
#include <cassert>

using namespace Eigen;
using namespace CollisionUtils;

void BoxBoxCollision::detectCollision(RigidBody* body0, RigidBody* body1, CollisionDetect* detector) {
    Box* boxA = dynamic_cast<Box*>(body0->geometry.get());
    Box* boxB = dynamic_cast<Box*>(body1->geometry.get());

    // Set up the transforms for both boxes
    Transform<float, 3, Affine> transformA = Transform<float, 3, Affine>::Identity();
    transformA.translate(body0->x);
    transformA.rotate(body0->q);

    Transform<float, 3, Affine> transformB = Transform<float, 3, Affine>::Identity();
    transformB.translate(body1->x);
    transformB.rotate(body1->q);

    Transform<float, 3, Affine> atx = transformA;
    Transform<float, 3, Affine> btx = transformB;
    Vector3f eA = boxA->dim * 0.5f;
    Vector3f eB = boxB->dim * 0.5f;

    // Compute rotation matrix from A to B
    Matrix3f C = atx.rotation().transpose() * btx.rotation();

    // Check for parallel faces
    Matrix3f absC;
    bool parallel = false;
    const float kCosTol = 1.0e-6f;
    
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float val = std::abs(C(i, j));
            absC(i, j) = val;

            if (val + kCosTol >= 1.0f)
                parallel = true;
        }
    }

    // Translation from A to B in A's coordinate system
    Vector3f t = atx.rotation().transpose() * (btx.translation() - atx.translation());

    // Variables to track the best separating axis
    float s;
    float aMax = -std::numeric_limits<float>::max();
    float bMax = -std::numeric_limits<float>::max();
    float eMax = -std::numeric_limits<float>::max();
    int aAxis = INVALID_AXIS;
    int bAxis = INVALID_AXIS;
    int eAxis = INVALID_AXIS;
    Vector3f nA;
    Vector3f nB;
    Vector3f nE;

    // Test box A's face normals (x, y, and z axes)
    s = std::abs(t.x()) - (eA.x() + absC.row(0).dot(eB));
    if (TrackFaceAxis(&aAxis, 0, s, &aMax, atx.rotation().col(0), &nA))
        return;

    s = std::abs(t.y()) - (eA.y() + absC.row(1).dot(eB));
    if (TrackFaceAxis(&aAxis, 1, s, &aMax, atx.rotation().col(1), &nA))
        return;

    s = std::abs(t.z()) - (eA.z() + absC.row(2).dot(eB));
    if (TrackFaceAxis(&aAxis, 2, s, &aMax, atx.rotation().col(2), &nA))
        return;

    // Test box B's face normals
    s = std::abs(t.dot(C.col(0))) - (eB.x() + absC.col(0).dot(eA));
    if (TrackFaceAxis(&bAxis, 3, s, &bMax, btx.rotation().col(0), &nB))
        return;

    s = std::abs(t.dot(C.col(1))) - (eB.y() + absC.col(1).dot(eA));
    if (TrackFaceAxis(&bAxis, 4, s, &bMax, btx.rotation().col(1), &nB))
        return;

    s = std::abs(t.dot(C.col(2))) - (eB.z() + absC.col(2).dot(eA));
    if (TrackFaceAxis(&bAxis, 5, s, &bMax, btx.rotation().col(2), &nB))
        return;

    // Test all 9 edge-edge combinations (if not parallel)
    if (!parallel) {
        float rA, rB;

        // A.x <cross> B.x
        rA = eA.y() * absC(0, 2) + eA.z() * absC(0, 1);
        rB = eB.y() * absC(2, 0) + eB.z() * absC(1, 0);
        s = std::abs(t.z() * C(0, 1) - t.y() * C(0, 2)) - (rA + rB);
        if (TrackEdgeAxis(&eAxis, 6, s, &eMax, Vector3f(0.0f, -C(0, 2), C(0, 1)), &nE))
            return;

        // A.x <cross> B.y
        rA = eA.y() * absC(1, 2) + eA.z() * absC(1, 1);
        rB = eB.x() * absC(2, 0) + eB.z() * absC(0, 0);
        s = std::abs(t.z() * C(1, 1) - t.y() * C(1, 2)) - (rA + rB);
        if (TrackEdgeAxis(&eAxis, 7, s, &eMax, Vector3f(0.0f, -C(1, 2), C(1, 1)), &nE))
            return;

        // A.x <cross> B.z
        rA = eA.y() * absC(2, 2) + eA.z() * absC(2, 1);
        rB = eB.x() * absC(1, 0) + eB.y() * absC(0, 0);
        s = std::abs(t.z() * C(2, 1) - t.y() * C(2, 2)) - (rA + rB);
        if (TrackEdgeAxis(&eAxis, 8, s, &eMax, Vector3f(0.0f, -C(2, 2), C(2, 1)), &nE))
            return;

        // A.y <cross> B.x
        rA = eA.x() * absC(0, 2) + eA.z() * absC(0, 0);
        rB = eB.y() * absC(2, 1) + eB.z() * absC(1, 1);
        s = std::abs(t.x() * C(0, 2) - t.z() * C(0, 0)) - (rA + rB);
        if (TrackEdgeAxis(&eAxis, 9, s, &eMax, Vector3f(C(0, 2), 0.0f, -C(0, 0)), &nE))
            return;

        // A.y <cross> B.y
        rA = eA.x() * absC(1, 2) + eA.z() * absC(1, 0);
        rB = eB.x() * absC(2, 1) + eB.z() * absC(0, 1);
        s = std::abs(t.x() * C(1, 2) - t.z() * C(1, 0)) - (rA + rB);
        if (TrackEdgeAxis(&eAxis, 10, s, &eMax, Vector3f(C(1, 2), 0.0f, -C(1, 0)), &nE))
            return;

        // A.y <cross> B.z
        rA = eA.x() * absC(2, 2) + eA.z() * absC(2, 0);
        rB = eB.x() * absC(1, 1) + eB.y() * absC(0, 1);
        s = std::abs(t.x() * C(2, 2) - t.z() * C(2, 0)) - (rA + rB);
        if (TrackEdgeAxis(&eAxis, 11, s, &eMax, Vector3f(C(2, 2), 0.0f, -C(2, 0)), &nE))
            return;

        // A.z <cross> B.x
        rA = eA.x() * absC(0, 1) + eA.y() * absC(0, 0);
        rB = eB.y() * absC(2, 2) + eB.z() * absC(1, 2);
        s = std::abs(t.y() * C(0, 0) - t.x() * C(0, 1)) - (rA + rB);
        if (TrackEdgeAxis(&eAxis, 12, s, &eMax, Vector3f(-C(0, 1), C(0, 0), 0.0f), &nE))
            return;

        // A.z <cross> B.y
        rA = eA.x() * absC(1, 1) + eA.y() * absC(1, 0);
        rB = eB.x() * absC(2, 2) + eB.z() * absC(0, 2);
        s = std::abs(t.y() * C(1, 0) - t.x() * C(1, 1)) - (rA + rB);
        if (TrackEdgeAxis(&eAxis, 13, s, &eMax, Vector3f(-C(1, 1), C(1, 0), 0.0f), &nE))
            return;

        // A.z <cross> B.z
        rA = eA.x() * absC(2, 1) + eA.y() * absC(2, 0);
        rB = eB.x() * absC(1, 2) + eB.y() * absC(0, 2);
        s = std::abs(t.y() * C(2, 0) - t.x() * C(2, 1)) - (rA + rB);
        if (TrackEdgeAxis(&eAxis, 14, s, &eMax, Vector3f(-C(2, 1), C(2, 0), 0.0f), &nE))
            return;
    }

    // Select best separation axis based on tolerance comparisons
    const float kRelTol = 0.95f;
    const float kAbsTol = 0.01f;
    int axis;
    float sMax;
    Vector3f n;
    
    float faceMax = std::max(aMax, bMax);
    if (kRelTol * eMax > faceMax + kAbsTol) {
        axis = eAxis;
        sMax = eMax;
        n = nE;
    } else {
        if (kRelTol * bMax > aMax + kAbsTol) {
            axis = bAxis;
            sMax = bMax;
            n = nB;
        } else {
            axis = aAxis;
            sMax = aMax;
            n = nA;
        }
    }

    // If no valid axis found, no collision
    if (axis == INVALID_AXIS)
        return;

    // Handle face or edge contact
    if (axis < 6) {
        // Face contact
        Transform<float, 3, Affine> rtx;
        Transform<float, 3, Affine> itx;
        Vector3f eR;
        Vector3f eI;
        bool flip;

        if (axis < 3) {
            rtx = atx;
            itx = btx;
            eR = eA;
            eI = eB;
            flip = false;
        } else {
            rtx = btx;
            itx = atx;
            eR = eB;
            eI = eA;
            flip = true;
            n = -n;
        }

        // Compute incident face vertices
        ClipVertex incident[4];
        ComputeIncidentFace(itx, eI, n, incident);
        
        // Compute reference face edges and basis
        uint8_t clipEdges[4];
        Matrix3f basis;
        Vector3f e;
        ComputeReferenceEdgesAndBasis(eR, rtx, n, axis, clipEdges, &basis, &e);

        // Clip incident face against reference face side planes
        ClipVertex out[8];
        float depths[8];
        int outNum = Clip(rtx.translation(), e, clipEdges, basis, incident, out, depths);

        // Create contacts for each clipped point
        if (outNum) {
            Vector3f normal = normalizeNormal(flip ? -n : n, atx, btx);

            for (int i = 0; i < outNum; ++i) {
                FeaturePair pair = out[i].f;

                if (flip) {
                    std::swap(pair.inI, pair.inR);
                    std::swap(pair.outI, pair.outR);
                }

                detector->addContact(new Contact(body0, body1, out[i].v, normal, depths[i]));
            }
        }
    } else {
        // Edge contact
        n = atx.rotation() * n;
        n = normalizeNormal(n, atx, btx);

        // Find support edges
        Vector3f PA, QA;
        Vector3f PB, QB;
        SupportEdge(atx, eA, n, &PA, &QA);
        SupportEdge(btx, eB, -n, &PB, &QB);

        // Compute closest points between edges
        Vector3f CA, CB;
        EdgesContact(&CA, &CB, PA, QA, PB, QB);

        // Create a contact at the midpoint
        FeaturePair pair;
        pair.key = axis;

        detector->addContact(new Contact(body0, body1, 0.5f * (CA + CB), n, sMax));
    }
}

Eigen::Vector3f BoxBoxCollision::normalizeNormal(const Eigen::Vector3f& n, 
                                               const Eigen::Transform<float, 3, Eigen::Affine>& txA,
                                               const Eigen::Transform<float, 3, Eigen::Affine>& txB) {
    Eigen::Vector3f norm = n;
    if (norm.dot(txB.translation() - txA.translation()) < 0.0f)
        norm = -norm;
    return norm.normalized();
}