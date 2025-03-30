#pragma once
#include <Eigen/Dense>
#include <cstdint>

namespace CollisionUtils {

// Common constants
const int INVALID_AXIS = -1;

/**
 * Feature pair structure for tracking collision features.
 */
struct FeaturePair {
    union {
        struct {
            uint8_t inR;   // Reference in-edge
            uint8_t outR;  // Reference out-edge
            uint8_t inI;   // Incident in-edge
            uint8_t outI;  // Incident out-edge
        };
        int32_t key;      // Composite key for feature pair
    };
};

/**
 * Clip vertex structure for polygon clipping.
 */
struct ClipVertex {
    ClipVertex() { f.key = ~0; }
    Eigen::Vector3f v;    // Vertex position
    FeaturePair f;        // Feature pair information
};

/**
 * Calculate the signed distance from a point to a plane.
 */
float distancePointPlane(
    const Eigen::Vector3f& p, 
    const Eigen::Vector3f& plane_p, 
    const Eigen::Vector3f& plane_n
);

/**
 * Helper function to track separating axes for face collision.
 */
bool TrackFaceAxis(
    int* axis, 
    int n, 
    float s, 
    float* sMax, 
    const Eigen::Vector3f& normal, 
    Eigen::Vector3f* axisNormal
);

/**
 * Helper function to track separating axes for edge collision.
 */
bool TrackEdgeAxis(
    int* axis, 
    int n, 
    float s, 
    float* sMax, 
    const Eigen::Vector3f& normal, 
    Eigen::Vector3f* axisNormal
);

/**
 * Compute the reference edges and basis for a box face.
 */
void ComputeReferenceEdgesAndBasis(
    const Eigen::Vector3f& eR, 
    const Eigen::Transform<float, 3, Eigen::Affine>& rtx,
    Eigen::Vector3f n, 
    int axis, 
    uint8_t* out, 
    Eigen::Matrix3f* basis, 
    Eigen::Vector3f* e
);

/**
 * Compute the incident face vertices for box-box collision.
 */
void ComputeIncidentFace(
    const Eigen::Transform<float, 3, Eigen::Affine>& itx, 
    const Eigen::Vector3f& e, 
    Eigen::Vector3f n, 
    ClipVertex* out
);

/**
 * Perform orthographic clipping along one axis.
 */
int Orthographic(
    float sign, 
    float e, 
    int axis, 
    int clipEdge, 
    ClipVertex* in, 
    int inCount, 
    ClipVertex* out
);

/**
 * Perform full clipping for box-box contact generation.
 */

    uint8_t* clipEdges, 
    const Eigen::Matrix3f* basis,
    ClipVertex* incident, 
    ClipVertex* outVerts, 
    float* outDepths
);

/**
 * Calculate contact points for edge-edge contact.
 */
void EdgesContact(
    Eigen::Vector3f* CA, 
    Eigen::Vector3f* CB, 
    const Eigen::Vector3f& PA, 
    const Eigen::Vector3f& QA,
    const Eigen::Vector3f& PB, 
    const Eigen::Vector3f& QB
);

/**
 * Compute the support edge for a box in a given direction.
 */
void SupportEdge(
    const Eigen::Transform<float, 3, Eigen::Affine>& tx, 
    const Eigen::Vector3f& e, 
    Eigen::Vector3f n,
    Eigen::Vector3f* aOut, 
    Eigen::Vector3f* bOut
);

} // namespace CollisionUtils