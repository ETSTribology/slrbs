#include "collision/CollisionUtils.h"
#include <cassert>
#include <cmath>
#include <algorithm>

namespace CollisionUtils {

float distancePointPlane(const Eigen::Vector3f& p, const Eigen::Vector3f& plane_p, const Eigen::Vector3f& plane_n) {
    return (p - plane_p).dot(plane_n);
}

bool TrackFaceAxis(int* axis, int n, float s, float* sMax, const Eigen::Vector3f& normal, Eigen::Vector3f* axisNormal) {
    if (s > 0.0f)
        return true;
    if (s > *sMax) {
        *sMax = s;
        *axis = n;
        *axisNormal = normal;
    }
    return false;
}

bool TrackEdgeAxis(int* axis, int n, float s, float* sMax, const Eigen::Vector3f& normal, Eigen::Vector3f* axisNormal) {
    if (s > 0.0f)
        return true;
    float l = 1.0f / normal.norm();
    s *= l;
    if (s > *sMax) {
        *sMax = s;
        *axis = n;
        *axisNormal = normal * l;
    }
    return false;
}

void ComputeReferenceEdgesAndBasis(const Eigen::Vector3f& eR, const Eigen::Transform<float, 3, Eigen::Affine>& rtx,
    Eigen::Vector3f n, int axis, uint8_t* out, Eigen::Matrix3f* basis, Eigen::Vector3f* e)
{
    n = rtx.rotation().transpose() * n;
    if (axis >= 3)
        axis -= 3;
    
    switch (axis)
    {
    case 0:
        if (n.x() > 0.0f) {
            out[0] = 1; out[1] = 8; out[2] = 7; out[3] = 9;
            (*basis)(0, 0) = 0.0f; (*basis)(0, 1) = 0.0f; (*basis)(0, 2) = 1.0f;
            (*basis)(1, 0) = 0.0f; (*basis)(1, 1) = 1.0f; (*basis)(1, 2) = 0.0f;
            (*basis)(2, 0) = 1.0f; (*basis)(2, 1) = 0.0f; (*basis)(2, 2) = 0.0f;
            e->x() = eR.z(); e->y() = eR.y(); e->z() = eR.x();
        }
        else {
            out[0] = 11; out[1] = 3; out[2] = 10; out[3] = 5;
            (*basis)(0, 0) = 0.0f; (*basis)(0, 1) = 0.0f; (*basis)(0, 2) = -1.0f;
            (*basis)(1, 0) = 0.0f; (*basis)(1, 1) = 1.0f; (*basis)(1, 2) = 0.0f;
            (*basis)(2, 0) = -1.0f; (*basis)(2, 1) = 0.0f; (*basis)(2, 2) = 0.0f;
            e->x() = eR.z(); e->y() = eR.y(); e->z() = eR.x();
        }
        break;

    case 1:
        if (n.y() > 0.0f) {
            out[0] = 0; out[1] = 1; out[2] = 2; out[3] = 3;
            (*basis)(0, 0) = 1.0f; (*basis)(0, 1) = 0.0f; (*basis)(0, 2) = 0.0f;
            (*basis)(1, 0) = 0.0f; (*basis)(1, 1) = 0.0f; (*basis)(1, 2) = 1.0f;
            (*basis)(2, 0) = 0.0f; (*basis)(2, 1) = 1.0f; (*basis)(2, 2) = 0.0f;
            e->x() = eR.x(); e->y() = eR.z(); e->z() = eR.y();
        }
        else {
            out[0] = 4; out[1] = 5; out[2] = 6; out[3] = 7;
            (*basis)(0, 0) = 1.0f; (*basis)(0, 1) = 0.0f; (*basis)(0, 2) = 0.0f;
            (*basis)(1, 0) = 0.0f; (*basis)(1, 1) = 0.0f; (*basis)(1, 2) = -1.0f;
            (*basis)(2, 0) = 0.0f; (*basis)(2, 1) = -1.0f; (*basis)(2, 2) = 0.0f;
            e->x() = eR.x(); e->y() = eR.z(); e->z() = eR.y();
        }
        break;

    case 2:
        if (n.z() > 0.0f) {
            out[0] = 11; out[1] = 4; out[2] = 8; out[3] = 0;
            (*basis)(0, 0) = 0.0f; (*basis)(0, 1) = 1.0f; (*basis)(0, 2) = 0.0f;
            (*basis)(1, 0) = 1.0f; (*basis)(1, 1) = 0.0f; (*basis)(1, 2) = 0.0f;
            (*basis)(2, 0) = 0.0f; (*basis)(2, 1) = 0.0f; (*basis)(2, 2) = 1.0f;
            e->x() = eR.y(); e->y() = eR.x(); e->z() = eR.z();
        }
        else {
            out[0] = 6; out[1] = 10; out[2] = 2; out[3] = 9;
            (*basis)(0, 0) = 0.0f; (*basis)(0, 1) = -1.0f; (*basis)(0, 2) = 0.0f;
            (*basis)(1, 0) = 1.0f; (*basis)(1, 1) = 0.0f; (*basis)(1, 2) = 0.0f;
            (*basis)(2, 0) = 0.0f; (*basis)(2, 1) = 0.0f; (*basis)(2, 2) = -1.0f;
            e->x() = eR.y(); e->y() = eR.x(); e->z() = eR.z();
        }
        break;

    default:
        assert(false);
        break;
    }

    *basis = rtx.rotation() * (*basis);
}

void ComputeIncidentFace(const Eigen::Transform<float, 3, Eigen::Affine>& itx, 
                         const Eigen::Vector3f& e, Eigen::Vector3f n, ClipVertex* out) 
{
    n = -itx.rotation().transpose() * n;
    Eigen::Vector3f absN = n.cwiseAbs();
    
    if (absN.x() > absN.y() && absN.x() > absN.z()) {
        if (n.x() > 0.0f) {
            out[0].v = Eigen::Vector3f(e.x(), e.y(), -e.z());
            out[1].v = Eigen::Vector3f(e.x(), e.y(), e.z());
            out[2].v = Eigen::Vector3f(e.x(), -e.y(), e.z());
            out[3].v = Eigen::Vector3f(e.x(), -e.y(), -e.z());
            out[0].f.inI = 9; out[0].f.outI = 1;
            out[1].f.inI = 1; out[1].f.outI = 8;
            out[2].f.inI = 8; out[2].f.outI = 7;
            out[3].f.inI = 7; out[3].f.outI = 9;
        } else {
            out[0].v = Eigen::Vector3f(-e.x(), -e.y(), e.z());
            out[1].v = Eigen::Vector3f(-e.x(), e.y(), e.z());
            out[2].v = Eigen::Vector3f(-e.x(), e.y(), -e.z());
            out[3].v = Eigen::Vector3f(-e.x(), -e.y(), -e.z());
            out[0].f.inI = 5; out[0].f.outI = 11;
            out[1].f.inI = 11; out[1].f.outI = 3;
            out[2].f.inI = 3; out[2].f.outI = 10;
            out[3].f.inI = 10; out[3].f.outI = 5;
        }
    } else if (absN.y() > absN.z()) {
        if (n.y() > 0.0f) {
            out[0].v = Eigen::Vector3f(-e.x(), e.y(), -e.z());
            out[1].v = Eigen::Vector3f(e.x(), e.y(), -e.z());
            out[2].v = Eigen::Vector3f(e.x(), e.y(), e.z());
            out[3].v = Eigen::Vector3f(-e.x(), e.y(), e.z());
            out[0].f.inI = 3; out[0].f.outI = 0;
            out[1].f.inI = 0; out[1].f.outI = 1;
            out[2].f.inI = 1; out[2].f.outI = 2;
            out[3].f.inI = 2; out[3].f.outI = 3;
        } else {
            out[0].v = Eigen::Vector3f(e.x(), -e.y(), -e.z());
            out[1].v = Eigen::Vector3f(-e.x(), -e.y(), -e.z());
            out[2].v = Eigen::Vector3f(-e.x(), -e.y(), e.z());
            out[3].v = Eigen::Vector3f(e.x(), -e.y(), e.z());
            out[0].f.inI = 7; out[0].f.outI = 4;
            out[1].f.inI = 4; out[1].f.outI = 5;
            out[2].f.inI = 5; out[2].f.outI = 6;
            out[3].f.inI = 6; out[3].f.outI = 7;
        }
    } else {
        if (n.z() > 0.0f) {
            out[0].v = Eigen::Vector3f(-e.x(), e.y(), e.z());
            out[1].v = Eigen::Vector3f(e.x(), e.y(), e.z());
            out[2].v = Eigen::Vector3f(e.x(), -e.y(), e.z());
            out[3].v = Eigen::Vector3f(-e.x(), -e.y(), e.z());
            out[0].f.inI = 2; out[0].f.outI = 11;
            out[1].f.inI = 1; out[1].f.outI = 2;
            out[2].f.inI = 6; out[2].f.outI = 1;
            out[3].f.inI = 5; out[3].f.outI = 6;
        } else {
            out[0].v = Eigen::Vector3f(e.x(), -e.y(), -e.z());
            out[1].v = Eigen::Vector3f(e.x(), e.y(), -e.z());
            out[2].v = Eigen::Vector3f(-e.x(), e.y(), -e.z());
            out[3].v = Eigen::Vector3f(-e.x(), -e.y(), -e.z());
            out[0].f.inI = 9; out[0].f.outI = 4;
            out[1].f.inI = 0; out[1].f.outI = 9;
            out[2].f.inI = 3; out[2].f.outI = 0;
            out[3].f.inI = 10; out[3].f.outI = 3;
        }
    }

    // Transform incident face vertices to world space
    for (int i = 0; i < 4; ++i)
        out[i].v = itx * out[i].v;
}

int Orthographic(float sign, float e, int axis, int clipEdge, ClipVertex* in, int inCount, ClipVertex* out) {
    int outCount = 0;
    ClipVertex a = in[inCount - 1];
    
    for (int i = 0; i < inCount; ++i) {
        ClipVertex b = in[i];
        float da = sign * a.v[axis] - e;
        float db = sign * b.v[axis] - e;
        ClipVertex cv;
        
        // Both points are inside the plane (or on the plane within tolerance)
        if (((da < 0.0f && db < 0.0f) || std::abs(da) < 0.005f || std::abs(db) < 0.005f)) {
            assert(outCount < 8);
            out[outCount++] = b;
        }
        // Point a is outside, point b is inside - compute intersection
        else if (da < 0.0f && db >= 0.0f) {
            cv.f = b.f;
            cv.v = a.v + (b.v - a.v) * (da / (da - db));
            cv.f.outR = clipEdge;
            cv.f.outI = 0;
            assert(outCount < 8);
            out[outCount++] = cv;
        }
        // Point a is inside, point b is outside - compute intersection and keep b
        else if (da >= 0.0f && db < 0.0f) {
            cv.f = a.f;
            cv.v = a.v + (b.v - a.v) * (da / (da - db));
            cv.f.inR = clipEdge;
            cv.f.inI = 0;
            assert(outCount < 8);
            out[outCount++] = cv;
            assert(outCount < 8);
            out[outCount++] = b;
        }
        // Both points outside, no output
        
        a = b;
    }
    
    return outCount;
}

int Clip(const Eigen::Vector3f& rPos, const Eigen::Vector3f& e, uint8_t* clipEdges, const Eigen::Matrix3f& basis,
    ClipVertex* incident, ClipVertex* outVerts, float* outDepths)
{
    int inCount = 4, outCount = 0;
    ClipVertex in[8], out[8];
    
    // Transform incident face vertices to reference face coordinate system
    for (int i = 0; i < 4; ++i)
        in[i].v = basis.transpose() * (incident[i].v - rPos);
    
    // Clip against each of the four sides of the reference face
    outCount = Orthographic(1.0f, e.x(), 0, clipEdges[0], in, inCount, out);
    if (outCount == 0) return 0;
    
    inCount = Orthographic(1.0f, e.y(), 1, clipEdges[1], out, outCount, in);
    if (inCount == 0) return 0;
    
    outCount = Orthographic(-1.0f, e.x(), 0, clipEdges[2], in, inCount, out);
    if (outCount == 0) return 0;
    
    inCount = Orthographic(-1.0f, e.y(), 1, clipEdges[3], out, outCount, in);
    
    // Calculate contact points and penetration depths
    outCount = 0;
    for (int i = 0; i < inCount; ++i) {
        // Check depth against reference face
        float d = in[i].v.z() - e.z();
        if (d <= 0.0f) {
            // Transform back to world space
            outVerts[outCount].v = basis * in[i].v + rPos;
            outVerts[outCount].f = in[i].f;
            outDepths[outCount++] = d;
        }
    }
    
    assert(outCount <= 8);
    return outCount;
}

void EdgesContact(Eigen::Vector3f* CA, Eigen::Vector3f* CB, const Eigen::Vector3f& PA, const Eigen::Vector3f& QA,
    const Eigen::Vector3f& PB, const Eigen::Vector3f& QB) 
{
    // Direction vectors for the two edges
    Eigen::Vector3f DA = QA - PA;
    Eigen::Vector3f DB = QB - PB;
    
    // Vector between edge starting points
    Eigen::Vector3f r = PA - PB;
    
    // Compute dot products for solving parameters
    float a = DA.dot(DA);  // |DA|^2
    float e = DB.dot(DB);  // |DB|^2
    float f = DB.dot(r);   // DB · r
    float c = DA.dot(r);   // DA · r
    float b = DA.dot(DB);  // DA · DB
    
    // Compute denominator for solving the system
    float denom = a * e - b * b;
    
    // Compute parameters for closest points on both edges
    float TA = (b * f - c * e) / denom;
    float TB = (b * TA + f) / e;
    
    // Calculate the closest points on each edge
    *CA = PA + DA * TA;
    *CB = PB + DB * TB;
}

void SupportEdge(const Eigen::Transform<float, 3, Eigen::Affine>& tx, const Eigen::Vector3f& e, Eigen::Vector3f n,
    Eigen::Vector3f* aOut, Eigen::Vector3f* bOut) 
{
    // Transform normal to local space
    n = tx.rotation().transpose() * n;
    Eigen::Vector3f absN = n.cwiseAbs();
    
    // Find appropriate edge based on normal direction
    Eigen::Vector3f a, b;
    if (absN.x() > absN.y()) {
        if (absN.y() > absN.z()) { 
            a = Eigen::Vector3f(e.x(), e.y(), e.z()); 
            b = Eigen::Vector3f(e.x(), e.y(), -e.z()); 
        }
        else { 
            a = Eigen::Vector3f(e.x(), e.y(), e.z()); 
            b = Eigen::Vector3f(e.x(), -e.y(), e.z()); 
        }
    } else {
        if (absN.x() > absN.z()) { 
            a = Eigen::Vector3f(e.x(), e.y(), e.z()); 
            b = Eigen::Vector3f(e.x(), e.y(), -e.z()); 
        }
        else { 
            a = Eigen::Vector3f(e.x(), e.y(), e.z()); 
            b = Eigen::Vector3f(-e.x(), e.y(), e.z()); 
        }
    }
    
    // Orient the edge vertices based on normal direction
    float signx = n.x() >= 0.0f ? 1.0f : -1.0f;
    float signy = n.y() >= 0.0f ? 1.0f : -1.0f;
    float signz = n.z() >= 0.0f ? 1.0f : -1.0f;
    
    a.x() *= signx; a.y() *= signy; a.z() *= signz;
    b.x() *= signx; b.y() *= signy; b.z() *= signz;
    
    // Transform edge vertices to world space
    *aOut = tx * a; 
    *bOut = tx * b;
}

} // namespace CollisionUtils