#ifndef COLLISION_AABB_H
#define COLLISION_AABB_H

#include <Eigen/Dense>
#include <vector>

class AABB {
public:
    AABB() {
        min.setConstant(std::numeric_limits<double>::max());
        max.setConstant(-std::numeric_limits<double>::max());
    }

    AABB(const Eigen::Vector3d& min, const Eigen::Vector3d& max)
        : min(min), max(max) {}

    // Compute AABB from a set of vertices
    static AABB fromVertices(const std::vector<Eigen::Vector3d>& vertices) {
        AABB box;
        for (const auto& v : vertices) {
            box.expand(v);
        }
        return box;
    }

    // Expand AABB to include a point
    void expand(const Eigen::Vector3d& point) {
        min = min.cwiseMin(point);
        max = max.cwiseMax(point);
    }

    // Check if this AABB overlaps with another
    bool overlaps(const AABB& other) const {
        return (min.x() <= other.max.x() && max.x() >= other.min.x()) &&
               (min.y() <= other.max.y() && max.y() >= other.min.y()) &&
               (min.z() <= other.max.z() && max.z() >= other.min.z());
    }

    // Get center point
    Eigen::Vector3d getCenter() const {
        return (min + max) * 0.5;
    }

    // Get half-extents
    Eigen::Vector3d getHalfExtents() const {
        return (max - min) * 0.5;
    }

    // Transform AABB
    AABB transform(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) const {
        // Start with the translation
        Eigen::Vector3d newMin = translation;
        Eigen::Vector3d newMax = translation;

        // For each axis of the rotation matrix
        for (int i = 0; i < 3; ++i) {
            Eigen::Vector3d axis = rotation.col(i);
            Eigen::Vector3d contribution = axis * min[i];
            Eigen::Vector3d contributionMax = axis * max[i];

            // Update the bounds
            for (int j = 0; j < 3; ++j) {
                double minContrib = std::min(contribution[j], contributionMax[j]);
                double maxContrib = std::max(contribution[j], contributionMax[j]);
                newMin[j] += minContrib;
                newMax[j] += maxContrib;
            }
        }

        return AABB(newMin, newMax);
    }

    Eigen::Vector3d min;
    Eigen::Vector3d max;
};

#endif // COLLISION_AABB_H