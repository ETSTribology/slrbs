#pragma once

#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <omp.h>  // Add OpenMP header

struct VisualizationAABB {
    // Polyscope uses GLM internally for its rendering pipeline (32-bit floats)
    // The members are meant to be directly accessible without getters/setters
    glm::vec3 min;
    glm::vec3 max;

    VisualizationAABB() : min(glm::vec3(0)), max(glm::vec3(0)) {}
    VisualizationAABB(const glm::vec3& min, const glm::vec3& max) : min(min), max(max) {}

    glm::vec3 getCenter() const {
        return (min + max) * 0.5f;
    } // The const qualifier guarantees that the member function does not change any member variables of the object, except those marked as mutable.

    float getSurfaceArea() const {
        glm::vec3 d = max - min;
        return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }
};

class BVHNode {
public:
    VisualizationAABB bounds;
    std::unique_ptr<BVHNode> left;
    std::unique_ptr<BVHNode> right;
    std::vector<size_t> primitiveIndices;  // Indices into the mesh's triangle array

    BVHNode() = default;

    bool isLeaf() const {
        return !left && !right;
    }
};

// BVH (Bounding Volume Hierarchy) is a tree structure that partitions 3D space
// It is used to accelerate collision detection and spatial queries
class BVH {
public:
    // Initialize BVH with a default maximum tree depth
    BVH() : m_maxDepth(10) {}  // Deeper trees are more precise but require more memory

    // Template interface function that provides flexibility in face index representation
    // This is a wrapper that converts any valid face type to our internal ivec3 format
    // Parameters:
    //   vertices: 3D vertex positions of the mesh
    //   triangles: Face indices in any compatible format (must support operator[])
    // Example face types: std::array<size_t,3>, std::array<int,3>, custom_triangle_type
    // Usage:
    //   std::vector<std::array<size_t, 3>> faces;
    //   bvh.build(vertices, faces);  // Will convert to ivec3 internally
    template <typename FaceType>
    void build(const std::vector<glm::vec3>& vertices, const std::vector<FaceType>& triangles) {
        build(vertices, convertFacesToIVec3(triangles));
    }

    // Core implementation function that constructs the BVH tree
    // Always works with glm::ivec3 for consistency and GPU compatibility
    // Parameters:
    //   vertices: 3D vertex positions of the mesh (vec3 for floating-point precision)
    //   triangles: Face indices as ivec3 (integer vector for vertex indices)
    // This function:
    //   1. Computes AABBs for all triangles
    //   2. Creates the root node
    //   3. Recursively partitions space and triangles
    //   4. Builds the complete BVH tree structure
    void build(const std::vector<glm::vec3>& vertices, const std::vector<glm::ivec3>& triangles);

    // Creates a visual representation of the BVH structure using Polyscope
    void visualize() const;

    // Control the maximum depth of the BVH tree
    // Deeper trees: more precise spatial partitioning but more memory usage
    // Shallower trees: less memory but potentially slower queries
    void setMaxDepth(int depth) { m_maxDepth = depth; }
    int getMaxDepth() const { return m_maxDepth; }

private:
    std::unique_ptr<BVHNode> root;              // Root node of the BVH tree
    int m_maxDepth;                             // Maximum allowed tree depth

    // Recursive function to build the BVH tree (for array-based faces)
    // primitiveIndices: Indices of triangles in this node
    // vertices: All mesh vertices
    // triangles: All mesh triangles
    // depth: Current depth in the tree
    std::unique_ptr<BVHNode> buildRecursive(
        const std::vector<size_t>& primitiveIndices,
        const std::vector<glm::vec3>& vertices,
        const std::vector<std::array<size_t, 3>>& triangles,
        int depth
    );

    // Overloaded buildRecursive for ivec3-based faces
    // Same functionality but different triangle index type
    std::unique_ptr<BVHNode> buildRecursive(
        const std::vector<size_t>& primitiveIndices,
        const std::vector<glm::vec3>& vertices,
        const std::vector<glm::ivec3>& triangles,
        int depth
    );

    // Compute the AABB that encloses a triangle
    // v0, v1, v2: The three vertices of the triangle
    // Returns: AABB that tightly bounds the triangle
    VisualizationAABB computeTriangleBounds(
        const glm::vec3& v0,
        const glm::vec3& v1,
        const glm::vec3& v2
    ) const;

    // Helper function to convert different face types to glm::ivec3
    // Used to provide a consistent interface regardless of input face format
    // For static functions: No effect because they can't access class members anyway, so no const needed
    template <typename FaceType>
    static std::vector<glm::ivec3> convertFacesToIVec3(const std::vector<FaceType>& faces) {
        std::vector<glm::ivec3> convertedFaces(faces.size());  // Pre-allocate with size

        // Only parallelize for large enough face sets to avoid overhead
        // Memory Overhead:
        // Thread-local storage allocation
        // Cache coherency maintenance
        // Memory barriers and synchronization
        if (faces.size() > 1000) {
            #pragma omp parallel for
            for (int i = 0; i < static_cast<int>(faces.size()); ++i) {
                const auto& face = faces[i];
                convertedFaces[i] = glm::ivec3(face[0], face[1], face[2]);
            }
        } else {
            // Sequential for small sets
            for (size_t i = 0; i < faces.size(); ++i) {
                const auto& face = faces[i];
                convertedFaces[i] = glm::ivec3(face[0], face[1], face[2]);
            }
        }
        return convertedFaces;
    }
};