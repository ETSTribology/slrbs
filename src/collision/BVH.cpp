#include "collision/BVH.h"
#include "polyscope/polyscope.h"
#include "polyscope/curve_network.h"
#include <algorithm>
#include <queue>
#include <Eigen/Dense>
#include <numeric>

void BVH::build(const std::vector<glm::vec3>& vertices, const std::vector<glm::ivec3>& triangles) {
    // Initialize primitive indices sequentially
    std::vector<size_t> primitiveIndices(triangles.size());
    std::iota(primitiveIndices.begin(), primitiveIndices.end(), 0);
    
    // Build the tree recursively
    root = buildRecursive(primitiveIndices, vertices, triangles, 0);
}

std::unique_ptr<BVHNode> BVH::buildRecursive(
    const std::vector<size_t>& primitiveIndices,
    const std::vector<glm::vec3>& vertices,
    const std::vector<glm::ivec3>& triangles,
    int depth
) {
    auto node = std::make_unique<BVHNode>();
    
    // Initialize bounds
    glm::vec3 localMin(std::numeric_limits<float>::max());
    glm::vec3 localMax(-std::numeric_limits<float>::max());
    
    // Use parallel reduction for bounds computation if enough primitives
    if (primitiveIndices.size() > 1000) {
        const int num_threads = omp_get_max_threads();
        std::vector<glm::vec3> threadMin(num_threads, glm::vec3(std::numeric_limits<float>::max()));
        std::vector<glm::vec3> threadMax(num_threads, glm::vec3(-std::numeric_limits<float>::max()));
        
        #pragma omp parallel
        {
            int thread_id = omp_get_thread_num();
            #pragma omp for nowait
            for (int i = 0; i < static_cast<int>(primitiveIndices.size()); ++i) {
                const auto& tri = triangles[primitiveIndices[i]];
                const auto& v0 = vertices[tri.x];
                const auto& v1 = vertices[tri.y];
                const auto& v2 = vertices[tri.z];
                
                threadMin[thread_id] = glm::min(threadMin[thread_id], glm::min(glm::min(v0, v1), v2));
                threadMax[thread_id] = glm::max(threadMax[thread_id], glm::max(glm::max(v0, v1), v2));
            }
        }
        
        // Combine thread results
        for (int t = 0; t < num_threads; ++t) {
            localMin = glm::min(localMin, threadMin[t]);
            localMax = glm::max(localMax, threadMax[t]);
        }
    } else {
        // Sequential for small number of primitives
        for (size_t idx : primitiveIndices) {
            const auto& tri = triangles[idx];
            const auto& v0 = vertices[tri.x];
            const auto& v1 = vertices[tri.y];
            const auto& v2 = vertices[tri.z];
            
            localMin = glm::min(localMin, glm::min(glm::min(v0, v1), v2));
            localMax = glm::max(localMax, glm::max(glm::max(v0, v1), v2));
        }
    }
    
    node->bounds.min = localMin;
    node->bounds.max = localMax;
    
    // If few enough primitives or max depth reached, make a leaf
    // Build a binary tree structure like this:
    //            Root Node
    //            /        \
    //      Left Node    Right Node
    //      /     \        /     \
    // Leaf1    Leaf2   Leaf3   Leaf4
    //
    // Each internal node has an AABB and two children
    // Each leaf node has an AABB and triangle indices
    if (depth >= m_maxDepth || primitiveIndices.size() <= 4) {
        node->primitiveIndices = primitiveIndices;
        return node;
    }
    
    // Choose split axis and position
    // Calculate the axis to split along (x, y, or z) based on the extent of the bounding box
    glm::vec3 extent = localMax - localMin;
    int axis = 0;
    if (extent.y > extent.x) axis = 1;
    if (extent.z > extent[axis]) axis = 2;
    
    float splitPos = localMin[axis] + extent[axis] * 0.5f;
    
    // Partition primitives using parallel partitioning for large enough sets
    std::vector<size_t> leftPrims, rightPrims;
    if (primitiveIndices.size() > 1000) {
        std::vector<size_t> leftTemp(primitiveIndices.size());
        std::vector<size_t> rightTemp(primitiveIndices.size());
        std::vector<int> leftCount(omp_get_max_threads() + 1, 0);
        std::vector<int> rightCount(omp_get_max_threads() + 1, 0);
        
        // First pass: count elements for each thread
        #pragma omp parallel
        {
            int thread_id = omp_get_thread_num();
            int local_left = 0, local_right = 0;
            
            #pragma omp for nowait
            for (int i = 0; i < static_cast<int>(primitiveIndices.size()); ++i) {
                size_t idx = primitiveIndices[i];
                const auto& tri = triangles[idx];
                const auto& v0 = vertices[tri.x];
                const auto& v1 = vertices[tri.y];
                const auto& v2 = vertices[tri.z];
                
                glm::vec3 centroid = (v0 + v1 + v2) / 3.0f;
                
                if (centroid[axis] < splitPos) {
                    local_left++;
                } else {
                    local_right++;
                }
            }
            
            leftCount[thread_id + 1] = local_left;
            rightCount[thread_id + 1] = local_right;
        }
        
        // Compute prefix sums
        for (int i = 1; i < leftCount.size(); ++i) {
            leftCount[i] += leftCount[i-1];
            rightCount[i] += rightCount[i-1];
        }
        
        // Second pass: place elements
        #pragma omp parallel
        {
            int thread_id = omp_get_thread_num();
            int left_offset = leftCount[thread_id];
            int right_offset = rightCount[thread_id];
            
            #pragma omp for nowait
            for (int i = 0; i < static_cast<int>(primitiveIndices.size()); ++i) {
                size_t idx = primitiveIndices[i];
                const auto& tri = triangles[idx];
                const auto& v0 = vertices[tri.x];
                const auto& v1 = vertices[tri.y];
                const auto& v2 = vertices[tri.z];
                
                glm::vec3 centroid = (v0 + v1 + v2) / 3.0f;
                
                if (centroid[axis] < splitPos) {
                    leftTemp[left_offset++] = idx;
                } else {
                    rightTemp[right_offset++] = idx;
                }
            }
        }
        
        // Copy to final vectors
        leftPrims = std::vector<size_t>(leftTemp.begin(), leftTemp.begin() + leftCount.back());
        rightPrims = std::vector<size_t>(rightTemp.begin(), rightTemp.begin() + rightCount.back());
    } else {
        // Sequential partition for small sets
        for (size_t idx : primitiveIndices) {
            const auto& tri = triangles[idx];
            const auto& v0 = vertices[tri.x];
            const auto& v1 = vertices[tri.y];
            const auto& v2 = vertices[tri.z];
            
            glm::vec3 centroid = (v0 + v1 + v2) / 3.0f;
            
            if (centroid[axis] < splitPos) {
                leftPrims.push_back(idx);
            } else {
                rightPrims.push_back(idx);
            }
        }
    }
    
    // Handle edge case where all primitives end up on one side
    // If all triangles' centroids fall on one side of the split position, 
    // one child node will end up with all primitives, leading to inefficiencies
    if (leftPrims.empty() || rightPrims.empty()) {
        size_t mid = primitiveIndices.size() / 2;
        leftPrims = std::vector<size_t>(primitiveIndices.begin(), primitiveIndices.begin() + mid);
        rightPrims = std::vector<size_t>(primitiveIndices.begin() + mid, primitiveIndices.end());
    }
    
    // Build children using the recursive function
    node->left = buildRecursive(leftPrims, vertices, triangles, depth + 1);
    node->right = buildRecursive(rightPrims, vertices, triangles, depth + 1);
    
    return node;
}

void BVH::visualize() const {
    if (!root) return;
    
    struct QueueItem {
        const BVHNode* node;
        int depth;
        QueueItem(const BVHNode* n, int d) : node(n), depth(d) {}
    };
    
    std::queue<QueueItem> queue;
    queue.push(QueueItem(root.get(), 0));
    
    std::vector<Eigen::Vector3d> points;
    std::vector<std::array<size_t, 2>> edges;  
    std::vector<Eigen::Vector3d> colors;
    
    // Function to generate a unique color based on depth
    auto getColorForDepth = [](int depth) -> Eigen::Vector3d {
        float hue = fmod(depth * 0.618033988749895f, 1.0f);
        float saturation = 0.8f;
        float value = 1.0f;
        
        float c = value * saturation;
        float x = c * (1 - std::abs(fmod(hue * 6, 2) - 1));
        float m = value - c;
        
        float r, g, b;
        if(hue < 1.0f/6.0f)      { r = c; g = x; b = 0; }
        else if(hue < 2.0f/6.0f) { r = x; g = c; b = 0; }
        else if(hue < 3.0f/6.0f) { r = 0; g = c; b = x; }
        else if(hue < 4.0f/6.0f) { r = 0; g = x; b = c; }
        else if(hue < 5.0f/6.0f) { r = x; g = 0; b = c; }
        else                     { r = c; g = 0; b = x; }
        
        return Eigen::Vector3d(r + m, g + m, b + m);
    };
    
    while (!queue.empty()) {
        const auto& item = queue.front();
        const BVHNode* node = item.node;
        int depth = item.depth;
        queue.pop();
        
        // Add box vertices
        size_t baseIdx = points.size();
        std::array<glm::vec3, 8> boxVertices = {
            glm::vec3(node->bounds.min.x, node->bounds.min.y, node->bounds.min.z),
            glm::vec3(node->bounds.max.x, node->bounds.min.y, node->bounds.min.z),
            glm::vec3(node->bounds.max.x, node->bounds.max.y, node->bounds.min.z),
            glm::vec3(node->bounds.min.x, node->bounds.max.y, node->bounds.min.z),
            glm::vec3(node->bounds.min.x, node->bounds.min.y, node->bounds.max.z),
            glm::vec3(node->bounds.max.x, node->bounds.min.y, node->bounds.max.z),
            glm::vec3(node->bounds.max.x, node->bounds.max.y, node->bounds.max.z),
            glm::vec3(node->bounds.min.x, node->bounds.max.y, node->bounds.max.z)
        };
        
        // Get color based on depth
        Eigen::Vector3d color = getColorForDepth(depth);
        
        // Add vertices and their colors
        for (const auto& v : boxVertices) {
            points.push_back(Eigen::Vector3d(v.x, v.y, v.z));
            colors.push_back(color);
        }
        
        // Add box edges
        std::array<std::array<size_t, 2>, 12> boxEdges = {{
            {baseIdx + 0, baseIdx + 1}, {baseIdx + 1, baseIdx + 2},
            {baseIdx + 2, baseIdx + 3}, {baseIdx + 3, baseIdx + 0},
            {baseIdx + 4, baseIdx + 5}, {baseIdx + 5, baseIdx + 6},
            {baseIdx + 6, baseIdx + 7}, {baseIdx + 7, baseIdx + 4},
            {baseIdx + 0, baseIdx + 4}, {baseIdx + 1, baseIdx + 5},
            {baseIdx + 2, baseIdx + 6}, {baseIdx + 3, baseIdx + 7}
        }};
        
        for (const auto& edge : boxEdges) {
            edges.push_back(edge);
        }
        
        if (node->left) queue.push(QueueItem(node->left.get(), depth + 1));
        if (node->right) queue.push(QueueItem(node->right.get(), depth + 1));
    }
    
    // Create curve network for edges
    auto* network = polyscope::registerCurveNetwork("bvh_hierarchy", points, edges);
    network->setRadius(0.001);  
    
    // Add and enable vertex colors
    auto* colorQ = network->addNodeColorQuantity("hierarchy_level", colors);
    colorQ->setEnabled(true);
}