#pragma once

#include "collision/CollisionDetect.h"
#include "collision/AABB.h"
#include "collision/BVH.h"
#include <vector>


class CollisionDetectBVH : public CollisionDetect {
public:
    CollisionDetectBVH(RigidBodySystem* sys);
    ~CollisionDetectBVH();
    void detectCollisions();

private:
    BVHNode* m_root = nullptr;
    std::vector<int> m_indices;

    BVHNode* buildBVH(int start, int end);
    void deleteBVH(BVHNode* node);
    void queryPairs(BVHNode* a, BVHNode* b, std::vector<std::pair<int,int>>& out);
    AABB computeAABB(RigidBody* b);
};