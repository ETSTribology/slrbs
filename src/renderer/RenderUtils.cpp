#include "common/RenderUtils.h"

namespace slrbs {
namespace common {

glm::mat4 RenderUtils::eigenToGLM(const Eigen::Matrix4f& m) {
    glm::mat4 result;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result[j][i] = m(i, j);  // Note the transposition (Eigen is column-major, GLM is column-major but accessed differently)
        }
    }
    return result;
}

glm::mat4 RenderUtils::isometryToGLM(const Eigen::Isometry3f& iso) {
    // Extract the 3x3 rotation matrix and 3x1 translation vector
    Eigen::Matrix3f rot = iso.linear();
    Eigen::Vector3f trans = iso.translation();
    
    // Create a 4x4 matrix with the rotation and translation
    glm::mat4 result(1.0f); // Identity matrix
    
    // Fill in rotation part
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[j][i] = rot(i, j);
        }
    }
    
    // Fill in translation part
    result[3][0] = trans.x();
    result[3][1] = trans.y();
    result[3][2] = trans.z();
    
    return result;
}

glm::vec3 RenderUtils::eigenToGLM(const Eigen::Vector3f& v) {
    return glm::vec3(v.x(), v.y(), v.z());
}

Eigen::Vector3f RenderUtils::glmToEigen(const glm::vec3& v) {
    return Eigen::Vector3f(v.x, v.y, v.z);
}

} // namespace common
} // namespace slrbs
