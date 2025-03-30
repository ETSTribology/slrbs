#pragma once

#include <Eigen/Core>
#include <glm/glm.hpp>

namespace slrbs {
namespace common {

/**
 * @brief Utility functions for rendering and conversion between math libraries
 */
class RenderUtils {
public:
    /**
     * @brief Convert Eigen matrix to GLM matrix
     * @param m Input Eigen matrix
     * @return Equivalent GLM matrix
     */
    static glm::mat4 eigenToGLM(const Eigen::Matrix4f& m);
    
    /**
     * @brief Convert Eigen isometry to GLM matrix
     * @param iso Input Eigen isometry
     * @return Equivalent GLM matrix
     */
    static glm::mat4 isometryToGLM(const Eigen::Isometry3f& iso);
    
    /**
     * @brief Convert Eigen vector to GLM vector
     * @param v Input Eigen vector
     * @return Equivalent GLM vector
     */
    static glm::vec3 eigenToGLM(const Eigen::Vector3f& v);
    
    /**
     * @brief Convert GLM vector to Eigen vector
     * @param v Input GLM vector
     * @return Equivalent Eigen vector
     */
    static Eigen::Vector3f glmToEigen(const glm::vec3& v);
};

} // namespace common
} // namespace slrbs
