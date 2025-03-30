#pragma once

#include "renderer/RigidBodyRenderer.h"
#include <memory>
#include <string>

namespace slrbs {

/**
 * @class RendererFactory
 * @brief Factory to create appropriate renderer implementation
 */
class RendererFactory {
public:
    /**
     * @brief Create a renderer of the specified type
     * @param type Name of the renderer type ("polyscope", etc.)
     * @return Unique pointer to the created renderer
     */
    static std::unique_ptr<RigidBodyRenderer> createRenderer(const std::string& type);
    
    /**
     * @brief Create default renderer (currently Polyscope)
     * @return Unique pointer to the created renderer
     */
    static std::unique_ptr<RigidBodyRenderer> createDefaultRenderer();
};

} // namespace slrbs
