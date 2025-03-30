#include "renderer/RendererFactory.h"
#include "renderer/PolyscopeRenderer.h"
#include <iostream>

namespace slrbs {

std::unique_ptr<RigidBodyRenderer> RendererFactory::createRenderer(const std::string& type) {
    if (type == "polyscope") {
        return std::make_unique<PolyscopeRenderer>();
    }
    
    // Add other renderer types here
    
    // If unknown type, return default renderer
    std::cerr << "Unknown renderer type: " << type << ". Using default renderer." << std::endl;
    return createDefaultRenderer();
}

std::unique_ptr<RigidBodyRenderer> RendererFactory::createDefaultRenderer() {
    return std::make_unique<PolyscopeRenderer>();
}

} // namespace slrbs
