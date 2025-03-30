#pragma once

#include "renderer/RigidBodyRenderer.h"
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <glm/glm.hpp>

namespace polyscope {
    class SurfaceMesh;
    class PointCloud;
    class CurveNetwork;
}

namespace slrbs {

class RigidBody;
class Contact;
class Joint;
class MaterialLibrary;
class TextureManager;

/**
 * @class PolyscopeRenderer
 * @brief Implementation of RigidBodyRenderer using Polyscope for visualization
 */
class PolyscopeRenderer : public RigidBodyRenderer {
public:
    PolyscopeRenderer();
    ~PolyscopeRenderer() override;
    
    // Initialize the renderer
    void initialize() override;
    
    // Update methods
    void updateAll() override;
    void updateBodyTransform(const RigidBody& body) override;
    
    // Render methods
    void renderBodies(const std::vector<RigidBody*>& bodies) override;
    void renderContacts(const std::vector<Contact*>& contacts) override;
    void renderJoints(const std::vector<Joint*>& joints) override;
    void renderBoundingBoxes(const std::vector<RigidBody*>& bodies) override;
    
    // Material assignment
    void assignMaterial(RigidBody& body, const std::string& materialName) override;
    
    // Texture and color assignment
    void setBodyColor(RigidBody& body, const glm::vec3& color) override;
    void setBodyTexture(RigidBody& body, const std::string& texturePath) override;
    
private:
    // Helper methods
    void createBodyMesh(RigidBody& body);
    
    // Material and texture management
    std::unique_ptr<MaterialLibrary> materialLibrary;
    std::unique_ptr<TextureManager> textureManager;
    
    // Track polyscope mesh objects
    std::unordered_map<const RigidBody*, polyscope::SurfaceMesh*> bodyMeshes;
    
    // Track body textures
    std::unordered_map<const RigidBody*, std::string> bodyTextures;
};

} // namespace slrbs
