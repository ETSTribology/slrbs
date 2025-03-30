#include "renderer/PolyscopeRenderer.h"
#include "renderer/Material.h"
#include "renderer/TextureManager.h"
#include "rigidbody/RigidBody.h"
#include "joint/Joint.h"
#include "contact/Contact.h"
#include "common/RenderUtils.h"

#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>
#include <polyscope/curve_network.h>

#include <iostream>
#include <limits>

namespace slrbs {

PolyscopeRenderer::PolyscopeRenderer() {
    materialLibrary = std::make_unique<MaterialLibrary>();
    textureManager = std::make_unique<TextureManager>();
}

PolyscopeRenderer::~PolyscopeRenderer() {
    // Clean up mesh references
    bodyMeshes.clear();
}

void PolyscopeRenderer::initialize() {
    // Initialize materials
    materialLibrary->initialize();
    
    // Set visualization defaults
    setShowContacts(true);
    setShowJoints(true);
    setShowBoundingBoxes(false);
    
    std::cout << "PolyscopeRenderer initialized" << std::endl;
}

void PolyscopeRenderer::updateAll() {
    // Update all visual representations
    for (const auto& [body, mesh] : bodyMeshes) {
        if (body && mesh) {
            updateBodyTransform(*body);
        }
    }
}

void PolyscopeRenderer::updateBodyTransform(const RigidBody& body) {
    auto it = bodyMeshes.find(&body);
    if (it != bodyMeshes.end() && it->second) {
        // Create transformation matrix
        Eigen::Isometry3f tm = Eigen::Isometry3f::Identity();
        
        // Copy rotation part
        tm.linear() = body.q.toRotationMatrix();
        
        // Copy translation part
        tm.translation() = body.x;
        
        // Update mesh transform in polyscope
        it->second->setTransform(common::RenderUtils::isometryToGLM(tm));
    }
}

void PolyscopeRenderer::createBodyMesh(RigidBody& body) {
    // Skip if already created
    if (bodyMeshes.find(&body) != bodyMeshes.end()) {
        return;
    }
    
    // Create mesh based on geometry type
    if (body.meshV.size() > 0 && body.meshF.size() > 0) {
        // Convert vertices to glm::vec3
        std::vector<glm::vec3> vertices;
        vertices.reserve(body.meshV.size());
        for (const auto& v : body.meshV) {
            vertices.push_back(common::RenderUtils::eigenToGLM(v));
        }
        
        // Convert faces to expected format
        std::vector<std::vector<size_t>> faces;
        faces.reserve(body.meshF.size());
        for (const auto& f : body.meshF) {
            faces.push_back({
                static_cast<size_t>(f.x()),
                static_cast<size_t>(f.y()),
                static_cast<size_t>(f.z())
            });
        }
        
        // Register mesh with polyscope
        auto mesh = polyscope::registerSurfaceMesh(
            body.name, vertices, faces
        );
        
        // Apply color
        if (body.color.norm() > 0) {
            glm::vec3 color = common::RenderUtils::eigenToGLM(body.color);
            mesh->setSurfaceColor(color);
        }
        
        // Store reference
        bodyMeshes[&body] = mesh;
        
        // Set default material
        mesh->setMaterial("clay");
        
        // Apply texture if available
        if (body.texturePath.length() > 0) {
            setBodyTexture(body, body.texturePath);
        }
    }
}

void PolyscopeRenderer::setBodyColor(RigidBody& body, const glm::vec3& color) {
    auto it = bodyMeshes.find(&body);
    if (it != bodyMeshes.end() && it->second) {
        it->second->setSurfaceColor(color);
        
        // Store the color in eigen format for serialization
        body.color = common::RenderUtils::glmToEigen(color);
    }
}

void PolyscopeRenderer::setBodyTexture(RigidBody& body, const std::string& texturePath) {
    // Skip if texture path is empty
    if (texturePath.empty()) {
        return;
    }
    
    auto it = bodyMeshes.find(&body);
    if (it != bodyMeshes.end() && it->second) {
        // Load texture through texture manager
        std::string textureId = textureManager->loadTexture(texturePath);
        
        if (!textureId.empty()) {
            // Apply texture to polyscope mesh
            // Note: Polyscope doesn't support direct texture assignment in its API
            // We'd need a custom render pass, but for now we just record the texture
            bodyTextures[&body] = textureId;
            
            // Store texture path in the rigid body for serialization
            body.texturePath = texturePath;
            
            std::cout << "Texture applied to " << body.name << ": " << texturePath << std::endl;
        }
    }
}

void PolyscopeRenderer::renderBodies(const std::vector<RigidBody*>& bodies) {
    for (auto* body : bodies) {
        if (!body) continue;
        
        // Create mesh if needed
        if (bodyMeshes.find(body) == bodyMeshes.end()) {
            createBodyMesh(*body);
        }
        
        // Update transform
        updateBodyTransform(*body);
    }
}

void PolyscopeRenderer::renderContacts(const std::vector<Contact*>& contacts) {
    static const char* strContacts = "contacts";
    
    if (!getShowContacts() || contacts.empty()) {
        polyscope::removePointCloud(strContacts);
        return;
    }
    
    // Create vectors for contact points and normals
    std::vector<glm::vec3> points;
    std::vector<glm::vec3> normals;
    points.reserve(contacts.size());
    normals.reserve(contacts.size());
    
    for (size_t i = 0; i < contacts.size(); ++i) {
        points.push_back(common::RenderUtils::eigenToGLM(contacts[i]->p));
        normals.push_back(common::RenderUtils::eigenToGLM(contacts[i]->n));
    }
    
    // Register or update the point cloud
    auto pointCloud = polyscope::registerPointCloud(strContacts, points);
    pointCloud->setPointColor({1.0f, 0.0f, 0.0f});
    pointCloud->setPointRadius(0.005);
    
    // Add normal vectors
    auto normalVectors = pointCloud->addVectorQuantity("normal", normals);
    normalVectors->setVectorColor({1.0f, 1.0f, 0.0f});
    normalVectors->setVectorLengthScale(0.05f);
    normalVectors->setEnabled(true);
}

void PolyscopeRenderer::renderJoints(const std::vector<Joint*>& joints) {
    static const char* strJointPoints = "jointsPoint";
    static const char* strJointCurve = "jointsCurve";
    
    if (!getShowJoints() || joints.empty()) {
        polyscope::removePointCloud(strJointPoints);
        polyscope::removeCurveNetwork(strJointCurve);
        return;
    }
    
    // Create joint points and edges
    std::vector<glm::vec3> points;
    std::vector<std::array<size_t, 2>> edges;
    
    for (size_t i = 0; i < joints.size(); ++i) {
        const Joint* joint = joints[i];
        
        // Calculate attachment points in world space
        Eigen::Vector3f p0 = joint->body0->q * joint->r0 + joint->body0->x;
        Eigen::Vector3f p1 = joint->body1->q * joint->r1 + joint->body1->x;
        
        // Add points
        points.emplace_back(common::RenderUtils::eigenToGLM(p0));
        points.emplace_back(common::RenderUtils::eigenToGLM(p1));
        
        // Add edge between them
        size_t baseIndex = 2 * i;
        edges.push_back({baseIndex, baseIndex + 1});
    }
    
    // Create point cloud for joint points
    auto pointCloud = polyscope::registerPointCloud(strJointPoints, points);
    pointCloud->setPointColor({0.0f, 0.0f, 1.0f});
    pointCloud->setPointRadius(0.005);
    
    // Create curve network for joint connections
    auto curves = polyscope::registerCurveNetwork(strJointCurve, points, edges);
    curves->setCurveColor({0.0f, 0.5f, 1.0f});
    curves->setCurveWidth(0.002f);
}

void PolyscopeRenderer::renderBoundingBoxes(const std::vector<RigidBody*>& bodies) {
    static const char* strBoundingBoxes = "boundingBoxes";
    
    if (!getShowBoundingBoxes() || bodies.empty()) {
        polyscope::removeCurveNetwork(strBoundingBoxes);
        return;
    }
    
    // Count non-empty bounding boxes
    size_t boxCount = 0;
    for (const auto* body : bodies) {
        if (!body->fixed) boxCount++;
    }
    
    if (boxCount == 0) {
        polyscope::removeCurveNetwork(strBoundingBoxes);
        return;
    }
    
    // Create points and edges for all boxes
    std::vector<glm::vec3> points;
    std::vector<std::array<size_t, 2>> edges;
    
    size_t pointIndex = 0;
    for (const auto* body : bodies) {
        if (body->fixed) continue;
        
        // Transform the bounding box to world space
        Eigen::Vector3f halfSize = body->getExtents() / 2.0f;
        Eigen::Vector3f center = body->x;
        Eigen::Matrix3f rotation = body->q.toRotationMatrix();
        
        // Define the 8 corners of the box
        std::vector<Eigen::Vector3f> corners = {
            center + rotation * Eigen::Vector3f(-halfSize.x(), -halfSize.y(), -halfSize.z()),
            center + rotation * Eigen::Vector3f(halfSize.x(), -halfSize.y(), -halfSize.z()),
            center + rotation * Eigen::Vector3f(halfSize.x(), halfSize.y(), -halfSize.z()),
            center + rotation * Eigen::Vector3f(-halfSize.x(), halfSize.y(), -halfSize.z()),
            center + rotation * Eigen::Vector3f(-halfSize.x(), -halfSize.y(), halfSize.z()),
            center + rotation * Eigen::Vector3f(halfSize.x(), -halfSize.y(), halfSize.z()),
            center + rotation * Eigen::Vector3f(halfSize.x(), halfSize.y(), halfSize.z()),
            center + rotation * Eigen::Vector3f(-halfSize.x(), halfSize.y(), halfSize.z())
        };
        
        // Add points
        for (const auto& corner : corners) {
            points.emplace_back(common::RenderUtils::eigenToGLM(corner));
        }
        
        // Add 12 edges forming the box
        edges.push_back({pointIndex + 0, pointIndex + 1});
        edges.push_back({pointIndex + 1, pointIndex + 2});
        edges.push_back({pointIndex + 2, pointIndex + 3});
        edges.push_back({pointIndex + 3, pointIndex + 0});
        
        edges.push_back({pointIndex + 4, pointIndex + 5});
        edges.push_back({pointIndex + 5, pointIndex + 6});
        edges.push_back({pointIndex + 6, pointIndex + 7});
        edges.push_back({pointIndex + 7, pointIndex + 4});
        
        edges.push_back({pointIndex + 0, pointIndex + 4});
        edges.push_back({pointIndex + 1, pointIndex + 5});
        edges.push_back({pointIndex + 2, pointIndex + 6});
        edges.push_back({pointIndex + 3, pointIndex + 7});
        
        pointIndex += 8;
    }
    
    // Create curve network for bounding boxes
    auto network = polyscope::registerCurveNetwork(strBoundingBoxes, points, edges);
    network->setCurveColor({0.8f, 0.8f, 0.2f});
    network->setCurveWidth(0.001f);
}

void PolyscopeRenderer::assignMaterial(RigidBody& body, const std::string& materialName) {
    auto it = bodyMeshes.find(&body);
    if (it != bodyMeshes.end() && it->second) {
        it->second->setMaterial(materialName);
        
        // Update color if needed
        if (body.color.norm() > 0) {
            glm::vec3 color = common::RenderUtils::eigenToGLM(body.color);
            it->second->setSurfaceColor(color);
        }
    }
}

} // namespace slrbs
