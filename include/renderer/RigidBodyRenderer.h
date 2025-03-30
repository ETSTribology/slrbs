#pragma once

#include <vector>
#include <string>
#include <glm/glm.hpp>

namespace slrbs {

class RigidBody;
class Contact;
class Joint;

/**
 * @class RigidBodyRenderer
 * @brief Abstract interface for rendering rigid bodies and physics elements
 */
class RigidBodyRenderer {
public:
    RigidBodyRenderer() 
        : m_showContacts(true), m_showJoints(true), m_showBoundingBoxes(false) {}
    virtual ~RigidBodyRenderer() = default;

    // Initialize the renderer
    virtual void initialize() = 0;
    
    // Update methods
    virtual void updateAll() = 0;
    virtual void updateBodyTransform(const RigidBody& body) = 0;
    
    // Render methods
    virtual void renderBodies(const std::vector<RigidBody*>& bodies) = 0;
    virtual void renderContacts(const std::vector<Contact*>& contacts) = 0;
    virtual void renderJoints(const std::vector<Joint*>& joints) = 0;
    virtual void renderBoundingBoxes(const std::vector<RigidBody*>& bodies) = 0;
    
    // Material assignment
    virtual void assignMaterial(RigidBody& body, const std::string& materialName) = 0;
    
    // Texture and color assignment
    virtual void setBodyColor(RigidBody& body, const glm::vec3& color) = 0;
    virtual void setBodyTexture(RigidBody& body, const std::string& texturePath) = 0;
    
    // Debug visualization toggles
    void setShowContacts(bool show) { m_showContacts = show; }
    void setShowJoints(bool show) { m_showJoints = show; }
    void setShowBoundingBoxes(bool show) { m_showBoundingBoxes = show; }
    
    bool getShowContacts() const { return m_showContacts; }
    bool getShowJoints() const { return m_showJoints; }
    bool getShowBoundingBoxes() const { return m_showBoundingBoxes; }

private:
    // Visualization state
    bool m_showContacts;
    bool m_showJoints;
    bool m_showBoundingBoxes;
};

} // namespace slrbs
