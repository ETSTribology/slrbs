#include "viewer/ViewerInteraction.h"
#include "viewer/SimViewer.h"
#include "rigidbody/RigidBody.h"

#include <GLFW/glfw3.h>
#include <polyscope/polyscope.h>
#include <polyscope/view.h>

#include <iostream>

namespace slrbs {

// Static callback functions for GLFW
static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    ViewerInteraction* interaction = static_cast<ViewerInteraction*>(glfwGetWindowUserPointer(window));
    if (interaction) {
        interaction->processMouseButton(button, action, mods);
    }
}

static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    ViewerInteraction* interaction = static_cast<ViewerInteraction*>(glfwGetWindowUserPointer(window));
    if (interaction) {
        interaction->processMouseMovement(xpos, ypos);
    }
}

static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    ViewerInteraction* interaction = static_cast<ViewerInteraction*>(glfwGetWindowUserPointer(window));
    if (interaction) {
        interaction->processMouseScroll(xoffset, yoffset);
    }
}

static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    ViewerInteraction* interaction = static_cast<ViewerInteraction*>(glfwGetWindowUserPointer(window));
    if (interaction) {
        interaction->processKeyboard(key, scancode, action, mods);
    }
}

ViewerInteraction::ViewerInteraction(SimViewer* viewer)
    : viewer(viewer), window(nullptr), lastMouseX(0), lastMouseY(0),
      firstMouse(true), mousePressed(false), activeButton(-1), 
      cameraPitch(0), cameraYaw(0), cameraDistance(10),
      selectedObject(nullptr) {
}

ViewerInteraction::~ViewerInteraction() {
}

void ViewerInteraction::initialize(GLFWwindow* window) {
    this->window = window;
    
    // Store the previous user pointer
    void* prevUserPtr = glfwGetWindowUserPointer(window);
    
    // Set this instance as the user pointer for callbacks
    glfwSetWindowUserPointer(window, this);
    
    // Set callbacks
    glfwSetMouseButtonCallback(window, mouseButtonCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetScrollCallback(window, scrollCallback);
    glfwSetKeyCallback(window, keyCallback);
    
    // Initialize camera from config
    const CameraConfig& camConfig = viewer->getConfig().camera;
    cameraPitch = camConfig.orbitPitch;
    cameraYaw = camConfig.orbitYaw;
    cameraDistance = camConfig.orbitRadius;
    
    // Apply initial camera settings to polyscope
    updateCamera();
    
    // Restore the previous user pointer
    glfwSetWindowUserPointer(window, prevUserPtr);
}

void ViewerInteraction::update() {
    // This method is called each frame to update any continuous interactions
    
    // Handle keyboard input that should be continuous
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        // Move camera forward
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        // Move camera backward
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        // Move camera left
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        // Move camera right
    }
}

void ViewerInteraction::updateCamera() {
    // Save current values back to config
    viewer->getConfig().camera.orbitPitch = cameraPitch;
    viewer->getConfig().camera.orbitYaw = cameraYaw;
    viewer->getConfig().camera.orbitRadius = cameraDistance;
    
    // Calculate camera position from spherical coordinates
    float pitch = glm::radians(cameraPitch);
    float yaw = glm::radians(cameraYaw);
    
    // Convert spherical to Cartesian coordinates
    cameraPosition.x() = cameraDistance * cos(pitch) * cos(yaw);
    cameraPosition.y() = cameraDistance * sin(pitch);
    cameraPosition.z() = cameraDistance * cos(pitch) * sin(yaw);
    
    // Apply to polyscope
    glm::vec3 eye(cameraPosition.x(), cameraPosition.y(), cameraPosition.z());
    glm::vec3 target(0.0f, 0.0f, 0.0f); // Look at the origin
    glm::vec3 up(0.0f, 1.0f, 0.0f);     // Y-up
    
    polyscope::view::lookAt(eye, target, up);
}

void ViewerInteraction::resetCamera() {
    // Reset to default values from config
    const CameraConfig& defaultCam = viewer->getConfig().camera;
    cameraPitch = defaultCam.orbitPitch;
    cameraYaw = defaultCam.orbitYaw;
    cameraDistance = defaultCam.orbitRadius;
    
    updateCamera();
}

void ViewerInteraction::processMouseMovement(double xpos, double ypos) {
    if (firstMouse) {
        lastMouseX = xpos;
        lastMouseY = ypos;
        firstMouse = false;
        return;
    }
    
    // Calculate offsets
    double xoffset = xpos - lastMouseX;
    double yoffset = lastMouseY - ypos; // Reversed: y ranges bottom to top
    
    lastMouseX = xpos;
    lastMouseY = ypos;
    
    // Check if ImGui is using the mouse
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
        return;
    }
    
    // Only rotate if middle or right mouse button is pressed
    if (mousePressed && (activeButton == GLFW_MOUSE_BUTTON_RIGHT || activeButton == GLFW_MOUSE_BUTTON_MIDDLE)) {
        const float sensitivity = 0.1f;
        
        // Apply sensitivity
        xoffset *= sensitivity;
        yoffset *= sensitivity;
        
        // Update camera angles
        cameraYaw += static_cast<float>(xoffset);
        cameraPitch += static_cast<float>(yoffset);
        
        // Constrain pitch
        if (cameraPitch > 89.0f) cameraPitch = 89.0f;
        if (cameraPitch < -89.0f) cameraPitch = -89.0f;
        
        // Update the camera
        updateCamera();
    }
    
    // Update picking ray for selection
    updatePickingRay();
}

void ViewerInteraction::processMouseButton(int button, int action, int mods) {
    // Check if ImGui is using the mouse
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
        return;
    }
    
    if (action == GLFW_PRESS) {
        mousePressed = true;
        activeButton = button;
        
        // Handle left click for object selection
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            // Update the picking ray
            updatePickingRay();
            
            // TODO: Perform ray cast against rigid bodies to select object
            // For now, just clear the selection
            clearSelection();
        }
    } else if (action == GLFW_RELEASE) {
        if (button == activeButton) {
            mousePressed = false;
            activeButton = -1;
        }
    }
}

void ViewerInteraction::processMouseScroll(double xoffset, double yoffset) {
    // Check if ImGui is using the mouse
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
        return;
    }
    
    // Zoom with mouse wheel
    const float zoomSpeed = 0.5f;
    cameraDistance -= static_cast<float>(yoffset) * zoomSpeed;
    
    // Enforce minimum and maximum zoom levels
    if (cameraDistance < 1.0f) cameraDistance = 1.0f;
    if (cameraDistance > 100.0f) cameraDistance = 100.0f;
    
    // Update the camera
    updateCamera();
}

void ViewerInteraction::processKeyboard(int key, int scancode, int action, int mods) {
    // Check if ImGui is using the keyboard
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureKeyboard) {
        return;
    }
    
    // Handle keyboard input for simulation control
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        switch (key) {
            case GLFW_KEY_ESCAPE:
                glfwSetWindowShouldClose(window, GLFW_TRUE);
                break;
            case GLFW_KEY_SPACE:
                viewer->getSimulation().togglePause();
                break;
            case GLFW_KEY_R:
                if (mods & GLFW_MOD_CONTROL) {
                    resetCamera();
                } else {
                    viewer->getSimulation().reset();
                }
                break;
            default:
                break;
        }
    }
}

void ViewerInteraction::updatePickingRay() {
    if (!window) return;
    
    // Get screen coordinates of mouse
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    
    // Get window size
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    
    // Convert to normalized device coordinates (-1 to 1)
    float x = (2.0f * xpos) / width - 1.0f;
    float y = 1.0f - (2.0f * ypos) / height;
    
    // Get view and projection matrices
    glm::mat4 proj = polyscope::view::getCameraPerspectiveMatrix();
    glm::mat4 view = polyscope::view::getCameraViewMatrix();
    
    // Create combined inverse matrix
    glm::mat4 invMat = glm::inverse(proj * view);
    
    // Create points on near and far planes in NDC
    glm::vec4 nearPoint(x, y, -1.0f, 1.0f);
    glm::vec4 farPoint(x, y, 1.0f, 1.0f);
    
    // Transform to world space
    glm::vec4 nearWorld = invMat * nearPoint;
    nearWorld /= nearWorld.w;
    
    glm::vec4 farWorld = invMat * farPoint;
    farWorld /= farWorld.w;
    
    // Create ray
    glm::vec3 rayOrigin(nearWorld);
    glm::vec3 rayDirection = glm::normalize(glm::vec3(farWorld) - glm::vec3(nearWorld));
    
    // Now we can use rayOrigin and rayDirection to perform object selection
    // by checking intersections with rigid bodies
    
    // Find closest intersection
    float closestDist = std::numeric_limits<float>::max();
    RigidBody* closestBody = nullptr;
    
    for (auto* body : viewer->getRigidBodySystem().getBodies()) {
        // Simple bounding sphere test for demonstration
        // A real implementation would use proper collision detection
        float radius = body->boundingRadius;
        glm::vec3 center(body->x.x(), body->x.y(), body->x.z());
        
        // Ray-sphere intersection test
        glm::vec3 oc = rayOrigin - center;
        float a = glm::dot(rayDirection, rayDirection);
        float b = 2.0f * glm::dot(oc, rayDirection);
        float c = glm::dot(oc, oc) - radius * radius;
        float discriminant = b*b - 4*a*c;
        
        if (discriminant >= 0) {
            float dist = (-b - sqrt(discriminant)) / (2.0f * a);
            if (dist > 0 && dist < closestDist) {
                closestDist = dist;
                closestBody = body;
            }
        }
    }
    
    // If we found an intersection, select the object
    if (closestBody) {
        selectObject(closestBody);
    } else {
        clearSelection();
    }
}

void ViewerInteraction::selectObject(RigidBody* object) {
    selectedObject = object;
    std::cout << "Selected object: " << (object ? object->name : "none") << std::endl;
}

void ViewerInteraction::clearSelection() {
    selectedObject = nullptr;
}

} // namespace slrbs
