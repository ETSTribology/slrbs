#pragma once

#include <memory>
#include <Eigen/Core>

struct GLFWwindow;

namespace slrbs {

class SimViewer;
class RigidBody;

/**
 * @class ViewerInteraction
 * @brief Handles user interaction with the simulation.
 */
class ViewerInteraction {
public:
    ViewerInteraction(SimViewer* viewer);
    ~ViewerInteraction();

    void initialize(GLFWwindow* window);
    void update();
    
    // Camera controls
    void updateCamera();
    void resetCamera();
    
    // Mouse interaction
    void processMouseMovement(double xpos, double ypos);
    void processMouseButton(int button, int action, int mods);
    void processMouseScroll(double xoffset, double yoffset);
    
    // Keyboard interaction
    void processKeyboard(int key, int scancode, int action, int mods);
    
    // Object selection
    RigidBody* getSelectedObject() const { return selectedObject; }
    void selectObject(RigidBody* object);
    void clearSelection();

private:
    // Private methods
    void updatePickingRay();
    
    // Member variables
    SimViewer* viewer;
    GLFWwindow* window;
    
    // Mouse state
    double lastMouseX;
    double lastMouseY;
    bool firstMouse;
    bool mousePressed;
    int activeButton;
    
    // Camera state
    float cameraPitch;
    float cameraYaw;
    float cameraDistance;
    Eigen::Vector3f cameraPosition;
    Eigen::Vector3f cameraFront;
    Eigen::Vector3f cameraUp;
    
    // Selection state
    RigidBody* selectedObject;
};

} // namespace slrbs
