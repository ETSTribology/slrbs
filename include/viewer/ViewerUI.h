#pragma once

#include <memory>

struct GLFWwindow;

namespace slrbs {

class SimViewer;
class RigidBodySystem;

/**
 * @class ViewerUI
 * @brief Manages the UI elements of the simulation viewer.
 */
class ViewerUI {
public:
    ViewerUI(SimViewer* viewer);
    ~ViewerUI();

    void initialize(GLFWwindow* window);
    void render(const RigidBodySystem& system);
    void shutdown();

private:
    // UI panels
    void renderMainMenuBar();
    void renderSimulationControls();
    void renderObjectProperties();
    void renderPerformanceMetrics();
    
    // Helper functions
    void updateFPS();

    // Member variables
    SimViewer* viewer;
    GLFWwindow* window;
    
    // Timing data
    float lastFrameTime;
    float deltaTime;
    float fps;
    
    // UI visibility flags
    bool showMainMenu = true;
    bool showSimulationControls = true;
    bool showObjectProperties = true;
    bool showPerformanceMetrics = true;
};

} // namespace slrbs
