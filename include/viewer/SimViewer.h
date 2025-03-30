#pragma once

#include "viewer/ViewerConfig.h"
#include <memory>
#include <string>
#include <functional>

struct GLFWwindow;

namespace slrbs {

class ViewerUI;
class ViewerInteraction;
class ScenarioManager;
class SimulationManager;
class RigidBodySystem;
class RigidBodySystemState;
class RigidBodyRenderer;

/**
 * @class SimViewer
 * @brief Main viewer class for the rigid body simulation application.
 */
class SimViewer {
public:
    SimViewer();
    ~SimViewer();

    // Main functions
    void initialize();
    void run();
    void shutdown();
    
    // Simulation control
    void reset();
    void save();
    void loadScenario(int scenarioType);
    
    // Geometric stiffness damping
    void preStep(RigidBodySystem& system, float h);

    // Access to components
    ViewerConfig& getConfig() { return config; }
    const ViewerConfig& getConfig() const { return config; }
    ViewerInteraction& getInteraction() { return *interaction; }
    ScenarioManager& getScenarioManager() { return *scenarioManager; }
    SimulationManager& getSimulation() { return *simulationManager; }
    RigidBodySystem& getRigidBodySystem() { return *rigidBodySystem; }
    RigidBodyRenderer& getRenderer() { return *rigidBodyRenderer; }

    // GUI functionality
    void drawGUI();
    
    // Performance metrics
    void computePerformanceMetrics();

private:
    // Main rendering loop
    void mainLoop();
    void render();
    
    // Initialize components
    void setupPolyscope();
    void setupWindow();
    
    // Member variables
    ViewerConfig config;
    GLFWwindow* window;
    
    // Components
    std::unique_ptr<ViewerUI> ui;
    std::unique_ptr<ViewerInteraction> interaction;
    std::unique_ptr<ScenarioManager> scenarioManager;
    std::unique_ptr<SimulationManager> simulationManager;
    std::unique_ptr<RigidBodySystem> rigidBodySystem;
    std::unique_ptr<RigidBodyRenderer> rigidBodyRenderer;
    
    // Simulation state backup for resets
    std::unique_ptr<RigidBodySystemState> resetState;
    
    // Simulation parameters
    bool adaptiveTimesteps = false;
    bool geometricStiffnessDamping = false;
    float alpha = 0.01f;
    float timeStep = 0.01667f;  // Default to ~60fps
    int substeps = 1;
    bool paused = true;
    bool stepOnce = false;
    bool enableCollisions = true;
    bool enableScreenshots = false;
    bool enableLogging = false;
    
    // Performance metrics
    float dynamicsTime = 0.0f;
    int frameCounter = 0;
    float kineticEnergy = 0.0f;
    float constraintError = 0.0f;
};

} // namespace slrbs
