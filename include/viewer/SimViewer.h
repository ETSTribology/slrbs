#pragma once

#include "util/Types.h"
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <array>
#include <chrono>

namespace polyscope {
    class SurfaceMesh;
    class PointCloud;
    class CameraParameters;
}

class Contact;
class RigidBodySystem;
class RigidBody;
class RigidBodySystemState;
class Joint;

class SimViewer {
public:
    SimViewer();
    virtual ~SimViewer();

    void start();
    void reset();
    void save();

private:
    // Built-in scenario creation methods
    void createMarbleBox();
    void createSphereOnBox();
    void createSwingingBox();
    void createCylinderOnPlane();
    void createCarScene();

    // JSON scenario handling methods
    void loadScenarioFromJSON(const std::string& filename);
    void refreshScenariosList();
    void drawScenarioSelectionGUI();

    // Main rendering and GUI methods
    void draw();
    void drawGUI();
    void drawColorDebugUI();
    void preStep(std::vector<RigidBody*>& bodies);

private:
    // Simulation parameters
    float m_dt;                         // Time step parameter
    int m_subSteps;                     // Number of substeps per frame
    bool m_paused;                      // Pause the simulation
    bool m_stepOnce;                    // Advance the simulation by one frame and then stop
    bool m_enableCollisions;            // Enable/disable collisions
    bool m_enableScreenshots;           // Enable/disable saving screenshots
    bool m_drawContacts;                // Enable drawing contacts
    bool m_drawConstraints;             // Enable constraint visualization
    float m_dynamicsTime;               // Compute time for the dynamics step (in ms)
    std::unique_ptr<RigidBodySystemState> m_resetState;

    // UI state
    int m_selectedScenario;             // Currently selected scenario in the UI
    int m_selectedBodyIndex = -1;       // Currently selected body in the hierarchy view
    std::vector<std::string> m_availableScenarios; // List of available JSON scenarios
};