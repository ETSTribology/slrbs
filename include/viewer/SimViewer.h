#pragma once

/**
 * @file SimViewer.h
 *
 * @brief Viewer for a cloth simulation application.
 *
 */

#include "util/Types.h"
#include <memory>
#include <vector>
#include <string>

namespace polyscope
{
    class SurfaceMesh;
    class PointCloud;
}

class Contact;
class RigidBodySystem;
class RigidBody;
class RigidBodyState;
class RigidBodySystemState;


class SimViewer 
{
public:
    SimViewer();
    virtual ~SimViewer();

    void start();
    void reset();
    void save();

private:
    void createMarbleBox();
    void createSphereOnBox();
    void createSwingingBox();
    void createCylinderOnPlane();
    void createCarScene();
    
    // Method to load a JSON scenario
    void loadScenarioFromJSON(const std::string& filename);
    
    // Method to refresh the available scenarios list
    void refreshScenariosList();

    void draw();
    void drawGUI();
    void drawScenarioSelectionGUI();

    void preStep(std::vector<RigidBody*>&);

private:

    // Simulation parameters
    float m_dt;                         // Time step parameter.
    int m_subSteps;
    bool m_paused;                      // Pause the simulation.
    bool m_stepOnce;                    // Advance the simulation by one frame and then stop.
    bool m_enableCollisions;            // enable/disable collisions
    bool m_enableScreenshots;           // enable/disable saving screenshots
    bool m_drawContacts;                // enable drawing contacts
    bool m_drawConstraints;             // enable constraint viz
    float m_dynamicsTime;               // Compute time for the dynamics step (in ms)
    std::unique_ptr<RigidBodySystemState> m_resetState;
    
    // Available JSON scenarios
    std::vector<std::string> m_availableScenarios;
    int m_selectedScenario;
};