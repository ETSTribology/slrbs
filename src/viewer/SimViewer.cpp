#include "viewer/SimViewer.h"

#include "polyscope/polyscope.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/view.h"
#include "imgui.h"

#include <chrono>
#include <iostream>
#include <functional>

#include "contact/Contact.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBodyState.h"
#include "rigidbody/Scenarios.h"
#include "util/ScenarioLoader.h"

using namespace std;

// Static variables and helper functions for visualization and UI
namespace {
    static RigidBodySystem* m_rigidBodySystem = new RigidBodySystem;
    
    static const char* strContacts = "contacts";
    static const char* strJointPoints = "jointsPoint";
    static const char* strJointCurve = "jointsCurve";

    // Update the visual representation of rigid bodies
    static void updateRigidBodyMeshes(RigidBodySystem& _rigidBodySystem) {
        auto& bodies = _rigidBodySystem.getBodies();
        for(unsigned int k = 0; k < bodies.size(); ++k) { 
            if (!bodies[k]->mesh) continue;

            Eigen::Isometry3f tm = Eigen::Isometry3f::Identity();
        
            // Copy rotation part
            tm.linear() = bodies[k]->q.toRotationMatrix();

            // Copy translation part
            tm.translation() = bodies[k]->x;

            bodies[k]->mesh->setTransform(glm::make_mat4x4(tm.data()));
            
            // Apply visual properties if they exist
            if (bodies[k]->visualProperties.size() > 0) {
                // Color
                if (bodies[k]->visualProperties.count("color_r") > 0) {
                    bodies[k]->mesh->setSurfaceColor({
                        bodies[k]->visualProperties["color_r"],
                        bodies[k]->visualProperties["color_g"],
                        bodies[k]->visualProperties["color_b"]
                    });
                }
                
                // Transparency
                if (bodies[k]->visualProperties.count("transparency") > 0) {
                    bodies[k]->mesh->setTransparency(bodies[k]->visualProperties["transparency"]);
                }
                
                // Smooth shading
                if (bodies[k]->visualProperties.count("smooth_shade") > 0) {
                    bodies[k]->mesh->setSmoothShade(bodies[k]->visualProperties["smooth_shade"] > 0.5f);
                }
                
                // Edge width
                if (bodies[k]->visualProperties.count("edge_width") > 0) {
                    bodies[k]->mesh->setEdgeWidth(bodies[k]->visualProperties["edge_width"]);
                }
            }
        }
    }

    // Update the visual representation of contact points
    static void updateContactPoints(RigidBodySystem& _rigidBodySystem) {
        const auto& contacts = _rigidBodySystem.getContacts();
        const unsigned int numContacts = contacts.size();

        if (numContacts == 0) {
            polyscope::removePointCloud("contacts");
        } else {
            Eigen::MatrixXf contactP(numContacts, 3);
            Eigen::MatrixXf contactN(numContacts, 3);

            for (unsigned int i = 0; i < numContacts; ++i) {
                contactP.row(i)(0) = contacts[i]->p(0); 
                contactP.row(i)(1) = contacts[i]->p(1); 
                contactP.row(i)(2) = contacts[i]->p(2);
                contactN.row(i)(0) = contacts[i]->n(0); 
                contactN.row(i)(1) = contacts[i]->n(1); 
                contactN.row(i)(2) = contacts[i]->n(2);
            }

            auto pointCloud = polyscope::registerPointCloud("contacts", contactP);

            pointCloud->setPointColor({ 1.0f, 0.0f, 0.0f });
            pointCloud->setPointRadius(0.005);
            pointCloud->addVectorQuantity("normal", contactN)
                ->setVectorColor({ 1.0f, 1.0f, 0.0f })
                ->setVectorLengthScale(0.05f)
                ->setEnabled(true);
        }
    }

    // Update the visual representation of joints
    static void updateJointViz(RigidBodySystem& _rigidBodySystem) {
        const auto& joints = _rigidBodySystem.getJoints();
        const unsigned int numJoints = joints.size();

        if (numJoints == 0) {
            polyscope::removePointCloud("jointsPoint");
            polyscope::removeCurveNetwork("jointsCurve");
        } else {
            Eigen::MatrixXf jointP(2 * numJoints, 3);
            Eigen::MatrixXi jointE(numJoints, 2);
            
            for (unsigned int i = 0; i < numJoints; ++i) {
                const Eigen::Vector3f p0 = joints[i]->body0->q * joints[i]->r0 + joints[i]->body0->x;
                const Eigen::Vector3f p1 = joints[i]->body1->q * joints[i]->r1 + joints[i]->body1->x;

                jointP.row(2 * i) = p0;
                jointP.row(2 * i + 1) = p1;
                jointE.row(i) = Eigen::Vector2i(2 * i, 2 * i + 1);
            }

            auto pointCloud = polyscope::registerPointCloud("jointsPoint", jointP);
            pointCloud->setPointColor({ 0.0f, 0.0f, 1.0f });
            pointCloud->setPointRadius(0.005);
            
            auto curves = polyscope::registerCurveNetwork("jointsCurve", jointP, jointE);
            curves->setRadius(0.002f);
        }
    }
}

SimViewer::SimViewer() :
    m_dt(0.01f), m_subSteps(1), m_dynamicsTime(0.0f),
    m_paused(true), m_stepOnce(false),
    m_enableCollisions(true), m_enableScreenshots(false),
    m_drawContacts(true), m_drawConstraints(true),
    m_resetState(), m_selectedScenario(-1)
{
    m_resetState = std::make_unique<RigidBodySystemState>(*m_rigidBodySystem);
    refreshScenariosList();
    reset();
}

SimViewer::~SimViewer()
{
}

void SimViewer::reset()
{
    std::cout << " ---- Reset ----- " << std::endl;
    m_resetState->restore(*m_rigidBodySystem);
    m_dynamicsTime = 0.0f;

    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::save()
{
    std::cout << " ---- Saving current state ----- " << std::endl;
    m_resetState->save(*m_rigidBodySystem);
}

void SimViewer::start()
{
    // Setup Polyscope
    polyscope::options::programName = "slrbs";
    polyscope::options::verbosity = 0;
    polyscope::options::usePrefsFile = false;
    polyscope::options::alwaysRedraw = true;
    polyscope::options::ssaaFactor = 2;
    polyscope::options::openImGuiWindowForUserCallback = true;
    polyscope::options::groundPlaneHeightFactor = 0.0f; // adjust the plane height
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::TileReflection;
    polyscope::options::buildGui = false;
    polyscope::options::maxFPS = -1;
    polyscope::options::groundPlaneEnabled = true;
    polyscope::options::screenshotExtension = ".png";

    // Initialize
    polyscope::init();

    // Setup a viewing volume
    polyscope::options::automaticallyComputeSceneExtents = false;
    polyscope::state::lengthScale = 10.0f;
    polyscope::state::boundingBox = std::tuple<glm::vec3, glm::vec3>{ {-5., 0, -5.}, {5., 5., 5.} };

    // Specify the update callback
    polyscope::state::userCallback = std::bind(&SimViewer::draw, this);

    // Add pre-step hook
    m_rigidBodySystem->setPreStepFunc(std::bind(&SimViewer::preStep, this, std::placeholders::_1));

    // Show the window
    polyscope::show();
}

void SimViewer::refreshScenariosList()
{
    // Use ScenarioLoader to get available JSON scenarios
    m_availableScenarios = ScenarioLoader::listAvailableScenarios();
    
    if (m_availableScenarios.empty()) {
        std::cout << "No JSON scenario files found." << std::endl;
    } else {
        std::cout << "Found " << m_availableScenarios.size() << " JSON scenario files." << std::endl;
    }
    
    // Reset selection to none
    m_selectedScenario = -1;
}

void SimViewer::loadScenarioFromJSON(const std::string& filename)
{
    // Clear visualization structures first
    polyscope::removeAllStructures();
    
    // Use ScenarioLoader to load the scenario
    if (ScenarioLoader::loadFromFile(*m_rigidBodySystem, filename)) {
        std::cout << "Successfully loaded scenario from " << filename << std::endl;
        
        // Apply visual properties and update meshes
        updateRigidBodyMeshes(*m_rigidBodySystem);
        
        // Save state and reset view
        m_resetState->save(*m_rigidBodySystem);
        polyscope::resetScreenshotIndex();
    } else {
        std::cerr << "Failed to load scenario from " << filename << std::endl;
    }
}

void SimViewer::drawScenarioSelectionGUI()
{
    if (ImGui::CollapsingHeader("JSON Scenarios", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Refresh Scenarios List")) {
            refreshScenariosList();
        }
        
        // Display scenario list
        if (!m_availableScenarios.empty()) {
            ImGui::Text("Available Scenarios:");
            ImGui::Indent();
            
            for (int i = 0; i < m_availableScenarios.size(); i++) {
                if (ImGui::Selectable(m_availableScenarios[i].c_str(), m_selectedScenario == i)) {
                    m_selectedScenario = i;
                }
            }
            
            ImGui::Unindent();
            
            // Load button
            if (m_selectedScenario >= 0 && m_selectedScenario < m_availableScenarios.size()) {
                if (ImGui::Button("Load Selected Scenario")) {
                    loadScenarioFromJSON(m_availableScenarios[m_selectedScenario]);
                }
            }
        } else {
            ImGui::Text("No JSON scenario files found.");
        }
    }
}

void SimViewer::drawGUI()
{
    ImGui::Text("Simulation:");
    ImGui::Checkbox("Pause", &m_paused);
    if (ImGui::Button("Step once")) {
        m_stepOnce = true;
    }
    if (ImGui::Button("Reset")) {
        reset();
    }
    if (ImGui::Button("Save")) {
        save();
    }

    ImGui::PushItemWidth(100);
    ImGui::SliderFloat("Time step", &m_dt, 0.0f, 0.1f, "%.3f");
    ImGui::SliderInt("Num. sub-steps", &m_subSteps, 1, 20, "%u");
    ImGui::SliderInt("Solver iters.", &(m_rigidBodySystem->solverIter), 1, 100, "%u");
    ImGui::SliderFloat("Friction coeff.", &(Contact::mu), 0.0f, 2.0f, "%.2f");
    ImGui::RadioButton("PGS", &(m_rigidBodySystem->solverId), 0);  ImGui::SameLine();
    ImGui::RadioButton("Conj. Gradient (NO CONTACT)", &(m_rigidBodySystem->solverId), 1);
    ImGui::RadioButton("Conj. Residual (NO CONTACT)", &(m_rigidBodySystem->solverId), 2);
    ImGui::PopItemWidth();

    if (ImGui::Checkbox("Enable collision detection", &m_enableCollisions)) {
        m_rigidBodySystem->setEnableCollisionDetection(m_enableCollisions);
    }

    ImGui::Checkbox("Draw contacts", &m_drawContacts);
    ImGui::Checkbox("Draw constraints", &m_drawConstraints);
    ImGui::Checkbox("Enable screenshots", &m_enableScreenshots);

    // Built-in scenarios
    if (ImGui::CollapsingHeader("Built-in Scenarios", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::Button("Sphere on box")) {
            createSphereOnBox();
        }
        if (ImGui::Button("Marble box")) {
            createMarbleBox();
        }
        if (ImGui::Button("Swinging box")) {
            createSwingingBox();
        }
        if (ImGui::Button("Cylinder on plane")) {
            createCylinderOnPlane();
        }
        if (ImGui::Button("Create car scene")) {
            createCarScene();
        }
    }
    
    // JSON scenario selection UI
    drawScenarioSelectionGUI();

    ImGui::Text("Step time: %3.3f ms", m_dynamicsTime);
}

void SimViewer::draw()
{
    drawGUI();

    if (!m_paused || m_stepOnce) {
        auto start = std::chrono::high_resolution_clock::now();

        // Step the simulation.
        // The time step dt is divided by the number of sub-steps.
        const float dt = m_dt / (float)m_subSteps;
        for (int i = 0; i < m_subSteps; ++i) {
            m_rigidBodySystem->step(dt);
        }
        auto stop = std::chrono::high_resolution_clock::now();

        // Update visualization
        updateRigidBodyMeshes(*m_rigidBodySystem);

        if (m_drawContacts)
            updateContactPoints(*m_rigidBodySystem);
        else
            polyscope::removePointCloud(strContacts);

        if (m_drawConstraints)
            updateJointViz(*m_rigidBodySystem);
        else {
            polyscope::removePointCloud(strJointPoints);
            polyscope::removeCurveNetwork(strJointCurve);
        }

        // Calculate step time
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        m_dynamicsTime = (float)duration.count() / 1000.0f;

        if (m_enableScreenshots) {
            polyscope::screenshot(false);
        }

        // Clear step-once flag
        m_stepOnce = false;
    }
}

void SimViewer::createMarbleBox()
{
    polyscope::removeAllStructures(); // Clear visualizations
    Scenarios::createMarbleBox(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createSphereOnBox()
{
    polyscope::removeAllStructures(); // Clear visualizations
    Scenarios::createSphereOnBox(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createSwingingBox()
{
    polyscope::removeAllStructures(); // Clear visualizations
    Scenarios::createSwingingBoxes(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCylinderOnPlane()
{
    polyscope::removeAllStructures(); // Clear visualizations
    Scenarios::createCylinderOnPlane(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCarScene()
{
    polyscope::removeAllStructures(); // Clear visualizations
    Scenarios::createCarScene(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::preStep(std::vector<RigidBody*>& _bodies)
{
    // Hook for pre-step operations (currently empty)
    // Can be used for custom manipulations before physics step
    // Silencing warning about unused parameter
    (void)_bodies;
}