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
#include "util/VisualProperties.h"

using namespace std;

// Static variables and helper functions for visualization and UI
namespace
{
    static RigidBodySystem *m_rigidBodySystem = new RigidBodySystem;

    static const char *strContacts = "contacts";
    static const char *strJointPoints = "jointsPoint";
    static const char *strJointCurve = "jointsCurve";

    static void updateRigidBodyMeshes(RigidBodySystem &_rigidBodySystem)
    {
        auto &bodies = _rigidBodySystem.getBodies();
        for (unsigned int k = 0; k < bodies.size(); ++k)
        {
            if (!bodies[k]->mesh)
                continue;

            Eigen::Isometry3f tm = Eigen::Isometry3f::Identity();

            // Copy rotation part
            tm.linear() = bodies[k]->q.toRotationMatrix();

            // Copy translation part
            tm.translation() = bodies[k]->x;

            bodies[k]->mesh->setTransform(glm::make_mat4x4(tm.data()));

            // First, try to apply body-specific visual properties
            bodies[k]->applyVisualProperties();

            // Then, check for global visual properties as fallback
            bool foundGlobalProps = false;
            for (const auto &prop : g_visualProperties.bodyProperties)
            {
                if (prop.bodyIndex == k)
                {
                    bodies[k]->mesh->setSurfaceColor({prop.colorR, prop.colorG, prop.colorB});
                    bodies[k]->mesh->setTransparency(prop.transparency);
                    bodies[k]->mesh->setSmoothShade(prop.smoothShade);
                    bodies[k]->mesh->setEdgeWidth(prop.edgeWidth);
                    foundGlobalProps = true;
                    break;
                }
            }

            // If no visual properties were found, use default colors
            if (!foundGlobalProps && bodies[k]->visualProperties.empty())
            {
                // Apply default colors based on body type
                if (bodies[k]->fixed)
                {
                    // Ground/fixed bodies
                    bodies[k]->mesh->setSurfaceColor({0.5f, 0.5f, 0.5f});
                    bodies[k]->mesh->setTransparency(1.0f); // Set transparency to 1.0 (fully opaque)
                    bodies[k]->mesh->setSmoothShade(false);
                }
                else
                {
                    // Dynamic bodies - use vibrant colors based on index for variety
                    float hue = fmodf(k * 0.618033988749895f, 1.0f); // Golden ratio creates good distribution

                    // Simple HSV to RGB conversion for variety
                    float h = hue * 6.0f;
                    float c = 0.9f; // Chroma
                    float x = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));

                    float r = 0.0f, g = 0.0f, b = 0.0f;
                    if (h < 1.0f)
                    {
                        r = c;
                        g = x;
                    }
                    else if (h < 2.0f)
                    {
                        r = x;
                        g = c;
                    }
                    else if (h < 3.0f)
                    {
                        g = c;
                        b = x;
                    }
                    else if (h < 4.0f)
                    {
                        g = x;
                        b = c;
                    }
                    else if (h < 5.0f)
                    {
                        r = x;
                        b = c;
                    }
                    else
                    {
                        r = c;
                        b = x;
                    }

                    bodies[k]->mesh->setSurfaceColor({r, g, b});
                    bodies[k]->mesh->setTransparency(1.0f); // Set transparency to 1.0 (fully opaque)
                    bodies[k]->mesh->setSmoothShade(true);
                }
            }
        }
    }

    static void updateContactPoints(RigidBodySystem &_rigidBodySystem)
    {
        const auto &contacts = _rigidBodySystem.getContacts();
        const unsigned int numContacts = contacts.size();

        if (numContacts == 0)
        {
            polyscope::removePointCloud("contacts");
        }
        else
        {
            Eigen::MatrixXf contactP(numContacts, 3);
            Eigen::MatrixXf contactN(numContacts, 3);

            for (unsigned int i = 0; i < numContacts; ++i)
            {
                contactP.row(i)(0) = contacts[i]->p(0);
                contactP.row(i)(1) = contacts[i]->p(1);
                contactP.row(i)(2) = contacts[i]->p(2);
                contactN.row(i)(0) = contacts[i]->n(0);
                contactN.row(i)(1) = contacts[i]->n(1);
                contactN.row(i)(2) = contacts[i]->n(2);
            }

            auto pointCloud = polyscope::registerPointCloud("contacts", contactP);

            pointCloud->setPointColor({1.0f, 0.0f, 0.0f});
            pointCloud->setPointRadius(0.005);
            pointCloud->addVectorQuantity("normal", contactN)
                ->setVectorColor({1.0f, 1.0f, 0.0f})
                ->setVectorLengthScale(0.05f)
                ->setEnabled(true);
        }
    }

    // Update the visual representation of joints
    static void updateJointViz(RigidBodySystem &_rigidBodySystem)
    {
        const auto &joints = _rigidBodySystem.getJoints();
        const unsigned int numJoints = joints.size();

        if (numJoints == 0)
        {
            polyscope::removePointCloud("jointsPoint");
            polyscope::removeCurveNetwork("jointsCurve");
        }
        else
        {
            Eigen::MatrixXf jointP(2 * numJoints, 3);
            Eigen::MatrixXi jointE(numJoints, 2);

            for (unsigned int i = 0; i < numJoints; ++i)
            {
                const Eigen::Vector3f p0 = joints[i]->body0->q * joints[i]->r0 + joints[i]->body0->x;
                const Eigen::Vector3f p1 = joints[i]->body1->q * joints[i]->r1 + joints[i]->body1->x;

                jointP.row(2 * i) = p0;
                jointP.row(2 * i + 1) = p1;
                jointE.row(i) = Eigen::Vector2i(2 * i, 2 * i + 1);
            }

            auto pointCloud = polyscope::registerPointCloud("jointsPoint", jointP);
            pointCloud->setPointColor({0.0f, 0.0f, 1.0f});
            pointCloud->setPointRadius(0.005);

            auto curves = polyscope::registerCurveNetwork("jointsCurve", jointP, jointE);
            curves->setRadius(0.002f);
        }
    }
}

SimViewer::SimViewer() : m_dt(0.01f), m_subSteps(1), m_dynamicsTime(0.0f),
                         m_paused(true), m_stepOnce(false),
                         m_enableCollisions(true), m_enableScreenshots(false),
                         m_drawContacts(true), m_drawConstraints(true),
                         m_resetState(), m_selectedScenario(-1), m_selectedBodyIndex(-1)
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
    polyscope::state::boundingBox = std::tuple<glm::vec3, glm::vec3>{{-5., 0, -5.}, {5., 5., 5.}};

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

    if (m_availableScenarios.empty())
    {
        std::cout << "No JSON scenario files found." << std::endl;
    }
    else
    {
        std::cout << "Found " << m_availableScenarios.size() << " JSON scenario files." << std::endl;
    }

    // Reset selection to none
    m_selectedScenario = -1;
}

void SimViewer::loadScenarioFromJSON(const std::string &filename)
{
    // Clear visualization structures first
    polyscope::removeAllStructures();

    // Use ScenarioLoader to load the scenario
    if (ScenarioLoader::loadFromFile(*m_rigidBodySystem, filename))
    {
        std::cout << "Successfully loaded scenario from " << filename << std::endl;

        // Apply visual properties and update meshes
        updateRigidBodyMeshes(*m_rigidBodySystem);

        // Save state and reset view
        m_resetState->save(*m_rigidBodySystem);
        polyscope::resetScreenshotIndex();
    }
    else
    {
        std::cerr << "Failed to load scenario from " << filename << std::endl;
    }
}

void SimViewer::drawScenarioSelectionGUI()
{
    if (ImGui::CollapsingHeader("JSON Scenarios", ImGuiTreeNodeFlags_DefaultOpen))
    {
        if (ImGui::Button("Refresh Scenarios List"))
        {
            refreshScenariosList();
        }

        // Display scenario list
        if (!m_availableScenarios.empty())
        {
            ImGui::Text("Available Scenarios:");
            ImGui::Indent();

            for (int i = 0; i < m_availableScenarios.size(); i++)
            {
                if (ImGui::Selectable(m_availableScenarios[i].c_str(), m_selectedScenario == i))
                {
                    m_selectedScenario = i;
                }
            }

            ImGui::Unindent();

            // Load button
            if (m_selectedScenario >= 0 && m_selectedScenario < m_availableScenarios.size())
            {
                if (ImGui::Button("Load Selected Scenario"))
                {
                    loadScenarioFromJSON(m_availableScenarios[m_selectedScenario]);
                }
            }
        }
        else
        {
            ImGui::Text("No JSON scenario files found.");
        }
    }
}

void SimViewer::drawGUI()
{
    // Main control window
    ImGui::Begin("Simulation Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        ImGui::Text("Time: %.2f s", m_dynamicsTime);

        if (ImGui::Button(m_paused ? "Play" : "Pause", ImVec2(100, 0)))
            m_paused = !m_paused;

        ImGui::SameLine();
        if (ImGui::Button("Step", ImVec2(100, 0)))
            m_stepOnce = true;

        ImGui::SameLine();
        if (ImGui::Button("Reset", ImVec2(100, 0)))
            reset();

        ImGui::SameLine();
        if (ImGui::Button("Save", ImVec2(100, 0)))
            save();

        ImGui::Separator();

        ImGui::Checkbox("Enable collision detection", &m_enableCollisions);
        if (m_enableCollisions != m_rigidBodySystem->getEnableCollisionDetection())
            m_rigidBodySystem->setEnableCollisionDetection(m_enableCollisions);

        ImGui::Checkbox("Draw contacts", &m_drawContacts);
        ImGui::Checkbox("Draw constraints", &m_drawConstraints);
        ImGui::Checkbox("Enable screenshots", &m_enableScreenshots);
    }
    ImGui::End();

    // Solver settings window
    ImGui::Begin("Solver Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        ImGui::PushItemWidth(150);

        // Time step and sub-steps
        ImGui::SliderFloat("Time step", &m_dt, 0.001f, 0.1f, "%.3f s");
        ImGui::SliderInt("Sub-steps", &m_subSteps, 1, 20);

        // Solver iterations
        int solverIters = m_rigidBodySystem->getSolverIterations();
        if (ImGui::SliderInt("Solver iterations", &solverIters, 1, 100))
            m_rigidBodySystem->setSolverIterations(solverIters);

        // Friction coefficient
        ImGui::SliderFloat("Friction coeff.", &(Contact::mu), 0.0f, 2.0f, "%.2f");

        // Integrator selector (new feature)
        static const char* integrators[] = {
            "Explicit Euler", "Symplectic Euler", "Verlet", "RK4", "Implicit Euler"
        };
        static int currentIntegrator = 1; // Default to Symplectic Euler
        if (ImGui::Combo("Integrator", &currentIntegrator, integrators, IM_ARRAYSIZE(integrators)))
        {
            // Update the integrator in the physics system
            m_rigidBodySystem->setIntegratorType(static_cast<IntegratorType>(currentIntegrator));
        }

        ImGui::Separator();

        // Solver type radio buttons
        SolverType currentType = m_rigidBodySystem->getSolverType();

        ImGui::Text("Solver Type:");
        bool isPGS = (currentType == SolverType::PGS);
        if (ImGui::RadioButton("PGS", isPGS))
            m_rigidBodySystem->setSolverType(SolverType::PGS);

        ImGui::SameLine();
        bool isPGSSM = (currentType == SolverType::PGSSM);
        if (ImGui::RadioButton("PGSSM", isPGSSM))
            m_rigidBodySystem->setSolverType(SolverType::PGSSM);

        bool isConjGrad = (currentType == SolverType::CONJ_GRADIENT);
        if (ImGui::RadioButton("Conj. Gradient (NO CONTACT)", isConjGrad))
            m_rigidBodySystem->setSolverType(SolverType::CONJ_GRADIENT);

        ImGui::SameLine();
        bool isConjRes = (currentType == SolverType::CONJ_RESIDUAL);
        if (ImGui::RadioButton("Conj. Residual (NO CONTACT)", isConjRes))
            m_rigidBodySystem->setSolverType(SolverType::CONJ_RESIDUAL);

        ImGui::PopItemWidth();
    }
    ImGui::End();

    // Scenarios window
    ImGui::Begin("Scenarios", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        if (ImGui::CollapsingHeader("Built-in Scenarios", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Sphere on box", ImVec2(150, 0)))
                createSphereOnBox();

            ImGui::SameLine();
            if (ImGui::Button("Marble box", ImVec2(150, 0)))
                createMarbleBox();

            if (ImGui::Button("Swinging box", ImVec2(150, 0)))
                createSwingingBox();

            ImGui::SameLine();
            if (ImGui::Button("Cylinder on plane", ImVec2(150, 0)))
                createCylinderOnPlane();

            if (ImGui::Button("Create car scene", ImVec2(150, 0)))
                createCarScene();
        }

        // JSON scenario selection UI
        drawScenarioSelectionGUI();
    }
    ImGui::End();

    // Scene hierarchy window
    ImGui::Begin("Scene Hierarchy", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        // Rigid Bodies with selection capabilities
        auto& bodies = m_rigidBodySystem->getBodies();

        if (ImGui::TreeNode("Rigid Bodies"))
        {
            for (size_t i = 0; i < bodies.size(); ++i)
            {
                char buf[64];
                const auto* mesh = bodies[i]->mesh;
                std::string meshName;

                if (mesh)
                    meshName = mesh->name;
                else
                    meshName = "<no mesh>";

                snprintf(buf, sizeof(buf), "Body %zu: %s", i, meshName.c_str());

                // Make bodies selectable
                bool isSelected = (m_selectedBodyIndex == (int)i);
                if (ImGui::Selectable(buf, isSelected))
                    m_selectedBodyIndex = (int)i;

                // Right-click context menu
                if (ImGui::BeginPopupContextItem())
                {
                    if (ImGui::MenuItem("Focus Camera"))
                    {
                        // Focus Polyscope camera on this body
                        if (bodies[i]->mesh)
                        {
                            polyscope::view::lookAt(
                                glm::vec3(bodies[i]->x[0], bodies[i]->x[1], bodies[i]->x[2]),
                                glm::vec3(0, 0, 0),
                                false
                            );
                        }
                    }

                    if (ImGui::MenuItem("Toggle Fixed"))
                        bodies[i]->fixed = !bodies[i]->fixed;

                    ImGui::EndPopup();
                }
            }
            ImGui::TreePop();
        }

        // Display joints if any
        const auto& joints = m_rigidBodySystem->getJoints();
        if (!joints.empty() && ImGui::TreeNode("Joints"))
        {
            for (size_t i = 0; i < joints.size(); ++i)
            {
                char buf[64];
                snprintf(buf, sizeof(buf), "Joint %zu: %s", i, joints[i]->getTypeName().c_str());

                if (ImGui::TreeNode(buf))
                {
                    ImGui::Text("Body 0: %d", joints[i]->body0->id);
                    ImGui::Text("Body 1: %d", joints[i]->body1->id);

                    ImGui::TreePop();
                }
            }
            ImGui::TreePop();
        }

        // Selected body properties panel
        if (m_selectedBodyIndex >= 0 && m_selectedBodyIndex < (int)bodies.size())
        {
            if (ImGui::TreeNode("Selected Body Properties"))
            {
                auto* body = bodies[m_selectedBodyIndex];

                ImGui::Checkbox("Fixed", &body->fixed);
                ImGui::DragFloat3("Position", body->x.data(), 0.1f);

                // Quaternion display and editing
                float quat[4] = {body->q.w(), body->q.x(), body->q.y(), body->q.z()};
                if (ImGui::DragFloat4("Orientation (WXYZ)", quat, 0.01f))
                {
                    body->q.w() = quat[0];
                    body->q.x() = quat[1];
                    body->q.y() = quat[2];
                    body->q.z() = quat[3];
                    body->q.normalize();
                }

                ImGui::DragFloat3("Linear Velocity", body->xdot.data(), 0.1f);
                ImGui::DragFloat3("Angular Velocity", body->omega.data(), 0.1f);

                // Material properties
                if (ImGui::TreeNode("Material Properties"))
                {
                    float restitution = body->restitution;
                    if (ImGui::SliderFloat("Restitution", &restitution, 0.0f, 1.0f))
                        body->restitution = restitution;

                    float friction = body->friction;
                    if (ImGui::SliderFloat("Friction", &friction, 0.0f, 1.0f))
                        body->friction = friction;

                    float density = body->density;
                    if (ImGui::DragFloat("Density", &density, 0.1f, 0.1f, 10000.0f))
                        body->density = density;

                    ImGui::TreePop();
                }

                // Visual properties editor
                if (body->mesh)
                {
                    if (ImGui::TreeNode("Visual Properties"))
                    {
                        float color[3] = {0.5f, 0.5f, 0.5f};
                        if (body->visualProperties.count("colorR") > 0)
                        {
                            color[0] = body->visualProperties["colorR"];
                            color[1] = body->visualProperties["colorG"];
                            color[2] = body->visualProperties["colorB"];
                        }

                        if (ImGui::ColorEdit3("Color", color))
                        {
                            body->visualProperties["colorR"] = color[0];
                            body->visualProperties["colorG"] = color[1];
                            body->visualProperties["colorB"] = color[2];
                            body->applyVisualProperties();
                        }

                        float transparency = body->visualProperties.count("transparency") ?
                            body->visualProperties["transparency"] : 1.0f;
                        if (ImGui::SliderFloat("Transparency", &transparency, 0.0f, 1.0f))
                        {
                            body->visualProperties["transparency"] = transparency;
                            body->applyVisualProperties();
                        }

                        bool smoothShade = body->visualProperties.count("smoothShade") ?
                            body->visualProperties["smoothShade"] > 0.5f : false;
                        if (ImGui::Checkbox("Smooth Shading", &smoothShade))
                        {
                            body->visualProperties["smoothShade"] = smoothShade ? 1.0f : 0.0f;
                            body->applyVisualProperties();
                        }

                        float edgeWidth = body->visualProperties.count("edgeWidth") ?
                            body->visualProperties["edgeWidth"] : 1.0f;
                        if (ImGui::SliderFloat("Edge Width", &edgeWidth, 0.0f, 5.0f))
                        {
                            body->visualProperties["edgeWidth"] = edgeWidth;
                            body->applyVisualProperties();
                        }

                        ImGui::TreePop();
                    }
                }

                ImGui::TreePop();
            }
        }
    }
    ImGui::End();

    // Debug window
    ImGui::Begin("Debug", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        drawColorDebugUI();

        ImGui::Separator();

        if (ImGui::CollapsingHeader("Polyscope Debug"))
        {
            // Ground plane options
            bool groundPlane = polyscope::options::groundPlaneEnabled;
            if (ImGui::Checkbox("Ground Plane", &groundPlane))
                polyscope::options::groundPlaneEnabled = groundPlane;

            static const char* groundPlaneModeNames[] = {
                "None", "Tile", "TileReflection", "Shadow"
            };
            int groundPlaneMode = static_cast<int>(polyscope::options::groundPlaneMode);
            if (ImGui::Combo("Ground Plane Mode", &groundPlaneMode, groundPlaneModeNames, 4))
                polyscope::options::groundPlaneMode = static_cast<polyscope::GroundPlaneMode>(groundPlaneMode);

            ImGui::SliderFloat("Ground Plane Height", &polyscope::options::groundPlaneHeightFactor, -2.0f, 2.0f);

            // Rendering options
            ImGui::Separator();
            ImGui::Text("Rendering Options:");

            ImGui::SliderInt("SSAA Factor", &polyscope::options::ssaaFactor, 1, 4);

            bool showAxis = polyscope::options::axisEnabled;
            if (ImGui::Checkbox("Show Axes", &showAxis))
                polyscope::options::axisEnabled = showAxis;

            bool shadowsEnabled = polyscope::options::shadowsEnabled;
            if (ImGui::Checkbox("Shadows", &shadowsEnabled))
                polyscope::options::shadowsEnabled = shadowsEnabled;

            bool transparencyEnabled = polyscope::options::transparencyMode;
            if (ImGui::Checkbox("Transparency", &transparencyEnabled))
                polyscope::options::transparencyMode = transparencyEnabled;

            // Camera controls
            ImGui::Separator();
            ImGui::Text("Camera Controls:");

            if (ImGui::Button("Reset Camera"))
                polyscope::view::resetCameraToDefault();

            if (ImGui::Button("Top View"))
                polyscope::view::lookAt(glm::vec3(0, 10, 0), glm::vec3(0, 0, 0), false);

            ImGui::SameLine();
            if (ImGui::Button("Side View"))
                polyscope::view::lookAt(glm::vec3(10, 0, 0), glm::vec3(0, 0, 0), false);

            ImGui::SameLine();
            if (ImGui::Button("Front View"))
                polyscope::view::lookAt(glm::vec3(0, 0, 10), glm::vec3(0, 0, 0), false);

            // Debug visualization toggles
            ImGui::Separator();
            ImGui::Text("Debug Visualization:");

            bool showWireframe = false; // Connect to actual state if available
            if (ImGui::Checkbox("Wireframe", &showWireframe))
            {
                auto& bodies = m_rigidBodySystem->getBodies();
                for (auto& body : bodies)
                {
                    if (body->mesh)
                    {
                        body->visualProperties["showWireframe"] = showWireframe ? 1.0f : 0.0f;
                        body->applyVisualProperties();
                    }
                }
            }

            bool showColliders = false; // Connect to actual state if available
            if (ImGui::Checkbox("Show Colliders", &showColliders))
            {
                // TODO: Implement visualization of collision geometry
            }
        }
    }
    ImGui::End();

    // Performance window
    ImGui::Begin("Performance", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        ImGui::Text("Physics step time: %.3f ms", m_dynamicsTime);

        // FPS counter
        static float fpsValues[120] = {}; // 2 seconds at 60 FPS
        static int fpsOffset = 0;
        static float fpsRefreshTime = 0.0f;
        static float averageFps = 0.0f;

        // Update fps every 1/2 second
        static auto lastTime = std::chrono::high_resolution_clock::now();
        auto currentTime = std::chrono::high_resolution_clock::now();
        float deltaTime = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - lastTime).count();
        lastTime = currentTime;

        fpsRefreshTime += deltaTime;
        float fps = 1.0f / deltaTime;
        fpsValues[fpsOffset] = fps;
        fpsOffset = (fpsOffset + 1) % IM_ARRAYSIZE(fpsValues);

        if (fpsRefreshTime >= 0.5f)
        {
            fpsRefreshTime = 0;
            float sum = 0;
            for (int i = 0; i < IM_ARRAYSIZE(fpsValues); i++)
                sum += fpsValues[i];
            averageFps = sum / IM_ARRAYSIZE(fpsValues);
        }

        ImGui::Text("Framerate: %.1f FPS", averageFps);
        ImGui::PlotLines("FPS", fpsValues, IM_ARRAYSIZE(fpsValues), fpsOffset, nullptr, 0.0f, 120.0f, ImVec2(200, 50));

        // System stats
        ImGui::Separator();
        ImGui::Text("System Statistics:");
        ImGui::Text("Bodies: %zu", m_rigidBodySystem->getBodies().size());
        ImGui::Text("Contacts: %zu", m_rigidBodySystem->getContacts().size());
        ImGui::Text("Joints: %zu", m_rigidBodySystem->getJoints().size());
    }
    ImGui::End();
}

void SimViewer::draw()
{
    drawGUI();

    if (!m_paused || m_stepOnce)
    {
        auto start = std::chrono::high_resolution_clock::now();

        // Step the simulation.
        // The time step dt is divided by the number of sub-steps.
        const float dt = m_dt / (float)m_subSteps;
        for (int i = 0; i < m_subSteps; ++i)
        {
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
        else
        {
            polyscope::removePointCloud(strJointPoints);
            polyscope::removeCurveNetwork(strJointCurve);
        }

        // Calculate step time
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        m_dynamicsTime = (float)duration.count() / 1000.0f;

        if (m_enableScreenshots)
        {
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

void SimViewer::preStep(std::vector<RigidBody *> &_bodies)
{
    // Hook for pre-step operations (currently empty)
    // Can be used for custom manipulations before physics step
    // Silencing warning about unused parameter
    (void)_bodies;
}

void SimViewer::drawColorDebugUI()
{
    if (ImGui::CollapsingHeader("Color Debug", ImGuiTreeNodeFlags_DefaultOpen))
    {
        const auto &bodies = m_rigidBodySystem->getBodies();
        if (bodies.empty())
        {
            ImGui::Text("No bodies in scene");
            return;
        }

        // Select a body to edit
        static int selectedBody = 0;
        if (selectedBody >= (int)bodies.size())
            selectedBody = 0;

        std::vector<const char *> bodyLabels;
        for (size_t i = 0; i < bodies.size(); i++)
        {
            static char buffer[32];
            sprintf(buffer, "Body %zu", i);
            bodyLabels.push_back(buffer);
        }

        ImGui::Combo("Select Body", &selectedBody, bodyLabels.data(), (int)bodies.size());

        // Get the selected body
        RigidBody *body = bodies[selectedBody];
        if (!body->mesh)
        {
            ImGui::Text("Selected body has no mesh");
            return;
        }

        // Edit colors
        static float color[3] = {0.5f, 0.5f, 0.5f};
        static float transparency = 1.0f; // Default to fully opaque
        static bool smoothShade = true;
        static float edgeWidth = 1.0f;

        // Show current values
        bool hasColor = body->visualProperties.count("colorR") > 0;
        if (hasColor)
        {
            color[0] = body->visualProperties["colorR"];
            color[1] = body->visualProperties["colorG"];
            color[2] = body->visualProperties["colorB"];
        }

        bool hasTransparency = body->visualProperties.count("transparency") > 0;
        if (hasTransparency)
        {
            transparency = body->visualProperties["transparency"];
        }

        bool hasSmoothShade = body->visualProperties.count("smoothShade") > 0;
        if (hasSmoothShade)
        {
            smoothShade = body->visualProperties["smoothShade"] > 0.5f;
        }

        bool hasEdgeWidth = body->visualProperties.count("edgeWidth") > 0;
        if (hasEdgeWidth)
        {
            edgeWidth = body->visualProperties["edgeWidth"];
        }

        // Color editor
        bool changed = false;
        changed |= ImGui::ColorEdit3("Color", color);
        changed |= ImGui::SliderFloat("Transparency", &transparency, 0.0f, 1.0f);
        changed |= ImGui::Checkbox("Smooth Shading", &smoothShade);
        changed |= ImGui::SliderFloat("Edge Width", &edgeWidth, 0.1f, 3.0f);

        if (changed)
        {
            body->visualProperties["colorR"] = color[0];
            body->visualProperties["colorG"] = color[1];
            body->visualProperties["colorB"] = color[2];
            body->visualProperties["transparency"] = transparency;
            body->visualProperties["smoothShade"] = smoothShade ? 1.0f : 0.0f;
            body->visualProperties["edgeWidth"] = edgeWidth;

            // Apply changes immediately
            body->applyVisualProperties();
        }

        // Button to reset to default colors
        if (ImGui::Button("Reset to Default"))
        {
            body->visualProperties.clear();
            updateRigidBodyMeshes(*m_rigidBodySystem);
        }
    }
}