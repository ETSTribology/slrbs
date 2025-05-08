#include "viewer/SimViewer.h"

#include "polyscope/polyscope.h"
#include "polyscope/curve_network.h"
#include "polyscope/point_cloud.h"
#include "polyscope/pick.h"
#include "polyscope/surface_mesh.h"
#include "polyscope/view.h"
#include <functional>
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
    static RigidBodySystem* m_rigidBodySystem = new RigidBodySystem;

    static const char* strContacts    = "contacts";
    static const char* strJointPoints = "jointsPoint";
    static const char* strJointCurve  = "jointsCurve";

    static void updateRigidBodyMeshes(RigidBodySystem& _rigidBodySystem)
    {
        auto& bodies = _rigidBodySystem.getBodies();
        for (unsigned int k = 0; k < bodies.size(); ++k)
        {
            if (!bodies[k]->mesh) continue;

            Eigen::Isometry3f tm = Eigen::Isometry3f::Identity();
            tm.linear()      = bodies[k]->q.toRotationMatrix();
            tm.translation() = bodies[k]->x;
            bodies[k]->mesh->setTransform(glm::make_mat4x4(tm.data()));

            // Apply body‐specific
            bodies[k]->applyVisualProperties();

            // Fallback to global
            bool foundGlobal = false;
            for (auto const& prop : g_visualProperties.bodyProperties)
            {
                if (prop.bodyIndex == (int)k)
                {
                    bodies[k]->mesh->setSurfaceColor({prop.colorR, prop.colorG, prop.colorB});
                    bodies[k]->mesh->setTransparency(prop.transparency);
                    bodies[k]->mesh->setSmoothShade(prop.smoothShade);
                    bodies[k]->mesh->setEdgeWidth(prop.edgeWidth);
                    foundGlobal = true;
                    break;
                }
            }

            // Default
            if (!foundGlobal && bodies[k]->visualProperties.empty())
            {
                if (bodies[k]->fixed)
                {
                    bodies[k]->mesh->setSurfaceColor({0.5f, 0.5f, 0.5f});
                    bodies[k]->mesh->setTransparency(1.0f);
                    bodies[k]->mesh->setSmoothShade(false);
                }
                else
                {
                    float hue = fmodf(k * 0.618033988749895f, 1.0f);
                    float h   = hue * 6.0f;
                    float c   = 0.9f;
                    float x   = c * (1.0f - fabsf(fmodf(h, 2.0f) - 1.0f));
                    float r = 0, g = 0, b = 0;
                    if      (h < 1.0f) { r = c; g = x; }
                    else if (h < 2.0f) { r = x; g = c; }
                    else if (h < 3.0f) { g = c; b = x; }
                    else if (h < 4.0f) { g = x; b = c; }
                    else if (h < 5.0f) { r = x; b = c; }
                    else               { r = c; b = x; }
                    bodies[k]->mesh->setSurfaceColor({r, g, b});
                    bodies[k]->mesh->setTransparency(1.0f);
                    bodies[k]->mesh->setSmoothShade(true);
                }
            }
        }
    }

    static void updateContactPoints(RigidBodySystem& _rigidBodySystem)
    {
        const auto& contacts    = _rigidBodySystem.getContacts();
        const unsigned int nCtrs = (unsigned int)contacts.size();

        if (nCtrs == 0)
        {
            polyscope::removePointCloud(strContacts);
        }
        else
        {
            Eigen::MatrixXf P(nCtrs, 3), N(nCtrs, 3);
            for (unsigned int i = 0; i < nCtrs; ++i)
            {
                P.row(i) << contacts[i]->p(0), contacts[i]->p(1), contacts[i]->p(2);
                N.row(i) << contacts[i]->n(0), contacts[i]->n(1), contacts[i]->n(2);
            }
            auto pc = polyscope::registerPointCloud(strContacts, P);
            pc->setPointColor({1,0,0});
            pc->setPointRadius(0.005f);
            pc->addVectorQuantity("normal", N)
              ->setVectorColor({1,1,0})
              ->setVectorLengthScale(0.05f)
              ->setEnabled(true);
        }
    }

    static void updateJointViz(RigidBodySystem& _rigidBodySystem)
    {
        const auto& joints     = _rigidBodySystem.getJoints();
        const unsigned int nJn = (unsigned int)joints.size();

        if (nJn == 0)
        {
            polyscope::removePointCloud(strJointPoints);
            polyscope::removeCurveNetwork(strJointCurve);
        }
        else
        {
            Eigen::MatrixXf P(2*nJn, 3);
            Eigen::MatrixXi E(nJn, 2);
            for (unsigned int i = 0; i < nJn; ++i)
            {
                auto p0 = joints[i]->body0->q * joints[i]->r0 + joints[i]->body0->x;
                auto p1 = joints[i]->body1->q * joints[i]->r1 + joints[i]->body1->x;
                P.row(2*i  ) = p0;
                P.row(2*i+1) = p1;
                E.row(i)     = Eigen::Vector2i(2*i, 2*i+1);
            }
            auto pc = polyscope::registerPointCloud(strJointPoints, P);
            pc->setPointColor({0,0,1});
            pc->setPointRadius(0.005f);
            polyscope::registerCurveNetwork(strJointCurve, P, E)
                     ->setRadius(0.002f);
        }
    }
}

SimViewer::SimViewer()
 : m_dt(0.01f), m_subSteps(1), m_dynamicsTime(0.0f),
   m_paused(true), m_stepOnce(false),
   m_enableCollisions(true), m_enableScreenshots(false),
   m_drawContacts(true), m_drawConstraints(true),
   m_selectedScenario(-1), m_selectedBodyIndex(-1)
{
    m_resetState = make_unique<RigidBodySystemState>(*m_rigidBodySystem);
    refreshScenariosList();
    reset();
}

SimViewer::~SimViewer() = default;

void SimViewer::reset()
{
    cout << "---- Reset ----" << endl;
    m_resetState->restore(*m_rigidBodySystem);
    m_dynamicsTime = 0.0f;
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::save()
{
    cout << "---- Saving current state ----" << endl;
    m_resetState->save(*m_rigidBodySystem);
}

void SimViewer::start()
{
    // Polyscope options
    polyscope::options::programName = "slrbs";
    polyscope::options::verbosity   = 0;
    polyscope::options::usePrefsFile = false;
    polyscope::options::alwaysRedraw = true;
    polyscope::options::ssaaFactor   = 2;
    polyscope::options::openImGuiWindowForUserCallback = true;
    polyscope::options::groundPlaneHeightFactor = 0.0f;
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::TileReflection;
    polyscope::options::buildGui       = false;
    polyscope::options::maxFPS         = -1;
    polyscope::options::groundPlaneEnabled = true;
    polyscope::options::screenshotExtension = ".png";

    polyscope::init();

    polyscope::options::automaticallyComputeSceneExtents = false;
    polyscope::state::lengthScale = 10.0f;
    polyscope::state::boundingBox =
      std::tuple<glm::vec3, glm::vec3>{{-5,0,-5}, {5,5,5}};

    polyscope::state::userCallback =
      std::bind(&SimViewer::draw, this);

    m_rigidBodySystem->setPreStepFunc(
        [this](std::vector<RigidBody*>& bodies) {
            this->preStep(bodies);
        });

    polyscope::show();
}

void SimViewer::refreshScenariosList()
{
    m_availableScenarios = ScenarioLoader::listAvailableScenarios();
    if (m_availableScenarios.empty())
        cout << "No JSON scenario files found." << endl;
    else
        cout << "Found " << m_availableScenarios.size() << " JSON scenario files." << endl;
    m_selectedScenario = -1;
}

void SimViewer::loadScenarioFromJSON(const string& filename)
{
    polyscope::removeAllStructures();
    if (ScenarioLoader::loadFromFile(*m_rigidBodySystem, filename))
    {
        cout << "Successfully loaded scenario from " << filename << endl;
        updateRigidBodyMeshes(*m_rigidBodySystem);
        m_resetState->save(*m_rigidBodySystem);
        polyscope::resetScreenshotIndex();
    }
    else
    {
        cerr << "Failed to load scenario from " << filename << endl;
    }
}

void SimViewer::drawScenarioSelectionGUI()
{
    if (!ImGui::CollapsingHeader("JSON Scenarios", ImGuiTreeNodeFlags_DefaultOpen))
        return;
    if (ImGui::Button("Refresh Scenarios List"))
        refreshScenariosList();
    if (m_availableScenarios.empty())
    {
        ImGui::Text("No JSON scenario files found.");
    }
    else
    {
        ImGui::Text("Available Scenarios:");
        ImGui::Indent();
        for (int i = 0; i < (int)m_availableScenarios.size(); ++i)
        {
            if (ImGui::Selectable(m_availableScenarios[i].c_str(), m_selectedScenario == i))
                m_selectedScenario = i;
        }
        ImGui::Unindent();
        if (m_selectedScenario >= 0 && m_selectedScenario < (int)m_availableScenarios.size())
        {
            if (ImGui::Button("Load Selected Scenario"))
                loadScenarioFromJSON(m_availableScenarios[m_selectedScenario]);
        }
    }
}

void SimViewer::drawGUI()
{
    // Simulation Controls
    ImGui::Begin("Simulation Controls", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        ImGui::Text("Time: %.2f s", m_dynamicsTime);
        if (ImGui::Button(m_paused ? "Play" : "Pause", ImVec2(100,0)))
            m_paused = !m_paused;
        ImGui::SameLine();
        if (ImGui::Button("Step", ImVec2(100,0))) m_stepOnce = true;
        ImGui::SameLine();
        if (ImGui::Button("Reset", ImVec2(100,0))) reset();
        ImGui::SameLine();
        if (ImGui::Button("Save", ImVec2(100,0))) save();
        ImGui::Separator();
        ImGui::Checkbox("Enable collision detection", &m_enableCollisions);
        if (m_enableCollisions != m_rigidBodySystem->getEnableCollisionDetection())
            m_rigidBodySystem->setEnableCollisionDetection(m_enableCollisions);
        ImGui::Checkbox("Draw contacts", &m_drawContacts);
        ImGui::Checkbox("Draw constraints", &m_drawConstraints);
        ImGui::Checkbox("Enable screenshots", &m_enableScreenshots);
    }
    ImGui::End();

    ImGui::Begin("Solver Settings", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::PushItemWidth(150);
    {
        // Time step & substeps
        ImGui::SliderFloat("Time step", &m_dt, 0.001f, 0.1f, "%.3f s");
        ImGui::SliderInt("Sub-steps", &m_subSteps, 1, 20);

        // Solver iterations & friction
        int iters = m_rigidBodySystem->getSolverIterations();
        if (ImGui::SliderInt("Solver iterations", &iters, 1, 100))
            m_rigidBodySystem->setSolverIterations(iters);
        ImGui::SliderFloat("Friction coeff.", &Contact::mu, 0.0f, 2.0f, "%.2f");

        // Integrator selection with tooltip
        static const char* integrators[] = {
            "Explicit Euler", "Symplectic Euler", "Verlet", "RK4", "Implicit Euler"
        };
        int currentIntegrator = static_cast<int>(m_rigidBodySystem->getIntegratorType());
        if (ImGui::Combo("Integrator", &currentIntegrator, integrators, IM_ARRAYSIZE(integrators))) {
            m_rigidBodySystem->setIntegratorType(static_cast<IntegratorType>(currentIntegrator));
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Choose your integration method. "
                              "Implicit Euler offers extra stability options.");

        // If using Implicit Euler, show extra parameters
        if (m_rigidBodySystem->getIntegrationMethod() == IntegrationMethod::IMPLICIT_EULER) {
            ImGui::Separator();
            ImGui::Text("Implicit-Euler Settings");

            float d = m_rigidBodySystem->getImplicitDamping();
            if (ImGui::SliderFloat("Damping", &d, 0.0f, 1.0f, "%.2f"))
                m_rigidBodySystem->setImplicitDamping(d);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Global velocity damping factor.");

            float gd = m_rigidBodySystem->getGyroscopicDamping();
            if (ImGui::SliderFloat("Gyro Damping", &gd, 0.0f, 1.0f, "%.2f"))
                m_rigidBodySystem->setGyroscopicDamping(gd);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Damping for gyroscopic forces.");

            bool lim = m_rigidBodySystem->getVelocityLimitingEnabled();
            if (ImGui::Checkbox("Limit Velocities", &lim))
                m_rigidBodySystem->setVelocityLimitingEnabled(lim);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Toggle to clamp extreme velocities.");

            float mv = m_rigidBodySystem->getMaxLinearVelocity();
            if (ImGui::DragFloat("Max Lin Vel", &mv, 1.0f, 0.0f, 1000.0f))
                m_rigidBodySystem->setMaxLinearVelocity(mv);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Max linear speed clamp.");

            float mav = m_rigidBodySystem->getMaxAngularVelocity();
            if (ImGui::DragFloat("Max Ang Vel", &mav, 1.0f, 0.0f, 1000.0f))
                m_rigidBodySystem->setMaxAngularVelocity(mav);
            if (ImGui::IsItemHovered()) ImGui::SetTooltip("Max angular speed clamp.");
        }

        ImGui::Separator();
        ImGui::Text("Solver Type:");
        SolverType st = m_rigidBodySystem->getSolverType();
        if (ImGui::RadioButton("PGS", st==SolverType::PGS))           m_rigidBodySystem->setSolverType(SolverType::PGS);
        ImGui::SameLine();
        if (ImGui::RadioButton("PGSSM", st==SolverType::PGSSM))       m_rigidBodySystem->setSolverType(SolverType::PGSSM);
        if (ImGui::RadioButton("Conj Grad", st==SolverType::CONJ_GRADIENT)) m_rigidBodySystem->setSolverType(SolverType::CONJ_GRADIENT);
        ImGui::SameLine();
        if (ImGui::RadioButton("Conj Residual", st==SolverType::CONJ_RESIDUAL)) m_rigidBodySystem->setSolverType(SolverType::CONJ_RESIDUAL);
    }
    ImGui::PopItemWidth();
    ImGui::End();

    // Scenarios
    ImGui::Begin("Scenarios", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        if (ImGui::CollapsingHeader("Built-in Scenarios", ImGuiTreeNodeFlags_DefaultOpen))
        {
            if (ImGui::Button("Sphere on box", ImVec2(150,0)))    createSphereOnBox();
            ImGui::SameLine();
            if (ImGui::Button("Marble box",    ImVec2(150,0)))    createMarbleBox();
            if (ImGui::Button("Swinging box",  ImVec2(150,0)))    createSwingingBox();
            ImGui::SameLine();
            if (ImGui::Button("Cylinder on plane", ImVec2(150,0))) createCylinderOnPlane();
            if (ImGui::Button("Create car scene",     ImVec2(150,0))) createCarScene();
        }
        drawScenarioSelectionGUI();
    }
    ImGui::End();

    // Scene Hierarchy
    ImGui::Begin("Scene Hierarchy", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        auto& bodies = m_rigidBodySystem->getBodies();
        if (ImGui::TreeNode("Rigid Bodies"))
        {
            for (size_t i = 0; i < bodies.size(); ++i)
            {
                char buf[64];
                string name = bodies[i]->mesh ? bodies[i]->mesh->name : "<no mesh>";
                snprintf(buf,64,"Body %zu: %s", i, name.c_str());
                bool sel = (m_selectedBodyIndex==(int)i);
                if (ImGui::Selectable(buf, sel)) m_selectedBodyIndex = i;
                if (ImGui::BeginPopupContextItem())
                {
                    if (ImGui::MenuItem("Focus Camera"))
                    {
                        if (bodies[i]->mesh)
                        {
                            polyscope::view::lookAt(
                                glm::vec3(bodies[i]->x[0], bodies[i]->x[1], bodies[i]->x[2]),
                                glm::vec3(0,0,0), false
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
        const auto& joints = m_rigidBodySystem->getJoints();
        if (!joints.empty() && ImGui::TreeNode("Joints"))
        {
            for (size_t i = 0; i < joints.size(); ++i)
            {
                char buf[64];
                snprintf(buf,64,"Joint %zu: %s", i, joints[i]->getTypeName().c_str());
                if (ImGui::TreeNode(buf))
                {
                    ImGui::Text("Body 0: %d", joints[i]->body0->id);
                    ImGui::Text("Body 1: %d", joints[i]->body1->id);
                    ImGui::TreePop();
                }
            }
            ImGui::TreePop();
        }
        if (m_selectedBodyIndex >= 0 && m_selectedBodyIndex < (int)bodies.size())
        {
            if (ImGui::TreeNode("Selected Body Properties"))
            {
                auto* body = bodies[m_selectedBodyIndex];
                ImGui::Checkbox("Fixed", &body->fixed);
                ImGui::DragFloat3("Position", body->x.data(), 0.1f);
                float quat[4] = {body->q.w(), body->q.x(), body->q.y(), body->q.z()};
                if (ImGui::DragFloat4("Orientation (WXYZ)", quat, 0.01f))
                {
                    body->q = Eigen::Quaternionf(quat[0], quat[1], quat[2], quat[3]);
                    body->q.normalize();
                }
                ImGui::DragFloat3("Linear Velocity",  body->xdot.data(), 0.1f);
                ImGui::DragFloat3("Angular Velocity", body->omega.data(), 0.1f);

                if (ImGui::TreeNode("Material Properties"))
                {
                    float r  = body->restitution;
                    float f  = body->friction;
                    float d  = body->density;
                    if (ImGui::SliderFloat("Restitution", &r, 0,1)) body->restitution = r;
                    if (ImGui::SliderFloat("Friction",    &f, 0,1)) body->friction    = f;
                    if (ImGui::DragFloat("Density", &d, 0.1f, 0.1f, 10000.0f)) body->density = d;
                    ImGui::TreePop();
                }

                if (body->mesh && ImGui::TreeNode("Visual Properties"))
                {
                    float col[3] = {0.5f,0.5f,0.5f};
                    if (body->visualProperties.count("colorR"))
                    {
                        col[0] = body->visualProperties["colorR"];
                        col[1] = body->visualProperties["colorG"];
                        col[2] = body->visualProperties["colorB"];
                    }
                    if (ImGui::ColorEdit3("Color", col))
                    {
                        body->visualProperties["colorR"] = col[0];
                        body->visualProperties["colorG"] = col[1];
                        body->visualProperties["colorB"] = col[2];
                        body->applyVisualProperties();
                    }
                    float trans = body->visualProperties.count("transparency")
                                  ? body->visualProperties["transparency"]
                                  : 1.0f;
                    if (ImGui::SliderFloat("Transparency", &trans, 0,1))
                    {
                        body->visualProperties["transparency"] = trans;
                        body->applyVisualProperties();
                    }
                    bool ss = body->visualProperties.count("smoothShade")
                              ? (body->visualProperties["smoothShade"]>0.5f)
                              : false;
                    if (ImGui::Checkbox("Smooth Shading", &ss))
                    {
                        body->visualProperties["smoothShade"] = ss?1.0f:0.0f;
                        body->applyVisualProperties();
                    }
                    float ew = body->visualProperties.count("edgeWidth")
                               ? body->visualProperties["edgeWidth"]
                               : 1.0f;
                    if (ImGui::SliderFloat("Edge Width", &ew, 0.0f,5.0f))
                    {
                        body->visualProperties["edgeWidth"] = ew;
                        body->applyVisualProperties();
                    }
                    ImGui::TreePop();
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
            // Ground plane toggle
            bool gp = polyscope::options::groundPlaneEnabled;
            if (ImGui::Checkbox("Ground Plane", &gp))
                polyscope::options::groundPlaneEnabled = gp;

            // Ground plane mode
            static const char* modes[] = {"None","Tile","TileReflection","Shadow"};
            int mode = (int)polyscope::options::groundPlaneMode;
            if (ImGui::Combo("Ground Plane Mode", &mode, modes, IM_ARRAYSIZE(modes)))
                polyscope::options::groundPlaneMode =
                  (polyscope::GroundPlaneMode)mode;

            // Ground plane height (ScaledValue<float>)
            {
                auto& gph = polyscope::options::groundPlaneHeightFactor;
                float h = gph.asAbsolute();
                if (ImGui::SliderFloat("Ground Plane Height", &h, -2.0f, 2.0f)) {
                    gph = polyscope::ScaledValue<float>::absolute(h);
                }


            }

            // Rendering options
            ImGui::Separator();
            ImGui::Text("Rendering Options:");
            ImGui::SliderInt("SSAA Factor", &polyscope::options::ssaaFactor, 1, 4);

            // Camera controls
            ImGui::Separator();
            ImGui::Text("Camera Controls:");
            if (ImGui::Button("Top View"))
                polyscope::view::lookAt({0,10,0},{0,0,0},{0,0,1},false);
            ImGui::SameLine();
            if (ImGui::Button("Side View"))
                polyscope::view::lookAt({10,0,0},{0,0,0},false);
            ImGui::SameLine();
            if (ImGui::Button("Front View"))
                polyscope::view::lookAt({0,0,10},{0,0,0},false);

            // Note: axisEnabled, shadowsEnabled, transparencyMode, resetCameraToDefault()
            //       were removed in the latest Polyscope API.
        }
    }
    ImGui::End();

    // Performance window
    ImGui::Begin("Performance", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    {
        ImGui::Text("Physics step time: %.3f ms", m_dynamicsTime);
        static float fpsVals[120]={};
        static int   fpsOff=0;
        static float fpsAcc=0, avgF=0;
        static auto  lastT=chrono::high_resolution_clock::now();

        auto curr = chrono::high_resolution_clock::now();
        float dt = chrono::duration<float>(curr-lastT).count();
        lastT = curr;
        fpsAcc += dt;
        float fps = 1.0f/dt;
        fpsVals[fpsOff] = fps;
        fpsOff = (fpsOff+1)%120;
        if (fpsAcc>0.5f)
        {
            fpsAcc = 0;
            float sum=0;
            for (float v:fpsVals) sum += v;
            avgF = sum/120;
        }
        ImGui::Text("Framerate: %.1f FPS", avgF);
        ImGui::PlotLines("FPS", fpsVals, 120, fpsOff, nullptr, 0.0f, 120.0f, ImVec2(200,50));

        ImGui::Separator();
        ImGui::Text("Bodies:   %zu", m_rigidBodySystem->getBodies().size());
        ImGui::Text("Contacts: %zu", m_rigidBodySystem->getContacts().size());
        ImGui::Text("Joints:   %zu", m_rigidBodySystem->getJoints().size());
    }
    ImGui::End();
}

void SimViewer::draw()
{
    drawGUI();
    if (!m_paused || m_stepOnce)
    {
        auto t0 = chrono::high_resolution_clock::now();
        float dtSub = m_dt / float(m_subSteps);
        for (int i = 0; i < m_subSteps; ++i)
            m_rigidBodySystem->step(dtSub);
        auto t1 = chrono::high_resolution_clock::now();

        updateRigidBodyMeshes(*m_rigidBodySystem);
        if (m_drawContacts)    updateContactPoints(*m_rigidBodySystem);
        else polyscope::removePointCloud(strContacts);

        if (m_drawConstraints) updateJointViz(*m_rigidBodySystem);
        else {
            polyscope::removePointCloud(strJointPoints);
            polyscope::removeCurveNetwork(strJointCurve);
        }

        m_dynamicsTime = chrono::duration_cast<chrono::microseconds>(t1-t0).count() * 1e-3f;
        if (m_enableScreenshots)
            polyscope::screenshot(false);

        m_stepOnce = false;
    }
}

void SimViewer::createMarbleBox()
{
    polyscope::removeAllStructures();
    Scenarios::createMarbleBox(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createSphereOnBox()
{
    polyscope::removeAllStructures();
    Scenarios::createSphereOnBox(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createSwingingBox()
{
    polyscope::removeAllStructures();
    Scenarios::createSwingingBoxes(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCylinderOnPlane()
{
    polyscope::removeAllStructures();
    Scenarios::createCylinderOnPlane(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::createCarScene()
{
    polyscope::removeAllStructures();
    Scenarios::createCarScene(*m_rigidBodySystem);
    m_resetState->save(*m_rigidBodySystem);
    updateRigidBodyMeshes(*m_rigidBodySystem);
    polyscope::resetScreenshotIndex();
}

void SimViewer::preStep(vector<RigidBody*>& _bodies)
{
    (void)_bodies;
}

void SimViewer::drawColorDebugUI()
{
    if (!ImGui::CollapsingHeader("Color Debug", ImGuiTreeNodeFlags_DefaultOpen))
        return;
    const auto& bodies = m_rigidBodySystem->getBodies();
    if (bodies.empty())
    {
        ImGui::Text("No bodies in scene");
        return;
    }
    static int sel=0;
    sel = min(sel, (int)bodies.size()-1);
    vector<const char*> labels;
    for (size_t i=0;i<bodies.size();++i)
    {
        static char buf[32];
        sprintf(buf,"Body %zu",i);
        labels.push_back(buf);
    }
    ImGui::Combo("Select Body",&sel,labels.data(),labels.size());

    auto* body = bodies[sel];
    if (!body->mesh)
    {
        ImGui::Text("Selected body has no mesh");
        return;
    }

    static float col[3]={0.5f,0.5f,0.5f};
    static float tr=1.0f;
    static bool  ss=true;
    static float ew=1.0f;

    if (body->visualProperties.count("colorR"))
    {
        col[0]=body->visualProperties["colorR"];
        col[1]=body->visualProperties["colorG"];
        col[2]=body->visualProperties["colorB"];
    }
    if (body->visualProperties.count("transparency"))
        tr = body->visualProperties["transparency"];
    if (body->visualProperties.count("smoothShade"))
        ss = (body->visualProperties["smoothShade"]>0.5f);
    if (body->visualProperties.count("edgeWidth"))
        ew = body->visualProperties["edgeWidth"];

    bool changed = false;
    changed |= ImGui::ColorEdit3("Color", col);
    changed |= ImGui::SliderFloat("Transparency",&tr,0,1);
    changed |= ImGui::Checkbox("Smooth Shading",&ss);
    changed |= ImGui::SliderFloat("Edge Width",&ew,0.1f,3.0f);

    if (changed)
    {
        body->visualProperties["colorR"]=col[0];
        body->visualProperties["colorG"]=col[1];
        body->visualProperties["colorB"]=col[2];
        body->visualProperties["transparency"]=tr;
        body->visualProperties["smoothShade"]= ss?1.0f:0.0f;
        body->visualProperties["edgeWidth"]=ew;
        body->applyVisualProperties();
    }

    if (ImGui::Button("Reset to Default"))
    {
        body->visualProperties.clear();
        updateRigidBodyMeshes(*m_rigidBodySystem);
    }
}
