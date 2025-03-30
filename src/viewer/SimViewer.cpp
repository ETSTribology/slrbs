#include "viewer/SimViewer.h"
#include "viewer/ViewerUI.h"
#include "viewer/ViewerInteraction.h"
#include "viewer/ScenarioManager.h"
#include "viewer/SimulationManager.h"
#include "utils/JsonLoader.h"
#include "rigidbody/RigidBodySystem.h"
#include "rigidbody/RigidBodyState.h"
#include "contact/Contact.h"
#include "scenarios/Scenarios.h"
#include "log/CSVLogger.h"
#include "renderer/RendererFactory.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <polyscope/polyscope.h>
#include <polyscope/point_cloud.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/view.h>
#include <polyscope/curve_network.h>

#include <iostream>
#include <stdexcept>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <functional>

namespace slrbs {

// For static data that was previously in anonymous namespace
namespace {
    static CSVLogger s_log;
    static int s_substepsIdx = -1;
    static int s_kinEnergyIdx = -1;
}

// GLFW error callback
static void glfwErrorCallback(int error, const char* description) {
    std::cerr << "GLFW Error " << error << ": " << description << std::endl;
}

// GLFW window resize callback
static void glfwResizeCallback(GLFWwindow* window, int width, int height) {
    // Get user pointer and cast to SimViewer
    SimViewer* viewer = static_cast<SimViewer*>(glfwGetWindowUserPointer(window));
    if (viewer) {
        // Update config
        viewer->getConfig().windowWidth = width;
        viewer->getConfig().windowHeight = height;
        
        // Update viewport
        glViewport(0, 0, width, height);
    }
}

SimViewer::SimViewer() : window(nullptr) {
    // Load configuration from JSON
    try {
        config = loadViewerConfigFromJson("default_viewer.json");
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config: " << e.what() << std::endl;
        std::cerr << "Using default configuration." << std::endl;
    }
    
    // Initialize components
    rigidBodySystem = std::make_unique<RigidBodySystem>();
    
    // Create the renderer
    rigidBodyRenderer = RendererFactory::createDefaultRenderer();
    
    // Initialize logging
    s_kinEnergyIdx = s_log.addField("kin_energy");
    s_substepsIdx = s_log.addField("substeps");
    
    // Create reset state
    resetState = std::make_unique<RigidBodySystemState>(*rigidBodySystem);
}

SimViewer::~SimViewer() {
    shutdown();
}

void SimViewer::initialize() {
    setupWindow();
    setupPolyscope();
    
    // Initialize renderer
    rigidBodyRenderer->initialize();
    
    // Initialize components
    ui = std::make_unique<ViewerUI>(this);
    ui->initialize(window);
    
    interaction = std::make_unique<ViewerInteraction>(this);
    interaction->initialize(window);
    
    scenarioManager = std::make_unique<ScenarioManager>(this);
    scenarioManager->initialize();
    
    simulationManager = std::make_unique<SimulationManager>(this);
    simulationManager->initialize();
    
    // Set background color
    glm::vec3& bg = config.rendering.backgroundColor;
    glClearColor(bg.r, bg.g, bg.b, 1.0f);
    
    // Add pre-step hook for geometric stiffness damping
    rigidBodySystem->setPreStepFunc([this](RigidBodySystem& system, float h) {
        this->preStep(system, h);
    });
    
    std::cout << "SimViewer initialized successfully." << std::endl;
}

void SimViewer::setupWindow() {
    // Set error callback
    glfwSetErrorCallback(glfwErrorCallback);
    
    // Initialize GLFW
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }
    
    // OpenGL context settings
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    // Mac compatibility
    #ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif
    
    // Create window
    window = glfwCreateWindow(
        config.windowWidth,
        config.windowHeight,
        config.windowTitle.c_str(),
        nullptr, nullptr
    );
    
    if (!window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    
    // Set window user pointer for callbacks
    glfwSetWindowUserPointer(window, this);
    
    // Set resize callback
    glfwSetWindowSizeCallback(window, glfwResizeCallback);
    
    // Make the window's context current
    glfwMakeContextCurrent(window);
    
    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwDestroyWindow(window);
        glfwTerminate();
        throw std::runtime_error("Failed to initialize GLAD");
    }
    
    // Enable vsync
    glfwSwapInterval(1);
    
    // Set viewport
    glViewport(0, 0, config.windowWidth, config.windowHeight);
    
    std::cout << "Window setup completed." << std::endl;
}

void SimViewer::setupPolyscope() {
    // Initialize polyscope settings
    polyscope::options::programName = config.windowTitle;
    polyscope::options::verbosity = 0;
    polyscope::options::usePrefsFile = false;
    polyscope::options::alwaysRedraw = true;
    polyscope::options::ssaaFactor = 2;
    polyscope::options::openImGuiWindowForUserCallback = true;
    polyscope::options::groundPlaneHeightFactor = 0.0f;
    polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;
    polyscope::options::buildGui = true;
    polyscope::options::maxFPS = 60;
    polyscope::options::groundPlaneEnabled = true;
    polyscope::options::screenshotExtension = ".png";
    
    // Set window size
    polyscope::view::windowWidth = config.windowWidth;
    polyscope::view::windowHeight = config.windowHeight;
    
    // Initialize
    polyscope::init();
    
    // Setup viewing volume
    polyscope::options::automaticallyComputeSceneExtents = false;
    polyscope::state::lengthScale = 10.0f;
    polyscope::state::boundingBox = std::tuple<glm::vec3, glm::vec3>{
        {-5.0, 0.0, -5.0}, {5.0, 5.0, 5.0}
    };
    
    // Set user callback to our render function
    polyscope::state::userCallback = [this]() { this->drawGUI(); };
    
    std::cout << "Polyscope setup completed." << std::endl;
}

void SimViewer::run() {
    // Use polyscope's main loop instead of our own
    polyscope::show();
}

void SimViewer::shutdown() {
    std::cout << "Shutting down SimViewer..." << std::endl;
    
    // Save current config
    try {
        saveViewerConfigToJson(config, "user_config.json");
    } catch (const std::exception& e) {
        std::cerr << "Failed to save config: " << e.what() << std::endl;
    }
    
    // Cleanup components in reverse order
    if (ui) ui->shutdown();
    if (simulationManager) simulationManager.reset();
    if (scenarioManager) scenarioManager.reset();
    if (interaction) interaction.reset();
    if (rigidBodyRenderer) rigidBodyRenderer.reset();
    if (ui) ui.reset();
    
    // Clean up polyscope
    polyscope::shutdown();
    
    // Clean up GLFW
    if (window) {
        glfwDestroyWindow(window);
        window = nullptr;
    }
    glfwTerminate();
    
    std::cout << "SimViewer shutdown completed." << std::endl;
}

void SimViewer::drawGUI() {
    // Check if we should update the simulation
    bool shouldUpdate = !paused || stepOnce;
    
    if (shouldUpdate) {
        auto start = std::chrono::high_resolution_clock::now();

        // Automatically choose next substeps based on geometric stiffness
        if (adaptiveTimesteps) {
            for (auto b : rigidBodySystem->getBodies())
                b->gsSum.setZero();

            // Compute geometric stiffness contributions
            for (auto j : rigidBodySystem->getJoints()) {
                j->computeGeometricStiffness();
                j->body0->gsSum += j->G0;
                j->body1->gsSum += j->G1;
            }

            for (auto c : rigidBodySystem->getContacts()) {
                c->computeGeometricStiffness();
                c->body0->gsSum += c->G0;
                c->body1->gsSum += c->G1;
            }

            float maxK_M = 0.0f;
            for (auto b : rigidBodySystem->getBodies()) {
                if (b->fixed) continue;

                for (int c = 0; c < 3; c++) {
                    const float m = b->mass;
                    const float k = 2.0f * b->gsSum.col(c).norm();
                    maxK_M = std::max(k / m, maxK_M);
                }

                for (int c = 0; c < 3; c++) {
                    const float m = b->I(c, c);
                    const float k = 2.0f * b->gsSum.col(c + 3).norm();
                    maxK_M = std::max(k / m, maxK_M);
                }
            }

            const float dt = 2.0f * alpha * std::sqrt(1.0f / std::max(maxK_M, 0.0001f));
            substeps = std::max(1, (int)ceil(timeStep / dt));
        }

        // Get step size accounting for substeps
        const float dt = timeStep / (float)substeps;

        // Step the simulation
        for(int i = 0; i < substeps; ++i) {
            rigidBodySystem->step(dt);
        }

        // Finish timing
        auto stop = std::chrono::high_resolution_clock::now();

        // Update frame counter
        ++frameCounter;

        // Update visualizations
        rigidBodyRenderer->renderBodies(rigidBodySystem->getBodies());
        rigidBodyRenderer->renderContacts(rigidBodySystem->getContacts());
        rigidBodyRenderer->renderJoints(rigidBodySystem->getJoints());
        rigidBodyRenderer->renderBoundingBoxes(rigidBodySystem->getBodies());

        // Calculate computation time
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        dynamicsTime = (float)duration.count() / 1000.0f;

        // Take screenshot if enabled
        if (enableScreenshots) {
            polyscope::screenshot(false);
        }

        // Clear step-once flag
        stepOnce = false;

        // Compute performance metrics
        computePerformanceMetrics();
    }
    
    // Display the UI
    ImGui::Text("Simulation:");
    
    // Basic controls
    ImGui::Checkbox("Pause", &paused);
    
    if (ImGui::Button("Step once")) {
        stepOnce = true;
    }
    
    if (ImGui::Button("Reset")) {
        reset();
    }
    
    if (ImGui::Button("Save")) {
        save();
    }
    
    // Advanced controls
    ImGui::PushItemWidth(100);
    
    ImGui::Checkbox("Adaptive time steps", &adaptiveTimesteps);
    ImGui::Checkbox("Geometric stiffness damping", &geometricStiffnessDamping);
    
    if (adaptiveTimesteps || geometricStiffnessDamping) {
        ImGui::SliderFloat("Alpha", &alpha, 0.0f, 1.0f);
    }
    
    ImGui::SliderFloat("Time step", &timeStep, 0.0f, 0.1f, "%.3f");
    ImGui::SliderInt("Num. sub-steps", &substeps, 1, 20, "%u");
    ImGui::SliderInt("Solver iters.", &(rigidBodySystem->solverIter), 1, 1000, "%u");
    ImGui::SliderFloat("Friction coeff.", &(Contact::mu), 0.0f, 2.0f, "%.2f");
    
    ImGui::RadioButton("PGS", &(rigidBodySystem->solverId), 0);  
    ImGui::SameLine();
    ImGui::RadioButton("Conj. Gradient (NO CONTACT)", &(rigidBodySystem->solverId), 1);
    ImGui::RadioButton("Conj. Residual (NO CONTACT)", &(rigidBodySystem->solverId), 2);
    ImGui::RadioButton("BPP", &(rigidBodySystem->solverId), 3);
    
    ImGui::PopItemWidth();
    
    // Debug visualization options
    if (ImGui::Checkbox("Enable collision detection", &enableCollisions)) {
        rigidBodySystem->setEnableCollisionDetection(enableCollisions);
    }
    
    bool showContacts = rigidBodyRenderer->getShowContacts();
    if (ImGui::Checkbox("Draw contacts", &showContacts)) {
        rigidBodyRenderer->setShowContacts(showContacts);
    }
    
    bool showJoints = rigidBodyRenderer->getShowJoints();
    if (ImGui::Checkbox("Draw constraints", &showJoints)) {
        rigidBodyRenderer->setShowJoints(showJoints);
    }
    
    bool showBoundingBoxes = rigidBodyRenderer->getShowBoundingBoxes();
    if (ImGui::Checkbox("Show bounding boxes", &showBoundingBoxes)) {
        rigidBodyRenderer->setShowBoundingBoxes(showBoundingBoxes);
    }
    
    ImGui::Checkbox("Enable screenshots", &enableScreenshots);
    ImGui::Checkbox("Enable logging", &enableLogging);
    
    // Scenario buttons
    ImGui::Separator();
    ImGui::Text("Scenarios:");
    
    if (ImGui::Button("Sphere on box")) loadScenario(0);
    if (ImGui::Button("Marble box")) loadScenario(1);
    if (ImGui::Button("Swinging box")) loadScenario(2);
    if (ImGui::Button("Stack")) loadScenario(3);
    if (ImGui::Button("Cylinder on plane")) loadScenario(4);
    if (ImGui::Button("Create car scene")) loadScenario(5);
    if (ImGui::Button("Create bridge scene")) loadScenario(6);
    
    // Performance metrics
    ImGui::Separator();
    ImGui::Text("Performance:");
    
    ImGui::Text("Kinetic energy: %8.3f", kineticEnergy);
    ImGui::Text("Step time: %3.3f ms", dynamicsTime);
    ImGui::Text("Constraint error: %6.6f", constraintError);
    ImGui::Text("Frame: %d", frameCounter);
}

void SimViewer::reset() {
    std::cout << "---- Reset ----" << std::endl;
    resetState->restore(*rigidBodySystem);
    dynamicsTime = 0.0f;
    frameCounter = 0;
    kineticEnergy = 0.0f;
    constraintError = 0.0f;

    if (enableLogging) {
        s_log.clear();
    }

    // Update visualization
    rigidBodyRenderer->updateAll();
    
    polyscope::resetScreenshotIndex();
}

void SimViewer::save() {
    std::cout << "---- Saving current state ----" << std::endl;
    resetState->save(*rigidBodySystem);
}

void SimViewer::loadScenario(int scenarioType) {
    scenarioManager->loadScenario(scenarioType);
    
    // Apply colors and textures from the loaded scenario
    for (auto* body : rigidBodySystem->getBodies()) {
        // Apply color if defined
        if (body->color.norm() > 0) {
            glm::vec3 color(body->color.x(), body->color.y(), body->color.z());
            rigidBodyRenderer->setBodyColor(*body, color);
        }
        
        // Apply texture if defined
        if (!body->texturePath.empty()) {
            rigidBodyRenderer->setBodyTexture(*body, body->texturePath);
        }
    }
}

void SimViewer::preStep(RigidBodySystem& system, float h) {
    // Skip if geometric stiffness damping is disabled
    if (!geometricStiffnessDamping) return;
    
    auto& bodies = system.getBodies();
    auto& joints = system.getJoints();
    auto& contacts = system.getContacts();

    // Reset geometric stiffness damping values.
    for (auto b : bodies)
        b->gsDamp.setZero();

    // Recompute geometric stiffness damping.
    // The damping will be added to the bodies' inertia matrices during integration.
    for (auto j : joints) {
        j->computeGeometricStiffness();
        j->body0->gsSum += j->G0;
        j->body1->gsSum += j->G1;
    }

    for (auto c : contacts) {
        c->computeGeometricStiffness();
        c->body0->gsSum += c->G0;
        c->body1->gsSum += c->G1;
    }

    for (auto b : system.getBodies()) {
        if (b->fixed)
            continue;

        for (int c = 0; c < 3; c++) {
            const float m = b->I(c, c);
            const float k = 2.0f * b->gsSum.col(c + 3).norm();
            b->gsDamp(c) = std::max(0.0f, h * h * k - 4 * alpha * m);
        }
        // Update inertia matrix so solver has most recent gs damping information
        b->updateInertiaMatrix();
    }
}

void SimViewer::computePerformanceMetrics() {
    // Compute kinetic energy
    kineticEnergy = 0.0f;
    for (RigidBody* b : rigidBodySystem->getBodies()) {
        const auto I = b->q * b->Ibody * b->q.inverse();
        kineticEnergy += 0.5f * (b->mass * b->xdot.squaredNorm() + b->omega.dot(I * b->omega));
    }
    
    // Compute constraint error
    constraintError = 0.0f;
    for (Joint* j : rigidBodySystem->getJoints()) {
        constraintError += j->phi.lpNorm<1>();
    }
    
    // Log values if enabled
    if (enableLogging) {
        s_log.pushVal(s_kinEnergyIdx, kineticEnergy);
        s_log.pushVal(s_substepsIdx, static_cast<float>(substeps));
    }
}

} // namespace slrbs
