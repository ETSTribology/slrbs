#include "viewer/ViewerUI.h"
#include "viewer/SimViewer.h"
#include "rigidbody/RigidBodySystem.h"
#include "viewer/ViewerConfig.h"
#include "viewer/SimulationManager.h"
#include "viewer/ViewerInteraction.h"
#include "viewer/ScenarioManager.h"
#include "viewer/RigidBodyRenderer.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace slrbs {

ViewerUI::ViewerUI(SimViewer* viewer) : viewer(viewer), window(nullptr) {
    // Initialize timing variables
    lastFrameTime = 0.0f;
    deltaTime = 0.0f;
    fps = 0.0f;
}

ViewerUI::~ViewerUI() {
    shutdown();
}

void ViewerUI::initialize(GLFWwindow* window) {
    this->window = window;
    
    // Setup Dear ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable keyboard navigation
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;      // Enable docking
    
    // Setup style
    ImGui::StyleColorsDark();
    
    // Setup platform/renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
    
    // Scale UI
    ImGui::GetStyle().ScaleAllSizes(viewer->getConfig().ui.uiScale);
}

void ViewerUI::render(const RigidBodySystem& system) {
    // Start new ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    
    // Update FPS counter
    updateFPS();
    
    // Render UI components
    if (showMainMenu) renderMainMenuBar();
    if (showSimulationControls) renderSimulationControls();
    if (showObjectProperties) renderObjectProperties();
    if (showPerformanceMetrics) renderPerformanceMetrics();
    
    // Render ImGui
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ViewerUI::shutdown() {
    if (ImGui::GetCurrentContext()) {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
    }
}

void ViewerUI::updateFPS() {
    // Calculate delta time and FPS
    float currentTime = static_cast<float>(glfwGetTime());
    deltaTime = currentTime - lastFrameTime;
    lastFrameTime = currentTime;
    
    // Smooth FPS calculation using moving average
    static const int FPS_SAMPLE_COUNT = 60;
    static float fpsSamples[FPS_SAMPLE_COUNT] = {0};
    static int currentSample = 0;
    
    fpsSamples[currentSample] = 1.0f / (deltaTime > 0.0f ? deltaTime : 0.001f);
    currentSample = (currentSample + 1) % FPS_SAMPLE_COUNT;
    
    float fpsSum = 0.0f;
    for (int i = 0; i < FPS_SAMPLE_COUNT; i++) {
        fpsSum += fpsSamples[i];
    }
    fps = fpsSum / FPS_SAMPLE_COUNT;
}

void ViewerUI::renderMainMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Load Scene", "Ctrl+O")) {
                // TODO: Implement scene loading dialog
            }
            if (ImGui::MenuItem("Save Scene", "Ctrl+S")) {
                viewer->getSimulation().save();
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit", "Alt+F4")) {
                glfwSetWindowShouldClose(window, true);
            }
            ImGui::EndMenu();
        }
        
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Simulation Controls", NULL, &showSimulationControls);
            ImGui::MenuItem("Object Properties", NULL, &showObjectProperties);
            ImGui::MenuItem("Performance Metrics", NULL, &showPerformanceMetrics);
            ImGui::Separator();
            
            // Camera settings
            if (ImGui::BeginMenu("Camera")) {
                if (ImGui::MenuItem("Reset Camera")) {
                    viewer->getInteraction().resetCamera();
                }
                
                float fov = viewer->getConfig().camera.fov;
                if (ImGui::SliderFloat("FOV", &fov, 30.0f, 90.0f)) {
                    viewer->getConfig().camera.fov = fov;
                    viewer->getInteraction().updateCamera();
                }
                
                ImGui::EndMenu();
            }
            
            // Rendering settings
            if (ImGui::BeginMenu("Rendering")) {
                ImGui::Checkbox("Show Grid", &viewer->getConfig().rendering.showGrid);
                ImGui::Checkbox("Show Axes", &viewer->getConfig().rendering.showAxes);
                ImGui::Checkbox("Show Bounding Boxes", &viewer->getConfig().rendering.showBoundingBoxes);
                ImGui::Checkbox("Enable Shadows", &viewer->getConfig().rendering.enableShadows);
                
                // Background color
                float bgColor[3] = {
                    viewer->getConfig().rendering.backgroundColor.r,
                    viewer->getConfig().rendering.backgroundColor.g,
                    viewer->getConfig().rendering.backgroundColor.b
                };
                if (ImGui::ColorEdit3("Background", bgColor)) {
                    viewer->getConfig().rendering.backgroundColor = glm::vec3(bgColor[0], bgColor[1], bgColor[2]);
                    glClearColor(bgColor[0], bgColor[1], bgColor[2], 1.0f);
                }
                
                ImGui::EndMenu();
            }
            
            ImGui::EndMenu();
        }
        
        if (ImGui::BeginMenu("Simulation")) {
            if (ImGui::MenuItem("Start/Pause", "Space")) {
                viewer->getSimulation().togglePause();
            }
            if (ImGui::MenuItem("Reset", "R")) {
                viewer->getSimulation().reset();
            }
            
            ImGui::Separator();
            
            if (ImGui::BeginMenu("Scenarios")) {
                const auto& scenarios = viewer->getScenarioManager().getScenarios();
                for (size_t i = 0; i < scenarios.size(); i++) {
                    if (ImGui::MenuItem(scenarios[i].name.c_str())) {
                        viewer->getScenarioManager().loadScenario(i);
                    }
                }
                ImGui::EndMenu();
            }
            
            ImGui::EndMenu();
        }
        
        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem("About")) {
                // TODO: Show about dialog
            }
            if (ImGui::MenuItem("Controls")) {
                // TODO: Show controls dialog
            }
            ImGui::EndMenu();
        }
        
        // Display FPS on the right side of the menu bar if enabled
        if (viewer->getConfig().ui.showFPS) {
            auto windowWidth = ImGui::GetWindowSize().x;
            std::stringstream fpsText;
            fpsText << std::fixed << std::setprecision(1) << fps << " FPS";
            auto textWidth = ImGui::CalcTextSize(fpsText.str().c_str()).x;
            ImGui::SameLine(windowWidth - textWidth - 10);
            ImGui::Text("%s", fpsText.str().c_str());
        }
        
        ImGui::EndMainMenuBar();
    }
}

void ViewerUI::renderSimulationControls() {
    if (ImGui::Begin("Simulation Controls", &showSimulationControls)) {
        auto& simulation = viewer->getSimulation();
        
        // Simulation control buttons
        bool isPaused = simulation.isPaused();
        if (ImGui::Checkbox("Pause", &isPaused)) {
            if (isPaused) {
                simulation.pause();
            } else {
                simulation.resume();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Step")) {
            simulation.step();
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset")) {
            simulation.reset();
        }
        
        // Time scale slider
        float timeScale = simulation.getTimeScale();
        if (ImGui::SliderFloat("Time Scale", &timeScale, 0.1f, 2.0f, "%.2f")) {
            simulation.setTimeScale(timeScale);
        }
        
        // Substeps slider
        int substeps = simulation.getSubsteps();
        if (ImGui::SliderInt("Substeps", &substeps, 1, 20)) {
            simulation.setSubsteps(substeps);
        }
        
        // Solver iterations
        int solverIters = simulation.getSolverIterations();
        if (ImGui::SliderInt("Solver Iterations", &solverIters, 1, 100)) {
            simulation.setSolverIterations(solverIters);
        }
        
        // Adaptive timesteps
        bool adaptiveTimesteps = simulation.getAdaptiveTimesteps();
        if (ImGui::Checkbox("Adaptive Timesteps", &adaptiveTimesteps)) {
            simulation.setAdaptiveTimesteps(adaptiveTimesteps);
        }
        
        // Debug visualization options
        SimViewer* simViewer = static_cast<SimViewer*>(viewer);
        bool showContacts = simViewer->showContacts;
        bool showJoints = simViewer->showJoints;
        bool showBoundingBoxes = simViewer->showBoundingBoxes;
        
        if (ImGui::Checkbox("Show Contacts", &showContacts)) {
            simViewer->showContacts = showContacts;
        }
        
        if (ImGui::Checkbox("Show Joints", &showJoints)) {
            simViewer->showJoints = showJoints;
        }
        
        if (ImGui::Checkbox("Show Bounding Boxes", &showBoundingBoxes)) {
            simViewer->showBoundingBoxes = showBoundingBoxes;
            viewer->getConfig().rendering.showBoundingBoxes = showBoundingBoxes;
        }
    }
    ImGui::End();
}

void ViewerUI::renderObjectProperties() {
    if (ImGui::Begin("Object Properties", &showObjectProperties)) {
        auto selectedObject = viewer->getInteraction().getSelectedObject();
        
        if (selectedObject) {
            // Display object properties
            ImGui::Text("Selected Object: %s", selectedObject->name.c_str());
            ImGui::Separator();
            
            // Position
            const auto& position = selectedObject->x;
            float pos[3] = {position.x(), position.y(), position.z()};
            if (ImGui::InputFloat3("Position", pos, "%.3f")) {
                selectedObject->x = Eigen::Vector3f(pos[0], pos[1], pos[2]);
            }
            
            // Material dropdown
            static int currentMat = 0;
            const char* materials[] = { "Default", "Metal", "Plastic", "Glass", "Emissive" };
            if (ImGui::Combo("Material", &currentMat, materials, IM_ARRAYSIZE(materials))) {
                viewer->getRenderer().assignMaterial(*selectedObject, materials[currentMat]);
            }
            
            // Color picker
            Eigen::Vector3f color = selectedObject->color;
            float col[3] = {color.x(), color.y(), color.z()};
            if (ImGui::ColorEdit3("Color", col)) {
                selectedObject->color = Eigen::Vector3f(col[0], col[1], col[2]);
            }
            
            // Mass properties
            float mass = selectedObject->mass;
            if (ImGui::InputFloat("Mass", &mass, 0.1f, 1.0f, "%.3f")) {
                selectedObject->mass = mass;
            }
            
            // Physics properties
            bool isStatic = selectedObject->fixed;
            if (ImGui::Checkbox("Static", &isStatic)) {
                selectedObject->fixed = isStatic;
            }
            
            // Texture selection
            if (ImGui::Button("Load Texture")) {
                // TODO: Implement texture selection dialog
            }
            
        } else {
            ImGui::Text("No object selected");
            ImGui::Text("Click on an object to select it");
        }
    }
    ImGui::End();
}

void ViewerUI::renderPerformanceMetrics() {
    if (ImGui::Begin("Performance Metrics", &showPerformanceMetrics)) {
        ImGui::Text("FPS: %.1f", fps);
        ImGui::Text("Frame Time: %.2f ms", deltaTime * 1000.0f);
        
        auto& simulation = viewer->getSimulation();
        
        ImGui::Separator();
        ImGui::Text("Physics Statistics:");
        ImGui::Text("Dynamics Time: %.2f ms", simulation.getDynamicsTime());
        ImGui::Text("Kinetic Energy: %.3f", simulation.getKineticEnergy());
        ImGui::Text("Constraint Error: %.6f", simulation.getConstraintError());
        ImGui::Text("Frame Count: %d", simulation.getFrameCount());
        
        // System statistics
        ImGui::Separator();
        ImGui::Text("System:");
        
        // Memory usage
        static float memoryUsage = 0.0f;
        static auto lastMemCheck = std::chrono::high_resolution_clock::now();
        auto now = std::chrono::high_resolution_clock::now();
        
        if (std::chrono::duration_cast<std::chrono::seconds>(now - lastMemCheck).count() > 1) {
            // TODO: Update memory usage
            lastMemCheck = now;
        }
        
        ImGui::Text("Memory Usage: %.1f MB", memoryUsage);
    }
    ImGui::End();
}

} // namespace slrbs
