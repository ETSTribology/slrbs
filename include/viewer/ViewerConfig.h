#pragma once

#include <string>
#include <glm/glm.hpp>

namespace slrbs {

/**
 * @struct UIConfig
 * @brief Configuration parameters for the UI
 */
struct UIConfig {
    float uiScale = 1.0f;
    bool showFPS = true;
    std::string fontPath = "";
};

/**
 * @struct CameraConfig
 * @brief Configuration parameters for the camera
 */
struct CameraConfig {
    float fov = 45.0f;
    float nearPlane = 0.1f;
    float farPlane = 1000.0f;
    float orbitRadius = 10.0f;
    float orbitPitch = 30.0f;
    float orbitYaw = 0.0f;
};

/**
 * @struct RenderingConfig
 * @brief Configuration parameters for rendering
 */
struct RenderingConfig {
    bool showGrid = true;
    bool showAxes = true;
    bool showBoundingBoxes = false;
    bool enableShadows = true;
    glm::vec3 backgroundColor{0.2f, 0.2f, 0.2f};
};

/**
 * @struct ViewerConfig
 * @brief Overall configuration for the viewer
 */
struct ViewerConfig {
    UIConfig ui;
    CameraConfig camera;
    RenderingConfig rendering;
    
    // Window properties
    int windowWidth = 1280;
    int windowHeight = 720;
    std::string windowTitle = "SLRBS - Simple Lightweight Rigid Body Simulator";
};

/**
 * @brief Load viewer configuration from a JSON file
 * @param filename The name of the JSON file in the configs directory
 * @return The loaded configuration, or default if loading fails
 */
ViewerConfig loadViewerConfigFromJson(const std::string& filename);

/**
 * @brief Save viewer configuration to a JSON file
 * @param config The configuration to save
 * @param filename The name of the JSON file in the configs directory
 */
void saveViewerConfigToJson(const ViewerConfig& config, const std::string& filename);

} // namespace slrbs
