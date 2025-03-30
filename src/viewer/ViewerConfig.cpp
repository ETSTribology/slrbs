#include "viewer/ViewerConfig.h"
#include "utils/JsonLoader.h"

#include <iostream>
#include <filesystem>
#include <stdexcept>

namespace slrbs {

// Function declarations
ViewerConfig loadViewerConfigFromJson(const std::string& filename);
void saveViewerConfigToJson(const ViewerConfig& config, const std::string& filename);

// Helper functions to load config elements from JSON
void loadUIConfig(UIConfig& config, const nlohmann::json& json) {
    if (json.contains("uiScale")) config.uiScale = json["uiScale"];
    if (json.contains("showFPS")) config.showFPS = json["showFPS"];
    if (json.contains("fontPath")) config.fontPath = json["fontPath"];
}

void loadCameraConfig(CameraConfig& config, const nlohmann::json& json) {
    if (json.contains("fieldOfView")) config.fov = json["fieldOfView"];
    if (json.contains("nearClipPlane")) config.nearPlane = json["nearClipPlane"];
    if (json.contains("farClipPlane")) config.farPlane = json["farClipPlane"];
    if (json.contains("orbitRadius")) config.orbitRadius = json["orbitRadius"];
    if (json.contains("orbitPitch")) config.orbitPitch = json["orbitPitch"];
    if (json.contains("orbitYaw")) config.orbitYaw = json["orbitYaw"];
    
    // Handle target position if present
    if (json.contains("target")) {
        const auto& target = json["target"];
        if (target.contains("x") && target.contains("y") && target.contains("z")) {
            // This could be stored in a camera target field if needed
            // Currently just using orbit parameters
        }
    }
}

void loadRenderingConfig(RenderingConfig& config, const nlohmann::json& json) {
    if (json.contains("showGrid")) config.showGrid = json["showGrid"];
    if (json.contains("showAxes")) config.showAxes = json["showAxes"];
    if (json.contains("showBoundingBoxes")) config.showBoundingBoxes = json["showBoundingBoxes"];
    if (json.contains("enableShadows")) config.enableShadows = json["enableShadows"];
    
    // Load background color if present
    if (json.contains("backgroundColor")) {
        const auto& bg = json["backgroundColor"];
        if (bg.contains("r") && bg.contains("g") && bg.contains("b")) {
            config.backgroundColor = glm::vec3(
                bg["r"].get<float>(),
                bg["g"].get<float>(),
                bg["b"].get<float>()
            );
        }
    }
}

// Function to load full viewer configuration from JSON
ViewerConfig loadViewerConfigFromJson(const std::string& filename) {
    try {
        std::filesystem::path configPath = JsonLoader::getResourcesPath() / "configs" / filename;
        
        // Log the path being accessed
        std::cout << "Loading viewer config from: " << configPath.string() << std::endl;
        
        if (!std::filesystem::exists(configPath)) {
            throw std::runtime_error("Configuration file not found: " + configPath.string());
        }
        
        // Load configuration from file
        nlohmann::json configJson = JsonLoader::loadFromFile(configPath.string());
        
        ViewerConfig config;
        
        // Load window settings
        if (configJson.contains("window")) {
            const auto& window = configJson["window"];
            if (window.contains("width")) config.windowWidth = window["width"];
            if (window.contains("height")) config.windowHeight = window["height"];
            if (window.contains("title")) config.windowTitle = window["title"];
        }
        
        // Load UI settings
        if (configJson.contains("ui")) {
            loadUIConfig(config.ui, configJson["ui"]);
        }
        
        // Load camera settings
        if (configJson.contains("camera")) {
            loadCameraConfig(config.camera, configJson["camera"]);
        }
        
        // Load rendering settings
        if (configJson.contains("rendering")) {
            loadRenderingConfig(config.rendering, configJson["rendering"]);
        } 
        
        // Load debug settings (for backward compatibility or additional features)
        if (configJson.contains("debug")) {
            const auto& debug = configJson["debug"];
            if (debug.contains("showBoundingBoxes")) {
                config.rendering.showBoundingBoxes = debug["showBoundingBoxes"];
            }
            
            // Other debug options could be loaded here
        }
        
        std::cout << "Viewer configuration loaded successfully." << std::endl;
        return config;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading viewer config: " << e.what() << std::endl;
        std::cerr << "Using default configuration." << std::endl;
        return ViewerConfig();
    }
}

// Function to save viewer configuration to JSON
void saveViewerConfigToJson(const ViewerConfig& config, const std::string& filename) {
    try {
        nlohmann::json configJson;
        
        // Window settings
        configJson["window"] = {
            {"width", config.windowWidth},
            {"height", config.windowHeight},
            {"title", config.windowTitle}
        };
        
        // UI settings
        configJson["ui"] = {
            {"uiScale", config.ui.uiScale},
            {"showFPS", config.ui.showFPS},
            {"fontPath", config.ui.fontPath}
        };
        
        // Camera settings
        configJson["camera"] = {
            {"fieldOfView", config.camera.fov},
            {"nearClipPlane", config.camera.nearPlane},
            {"farClipPlane", config.camera.farPlane},
            {"orbitRadius", config.camera.orbitRadius},
            {"orbitPitch", config.camera.orbitPitch},
            {"orbitYaw", config.camera.orbitYaw},
            {"target", {
                {"x", 0.0f},  // Add actual target if implemented
                {"y", 0.0f},
                {"z", 0.0f}
            }}
        };
        
        // Rendering settings
        configJson["rendering"] = {
            {"showGrid", config.rendering.showGrid},
            {"showAxes", config.rendering.showAxes},
            {"showBoundingBoxes", config.rendering.showBoundingBoxes},
            {"enableShadows", config.rendering.enableShadows},
            {"backgroundColor", {
                {"r", config.rendering.backgroundColor.r},
                {"g", config.rendering.backgroundColor.g},
                {"b", config.rendering.backgroundColor.b}
            }}
        };
        
        // Create configs directory if it doesn't exist
        std::filesystem::path configsDir = JsonLoader::getResourcesPath() / "configs";
        if (!std::filesystem::exists(configsDir)) {
            std::filesystem::create_directories(configsDir);
        }
        
        // Save the configuration file
        std::filesystem::path configPath = configsDir / filename;
        std::cout << "Saving viewer config to: " << configPath.string() << std::endl;
        JsonLoader::saveToFile(configJson, configPath.string(), true);
        std::cout << "Viewer configuration saved successfully." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error saving viewer config: " << e.what() << std::endl;
    }
}

} // namespace slrbs
