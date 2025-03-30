#include "utils/JsonLoader.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <filesystem>

namespace slrbs {

std::filesystem::path JsonLoader::getResourcesPath() {
    // Try different paths to find resources directory
    std::vector<std::filesystem::path> candidates = {
        "./resources",                          // Running from project root
        "../resources",                         // Running from build dir
        "../../resources",                      // Running from nested build dir
        "../../../resources",                   // Running from deeper nested build dir
    };
    
    for (const auto& path : candidates) {
        if (std::filesystem::exists(path) && std::filesystem::is_directory(path)) {
            return std::filesystem::absolute(path);
        }
    }
    
    // Fallback - return the most likely path even if it doesn't exist
    std::cerr << "Warning: Resources directory not found. Using default path." << std::endl;
    return "./resources";
}

std::filesystem::path JsonLoader::getTexturesPath() {
    return getResourcesPath() / "textures";
}

std::string JsonLoader::resolveTexturePath(const std::string& texturePath) {
    std::filesystem::path path(texturePath);
    
    // If it's already an absolute path, use it directly
    if (path.is_absolute()) {
        return texturePath;
    }
    
    // Try relative to current directory
    if (std::filesystem::exists(path)) {
        return path.string();
    }
    
    // Try in textures directory
    std::filesystem::path texturesDir = getTexturesPath();
    std::filesystem::path resolvedPath = texturesDir / path;
    
    if (std::filesystem::exists(resolvedPath)) {
        return resolvedPath.string();
    }
    
    // Return original path if not found (will be handled as error later)
    return texturePath;
}

nlohmann::json JsonLoader::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
    }
    
    nlohmann::json data;
    try {
        file >> data;
    } catch (const nlohmann::json::parse_error& e) {
        throw std::runtime_error("Failed to parse JSON: " + std::string(e.what()));
    }
    
    return data;
}

void JsonLoader::saveToFile(const nlohmann::json& data, const std::string& filename, bool pretty) {
    // Create directory if it doesn't exist
    std::filesystem::path filePath(filename);
    if (!filePath.parent_path().empty()) {
        std::filesystem::create_directories(filePath.parent_path());
    }
    
    std::ofstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + filename);
    }
    
    if (pretty) {
        file << data.dump(4); // 4 spaces indentation
    } else {
        file << data;
    }
}

} // namespace slrbs
