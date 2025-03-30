#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <filesystem>

namespace slrbs {

/**
 * @brief Utility class for handling JSON files
 */
class JsonLoader {
public:
    /**
     * @brief Get the path to the resources directory
     * @return Path to resources directory
     */
    static std::filesystem::path getResourcesPath();
    
    /**
     * @brief Get the path to the textures directory
     * @return Path to textures directory
     */
    static std::filesystem::path getTexturesPath();
    
    /**
     * @brief Resolve a texture path to an absolute path
     * @param texturePath Relative or absolute path to texture
     * @return Resolved absolute path
     */
    static std::string resolveTexturePath(const std::string& texturePath);
    
    /**
     * @brief Load a JSON file and return its content
     * @param filename Path to the JSON file
     * @return JSON object read from the file
     * @throws std::runtime_error if file cannot be opened or parsed
     */
    static nlohmann::json loadFromFile(const std::string& filename);

    /**
     * @brief Save a JSON object to a file
     * @param data JSON object to save
     * @param filename Path to save the file
     * @param pretty Whether to format the JSON with indentation (pretty print)
     * @throws std::runtime_error if file cannot be opened or written
     */
    static void saveToFile(const nlohmann::json& data, const std::string& filename, bool pretty = true);
};

} // namespace slrbs
