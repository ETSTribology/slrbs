#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <array>
#include <glm/glm.hpp>

namespace slrbs {

/**
 * @class Material
 * @brief Represents a material used for rendering
 */
class Material {
public:
    Material(const std::string& name);
    ~Material();
    
    // Material properties
    std::string name;
    bool isBlendable = true;

    // Material properties
    glm::vec3 ambient;
    glm::vec3 diffuse;
    glm::vec3 specular;
    float shininess;
    
    // PBR properties
    float metallic;
    float roughness;
    float ao;
    
    // Texture maps
    std::string diffuseMap;
    std::string specularMap;
    std::string normalMap;
    std::string roughnessMap;
    std::string aoMap;
    
    // Shadowing properties
    bool castShadows;
    bool receiveShadows;
    
    // Alpha properties
    float opacity;
    bool alphaBlending;
    
    // Emission
    glm::vec3 emission;
    float emissionStrength;
    std::string emissionMap;
    
    // Set default materials
    static Material createDefault();
    static Material createMetal();
    static Material createPlastic();
    static Material createGlass();
    static Material createEmissive();
};

/**
 * @class MaterialLibrary
 * @brief Manages materials for the renderer
 */
class MaterialLibrary {
public:
    MaterialLibrary();
    ~MaterialLibrary();
    
    void initialize();
    
    // Add a new material to the library
    void addMaterial(const std::string& name, bool blendable = true);
    
    // Register materials with Polyscope
    void registerBlendableMaterial(const std::string& name, 
                                   const std::string& baseFilename, 
                                   const std::string& extension);
    
    void registerStaticMaterial(const std::string& name, 
                                const std::string& filename);
                                
    // Get a material by name
    Material* getMaterial(const std::string& name);
    
private:
    std::unordered_map<std::string, Material> materials;
};

} // namespace slrbs
