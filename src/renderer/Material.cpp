#include "renderer/Material.h"
#include <polyscope/polyscope.h>
#include <iostream>

namespace slrbs {

Material::Material()
    : ambient(0.1f, 0.1f, 0.1f)
    , diffuse(0.7f, 0.7f, 0.7f)
    , specular(0.3f, 0.3f, 0.3f)
    , shininess(32.0f)
    , metallic(0.0f)
    , roughness(0.5f)
    , ao(1.0f)
    , diffuseMap("")
    , specularMap("")
    , normalMap("")
    , roughnessMap("")
    , aoMap("")
    , castShadows(true)
    , receiveShadows(true)
    , opacity(1.0f)
    , alphaBlending(false)
    , emission(0.0f, 0.0f, 0.0f)
    , emissionStrength(0.0f)
    , emissionMap("") {
}

Material::Material(const std::string& name) : name(name) {
}

Material::~Material() {
}

Material Material::createDefault() {
    Material material;
    material.ambient = glm::vec3(0.1f, 0.1f, 0.1f);
    material.diffuse = glm::vec3(0.7f, 0.7f, 0.7f);
    material.specular = glm::vec3(0.3f, 0.3f, 0.3f);
    material.shininess = 32.0f;
    material.metallic = 0.0f;
    material.roughness = 0.5f;
    return material;
}

Material Material::createMetal() {
    Material material;
    material.ambient = glm::vec3(0.25f, 0.25f, 0.25f);
    material.diffuse = glm::vec3(0.4f, 0.4f, 0.4f);
    material.specular = glm::vec3(0.774597f, 0.774597f, 0.774597f);
    material.shininess = 76.8f;
    material.metallic = 1.0f;
    material.roughness = 0.1f;
    return material;
}

Material Material::createPlastic() {
    Material material;
    material.ambient = glm::vec3(0.0f, 0.0f, 0.0f);
    material.diffuse = glm::vec3(0.55f, 0.55f, 0.55f);
    material.specular = glm::vec3(0.7f, 0.7f, 0.7f);
    material.shininess = 32.0f;
    material.metallic = 0.0f;
    material.roughness = 0.4f;
    return material;
}

Material Material::createGlass() {
    Material material;
    material.ambient = glm::vec3(0.0f, 0.0f, 0.0f);
    material.diffuse = glm::vec3(0.5f, 0.5f, 0.5f);
    material.specular = glm::vec3(0.95f, 0.95f, 0.95f);
    material.shininess = 128.0f;
    material.metallic = 0.0f;
    material.roughness = 0.05f;
    material.opacity = 0.2f;
    material.alphaBlending = true;
    return material;
}

Material Material::createEmissive() {
    Material material;
    material.ambient = glm::vec3(0.1f, 0.1f, 0.1f);
    material.diffuse = glm::vec3(0.5f, 0.5f, 0.5f);
    material.specular = glm::vec3(0.0f, 0.0f, 0.0f);
    material.shininess = 1.0f;
    material.emission = glm::vec3(1.0f, 0.7f, 0.3f); 
    material.emissionStrength = 1.0f;
    return material;
}

MaterialLibrary::MaterialLibrary() {
}

MaterialLibrary::~MaterialLibrary() {
}

void MaterialLibrary::initialize() {
    // Polyscope comes with built-in materials:
    // - clay (blendable)
    // - wax (blendable)
    // - candy (blendable)
    // - flat (blendable)
    // - mud (non-blendable)
    // - ceramic (non-blendable)
    // - jade (non-blendable)
    // - normal (non-blendable)
    
    // Add default materials
    addMaterial("clay", true);
    addMaterial("wax", true);
    addMaterial("candy", true);
    addMaterial("flat", true);
    addMaterial("mud", false);
    addMaterial("ceramic", false);
    addMaterial("jade", false);
    addMaterial("normal", false);
    
    std::cout << "MaterialLibrary initialized with default materials" << std::endl;
}

void MaterialLibrary::addMaterial(const std::string& name, bool blendable) {
    Material material(name);
    material.isBlendable = blendable;
    materials[name] = material;
}

void MaterialLibrary::registerBlendableMaterial(
    const std::string& name, const std::string& baseFilename, const std::string& extension) {
    
    try {
        // Construct filenames for the four basis materials
        std::array<std::string, 4> filenames = {
            baseFilename + "_r" + extension,
            baseFilename + "_g" + extension,
            baseFilename + "_b" + extension,
            baseFilename + "_k" + extension
        };
        
        // Register with Polyscope
        polyscope::loadBlendableMaterial(name, filenames);
        
        // Add to our library
        addMaterial(name, true);
        
        std::cout << "Registered blendable material: " << name << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to register blendable material: " << name 
                  << " - " << e.what() << std::endl;
    }
}

void MaterialLibrary::registerStaticMaterial(const std::string& name, const std::string& filename) {
    try {
        // Register with Polyscope
        polyscope::loadStaticMaterial(name, filename);
        
        // Add to our library
        addMaterial(name, false);
        
        std::cout << "Registered static material: " << name << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Failed to register static material: " << name 
                  << " - " << e.what() << std::endl;
    }
}

Material* MaterialLibrary::getMaterial(const std::string& name) {
    auto it = materials.find(name);
    if (it != materials.end()) {
        return &(it->second);
    }
    return nullptr;
}

} // namespace slrbs
