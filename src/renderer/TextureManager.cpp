#include "renderer/TextureManager.h"
#include "utils/TextureLoader.h"
#include <iostream>
#include <random>
#include <chrono>
#include <filesystem>

namespace slrbs {

TextureManager::TextureManager() {
    // Initialize random generator for unique IDs
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
}

TextureManager::~TextureManager() {
    // Clean up all textures
    for (auto& pair : textures) {
        if (pair.second.id != 0) {
            glDeleteTextures(1, &pair.second.id);
        }
    }
    textures.clear();
}

std::string TextureManager::loadTexture(const std::string& path) {
    // Generate a unique name based on the path
    std::filesystem::path filePath(path);
    std::string name = filePath.stem().string();
    return loadTexture(name, path);
}

std::string TextureManager::loadTexture(const std::string& name, const std::string& path) {
    // Check if texture with this name already exists
    for (const auto& pair : textures) {
        if (pair.second.path == path) {
            return pair.first;
        }
    }
    
    // Load texture
    TextureLoader loader;
    int width, height, channels;
    GLuint textureID = loader.loadTexture(path, width, height, channels);
    
    if (textureID == 0) {
        std::cerr << "Failed to load texture: " << path << std::endl;
        return "";
    }
    
    // Store texture info
    std::string id = name;
    if (textures.find(id) != textures.end()) {
        // If name already exists, make it unique
        id = name + "_" + generateUniqueId();
    }
    
    Texture texture;
    texture.id = textureID;
    texture.path = path;
    texture.type = "diffuse"; // Default type
    texture.width = width;
    texture.height = height;
    texture.channels = channels;
    
    textures[id] = texture;
    
    return id;
}

std::string TextureManager::loadCubemap(const std::vector<std::string>& faces) {
    if (faces.size() != 6) {
        std::cerr << "Cubemap requires exactly 6 face textures" << std::endl;
        return "";
    }
    
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);
    
    TextureLoader loader;
    int width, height, channels;
    
    for (unsigned int i = 0; i < faces.size(); i++) {
        int faceWidth, faceHeight, faceChannels;
        GLuint faceTexture = loader.loadTexture(faces[i], faceWidth, faceHeight, faceChannels);
        
        if (faceTexture == 0) {
            std::cerr << "Failed to load cubemap face: " << faces[i] << std::endl;
            glDeleteTextures(1, &textureID);
            return "";
        }
        
        // For first face, set width/height/channels
        if (i == 0) {
            width = faceWidth;
            height = faceHeight;
            channels = faceChannels;
        }
        
        // Clean up face texture - we only need its data which is now uploaded
        glDeleteTextures(1, &faceTexture);
    }
    
    // Set texture parameters
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    
    // Create unique ID
    std::string id = "cubemap_" + generateUniqueId();
    
    // Store texture info
    Texture texture;
    texture.id = textureID;
    texture.type = "cubemap";
    texture.width = width;
    texture.height = height;
    texture.channels = channels;
    
    textures[id] = texture;
    
    return id;
}

Texture* TextureManager::getTexture(const std::string& id) {
    auto it = textures.find(id);
    if (it != textures.end()) {
        return &it->second;
    }
    return nullptr;
}

void TextureManager::bindTexture(const std::string& id, unsigned int textureUnit) {
    auto texture = getTexture(id);
    if (!texture) {
        std::cerr << "Texture not found: " << id << std::endl;
        return;
    }
    
    glActiveTexture(GL_TEXTURE0 + textureUnit);
    
    if (texture->type == "cubemap") {
        glBindTexture(GL_TEXTURE_CUBE_MAP, texture->id);
    } else {
        glBindTexture(GL_TEXTURE_2D, texture->id);
    }
}

void TextureManager::unbindTexture(unsigned int textureUnit) {
    glActiveTexture(GL_TEXTURE0 + textureUnit);
    glBindTexture(GL_TEXTURE_2D, 0);
}

std::string TextureManager::createSolidColorTexture(const glm::vec4& color) {
    TextureLoader loader;
    GLuint textureID = loader.createSolidColorTexture(color);
    
    std::string id = "color_" + generateUniqueId();
    
    Texture texture;
    texture.id = textureID;
    texture.type = "color";
    texture.width = 1;
    texture.height = 1;
    texture.channels = 4;
    
    textures[id] = texture;
    
    return id;
}

std::string TextureManager::createCheckerboardTexture(int width, int height,
                                                    const glm::vec4& color1,
                                                    const glm::vec4& color2) {
    TextureLoader loader;
    GLuint textureID = loader.createCheckerboardTexture(width, height, color1, color2);
    
    std::string id = "checker_" + generateUniqueId();
    
    Texture texture;
    texture.id = textureID;
    texture.type = "checker";
    texture.width = width;
    texture.height = height;
    texture.channels = 4;
    
    textures[id] = texture;
    
    return id;
}

std::string TextureManager::generateUniqueId() {
    // Generate a random unique ID
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<> dis(0, 999999);
    
    return std::to_string(dis(gen));
}

GLuint TextureManager::createTextureFromData(unsigned char* data, int width, int height, int channels) {
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    
    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    // Upload texture data to GPU
    GLenum format = GL_RGB;
    if (channels == 1)
        format = GL_RED;
    else if (channels == 3)
        format = GL_RGB;
    else if (channels == 4)
        format = GL_RGBA;
    
    glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    
    return textureID;
}

} // namespace slrbs
