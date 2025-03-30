#pragma once

#include <string>
#include <unordered_map>
#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>

namespace slrbs {

struct Texture {
    GLuint id;
    std::string type;
    std::string path;
    int width;
    int height;
    int channels;
};

class TextureManager {
public:
    TextureManager();
    ~TextureManager();
    
    // Texture loading
    std::string loadTexture(const std::string& path);
    std::string loadTexture(const std::string& name, const std::string& path);
    std::string loadCubemap(const std::vector<std::string>& faces);
    
    // Texture management
    Texture* getTexture(const std::string& id);
    void bindTexture(const std::string& id, unsigned int textureUnit = 0);
    void unbindTexture(unsigned int textureUnit = 0);
    
    // Texture creation utilities
    std::string createSolidColorTexture(const glm::vec4& color);
    std::string createCheckerboardTexture(int width, int height, 
                                        const glm::vec4& color1 = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f),
                                        const glm::vec4& color2 = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
    
private:
    std::unordered_map<std::string, Texture> textures;
    std::string generateUniqueId();
    GLuint createTextureFromData(unsigned char* data, int width, int height, int channels);
};

} // namespace slrbs
