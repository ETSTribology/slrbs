#include "utils/TextureLoader.h"
#include <iostream>
#include <stb_image.h>
#include <glad/glad.h>

namespace slrbs {

TextureLoader::TextureLoader() {
    // Set default stbi configuration
    stbi_set_flip_vertically_on_load(true);
}

TextureLoader::~TextureLoader() {
    // Nothing to clean up
}

GLuint TextureLoader::loadTexture(const std::string& path, int& width, int& height, int& channels) {
    // Load image using stb_image
    unsigned char* data = stbi_load(path.c_str(), &width, &height, &channels, 0);
    
    if (!data) {
        std::cerr << "Failed to load texture: " << path << std::endl;
        std::cerr << "STBI error: " << stbi_failure_reason() << std::endl;
        return 0;
    }
    
    // Create OpenGL texture
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
    
    // Free image data
    stbi_image_free(data);
    
    return textureID;
}

GLuint TextureLoader::createSolidColorTexture(const glm::vec4& color) {
    // Create a 1x1 texture with the specified color
    unsigned char data[4] = {
        static_cast<unsigned char>(color.r * 255),
        static_cast<unsigned char>(color.g * 255),
        static_cast<unsigned char>(color.b * 255),
        static_cast<unsigned char>(color.a * 255)
    };
    
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    
    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    // Upload texture data
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    
    return textureID;
}

GLuint TextureLoader::createCheckerboardTexture(int width, int height, 
                                              const glm::vec4& color1, 
                                              const glm::vec4& color2) {
    // Create a checkerboard pattern texture
    unsigned char* data = new unsigned char[width * height * 4];
    
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            bool isColor1 = (x / 16 + y / 16) % 2 == 0;
            glm::vec4 color = isColor1 ? color1 : color2;
            
            int index = (y * width + x) * 4;
            data[index + 0] = static_cast<unsigned char>(color.r * 255);
            data[index + 1] = static_cast<unsigned char>(color.g * 255);
            data[index + 2] = static_cast<unsigned char>(color.b * 255);
            data[index + 3] = static_cast<unsigned char>(color.a * 255);
        }
    }
    
    GLuint textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    
    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    // Upload texture data
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
    
    delete[] data;
    
    return textureID;
}

} // namespace slrbs
