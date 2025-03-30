#pragma once

#include <string>
#include <glad/glad.h>
#include <glm/glm.hpp>

namespace slrbs {

/**
 * @brief Utility class for loading texture images
 */
class TextureLoader {
public:
    TextureLoader();
    ~TextureLoader();
    
    /**
     * @brief Load a texture from a file
     * @param path Path to the texture file
     * @param width Output parameter for texture width
     * @param height Output parameter for texture height
     * @param channels Output parameter for number of channels
     * @return OpenGL texture ID, or 0 if loading failed
     */
    GLuint loadTexture(const std::string& path, int& width, int& height, int& channels);
    
    /**
     * @brief Create a 1x1 solid color texture
     * @param color Color for the texture
     * @return OpenGL texture ID
     */
    GLuint createSolidColorTexture(const glm::vec4& color);
    
    /**
     * @brief Create a checkerboard pattern texture
     * @param width Texture width
     * @param height Texture height
     * @param color1 First color for checkerboard
     * @param color2 Second color for checkerboard
     * @return OpenGL texture ID
     */
    GLuint createCheckerboardTexture(int width, int height,
                                     const glm::vec4& color1 = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f),
                                     const glm::vec4& color2 = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
};

} // namespace slrbs
