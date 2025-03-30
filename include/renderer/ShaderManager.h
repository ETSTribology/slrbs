#pragma once

#include <string>
#include <unordered_map>
#include <glad/glad.h>
#include <glm/glm.hpp>
#include "renderer/ShaderVars.h"

namespace slrbs {

class ShaderManager {
public:
    ShaderManager();
    ~ShaderManager();
    
    // Shader loading
    GLuint loadShader(const std::string& name, 
                     const std::string& vertexPath, 
                     const std::string& fragmentPath);
                     
    // Shader registration
    void registerShader(const std::string& name, GLuint programId);
    
    // Shader usage
    GLuint getShader(const std::string& name) const;
    void useShader(const std::string& name);
    void useShader(GLuint shaderId);
    
    // Uniform setters
    void setMat4(GLuint shaderId, const std::string& name, const glm::mat4& matrix);
    void setVec3(GLuint shaderId, const std::string& name, const glm::vec3& vector);
    void setVec4(GLuint shaderId, const std::string& name, const glm::vec4& vector);
    void setFloat(GLuint shaderId, const std::string& name, float value);
    void setInt(GLuint shaderId, const std::string& name, int value);
    void setBool(GLuint shaderId, const std::string& name, bool value);
    
    // Shader variables handling
    ShaderVars getShaderVars(const std::string& name);
    
private:
    std::unordered_map<std::string, GLuint> shaders;
    std::unordered_map<std::string, ShaderVars> shaderVarsMap;
    
    // Shader compilation
    GLuint compileShader(const std::string& source, GLenum type);
    GLuint createProgram(const std::string& vertexSource, const std::string& fragmentSource);
    std::string loadShaderSource(const std::string& filePath);
};

} // namespace slrbs
