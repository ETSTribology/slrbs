#include "renderer/ShaderManager.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <glm/gtc/type_ptr.hpp>

namespace slrbs {

ShaderManager::ShaderManager() {
    // Nothing to initialize
}

ShaderManager::~ShaderManager() {
    // Clean up all shaders
    for (auto& shader : shaders) {
        glDeleteProgram(shader.second);
    }
}

GLuint ShaderManager::loadShader(const std::string& name, const std::string& vertexPath, const std::string& fragmentPath) {
    // Load shader source from files
    std::string vertexSource = loadShaderSource(vertexPath);
    std::string fragmentSource = loadShaderSource(fragmentPath);
    
    if (vertexSource.empty() || fragmentSource.empty()) {
        std::cerr << "Failed to load shader sources: " << name << std::endl;
        return 0;
    }
    
    // Create shader program
    GLuint programId = createProgram(vertexSource, fragmentSource);
    if (programId == 0) {
        return 0;
    }
    
    // Register shader
    registerShader(name, programId);
    
    return programId;
}

void ShaderManager::registerShader(const std::string& name, GLuint programId) {
    // If shader with this name already exists, delete old one
    auto it = shaders.find(name);
    if (it != shaders.end()) {
        glDeleteProgram(it->second);
    }
    
    shaders[name] = programId;
    
    // Setup shader variables
    ShaderVars vars;
    vars.program = nullptr; // We're not using QOpenGLShaderProgram here
    vars.mvMatrixLoc = glGetUniformLocation(programId, "model");
    vars.projMatrixLoc = glGetUniformLocation(programId, "projection");
    vars.normalMatrixLoc = glGetUniformLocation(programId, "normalMatrix");
    vars.vPositionLoc = glGetAttribLocation(programId, "aPos");
    vars.vNormalLoc = glGetAttribLocation(programId, "aNormal");
    vars.lPositionLoc = glGetUniformLocation(programId, "lightPos");
    vars.KdLoc = glGetUniformLocation(programId, "material.diffuse");
    vars.KsLoc = glGetUniformLocation(programId, "material.specular");
    vars.KnLoc = glGetUniformLocation(programId, "material.shininess");
    vars.lKdLoc = glGetUniformLocation(programId, "light.diffuse");
    vars.lKsLoc = glGetUniformLocation(programId, "light.specular");
    vars.lKaLoc = glGetUniformLocation(programId, "light.ambient");
    vars.useLightingLoc = glGetUniformLocation(programId, "useLighting");
    
    // Store for later use
    shaderVarsMap[name] = vars;
}

GLuint ShaderManager::getShader(const std::string& name) const {
    auto it = shaders.find(name);
    if (it != shaders.end()) {
        return it->second;
    }
    return 0;
}

void ShaderManager::useShader(const std::string& name) {
    GLuint shader = getShader(name);
    if (shader != 0) {
        glUseProgram(shader);
    } else {
        std::cerr << "Shader not found: " << name << std::endl;
    }
}

void ShaderManager::useShader(GLuint shaderId) {
    glUseProgram(shaderId);
}

void ShaderManager::setMat4(GLuint shaderId, const std::string& name, const glm::mat4& matrix) {
    GLint location = glGetUniformLocation(shaderId, name.c_str());
    if (location != -1) {
        glUniformMatrix4fv(location, 1, GL_FALSE, glm::value_ptr(matrix));
    }
}

void ShaderManager::setVec3(GLuint shaderId, const std::string& name, const glm::vec3& vector) {
    GLint location = glGetUniformLocation(shaderId, name.c_str());
    if (location != -1) {
        glUniform3fv(location, 1, glm::value_ptr(vector));
    }
}

void ShaderManager::setVec4(GLuint shaderId, const std::string& name, const glm::vec4& vector) {
    GLint location = glGetUniformLocation(shaderId, name.c_str());
    if (location != -1) {
        glUniform4fv(location, 1, glm::value_ptr(vector));
    }
}

void ShaderManager::setFloat(GLuint shaderId, const std::string& name, float value) {
    GLint location = glGetUniformLocation(shaderId, name.c_str());
    if (location != -1) {
        glUniform1f(location, value);
    }
}

void ShaderManager::setInt(GLuint shaderId, const std::string& name, int value) {
    GLint location = glGetUniformLocation(shaderId, name.c_str());
    if (location != -1) {
        glUniform1i(location, value);
    }
}

void ShaderManager::setBool(GLuint shaderId, const std::string& name, bool value) {
    GLint location = glGetUniformLocation(shaderId, name.c_str());
    if (location != -1) {
        glUniform1i(location, static_cast<int>(value));
    }
}

ShaderVars ShaderManager::getShaderVars(const std::string& name) {
    auto it = shaderVarsMap.find(name);
    if (it != shaderVarsMap.end()) {
        return it->second;
    }
    
    // Return empty shader vars if not found
    return ShaderVars();
}

GLuint ShaderManager::compileShader(const std::string& source, GLenum type) {
    GLuint shader = glCreateShader(type);
    const char* src = source.c_str();
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);
    
    // Check for compile errors
    GLint success;
    char infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::" << (type == GL_VERTEX_SHADER ? "VERTEX" : "FRAGMENT") 
                  << "::COMPILATION_FAILED\n" << infoLog << std::endl;
        glDeleteShader(shader);
        return 0;
    }
    
    return shader;
}

GLuint ShaderManager::createProgram(const std::string& vertexSource, const std::string& fragmentSource) {
    // Compile shaders
    GLuint vertexShader = compileShader(vertexSource, GL_VERTEX_SHADER);
    GLuint fragmentShader = compileShader(fragmentSource, GL_FRAGMENT_SHADER);
    
    if (vertexShader == 0 || fragmentShader == 0) {
        if (vertexShader != 0) glDeleteShader(vertexShader);
        if (fragmentShader != 0) glDeleteShader(fragmentShader);
        return 0;
    }
    
    // Link shaders
    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);
    
    // Check for linking errors
    GLint success;
    char infoLog[512];
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(program, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glDeleteProgram(program);
        return 0;
    }
    
    // Delete shaders as they're linked into our program now and no longer necessary
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    
    return program;
}

std::string ShaderManager::loadShaderSource(const std::string& filePath) {
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "ERROR::SHADER::FILE_NOT_FOUND: " << filePath << std::endl;
        return "";
    }
    
    std::stringstream stream;
    stream << file.rdbuf();
    file.close();
    
    return stream.str();
}

} // namespace slrbs
