#include "OpenGLComputeProgram.h"

#include <glad/glad.h>

#include <algorithm>
#include <stdexcept>
#include <string>

namespace spraythickness::opengl
{
    namespace
    {
        std::string shaderLog(unsigned int shader)
        {
            int length = 0;
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &length);
            std::string log(static_cast<std::size_t>(std::max(length, 1)), '\0');
            glGetShaderInfoLog(shader, length, nullptr, log.data());
            return log;
        }

        std::string programLog(unsigned int program)
        {
            int length = 0;
            glGetProgramiv(program, GL_INFO_LOG_LENGTH, &length);
            std::string log(static_cast<std::size_t>(std::max(length, 1)), '\0');
            glGetProgramInfoLog(program, length, nullptr, log.data());
            return log;
        }
    }

    OpenGLComputeProgram::OpenGLComputeProgram(const char* source, const char* debugName)
    {
        const unsigned int shader = glCreateShader(GL_COMPUTE_SHADER);
        glShaderSource(shader, 1, &source, nullptr);
        glCompileShader(shader);
        int compiled = 0;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
        if(compiled == 0) {
            const std::string log = shaderLog(shader);
            glDeleteShader(shader);
            throw std::runtime_error(std::string(debugName) + " compile failed: " + log);
        }

        m_program = glCreateProgram();
        glAttachShader(m_program, shader);
        glLinkProgram(m_program);
        glDeleteShader(shader);
        int linked = 0;
        glGetProgramiv(m_program, GL_LINK_STATUS, &linked);
        if(linked == 0) {
            const std::string log = programLog(m_program);
            glDeleteProgram(m_program);
            m_program = 0;
            throw std::runtime_error(std::string(debugName) + " link failed: " + log);
        }
    }

    OpenGLComputeProgram::~OpenGLComputeProgram()
    {
        if(m_program != 0) {
            glDeleteProgram(m_program);
        }
    }

    void OpenGLComputeProgram::use() const
    {
        glUseProgram(m_program);
    }

    void OpenGLComputeProgram::setInt(const char* name, int value) const
    {
        glUniform1i(glGetUniformLocation(m_program, name), value);
    }

    void OpenGLComputeProgram::setFloat(const char* name, float value) const
    {
        glUniform1f(glGetUniformLocation(m_program, name), value);
    }

    int OpenGLComputeProgram::uniformLocation(const char* name) const
    {
        return glGetUniformLocation(m_program, name);
    }
}
