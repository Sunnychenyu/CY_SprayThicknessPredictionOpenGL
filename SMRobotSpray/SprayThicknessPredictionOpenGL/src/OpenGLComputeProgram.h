#pragma once

#include <string>

namespace spraythickness::opengl
{
    class OpenGLComputeProgram
    {
    public:
        OpenGLComputeProgram(const char* source, const char* debugName);
        ~OpenGLComputeProgram();

        OpenGLComputeProgram(const OpenGLComputeProgram&) = delete;
        OpenGLComputeProgram& operator=(const OpenGLComputeProgram&) = delete;

        unsigned int id() const { return m_program; }
        void use() const;
        void setInt(const char* name, int value) const;
        void setFloat(const char* name, float value) const;
        int uniformLocation(const char* name) const;

    private:
        unsigned int m_program{ 0 };
    };
}
