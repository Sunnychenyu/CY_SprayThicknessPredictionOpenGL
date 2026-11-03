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

    void OpenGLComputeProgram::setCalibratedCurves(const CalibratedEfficiencyCurve& distance,const CalibratedEfficiencyCurve& angle) const {
        if(m_curvesUploaded && m_distanceCurve==distance && m_angleCurve==angle)return;
        uploadCurve("distance",distance);uploadCurve("angle",angle);
        m_distanceCurve=distance;m_angleCurve=angle;m_curvesUploaded=true;
    }

    void OpenGLComputeProgram::uploadCurve(const char* prefix,const CalibratedEfficiencyCurve& curve) const {
            if(curve.x.size()>128 || curve.x.size()!=curve.y.size())
                throw std::invalid_argument("Efficiency interpolation requires at most 128 matching nodes.");
            const std::string name(prefix);
            setInt((name+"CurveKind").c_str(),static_cast<int>(curve.kind));
            setFloat((name+"CurveLowerRampEnd").c_str(),float(curve.lowerRampEnd));
            setInt((name+"CurveCount").c_str(),static_cast<int>(curve.x.size()));
            float parameters[6]{};
            for(std::size_t i=0;i<curve.parameters.size() && i<6;++i)parameters[i]=float(curve.parameters[i]);
            glUniform1fv(uniformLocation((name+"CurveParameters[0]").c_str()),6,parameters);
            std::vector<float> nodes;nodes.reserve(curve.x.size()*2);
            for(std::size_t i=0;i<curve.x.size();++i){nodes.push_back(float(curve.x[i]));nodes.push_back(float(curve.y[i]));}
            if(!nodes.empty())glUniform2fv(uniformLocation((name+"CurveNodes[0]").c_str()),int(curve.x.size()),nodes.data());
        }
    void OpenGLComputeProgram::setDensityCurves(const CalibratedEfficiencyCurve& distance,const CalibratedEfficiencyCurve& angle) const {
        if(distance.x.size()>32 || angle.x.size()>32)throw std::invalid_argument("GPU density interpolation supports at most 32 nodes per curve.");
        if(m_densityUploaded && m_densityDistanceCurve==distance && m_densityAngleCurve==angle)return;
        uploadCurve("densityDistance",distance);uploadCurve("densityAngle",angle);
        m_densityDistanceCurve=distance;m_densityAngleCurve=angle;m_densityUploaded=true;
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
