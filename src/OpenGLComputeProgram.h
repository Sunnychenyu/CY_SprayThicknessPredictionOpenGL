#pragma once

#include <string>
#include <SprayThicknessPrediction/DepositionRuntime.h>

namespace spraythickness::opengl
{
    class OpenGLComputeProgram
    {
    public:
        OpenGLComputeProgram(const char* source, const char* debugName);
        ~OpenGLComputeProgram();

        OpenGLComputeProgram(const OpenGLComputeProgram&) = delete;
        OpenGLComputeProgram& operator=(const OpenGLComputeProgram&) = delete;

        void setCalibratedCurves(const CalibratedEfficiencyCurve& distance,const CalibratedEfficiencyCurve& angle) const;
        void setDensityCurves(const CalibratedEfficiencyCurve& distance,const CalibratedEfficiencyCurve& angle) const;
        void uploadCurve(const char* prefix,const CalibratedEfficiencyCurve& curve) const;
        unsigned int id() const { return m_program; }
        void use() const;
        void setInt(const char* name, int value) const;
        void setFloat(const char* name, float value) const;
        int uniformLocation(const char* name) const;

    private:
        mutable bool m_curvesUploaded=false,m_densityUploaded=false;
        mutable CalibratedEfficiencyCurve m_densityDistanceCurve,m_densityAngleCurve;
        mutable CalibratedEfficiencyCurve m_distanceCurve,m_angleCurve;
        unsigned int m_program{ 0 };
    };
}
