#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <memory>

namespace spraythickness::opengl
{
    class OpenGLThicknessPredictionBackend final : public IThicknessPredictionBackend
    {
    public:
        OpenGLThicknessPredictionBackend();
        ~OpenGLThicknessPredictionBackend() override;

        OpenGLThicknessPredictionBackend(const OpenGLThicknessPredictionBackend&) = delete;
        OpenGLThicknessPredictionBackend& operator=(const OpenGLThicknessPredictionBackend&) = delete;

        ThicknessPredictionResult predict(
            const ThicknessPredictionTask& task,
            const ThicknessPredictionExecution& execution = {}) override;

    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
