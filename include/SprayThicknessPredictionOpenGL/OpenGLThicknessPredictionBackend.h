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

        void beginOnline(ThicknessPredictionTask task);
        ThicknessPredictionResult appendOnline(
            const spraytrajectory::SprayTrajectory& trajectory);
        void appendOnline(const spraytrajectory::SprayTrajectory& trajectory,
            ThicknessPredictionResult& result);
        void appendOnline(const spraytrajectory::SprayTrajectory& trajectory,
            OnlineThicknessSnapshot& snapshot);
        void endOnline();

    private:
        void appendOnlineImpl(const spraytrajectory::SprayTrajectory& trajectory,
            ThicknessPredictionResult* result, OnlineThicknessSnapshot* snapshot);
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
