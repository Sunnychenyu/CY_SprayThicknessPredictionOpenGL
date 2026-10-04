#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <memory>
#include <array>

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
        // Worker-thread update for subsequent batches; retains thickness,
        // thermal history and all resident geometry buffers.
        void setOnlineToolDirections(const Eigen::Vector3d& sprayDirectionLocal,
            const Eigen::Vector3d& powderFeedDirectionLocal);
        std::function<double(const Eigen::Vector3d&)> onlineSurfaceDistanceQuery() const;
        // Copy static visibility geometry once, without CPU readback. Caller
        // owns the destination buffers and synchronizes before publication.
        void copyOnlineVisibilityGeometry(const std::array<unsigned int, 4>& buffers);
        // An empty trajectory requests an explicit CPU snapshot without adding
        // deposition (for export/verification), also after appendOnlineGpu().
        ThicknessPredictionResult appendOnline(
            const spraytrajectory::SprayTrajectory& trajectory);
        void appendOnline(const spraytrajectory::SprayTrajectory& trajectory,
            ThicknessPredictionResult& result);
        void appendOnline(const spraytrajectory::SprayTrajectory& trajectory,
            OnlineThicknessSnapshot& snapshot);
        // Upload once per model. Indices flatten the display submeshes in order.
        void setOnlineDisplayMapping(const std::vector<std::uint32_t>& sampleIndices);
        // Caller owns a free shared-context display buffer. A reused buffer must
        // have no retained consumers and its last draw fence must be signaled.
        // Returns only after GPU completion; snapshot contains statistics, no field array.
        // With fullStatistics=false only the current range/count are reduced;
        // the other metrics retain the last complete statistics snapshot.
        void appendOnlineGpu(const spraytrajectory::SprayTrajectory& trajectory,
            unsigned int displayBuffer, OnlineThicknessSnapshot& snapshot,
            const std::function<bool()>& canceled = {}, bool fullStatistics = true);
        void endOnline();

    private:
        void appendOnlineImpl(const spraytrajectory::SprayTrajectory& trajectory,
            ThicknessPredictionResult* result, OnlineThicknessSnapshot* snapshot,
            unsigned int displayBuffer = 0, const std::function<bool()>& canceled = {},
            bool fullStatistics = true);
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
