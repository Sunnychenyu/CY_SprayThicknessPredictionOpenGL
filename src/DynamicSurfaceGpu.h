#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <SprayThicknessPredictionOpenGL/ThicknessBvh.h>
#include <array>
#include <memory>

namespace spraythickness::opengl
{
    // Uses the existing deposition buffers without owning them. All trial
    // geometry, checkpoints and acceptance checks remain on the GPU.
    class DynamicSurfaceGpu
    {
    public:
        static void prepareSurface(sprayworkpiece::WorkpieceModel& surface);
        DynamicSurfaceGpu(const sprayworkpiece::WorkpieceModel& surface,
            const DynamicSurfaceOptions& options, const ThicknessBvh& bvh,
            const std::array<unsigned int, 11>& buffers);
        ~DynamicSurfaceGpu();
        void checkpoint();
        bool tryAdvance();
        void rollback();
        double nextIntervalScale() const;
        void rebuildIfNeeded(const std::function<void(const ThicknessBvh&)>& upload);
        void bindDepositionBuffers() const;
        std::shared_ptr<DynamicSurfaceSnapshot> snapshot(double simulationSeconds,
            double elapsedSeconds, bool includeHistory = false) const;
    private:
        struct Impl;
        std::unique_ptr<Impl> m_impl;
    };
}
