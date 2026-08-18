#pragma once

#include <SprayThicknessPredictionOpenGL/PeriodicSectorReduction.h>

#include <functional>

namespace spraythickness::opengl
{
    struct PeriodicGpuMappingCallbacks
    {
        std::function<void(std::size_t current, std::size_t total, const char* phase)>
            progress;
        std::function<bool()> canceled;
    };

    PeriodicSectorReduction buildPeriodicSectorReductionGpu(
        const sprayworkpiece::WorkpieceModel& workpiece,
        const PeriodicLocalPredictionOptions& options,
        const PeriodicGpuMappingCallbacks& callbacks = {});
}
