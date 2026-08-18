#pragma once

#include <SprayTrajectoryCore/SprayTrajectory.h>

#include <filesystem>
#include <string>
#include <vector>

namespace spraytrajectory
{
    struct LegacySprayTrajectoryLoadOptions
    {
        double lengthScaleToMeters{ 1.0e-3 };
        std::string processId{ "paper_gaussian" };
    };

    struct LegacySprayTrajectoryLoadResult
    {
        SprayTrajectory trajectory;
        std::vector<std::string> warnings;
        bool success{ false };
    };

    class LegacySprayTrajectoryIo
    {
    public:
        static LegacySprayTrajectoryLoadResult loadMatrixText(
            const std::filesystem::path& path,
            const LegacySprayTrajectoryLoadOptions& options = {});
    };
}
