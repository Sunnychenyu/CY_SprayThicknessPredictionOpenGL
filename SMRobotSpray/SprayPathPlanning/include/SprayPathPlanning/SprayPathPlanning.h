#pragma once

#include <SprayCore/SprayModel.h>
#include <SprayTrajectoryCore/SprayTrajectory.h>
#include <WorkpieceCore/WorkpieceModel.h>

#include <string>
#include <vector>

namespace sprayplanning
{
    struct SprayPlanningOptions
    {
        double overlapRatio{ 0.5 };
        double targetDistance{ 0.25 };
        double pathSpeed{ 0.1 };
        double margin{ 0.05 };
        int regionId{ -1 };
        std::string processId;
    };

    struct SprayPlanningTask
    {
        std::string name;
        sprayworkpiece::WorkpieceModel workpiece;
        spraycore::SprayTool tool;
        spraycore::SprayProcess process;
        SprayPlanningOptions options;
    };

    struct SprayPlanningResult
    {
        spraytrajectory::SprayTrajectory trajectory;
        std::vector<std::string> warnings;
        bool success{ false };
    };

    class CoveragePathPlanner
    {
    public:
        static SprayPlanningResult planRasterPath(const SprayPlanningTask& task);
    };
}
