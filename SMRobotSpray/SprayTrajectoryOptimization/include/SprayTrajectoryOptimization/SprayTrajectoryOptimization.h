#pragma once

#include <SprayCore/SprayModel.h>
#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <SprayTrajectoryCore/SprayTrajectory.h>
#include <WorkpieceCore/WorkpieceModel.h>

#include <string>
#include <vector>

namespace sprayoptimization
{
    struct SprayTrajectoryEvaluation
    {
        spraythickness::ThicknessPredictionResult thickness;
        double cycleTime{ 0.0 };
        double score{ 0.0 };
        bool valid{ true };
        std::vector<std::string> warnings;
    };

    struct SprayEvaluationWeights
    {
        double meanAbsErrorWeight{ 1.0 };
        double maxAbsErrorWeight{ 1.0 };
        double cycleTimeWeight{ 0.0 };
    };

    struct SprayOptimizationOptions
    {
        spraythickness::ThicknessPredictionOptions thicknessOptions;
        SprayEvaluationWeights weights;
        std::vector<double> speedScaleCandidates{ 0.8, 1.0, 1.2 };
    };

    struct SprayOptimizationTask
    {
        sprayworkpiece::WorkpieceModel workpiece;
        spraycore::SprayTool tool;
        spraycore::SprayProcess process;
        spraytrajectory::SprayTrajectory initialTrajectory;
        SprayOptimizationOptions options;
    };

    struct SprayOptimizationResult
    {
        spraytrajectory::SprayTrajectory trajectory;
        SprayTrajectoryEvaluation evaluation;
        bool improved{ false };
        std::vector<std::string> warnings;
    };

    class SprayTrajectoryEvaluator
    {
    public:
        static SprayTrajectoryEvaluation evaluate(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const spraycore::SprayTool& tool,
            const spraycore::SprayProcess& process,
            const spraytrajectory::SprayTrajectory& trajectory,
            const SprayOptimizationOptions& options = {});
    };

    class SprayTrajectoryOptimizer
    {
    public:
        static SprayOptimizationResult optimizeSpeedScale(
            const SprayOptimizationTask& task);
    };
}
