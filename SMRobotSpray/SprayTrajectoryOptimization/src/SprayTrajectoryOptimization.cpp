#include <SprayTrajectoryOptimization/SprayTrajectoryOptimization.h>

#include <algorithm>
#include <cmath>

namespace sprayoptimization
{
    namespace
    {
        double meanAbsError(const spraythickness::ThicknessField& field)
        {
            if (field.results.empty())
                return 0.0;

            double sum = 0.0;
            for (const auto& result : field.results)
                sum += std::abs(result.error);

            return sum / static_cast<double>(field.results.size());
        }

        spraytrajectory::SprayTrajectory scaleTrajectoryTime(
            const spraytrajectory::SprayTrajectory& trajectory,
            double speedScale)
        {
            if (speedScale <= 0.0)
                return trajectory;

            spraytrajectory::SprayTrajectory scaled = trajectory;
            for (auto& segment : scaled.segments)
            {
                for (auto& point : segment.points)
                    point.time /= speedScale;
            }
            return scaled;
        }
    }

    SprayTrajectoryEvaluation SprayTrajectoryEvaluator::evaluate(
        const sprayworkpiece::WorkpieceModel& workpiece,
        const spraycore::SprayTool& tool,
        const spraycore::SprayProcess& process,
        const spraytrajectory::SprayTrajectory& trajectory,
        const SprayOptimizationOptions& options)
    {
        SprayTrajectoryEvaluation evaluation;
        evaluation.thickness = spraythickness::SprayThicknessPredictor::predict(
            workpiece,
            tool,
            process,
            trajectory,
            options.thicknessOptions);

        evaluation.cycleTime = trajectory.duration();

        const double meanAbs = meanAbsError(evaluation.thickness.field);
        evaluation.score =
            options.weights.meanAbsErrorWeight * meanAbs
            + options.weights.maxAbsErrorWeight * evaluation.thickness.metrics.maxAbsError
            + options.weights.cycleTimeWeight * evaluation.cycleTime;

        evaluation.warnings = evaluation.thickness.warnings;
        evaluation.valid = evaluation.warnings.empty();
        return evaluation;
    }

    SprayOptimizationResult SprayTrajectoryOptimizer::optimizeSpeedScale(
        const SprayOptimizationTask& task)
    {
        SprayOptimizationResult result;
        result.trajectory = task.initialTrajectory;
        result.evaluation = SprayTrajectoryEvaluator::evaluate(
            task.workpiece,
            task.tool,
            task.process,
            task.initialTrajectory,
            task.options);

        if (task.initialTrajectory.empty())
        {
            result.warnings.push_back("Initial spray trajectory is empty.");
            return result;
        }

        SprayTrajectoryEvaluation bestEvaluation = result.evaluation;
        spraytrajectory::SprayTrajectory bestTrajectory = task.initialTrajectory;

        for (double speedScale : task.options.speedScaleCandidates)
        {
            if (speedScale <= 0.0)
            {
                result.warnings.push_back("Skipped non-positive speed scale candidate.");
                continue;
            }

            spraytrajectory::SprayTrajectory candidate = scaleTrajectoryTime(
                task.initialTrajectory,
                speedScale);

            SprayTrajectoryEvaluation candidateEvaluation = SprayTrajectoryEvaluator::evaluate(
                task.workpiece,
                task.tool,
                task.process,
                candidate,
                task.options);

            if (candidateEvaluation.score < bestEvaluation.score)
            {
                bestEvaluation = candidateEvaluation;
                bestTrajectory = candidate;
            }
        }

        result.improved = bestEvaluation.score < result.evaluation.score;
        result.trajectory = bestTrajectory;
        result.evaluation = bestEvaluation;
        return result;
    }
}
