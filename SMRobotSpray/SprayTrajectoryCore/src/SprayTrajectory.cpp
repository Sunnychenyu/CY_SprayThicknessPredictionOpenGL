#include <SprayTrajectoryCore/SprayTrajectory.h>

#include <algorithm>

namespace spraytrajectory
{
    namespace
    {
        double clamp01(double value)
        {
            if (value < 0.0)
                return 0.0;
            if (value > 1.0)
                return 1.0;
            return value;
        }

        Eigen::Isometry3d interpolatePose(
            const Eigen::Isometry3d& a,
            const Eigen::Isometry3d& b,
            double t)
        {
            Eigen::Quaterniond qa(a.rotation());
            Eigen::Quaterniond qb(b.rotation());

            Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
            result.linear() = qa.slerp(t, qb).normalized().toRotationMatrix();
            result.translation() = a.translation() * (1.0 - t) + b.translation() * t;
            return result;
        }

        std::vector<double> interpolateJoints(
            const std::vector<double>& a,
            const std::vector<double>& b,
            double t)
        {
            if (a.size() != b.size())
                return t < 0.5 ? a : b;

            std::vector<double> value(a.size(), 0.0);
            for (size_t i = 0; i < a.size(); ++i)
                value[i] = a[i] * (1.0 - t) + b[i] * t;
            return value;
        }
    }

    bool SprayTrajectory::empty() const
    {
        return flattenedPoints().empty();
    }

    double SprayTrajectory::duration() const
    {
        const auto points = flattenedPoints();
        if (points.empty())
            return 0.0;
        return points.back().time - points.front().time;
    }

    std::vector<SprayPathPoint> SprayTrajectory::flattenedPoints() const
    {
        std::vector<SprayPathPoint> points;
        for (const auto& segment : segments)
        {
            for (auto point : segment.points)
            {
                if (point.processId.empty())
                    point.processId = segment.processId;
                point.sprayEnabled = point.sprayEnabled && segment.sprayEnabled;
                points.push_back(point);
            }
        }

        std::sort(points.begin(), points.end(),
            [](const SprayPathPoint& a, const SprayPathPoint& b)
            {
                return a.time < b.time;
            });
        return points;
    }

    SprayTrajectorySample SprayTrajectorySampler::evaluate(
        const SprayTrajectory& trajectory,
        double time)
    {
        const auto points = trajectory.flattenedPoints();
        if (points.empty())
            return {};

        if (time <= points.front().time)
        {
            const auto& point = points.front();
            return { point.time, point.tcpPose, point.jointValues, point.sprayEnabled,
                point.processId, point.targetDistance, point.targetNormal, point.workpieceRegionId };
        }

        if (time >= points.back().time)
        {
            const auto& point = points.back();
            return { point.time, point.tcpPose, point.jointValues, point.sprayEnabled,
                point.processId, point.targetDistance, point.targetNormal, point.workpieceRegionId };
        }

        auto next = std::lower_bound(
            points.begin(),
            points.end(),
            time,
            [](const SprayPathPoint& point, double value)
            {
                return point.time < value;
            });

        if (next == points.begin())
        {
            const auto& point = *next;
            return { point.time, point.tcpPose, point.jointValues, point.sprayEnabled,
                point.processId, point.targetDistance, point.targetNormal, point.workpieceRegionId };
        }

        const auto prev = next - 1;
        const double span = next->time - prev->time;
        const double alpha = span > 0.0 ? clamp01((time - prev->time) / span) : 0.0;

        SprayTrajectorySample sample;
        sample.time = time;
        sample.tcpPose = interpolatePose(prev->tcpPose, next->tcpPose, alpha);
        sample.jointValues = interpolateJoints(prev->jointValues, next->jointValues, alpha);
        sample.sprayEnabled = alpha < 0.5 ? prev->sprayEnabled : next->sprayEnabled;
        sample.processId = alpha < 0.5 ? prev->processId : next->processId;
        sample.targetDistance = prev->targetDistance * (1.0 - alpha) + next->targetDistance * alpha;
        Eigen::Vector3d targetNormal = prev->targetNormal * (1.0 - alpha) + next->targetNormal * alpha;
        sample.targetNormal = targetNormal.norm() > 1.0e-12 ? targetNormal.normalized() : Eigen::Vector3d::UnitZ();
        sample.workpieceRegionId = alpha < 0.5 ? prev->workpieceRegionId : next->workpieceRegionId;
        return sample;
    }

    std::vector<SprayTrajectorySample> SprayTrajectorySampler::sample(
        const SprayTrajectory& trajectory,
        double timeStep)
    {
        std::vector<SprayTrajectorySample> samples;
        const auto points = trajectory.flattenedPoints();
        if (points.empty() || timeStep <= 0.0)
            return samples;

        const double begin = points.front().time;
        const double end = points.back().time;
        for (double t = begin; t < end; t += timeStep)
            samples.push_back(evaluate(trajectory, t));

        samples.push_back(evaluate(trajectory, end));
        return samples;
    }
}
