#include <SprayPathPlanning/SprayPathPlanning.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace sprayplanning
{
    namespace
    {
        bool sampleEnabled(
            const sprayworkpiece::SurfaceSample& sample,
            int regionId)
        {
            return sample.valid && (regionId < 0 || sample.regionId == regionId);
        }

        Eigen::Vector3d safeNormalized(
            const Eigen::Vector3d& value,
            const Eigen::Vector3d& fallback)
        {
            const double norm = value.norm();
            if (norm <= 1.0e-12)
                return fallback;
            return value / norm;
        }

        Eigen::Isometry3d makePose(
            const Eigen::Vector3d& position,
            const Eigen::Vector3d& sprayDirection,
            const Eigen::Vector3d& travelDirection)
        {
            const Eigen::Vector3d zAxis = safeNormalized(-sprayDirection, Eigen::Vector3d::UnitZ());
            Eigen::Vector3d xAxis = travelDirection - travelDirection.dot(zAxis) * zAxis;
            xAxis = safeNormalized(xAxis, Eigen::Vector3d::UnitX());
            Eigen::Vector3d yAxis = safeNormalized(zAxis.cross(xAxis), Eigen::Vector3d::UnitY());
            xAxis = safeNormalized(yAxis.cross(zAxis), Eigen::Vector3d::UnitX());

            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.linear().col(0) = xAxis;
            pose.linear().col(1) = yAxis;
            pose.linear().col(2) = zAxis;
            pose.translation() = position;
            return pose;
        }
    }

    SprayPlanningResult CoveragePathPlanner::planRasterPath(const SprayPlanningTask& task)
    {
        SprayPlanningResult result;
        result.trajectory.name = task.name.empty() ? "spray_raster_path" : task.name;

        if (task.workpiece.samples.empty())
        {
            result.warnings.push_back("Workpiece has no surface samples.");
            return result;
        }

        if (task.options.pathSpeed <= 0.0)
        {
            result.warnings.push_back("Path speed must be positive.");
            return result;
        }

        const double sprayWidth = task.process.sprayWidth > 0.0
            ? task.process.sprayWidth
            : task.tool.nozzle.pattern.width;
        if (sprayWidth <= 0.0)
        {
            result.warnings.push_back("Spray width must be positive.");
            return result;
        }

        Eigen::Vector3d minPoint(
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        Eigen::Vector3d maxPoint(
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest());
        Eigen::Vector3d normalSum = Eigen::Vector3d::Zero();
        size_t validCount = 0;

        for (const auto& sample : task.workpiece.samples)
        {
            if (!sampleEnabled(sample, task.options.regionId))
                continue;

            minPoint = minPoint.cwiseMin(sample.position);
            maxPoint = maxPoint.cwiseMax(sample.position);
            normalSum += safeNormalized(sample.normal, Eigen::Vector3d::UnitZ());
            ++validCount;
        }

        if (validCount == 0)
        {
            result.warnings.push_back("No valid samples found for requested region.");
            return result;
        }

        const Eigen::Vector3d normal = safeNormalized(normalSum, Eigen::Vector3d::UnitZ());
        const Eigen::Vector3d travelDirection = Eigen::Vector3d::UnitX();
        const Eigen::Vector3d rowDirection = Eigen::Vector3d::UnitY();

        const double overlap = std::clamp(task.options.overlapRatio, 0.0, 0.95);
        const double rowSpacing = sprayWidth * (1.0 - overlap);
        if (rowSpacing <= 0.0)
        {
            result.warnings.push_back("Computed row spacing is not positive.");
            return result;
        }

        const double x0 = minPoint.x() - task.options.margin;
        const double x1 = maxPoint.x() + task.options.margin;
        const double y0 = minPoint.y() - task.options.margin;
        const double y1 = maxPoint.y() + task.options.margin;
        const double z = 0.5 * (minPoint.z() + maxPoint.z());

        double currentTime = 0.0;
        int passIndex = 0;

        for (double y = y0; y <= y1 + 1.0e-9; y += rowSpacing)
        {
            const bool reverse = (passIndex % 2) != 0;
            const double startX = reverse ? x1 : x0;
            const double endX = reverse ? x0 : x1;
            const Eigen::Vector3d travel = reverse ? -travelDirection : travelDirection;

            spraytrajectory::SpraySegment segment;
            segment.processId = task.options.processId.empty() ? task.process.id : task.options.processId;
            segment.sprayEnabled = true;
            segment.passIndex = passIndex;

            const Eigen::Vector3d startSurface(startX, y, z);
            const Eigen::Vector3d endSurface(endX, y, z);
            const Eigen::Vector3d startTool = startSurface + normal * task.options.targetDistance;
            const Eigen::Vector3d endTool = endSurface + normal * task.options.targetDistance;

            const double length = (endTool - startTool).norm();
            const double duration = length / task.options.pathSpeed;

            spraytrajectory::SprayPathPoint startPoint;
            startPoint.time = currentTime;
            startPoint.tcpPose = makePose(startTool, -normal, travel);
            startPoint.sprayEnabled = true;
            startPoint.processId = segment.processId;
            startPoint.targetDistance = task.options.targetDistance;
            startPoint.targetNormal = normal;
            startPoint.workpieceRegionId = task.options.regionId;

            spraytrajectory::SprayPathPoint endPoint = startPoint;
            endPoint.time = currentTime + duration;
            endPoint.tcpPose = makePose(endTool, -normal, travel);

            segment.points.push_back(startPoint);
            segment.points.push_back(endPoint);
            result.trajectory.segments.push_back(segment);

            currentTime += duration;
            if (y + rowSpacing <= y1 + 1.0e-9)
            {
                currentTime += rowSpacing / task.options.pathSpeed;
            }
            ++passIndex;
        }

        result.success = !result.trajectory.empty();
        if (!result.success)
            result.warnings.push_back("Raster planner produced no trajectory.");
        return result;
    }
}
