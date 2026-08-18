#pragma once

#include <RobotTrajectoryCore/RobotTrajectory.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace spraytrajectory
{
    struct SprayProcessState
    {
        std::string processId;
        bool sprayEnabled{ false };
    };

    struct SprayPathPoint
    {
        double time{ 0.0 };
        Eigen::Isometry3d tcpPose = Eigen::Isometry3d::Identity();
        std::vector<double> jointValues;
        bool sprayEnabled{ false };
        std::string processId;
        double targetDistance{ 0.0 };
        Eigen::Vector3d targetNormal = Eigen::Vector3d::UnitZ();
        int workpieceRegionId{ -1 };
    };

    struct SpraySegment
    {
        std::vector<SprayPathPoint> points;
        std::string processId;
        bool sprayEnabled{ true };
        int passIndex{ 0 };
    };

    class SprayTrajectory
    {
    public:
        std::string name;
        robottrajectory::CartesianTrajectory baseCartesianTrajectory;
        std::vector<SpraySegment> segments;

        bool empty() const;
        double duration() const;
        std::vector<SprayPathPoint> flattenedPoints() const;
    };

    struct SprayTrajectorySample
    {
        double time{ 0.0 };
        Eigen::Isometry3d tcpPose = Eigen::Isometry3d::Identity();
        std::vector<double> jointValues;
        bool sprayEnabled{ false };
        std::string processId;
        double targetDistance{ 0.0 };
        Eigen::Vector3d targetNormal = Eigen::Vector3d::UnitZ();
        int workpieceRegionId{ -1 };
    };

    class SprayTrajectorySampler
    {
    public:
        static std::vector<SprayTrajectorySample> sample(
            const SprayTrajectory& trajectory,
            double timeStep);

        static SprayTrajectorySample evaluate(
            const SprayTrajectory& trajectory,
            double time);
    };
}
