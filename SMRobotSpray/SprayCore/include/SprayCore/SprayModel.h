#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

namespace spraycore
{
    enum class SprayPatternType
    {
        Uniform,
        Linear,
        Gaussian
    };

    struct SprayPattern
    {
        SprayPatternType type{ SprayPatternType::Gaussian };
        double width{ 0.1 };
        double falloff{ 1.0 };

        double evaluate(double lateralDistance) const;
    };

    struct SprayNozzle
    {
        std::string name;
        SprayPattern pattern;
    };

    struct SprayTool
    {
        std::string name;
        std::string mountLink;
        Eigen::Isometry3d T_link_tool = Eigen::Isometry3d::Identity();
        Eigen::Vector3d sprayDirectionLocal = -Eigen::Vector3d::UnitZ();
        double effectiveDistanceMin{ 0.05 };
        double effectiveDistanceMax{ 0.5 };
        SprayNozzle nozzle;

        bool isDistanceValid(double distance) const;
        Eigen::Vector3d worldSprayDirection(const Eigen::Isometry3d& toolPose) const;
    };

    struct SprayProcess
    {
        std::string id;
        std::string name;
        double flowRate{ 1.0 };
        double sprayWidth{ 0.1 };
        double atomizationFactor{ 1.0 };
        double materialScale{ 1.0 };
        double defaultSpeed{ 0.1 };
    };

    class SprayKernel
    {
    public:
        static double evaluateDepositRate(
            const SprayTool& tool,
            const SprayProcess& process,
            const Eigen::Isometry3d& toolPose,
            const Eigen::Vector3d& samplePosition,
            const Eigen::Vector3d& sampleNormal);
    };
}
