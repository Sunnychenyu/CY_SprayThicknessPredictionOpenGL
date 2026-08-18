#include <SprayCore/SprayModel.h>

#include <algorithm>
#include <cmath>

namespace spraycore
{
    namespace
    {
        double safeNorm(const Eigen::Vector3d& value)
        {
            const double norm = value.norm();
            return norm > 1.0e-12 ? norm : 0.0;
        }
    }

    double SprayPattern::evaluate(double lateralDistance) const
    {
        const double halfWidth = width * 0.5;
        if (halfWidth <= 0.0)
            return 0.0;

        const double x = std::abs(lateralDistance);
        if (x > halfWidth)
            return 0.0;

        switch (type)
        {
        case SprayPatternType::Uniform:
            return 1.0;
        case SprayPatternType::Linear:
            return std::max(0.0, 1.0 - x / halfWidth);
        case SprayPatternType::Gaussian:
        default:
        {
            const double sigma = std::max(halfWidth / std::max(falloff, 1.0e-6), 1.0e-9);
            return std::exp(-(x * x) / (2.0 * sigma * sigma));
        }
        }
    }

    bool SprayTool::isDistanceValid(double distance) const
    {
        return distance >= effectiveDistanceMin && distance <= effectiveDistanceMax;
    }

    Eigen::Vector3d SprayTool::worldSprayDirection(const Eigen::Isometry3d& toolPose) const
    {
        Eigen::Vector3d direction = toolPose.linear() * sprayDirectionLocal;
        const double norm = safeNorm(direction);
        if (norm == 0.0)
            return -Eigen::Vector3d::UnitZ();
        return direction / norm;
    }

    double SprayKernel::evaluateDepositRate(
        const SprayTool& tool,
        const SprayProcess& process,
        const Eigen::Isometry3d& toolPose,
        const Eigen::Vector3d& samplePosition,
        const Eigen::Vector3d& sampleNormal)
    {
        const Eigen::Vector3d sprayDirection = tool.worldSprayDirection(toolPose);
        const Eigen::Vector3d toSample = samplePosition - toolPose.translation();
        const double axialDistance = toSample.dot(sprayDirection);

        if (!tool.isDistanceValid(axialDistance))
            return 0.0;

        const Eigen::Vector3d axial = sprayDirection * axialDistance;
        const double lateralDistance = (toSample - axial).norm();

        SprayPattern pattern = tool.nozzle.pattern;
        if (process.sprayWidth > 0.0)
            pattern.width = process.sprayWidth;

        const double lateralWeight = pattern.evaluate(lateralDistance);
        if (lateralWeight <= 0.0)
            return 0.0;

        Eigen::Vector3d normal = sampleNormal;
        const double normalNorm = safeNorm(normal);
        if (normalNorm == 0.0)
            return 0.0;
        normal /= normalNorm;

        const double angleWeight = std::max(0.0, normal.dot(-sprayDirection));
        return process.flowRate * process.atomizationFactor * process.materialScale
            * lateralWeight * angleWeight;
    }
}
