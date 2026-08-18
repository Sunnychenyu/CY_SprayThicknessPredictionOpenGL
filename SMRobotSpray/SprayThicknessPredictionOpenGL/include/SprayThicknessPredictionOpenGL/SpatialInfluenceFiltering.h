#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <SprayThicknessPredictionOpenGL/PeriodicSectorReduction.h>

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace spraythickness::opengl
{
    struct SpatialSprayGrid
    {
        Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
        Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
        Eigen::Vector3i dimensions = Eigen::Vector3i::Ones();
        double cellSize{ 0.0 };
        bool manualCellSize{ false };
        double coneSlope{ 0.0 };
        double contributionCutoffRatio{ 1.0e-6 };
        std::vector<std::uint32_t> cellOffsets;
        std::vector<std::uint32_t> candidateSprayIndices;
        std::vector<std::uint32_t> vertexCellIndices;
        std::size_t candidateCellPairs{ 0 };
        std::size_t candidateVertexPairs{ 0 };
        std::string failureReason;

        bool valid() const
        {
            return failureReason.empty()
                && dimensions.x() > 0
                && dimensions.y() > 0
                && dimensions.z() > 0
                && cellOffsets.size() == cellCount() + 1
                && !vertexCellIndices.empty();
        }

        std::size_t cellCount() const
        {
            return static_cast<std::size_t>(dimensions.x())
                * static_cast<std::size_t>(dimensions.y())
                * static_cast<std::size_t>(dimensions.z());
        }
    };

    using SpatialGridProgress = std::function<void(
        const char* phase,
        std::size_t current,
        std::size_t total)>;

    SpatialSprayGrid buildSpatialSprayGrid(
        const sprayworkpiece::WorkpieceModel& workpiece,
        const std::vector<PeriodicSpraySample>& samples,
        const spraythickness::PaperGaussianParameters& deposition,
        double contributionCutoffRatio,
        double overrideCellSizeMeters = 0.0,
        const SpatialGridProgress& progress = {});

    bool finiteConeMayInfluencePoint(
        const PeriodicSpraySample& sample,
        const Eigen::Vector3d& vertexPosition,
        const spraythickness::PaperGaussianParameters& deposition,
        double contributionCutoffRatio);
}
