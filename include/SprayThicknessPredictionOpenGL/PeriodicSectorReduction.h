#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace spraythickness::opengl
{
    struct PeriodicSectorBinding
    {
        std::array<std::uint32_t, 3> sourceVertexIndices{};
        std::array<float, 3> weights{};
    };

    struct PeriodicSectorReduction
    {
        std::vector<std::uint32_t> predictionVertexIndices;
        std::vector<std::uint32_t> predictionTriangleIndices;
        std::vector<PeriodicSectorBinding> fullVertexBindings;
        Eigen::Vector3d localBoundsCenter = Eigen::Vector3d::Zero();
        double localBoundsRadius{ 0.0 };
        double maximumMappingDistance{ 0.0 };
        std::size_t localTriangleCount{ 0 };
        bool mappingCacheHit{ false };
        std::string failureReason;

        bool localSelectionValid() const
        {
            return failureReason.empty()
                && !predictionVertexIndices.empty()
                && !predictionTriangleIndices.empty();
        }

        bool valid() const
        {
            return localSelectionValid()
                && !fullVertexBindings.empty();
        }
    };

    struct PeriodicSpraySample
    {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d direction = Eigen::Vector3d::UnitX();
        Eigen::Vector3d majorAxis = Eigen::Vector3d::UnitY();
        Eigen::Vector3d minorAxis = Eigen::Vector3d::UnitZ();
        double time = 0.0;
        double duration = 0.0;
    };

    PeriodicSectorReduction buildPeriodicSectorReduction(
        const sprayworkpiece::WorkpieceModel& workpiece,
        const PeriodicLocalPredictionOptions& options,
        bool buildFullVertexBindings = true);

    std::vector<PeriodicSpraySample> makePeriodicSpraySamples(
        const ThicknessPredictionTask& task);

    std::vector<std::size_t> selectPeriodicSpraySampleIndices(
        const std::vector<PeriodicSpraySample>& samples,
        const PeriodicSectorReduction& reduction,
        const AdvancedThicknessPredictionOptions& options);

    void expandPeriodicSectorThickness(
        const PeriodicSectorReduction& reduction,
        const std::vector<float>& localThicknessByFullVertex,
        std::vector<float>& expandedThickness);
}
