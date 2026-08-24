#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace spraythickness::opengl
{
    struct AxisymmetricProfilePoint
    {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        Eigen::Vector2d sectionPosition = Eigen::Vector2d::Zero();
    };

    struct AxisymmetricProfileContour
    {
        std::vector<AxisymmetricProfilePoint> points;
        bool closed{ false };
    };

    struct AxisymmetricProfileSlice
    {
        Eigen::Vector3d axisOrigin = Eigen::Vector3d::Zero();
        Eigen::Vector3d axisDirection = Eigen::Vector3d::UnitZ();
        Eigen::Vector3d radialDirection = Eigen::Vector3d::UnitX();
        std::vector<AxisymmetricProfileContour> contours;
        Eigen::Vector2d minimum = Eigen::Vector2d::Zero();
        Eigen::Vector2d maximum = Eigen::Vector2d::Zero();
        std::string failureReason;

        bool valid() const
        {
            return failureReason.empty() && !contours.empty();
        }
    };

    struct AxisymmetricProfileSelection
    {
        bool enabled{ false };
        Eigen::Vector2d minimum = Eigen::Vector2d::Zero();
        Eigen::Vector2d maximum = Eigen::Vector2d::Zero();
        // Closed section-space boundary. The legacy bounds remain populated
        // as its envelope for compatibility with existing callers.
        std::vector<Eigen::Vector2d> polygon;

        bool contains(const Eigen::Vector2d& point) const;
    };

    struct AxisymmetricProfileReduction
    {
        std::vector<sprayworkpiece::SurfaceSample> predictionSamples;
        std::vector<spraythickness::AxisymmetricProfileSampleSegment> sampleSegments;
        std::vector<spraythickness::AxisymmetricProfileBinding> fullVertexBindings;
        std::vector<AxisymmetricProfilePoint> displayLineSegments;
        std::size_t selectedContourPointCount{ 0 };
        std::size_t requestedSampleCount{ 0 };
        std::size_t actualSampleCount{ 0 };
        double selectedArcLengthMeters{ 0.0 };
        std::string failureReason;

        bool valid() const
        {
            return failureReason.empty() && !predictionSamples.empty()
                && !sampleSegments.empty();
        }
    };

    AxisymmetricProfileSlice buildAxisymmetricProfileSlice(
        const sprayworkpiece::WorkpieceModel& workpiece,
        const Eigen::Vector3d& axisOrigin,
        const Eigen::Vector3d& axisDirection,
        const Eigen::Vector3d& referenceDirection);

    AxisymmetricProfileReduction buildAxisymmetricProfileReduction(
        const AxisymmetricProfileSlice& slice,
        const AxisymmetricProfileSelection& selection,
        std::size_t sampleCount = 1024);
}
