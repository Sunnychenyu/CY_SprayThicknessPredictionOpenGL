#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace sprayworkpiece
{
    struct SurfaceSample
    {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
        double areaWeight{ 1.0 };
        int regionId{ -1 };
        double targetThickness{ 0.0 };
        bool valid{ true };
    };

    struct SurfaceRegion
    {
        int id{ -1 };
        std::string name;
        double targetThickness{ 0.0 };
        bool enabled{ true };
    };

    struct TargetThicknessProfile
    {
        double defaultThickness{ 0.0 };
        std::vector<SurfaceRegion> regions;
    };

    struct WorkpieceSamplingOptions
    {
        double sampleSpacing{ 0.01 };
        bool useAreaWeight{ true };
    };

    class WorkpieceModel
    {
    public:
        std::string name;
        std::string sourceMeshPath;
        std::vector<SurfaceSample> samples;
        std::vector<SurfaceRegion> regions;

        bool empty() const;
        size_t sampleCount() const;

        void addRegion(const SurfaceRegion& region);
        void addSample(const SurfaceSample& sample);
        const SurfaceRegion* findRegion(int regionId) const;
        void applyRegionTargetThickness();
    };
}
