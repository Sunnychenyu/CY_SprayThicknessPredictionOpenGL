#include "PeriodicSectorReduction.h"

#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

namespace
{
    constexpr double kPi = 3.1415926535897932384626433832795;

    sprayworkpiece::WorkpieceModel makeCylinder(bool irregularTessellation)
    {
        constexpr std::size_t ringSamples = 16;
        sprayworkpiece::WorkpieceModel workpiece;
        for(std::size_t ring = 0; ring < 2; ++ring) {
            const double z = 0.2 * static_cast<double>(ring);
            for(std::size_t sample = 0; sample < ringSamples; ++sample) {
                const double uniformAngle = 2.0 * kPi * static_cast<double>(sample)
                    / static_cast<double>(ringSamples);
                const double angle = uniformAngle + (irregularTessellation
                    ? 0.035 * std::sin(1.7 * static_cast<double>(sample))
                    : 0.0);
                sprayworkpiece::SurfaceSample surface;
                surface.position = Eigen::Vector3d(
                    0.1 * std::cos(angle), 0.1 * std::sin(angle), z);
                surface.normal = Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);
                workpiece.samples.push_back(surface);
            }
        }
        for(std::size_t sample = 0; sample < ringSamples; ++sample) {
            const std::uint32_t next = static_cast<std::uint32_t>((sample + 1) % ringSamples);
            const std::uint32_t lower = static_cast<std::uint32_t>(sample);
            const std::uint32_t upper = static_cast<std::uint32_t>(ringSamples + sample);
            const std::uint32_t upperNext = static_cast<std::uint32_t>(ringSamples + next);
            workpiece.triangleIndices.insert(
                workpiece.triangleIndices.end(),
                { lower, next, upper, next, upperNext, upper });
        }
        return workpiece;
    }

    int verifyReduction(bool irregularTessellation, double mappingTolerance)
    {
        const sprayworkpiece::WorkpieceModel workpiece = makeCylinder(irregularTessellation);
        spraythickness::PeriodicLocalPredictionOptions options;
        options.enabled = true;
        options.axisOrigin = Eigen::Vector3d::Zero();
        options.axisDirection = Eigen::Vector3d::UnitZ();
        options.referenceDirection = Eigen::Vector3d::UnitX();
        options.sectorCount = 4;
        options.angularHaloRadians = kPi / 8.0;
        options.maximumMappingDistanceMeters = mappingTolerance;
        options.maximumNormalAngleDegrees = 5.0;

        const spraythickness::opengl::PeriodicSectorReduction preview =
            spraythickness::opengl::buildPeriodicSectorReduction(workpiece, options, false);
        if(!preview.localSelectionValid() || !preview.fullVertexBindings.empty()) {
            std::cerr << "Lightweight periodic preview contract failed.\n";
            return 1;
        }

        const spraythickness::opengl::PeriodicSectorReduction reduction =
            spraythickness::opengl::buildPeriodicSectorReduction(workpiece, options);
        if(!reduction.valid()) {
            std::cerr << reduction.failureReason << '\n';
            return 2;
        }
        if(reduction.predictionVertexIndices.size() >= workpiece.samples.size()) {
            std::cerr << "The periodic sector did not reduce the prediction vertex count.\n";
            return 3;
        }

        std::vector<float> localThickness(workpiece.samples.size(), 0.0f);
        for(const std::uint32_t vertex : reduction.predictionVertexIndices) {
            localThickness[vertex] = static_cast<float>(
                1.0 + workpiece.samples[vertex].position.z());
        }
        std::vector<float> expanded;
        spraythickness::opengl::expandPeriodicSectorThickness(
            reduction, localThickness, expanded);
        if(expanded.size() != workpiece.samples.size()) {
            return 4;
        }
        for(std::size_t vertex = 0; vertex < expanded.size(); ++vertex) {
            const double expected = 1.0 + workpiece.samples[vertex].position.z();
            if(std::abs(static_cast<double>(expanded[vertex]) - expected) > 1.0e-5) {
                std::cerr << "Expanded thickness mismatch at vertex " << vertex << '\n';
                return 5;
            }
        }
        return 0;
    }

    sprayworkpiece::SurfaceSample makeAngularSample(double degrees)
    {
        const double angle = degrees * kPi / 180.0;
        sprayworkpiece::SurfaceSample sample;
        sample.position = Eigen::Vector3d(std::cos(angle), std::sin(angle), 0.0);
        sample.normal = sample.position.normalized();
        return sample;
    }

    int verifyExactBoundarySelection()
    {
        sprayworkpiece::WorkpieceModel workpiece;
        workpiece.samples = {
            makeAngularSample(-60.0),
            makeAngularSample(60.0),
            makeAngularSample(120.0),
            makeAngularSample(100.0),
            makeAngularSample(140.0),
            makeAngularSample(160.0)
        };
        workpiece.triangleIndices = { 0, 1, 2, 3, 4, 5 };

        spraythickness::PeriodicLocalPredictionOptions options;
        options.enabled = true;
        options.axisOrigin = Eigen::Vector3d::Zero();
        options.axisDirection = Eigen::Vector3d::UnitZ();
        options.referenceDirection = Eigen::Vector3d::UnitX();
        options.sectorCount = 4;
        options.angularHaloRadians = kPi / 2.0;

        const spraythickness::opengl::PeriodicSectorReduction reduction =
            spraythickness::opengl::buildPeriodicSectorReduction(workpiece, options, false);
        if(!reduction.localSelectionValid()
            || reduction.predictionTriangleIndices.size() != 1
            || reduction.predictionTriangleIndices.front() != 0
            || reduction.predictionVertexIndices.size() != 3) {
            std::cerr << "Exact triangle-sector selection failed.\n";
            return 1;
        }
        return 0;
    }
}

int main()
{
    const int alignedResult = verifyReduction(false, 1.0e-6);
    if(alignedResult != 0) {
        return alignedResult;
    }
    const int irregularResult = verifyReduction(true, 1.0e-3);
    if(irregularResult != 0) {
        return 10 + irregularResult;
    }
    const int boundaryResult = verifyExactBoundarySelection();
    if(boundaryResult != 0) {
        return 20 + boundaryResult;
    }
    return 0;
}
