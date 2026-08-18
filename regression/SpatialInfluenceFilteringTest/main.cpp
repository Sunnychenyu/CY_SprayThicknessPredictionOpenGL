#include <SprayThicknessPredictionOpenGL/SpatialInfluenceFiltering.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <vector>

namespace
{
    spraythickness::opengl::PeriodicSpraySample makeSpray(
        double x,
        double y,
        double time)
    {
        spraythickness::opengl::PeriodicSpraySample sample;
        sample.position = Eigen::Vector3d(x, y, 0.3);
        sample.direction = Eigen::Vector3d(0.0, 0.0, -1.0);
        sample.majorAxis = Eigen::Vector3d::UnitX();
        sample.minorAxis = Eigen::Vector3d::UnitY();
        sample.time = time;
        sample.duration = 0.02;
        return sample;
    }

    sprayworkpiece::WorkpieceModel makeWorkpiece()
    {
        sprayworkpiece::WorkpieceModel workpiece;
        for(int y = 0; y < 5; ++y) {
            for(int x = 0; x < 5; ++x) {
                sprayworkpiece::SurfaceSample sample;
                sample.position = Eigen::Vector3d(
                    -0.1 + 0.05 * static_cast<double>(x),
                    -0.1 + 0.05 * static_cast<double>(y),
                    0.0);
                sample.normal = Eigen::Vector3d::UnitZ();
                workpiece.samples.push_back(sample);
            }
        }
        workpiece.triangleIndices = { 0, 1, 5 };
        return workpiece;
    }

    bool verifyCandidates(
        const spraythickness::opengl::SpatialSprayGrid& grid,
        const sprayworkpiece::WorkpieceModel& workpiece,
        const std::vector<spraythickness::opengl::PeriodicSpraySample>& sprays,
        const spraythickness::PaperGaussianParameters& deposition,
        double cutoff)
    {
        for(std::size_t vertex = 0; vertex < workpiece.samples.size(); ++vertex) {
            const std::size_t cell = grid.vertexCellIndices[vertex];
            const auto begin = grid.cellOffsets[cell];
            const auto end = grid.cellOffsets[cell + 1];
            std::vector<std::uint32_t> candidates(
                grid.candidateSprayIndices.begin() + begin,
                grid.candidateSprayIndices.begin() + end);
            if(!std::is_sorted(candidates.begin(), candidates.end())) {
                std::cerr << "Candidate spray indices are not time ordered.\n";
                return false;
            }
            for(std::size_t spray = 0; spray < sprays.size(); ++spray) {
                const bool hasExactInfluence =
                    spraythickness::opengl::finiteConeMayInfluencePoint(
                        sprays[spray],
                        workpiece.samples[vertex].position,
                        deposition,
                        cutoff);
                const bool isCandidate = std::find(
                    candidates.begin(),
                    candidates.end(),
                    static_cast<std::uint32_t>(spray)) != candidates.end();
                if(hasExactInfluence && !isCandidate) {
                    std::cerr << "Spatial grid dropped an influencing spray point.\n";
                    return false;
                }
            }
        }
        return true;
    }

    bool sameDimensions(
        const Eigen::Vector3i& first,
        const Eigen::Vector3i& second)
    {
        return (first.array() == second.array()).all();
    }
}

int main()
{
    const sprayworkpiece::WorkpieceModel workpiece = makeWorkpiece();
    const std::vector<spraythickness::opengl::PeriodicSpraySample> sprays = {
        makeSpray(0.0, 0.0, 0.0),
        makeSpray(0.04, 0.0, 0.02),
        makeSpray(1.0, 1.0, 0.04)
    };
    spraythickness::PaperGaussianParameters deposition;
    deposition.sigmaPhiRadians = 0.20;
    deposition.sigmaPsiRadians = 0.20;
    const double cutoff = 1.0e-4;
    const spraythickness::opengl::SpatialSprayGrid grid =
        spraythickness::opengl::buildSpatialSprayGrid(
            workpiece,
            sprays,
            deposition,
            cutoff);
    if(!grid.valid()) {
        std::cerr << grid.failureReason << '\n';
        return 1;
    }
    const double maximumExtent = 0.2;
    const double maximumAxialDistance = 0.3;
    const double gaussianRadius = std::sqrt(-2.0 * std::log(cutoff));
    const double maximumSigma = std::max(
        std::abs(deposition.sigmaPhiRadians),
        std::abs(deposition.sigmaPsiRadians));
    const double maximumOffset = std::max(
        std::abs(deposition.phiOffsetRadians),
        std::abs(deposition.psiOffsetRadians));
    const double coneHalfAngle = std::clamp(
        maximumOffset + gaussianRadius * maximumSigma,
        1.0e-6,
        1.5533430342749532);
    const double maximumConeRadius = maximumAxialDistance
        * std::tan(coneHalfAngle);
    const double expectedAutomaticCellSize = std::max(
        maximumExtent / 64.0,
        maximumConeRadius / 4.0);
    if(std::abs(grid.cellSize - expectedAutomaticCellSize) > 1.0e-12) {
        std::cerr << "Automatic spatial grid formula changed unexpectedly.\n";
        return 2;
    }

    if(grid.manualCellSize || !verifyCandidates(
        grid, workpiece, sprays, deposition, cutoff)) {
        return 3;
    }
    if(grid.candidateVertexPairs == 0
        || grid.candidateVertexPairs >= workpiece.samples.size() * sprays.size()) {
        std::cerr << "The test case did not reduce candidate pairs.\n";
        return 4;
    }

    const double manualCellSizeMeters = 0.04;
    const spraythickness::opengl::SpatialSprayGrid manualGrid =
        spraythickness::opengl::buildSpatialSprayGrid(
            workpiece,
            sprays,
            deposition,
            cutoff,
            manualCellSizeMeters);
    if(!manualGrid.valid() || !manualGrid.manualCellSize
        || std::abs(manualGrid.cellSize - manualCellSizeMeters) > 1.0e-12
        || sameDimensions(manualGrid.dimensions, grid.dimensions)
        || !verifyCandidates(manualGrid, workpiece, sprays, deposition, cutoff)) {
        std::cerr << "Manual spatial grid cell size was not applied correctly.\n";
        return 5;
    }

    const double invalidSizes[] = { 0.0, -0.04, 1.0e-6, 2.0,
        std::numeric_limits<double>::quiet_NaN() };
    for(const double invalidSize : invalidSizes) {
        const spraythickness::opengl::SpatialSprayGrid invalidGrid =
            spraythickness::opengl::buildSpatialSprayGrid(
                workpiece,
                sprays,
                deposition,
                cutoff,
                invalidSize);
        if(!invalidGrid.valid() || invalidGrid.manualCellSize
            || std::abs(invalidGrid.cellSize - grid.cellSize) > 1.0e-12
            || !sameDimensions(invalidGrid.dimensions, grid.dimensions)) {
            std::cerr << "Invalid manual grid cell size did not fall back to automatic mode.\n";
            return 6;
        }
    }
    return 0;
}
