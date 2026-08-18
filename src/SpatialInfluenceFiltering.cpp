#include <SprayThicknessPredictionOpenGL/SpatialInfluenceFiltering.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace spraythickness::opengl
{
    namespace
    {
        constexpr double kPi = 3.1415926535897932384626433832795;
        constexpr std::size_t kMaximumGridAxis = 96;
        constexpr std::uint64_t kMaximumCandidatePairs = 64ull * 1024ull * 1024ull;
        constexpr double kMinimumManualCellSizeMeters = 1.0e-5;
        constexpr double kMaximumManualCellSizeMeters = 1.0;

        struct Bounds
        {
            Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
            Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
        };

        Bounds workpieceBounds(const sprayworkpiece::WorkpieceModel& workpiece)
        {
            Bounds bounds;
            bounds.minimum = Eigen::Vector3d::Constant(
                std::numeric_limits<double>::max());
            bounds.maximum = Eigen::Vector3d::Constant(
                std::numeric_limits<double>::lowest());
            for(const auto& sample : workpiece.samples) {
                bounds.minimum = bounds.minimum.cwiseMin(sample.position);
                bounds.maximum = bounds.maximum.cwiseMax(sample.position);
            }
            return bounds;
        }

        double coneSlope(
            const spraythickness::PaperGaussianParameters& deposition,
            double contributionCutoffRatio)
        {
            const double cutoff = std::clamp(
                contributionCutoffRatio,
                1.0e-12,
                0.999999);
            const double gaussianRadius = std::sqrt(-2.0 * std::log(cutoff));
            const double maximumSigma = std::max(
                std::abs(deposition.sigmaPhiRadians),
                std::abs(deposition.sigmaPsiRadians));
            const double maximumOffset = std::max(
                std::abs(deposition.phiOffsetRadians),
                std::abs(deposition.psiOffsetRadians));
            const double halfAngle = std::clamp(
                maximumOffset + gaussianRadius * maximumSigma,
                1.0e-6,
                1.5533430342749532);
            return std::tan(halfAngle);
        }

        std::size_t cellIndex(
            const Eigen::Vector3i& dimensions,
            const Eigen::Vector3i& coordinate)
        {
            return static_cast<std::size_t>(coordinate.x())
                + static_cast<std::size_t>(dimensions.x()) * (
                    static_cast<std::size_t>(coordinate.y())
                    + static_cast<std::size_t>(dimensions.y())
                        * static_cast<std::size_t>(coordinate.z()));
        }

        Eigen::Vector3i pointCell(
            const SpatialSprayGrid& grid,
            const Eigen::Vector3d& position)
        {
            Eigen::Vector3i coordinate = Eigen::Vector3i::Zero();
            for(int axis = 0; axis < 3; ++axis) {
                const double normalized = (position[axis] - grid.minimum[axis])
                    / std::max(grid.cellSize, 1.0e-12);
                coordinate[axis] = std::clamp(
                    static_cast<int>(std::floor(normalized)),
                    0,
                    grid.dimensions[axis] - 1);
            }
            return coordinate;
        }

        bool coneMayInfluenceSphere(
            const PeriodicSpraySample& sample,
            const Eigen::Vector3d& center,
            double radius,
            double slope)
        {
            const Eigen::Vector3d direction = sample.direction.normalized();
            if(direction.squaredNorm() <= 1.0e-12) {
                return false;
            }
            const Eigen::Vector3d offset = center - sample.position;
            const double axialDistance = offset.dot(direction);
            if(axialDistance + radius <= 0.0) {
                return false;
            }
            const Eigen::Vector3d lateral = offset
                - axialDistance * direction;
            const double allowedLateral = radius
                + std::max(0.0, axialDistance + radius) * slope;
            return lateral.norm() <= allowedLateral;
        }

        std::array<Eigen::Vector3d, 8> boundsCorners(const Bounds& bounds)
        {
            return {
                Eigen::Vector3d(bounds.minimum.x(), bounds.minimum.y(), bounds.minimum.z()),
                Eigen::Vector3d(bounds.maximum.x(), bounds.minimum.y(), bounds.minimum.z()),
                Eigen::Vector3d(bounds.minimum.x(), bounds.maximum.y(), bounds.minimum.z()),
                Eigen::Vector3d(bounds.maximum.x(), bounds.maximum.y(), bounds.minimum.z()),
                Eigen::Vector3d(bounds.minimum.x(), bounds.minimum.y(), bounds.maximum.z()),
                Eigen::Vector3d(bounds.maximum.x(), bounds.minimum.y(), bounds.maximum.z()),
                Eigen::Vector3d(bounds.minimum.x(), bounds.maximum.y(), bounds.maximum.z()),
                Eigen::Vector3d(bounds.maximum.x(), bounds.maximum.y(), bounds.maximum.z())
            };
        }

        void fail(SpatialSprayGrid& grid, const char* reason)
        {
            grid.failureReason = reason;
            grid.cellOffsets.clear();
            grid.candidateSprayIndices.clear();
            grid.vertexCellIndices.clear();
        }
    }

    bool finiteConeMayInfluencePoint(
        const PeriodicSpraySample& sample,
        const Eigen::Vector3d& vertexPosition,
        const spraythickness::PaperGaussianParameters& deposition,
        double contributionCutoffRatio)
    {
        const Eigen::Vector3d direction = sample.direction.normalized();
        if(direction.squaredNorm() <= 1.0e-12) {
            return false;
        }
        const Eigen::Vector3d offset = vertexPosition - sample.position;
        const double axialDistance = offset.dot(direction);
        if(axialDistance <= 1.0e-12) {
            return false;
        }
        const Eigen::Vector3d major = sample.majorAxis.normalized();
        const Eigen::Vector3d minor = sample.minorAxis.normalized();
        const double phi = std::atan2(offset.dot(major), axialDistance)
            - deposition.phiOffsetRadians;
        const double psi = std::atan2(offset.dot(minor), axialDistance)
            - deposition.psiOffsetRadians;
        const double cosine = std::cos(deposition.rotationRadians);
        const double sine = std::sin(deposition.rotationRadians);
        const double rotatedPhi = cosine * phi + sine * psi;
        const double rotatedPsi = -sine * phi + cosine * psi;
        const double normalizedPhi = rotatedPhi
            / std::max(std::abs(deposition.sigmaPhiRadians), 1.0e-12);
        const double normalizedPsi = rotatedPsi
            / std::max(std::abs(deposition.sigmaPsiRadians), 1.0e-12);
        const double pattern = std::exp(-0.5 * (
            normalizedPhi * normalizedPhi
            + normalizedPsi * normalizedPsi));
        return pattern >= std::clamp(
            contributionCutoffRatio,
            1.0e-12,
            0.999999);
    }

    SpatialSprayGrid buildSpatialSprayGrid(
        const sprayworkpiece::WorkpieceModel& workpiece,
        const std::vector<PeriodicSpraySample>& samples,
        const spraythickness::PaperGaussianParameters& deposition,
        double contributionCutoffRatio,
        double overrideCellSizeMeters,
        const SpatialGridProgress& progress)
    {
        SpatialSprayGrid grid;
        if(workpiece.samples.empty() || samples.empty()) {
            fail(grid, "Spatial filtering requires a non-empty model and trajectory.");
            return grid;
        }

        const Bounds bounds = workpieceBounds(workpiece);
        Eigen::Vector3d extent = bounds.maximum - bounds.minimum;
        for(int axis = 0; axis < 3; ++axis) {
            extent[axis] = std::max(extent[axis], 1.0e-9);
        }
        grid.minimum = bounds.minimum;
        grid.maximum = bounds.maximum;
        grid.contributionCutoffRatio = std::clamp(
            contributionCutoffRatio,
            1.0e-12,
            0.999999);
        grid.coneSlope = coneSlope(deposition, grid.contributionCutoffRatio);

        const auto corners = boundsCorners(bounds);
        double maximumAxialDistance = 0.0;
        for(const auto& sample : samples) {
            const Eigen::Vector3d direction = sample.direction.normalized();
            if(direction.squaredNorm() <= 1.0e-12) {
                continue;
            }
            for(const auto& corner : corners) {
                maximumAxialDistance = std::max(
                    maximumAxialDistance,
                    (corner - sample.position).dot(direction));
            }
        }
        const double maximumConeRadius = maximumAxialDistance * grid.coneSlope;
        const double maximumExtent = extent.maxCoeff();
        const double automaticCellSize = std::max(
            maximumExtent / 64.0,
            maximumConeRadius / 4.0);
        const double minimumRepresentableCellSize = maximumExtent
            / static_cast<double>(kMaximumGridAxis);
        grid.manualCellSize = std::isfinite(overrideCellSizeMeters)
            && overrideCellSizeMeters >= kMinimumManualCellSizeMeters
            && overrideCellSizeMeters <= kMaximumManualCellSizeMeters
            && overrideCellSizeMeters >= minimumRepresentableCellSize;
        grid.cellSize = grid.manualCellSize
            ? overrideCellSizeMeters
            : automaticCellSize;
        if(!std::isfinite(grid.cellSize) || grid.cellSize <= 0.0) {
            fail(grid, "Failed to determine a valid spatial grid cell size.");
            return grid;
        }
        for(int axis = 0; axis < 3; ++axis) {
            const std::size_t dimension = static_cast<std::size_t>(std::ceil(
                extent[axis] / grid.cellSize));
            grid.dimensions[axis] = static_cast<int>(std::clamp<std::size_t>(
                dimension,
                1,
                kMaximumGridAxis));
        }
        const std::size_t cells = grid.cellCount();
        std::vector<std::vector<std::uint32_t>> cellCandidates(cells);
        const double cellRadius = 0.5 * std::sqrt(3.0) * grid.cellSize;

        for(std::size_t sampleIndex = 0; sampleIndex < samples.size(); ++sampleIndex) {
            const PeriodicSpraySample& sample = samples[sampleIndex];
            const Eigen::Vector3d direction = sample.direction.normalized();
            if(direction.squaredNorm() <= 1.0e-12) {
                continue;
            }
            double tMax = 0.0;
            for(const auto& corner : corners) {
                tMax = std::max(tMax, (corner - sample.position).dot(direction));
            }
            if(tMax <= 0.0) {
                continue;
            }

            Eigen::Vector3d coneMinimum = sample.position;
            Eigen::Vector3d coneMaximum = sample.position;
            for(int axis = 0; axis < 3; ++axis) {
                const double perpendicular = std::sqrt(std::max(
                    0.0,
                    1.0 - direction[axis] * direction[axis]));
                const double axialExtent = std::abs(direction[axis]) * tMax;
                const double lateralExtent = perpendicular * grid.coneSlope * tMax;
                const double extentAlongAxis = axialExtent + lateralExtent;
                coneMinimum[axis] -= extentAlongAxis;
                coneMaximum[axis] += extentAlongAxis;
            }
            coneMinimum = coneMinimum.cwiseMax(bounds.minimum);
            coneMaximum = coneMaximum.cwiseMin(bounds.maximum);
            if((coneMaximum.array() < coneMinimum.array()).any()) {
                continue;
            }

            const Eigen::Vector3i minimumCell = pointCell(grid, coneMinimum);
            const Eigen::Vector3i maximumCell = pointCell(grid, coneMaximum);
            for(int z = minimumCell.z(); z <= maximumCell.z(); ++z) {
                for(int y = minimumCell.y(); y <= maximumCell.y(); ++y) {
                    for(int x = minimumCell.x(); x <= maximumCell.x(); ++x) {
                        const Eigen::Vector3i coordinate(x, y, z);
                        const Eigen::Vector3d center = grid.minimum
                            + (coordinate.cast<double>().array() + 0.5)
                                .matrix() * grid.cellSize;
                        if(!coneMayInfluenceSphere(
                            sample,
                            center,
                            cellRadius,
                            grid.coneSlope)) {
                            continue;
                        }
                        cellCandidates[cellIndex(grid.dimensions, coordinate)]
                            .push_back(static_cast<std::uint32_t>(sampleIndex));
                    }
                }
            }
            if(progress && (sampleIndex == 0 || sampleIndex + 1 == samples.size()
                || (sampleIndex + 1) % std::max<std::size_t>(1, samples.size() / 100) == 0)) {
                progress("assigning spray influence cells", sampleIndex + 1, samples.size());
            }
        }

        grid.cellOffsets.resize(cells + 1, 0U);
        std::uint64_t candidateCellPairs = 0;
        for(std::size_t cell = 0; cell < cells; ++cell) {
            candidateCellPairs += cellCandidates[cell].size();
            if(candidateCellPairs > kMaximumCandidatePairs
                || candidateCellPairs > std::numeric_limits<std::uint32_t>::max()) {
                fail(grid, "Spatial candidate grid exceeded the configured memory budget.");
                return grid;
            }
            grid.cellOffsets[cell + 1] = static_cast<std::uint32_t>(candidateCellPairs);
            grid.candidateCellPairs += cellCandidates[cell].size();
        }
        grid.candidateSprayIndices.reserve(grid.cellOffsets.back());
        for(const auto& candidates : cellCandidates) {
            grid.candidateSprayIndices.insert(
                grid.candidateSprayIndices.end(),
                candidates.begin(),
                candidates.end());
        }

        grid.vertexCellIndices.reserve(workpiece.samples.size());
        for(const auto& sample : workpiece.samples) {
            const Eigen::Vector3i coordinate = pointCell(grid, sample.position);
            const std::size_t cell = cellIndex(grid.dimensions, coordinate);
            grid.vertexCellIndices.push_back(static_cast<std::uint32_t>(cell));
            grid.candidateVertexPairs += grid.cellOffsets[cell + 1]
                - grid.cellOffsets[cell];
        }
        if(grid.vertexCellIndices.empty()) {
            fail(grid, "Spatial grid did not produce vertex cell bindings.");
            return grid;
        }
        return grid;
    }
}
