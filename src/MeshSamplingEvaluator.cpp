#include <SprayThicknessPredictionOpenGL/MeshSamplingEvaluator.h>
#include <SprayThicknessPredictionOpenGL/PeriodicSectorReduction.h>
#include <SprayThicknessPredictionOpenGL/ThicknessBvh.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace spraythickness::opengl
{
    namespace
    {
        using Vector = Eigen::Vector3d;
        using Clock = std::chrono::steady_clock;

        void checkCancelled(const ThicknessPredictionExecution& execution)
        {
            if(execution.cancelRequested && execution.cancelRequested->load()) {
                throw std::runtime_error("Mesh sampling assessment cancelled.");
            }
        }

        struct PositionKey
        {
            std::array<std::int64_t, 3> coordinates;
            bool operator==(const PositionKey& other) const
            {
                return coordinates == other.coordinates;
            }
        };

        struct PositionHash
        {
            std::size_t operator()(const PositionKey& key) const
            {
                std::size_t hash = 0;
                for(auto coordinate : key.coordinates) {
                    hash ^= std::hash<std::int64_t>{}(coordinate)
                        + 0x9e3779b9U + (hash << 6) + (hash >> 2);
                }
                return hash;
            }
        };

        struct RayHit
        {
            double distance = std::numeric_limits<double>::infinity();
            Vector normal = Vector::Zero();
            bool found = false;
        };

        Vector nodeMinimum(const GpuBvhNode& node)
        {
            const Vector value(node.minimum[0], node.minimum[1], node.minimum[2]);
            return value * 0.001 - Vector::Constant(
                value.cwiseAbs().maxCoeff() * 2.0e-10 + 1.0e-10);
        }

        Vector nodeMaximum(const GpuBvhNode& node)
        {
            const Vector value(node.maximum[0], node.maximum[1], node.maximum[2]);
            return value * 0.001 + Vector::Constant(
                value.cwiseAbs().maxCoeff() * 2.0e-10 + 1.0e-10);
        }

        bool rayIntersectsBounds(const Vector& origin, const Vector& direction,
            const GpuBvhNode& node, double maximumDistance)
        {
            double nearDistance = 0.0;
            double farDistance = maximumDistance;
            const Vector minimum = nodeMinimum(node);
            const Vector maximum = nodeMaximum(node);
            for(int axis = 0; axis < 3; ++axis) {
                if(std::abs(direction[axis]) < 1.0e-15) {
                    if(origin[axis] < minimum[axis] || origin[axis] > maximum[axis]) return false;
                    continue;
                }
                const double first = (minimum[axis] - origin[axis]) / direction[axis];
                const double second = (maximum[axis] - origin[axis]) / direction[axis];
                nearDistance = std::max(nearDistance, std::min(first, second));
                farDistance = std::min(farDistance, std::max(first, second));
                if(nearDistance > farDistance) return false;
            }
            return true;
        }

        // Retain the complete substrate BVH even for restricted prediction lists.
        class SurfaceQuery
        {
        public:
            SurfaceQuery(const sprayworkpiece::WorkpieceModel& workpiece,
                const ThicknessBvh& bvh, const ThicknessPredictionExecution& execution)
                : m_workpiece(workpiece), m_bvh(bvh), m_execution(execution)
            {
                m_stack.reserve(128);
            }

            std::array<std::uint32_t, 3> indices(std::size_t triangle) const
            {
                const auto& list = m_workpiece.triangleIndices;
                return { list[triangle * 3], list[triangle * 3 + 1], list[triangle * 3 + 2] };
            }

            RayHit firstHit(const Vector& origin, const Vector& direction,
                double maximumDistance = std::numeric_limits<double>::infinity())
            {
                RayHit hit;
                hit.distance = maximumDistance;
                m_stack.clear();
                m_stack.push_back(0);
                while(!m_stack.empty()) {
                    if((++m_visits & 4095U) == 0) checkCancelled(m_execution);
                    const auto& node = m_bvh.nodes[m_stack.back()];
                    m_stack.pop_back();
                    if(!rayIntersectsBounds(origin, direction, node, hit.distance)) continue;
                    if(node.triangleCount > 0) {
                        for(int index = 0; index < node.triangleCount; ++index) {
                            intersectTriangle(origin, direction,
                                m_bvh.triangleOrder[node.firstTriangle + index], hit);
                        }
                    } else {
                        m_stack.push_back(node.leftChild);
                        m_stack.push_back(node.rightChild);
                    }
                }
                return hit;
            }

        private:
            void intersectTriangle(const Vector& origin, const Vector& direction,
                std::size_t triangle, RayHit& hit) const
            {
                const auto ids = indices(triangle);
                const auto& a = m_workpiece.samples[ids[0]];
                const auto& b = m_workpiece.samples[ids[1]];
                const auto& c = m_workpiece.samples[ids[2]];
                const Vector edge1 = b.position - a.position;
                const Vector edge2 = c.position - a.position;
                const Vector p = direction.cross(edge2);
                const double determinant = edge1.dot(p);
                if(std::abs(determinant) <= 1.0e-12 * edge1.norm() * edge2.norm()) return;
                const Vector offset = origin - a.position;
                const double u = offset.dot(p) / determinant;
                const Vector q = offset.cross(edge1);
                const double v = direction.dot(q) / determinant;
                constexpr double tolerance = 1.0e-9;
                if(u < -tolerance || v < -tolerance || u + v > 1.0 + tolerance) return;
                const double distance = edge2.dot(q) / determinant;
                if(distance <= 1.0e-10 || distance > hit.distance) return;
                hit.found = true;
                hit.distance = distance;
                hit.normal = a.normal * (1.0 - u - v) + b.normal * u + c.normal * v;
                if(hit.normal.squaredNorm() < 1.0e-20) hit.normal = edge1.cross(edge2);
            }

            const sprayworkpiece::WorkpieceModel& m_workpiece;
            const ThicknessBvh& m_bvh;
            const ThicknessPredictionExecution& m_execution;
            std::vector<std::int32_t> m_stack;
            std::size_t m_visits{ 0 };
        };

        class CoreFootprint
        {
        public:
            CoreFootprint(const PaperGaussianParameters& deposition, double ratio)
                : m_parameters(deposition), m_radiusSquared(-2.0 * std::log(ratio)),
                  m_cos(std::cos(deposition.rotationRadians)),
                  m_sin(std::sin(deposition.rotationRadians))
            {
                const double phiExtent = std::sqrt(m_radiusSquared *
                    (std::pow(m_cos * deposition.sigmaPhiRadians, 2)
                        + std::pow(m_sin * deposition.sigmaPsiRadians, 2)));
                const double psiExtent = std::sqrt(m_radiusSquared *
                    (std::pow(m_sin * deposition.sigmaPhiRadians, 2)
                        + std::pow(m_cos * deposition.sigmaPsiRadians, 2)));
                const double phiMin = deposition.phiOffsetRadians - phiExtent;
                const double phiMax = deposition.phiOffsetRadians + phiExtent;
                const double psiMin = deposition.psiOffsetRadians - psiExtent;
                const double psiMax = deposition.psiOffsetRadians + psiExtent;
                valid = deposition.sigmaPhiRadians > 0.0 && deposition.sigmaPsiRadians > 0.0
                    && std::isfinite(phiMin) && std::isfinite(phiMax)
                    && std::isfinite(psiMin) && std::isfinite(psiMax)
                    && std::max({ std::abs(phiMin), std::abs(phiMax),
                        std::abs(psiMin), std::abs(psiMax) }) < 1.55;
                m_planes = { Vector(0, 0, 1), Vector(1, 0, -std::tan(phiMin)),
                    Vector(-1, 0, std::tan(phiMax)), Vector(0, 1, -std::tan(psiMin)),
                    Vector(0, -1, std::tan(psiMax)) };
            }

            Eigen::Vector2d coordinates(const Eigen::Vector2d& slopes) const
            {
                const double phi = std::atan(slopes.x()) - m_parameters.phiOffsetRadians;
                const double psi = std::atan(slopes.y()) - m_parameters.psiOffsetRadians;
                return { (m_cos * phi + m_sin * psi) / m_parameters.sigmaPhiRadians,
                    (-m_sin * phi + m_cos * psi) / m_parameters.sigmaPsiRadians };
            }

            bool contains(const Vector& local, Eigen::Vector2d* coordinate = nullptr) const
            {
                if(local.z() <= 1.0e-10) return false;
                const Eigen::Vector2d value = coordinates(local.head<2>() / local.z());
                if(coordinate) *coordinate = value;
                return value.squaredNorm() <= m_radiusSquared + 1.0e-10;
            }

            Vector local(const PeriodicSpraySample& spray, const Vector& position) const
            {
                const Vector offset = position - spray.position;
                return { offset.dot(spray.majorAxis), offset.dot(spray.minorAxis),
                    offset.dot(spray.direction) };
            }

            bool intersectsBounds(const PeriodicSpraySample& spray, const GpuBvhNode& node) const
            {
                const Vector minimum = nodeMinimum(node);
                const Vector maximum = nodeMaximum(node);
                const Vector center = (minimum + maximum) * 0.5 - spray.position;
                const Vector half = (maximum - minimum) * 0.5;
                for(const auto& plane : m_planes) {
                    const Vector world = spray.majorAxis * plane.x()
                        + spray.minorAxis * plane.y() + spray.direction * plane.z();
                    if(world.dot(center) + world.cwiseAbs().dot(half) < -1.0e-10) return false;
                }
                return true;
            }

            Eigen::Vector2d peakSlopes() const
            {
                return { std::tan(m_parameters.phiOffsetRadians),
                    std::tan(m_parameters.psiOffsetRadians) };
            }

            // Perspective projection keeps triangle edges straight. Test the
            // peak and numerically minimize the exact angular Gaussian on edges;
            // a footprint entirely inside a large triangle must not count as a miss.
            bool triangleDirection(const std::array<Vector, 3>& triangle, Vector& direction) const
            {
                std::vector<Vector> polygon(triangle.begin(), triangle.end());
                for(std::size_t p = 0; p < m_planes.size(); ++p) {
                    const Vector& plane = m_planes[p];
                    std::vector<Vector> clipped;
                    const double offset = p == 0 ? 1.0e-10 : 0.0;
                    for(std::size_t i = 0; i < polygon.size(); ++i) {
                        const Vector& a = polygon[i];
                        const Vector& b = polygon[(i + 1) % polygon.size()];
                        const double da = plane.dot(a) - offset;
                        const double db = plane.dot(b) - offset;
                        if(da >= 0) clipped.push_back(a);
                        if((da >= 0) != (db >= 0)) clipped.push_back(a + (b - a) * (da / (da - db)));
                    }
                    polygon = std::move(clipped);
                    if(polygon.empty()) return false;
                }
                std::vector<Eigen::Vector2d> projected;
                for(const Vector& vertex : polygon) projected.push_back(vertex.head<2>() / vertex.z());
                const Eigen::Vector2d peak = peakSlopes();
                bool positive = false, negative = false;
                for(std::size_t i = 0; i < projected.size(); ++i) {
                    const Eigen::Vector2d edge = projected[(i + 1) % projected.size()] - projected[i];
                    const Eigen::Vector2d delta = peak - projected[i];
                    const double cross = edge.x() * delta.y() - edge.y() * delta.x();
                    positive = positive || cross > 1.0e-14;
                    negative = negative || cross < -1.0e-14;
                }
                if(projected.size() >= 3 && (positive || negative) && !(positive && negative)) {
                    direction = Vector(peak.x(), peak.y(), 1.0);
                    return true;
                }
                for(std::size_t i = 0; i < projected.size(); ++i) {
                    const Eigen::Vector2d a = projected[i];
                    const Eigen::Vector2d edge = projected[(i + 1) % projected.size()] - a;
                    // Separate brackets avoid assuming global convexity after atan.
                    for(int bracket = 0; bracket < 8; ++bracket) {
                        double lo = bracket / 8.0, hi = (bracket + 1) / 8.0;
                        for(int iteration = 0; iteration < 28; ++iteration) {
                            const double left = (2.0 * lo + hi) / 3.0;
                            const double right = (lo + 2.0 * hi) / 3.0;
                            if(coordinates(a + left * edge).squaredNorm()
                                < coordinates(a + right * edge).squaredNorm()) hi = right;
                            else lo = left;
                        }
                        for(double t : { bracket / 8.0, (lo + hi) * 0.5, (bracket + 1) / 8.0 }) {
                            const Eigen::Vector2d point = a + t * edge;
                            if(coordinates(point).squaredNorm() <= m_radiusSquared + 1.0e-9) {
                                direction = Vector(point.x(), point.y(), 1.0);
                                return true;
                            }
                        }
                    }
                }
                return false;
            }

            bool valid = false;

        private:
            const PaperGaussianParameters& m_parameters;
            double m_radiusSquared, m_cos, m_sin;
            std::array<Vector, 5> m_planes;
        };

        struct FootprintCoverage
        {
            std::size_t count{ 0 };
            Eigen::Vector2d mean = Eigen::Vector2d::Zero();
            Eigen::Matrix2d covarianceSum = Eigen::Matrix2d::Zero();
            unsigned directions{ 0 };
            bool centerSampled{ false };

            void add(const Eigen::Vector2d& coordinate)
            {
                const Eigen::Vector2d delta = coordinate - mean;
                mean += delta / static_cast<double>(++count);
                covarianceSum += delta * (coordinate - mean).transpose();
                centerSampled = centerSampled || coordinate.squaredNorm() <= 1.0;
                if(coordinate.x() <= -0.5) directions |= 1U;
                if(coordinate.x() >= 0.5) directions |= 2U;
                if(coordinate.y() <= -0.5) directions |= 4U;
                if(coordinate.y() >= 0.5) directions |= 8U;
            }

            bool bothDirectionsSampled() const
            {
                if(count < 3 || directions != 15U) return false;
                const double trace = covarianceSum.trace();
                const double difference = std::hypot(covarianceSum(0, 0) - covarianceSum(1, 1),
                    2.0 * covarianceSum(0, 1));
                // A diagonal line has both coordinate signs but no 2-D coverage.
                return trace > 0.0 && trace - difference >= 0.05 * (trace + difference);
            }
        };

        class PointEvaluator
        {
        public:
            PointEvaluator(const ThicknessPredictionTask& task, const ThicknessBvh& bvh,
                const ThicknessPredictionExecution& execution, double ratio)
                : m_task(task), m_bvh(bvh), m_execution(execution), m_core(task.options.deposition, ratio),
                  m_query(task.workpiece, bvh, execution)
            {
                const auto& samples = task.workpiece.samples;
                std::unordered_map<PositionKey, std::size_t, PositionHash> positions;
                positions.reserve(samples.size());
                m_positionIds.resize(samples.size());
                m_vertexStamps.resize(samples.size(), 0);
                m_uniqueStamps.resize(samples.size(), 0);
                m_selected.resize(samples.size(), task.options.spatialFiltering.predictionVertexIndices.empty());
                for(auto index : task.options.spatialFiltering.predictionVertexIndices) {
                    if(index >= samples.size()) throw std::runtime_error("Invalid assessment prediction vertex index.");
                    m_selected[index] = true;
                }
                for(std::size_t i = 0; i < samples.size(); ++i) {
                    if((i & 4095U) == 0) checkCancelled(execution);
                    PositionKey key;
                    // Nanometre bins merge coincident STL positions, not real mesh cells.
                    for(int axis = 0; axis < 3; ++axis) key.coordinates[axis] =
                        static_cast<std::int64_t>(std::llround(samples[i].position[axis] * 1.0e9));
                    m_positionIds[i] = positions.emplace(key, positions.size()).first->second;
                }
            }

            MeshSamplingPointAssessment evaluate(const PeriodicSpraySample& spray)
            {
                MeshSamplingPointAssessment result;
                result.sampleIndex = spray.sourceIndex;
                result.timeSeconds = spray.time;
                result.durationSeconds = spray.duration;
                ++m_stamp;
                m_candidates.clear();
                m_stack = { 0 };
                FootprintCoverage coverage;
                while(!m_stack.empty()) {
                    if((++m_visits & 4095U) == 0) checkCancelled(m_execution);
                    const auto& node = m_bvh.nodes[m_stack.back()];
                    m_stack.pop_back();
                    if(!m_core.intersectsBounds(spray, node)) continue;
                    if(node.triangleCount <= 0) {
                        m_stack.push_back(node.leftChild);
                        m_stack.push_back(node.rightChild);
                        continue;
                    }
                    for(int t = 0; t < node.triangleCount; ++t) {
                        const auto triangle = m_bvh.triangleOrder[node.firstTriangle + t];
                        m_candidates.push_back(triangle);
                        for(auto index : m_query.indices(triangle)) visitVertex(spray, index, result, coverage);
                    }
                }
                result.centerSampled = coverage.centerSampled;
                result.bothDirectionsSampled = coverage.bothDirectionsSampled();
                if(result.visibleVertexCount > 0) {
                    result.status = MeshSamplingStatus::Insufficient;
                    return result;
                }
                bool backFacing = false;
                const auto peak = m_core.peakSlopes();
                if(visibleDirection(spray, Vector(peak.x(), peak.y(), 1), backFacing)) {
                    result.status = MeshSamplingStatus::Insufficient;
                    return result;
                }
                for(auto triangle : m_candidates) {
                    if((++m_visits & 255U) == 0) checkCancelled(m_execution);
                    const auto ids = m_query.indices(triangle);
                    std::array<Vector, 3> local;
                    for(int i = 0; i < 3; ++i) local[i] =
                        m_core.local(spray, m_task.workpiece.samples[ids[i]].position);
                    Vector direction;
                    if(m_core.triangleDirection(local, direction)
                        && visibleDirection(spray, direction, backFacing)) {
                        result.status = MeshSamplingStatus::Insufficient;
                        return result;
                    }
                }
                result.status = backFacing ? MeshSamplingStatus::BackFacing : MeshSamplingStatus::Missed;
                return result;
            }

        private:
            void visitVertex(const PeriodicSpraySample& spray, std::uint32_t index,
                MeshSamplingPointAssessment& result, FootprintCoverage& coverage)
            {
                if(m_vertexStamps[index] == m_stamp) return;
                m_vertexStamps[index] = m_stamp;
                const auto& sample = m_task.workpiece.samples[index];
                const auto positionId = m_positionIds[index];
                if(!m_selected[index] || !sample.valid || m_uniqueStamps[positionId] == m_stamp
                    || sample.normal.dot(spray.position - sample.position) <= 0.0) return;
                Eigen::Vector2d coordinate;
                if(!m_core.contains(m_core.local(spray, sample.position), &coordinate)) return;
                const Vector delta = sample.position - spray.position;
                const double distance = delta.norm();
                const double rayLimit = distance - std::max(1.0e-9, m_task.options.shadowBiasMeters);
                if(m_query.firstHit(spray.position, delta / distance, rayLimit).found) return;
                m_uniqueStamps[positionId] = m_stamp;
                ++result.visibleVertexCount;
                coverage.add(coordinate);
            }

            bool visibleDirection(const PeriodicSpraySample& spray, const Vector& localDirection,
                bool& backFacing)
            {
                const Vector direction = (spray.majorAxis * localDirection.x()
                    + spray.minorAxis * localDirection.y() + spray.direction * localDirection.z()).normalized();
                const RayHit hit = m_query.firstHit(spray.position, direction);
                if(!hit.found) return false;
                if(hit.normal.dot(-direction) > 0.0) return true;
                backFacing = true;
                return false;
            }

            const ThicknessPredictionTask& m_task;
            const ThicknessBvh& m_bvh;
            const ThicknessPredictionExecution& m_execution;
            CoreFootprint m_core;
            SurfaceQuery m_query;
            std::vector<std::size_t> m_positionIds, m_vertexStamps, m_uniqueStamps;
            std::vector<bool> m_selected;
            std::vector<std::int32_t> m_stack;
            std::vector<std::uint32_t> m_candidates;
            std::size_t m_stamp{ 0 }, m_visits{ 0 };
        };

        void accumulate(MeshSamplingAssessment& assessment, MeshSamplingPointAssessment point)
        {
            if(point.status == MeshSamplingStatus::Missed) ++assessment.missedPointCount;
            else if(point.status == MeshSamplingStatus::BackFacing) ++assessment.backFacingPointCount;
            else {
                const auto count = point.visibleVertexCount;
                if(assessment.assessedPointCount == 0 || count < assessment.minimumVisibleVertexCount) {
                    assessment.minimumVisibleVertexCount = count;
                    assessment.worstSampleIndex = point.sampleIndex;
                    assessment.worstTimeSeconds = point.timeSeconds;
                }
                ++assessment.assessedPointCount;
                assessment.averageVisibleVertexCount +=
                    (static_cast<double>(count) - assessment.averageVisibleVertexCount) / assessment.assessedPointCount;
                assessment.assessedDurationSeconds += point.durationSeconds;
                if(count < assessment.minimumVertexCount) {
                    ++assessment.insufficientPointCount;
                    assessment.insufficientDurationSeconds += point.durationSeconds;
                } else if(count < assessment.recommendedVertexCount) point.status = MeshSamplingStatus::Minimum;
                else {
                    point.status = MeshSamplingStatus::Recommended;
                    ++assessment.recommendedPointCount;
                    assessment.recommendedDurationSeconds += point.durationSeconds;
                }
                if(!point.centerSampled || !point.bothDirectionsSampled) ++assessment.limitedDistributionPointCount;
                if(!point.centerSampled) ++assessment.centerGapPointCount;
                if(!point.bothDirectionsSampled) ++assessment.directionalGapPointCount;
            }
            assessment.points.push_back(point);
        }
    }

    MeshSamplingAssessment MeshSamplingEvaluator::summarize(
        const ThicknessPredictionTask& task,
        std::vector<MeshSamplingPointAssessment> points)
    {
        MeshSamplingAssessment result;
        const auto sampled = task.options.base.trajectorySamplingMode == TrajectorySamplingMode::OriginalPoints
            ? spraytrajectory::SprayTrajectorySampler::originalSamples(task.trajectory)
            : spraytrajectory::SprayTrajectorySampler::sample(task.trajectory, task.options.base.timeStep);
        result.totalPointCount = sampled.size();
        for(std::size_t i = 0; i < sampled.size(); ++i) {
            const auto& sample = sampled[i];
            if(!sample.sprayEnabled) ++result.airMovePointCount;
            else if(i + 1 == sampled.size() || sampled[i + 1].time <= sample.time
                || (!sample.processId.empty() && !task.process.id.empty() && sample.processId != task.process.id)) {
                ++result.ignoredPointCount;
            }
        }
        result.applicable = true;
        result.points.reserve(points.size());
        for(auto& point : points) accumulate(result, point);
        return result;
    }

    MeshSamplingAssessment MeshSamplingEvaluator::evaluate(
        const ThicknessPredictionTask& task, const ThicknessPredictionExecution& execution)
    {
        const auto started = Clock::now();
        MeshSamplingAssessment result;
        const auto finish = [&]() {
            result.elapsedMilliseconds = std::chrono::duration<double, std::milli>(Clock::now() - started).count();
            return std::move(result);
        };
        checkCancelled(execution);
        if(task.options.periodicLocal.enabled || task.options.axisymmetricProfile.enabled) {
            result.unavailableReason = "Reduced prediction modes do not use the full surface vertex set.";
            return finish();
        }
        if(task.workpiece.triangleIndices.empty()) {
            result.unavailableReason = "Triangle topology is required to distinguish a missed footprint from coarse sampling.";
            return finish();
        }
        if(!CoreFootprint(task.options.deposition, result.corePatternRatio).valid) {
            result.unavailableReason = "Gaussian angular parameters cannot define a forward core footprint.";
            return finish();
        }
        result = summarize(task, {});
        result.applicable = false;
        const auto sprays = makePeriodicSpraySamples(task);
        const auto bvh = ThicknessBvhBuilder::build(task.workpiece, 8,
            [&](const char*, std::size_t, std::size_t) { checkCancelled(execution); });
        checkCancelled(execution);
        if(bvh.empty()) {
            result.unavailableReason = "No usable substrate triangles.";
            return finish();
        }
        PointEvaluator evaluator(task, bvh, execution, result.corePatternRatio);
        result.applicable = true;
        result.points.reserve(sprays.size());
        auto lastProgress = Clock::now();
        for(std::size_t i = 0; i < sprays.size(); ++i) {
            checkCancelled(execution);
            accumulate(result, evaluator.evaluate(sprays[i]));
            const auto now = Clock::now();
            if(execution.progress && (now - lastProgress > std::chrono::milliseconds(100) || i + 1 == sprays.size())) {
                execution.progress(static_cast<double>(i + 1) / sprays.size(), "Assessing visible mesh samples per spray footprint...");
                lastProgress = now;
            }
        }
        return finish();
    }
}
