#include "PeriodicSectorReduction.h"
#include "OpenGLComputeProgram.h"
#include "PeriodicSectorMappingShader.h"

#include <SprayThicknessPredictionOpenGL/ThicknessBvh.h>

#include <Eigen/Geometry>

#include <glad/glad.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

namespace spraythickness::opengl
{
    namespace
    {
        constexpr double kPi = 3.1415926535897932384626433832795;
        constexpr double kTwoPi = 2.0 * kPi;
        constexpr std::uint64_t kPeriodicCacheMagic = 0x5253323032365052ull;
        constexpr std::uint32_t kPeriodicCacheVersion = 3;

        std::mutex g_periodicCacheMutex;
        std::unordered_map<std::uint64_t, PeriodicSectorReduction> g_periodicCache;

        struct PeriodicCacheHeader
        {
            std::uint64_t magic = kPeriodicCacheMagic;
            std::uint32_t version = kPeriodicCacheVersion;
            std::uint32_t reserved = 0;
            std::uint64_t key = 0;
            std::uint64_t sampleCount = 0;
            std::uint64_t triangleIndexCount = 0;
            std::uint64_t predictionVertexCount = 0;
            std::uint64_t predictionTriangleCount = 0;
            std::uint64_t bindingCount = 0;
            std::uint64_t localTriangleCount = 0;
            double localBoundsCenter[3] = { 0.0, 0.0, 0.0 };
            double localBoundsRadius = 0.0;
            double maximumMappingDistance = 0.0;
        };

        std::filesystem::path findProjectRoot()
        {
            std::filesystem::path current = std::filesystem::current_path();
            for(int depth = 0; depth < 10 && !current.empty(); ++depth) {
                if(std::filesystem::exists(current / "CMakeLists.txt") &&
                    std::filesystem::exists(current / "SMRobotSpray")) {
                    return current;
                }
                const std::filesystem::path parent = current.parent_path();
                if(parent == current) {
                    break;
                }
                current = parent;
            }
            return {};
        }

        std::filesystem::path periodicCachePath(std::uint64_t key)
        {
            const std::filesystem::path root = findProjectRoot();
            if(root.empty()) {
                return {};
            }
            return root / ".cache" / "thickness_periodic" /
                (std::to_string(key) + ".bin");
        }

        void appendHash(
            std::uint64_t& hash,
            const void* data,
            std::size_t size)
        {
            constexpr std::uint64_t kFnvPrime = 1099511628211ull;
            const auto* bytes = static_cast<const std::uint8_t*>(data);
            for(std::size_t index = 0; index < size; ++index) {
                hash ^= bytes[index];
                hash *= kFnvPrime;
            }
        }

        template <typename Value>
        void appendHash(std::uint64_t& hash, const Value& value)
        {
            appendHash(hash, &value, sizeof(value));
        }

        std::uint64_t periodicCacheKey(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const PeriodicLocalPredictionOptions& options,
            const Eigen::Vector3d& axis,
            const Eigen::Vector3d& radialX)
        {
            constexpr std::uint64_t kFnvOffset = 14695981039346656037ull;
            std::uint64_t hash = kFnvOffset;
            for(const auto& sample : workpiece.samples) {
                appendHash(hash, sample.position.x());
                appendHash(hash, sample.position.y());
                appendHash(hash, sample.position.z());
                appendHash(hash, sample.normal.x());
                appendHash(hash, sample.normal.y());
                appendHash(hash, sample.normal.z());
            }
            if(!workpiece.triangleIndices.empty()) {
                appendHash(
                    hash,
                    workpiece.triangleIndices.data(),
                    workpiece.triangleIndices.size() * sizeof(std::uint32_t));
            }
            appendHash(hash, options.axisOrigin.x());
            appendHash(hash, options.axisOrigin.y());
            appendHash(hash, options.axisOrigin.z());
            appendHash(hash, axis.x());
            appendHash(hash, axis.y());
            appendHash(hash, axis.z());
            appendHash(hash, radialX.x());
            appendHash(hash, radialX.y());
            appendHash(hash, radialX.z());
            const std::uint64_t sectorCount = options.sectorCount;
            appendHash(hash, sectorCount);
            appendHash(hash, options.maximumMappingDistanceMeters);
            appendHash(hash, options.maximumNormalAngleDegrees);
            return hash;
        }

        template <typename Value>
        bool readValue(std::ifstream& input, Value& value)
        {
            return static_cast<bool>(input.read(
                reinterpret_cast<char*>(&value), sizeof(value)));
        }

        template <typename Value>
        bool readVector(
            std::ifstream& input,
            std::vector<Value>& values,
            std::uint64_t count)
        {
            if(count > 1000000000ull) {
                return false;
            }
            values.resize(static_cast<std::size_t>(count));
            return count == 0 || static_cast<bool>(input.read(
                reinterpret_cast<char*>(values.data()),
                static_cast<std::streamsize>(values.size() * sizeof(Value))));
        }

        bool loadPeriodicCache(
            std::uint64_t key,
            const sprayworkpiece::WorkpieceModel& workpiece,
            PeriodicSectorReduction& result)
        {
            const std::filesystem::path path = periodicCachePath(key);
            if(path.empty() || !std::filesystem::exists(path)) {
                return false;
            }
            std::ifstream input(path, std::ios::binary);
            PeriodicCacheHeader header;
            if(!readValue(input, header) ||
                header.magic != kPeriodicCacheMagic ||
                header.version != kPeriodicCacheVersion ||
                header.key != key ||
                header.sampleCount != workpiece.samples.size() ||
                header.triangleIndexCount != workpiece.triangleIndices.size() ||
                header.bindingCount != workpiece.samples.size()) {
                return false;
            }
            if(!readVector(input, result.predictionVertexIndices, header.predictionVertexCount) ||
                !readVector(input, result.predictionTriangleIndices, header.predictionTriangleCount) ||
                !readVector(input, result.fullVertexBindings, header.bindingCount)) {
                return false;
            }
            const std::size_t triangleCount = workpiece.triangleIndices.size() / 3;
            for(const std::uint32_t vertex : result.predictionVertexIndices) {
                if(vertex >= workpiece.samples.size()) {
                    return false;
                }
            }
            for(const std::uint32_t triangle : result.predictionTriangleIndices) {
                if(triangle >= triangleCount) {
                    return false;
                }
            }
            for(const PeriodicSectorBinding& binding : result.fullVertexBindings) {
                for(const std::uint32_t vertex : binding.sourceVertexIndices) {
                    if(vertex >= workpiece.samples.size()) {
                        return false;
                    }
                }
            }
            result.localBoundsCenter = Eigen::Vector3d(
                header.localBoundsCenter[0],
                header.localBoundsCenter[1],
                header.localBoundsCenter[2]);
            result.localBoundsRadius = header.localBoundsRadius;
            result.maximumMappingDistance = header.maximumMappingDistance;
            result.localTriangleCount = static_cast<std::size_t>(header.localTriangleCount);
            result.mappingCacheHit = true;
            return result.localSelectionValid() && result.valid();
        }

        void savePeriodicCache(
            std::uint64_t key,
            const sprayworkpiece::WorkpieceModel& workpiece,
            const PeriodicSectorReduction& result)
        {
            const std::filesystem::path path = periodicCachePath(key);
            if(path.empty() || !result.valid()) {
                return;
            }
            std::error_code error;
            std::filesystem::create_directories(path.parent_path(), error);
            if(error) {
                return;
            }
            std::ofstream output(path, std::ios::binary | std::ios::trunc);
            if(!output) {
                return;
            }
            PeriodicCacheHeader header;
            header.key = key;
            header.sampleCount = workpiece.samples.size();
            header.triangleIndexCount = workpiece.triangleIndices.size();
            header.predictionVertexCount = result.predictionVertexIndices.size();
            header.predictionTriangleCount = result.predictionTriangleIndices.size();
            header.bindingCount = result.fullVertexBindings.size();
            header.localTriangleCount = result.localTriangleCount;
            header.localBoundsCenter[0] = result.localBoundsCenter.x();
            header.localBoundsCenter[1] = result.localBoundsCenter.y();
            header.localBoundsCenter[2] = result.localBoundsCenter.z();
            header.localBoundsRadius = result.localBoundsRadius;
            header.maximumMappingDistance = result.maximumMappingDistance;
            output.write(reinterpret_cast<const char*>(&header), sizeof(header));
            output.write(
                reinterpret_cast<const char*>(result.predictionVertexIndices.data()),
                static_cast<std::streamsize>(result.predictionVertexIndices.size() * sizeof(std::uint32_t)));
            output.write(
                reinterpret_cast<const char*>(result.predictionTriangleIndices.data()),
                static_cast<std::streamsize>(result.predictionTriangleIndices.size() * sizeof(std::uint32_t)));
            output.write(
                reinterpret_cast<const char*>(result.fullVertexBindings.data()),
                static_cast<std::streamsize>(result.fullVertexBindings.size() * sizeof(PeriodicSectorBinding)));
        }

        struct ClosestTriangle
        {
            std::array<std::uint32_t, 3> indices{};
            std::array<float, 3> weights{};
            double squaredDistance = std::numeric_limits<double>::max();
            double normalAlignment = -1.0;
            bool found{ false };
        };

        struct LocalMesh
        {
            sprayworkpiece::WorkpieceModel workpiece;
            std::vector<std::uint32_t> localToGlobal;
            std::vector<std::uint32_t> globalTriangleIndices;
        };

        double cross2d(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs)
        {
            return lhs.x() * rhs.y() - lhs.y() * rhs.x();
        }

        bool pointInsideAngularSector(
            const Eigen::Vector2d& point,
            const Eigen::Vector2d& lowerBoundary,
            const Eigen::Vector2d& upperBoundary,
            double tolerance)
        {
            return cross2d(lowerBoundary, point) >= -tolerance
                && cross2d(upperBoundary, point) <= tolerance;
        }

        bool segmentIntersectsBoundaryRay(
            const Eigen::Vector2d& start,
            const Eigen::Vector2d& end,
            const Eigen::Vector2d& rayDirection,
            double tolerance)
        {
            const Eigen::Vector2d edge = end - start;
            const double denominator = cross2d(edge, rayDirection);
            if(std::abs(denominator) <= tolerance) {
                if(std::abs(cross2d(start, rayDirection)) > tolerance) {
                    return false;
                }
                return std::max(
                    start.dot(rayDirection), end.dot(rayDirection)) >= -tolerance;
            }

            const double segmentParameter =
                -cross2d(start, rayDirection) / denominator;
            constexpr double kParameterTolerance = 1.0e-12;
            if(segmentParameter < -kParameterTolerance
                || segmentParameter > 1.0 + kParameterTolerance) {
                return false;
            }
            const Eigen::Vector2d intersection = start + segmentParameter * edge;
            return intersection.dot(rayDirection) >= -tolerance;
        }

        bool triangleIntersectsAngularSector(
            const std::array<Eigen::Vector2d, 3>& triangle,
            const Eigen::Vector2d& lowerBoundary,
            const Eigen::Vector2d& upperBoundary)
        {
            double maximumRadius = 0.0;
            for(const Eigen::Vector2d& point : triangle) {
                maximumRadius = std::max(maximumRadius, point.norm());
            }
            const double tolerance = std::max(1.0e-12, maximumRadius * 1.0e-12);
            for(const Eigen::Vector2d& point : triangle) {
                if(pointInsideAngularSector(
                    point, lowerBoundary, upperBoundary, tolerance)) {
                    return true;
                }
            }

            for(std::size_t edge = 0; edge < triangle.size(); ++edge) {
                const Eigen::Vector2d& start = triangle[edge];
                const Eigen::Vector2d& end = triangle[(edge + 1) % triangle.size()];
                if(segmentIntersectsBoundaryRay(
                        start, end, lowerBoundary, tolerance)
                    || segmentIntersectsBoundaryRay(
                        start, end, upperBoundary, tolerance)) {
                    return true;
                }
            }
            return false;
        }

        double angleAroundAxis(
            const Eigen::Vector3d& position,
            const Eigen::Vector3d& origin,
            const Eigen::Vector3d& axis,
            const Eigen::Vector3d& radialX,
            const Eigen::Vector3d& radialY)
        {
            const Eigen::Vector3d offset = position - origin;
            const Eigen::Vector3d radial = offset - offset.dot(axis) * axis;
            return std::atan2(radial.dot(radialY), radial.dot(radialX));
        }

        Eigen::Vector3d canonicalizeVector(
            const Eigen::Vector3d& value,
            const Eigen::Vector3d& axis,
            double angle)
        {
            return Eigen::AngleAxisd(angle, axis) * value;
        }

        Eigen::Vector3d canonicalizePosition(
            const Eigen::Vector3d& position,
            const Eigen::Vector3d& origin,
            const Eigen::Vector3d& axis,
            double angle)
        {
            return origin + canonicalizeVector(position - origin, axis, angle);
        }

        struct VertexCellKey
        {
            std::int64_t x{ 0 };
            std::int64_t y{ 0 };
            std::int64_t z{ 0 };

            bool operator==(const VertexCellKey& other) const
            {
                return x == other.x && y == other.y && z == other.z;
            }
        };

        struct VertexCellKeyHash
        {
            std::size_t operator()(const VertexCellKey& key) const
            {
                std::size_t value = std::hash<std::int64_t>{}(key.x);
                value ^= std::hash<std::int64_t>{}(key.y)
                    + 0x9e3779b9U + (value << 6U) + (value >> 2U);
                value ^= std::hash<std::int64_t>{}(key.z)
                    + 0x9e3779b9U + (value << 6U) + (value >> 2U);
                return value;
            }
        };

        VertexCellKey vertexCellKey(const Eigen::Vector3d& position, double cellSize)
        {
            return {
                static_cast<std::int64_t>(std::floor(position.x() / cellSize)),
                static_cast<std::int64_t>(std::floor(position.y() / cellSize)),
                static_cast<std::int64_t>(std::floor(position.z() / cellSize))
            };
        }

        void applyExactVertexBindings(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const LocalMesh& localMesh,
            const PeriodicLocalPredictionOptions& options,
            const Eigen::Vector3d& axis,
            const Eigen::Vector3d& radialX,
            const Eigen::Vector3d& radialY,
            double sectorAngle,
            double minimumNormalDot,
            PeriodicSectorReduction& result)
        {
            if(localMesh.workpiece.samples.empty()
                || result.fullVertexBindings.size() != workpiece.samples.size()) {
                return;
            }

            Eigen::Vector3d minimum = localMesh.workpiece.samples.front().position;
            Eigen::Vector3d maximum = minimum;
            for(const auto& sample : localMesh.workpiece.samples) {
                minimum = minimum.cwiseMin(sample.position);
                maximum = maximum.cwiseMax(sample.position);
            }
            const double positionTolerance = std::max(
                1.0e-9,
                (maximum - minimum).norm() * 1.0e-10);
            const double squaredPositionTolerance = positionTolerance * positionTolerance;
            std::unordered_map<VertexCellKey,
                std::vector<std::uint32_t>, VertexCellKeyHash> localVerticesByCell;
            localVerticesByCell.reserve(localMesh.workpiece.samples.size());
            for(std::size_t localIndex = 0;
                localIndex < localMesh.workpiece.samples.size();
                ++localIndex) {
                localVerticesByCell[vertexCellKey(
                    localMesh.workpiece.samples[localIndex].position,
                    positionTolerance)].push_back(static_cast<std::uint32_t>(localIndex));
            }

            for(std::size_t vertex = 0; vertex < workpiece.samples.size(); ++vertex) {
                const auto& sample = workpiece.samples[vertex];
                const double angle = angleAroundAxis(
                    sample.position, options.axisOrigin, axis, radialX, radialY);
                const double canonicalAngle = std::remainder(angle, sectorAngle);
                const double rotationAngle = canonicalAngle - angle;
                const Eigen::Vector3d canonicalPosition = canonicalizePosition(
                    sample.position, options.axisOrigin, axis, rotationAngle);
                Eigen::Vector3d canonicalNormal = canonicalizeVector(
                    sample.normal, axis, rotationAngle);
                if(canonicalNormal.norm() <= 1.0e-12) {
                    continue;
                }
                canonicalNormal.normalize();

                const VertexCellKey center = vertexCellKey(
                    canonicalPosition, positionTolerance);
                double bestDistance = squaredPositionTolerance;
                double bestNormalAlignment = minimumNormalDot;
                std::uint32_t bestLocalIndex = std::numeric_limits<std::uint32_t>::max();
                for(std::int64_t offsetX = -1; offsetX <= 1; ++offsetX) {
                    for(std::int64_t offsetY = -1; offsetY <= 1; ++offsetY) {
                        for(std::int64_t offsetZ = -1; offsetZ <= 1; ++offsetZ) {
                            const auto iterator = localVerticesByCell.find({
                                center.x + offsetX,
                                center.y + offsetY,
                                center.z + offsetZ});
                            if(iterator == localVerticesByCell.end()) {
                                continue;
                            }
                            for(const std::uint32_t localIndex : iterator->second) {
                                const auto& localSample = localMesh.workpiece.samples[localIndex];
                                const double squaredDistance = (
                                    canonicalPosition - localSample.position).squaredNorm();
                                Eigen::Vector3d localNormal = localSample.normal;
                                if(squaredDistance > bestDistance
                                    || localNormal.norm() <= 1.0e-12) {
                                    continue;
                                }
                                localNormal.normalize();
                                const double normalAlignment = localNormal.dot(canonicalNormal);
                                if(normalAlignment < bestNormalAlignment) {
                                    continue;
                                }
                                if(squaredDistance < bestDistance
                                    || normalAlignment > bestNormalAlignment
                                    || bestLocalIndex == std::numeric_limits<std::uint32_t>::max()) {
                                    bestDistance = squaredDistance;
                                    bestNormalAlignment = normalAlignment;
                                    bestLocalIndex = localIndex;
                                }
                            }
                        }
                    }
                }
                if(bestLocalIndex == std::numeric_limits<std::uint32_t>::max()) {
                    continue;
                }
                const std::uint32_t sourceVertex = localMesh.localToGlobal[bestLocalIndex];
                result.fullVertexBindings[vertex].sourceVertexIndices = {
                    sourceVertex, sourceVertex, sourceVertex};
                result.fullVertexBindings[vertex].weights = { 1.0f, 0.0f, 0.0f };
            }
        }

        double squaredDistanceToBox(
            const Eigen::Vector3d& pointMillimeters,
            const GpuBvhNode& node)
        {
            double squaredDistance = 0.0;
            for(int component = 0; component < 3; ++component) {
                const double minimum = node.minimum[component];
                const double maximum = node.maximum[component];
                if(pointMillimeters[component] < minimum) {
                    const double delta = minimum - pointMillimeters[component];
                    squaredDistance += delta * delta;
                } else if(pointMillimeters[component] > maximum) {
                    const double delta = pointMillimeters[component] - maximum;
                    squaredDistance += delta * delta;
                }
            }
            return squaredDistance;
        }

        std::array<double, 3> closestPointWeights(
            const Eigen::Vector3d& point,
            const Eigen::Vector3d& a,
            const Eigen::Vector3d& b,
            const Eigen::Vector3d& c)
        {
            const Eigen::Vector3d ab = b - a;
            const Eigen::Vector3d ac = c - a;
            const Eigen::Vector3d ap = point - a;
            const double d1 = ab.dot(ap);
            const double d2 = ac.dot(ap);
            if(d1 <= 0.0 && d2 <= 0.0) {
                return { 1.0, 0.0, 0.0 };
            }

            const Eigen::Vector3d bp = point - b;
            const double d3 = ab.dot(bp);
            const double d4 = ac.dot(bp);
            if(d3 >= 0.0 && d4 <= d3) {
                return { 0.0, 1.0, 0.0 };
            }

            const double vc = d1 * d4 - d3 * d2;
            if(vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
                const double v = d1 / (d1 - d3);
                return { 1.0 - v, v, 0.0 };
            }

            const Eigen::Vector3d cp = point - c;
            const double d5 = ab.dot(cp);
            const double d6 = ac.dot(cp);
            if(d6 >= 0.0 && d5 <= d6) {
                return { 0.0, 0.0, 1.0 };
            }

            const double vb = d5 * d2 - d1 * d6;
            if(vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
                const double w = d2 / (d2 - d6);
                return { 1.0 - w, 0.0, w };
            }

            const double va = d3 * d6 - d5 * d4;
            if(va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0) {
                const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                return { 0.0, 1.0 - w, w };
            }

            const double denominator = 1.0 / (va + vb + vc);
            const double v = vb * denominator;
            const double w = vc * denominator;
            return { 1.0 - v - w, v, w };
        }

        Eigen::Vector3d weightedPoint(
            const std::array<double, 3>& weights,
            const Eigen::Vector3d& a,
            const Eigen::Vector3d& b,
            const Eigen::Vector3d& c)
        {
            return weights[0] * a + weights[1] * b + weights[2] * c;
        }

        LocalMesh buildLocalMesh(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const Eigen::Vector3d& origin,
            const Eigen::Vector3d& axis,
            const Eigen::Vector3d& radialX,
            const Eigen::Vector3d& radialY,
            double halfSectorAngle)
        {
            LocalMesh result;
            std::unordered_map<std::uint32_t, std::uint32_t> globalToLocal;
            const auto addVertex = [&](std::uint32_t globalIndex) {
                const auto existing = globalToLocal.find(globalIndex);
                if(existing != globalToLocal.end()) {
                    return existing->second;
                }
                const std::uint32_t localIndex =
                    static_cast<std::uint32_t>(result.workpiece.samples.size());
                globalToLocal.emplace(globalIndex, localIndex);
                result.workpiece.samples.push_back(workpiece.samples[globalIndex]);
                result.localToGlobal.push_back(globalIndex);
                return localIndex;
            };

            const double boundaryCosine = std::cos(halfSectorAngle);
            const double boundarySine = std::sin(halfSectorAngle);
            const Eigen::Vector2d lowerBoundary(boundaryCosine, -boundarySine);
            const Eigen::Vector2d upperBoundary(boundaryCosine, boundarySine);
            const std::size_t triangleCount = workpiece.triangleIndices.size() / 3;
            for(std::size_t triangle = 0; triangle < triangleCount; ++triangle) {
                const std::uint32_t global0 = workpiece.triangleIndices[triangle * 3];
                const std::uint32_t global1 = workpiece.triangleIndices[triangle * 3 + 1];
                const std::uint32_t global2 = workpiece.triangleIndices[triangle * 3 + 2];
                const auto radialPoint = [&](std::uint32_t globalIndex) {
                    const Eigen::Vector3d offset =
                        workpiece.samples[globalIndex].position - origin;
                    return Eigen::Vector2d(
                        offset.dot(radialX), offset.dot(radialY));
                };
                const std::array<Eigen::Vector2d, 3> radialTriangle{
                    radialPoint(global0), radialPoint(global1), radialPoint(global2)
                };
                if(!triangleIntersectsAngularSector(
                    radialTriangle, lowerBoundary, upperBoundary)) {
                    continue;
                }

                result.workpiece.triangleIndices.push_back(addVertex(global0));
                result.workpiece.triangleIndices.push_back(addVertex(global1));
                result.workpiece.triangleIndices.push_back(addVertex(global2));
                result.globalTriangleIndices.push_back(
                    static_cast<std::uint32_t>(triangle));
            }
            return result;
        }

        Eigen::Vector3d orthogonalAxis(
            const Eigen::Vector3d& candidate,
            const Eigen::Vector3d& direction,
            const Eigen::Vector3d& fallback)
        {
            Eigen::Vector3d axis = candidate - candidate.dot(direction) * direction;
            if(axis.norm() <= 1.0e-12) {
                axis = fallback - fallback.dot(direction) * direction;
            }
            if(axis.norm() <= 1.0e-12) {
                const Eigen::Vector3d canonical = std::abs(direction.x()) < 0.9
                    ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
                axis = canonical - canonical.dot(direction) * direction;
            }
            return axis.normalized();
        }

        ClosestTriangle findClosestTriangle(
            const LocalMesh& localMesh,
            const ThicknessBvh& bvh,
            const Eigen::Vector3d& queryPosition,
            const Eigen::Vector3d& queryNormal,
            double minimumNormalDot)
        {
            ClosestTriangle closest;
            if(bvh.empty()) {
                return closest;
            }

            const Eigen::Vector3d queryMillimeters = queryPosition * 1000.0;
            std::vector<std::int32_t> stack;
            stack.reserve(64);
            stack.push_back(0);
            while(!stack.empty()) {
                const std::int32_t nodeIndex = stack.back();
                stack.pop_back();
                if(nodeIndex < 0 || nodeIndex >= static_cast<std::int32_t>(bvh.nodes.size())) {
                    continue;
                }
                const GpuBvhNode& node = bvh.nodes[static_cast<std::size_t>(nodeIndex)];
                if(squaredDistanceToBox(queryMillimeters, node)
                    > closest.squaredDistance * 1.0e6) {
                    continue;
                }
                if(node.triangleCount == 0) {
                    stack.push_back(node.leftChild);
                    stack.push_back(node.rightChild);
                    continue;
                }

                for(int offset = 0; offset < node.triangleCount; ++offset) {
                    const std::uint32_t triangle = bvh.triangleOrder[
                        static_cast<std::size_t>(node.firstTriangle + offset)];
                    const std::uint32_t local0 = localMesh.workpiece.triangleIndices[triangle * 3];
                    const std::uint32_t local1 = localMesh.workpiece.triangleIndices[triangle * 3 + 1];
                    const std::uint32_t local2 = localMesh.workpiece.triangleIndices[triangle * 3 + 2];
                    const auto& sample0 = localMesh.workpiece.samples[local0];
                    const auto& sample1 = localMesh.workpiece.samples[local1];
                    const auto& sample2 = localMesh.workpiece.samples[local2];
                    const std::array<double, 3> weights = closestPointWeights(
                        queryPosition, sample0.position, sample1.position, sample2.position);
                    Eigen::Vector3d interpolatedNormal = weights[0] * sample0.normal
                        + weights[1] * sample1.normal + weights[2] * sample2.normal;
                    if(interpolatedNormal.norm() <= 1.0e-12) {
                        continue;
                    }
                    interpolatedNormal.normalize();
                    if(interpolatedNormal.dot(queryNormal) < minimumNormalDot) {
                        continue;
                    }
                    const Eigen::Vector3d closestPoint = weightedPoint(
                        weights, sample0.position, sample1.position, sample2.position);
                    const double squaredDistance = (queryPosition - closestPoint).squaredNorm();
                    const double distanceTieTolerance = closest.found
                        ? std::max(1.0e-18, closest.squaredDistance * 1.0e-4 + 1.0e-18)
                        : 0.0;
                    const bool closer = !closest.found
                        || squaredDistance + distanceTieTolerance < closest.squaredDistance;
                    const bool sameDistance = closest.found
                        && std::abs(squaredDistance - closest.squaredDistance)
                            <= distanceTieTolerance;
                    if((!closer && !sameDistance)
                        || (sameDistance && interpolatedNormal.dot(queryNormal)
                            < closest.normalAlignment)) {
                        continue;
                    }
                    closest.squaredDistance = squaredDistance;
                    closest.normalAlignment = interpolatedNormal.dot(queryNormal);
                    closest.indices = {
                        localMesh.localToGlobal[local0],
                        localMesh.localToGlobal[local1],
                        localMesh.localToGlobal[local2]
                    };
                    closest.weights = {
                        static_cast<float>(weights[0]),
                        static_cast<float>(weights[1]),
                        static_cast<float>(weights[2])
                    };
                    closest.found = true;
                }
            }
            return closest;
        }

        struct alignas(16) GpuBindingIndices
        {
            std::array<std::uint32_t, 4> values{};
        };

        struct alignas(16) GpuBindingWeights
        {
            std::array<float, 4> values{};
        };

        class GpuMappingBuffer
        {
        public:
            GpuMappingBuffer()
            {
                glGenBuffers(1, &m_id);
            }

            ~GpuMappingBuffer()
            {
                if(m_id != 0) {
                    glDeleteBuffers(1, &m_id);
                }
            }

            GpuMappingBuffer(const GpuMappingBuffer&) = delete;
            GpuMappingBuffer& operator=(const GpuMappingBuffer&) = delete;

            void upload(
                unsigned int binding,
                const void* data,
                std::size_t size,
                unsigned int usage)
            {
                glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_id);
                glBufferData(
                    GL_SHADER_STORAGE_BUFFER,
                    static_cast<GLsizeiptr>(size),
                    data,
                    usage);
                glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, m_id);
                m_size = size;
            }

            void update(const void* data, std::size_t size)
            {
                if(size > m_size) {
                    throw std::runtime_error(
                        "Periodic GPU mapping buffer update exceeds its allocation.");
                }
                glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_id);
                glBufferSubData(
                    GL_SHADER_STORAGE_BUFFER,
                    0,
                    static_cast<GLsizeiptr>(size),
                    data);
            }

            void download(void* data, std::size_t size) const
            {
                if(size > m_size) {
                    throw std::runtime_error(
                        "Periodic GPU mapping buffer read exceeds its allocation.");
                }
                glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_id);
                glGetBufferSubData(
                    GL_SHADER_STORAGE_BUFFER,
                    0,
                    static_cast<GLsizeiptr>(size),
                    data);
            }

        private:
            unsigned int m_id = 0;
            std::size_t m_size = 0;
        };

        void checkGpuMappingErrors(const char* phase)
        {
            const GLenum error = glGetError();
            if(error != GL_NO_ERROR) {
                throw std::runtime_error(
                    std::string("Periodic GPU mapping OpenGL error during ")
                    + phase + ": " + std::to_string(static_cast<unsigned int>(error)));
            }
        }

        std::vector<std::array<float, 4>> makeGpuSurfaceData(
            const sprayworkpiece::WorkpieceModel& workpiece,
            std::size_t begin,
            std::size_t count)
        {
            std::vector<std::array<float, 4>> result;
            result.reserve(count * 2);
            for(std::size_t index = begin; index < begin + count; ++index) {
                const auto& sample = workpiece.samples[index];
                result.push_back({
                    static_cast<float>(sample.position.x() * 1000.0),
                    static_cast<float>(sample.position.y() * 1000.0),
                    static_cast<float>(sample.position.z() * 1000.0),
                    0.0f
                });
                Eigen::Vector3d normal = sample.normal;
                if(normal.norm() <= 1.0e-12) {
                    normal = Eigen::Vector3d::UnitZ();
                } else {
                    normal.normalize();
                }
                result.push_back({
                    static_cast<float>(normal.x()),
                    static_cast<float>(normal.y()),
                    static_cast<float>(normal.z()),
                    0.0f
                });
            }
            return result;
        }

        std::vector<std::array<float, 4>> makeGpuCanonicalSurfaceData(
            const sprayworkpiece::WorkpieceModel& workpiece,
            std::size_t begin,
            std::size_t count,
            const PeriodicLocalPredictionOptions& options,
            const Eigen::Vector3d& axis,
            const Eigen::Vector3d& radialX,
            const Eigen::Vector3d& radialY,
            double sectorAngle)
        {
            std::vector<std::array<float, 4>> result;
            result.reserve(count * 2);
            for(std::size_t index = begin; index < begin + count; ++index) {
                const auto& sample = workpiece.samples[index];
                const double angle = angleAroundAxis(
                    sample.position, options.axisOrigin, axis, radialX, radialY);
                const double canonicalAngle = std::remainder(angle, sectorAngle);
                const double rotationAngle = canonicalAngle - angle;
                const Eigen::Vector3d canonicalPosition = canonicalizePosition(
                    sample.position, options.axisOrigin, axis, rotationAngle);
                Eigen::Vector3d canonicalNormal = canonicalizeVector(
                    sample.normal, axis, rotationAngle);
                if(canonicalNormal.norm() <= 1.0e-12) {
                    canonicalNormal = Eigen::Vector3d::UnitZ();
                } else {
                    canonicalNormal.normalize();
                }
                result.push_back({
                    static_cast<float>(canonicalPosition.x() * 1000.0),
                    static_cast<float>(canonicalPosition.y() * 1000.0),
                    static_cast<float>(canonicalPosition.z() * 1000.0),
                    0.0f
                });
                result.push_back({
                    static_cast<float>(canonicalNormal.x()),
                    static_cast<float>(canonicalNormal.y()),
                    static_cast<float>(canonicalNormal.z()),
                    0.0f
                });
            }
            return result;
        }

        void buildFullVertexBindingsGpu(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const LocalMesh& localMesh,
            const ThicknessBvh& localBvh,
            const PeriodicLocalPredictionOptions& options,
            const Eigen::Vector3d& axis,
            const Eigen::Vector3d& radialX,
            const Eigen::Vector3d& radialY,
            double sectorAngle,
            PeriodicSectorReduction& result,
            const PeriodicGpuMappingCallbacks& callbacks)
        {
            if(glGetString(GL_VERSION) == nullptr || glDispatchCompute == nullptr) {
                throw std::runtime_error(
                    "OpenGL compute shaders are unavailable for periodic mapping.");
            }

            const auto localSurface = makeGpuSurfaceData(
                localMesh.workpiece, 0, localMesh.workpiece.samples.size());
            GLint64 maximumBlockSize = 0;
            glGetInteger64v(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &maximumBlockSize);
            if(maximumBlockSize <= 0) {
                throw std::runtime_error(
                    "OpenGL did not report a valid shader storage buffer limit.");
            }
            const auto requireBufferCapacity = [maximumBlockSize](
                std::size_t bytes,
                const char* name) {
                if(bytes > static_cast<std::size_t>(maximumBlockSize)) {
                    throw std::runtime_error(
                        std::string("Periodic GPU mapping ") + name
                        + " exceeds GL_MAX_SHADER_STORAGE_BLOCK_SIZE.");
                }
            };
            requireBufferCapacity(
                localSurface.size() * sizeof(localSurface.front()),
                "local surface buffer");
            requireBufferCapacity(
                localMesh.workpiece.triangleIndices.size() * sizeof(std::uint32_t),
                "local triangle buffer");
            requireBufferCapacity(
                localBvh.nodes.size() * sizeof(GpuBvhNode),
                "local BVH buffer");
            requireBufferCapacity(
                localBvh.triangleOrder.size() * sizeof(std::uint32_t),
                "local BVH order buffer");
            requireBufferCapacity(
                localMesh.localToGlobal.size() * sizeof(std::uint32_t),
                "local-to-global buffer");

            constexpr std::size_t kPreferredBatchVertices = 1U << 20;
            const std::size_t blockLimitedBatch = static_cast<std::size_t>(
                maximumBlockSize) / (2 * sizeof(std::array<float, 4>));
            const std::size_t batchCapacity = std::max<std::size_t>(
                1,
                std::min({
                    kPreferredBatchVertices,
                    workpiece.samples.size(),
                    blockLimitedBatch
                }));
            const std::size_t batchCount =
                (workpiece.samples.size() + batchCapacity - 1) / batchCapacity;

            OpenGLComputeProgram program(
                kPeriodicSectorMappingShader,
                "Periodic sector mapping compute shader");
            GpuMappingBuffer fullSurfaceBuffer;
            GpuMappingBuffer localSurfaceBuffer;
            GpuMappingBuffer localTriangleBuffer;
            GpuMappingBuffer bvhBuffer;
            GpuMappingBuffer bvhOrderBuffer;
            GpuMappingBuffer localToGlobalBuffer;
            GpuMappingBuffer bindingIndexBuffer;
            GpuMappingBuffer bindingWeightBuffer;

            fullSurfaceBuffer.upload(
                0,
                nullptr,
                batchCapacity * 2 * sizeof(std::array<float, 4>),
                GL_DYNAMIC_DRAW);
            localSurfaceBuffer.upload(
                1,
                localSurface.data(),
                localSurface.size() * sizeof(localSurface.front()),
                GL_STATIC_DRAW);
            localTriangleBuffer.upload(
                2,
                localMesh.workpiece.triangleIndices.data(),
                localMesh.workpiece.triangleIndices.size() * sizeof(std::uint32_t),
                GL_STATIC_DRAW);
            bvhBuffer.upload(
                3,
                localBvh.nodes.data(),
                localBvh.nodes.size() * sizeof(GpuBvhNode),
                GL_STATIC_DRAW);
            bvhOrderBuffer.upload(
                4,
                localBvh.triangleOrder.data(),
                localBvh.triangleOrder.size() * sizeof(std::uint32_t),
                GL_STATIC_DRAW);
            localToGlobalBuffer.upload(
                5,
                localMesh.localToGlobal.data(),
                localMesh.localToGlobal.size() * sizeof(std::uint32_t),
                GL_STATIC_DRAW);
            bindingIndexBuffer.upload(
                6,
                nullptr,
                batchCapacity * sizeof(GpuBindingIndices),
                GL_DYNAMIC_READ);
            bindingWeightBuffer.upload(
                7,
                nullptr,
                batchCapacity * sizeof(GpuBindingWeights),
                GL_DYNAMIC_READ);

            program.use();
            program.setFloat(
                "minimumNormalDot",
                static_cast<float>(std::cos(
                    std::clamp(options.maximumNormalAngleDegrees, 0.0, 180.0)
                    * kPi / 180.0)));
            checkGpuMappingErrors("input upload");

            result.fullVertexBindings.resize(workpiece.samples.size());
            std::vector<GpuBindingIndices> gpuIndices(batchCapacity);
            std::vector<GpuBindingWeights> gpuWeights(batchCapacity);
            const double maximumMappingDistance = std::max(
                0.0, options.maximumMappingDistanceMeters);
            for(std::size_t batch = 0; batch < batchCount; ++batch) {
                if(callbacks.canceled && callbacks.canceled()) {
                    result.failureReason = "Periodic GPU mapping was canceled.";
                    result.fullVertexBindings.clear();
                    return;
                }
                const std::size_t begin = batch * batchCapacity;
                const std::size_t count = std::min(
                    batchCapacity, workpiece.samples.size() - begin);
                if(callbacks.progress) {
                    callbacks.progress(batch, batchCount, "GPU mapping");
                }
                const auto fullSurface = makeGpuCanonicalSurfaceData(
                    workpiece,
                    begin,
                    count,
                    options,
                    axis,
                    radialX,
                    radialY,
                    sectorAngle);
                fullSurfaceBuffer.update(
                    fullSurface.data(),
                    fullSurface.size() * sizeof(fullSurface.front()));
                program.setInt("vertexCount", static_cast<int>(count));
                glDispatchCompute(
                    static_cast<unsigned int>((count + 255) / 256),
                    1,
                    1);
                glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
                bindingIndexBuffer.download(
                    gpuIndices.data(), count * sizeof(GpuBindingIndices));
                bindingWeightBuffer.download(
                    gpuWeights.data(), count * sizeof(GpuBindingWeights));
                checkGpuMappingErrors("batch dispatch and readback");

                for(std::size_t localIndex = 0; localIndex < count; ++localIndex) {
                    const std::uint32_t status = gpuIndices[localIndex].values[3];
                    if(status != 1U) {
                        result.failureReason = status == 2U
                            ? "Periodic GPU mapping BVH traversal stack overflowed."
                            : "A full-model vertex could not be mapped on the GPU.";
                        result.fullVertexBindings.clear();
                        return;
                    }
                    const double mappingDistance =
                        static_cast<double>(gpuWeights[localIndex].values[3]) * 0.001;
                    result.maximumMappingDistance = std::max(
                        result.maximumMappingDistance, mappingDistance);
                    if(!std::isfinite(mappingDistance) ||
                        mappingDistance > maximumMappingDistance) {
                        result.failureReason =
                            "The GPU base-sector mapping distance exceeds the configured tolerance.";
                        result.fullVertexBindings.clear();
                        return;
                    }
                    const auto& indexValues = gpuIndices[localIndex].values;
                    const auto& weightValues = gpuWeights[localIndex].values;
                    const float weightSum =
                        weightValues[0] + weightValues[1] + weightValues[2];
                    if(indexValues[0] >= workpiece.samples.size() ||
                        indexValues[1] >= workpiece.samples.size() ||
                        indexValues[2] >= workpiece.samples.size() ||
                        !std::isfinite(weightValues[0]) ||
                        !std::isfinite(weightValues[1]) ||
                        !std::isfinite(weightValues[2]) ||
                        std::abs(weightSum - 1.0f) > 1.0e-3f ||
                        weightValues[0] < -1.0e-4f ||
                        weightValues[1] < -1.0e-4f ||
                        weightValues[2] < -1.0e-4f) {
                        result.failureReason =
                            "Periodic GPU mapping produced invalid interpolation bindings.";
                        result.fullVertexBindings.clear();
                        return;
                    }
                    PeriodicSectorBinding& binding =
                        result.fullVertexBindings[begin + localIndex];
                    binding.sourceVertexIndices = {
                        indexValues[0],
                        indexValues[1],
                        indexValues[2]
                    };
                    binding.weights = {
                        weightValues[0],
                        weightValues[1],
                        weightValues[2]
                    };
                }
                if(callbacks.progress) {
                    callbacks.progress(batch + 1, batchCount, "GPU mapping");
                }
            }

            if(callbacks.progress) {
                callbacks.progress(batchCount, batchCount, "validating GPU mapping");
            }
            constexpr std::size_t kMaximumValidationSamples = 2048;
            const std::size_t validationStride = std::max<std::size_t>(
                1,
                (workpiece.samples.size() + kMaximumValidationSamples - 1)
                    / kMaximumValidationSamples);
            const double validationTolerance = std::max(
                1.0e-6,
                maximumMappingDistance * 0.01);
            for(std::size_t vertex = 0;
                vertex < workpiece.samples.size();
                vertex += validationStride) {
                const auto& sample = workpiece.samples[vertex];
                const double angle = angleAroundAxis(
                    sample.position, options.axisOrigin, axis, radialX, radialY);
                const double canonicalAngle = std::remainder(angle, sectorAngle);
                const double rotationAngle = canonicalAngle - angle;
                const Eigen::Vector3d canonicalPosition = canonicalizePosition(
                    sample.position, options.axisOrigin, axis, rotationAngle);
                Eigen::Vector3d canonicalNormal = canonicalizeVector(
                    sample.normal, axis, rotationAngle);
                if(canonicalNormal.norm() <= 1.0e-12) {
                    canonicalNormal = Eigen::Vector3d::UnitZ();
                } else {
                    canonicalNormal.normalize();
                }
                const ClosestTriangle cpuClosest = findClosestTriangle(
                    localMesh,
                    localBvh,
                    canonicalPosition,
                    canonicalNormal,
                    std::cos(
                        std::clamp(options.maximumNormalAngleDegrees, 0.0, 180.0)
                        * kPi / 180.0));
                if(!cpuClosest.found) {
                    result.failureReason =
                        "CPU validation could not map a sampled periodic vertex.";
                    result.fullVertexBindings.clear();
                    return;
                }
                const PeriodicSectorBinding& gpuBinding =
                    result.fullVertexBindings[vertex];
                const Eigen::Vector3d gpuPoint =
                    static_cast<double>(gpuBinding.weights[0])
                        * workpiece.samples[gpuBinding.sourceVertexIndices[0]].position
                    + static_cast<double>(gpuBinding.weights[1])
                        * workpiece.samples[gpuBinding.sourceVertexIndices[1]].position
                    + static_cast<double>(gpuBinding.weights[2])
                        * workpiece.samples[gpuBinding.sourceVertexIndices[2]].position;
                const double gpuDistance = (canonicalPosition - gpuPoint).norm();
                const double cpuDistance = std::sqrt(cpuClosest.squaredDistance);
                if(!std::isfinite(gpuDistance) ||
                    gpuDistance > cpuDistance + validationTolerance) {
                    result.failureReason =
                        "Periodic GPU mapping exceeded the CPU validation tolerance.";
                    result.fullVertexBindings.clear();
                    return;
                }
            }
        }
    }

    namespace
    {
        PeriodicSectorReduction buildPeriodicSectorReductionImpl(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const PeriodicLocalPredictionOptions& options,
            bool buildFullVertexBindings,
            bool useGpu,
            const PeriodicGpuMappingCallbacks& callbacks)
    {
        PeriodicSectorReduction result;
        if(!options.enabled) {
            result.failureReason = "Periodic local prediction is disabled.";
            return result;
        }
        if(options.sectorCount < 2) {
            result.failureReason = "Periodic local prediction requires at least two sectors.";
            return result;
        }
        if(workpiece.samples.empty() || workpiece.triangleIndices.empty()) {
            result.failureReason = "Periodic local prediction requires a triangle mesh.";
            return result;
        }

        Eigen::Vector3d axis = options.axisDirection;
        if(!axis.allFinite() || axis.norm() <= 1.0e-12) {
            result.failureReason = "The periodic rotation axis direction is invalid.";
            return result;
        }
        axis.normalize();
        Eigen::Vector3d radialX = options.referenceDirection
            - options.referenceDirection.dot(axis) * axis;
        if(!radialX.allFinite() || radialX.norm() <= 1.0e-12) {
            const Eigen::Vector3d fallback = std::abs(axis.x()) < 0.9
                ? Eigen::Vector3d::UnitX()
                : Eigen::Vector3d::UnitY();
            radialX = fallback - fallback.dot(axis) * axis;
        }
        radialX.normalize();
        const Eigen::Vector3d radialY = axis.cross(radialX).normalized();
        const double sectorAngle = kTwoPi / static_cast<double>(options.sectorCount);
        const double halfSectorAngle = sectorAngle * 0.5;

        const std::uint64_t cacheKey = periodicCacheKey(
            workpiece, options, axis, radialX);
        if(buildFullVertexBindings) {
            {
                std::lock_guard<std::mutex> lock(g_periodicCacheMutex);
                const auto cached = g_periodicCache.find(cacheKey);
                if(cached != g_periodicCache.end()) {
                    result = cached->second;
                    result.mappingCacheHit = true;
                    return result;
                }
            }

            PeriodicSectorReduction diskCached;
            if(loadPeriodicCache(cacheKey, workpiece, diskCached)) {
                {
                    std::lock_guard<std::mutex> lock(g_periodicCacheMutex);
                    g_periodicCache[cacheKey] = diskCached;
                }
                diskCached.mappingCacheHit = true;
                return diskCached;
            }
        }

        LocalMesh localMesh = buildLocalMesh(
            workpiece,
            options.axisOrigin,
            axis,
            radialX,
            radialY,
            halfSectorAngle);
        if(localMesh.workpiece.triangleIndices.empty()) {
            result.failureReason = "The base periodic sector contains no mesh triangles.";
            return result;
        }

        result.predictionVertexIndices = localMesh.localToGlobal;
        result.predictionTriangleIndices = localMesh.globalTriangleIndices;
        result.localTriangleCount = localMesh.workpiece.triangleIndices.size() / 3;
        Eigen::Vector3d minimum = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::max());
        Eigen::Vector3d maximum = Eigen::Vector3d::Constant(
            std::numeric_limits<double>::lowest());
        for(const std::uint32_t globalIndex : result.predictionVertexIndices) {
            const Eigen::Vector3d& position = workpiece.samples[globalIndex].position;
            minimum = minimum.cwiseMin(position);
            maximum = maximum.cwiseMax(position);
        }
        result.localBoundsCenter = (minimum + maximum) * 0.5;
        for(const std::uint32_t globalIndex : result.predictionVertexIndices) {
            result.localBoundsRadius = std::max(
                result.localBoundsRadius,
                (workpiece.samples[globalIndex].position - result.localBoundsCenter).norm());
        }
        if(!buildFullVertexBindings) {
            return result;
        }

        const ThicknessBvh localBvh = ThicknessBvhBuilder::build(localMesh.workpiece);
        if(localBvh.empty()) {
            result.failureReason = "Failed to build the base-sector mapping BVH.";
            return result;
        }

        const double minimumNormalDot = std::cos(
            std::clamp(options.maximumNormalAngleDegrees, 0.0, 180.0) * kPi / 180.0);
        const double maximumMappingDistance = std::max(
            0.0, options.maximumMappingDistanceMeters);
        if(useGpu) {
            buildFullVertexBindingsGpu(
                workpiece,
                localMesh,
                localBvh,
                options,
                axis,
                radialX,
                radialY,
                sectorAngle,
                result,
                callbacks);
            if(!result.failureReason.empty()) {
                return result;
            }
        } else {
            result.fullVertexBindings.reserve(workpiece.samples.size());
            for(const auto& sample : workpiece.samples) {
                const double angle = angleAroundAxis(
                    sample.position, options.axisOrigin, axis, radialX, radialY);
                const double canonicalAngle = std::remainder(angle, sectorAngle);
                const double rotationAngle = canonicalAngle - angle;
                const Eigen::Vector3d canonicalPosition = canonicalizePosition(
                    sample.position, options.axisOrigin, axis, rotationAngle);
                Eigen::Vector3d canonicalNormal = canonicalizeVector(
                    sample.normal, axis, rotationAngle);
                if(canonicalNormal.norm() <= 1.0e-12) {
                    canonicalNormal = Eigen::Vector3d::UnitZ();
                } else {
                    canonicalNormal.normalize();
                }

                const ClosestTriangle closest = findClosestTriangle(
                    localMesh, localBvh, canonicalPosition, canonicalNormal, minimumNormalDot);
                if(!closest.found) {
                    result.failureReason =
                        "A full-model vertex could not be mapped to the base periodic sector.";
                    result.fullVertexBindings.clear();
                    return result;
                }
                const double mappingDistance = std::sqrt(closest.squaredDistance);
                result.maximumMappingDistance = std::max(
                    result.maximumMappingDistance, mappingDistance);
                if(mappingDistance > maximumMappingDistance) {
                    result.failureReason =
                        "The base-sector mapping distance exceeds the configured tolerance.";
                    result.fullVertexBindings.clear();
                    return result;
                }
                PeriodicSectorBinding binding;
                binding.sourceVertexIndices = closest.indices;
                binding.weights = closest.weights;
                result.fullVertexBindings.push_back(binding);
            }
        }
        // Prefer an exact source vertex whenever a periodic copy lands on a
        // vertex evaluated in the base sector. This avoids crossing a sharp
        // feature through nearest-triangle interpolation.
        applyExactVertexBindings(
            workpiece,
            localMesh,
            options,
            axis,
            radialX,
            radialY,
            sectorAngle,
            minimumNormalDot,
            result);
        savePeriodicCache(cacheKey, workpiece, result);
        {
            std::lock_guard<std::mutex> lock(g_periodicCacheMutex);
            g_periodicCache[cacheKey] = result;
        }
        return result;
    }
    }

    PeriodicSectorReduction buildPeriodicSectorReduction(
        const sprayworkpiece::WorkpieceModel& workpiece,
        const PeriodicLocalPredictionOptions& options,
        bool buildFullVertexBindings)
    {
        return buildPeriodicSectorReductionImpl(
            workpiece,
            options,
            buildFullVertexBindings,
            false,
            {});
    }

    PeriodicSectorReduction buildPeriodicSectorReductionGpu(
        const sprayworkpiece::WorkpieceModel& workpiece,
        const PeriodicLocalPredictionOptions& options,
        const PeriodicGpuMappingCallbacks& callbacks)
    {
        return buildPeriodicSectorReductionImpl(
            workpiece,
            options,
            true,
            true,
            callbacks);
    }

    std::vector<PeriodicSpraySample> makePeriodicSpraySamples(
        const ThicknessPredictionTask& task)
    {
        const auto trajectorySamples = task.options.base.trajectorySamplingMode
                == TrajectorySamplingMode::OriginalPoints
            ? spraytrajectory::SprayTrajectorySampler::originalSamples(task.trajectory)
            : spraytrajectory::SprayTrajectorySampler::sample(
                task.trajectory,
                task.options.base.timeStep);
        std::vector<PeriodicSpraySample> result;
        if(trajectorySamples.empty()) {
            return result;
        }

        result.reserve(trajectorySamples.size());
        for(std::size_t index = 0; index < trajectorySamples.size(); ++index) {
            const auto& sample = trajectorySamples[index];
            if(!sample.sprayEnabled) {
                continue;
            }
            if(!sample.processId.empty() && !task.process.id.empty()
                && sample.processId != task.process.id) {
                continue;
            }

            const double duration = index + 1 < trajectorySamples.size()
                ? std::max(0.0, trajectorySamples[index + 1].time - sample.time)
                : 0.0;
            if(duration <= 0.0) {
                continue;
            }

            const Eigen::Isometry3d toolPose = sample.tcpPose * task.tool.T_link_tool;
            const Eigen::Vector3d direction =
                task.tool.worldSprayDirection(toolPose).normalized();
            const Eigen::Vector3d preferredMajor =
                toolPose.linear() * task.tool.powderFeedDirectionLocal;
            const Eigen::Vector3d major = orthogonalAxis(
                preferredMajor, direction, toolPose.linear().col(1));
            Eigen::Vector3d minor = direction.cross(major);
            if(minor.norm() <= 1.0e-12) {
                minor = orthogonalAxis(
                    toolPose.linear().col(1), direction, Eigen::Vector3d::UnitY());
            } else {
                minor.normalize();
            }

            PeriodicSpraySample resultSample;
            resultSample.position = toolPose.translation();
            resultSample.direction = direction;
            resultSample.majorAxis = major;
            resultSample.minorAxis = minor;
            resultSample.time = sample.time;
            resultSample.duration = duration;
            result.push_back(resultSample);
        }
        return result;
    }

    std::vector<std::size_t> selectPeriodicSpraySampleIndices(
        const std::vector<PeriodicSpraySample>& samples,
        const PeriodicSectorReduction& reduction,
        const AdvancedThicknessPredictionOptions& options)
    {
        const double cutoff = std::clamp(
            options.periodicLocal.contributionCutoffRatio,
            1.0e-12,
            0.999999);
        const double gaussianRadius = std::sqrt(-2.0 * std::log(cutoff));
        const double maximumSigma = std::max(
            std::abs(options.deposition.sigmaPhiRadians),
            std::abs(options.deposition.sigmaPsiRadians));
        const double maximumOffset = std::max(
            std::abs(options.deposition.phiOffsetRadians),
            std::abs(options.deposition.psiOffsetRadians));
        const double halfAngle = std::clamp(
            maximumOffset + gaussianRadius * maximumSigma,
            1.0e-6,
            1.5533430342749532);
        const double coneSlope = std::tan(halfAngle);

        std::vector<std::size_t> selected;
        selected.reserve(samples.size());
        for(std::size_t index = 0; index < samples.size(); ++index) {
            const PeriodicSpraySample& sample = samples[index];
            Eigen::Vector3d direction = sample.direction;
            if(direction.norm() <= 1.0e-12) {
                continue;
            }
            direction.normalize();
            const Eigen::Vector3d toCenter = reduction.localBoundsCenter - sample.position;
            const double axialDistance = toCenter.dot(direction);
            if(axialDistance + reduction.localBoundsRadius <= 0.0) {
                continue;
            }
            const double lateralDistance = (
                toCenter - axialDistance * direction).norm();
            const double allowedLateralDistance = reduction.localBoundsRadius
                + std::max(0.0, axialDistance) * coneSlope;
            if(lateralDistance <= allowedLateralDistance) {
                selected.push_back(index);
            }
        }
        return selected;
    }

    void expandPeriodicSectorThickness(
        const PeriodicSectorReduction& reduction,
        const std::vector<float>& localThicknessByFullVertex,
        std::vector<float>& expandedThickness)
    {
        expandedThickness.assign(reduction.fullVertexBindings.size(), 0.0f);
        // Vertices in the base sector were evaluated directly by the GPU.
        // Preserve those values instead of sending them through the
        // nearest-triangle interpolation used for the other vertices.
        std::vector<std::uint8_t> directlyComputed(expandedThickness.size(), 0U);
        for(const std::uint32_t vertex : reduction.predictionVertexIndices) {
            if(vertex < directlyComputed.size()) {
                directlyComputed[vertex] = 1U;
            }
        }
        for(std::size_t vertex = 0; vertex < reduction.fullVertexBindings.size(); ++vertex) {
            if(directlyComputed[vertex] != 0U
                && vertex < localThicknessByFullVertex.size()) {
                expandedThickness[vertex] = localThicknessByFullVertex[vertex];
                continue;
            }
            const PeriodicSectorBinding& binding = reduction.fullVertexBindings[vertex];
            float value = 0.0f;
            for(std::size_t corner = 0; corner < 3; ++corner) {
                const std::uint32_t sourceIndex = binding.sourceVertexIndices[corner];
                if(sourceIndex < localThicknessByFullVertex.size()) {
                    value += binding.weights[corner] * localThicknessByFullVertex[sourceIndex];
                }
            }
            expandedThickness[vertex] = value;
        }
    }
}
