#include <SprayThicknessPredictionOpenGL/OpenGLThicknessPredictionBackend.h>

#include <SprayThicknessPredictionOpenGL/ThicknessBvh.h>
#include <SprayThicknessPredictionOpenGL/AxisymmetricProfileReduction.h>

#include "OpenGLComputeProgram.h"
#include "PaperGaussianHistoryShader.h"
#include "PeriodicSectorReduction.h"
#include "SpatialFilteredGaussianHistoryShader.h"
#include <SprayThicknessPredictionOpenGL/SpatialInfluenceFiltering.h>

#include <glad/glad.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <fstream>
#include <filesystem>
#include <unordered_map>
#include <stdexcept>
#include <thread>
#include <vector>
#ifdef _MSC_VER
#include <execution>
#endif

namespace spraythickness::opengl
{
    namespace
    {
        constexpr float kMetersToMillimeters = 1000.0f;
        constexpr double kMillimetersToMeters = 1.0e-3;
        constexpr std::uint64_t kFnvOffset = 14695981039346656037ull;
        constexpr std::uint64_t kFnvPrime = 1099511628211ull;
        constexpr std::uint64_t kBvhCacheMagic = 0x5253323032364256ull;
        constexpr std::uint32_t kBvhCacheVersion = 3;
        constexpr std::uint64_t kBatchProfileCacheMagic = 0x5253323032364250ull;
        constexpr std::uint32_t kBatchProfileCacheVersion = 1;
        constexpr std::uint64_t kAxisymmetricMappingCacheMagic = 0x5253323032364158ull;
        // The mapper now stores the selected profile segment and compares
        // normals in the meridian reference plane. Invalidate older bindings.
        constexpr std::uint32_t kAxisymmetricMappingCacheVersion = 3;
        constexpr std::uint64_t kInitialTargetVertexSprayPairs =
            128ull * 1024ull * 1024ull;

        double squaredDistanceToProfileSegment(
            const Eigen::Vector2d& point,
            const Eigen::Vector2d& first,
            const Eigen::Vector2d& second,
            double& interpolation)
        {
            const Eigen::Vector2d edge = second - first;
            const double squaredLength = edge.squaredNorm();
            interpolation = squaredLength > 1.0e-18
                ? std::clamp((point - first).dot(edge) / squaredLength, 0.0, 1.0)
                : 0.0;
            return (point - (first + interpolation * edge)).squaredNorm();
        }

        using AxisymmetricBindingVector =
            std::vector<spraythickness::AxisymmetricProfileBinding>;

        struct AxisymmetricProfileBvhNode
        {
            Eigen::Vector2d minimum = Eigen::Vector2d::Zero();
            Eigen::Vector2d maximum = Eigen::Vector2d::Zero();
            std::uint32_t first{ 0 };
            std::uint32_t count{ 0 };
            std::uint32_t left{ std::numeric_limits<std::uint32_t>::max() };
            std::uint32_t right{ std::numeric_limits<std::uint32_t>::max() };

            bool leaf() const
            {
                return left == std::numeric_limits<std::uint32_t>::max();
            }
        };

        class AxisymmetricProfileBvh
        {
        public:
            explicit AxisymmetricProfileBvh(
                const std::vector<spraythickness::AxisymmetricProfileSampleSegment>& segments,
                const std::vector<sprayworkpiece::SurfaceSample>& profileSamples)
                : m_segments(segments)
                , m_profileSamples(profileSamples)
            {
                m_segmentOrder.resize(segments.size());
                std::iota(m_segmentOrder.begin(), m_segmentOrder.end(), 0u);
                if(!m_segmentOrder.empty()) {
                    buildNode(0, m_segmentOrder.size());
                }
            }

            spraythickness::AxisymmetricProfileBinding nearest(
                const Eigen::Vector2d& point,
                const Eigen::Vector3d& queryNormal) const
            {
                spraythickness::AxisymmetricProfileBinding binding;
                if(m_nodes.empty()) {
                    return binding;
                }

                double bestDistance = std::numeric_limits<double>::max();
                double bestNormalAlignment = -1.0;
                const Eigen::Vector3d queryNormalSafe = queryNormal.squaredNorm()
                    > 1.0e-16
                    ? queryNormal.normalized()
                    : Eigen::Vector3d::UnitZ();
                std::uint32_t bestSegment = std::numeric_limits<std::uint32_t>::max();
                std::array<std::uint32_t, 64> pending{};
                std::size_t pendingCount = 1;
                pending[0] = 0;
                while(pendingCount > 0) {
                    const std::uint32_t nodeIndex = pending[--pendingCount];
                    const AxisymmetricProfileBvhNode& node = m_nodes[nodeIndex];
                    if(squaredDistanceToBounds(point, node.minimum, node.maximum)
                        > bestDistance) {
                        continue;
                    }
                    if(node.leaf()) {
                        for(std::uint32_t index = 0; index < node.count; ++index) {
                            const std::uint32_t segmentIndex = m_segmentOrder[node.first + index];
                            const auto& segment = m_segments[segmentIndex];
                            double interpolation = 0.0;
                            const double distance = squaredDistanceToProfileSegment(
                                point,
                                segment.firstSectionPosition,
                                segment.secondSectionPosition,
                                interpolation);
                            const auto& firstSample = m_profileSamples[
                                segment.firstSampleIndex];
                            const auto& secondSample = m_profileSamples[
                                segment.secondSampleIndex];
                            const Eigen::Vector3d profileNormalRaw = firstSample.normal
                                + interpolation * (secondSample.normal - firstSample.normal);
                            const Eigen::Vector3d profileNormal = profileNormalRaw.squaredNorm()
                                > 1.0e-16
                                ? profileNormalRaw.normalized()
                                : Eigen::Vector3d::UnitZ();
                            const double normalAlignment = profileNormal.dot(queryNormalSafe);
                            const double distanceTieTolerance = std::max(
                                1.0e-16,
                                bestDistance * 1.0e-4 + 1.0e-12);
                            const bool closer = distance + distanceTieTolerance < bestDistance;
                            const bool sameDistance = std::abs(distance - bestDistance)
                                <= distanceTieTolerance;
                            if((!closer && !sameDistance)
                                || (sameDistance && normalAlignment < bestNormalAlignment)
                                || (sameDistance && normalAlignment == bestNormalAlignment
                                    && segmentIndex >= bestSegment)) {
                                continue;
                            }
                            bestDistance = distance;
                            bestNormalAlignment = normalAlignment;
                            bestSegment = segmentIndex;
                            binding.firstSampleIndex = segment.firstSampleIndex;
                            binding.secondSampleIndex = segment.secondSampleIndex;
                            binding.segmentIndex = segmentIndex;
                            binding.interpolation = static_cast<float>(interpolation);
                            binding.active = true;
                        }
                        continue;
                    }

                    const AxisymmetricProfileBvhNode& left = m_nodes[node.left];
                    const AxisymmetricProfileBvhNode& right = m_nodes[node.right];
                    const double leftDistance = squaredDistanceToBounds(
                        point, left.minimum, left.maximum);
                    const double rightDistance = squaredDistanceToBounds(
                        point, right.minimum, right.maximum);
                    if(leftDistance <= rightDistance) {
                        if(rightDistance <= bestDistance) {
                            pending[pendingCount++] = node.right;
                        }
                        if(leftDistance <= bestDistance) {
                            pending[pendingCount++] = node.left;
                        }
                    } else {
                        if(leftDistance <= bestDistance) {
                            pending[pendingCount++] = node.left;
                        }
                        if(rightDistance <= bestDistance) {
                            pending[pendingCount++] = node.right;
                        }
                    }
                }
                return binding;
            }

        private:
            static constexpr std::size_t kLeafSegmentCount = 8;

            static double squaredDistanceToBounds(
                const Eigen::Vector2d& point,
                const Eigen::Vector2d& minimum,
                const Eigen::Vector2d& maximum)
            {
                const Eigen::Vector2d delta = (minimum - point).cwiseMax(0.0)
                    + (point - maximum).cwiseMax(0.0);
                return delta.squaredNorm();
            }

            AxisymmetricProfileBvhNode boundsForRange(
                std::size_t first,
                std::size_t count) const
            {
                AxisymmetricProfileBvhNode node;
                node.minimum = Eigen::Vector2d::Constant(
                    std::numeric_limits<double>::max());
                node.maximum = Eigen::Vector2d::Constant(
                    std::numeric_limits<double>::lowest());
                for(std::size_t index = first; index < first + count; ++index) {
                    const auto& segment = m_segments[m_segmentOrder[index]];
                    node.minimum = node.minimum.cwiseMin(segment.firstSectionPosition)
                        .cwiseMin(segment.secondSectionPosition);
                    node.maximum = node.maximum.cwiseMax(segment.firstSectionPosition)
                        .cwiseMax(segment.secondSectionPosition);
                }
                return node;
            }

            std::uint32_t buildNode(std::size_t first, std::size_t count)
            {
                const std::uint32_t nodeIndex = static_cast<std::uint32_t>(m_nodes.size());
                m_nodes.push_back(boundsForRange(first, count));
                AxisymmetricProfileBvhNode& node = m_nodes[nodeIndex];
                if(count <= kLeafSegmentCount) {
                    node.first = static_cast<std::uint32_t>(first);
                    node.count = static_cast<std::uint32_t>(count);
                    return nodeIndex;
                }

                const Eigen::Vector2d extent = node.maximum - node.minimum;
                const int axis = extent.x() >= extent.y() ? 0 : 1;
                const std::size_t middle = first + count / 2;
                std::nth_element(
                    m_segmentOrder.begin() + static_cast<std::ptrdiff_t>(first),
                    m_segmentOrder.begin() + static_cast<std::ptrdiff_t>(middle),
                    m_segmentOrder.begin() + static_cast<std::ptrdiff_t>(first + count),
                    [this, axis](std::uint32_t firstSegment, std::uint32_t secondSegment) {
                        const auto& first = m_segments[firstSegment];
                        const auto& second = m_segments[secondSegment];
                        const double firstCenter = (first.firstSectionPosition[axis]
                            + first.secondSectionPosition[axis]) * 0.5;
                        const double secondCenter = (second.firstSectionPosition[axis]
                            + second.secondSectionPosition[axis]) * 0.5;
                        return firstCenter == secondCenter
                            ? firstSegment < secondSegment
                            : firstCenter < secondCenter;
                    });
                m_nodes[nodeIndex].left = buildNode(first, middle - first);
                m_nodes[nodeIndex].right = buildNode(middle, first + count - middle);
                return nodeIndex;
            }

            const std::vector<spraythickness::AxisymmetricProfileSampleSegment>& m_segments;
            const std::vector<sprayworkpiece::SurfaceSample>& m_profileSamples;
            std::vector<std::uint32_t> m_segmentOrder;
            std::vector<AxisymmetricProfileBvhNode> m_nodes;
        };

        template<typename Function>
        void parallelForVertexRanges(std::size_t count, Function&& function)
        {
            constexpr std::size_t kMinimumVerticesPerWorker = 65536;
            const std::size_t byWork = std::max<std::size_t>(
                1,
                (count + kMinimumVerticesPerWorker - 1) / kMinimumVerticesPerWorker);
            const std::size_t hardware = std::max<std::size_t>(
                1,
                static_cast<std::size_t>(std::thread::hardware_concurrency()));
            const std::size_t workerCount = std::min<std::size_t>(
                std::min<std::size_t>(hardware, 16), byWork);
            if(workerCount == 1) {
                function(0, count);
                return;
            }

            std::vector<std::thread> workers;
            workers.reserve(workerCount);
            const std::size_t verticesPerWorker = (count + workerCount - 1) / workerCount;
            for(std::size_t worker = 0; worker < workerCount; ++worker) {
                const std::size_t first = worker * verticesPerWorker;
                const std::size_t last = std::min(count, first + verticesPerWorker);
                if(first >= last) {
                    continue;
                }
                workers.emplace_back([first, last, &function]() { function(first, last); });
            }
            for(std::thread& worker : workers) {
                worker.join();
            }
        }

        AxisymmetricBindingVector buildAxisymmetricProfileBindings(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const spraythickness::AxisymmetricProfilePredictionOptions& profile)
        {
            AxisymmetricBindingVector bindings(workpiece.samples.size());
            if(profile.sampleSegments.empty()) {
                return bindings;
            }
            const AxisymmetricProfileBvh segmentBvh(
                profile.sampleSegments,
                profile.predictionSamples);
            const Eigen::Vector3d axisOrigin = profile.axisOrigin;
            const Eigen::Vector3d axisDirection = profile.axisDirection.normalized();
            Eigen::Vector3d radialDirection = profile.radialDirection
                - profile.radialDirection.dot(axisDirection) * axisDirection;
            radialDirection = radialDirection.squaredNorm() > 1.0e-16
                ? radialDirection.normalized()
                : (std::abs(axisDirection.x()) < 0.9
                    ? Eigen::Vector3d::UnitX()
                    : Eigen::Vector3d::UnitY());
            Eigen::Vector3d tangentialDirection = axisDirection.cross(radialDirection);
            tangentialDirection = tangentialDirection.squaredNorm() > 1.0e-16
                ? tangentialDirection.normalized()
                : Eigen::Vector3d::UnitZ();
            AxisymmetricProfileSelection selection;
            selection.enabled = true;
            selection.minimum = profile.selectionMinimum;
            selection.maximum = profile.selectionMaximum;
            selection.polygon = profile.selectionPolygon;
            parallelForVertexRanges(bindings.size(),
                [&workpiece, &bindings, &segmentBvh, axisOrigin, axisDirection,
                    radialDirection, tangentialDirection, selection](
                    std::size_t first, std::size_t last) {
                    for(std::size_t vertex = first; vertex < last; ++vertex) {
                        const Eigen::Vector3d relative =
                            workpiece.samples[vertex].position - axisOrigin;
                        const double axial = relative.dot(axisDirection);
                        const Eigen::Vector3d radial = relative - axial * axisDirection;
                        const Eigen::Vector2d sectionPoint(radial.norm(), axial);
                        if(!selection.contains(sectionPoint)) {
                            continue;
                        }

                        // Profile normals are expressed in the reference
                        // meridian. Rotate each full-model normal back to that
                        // meridian before comparing candidate branches. A
                        // direct 3D dot product would make the same cylinder
                        // choose different branches at different azimuths.
                        const double radialReference = radial.dot(radialDirection);
                        const double tangentialReference = radial.dot(tangentialDirection);
                        const double radialLengthSquared = radial.squaredNorm();
                        const double azimuth = radialLengthSquared > 1.0e-16
                            ? std::atan2(tangentialReference, radialReference)
                            : 0.0;
                        const Eigen::Vector3d canonicalNormal =
                            Eigen::AngleAxisd(-azimuth, axisDirection)
                                * workpiece.samples[vertex].normal;
                        bindings[vertex] = segmentBvh.nearest(
                            sectionPoint,
                            canonicalNormal);
                    }
                });
            return bindings;
        }

        void expandAxisymmetricProfileThickness(
            const AxisymmetricBindingVector& bindings,
            const std::vector<float>& profileThickness,
            std::size_t profileOffset,
            std::vector<float>& expandedThickness)
        {
            expandedThickness.assign(bindings.size(), 0.0f);
            parallelForVertexRanges(bindings.size(),
                [&bindings, &profileThickness, profileOffset,
                    &expandedThickness](std::size_t first, std::size_t last) {
                    for(std::size_t vertex = first; vertex < last; ++vertex) {
                        const auto& binding = bindings[vertex];
                        if(!binding.active) {
                            continue;
                        }
                        const std::size_t firstSample = profileOffset + binding.firstSampleIndex;
                        const std::size_t secondSample = profileOffset + binding.secondSampleIndex;
                        if(firstSample >= profileThickness.size()
                            || secondSample >= profileThickness.size()) {
                            continue;
                        }
                        const float interpolation = std::clamp(
                            binding.interpolation, 0.0f, 1.0f);
                        expandedThickness[vertex] =
                            (1.0f - interpolation) * profileThickness[firstSample]
                            + interpolation * profileThickness[secondSample];
                    }
                });
        }

        struct BvhCacheHeader
        {
            std::uint64_t magic = kBvhCacheMagic;
            std::uint32_t version = kBvhCacheVersion;
            std::uint32_t nodeSize = sizeof(GpuBvhNode);
            std::uint64_t geometryHash = 0;
            std::uint64_t nodeCount = 0;
            std::uint64_t triangleCount = 0;
        };

        std::uint64_t hashWorkpieceGeometry(
            const sprayworkpiece::WorkpieceModel& workpiece)
        {
            std::uint64_t hash = kFnvOffset;
            const auto append = [&hash](const void* data, std::size_t size) {
                const auto* bytes = static_cast<const std::uint8_t*>(data);
                for(std::size_t index = 0; index < size; ++index) {
                    hash ^= bytes[index];
                    hash *= kFnvPrime;
                }
            };

            for(const auto& sample : workpiece.samples) {
                append(&sample.position.x(), sizeof(double));
                append(&sample.position.y(), sizeof(double));
                append(&sample.position.z(), sizeof(double));
                append(&sample.normal.x(), sizeof(double));
                append(&sample.normal.y(), sizeof(double));
                append(&sample.normal.z(), sizeof(double));
            }
            if(!workpiece.triangleIndices.empty()) {
                append(
                    workpiece.triangleIndices.data(),
                    workpiece.triangleIndices.size() * sizeof(std::uint32_t));
            }
            return hash;
        }

        std::uint64_t hashAxisymmetricProfileMapping(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const spraythickness::AxisymmetricProfilePredictionOptions& profile)
        {
            std::uint64_t hash = hashWorkpieceGeometry(workpiece);
            const auto append = [&hash](const void* data, std::size_t size) {
                const auto* bytes = static_cast<const std::uint8_t*>(data);
                for(std::size_t index = 0; index < size; ++index) {
                    hash ^= bytes[index];
                    hash *= kFnvPrime;
                }
            };
            append(profile.axisOrigin.data(), sizeof(double) * 3);
            append(profile.axisDirection.data(), sizeof(double) * 3);
            append(profile.selectionMinimum.data(), sizeof(double) * 2);
            append(profile.selectionMaximum.data(), sizeof(double) * 2);
            const std::size_t polygonSize = profile.selectionPolygon.size();
            append(&polygonSize, sizeof(polygonSize));
            for(const Eigen::Vector2d& point : profile.selectionPolygon) {
                append(point.data(), sizeof(double) * 2);
            }
            for(const auto& segment : profile.sampleSegments) {
                append(&segment.firstSampleIndex, sizeof(segment.firstSampleIndex));
                append(&segment.secondSampleIndex, sizeof(segment.secondSampleIndex));
                append(&segment.profilePathIndex, sizeof(segment.profilePathIndex));
                append(segment.firstSectionPosition.data(), sizeof(double) * 2);
                append(segment.secondSectionPosition.data(), sizeof(double) * 2);
            }
            return hash;
        }

        std::filesystem::path findProjectRoot();

        using AxisymmetricBindingCache =
            std::unordered_map<std::uint64_t,
                std::shared_ptr<const AxisymmetricBindingVector>>;

        struct AxisymmetricMappingCacheHeader
        {
            std::uint64_t magic{ kAxisymmetricMappingCacheMagic };
            std::uint32_t version{ kAxisymmetricMappingCacheVersion };
            std::uint32_t bindingSize{ 0 };
            std::uint64_t key{ 0 };
            std::uint64_t bindingCount{ 0 };
        };

        struct AxisymmetricMappingDiskBinding
        {
            std::uint32_t firstSampleIndex{ 0 };
            std::uint32_t secondSampleIndex{ 0 };
            std::uint32_t segmentIndex{ 0 };
            float interpolation{ 0.0f };
            std::uint32_t active{ 0 };
        };

        std::filesystem::path axisymmetricMappingCachePath(std::uint64_t key)
        {
            const std::filesystem::path root = findProjectRoot();
            if(root.empty()) {
                return {};
            }
            return root / ".cache" / "thickness_axisymmetric_mapping" /
                (std::to_string(key) + ".bin");
        }

        std::shared_ptr<const AxisymmetricBindingVector> loadAxisymmetricMappingCache(
            std::uint64_t key,
            std::size_t expectedBindingCount)
        {
            const std::filesystem::path path = axisymmetricMappingCachePath(key);
            if(path.empty() || !std::filesystem::exists(path)) {
                return {};
            }
            std::ifstream input(path, std::ios::binary);
            AxisymmetricMappingCacheHeader header;
            if(!input.read(reinterpret_cast<char*>(&header), sizeof(header))
                || header.magic != kAxisymmetricMappingCacheMagic
                || header.version != kAxisymmetricMappingCacheVersion
                || header.bindingSize != sizeof(AxisymmetricMappingDiskBinding)
                || header.key != key
                || header.bindingCount != expectedBindingCount) {
                return {};
            }
            std::vector<AxisymmetricMappingDiskBinding> diskBindings(expectedBindingCount);
            if(!input.read(
                reinterpret_cast<char*>(diskBindings.data()),
                static_cast<std::streamsize>(
                    diskBindings.size() * sizeof(AxisymmetricMappingDiskBinding)))) {
                return {};
            }
            auto bindings = std::make_shared<AxisymmetricBindingVector>(expectedBindingCount);
            for(std::size_t index = 0; index < expectedBindingCount; ++index) {
                (*bindings)[index].firstSampleIndex = diskBindings[index].firstSampleIndex;
                (*bindings)[index].secondSampleIndex = diskBindings[index].secondSampleIndex;
                (*bindings)[index].segmentIndex = diskBindings[index].segmentIndex;
                (*bindings)[index].interpolation = diskBindings[index].interpolation;
                (*bindings)[index].active = diskBindings[index].active != 0;
            }
            return bindings;
        }

        void saveAxisymmetricMappingCache(
            std::uint64_t key,
            const AxisymmetricBindingVector& bindings)
        {
            const std::filesystem::path path = axisymmetricMappingCachePath(key);
            if(path.empty() || bindings.empty()) {
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
            AxisymmetricMappingCacheHeader header;
            header.bindingSize = sizeof(AxisymmetricMappingDiskBinding);
            header.key = key;
            header.bindingCount = bindings.size();
            output.write(reinterpret_cast<const char*>(&header), sizeof(header));
            std::vector<AxisymmetricMappingDiskBinding> diskBindings(bindings.size());
            for(std::size_t index = 0; index < bindings.size(); ++index) {
                diskBindings[index].firstSampleIndex = bindings[index].firstSampleIndex;
                diskBindings[index].secondSampleIndex = bindings[index].secondSampleIndex;
                diskBindings[index].segmentIndex = bindings[index].segmentIndex;
                diskBindings[index].interpolation = bindings[index].interpolation;
                diskBindings[index].active = bindings[index].active ? 1u : 0u;
            }
            output.write(
                reinterpret_cast<const char*>(diskBindings.data()),
                static_cast<std::streamsize>(
                    diskBindings.size() * sizeof(AxisymmetricMappingDiskBinding)));
        }

        std::shared_ptr<const AxisymmetricBindingVector>
        cachedAxisymmetricProfileBindings(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const spraythickness::AxisymmetricProfilePredictionOptions& profile,
            bool& cacheHit)
        {
            cacheHit = false;
            static std::mutex cacheMutex;
            static AxisymmetricBindingCache cache;
            const std::uint64_t key = hashAxisymmetricProfileMapping(workpiece, profile);
            {
                const std::lock_guard<std::mutex> lock(cacheMutex);
                const auto iterator = cache.find(key);
                if(iterator != cache.end()) {
                    cacheHit = true;
                    return iterator->second;
                }
            }

            if(const auto diskCached = loadAxisymmetricMappingCache(
                key, workpiece.samples.size())) {
                const std::lock_guard<std::mutex> lock(cacheMutex);
                constexpr std::size_t kMaximumEntries = 4;
                if(cache.size() >= kMaximumEntries) {
                    cache.erase(cache.begin());
                }
                cache[key] = diskCached;
                cacheHit = true;
                return diskCached;
            }

            const auto built = std::make_shared<const AxisymmetricBindingVector>(
                buildAxisymmetricProfileBindings(workpiece, profile));
            saveAxisymmetricMappingCache(key, *built);
            {
                const std::lock_guard<std::mutex> lock(cacheMutex);
                constexpr std::size_t kMaximumEntries = 4;
                if(cache.size() >= kMaximumEntries) {
                    cache.erase(cache.begin());
                }
                cache[key] = built;
            }
            return built;
        }

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

        std::filesystem::path bvhCachePath(std::uint64_t geometryHash)
        {
            const std::filesystem::path root = findProjectRoot();
            if(root.empty()) {
                return {};
            }
            return root / ".cache" / "thickness_bvh" /
                (std::to_string(geometryHash) + ".bin");
        }

        void appendHashBytes(
            std::uint64_t& hash,
            const void* data,
            std::size_t size)
        {
            const auto* bytes = static_cast<const std::uint8_t*>(data);
            for(std::size_t index = 0; index < size; ++index) {
                hash ^= bytes[index];
                hash *= kFnvPrime;
            }
        }

        void appendHashString(std::uint64_t& hash, const std::string& value)
        {
            appendHashBytes(hash, value.data(), value.size());
            const std::uint8_t separator = 0;
            appendHashBytes(hash, &separator, sizeof(separator));
        }

        std::uint64_t powerOfTwoBucket(std::size_t value)
        {
            std::uint64_t bucket = 1;
            while(bucket < value && bucket < (1ull << 62)) {
                bucket <<= 1;
            }
            return bucket;
        }

        struct GpuCapabilities
        {
            std::string vendor;
            std::string renderer;
            std::string version;
            int maximumBlockSize{ 0 };
            int maximumStorageBufferBindings{ 0 };
            std::uint64_t fingerprint{ 0 };
        };

        GpuCapabilities queryGpuCapabilities()
        {
            GpuCapabilities capabilities;
            const auto readString = [](unsigned int name) {
                const auto* value = glGetString(name);
                return value == nullptr
                    ? std::string()
                    : std::string(reinterpret_cast<const char*>(value));
            };
            capabilities.vendor = readString(GL_VENDOR);
            capabilities.renderer = readString(GL_RENDERER);
            capabilities.version = readString(GL_VERSION);
            glGetIntegerv(
                GL_MAX_SHADER_STORAGE_BLOCK_SIZE,
                &capabilities.maximumBlockSize);
            glGetIntegerv(
                GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS,
                &capabilities.maximumStorageBufferBindings);
            capabilities.fingerprint = kFnvOffset;
            appendHashString(capabilities.fingerprint, capabilities.vendor);
            appendHashString(capabilities.fingerprint, capabilities.renderer);
            appendHashString(capabilities.fingerprint, capabilities.version);
            appendHashBytes(
                capabilities.fingerprint,
                &capabilities.maximumBlockSize,
                sizeof(capabilities.maximumBlockSize));
            appendHashBytes(
                capabilities.fingerprint,
                &capabilities.maximumStorageBufferBindings,
                sizeof(capabilities.maximumStorageBufferBindings));
            return capabilities;
        }

        std::filesystem::path batchProfileCachePath(std::uint64_t key)
        {
            const std::filesystem::path root = findProjectRoot();
            if(root.empty()) {
                return {};
            }
            return root / ".cache" / "thickness_gpu" /
                (std::to_string(key) + ".bin");
        }

        struct BatchProfileCacheHeader
        {
            std::uint64_t magic{ kBatchProfileCacheMagic };
            std::uint32_t version{ kBatchProfileCacheVersion };
            std::uint32_t reserved{ 0 };
            std::uint64_t key{ 0 };
            std::uint64_t batchSize{ 0 };
        };

        std::size_t loadBatchProfile(
            std::uint64_t key,
            std::size_t maximumBatch)
        {
            const std::filesystem::path path = batchProfileCachePath(key);
            if(path.empty() || !std::filesystem::exists(path)) {
                return 0;
            }
            std::ifstream input(path, std::ios::binary);
            BatchProfileCacheHeader header;
            if(!input.read(
                reinterpret_cast<char*>(&header),
                sizeof(header))
                || header.magic != kBatchProfileCacheMagic
                || header.version != kBatchProfileCacheVersion
                || header.key != key
                || header.batchSize == 0
                || header.batchSize > maximumBatch) {
                return 0;
            }
            return static_cast<std::size_t>(header.batchSize);
        }

        void saveBatchProfile(std::uint64_t key, std::size_t batchSize)
        {
            const std::filesystem::path path = batchProfileCachePath(key);
            if(path.empty() || batchSize == 0) {
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
            BatchProfileCacheHeader header;
            header.key = key;
            header.batchSize = batchSize;
            output.write(
                reinterpret_cast<const char*>(&header),
                sizeof(header));
        }

        std::uint64_t makeBatchProfileKey(
            const GpuCapabilities& capabilities,
            bool spatialFiltering,
            bool enableBvh,
            bool enableHistory,
            std::size_t vertexCount,
            double candidateRatio)
        {
            std::uint64_t key = capabilities.fingerprint;
            const std::uint8_t flags[] = {
                static_cast<std::uint8_t>(spatialFiltering),
                static_cast<std::uint8_t>(enableBvh),
                static_cast<std::uint8_t>(enableHistory)};
            appendHashBytes(key, flags, sizeof(flags));
            const std::uint64_t vertexBucket = powerOfTwoBucket(vertexCount);
            appendHashBytes(key, &vertexBucket, sizeof(vertexBucket));
            const auto ratioBucket = static_cast<std::uint32_t>(std::clamp(
                candidateRatio * 1000.0,
                0.0,
                1000.0));
            appendHashBytes(key, &ratioBucket, sizeof(ratioBucket));
            return key;
        }

        class AdaptiveBatchSelector
        {
        public:
            AdaptiveBatchSelector(
                std::uint64_t cacheKey,
                std::size_t maximumBatch,
                std::size_t initialBatch)
                : m_cacheKey(cacheKey)
                , m_maximumBatch(std::max<std::size_t>(1, maximumBatch))
            {
                const std::size_t cached = loadBatchProfile(
                    cacheKey,
                    m_maximumBatch);
                if(cached > 0) {
                    m_currentBatch = cached;
                    m_bestBatch = cached;
                    m_cached = true;
                    m_wasCached = true;
                } else {
                    m_currentBatch = std::clamp(
                        initialBatch,
                        std::size_t(1),
                        m_maximumBatch);
                    m_bestBatch = m_currentBatch;
                }
            }

            std::size_t next(std::size_t remaining) const
            {
                return std::min(
                    remaining,
                    std::max<std::size_t>(1, m_currentBatch));
            }

            bool measuring() const
            {
                return !m_cached && m_measurements < kMaximumMeasurements;
            }

            void record(
                std::size_t batchSize,
                std::uint64_t workItems,
                double elapsedMilliseconds)
            {
                if(m_cached || elapsedMilliseconds <= 0.0) {
                    return;
                }
                const double throughput = static_cast<double>(workItems)
                    / elapsedMilliseconds;
                if(throughput > m_bestThroughput) {
                    m_bestThroughput = throughput;
                    m_bestBatch = batchSize;
                }
                ++m_measurements;
                if(m_measurements >= kMaximumMeasurements
                    || m_currentBatch >= m_maximumBatch) {
                    m_currentBatch = m_bestBatch;
                    saveBatchProfile(m_cacheKey, m_bestBatch);
                    m_cached = true;
                    return;
                }
                if(elapsedMilliseconds > 500.0) {
                    m_currentBatch = std::max<std::size_t>(
                        1,
                        m_currentBatch / 2);
                } else {
                    m_currentBatch = std::min(
                        m_maximumBatch,
                        m_currentBatch * 2);
                }
            }

            bool wasCached() const { return m_wasCached; }
            bool isCalibrating() const { return !m_cached; }
            std::size_t selectedBatch() const { return m_bestBatch; }

            void finalize()
            {
                if(!m_cached && m_bestThroughput > 0.0) {
                    m_currentBatch = m_bestBatch;
                    saveBatchProfile(m_cacheKey, m_bestBatch);
                    m_cached = true;
                }
            }

        private:
            static constexpr std::size_t kMaximumMeasurements = 3;

            std::uint64_t m_cacheKey{ 0 };
            std::size_t m_maximumBatch{ 1 };
            std::size_t m_currentBatch{ 1 };
            std::size_t m_bestBatch{ 1 };
            double m_bestThroughput{ 0.0 };
            std::size_t m_measurements{ 0 };
            bool m_cached{ false };
            bool m_wasCached{ false };
        };

        std::shared_ptr<const ThicknessBvh> loadBvhCache(
            std::uint64_t geometryHash,
            std::size_t expectedTriangleCount,
            const ThicknessBvhProgress& progress)
        {
            const std::filesystem::path path = bvhCachePath(geometryHash);
            if(path.empty() || !std::filesystem::exists(path)) {
                return {};
            }
            std::ifstream input(path, std::ios::binary);
            BvhCacheHeader header;
            if(!input.read(reinterpret_cast<char*>(&header), sizeof(header)) ||
                header.magic != kBvhCacheMagic ||
                header.version != kBvhCacheVersion ||
                header.nodeSize != sizeof(GpuBvhNode) ||
                header.geometryHash != geometryHash ||
                header.triangleCount != expectedTriangleCount ||
                header.nodeCount == 0 || header.triangleCount == 0 ||
                header.nodeCount > 100000000 || header.triangleCount > 1000000000) {
                return {};
            }
            auto result = std::make_shared<ThicknessBvh>();
            result->nodes.resize(static_cast<std::size_t>(header.nodeCount));
            result->triangleOrder.resize(static_cast<std::size_t>(header.triangleCount));
            if(!input.read(
                reinterpret_cast<char*>(result->nodes.data()),
                static_cast<std::streamsize>(result->nodes.size() * sizeof(GpuBvhNode))) ||
                !input.read(
                reinterpret_cast<char*>(result->triangleOrder.data()),
                static_cast<std::streamsize>(result->triangleOrder.size() * sizeof(std::uint32_t)))) {
                return {};
            }
            for(const std::uint32_t triangle : result->triangleOrder) {
                if(triangle >= expectedTriangleCount) {
                    return {};
                }
            }
            for(const GpuBvhNode& node : result->nodes) {
                if(node.leftChild >= static_cast<std::int32_t>(result->nodes.size()) ||
                    node.rightChild >= static_cast<std::int32_t>(result->nodes.size()) ||
                    node.firstTriangle < 0 || node.triangleCount < 0 ||
                    static_cast<std::uint64_t>(node.firstTriangle) +
                        static_cast<std::uint64_t>(node.triangleCount) >
                        result->triangleOrder.size()) {
                    return {};
                }
            }
            if(progress) {
                progress("disk cache hit", 1, 1);
            }
            return result;
        }

        void saveBvhCache(std::uint64_t geometryHash, const ThicknessBvh& bvh)
        {
            const std::filesystem::path path = bvhCachePath(geometryHash);
            if(path.empty() || bvh.empty()) {
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
            BvhCacheHeader header;
            header.geometryHash = geometryHash;
            header.nodeCount = bvh.nodes.size();
            header.triangleCount = bvh.triangleOrder.size();
            output.write(reinterpret_cast<const char*>(&header), sizeof(header));
            output.write(
                reinterpret_cast<const char*>(bvh.nodes.data()),
                static_cast<std::streamsize>(bvh.nodes.size() * sizeof(GpuBvhNode)));
            output.write(
                reinterpret_cast<const char*>(bvh.triangleOrder.data()),
                static_cast<std::streamsize>(bvh.triangleOrder.size() * sizeof(std::uint32_t)));
        }

        std::shared_ptr<const ThicknessBvh> cachedBvh(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const ThicknessBvhProgress& progress)
        {
            static std::mutex cacheMutex;
            static std::unordered_map<
                std::uint64_t,
                std::shared_ptr<const ThicknessBvh>> cache;

            const std::uint64_t geometryHash = hashWorkpieceGeometry(workpiece);
            {
                std::lock_guard<std::mutex> lock(cacheMutex);
                const auto cached = cache.find(geometryHash);
                if(cached != cache.end()) {
                    return cached->second;
                }
            }

            if(const std::shared_ptr<const ThicknessBvh> disk =
                loadBvhCache(
                    geometryHash,
                    workpiece.triangleIndices.size() / 3,
                    progress)) {
                std::lock_guard<std::mutex> lock(cacheMutex);
                cache[geometryHash] = disk;
                return disk;
            }

            auto rebuilt = std::make_shared<ThicknessBvh>(
                ThicknessBvhBuilder::build(workpiece, 4, progress));
            saveBvhCache(geometryHash, *rebuilt);
            {
                std::lock_guard<std::mutex> lock(cacheMutex);
                cache[geometryHash] = rebuilt;
            }
            return rebuilt;
        }

        constexpr std::uint64_t kSpatialGridCacheMagic = 0x5253323032365347ull;
        constexpr std::uint32_t kSpatialGridCacheVersion = 2;

        struct SpatialGridCacheHeader
        {
            std::uint64_t magic{ kSpatialGridCacheMagic };
            std::uint32_t version{ kSpatialGridCacheVersion };
            std::uint32_t manualCellSize{ 0 };
            std::uint64_t key{ 0 };
            std::uint64_t vertexCount{ 0 };
            std::uint64_t cellOffsetCount{ 0 };
            std::uint64_t candidateCount{ 0 };
            std::uint64_t candidateCellPairs{ 0 };
            std::uint64_t candidateVertexPairs{ 0 };
            std::array<double, 3> minimum{};
            std::array<double, 3> maximum{};
            std::array<std::int32_t, 3> dimensions{};
            double cellSize{ 0.0 };
            double coneSlope{ 0.0 };
            double contributionCutoffRatio{ 0.0 };
        };

        std::uint64_t hashSpatialGridInputs(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const std::vector<PeriodicSpraySample>& samples,
            const spraythickness::PaperGaussianParameters& deposition,
            double contributionCutoffRatio,
            bool overrideGridCellSize,
            double gridCellSizeMeters)
        {
            std::uint64_t hash = hashWorkpieceGeometry(workpiece);
            const std::uint64_t sampleCount = samples.size();
            appendHashBytes(hash, &sampleCount, sizeof(sampleCount));
            for(const PeriodicSpraySample& sample : samples) {
                appendHashBytes(hash, sample.position.data(), 3 * sizeof(double));
                appendHashBytes(hash, sample.direction.data(), 3 * sizeof(double));
                appendHashBytes(hash, sample.majorAxis.data(), 3 * sizeof(double));
                appendHashBytes(hash, sample.minorAxis.data(), 3 * sizeof(double));
                appendHashBytes(hash, &sample.time, sizeof(sample.time));
                appendHashBytes(hash, &sample.duration, sizeof(sample.duration));
            }
            appendHashBytes(
                hash,
                &deposition.rotationRadians,
                sizeof(deposition.rotationRadians));
            appendHashBytes(
                hash,
                &deposition.phiOffsetRadians,
                sizeof(deposition.phiOffsetRadians));
            appendHashBytes(
                hash,
                &deposition.psiOffsetRadians,
                sizeof(deposition.psiOffsetRadians));
            appendHashBytes(
                hash,
                &deposition.sigmaPhiRadians,
                sizeof(deposition.sigmaPhiRadians));
            appendHashBytes(
                hash,
                &deposition.sigmaPsiRadians,
                sizeof(deposition.sigmaPsiRadians));
            appendHashBytes(
                hash,
                &contributionCutoffRatio,
                sizeof(contributionCutoffRatio));
            appendHashBytes(
                hash,
                &overrideGridCellSize,
                sizeof(overrideGridCellSize));
            appendHashBytes(
                hash,
                &gridCellSizeMeters,
                sizeof(gridCellSizeMeters));
            return hash;
        }

        std::filesystem::path spatialGridCachePath(std::uint64_t key)
        {
            const std::filesystem::path root = findProjectRoot();
            if(root.empty()) {
                return {};
            }
            return root / ".cache" / "thickness_spatial" /
                (std::to_string(key) + ".bin");
        }

        std::shared_ptr<const SpatialSprayGrid> loadSpatialGridCache(
            std::uint64_t key,
            std::size_t expectedVertexCount)
        {
            const std::filesystem::path path = spatialGridCachePath(key);
            if(path.empty() || !std::filesystem::exists(path)) {
                return {};
            }
            std::ifstream input(path, std::ios::binary);
            SpatialGridCacheHeader header;
            if(!input.read(
                reinterpret_cast<char*>(&header),
                sizeof(header))
                || header.magic != kSpatialGridCacheMagic
                || header.version != kSpatialGridCacheVersion
                || header.key != key
                || header.vertexCount != expectedVertexCount
                || header.cellOffsetCount == 0
                || header.candidateCount > 64ull * 1024ull * 1024ull
                || header.cellOffsetCount > 96ull * 96ull * 96ull + 1
                || header.dimensions[0] < 1 || header.dimensions[0] > 96
                || header.dimensions[1] < 1 || header.dimensions[1] > 96
                || header.dimensions[2] < 1 || header.dimensions[2] > 96) {
                return {};
            }

            auto grid = std::make_shared<SpatialSprayGrid>();
            for(int axis = 0; axis < 3; ++axis) {
                grid->minimum[axis] = header.minimum[axis];
                grid->maximum[axis] = header.maximum[axis];
                grid->dimensions[axis] = header.dimensions[axis];
            }
            grid->cellSize = header.cellSize;
            grid->manualCellSize = header.manualCellSize != 0;
            grid->coneSlope = header.coneSlope;
            grid->contributionCutoffRatio = header.contributionCutoffRatio;
            grid->candidateCellPairs = static_cast<std::size_t>(
                header.candidateCellPairs);
            grid->candidateVertexPairs = static_cast<std::size_t>(
                header.candidateVertexPairs);
            grid->cellOffsets.resize(static_cast<std::size_t>(header.cellOffsetCount));
            grid->candidateSprayIndices.resize(static_cast<std::size_t>(header.candidateCount));
            grid->vertexCellIndices.resize(expectedVertexCount);
            if(!input.read(
                reinterpret_cast<char*>(grid->cellOffsets.data()),
                static_cast<std::streamsize>(grid->cellOffsets.size()
                    * sizeof(std::uint32_t)))
                || !input.read(
                    reinterpret_cast<char*>(grid->candidateSprayIndices.data()),
                static_cast<std::streamsize>(grid->candidateSprayIndices.size()
                    * sizeof(std::uint32_t)))
                || !input.read(
                    reinterpret_cast<char*>(grid->vertexCellIndices.data()),
                static_cast<std::streamsize>(grid->vertexCellIndices.size()
                    * sizeof(std::uint32_t)))
                || !grid->valid()
                || grid->cellOffsets.back() != grid->candidateSprayIndices.size()) {
                return {};
            }
            return grid;
        }

        void saveSpatialGridCache(
            std::uint64_t key,
            const SpatialSprayGrid& grid)
        {
            if(!grid.valid()) {
                return;
            }
            const std::filesystem::path path = spatialGridCachePath(key);
            if(path.empty()) {
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
            SpatialGridCacheHeader header;
            header.key = key;
            header.vertexCount = grid.vertexCellIndices.size();
            header.cellOffsetCount = grid.cellOffsets.size();
            header.candidateCount = grid.candidateSprayIndices.size();
            header.candidateCellPairs = grid.candidateCellPairs;
            header.candidateVertexPairs = grid.candidateVertexPairs;
            header.manualCellSize = grid.manualCellSize ? 1U : 0U;
            for(int axis = 0; axis < 3; ++axis) {
                header.minimum[axis] = grid.minimum[axis];
                header.maximum[axis] = grid.maximum[axis];
                header.dimensions[axis] = grid.dimensions[axis];
            }
            header.cellSize = grid.cellSize;
            header.coneSlope = grid.coneSlope;
            header.contributionCutoffRatio = grid.contributionCutoffRatio;
            output.write(reinterpret_cast<const char*>(&header), sizeof(header));
            output.write(
                reinterpret_cast<const char*>(grid.cellOffsets.data()),
                static_cast<std::streamsize>(grid.cellOffsets.size()
                    * sizeof(std::uint32_t)));
            output.write(
                reinterpret_cast<const char*>(grid.candidateSprayIndices.data()),
                static_cast<std::streamsize>(grid.candidateSprayIndices.size()
                    * sizeof(std::uint32_t)));
            output.write(
                reinterpret_cast<const char*>(grid.vertexCellIndices.data()),
                static_cast<std::streamsize>(grid.vertexCellIndices.size()
                    * sizeof(std::uint32_t)));
        }

        std::shared_ptr<const SpatialSprayGrid> cachedSpatialGrid(
            const sprayworkpiece::WorkpieceModel& workpiece,
            const std::vector<PeriodicSpraySample>& samples,
            const spraythickness::PaperGaussianParameters& deposition,
            double contributionCutoffRatio,
            bool overrideGridCellSize,
            double gridCellSizeMeters,
            const SpatialGridProgress& progress,
            bool& cacheHit)
        {
            static std::mutex cacheMutex;
            static std::unordered_map<
                std::uint64_t,
                std::shared_ptr<const SpatialSprayGrid>> cache;

            const std::uint64_t key = hashSpatialGridInputs(
                workpiece,
                samples,
                deposition,
                contributionCutoffRatio,
                overrideGridCellSize,
                gridCellSizeMeters);
            {
                std::lock_guard<std::mutex> lock(cacheMutex);
                const auto cached = cache.find(key);
                if(cached != cache.end()) {
                    cacheHit = true;
                    if(progress) {
                        progress("memory cache hit", 1, 1);
                    }
                    return cached->second;
                }
            }
            if(const std::shared_ptr<const SpatialSprayGrid> disk =
                loadSpatialGridCache(key, workpiece.samples.size())) {
                std::lock_guard<std::mutex> lock(cacheMutex);
                cache[key] = disk;
                cacheHit = true;
                if(progress) {
                    progress("disk cache hit", 1, 1);
                }
                return disk;
            }

            auto built = std::make_shared<SpatialSprayGrid>(buildSpatialSprayGrid(
                workpiece,
                samples,
                deposition,
                contributionCutoffRatio,
                overrideGridCellSize ? gridCellSizeMeters : 0.0,
                progress));
            if(built->valid()) {
                saveSpatialGridCache(key, *built);
                std::lock_guard<std::mutex> lock(cacheMutex);
                cache[key] = built;
            }
            cacheHit = false;
            return built;
        }

        std::size_t computeAdaptiveBvhBatchSize(
            std::size_t vertexCount,
            std::size_t sprayPointCount,
            std::size_t pointSizeInBytes,
            int maximumBlockSize)
        {
            if(sprayPointCount == 0) {
                return 1;
            }

            const std::size_t maxBatchByMemory = maximumBlockSize > 0
                ? static_cast<std::size_t>(maximumBlockSize)
                    / std::max<std::size_t>(pointSizeInBytes, 1)
                : sprayPointCount;
            // This is only the first-run calibration starting point. The
            // runtime selector can grow beyond it when the current GPU proves
            // that larger batches deliver better throughput.
            const std::size_t maxBatchByWork = vertexCount > 0
                ? static_cast<std::size_t>(std::max<std::uint64_t>(
                    1, kInitialTargetVertexSprayPairs / vertexCount))
                : sprayPointCount;

            return std::max<std::size_t>(1, std::min({
                maxBatchByMemory,
                maxBatchByWork,
                sprayPointCount}));
        }

        struct alignas(16) LegacyGpuBvhNode
        {
            std::array<float, 4> minimum{};
            std::array<float, 4> maximum{};
            std::int32_t leftChild{ -1 };
            std::int32_t rightChild{ -1 };
            std::int32_t padding[2]{ 0, 0 };
        };

        struct LegacyGpuBvh
        {
            std::vector<LegacyGpuBvhNode> nodes;
            std::vector<std::uint32_t> leafTriangleIndices;
        };

        LegacyGpuBvh makeLegacyGpuBvh(const ThicknessBvh& bvh)
        {
            LegacyGpuBvh result;
            result.nodes.resize(bvh.nodes.size());
            result.leafTriangleIndices.resize(bvh.triangleOrder.size(), 0U);
            for(std::size_t index = 0; index < bvh.nodes.size(); ++index) {
                const GpuBvhNode& source = bvh.nodes[index];
                LegacyGpuBvhNode& target = result.nodes[index];
                target.minimum = source.minimum;
                target.maximum = source.maximum;
                if(source.triangleCount > 0) {
                    target.leftChild = -source.firstTriangle - 1;
                    target.rightChild = source.triangleCount;
                    for(std::int32_t triangle = 0;
                        triangle < source.triangleCount;
                        ++triangle) {
                        const std::size_t orderIndex = static_cast<std::size_t>(
                            source.firstTriangle + triangle);
                        if(orderIndex >= bvh.triangleOrder.size()) {
                            continue;
                        }
                        result.leafTriangleIndices[orderIndex] =
                            bvh.triangleOrder[orderIndex] * 3U;
                    }
                } else {
                    target.leftChild = source.leftChild;
                    target.rightChild = source.rightChild;
                }
            }
            return result;
        }

        class GpuBuffer
        {
        public:
            GpuBuffer() { glGenBuffers(1, &m_id); }
            ~GpuBuffer() { if(m_id != 0) glDeleteBuffers(1, &m_id); }
            GpuBuffer(const GpuBuffer&) = delete;
            GpuBuffer& operator=(const GpuBuffer&) = delete;
            std::size_t size() const { return m_size; }

            void bind(unsigned int binding) const
            {
                glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, m_id);
            }

            void upload(unsigned int binding, const void* data, std::size_t size, unsigned int usage)
            {
                m_size = size;
                glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_id);
                if(size > m_capacity) {
                    glBufferData(
                        GL_SHADER_STORAGE_BUFFER,
                        static_cast<GLsizeiptr>(size),
                        data,
                        usage);
                    m_capacity = size;
                } else if(size > 0) {
                    glBufferSubData(
                        GL_SHADER_STORAGE_BUFFER,
                        0,
                        static_cast<GLsizeiptr>(size),
                        data);
                }
                glBindBufferBase(GL_SHADER_STORAGE_BUFFER, binding, m_id);
            }

            void update(const void* data, std::size_t size)
            {
                glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_id);
                glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, static_cast<GLsizeiptr>(size), data);
            }

            void download(void* data, std::size_t size) const
            {
                glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_id);
                glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, static_cast<GLsizeiptr>(size), data);
            }

        private:
            unsigned int m_id{ 0 };
            std::size_t m_capacity{ 0 };
            std::size_t m_size{ 0 };
        };

        class GpuElapsedQuery
        {
        public:
            GpuElapsedQuery()
            {
                glGenQueries(1, &m_id);
            }

            ~GpuElapsedQuery()
            {
                if(m_active) {
                    glEndQuery(GL_TIME_ELAPSED);
                }
                if(m_id != 0) {
                    glDeleteQueries(1, &m_id);
                }
            }

            GpuElapsedQuery(const GpuElapsedQuery&) = delete;
            GpuElapsedQuery& operator=(const GpuElapsedQuery&) = delete;

            void begin()
            {
                if(m_id == 0) {
                    return;
                }
                glBeginQuery(GL_TIME_ELAPSED, m_id);
                m_active = true;
                m_finished = false;
            }

            void end()
            {
                if(!m_active) return;
                glEndQuery(GL_TIME_ELAPSED);
                m_active = false;
                m_finished = true;
            }

            double readMilliseconds() const
            {
                if(!m_finished) return 0.0;
                GLuint64 nanoseconds = 0;
                glGetQueryObjectui64v(m_id, GL_QUERY_RESULT, &nanoseconds);
                return static_cast<double>(nanoseconds) / 1.0e6;
            }

            double endAndReadMilliseconds()
            {
                if(!m_active) return 0.0;
                end();
                return readMilliseconds();
            }

        private:
            GLuint m_id{ 0 };
            bool m_active = false;
            bool m_finished = false;
        };

        struct SprayGpuSample
        {
            std::array<float, 4> positionAndDt{};
            std::array<float, 4> direction{};
            std::array<float, 4> majorAxis{};
            std::array<float, 4> minorAxis{};
            std::array<float, 4> time{};
        };

        class BackendGpuResources
        {
        public:
            BackendGpuResources()
            {
                for(auto& buffer : m_buffers) {
                    buffer = std::make_unique<GpuBuffer>();
                }
            }

            GpuBuffer& buffer(unsigned int binding)
            {
                return *m_buffers.at(binding);
            }

            OpenGLComputeProgram& program(
                bool spatialFiltering,
                bool enableBvh,
                bool enableHistory)
            {
                const std::uint32_t key =
                    (spatialFiltering ? 1u : 0u)
                    | (enableBvh ? 2u : 0u)
                    | (enableHistory ? 4u : 0u);
                const auto existing = m_programs.find(key);
                if(existing != m_programs.end()) {
                    return *existing->second;
                }
                const std::string source = spatialFiltering
                    ? makeSpatialFilteredGaussianHistoryShader(enableBvh, enableHistory)
                    : makePaperGaussianHistoryShader(enableBvh, enableHistory);
                const char* name = spatialFiltering
                    ? "Spatial filtered Paper Gaussian history"
                    : "Paper Gaussian history";
                auto program = std::make_unique<OpenGLComputeProgram>(
                    source.c_str(),
                    name);
                OpenGLComputeProgram& result = *program;
                m_programs.emplace(key, std::move(program));
                return result;
            }

        private:
            std::array<std::unique_ptr<GpuBuffer>, 14> m_buffers;
            std::unordered_map<
                std::uint32_t,
                std::unique_ptr<OpenGLComputeProgram>> m_programs;
        };

        std::array<float, 4> vector4(const Eigen::Vector3d& value, float w = 0.0f)
        {
            return {
                static_cast<float>(value.x()),
                static_cast<float>(value.y()),
                static_cast<float>(value.z()),
                w
            };
        }

        Eigen::Vector3f normalizedGpuVector(
            const Eigen::Vector3d& value,
            const Eigen::Vector3f& fallback)
        {
            const Eigen::Vector3f converted = value.cast<float>();
            const float lengthSquared = converted.squaredNorm();
            return lengthSquared > 1.0e-12f
                ? converted * (1.0f / std::sqrt(lengthSquared))
                : fallback;
        }

        float distanceEfficiencyCpu(float distanceMillimeters)
        {
            const float x = std::max(distanceMillimeters, 1.0e-6f);
            const float normalizedLogDistance =
                std::log(x / 106.8615f) / 0.0839f;
            return 0.9789f + 0.1456f * std::exp(
                -(normalizedLogDistance * normalizedLogDistance));
        }

        float angleEfficiencyCpu(float angleDegrees)
        {
            return std::max(
                1.0e-6f,
                (40.81414f + 0.73808f * angleDegrees
                    - 0.000838598f * angleDegrees * angleDegrees) * 0.01f);
        }

        void reportProgress(
            const ThicknessPredictionExecution& execution,
            double progress,
            const char* message)
        {
            if(execution.progress) {
                execution.progress(progress, message);
            }
        }

        void checkOpenGlErrors(const char* stage)
        {
            const GLenum error = glGetError();
            if(error == GL_NO_ERROR) {
                return;
            }
            throw std::runtime_error(
                std::string("OpenGL error at ") + stage + ": 0x"
                + std::to_string(static_cast<unsigned int>(error)));
        }

        bool canceled(const ThicknessPredictionExecution& execution)
        {
            return execution.cancelRequested != nullptr && execution.cancelRequested->load();
        }

        Eigen::Vector3d vector3(const std::array<float, 4>& value)
        {
            return Eigen::Vector3d(value[0], value[1], value[2]);
        }

        SprayGpuSample makeGpuSpraySample(const PeriodicSpraySample& sample)
        {
            SprayGpuSample gpuSample;
            gpuSample.positionAndDt = vector4(
                sample.position * kMetersToMillimeters,
                static_cast<float>(sample.duration));
            gpuSample.direction = vector4(
                normalizedGpuVector(sample.direction, Eigen::Vector3f::UnitX()).cast<double>());
            gpuSample.majorAxis = vector4(
                normalizedGpuVector(sample.majorAxis, Eigen::Vector3f::UnitY()).cast<double>());
            gpuSample.minorAxis = vector4(
                normalizedGpuVector(sample.minorAxis, Eigen::Vector3f::UnitZ()).cast<double>());
            gpuSample.time[0] = static_cast<float>(sample.time);
            return gpuSample;
        }

        std::vector<std::array<float, 4>> makeSurfaceData(
            const sprayworkpiece::WorkpieceModel& workpiece)
        {
            std::vector<std::array<float, 4>> result;
            result.reserve(workpiece.samples.size() * 2);
            for(const auto& sample : workpiece.samples) {
                result.push_back(vector4(sample.position * kMetersToMillimeters));
                const Eigen::Vector3d normal = sample.normal.norm() > 1.0e-12
                    ? sample.normal.normalized()
                    : Eigen::Vector3d::UnitZ();
                result.push_back(vector4(normal));
            }
            return result;
        }

        std::vector<std::array<float, 4>> makeSurfacePositionData(
            const sprayworkpiece::WorkpieceModel& workpiece)
        {
            std::vector<std::array<float, 4>> result;
            result.reserve(workpiece.samples.size());
            for(const auto& sample : workpiece.samples) {
                result.push_back(vector4(sample.position * kMetersToMillimeters));
            }
            return result;
        }

        void configureProgram(
            const OpenGLComputeProgram& program,
            const ThicknessPredictionTask& task,
            int vertexCount,
            int predictionVertexCount,
            bool usePredictionVertexBuffer)
        {
            const auto& options = task.options;
            const auto& deposition = options.deposition;
            const auto& history = options.history;
            const float referenceDistanceMillimeters = static_cast<float>(
                deposition.referenceDistanceMeters * kMetersToMillimeters);
            const float referenceAngle = static_cast<float>(
                deposition.referenceAngleDegrees);
            const float referenceExposure = static_cast<float>(
                std::max(deposition.referenceExposureSeconds, 1.0e-6));
            const float processScale = static_cast<float>(
                task.process.flowRate * task.process.atomizationFactor
                * task.process.materialScale);
            program.setInt("vertexCount", vertexCount);
            program.setInt("predictionVertexCount", predictionVertexCount);
            program.setInt("usePredictionVertexBuffer", usePredictionVertexBuffer ? 1 : 0);
            program.setInt("enableBvh", options.enableBvhOcclusion ? 1 : 0);
            program.setInt("enableHistory", options.enableHistoryCorrection ? 1 : 0);
            program.setFloat("shadowBiasMm", static_cast<float>(options.shadowBiasMeters * kMetersToMillimeters));
            program.setFloat("amplitudeMm", static_cast<float>(deposition.amplitudeMillimeters));
            program.setFloat("patternRotation", static_cast<float>(deposition.rotationRadians));
            program.setFloat("phiOffset", static_cast<float>(deposition.phiOffsetRadians));
            program.setFloat("psiOffset", static_cast<float>(deposition.psiOffsetRadians));
            program.setFloat("sigmaPhi", static_cast<float>(deposition.sigmaPhiRadians));
            program.setFloat("sigmaPsi", static_cast<float>(deposition.sigmaPsiRadians));
            program.setFloat("referenceDistanceMm", referenceDistanceMillimeters);
            program.setFloat("referenceAngleDegrees", referenceAngle);
            program.setFloat("referenceExposureSeconds", referenceExposure);
            program.setFloat("processScale", processScale);
            program.setFloat("patternCosine", std::cos(static_cast<float>(deposition.rotationRadians)));
            program.setFloat("patternSine", std::sin(static_cast<float>(deposition.rotationRadians)));
            program.setFloat("inverseSigmaPhi", 1.0f / std::max(
                static_cast<float>(deposition.sigmaPhiRadians), 1.0e-6f));
            program.setFloat("inverseSigmaPsi", 1.0f / std::max(
                static_cast<float>(deposition.sigmaPsiRadians), 1.0e-6f));
            program.setFloat("referenceProjection", std::max(
                1.0e-6f,
                std::cos((90.0f - referenceAngle)
                    * 3.14159265358979323846f / 180.0f)));
            program.setFloat(
                "referenceDistanceEfficiency",
                distanceEfficiencyCpu(referenceDistanceMillimeters));
            program.setFloat(
                "referenceAngleEfficiency",
                angleEfficiencyCpu(referenceAngle));
            program.setFloat(
                "referenceDistanceSquared",
                referenceDistanceMillimeters * referenceDistanceMillimeters);
            program.setFloat(
                "peakScale",
                static_cast<float>(deposition.amplitudeMillimeters)
                    * processScale / referenceExposure);
            program.setFloat("historyAmplitude", static_cast<float>(history.correctionAmplitude));
            program.setFloat("historyScaleSeconds", static_cast<float>(history.historyScaleSeconds));
            program.setFloat("referenceHistorySeconds", static_cast<float>(history.referenceHistorySeconds));
            program.setFloat("coolingTimeSeconds", static_cast<float>(history.coolingTimeSeconds));
            program.setFloat("activityThresholdRatio", static_cast<float>(history.activityThresholdRatio));
        }

        void configureSpatialProgram(
            const OpenGLComputeProgram& program,
            const ThicknessPredictionTask& task,
            int vertexCount,
            int predictionVertexCount,
            bool usePredictionVertexBuffer)
        {
            configureProgram(
                program,
                task,
                vertexCount,
                predictionVertexCount,
                usePredictionVertexBuffer);
            program.setInt("useSpatialCandidateBuffer", 1);
            program.setFloat(
                "contributionCutoffRatio",
                static_cast<float>(std::clamp(
                    task.options.spatialFiltering.contributionCutoffRatio,
                    1.0e-12,
                    0.999999)));
        }

        ThicknessPredictionResult predictSpatialFiltered(
            const ThicknessPredictionTask& task,
            const sprayworkpiece::WorkpieceModel& computationWorkpiece,
            const ThicknessPredictionExecution& execution,
            const std::vector<std::array<float, 4>>& surfaceData,
            const std::vector<std::array<float, 4>>& surfacePositionData,
            const std::vector<PeriodicSpraySample>& previewSpraySamples,
            const std::vector<SprayGpuSample>& allSpraySamples,
            BackendGpuResources& resources,
            const GpuCapabilities& capabilities)
        {
            PeriodicSectorReduction periodicReduction;
            bool periodicReductionApplied = false;
            double mappingMilliseconds = 0.0;
            double axisymmetricMappingMilliseconds = 0.0;
            bool axisymmetricMappingCacheHit = false;
            std::size_t axisymmetricBindingCount = 0;
            std::size_t axisymmetricActiveBindingCount = 0;
            std::size_t axisymmetricMappedZeroCount = 0;
            std::size_t axisymmetricMappedNonzeroCount = 0;
            std::size_t axisymmetricMappedInvalidCount = 0;
            std::size_t axisymmetricActiveSegmentCount = 0;
            std::size_t axisymmetricActiveProfilePathCount = 0;
            std::vector<std::string> predictionWarnings;
            const bool axisymmetricProfileApplied =
                task.options.axisymmetricProfile.enabled;
            if(axisymmetricProfileApplied && task.options.periodicLocal.enabled) {
                throw std::runtime_error(
                    "Axisymmetric profile and periodic sector prediction cannot be enabled together.");
            }
            std::vector<std::uint32_t> predictionVertexIndices;
            if(axisymmetricProfileApplied) {
                const std::size_t profileOffset = task.workpiece.samples.size();
                predictionVertexIndices.resize(
                    task.options.axisymmetricProfile.predictionSamples.size());
                for(std::size_t index = 0; index < predictionVertexIndices.size(); ++index) {
                    predictionVertexIndices[index] = static_cast<std::uint32_t>(
                        profileOffset + index);
                }
                predictionWarnings.push_back(
                    "Axisymmetric profile prediction maps sampled profile thickness back to the complete mesh.");
            } else if(!task.options.spatialFiltering.predictionVertexIndices.empty()) {
                predictionVertexIndices = task.options.spatialFiltering.predictionVertexIndices;
                std::vector<std::uint8_t> seen(task.workpiece.samples.size(), 0U);
                for(const std::uint32_t vertex : predictionVertexIndices) {
                    if(vertex >= task.workpiece.samples.size() || seen[vertex] != 0U) {
                        throw std::runtime_error(
                            "Restricted spatial prediction vertex list is invalid.");
                    }
                    seen[vertex] = 1U;
                }
                predictionWarnings.push_back(
                    "Restricted local vertex prediction was used; vertices outside the selected region remain zero.");
            } else {
                predictionVertexIndices.resize(task.workpiece.samples.size());
                std::iota(predictionVertexIndices.begin(), predictionVertexIndices.end(), 0U);
            }
            if(task.options.periodicLocal.enabled) {
                reportProgress(execution, 0.02, "Building periodic base-sector mapping");
                const auto mappingStart = std::chrono::steady_clock::now();
                PeriodicGpuMappingCallbacks mappingCallbacks;
                mappingCallbacks.canceled = [&execution]() {
                    return canceled(execution);
                };
                mappingCallbacks.progress = [&execution](
                    std::size_t current,
                    std::size_t total,
                    const char* phase) {
                    const double fraction = total > 0
                        ? static_cast<double>(current) / static_cast<double>(total)
                        : 1.0;
                    const std::string message = std::string("Periodic ") + phase + " ("
                        + std::to_string(current) + "/" + std::to_string(total) + ")";
                    reportProgress(execution, 0.02 + 0.02 * fraction, message.c_str());
                };
                std::string mappingFailure;
                try {
                    periodicReduction = buildPeriodicSectorReductionGpu(
                        task.workpiece,
                        task.options.periodicLocal,
                        mappingCallbacks);
                } catch(const std::exception& exception) {
                    mappingFailure = exception.what();
                }
                if(canceled(execution)) {
                    ThicknessPredictionResult result;
                    result.warnings.push_back("Thickness prediction was canceled.");
                    return result;
                }
                if(!mappingFailure.empty() || !periodicReduction.valid()) {
                    if(mappingFailure.empty()) {
                        mappingFailure = periodicReduction.failureReason;
                    }
                    if(!task.options.periodicLocal.fallbackToFullPrediction) {
                        throw std::runtime_error(
                            std::string("Periodic local mapping failed: ") + mappingFailure);
                    }
                    reportProgress(
                        execution,
                        0.04,
                        (std::string("Periodic local mapping unavailable; using full model: ")
                            + mappingFailure).c_str());
                } else {
                    predictionVertexIndices = periodicReduction.predictionVertexIndices;
                    periodicReductionApplied = true;
                    predictionWarnings.push_back(
                        "Periodic local prediction assumes rotationally periodic geometry, trajectory, occlusion, and thermal history.");
                }
                mappingMilliseconds = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - mappingStart).count();
            }
            const std::size_t spatialInputVertexCount = predictionVertexIndices.size();
            if(spatialInputVertexCount == 0) {
                throw std::runtime_error("Spatial filtering produced no prediction vertices.");
            }
            const auto gridStart = std::chrono::steady_clock::now();
            reportProgress(execution, 0.04, "Building ordered spatial spray grid");
            const SpatialGridProgress gridProgress = [&execution](
                const char* phase,
                std::size_t current,
                std::size_t total) {
                const double fraction = total > 0
                    ? static_cast<double>(current) / static_cast<double>(total)
                    : 1.0;
                const std::string message = std::string("Spatial grid ") + phase
                    + " (" + std::to_string(current) + "/"
                    + std::to_string(total) + ")";
                reportProgress(execution, 0.04 + 0.06 * fraction, message.c_str());
            };
            bool spatialGridCacheHit = false;
            sprayworkpiece::WorkpieceModel gridWorkpiece;
            gridWorkpiece.samples.reserve(predictionVertexIndices.size());
            for(const std::uint32_t vertex : predictionVertexIndices) {
                if(vertex >= computationWorkpiece.samples.size()) {
                    throw std::runtime_error(
                        "A prediction vertex is outside the computation surface buffer.");
                }
                gridWorkpiece.samples.push_back(computationWorkpiece.samples[vertex]);
            }
            const std::shared_ptr<const SpatialSprayGrid> gridHandle =
                cachedSpatialGrid(
                gridWorkpiece,
                previewSpraySamples,
                task.options.deposition,
                task.options.spatialFiltering.contributionCutoffRatio,
                task.options.spatialFiltering.overrideGridCellSize,
                task.options.spatialFiltering.gridCellSizeMeters,
                gridProgress,
                spatialGridCacheHit);
            if(!gridHandle || !gridHandle->valid()) {
                throw std::runtime_error(
                    !gridHandle || gridHandle->failureReason.empty()
                    ? "Spatial spray grid construction failed."
                    : gridHandle->failureReason);
            }
            const SpatialSprayGrid& grid = *gridHandle;
            const double gridMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - gridStart).count();
            std::size_t selectedCandidatePairs = 0;
            for(std::size_t predictionVertex = 0;
                predictionVertex < predictionVertexIndices.size();
                ++predictionVertex) {
                if(predictionVertex < grid.vertexCellIndices.size()) {
                    const std::size_t cell = grid.vertexCellIndices[predictionVertex];
                    if(cell + 1 < grid.cellOffsets.size()) {
                        selectedCandidatePairs += grid.cellOffsets[cell + 1]
                            - grid.cellOffsets[cell];
                    }
                }
            }
            const double totalPairs = static_cast<double>(spatialInputVertexCount)
                * static_cast<double>(allSpraySamples.size());
            const double candidateRatio = totalPairs > 0.0
                ? static_cast<double>(selectedCandidatePairs) / totalPairs
                : 0.0;
            const bool candidateVertexFilteringRequested =
                task.options.spatialFiltering.filterCandidateVertices
                && !periodicReductionApplied
                && !axisymmetricProfileApplied;
            std::vector<std::uint32_t> candidateVertexIndices;
            std::vector<std::uint32_t> candidateVertexCellIndices;
            if(candidateVertexFilteringRequested) {
                candidateVertexIndices.reserve(predictionVertexIndices.size());
                candidateVertexCellIndices.reserve(predictionVertexIndices.size());
                for(std::size_t predictionVertex = 0;
                    predictionVertex < predictionVertexIndices.size();
                    ++predictionVertex) {
                    const std::uint32_t sourceVertex = predictionVertexIndices[predictionVertex];
                    if(sourceVertex >= computationWorkpiece.samples.size()
                        || !computationWorkpiece.samples[sourceVertex].valid) {
                        continue;
                    }
                    if(predictionVertex >= grid.vertexCellIndices.size()) {
                        continue;
                    }
                    const std::size_t cell = grid.vertexCellIndices[predictionVertex];
                    if(cell + 1 >= grid.cellOffsets.size()
                        || grid.cellOffsets[cell] == grid.cellOffsets[cell + 1]) {
                        continue;
                    }
                    candidateVertexIndices.push_back(predictionVertexIndices[predictionVertex]);
                    candidateVertexCellIndices.push_back(
                        static_cast<std::uint32_t>(cell));
                }
            }
            // Keep the original complete-model result buffers. Only this
            // dispatch list changes, so skipped vertices remain exact zeroes.
            const bool candidateVertexFilteringApplied = candidateVertexFilteringRequested;
            const std::vector<std::uint32_t>& dispatchVertexIndices =
                candidateVertexFilteringApplied
                ? candidateVertexIndices
                : predictionVertexIndices;
            const std::vector<std::uint32_t>& dispatchVertexCellIndices =
                candidateVertexFilteringApplied
                ? candidateVertexCellIndices
                : grid.vertexCellIndices;
            const std::size_t predictionVertexCount = dispatchVertexIndices.size();
            const std::size_t spatialSkippedVertexCount = candidateVertexFilteringApplied
                ? spatialInputVertexCount - predictionVertexCount
                : 0;
            reportProgress(
                execution,
                0.10,
                (std::string(spatialGridCacheHit
                    ? "Spatial grid cache hit: cells="
                    : "Spatial grid ready: cells=")
                    + std::to_string(grid.cellCount())
                    + ", dimensions=" + std::to_string(grid.dimensions.x())
                    + "x" + std::to_string(grid.dimensions.y())
                    + "x" + std::to_string(grid.dimensions.z())
                    + ", cellSizeMm="
                    + std::to_string(grid.cellSize * kMetersToMillimeters)
                    + ", mode=" + (grid.manualCellSize ? "manual" : "automatic")
                    + ", candidateCellPairs="
                    + std::to_string(grid.candidateCellPairs)
                    + ", candidateVertexPairs="
                    + std::to_string(selectedCandidatePairs)
                    + ", candidateVertices="
                    + std::to_string(predictionVertexCount)
                    + "/" + std::to_string(spatialInputVertexCount)
                    + ", ratio=" + std::to_string(candidateRatio * 100.0)
                    + "% in " + std::to_string(gridMilliseconds) + " ms").c_str());

            if(canceled(execution)) {
                ThicknessPredictionResult result;
                result.warnings.push_back("Thickness prediction was canceled.");
                return result;
            }
            reportProgress(execution, 0.11, "Building thickness BVH");
            const ThicknessBvhProgress bvhProgress = [&execution](
                const char* phase,
                std::size_t current,
                std::size_t total) {
                const double fraction = total > 0
                    ? static_cast<double>(current) / static_cast<double>(total)
                    : 1.0;
                const std::string message = std::string("BVH ") + phase + " ("
                    + std::to_string(current) + "/" + std::to_string(total) + ")";
                reportProgress(execution, 0.11 + 0.04 * fraction, message.c_str());
            };
            const auto bvhStart = std::chrono::steady_clock::now();
            const std::shared_ptr<const ThicknessBvh> bvh =
                task.options.enableBvhOcclusion
                ? cachedBvh(task.workpiece, bvhProgress)
                : std::make_shared<ThicknessBvh>();
            const double bvhMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - bvhStart).count();
            if(task.options.enableBvhOcclusion && bvh->empty()) {
                throw std::runtime_error("Failed to build the workpiece BVH.");
            }
            reportProgress(execution, 0.15, "Thickness BVH ready");

            OpenGLComputeProgram& program = resources.program(
                true,
                task.options.enableBvhOcclusion,
                task.options.enableHistoryCorrection);
            program.use();
            configureSpatialProgram(
                program,
                task,
                static_cast<int>(computationWorkpiece.samples.size()),
                static_cast<int>(predictionVertexCount),
                periodicReductionApplied || axisymmetricProfileApplied
                    || candidateVertexFilteringApplied);
            checkOpenGlErrors("spatial compute program setup");

            const int maximumBlockSize = capabilities.maximumBlockSize;
            if(capabilities.maximumStorageBufferBindings < 14) {
                throw std::runtime_error(
                    "Spatial filtering requires at least 14 shader storage buffer bindings.");
            }
            const std::size_t maximumBufferSize = maximumBlockSize > 0
                ? static_cast<std::size_t>(maximumBlockSize)
                : std::numeric_limits<std::size_t>::max();
            const auto fitsBuffer = [maximumBufferSize](std::size_t bytes) {
                return bytes <= maximumBufferSize;
            };
            if(!fitsBuffer(allSpraySamples.size() * sizeof(SprayGpuSample))
                || !fitsBuffer(grid.cellOffsets.size() * sizeof(std::uint32_t))
                || !fitsBuffer(grid.candidateSprayIndices.size() * sizeof(std::uint32_t))) {
                throw std::runtime_error(
                    "Spatial filtering buffers exceed the OpenGL shader storage block limit.");
            }

            const std::size_t vertexCount = computationWorkpiece.samples.size();
            std::vector<float> thickness(vertexCount, 0.0f);
            std::vector<float> historyTau(vertexCount, 0.0f);
            std::vector<float> lastUpdateTime(
                vertexCount,
                static_cast<float>(allSpraySamples.front().time[0]));
            std::vector<float> historyFactor(vertexCount, 1.0f);
            const std::uint32_t dummyIndex = 0;
            const LegacyGpuBvh legacyBvh = makeLegacyGpuBvh(*bvh);
            const LegacyGpuBvhNode dummyNode{};
            GpuBuffer& surfaceBuffer = resources.buffer(0);
            GpuBuffer& thicknessBuffer = resources.buffer(1);
            GpuBuffer& sprayBuffer = resources.buffer(2);
            GpuBuffer& indexBuffer = resources.buffer(3);
            GpuBuffer& bvhBuffer = resources.buffer(4);
            GpuBuffer& orderBuffer = resources.buffer(5);
            GpuBuffer& historyBuffer = resources.buffer(6);
            GpuBuffer& lastTimeBuffer = resources.buffer(7);
            GpuBuffer& factorBuffer = resources.buffer(8);
            GpuBuffer& predictionVertexBuffer = resources.buffer(9);
            GpuBuffer& surfacePositionBuffer = resources.buffer(10);
            GpuBuffer& vertexCellBuffer = resources.buffer(11);
            GpuBuffer& cellOffsetBuffer = resources.buffer(12);
            GpuBuffer& candidateSprayBuffer = resources.buffer(13);

            const auto uploadStart = std::chrono::steady_clock::now();
            surfaceBuffer.upload(
                0,
                surfaceData.data(),
                surfaceData.size() * sizeof(surfaceData.front()),
                GL_STATIC_DRAW);
            thicknessBuffer.upload(
                1,
                thickness.data(),
                thickness.size() * sizeof(float),
                GL_DYNAMIC_COPY);
            sprayBuffer.upload(
                2,
                allSpraySamples.data(),
                allSpraySamples.size() * sizeof(SprayGpuSample),
                GL_STATIC_DRAW);
            indexBuffer.upload(
                3,
                task.workpiece.triangleIndices.data(),
                task.workpiece.triangleIndices.size() * sizeof(std::uint32_t),
                GL_STATIC_DRAW);
            bvhBuffer.upload(
                4,
                legacyBvh.nodes.empty() ? &dummyNode : legacyBvh.nodes.data(),
                legacyBvh.nodes.empty()
                    ? sizeof(dummyNode)
                    : legacyBvh.nodes.size() * sizeof(LegacyGpuBvhNode),
                GL_STATIC_DRAW);
            orderBuffer.upload(
                5,
                legacyBvh.leafTriangleIndices.empty()
                    ? &dummyIndex
                    : legacyBvh.leafTriangleIndices.data(),
                legacyBvh.leafTriangleIndices.empty()
                    ? sizeof(dummyIndex)
                    : legacyBvh.leafTriangleIndices.size() * sizeof(std::uint32_t),
                GL_STATIC_DRAW);
            historyBuffer.upload(6, historyTau.data(), historyTau.size() * sizeof(float), GL_DYNAMIC_COPY);
            lastTimeBuffer.upload(7, lastUpdateTime.data(), lastUpdateTime.size() * sizeof(float), GL_DYNAMIC_COPY);
            factorBuffer.upload(8, historyFactor.data(), historyFactor.size() * sizeof(float), GL_DYNAMIC_COPY);
            predictionVertexBuffer.upload(
                9,
                dispatchVertexIndices.data(),
                dispatchVertexIndices.size() * sizeof(std::uint32_t),
                GL_STATIC_DRAW);
            vertexCellBuffer.upload(
                11,
                dispatchVertexCellIndices.data(),
                dispatchVertexCellIndices.size() * sizeof(std::uint32_t),
                GL_STATIC_DRAW);
            cellOffsetBuffer.upload(
                12,
                grid.cellOffsets.data(),
                grid.cellOffsets.size() * sizeof(std::uint32_t),
                GL_STATIC_DRAW);
            candidateSprayBuffer.upload(
                13,
                grid.candidateSprayIndices.empty()
                    ? &dummyIndex
                    : grid.candidateSprayIndices.data(),
                grid.candidateSprayIndices.empty()
                    ? sizeof(dummyIndex)
                    : grid.candidateSprayIndices.size() * sizeof(std::uint32_t),
                GL_STATIC_DRAW);
            surfacePositionBuffer.upload(
                10,
                surfacePositionData.data(),
                surfacePositionData.size() * sizeof(surfacePositionData.front()),
                GL_STATIC_DRAW);
            checkOpenGlErrors("spatial input buffer upload");
            const double uploadMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - uploadStart).count();

            const int sprayPointCountLocation = program.uniformLocation("sprayPointCount");
            const auto dispatchStart = std::chrono::steady_clock::now();
            GpuElapsedQuery gpuElapsedQuery;
            gpuElapsedQuery.begin();
            const std::size_t initialBatch = computeAdaptiveBvhBatchSize(
                predictionVertexCount,
                allSpraySamples.size(),
                sizeof(SprayGpuSample),
                maximumBlockSize);
            const std::size_t requestedBatch = task.options.trajectoryBatchSize;
            const std::size_t maximumBatch = std::max<std::size_t>(1, std::min(
                requestedBatch == 0 ? allSpraySamples.size() : requestedBatch,
                allSpraySamples.size()));
            const std::uint64_t batchProfileKey = makeBatchProfileKey(
                capabilities,
                true,
                task.options.enableBvhOcclusion,
                task.options.enableHistoryCorrection,
                predictionVertexCount,
                candidateRatio);
            AdaptiveBatchSelector batchSelector(
                batchProfileKey,
                maximumBatch,
                initialBatch);
            std::vector<std::uint32_t> batchCellOffsets(grid.cellCount() + 1, 0U);
            std::vector<std::uint32_t> batchCandidateSprayIndices;
            std::vector<std::uint32_t> predictionVerticesPerCell(
                grid.cellCount(),
                0U);
            for(std::size_t predictionVertex = 0;
                predictionVertex < dispatchVertexCellIndices.size();
                ++predictionVertex) {
                const std::size_t cell = dispatchVertexCellIndices[predictionVertex];
                if(cell < predictionVerticesPerCell.size()) {
                    ++predictionVerticesPerCell[cell];
                }
            }
            reportProgress(
                execution,
                0.20,
                (std::string("Starting spatial GPU prediction: ")
                    + std::to_string(predictionVertexCount) + " prediction vertices, "
                    + std::to_string(allSpraySamples.size()) + " spray points, "
                    + std::to_string(selectedCandidatePairs)
                    + " candidate pairs, initial batch "
                    + std::to_string(batchSelector.next(allSpraySamples.size()))
                    + ", maximum batch " + std::to_string(maximumBatch)
                    + (batchSelector.wasCached()
                        ? ", profile cache hit"
                        : ", calibrating")).c_str());
            std::size_t begin = 0;
            std::size_t batch = 0;
            while(begin < allSpraySamples.size()) {
                if(canceled(execution)) {
                    ThicknessPredictionResult result;
                    result.warnings.push_back("Thickness prediction was canceled.");
                    return result;
                }
                const std::size_t currentBatchSize = batchSelector.next(
                    allSpraySamples.size() - begin);
                const std::size_t end = std::min(
                    begin + currentBatchSize,
                    allSpraySamples.size());
                batchCellOffsets[0] = 0U;
                batchCandidateSprayIndices.clear();
                std::uint64_t batchCandidatePairs = 0;
                for(std::size_t cell = 0; cell < grid.cellCount(); ++cell) {
                    const auto cellBegin = grid.candidateSprayIndices.begin()
                        + grid.cellOffsets[cell];
                    const auto cellEnd = grid.candidateSprayIndices.begin()
                        + grid.cellOffsets[cell + 1];
                    const auto candidateBegin = std::lower_bound(
                        cellBegin,
                        cellEnd,
                        static_cast<std::uint32_t>(begin));
                    const auto candidateEnd = std::lower_bound(
                        candidateBegin,
                        cellEnd,
                        static_cast<std::uint32_t>(end));
                    batchCandidateSprayIndices.insert(
                        batchCandidateSprayIndices.end(),
                        candidateBegin,
                        candidateEnd);
                    batchCandidatePairs += static_cast<std::uint64_t>(
                        candidateEnd - candidateBegin)
                        * static_cast<std::uint64_t>(
                            predictionVerticesPerCell[cell]);
                    batchCellOffsets[cell + 1] = static_cast<std::uint32_t>(
                        batchCandidateSprayIndices.size());
                }
                const bool measuringBatch = batchSelector.measuring();
                const auto batchStart = std::chrono::steady_clock::now();
                candidateSprayBuffer.update(
                    batchCandidateSprayIndices.empty()
                        ? &dummyIndex
                        : batchCandidateSprayIndices.data(),
                    batchCandidateSprayIndices.empty()
                        ? sizeof(dummyIndex)
                        : batchCandidateSprayIndices.size() * sizeof(std::uint32_t));
                cellOffsetBuffer.update(
                    batchCellOffsets.data(),
                    batchCellOffsets.size() * sizeof(std::uint32_t));
                glUniform1i(
                    sprayPointCountLocation,
                    static_cast<int>(allSpraySamples.size()));
                glDispatchCompute(
                    static_cast<unsigned int>((predictionVertexCount + 255) / 256),
                    1,
                    1);
                glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
                if(measuringBatch || (batch + 1) % 8 == 0
                    || end == allSpraySamples.size()) {
                    glFinish();
                } else {
                    glFlush();
                }
                checkOpenGlErrors("spatial batch dispatch");
                if(measuringBatch) {
                    const double batchMilliseconds =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - batchStart).count();
                    batchSelector.record(
                        currentBatchSize,
                        std::max<std::uint64_t>(1, batchCandidatePairs),
                        batchMilliseconds);
                }
                if(batch < 3 || end == allSpraySamples.size()
                    || end * 100 / allSpraySamples.size()
                        != begin * 100 / allSpraySamples.size()) {
                    reportProgress(
                        execution,
                        0.20 + 0.75 * static_cast<double>(end)
                            / static_cast<double>(allSpraySamples.size()),
                        (std::string("Spatial GPU batch ")
                            + std::to_string(batch + 1)
                            + ": candidates="
                            + std::to_string(batchCandidateSprayIndices.size())).c_str());
                }
                begin = end;
                ++batch;
            }
            batchSelector.finalize();
            checkOpenGlErrors("spatial dispatch");
            const double gpuMilliseconds = gpuElapsedQuery.endAndReadMilliseconds();
            const double dispatchMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - dispatchStart).count();

            const auto downloadStart = std::chrono::steady_clock::now();
            thicknessBuffer.download(thickness.data(), thickness.size() * sizeof(float));
            checkOpenGlErrors("spatial result readback");
            if(axisymmetricProfileApplied) {
                const std::size_t profileOffset = task.workpiece.samples.size();
                const std::size_t profileCount = task.options.axisymmetricProfile
                    .predictionSamples.size();
                std::size_t activeProfileSamples = 0;
                for(std::size_t index = 0; index < profileCount; ++index) {
                    const std::size_t thicknessIndex = profileOffset + index;
                    if(thicknessIndex < thickness.size()
                        && std::abs(thickness[thicknessIndex]) > 1.0e-9f) {
                        ++activeProfileSamples;
                    }
                }
                reportProgress(
                    execution,
                    0.98,
                    (std::string("Axisymmetric profile samples with thickness=")
                        + std::to_string(activeProfileSamples) + "/"
                        + std::to_string(profileCount)).c_str());
            }
            const double downloadMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - downloadStart).count();
            if(periodicReductionApplied) {
                std::vector<float> expandedThickness;
                expandPeriodicSectorThickness(
                    periodicReduction,
                    thickness,
                    expandedThickness);
                thickness = std::move(expandedThickness);
            } else if(axisymmetricProfileApplied) {
                const auto& profile = task.options.axisymmetricProfile;
                const std::size_t profileOffset = task.workpiece.samples.size();
                const auto mappingStart = std::chrono::steady_clock::now();
                const std::shared_ptr<const std::vector<AxisymmetricProfileBinding>>
                    fullVertexBindings = cachedAxisymmetricProfileBindings(
                        task.workpiece, profile, axisymmetricMappingCacheHit);
                axisymmetricBindingCount = fullVertexBindings->size();
                axisymmetricActiveBindingCount = static_cast<std::size_t>(std::count_if(
                    fullVertexBindings->begin(),
                    fullVertexBindings->end(),
                    [](const AxisymmetricProfileBinding& binding) {
                        return binding.active;
                    }));
                std::vector<std::uint8_t> activeSegmentFlags(profile.sampleSegments.size(), 0U);
                std::uint32_t maximumProfilePathIndex = 0;
                for(const auto& segment : profile.sampleSegments) {
                    maximumProfilePathIndex = std::max(
                        maximumProfilePathIndex, segment.profilePathIndex);
                }
                std::vector<std::uint8_t> activeProfilePathFlags(
                    profile.sampleSegments.empty() ? 0 : maximumProfilePathIndex + 1,
                    0U);
                for(const AxisymmetricProfileBinding& binding : *fullVertexBindings) {
                    if(!binding.active) {
                        continue;
                    }
                    if(binding.segmentIndex < activeSegmentFlags.size()) {
                        activeSegmentFlags[binding.segmentIndex] = 1U;
                    }
                    if(binding.segmentIndex < profile.sampleSegments.size()) {
                        const std::uint32_t pathIndex =
                            profile.sampleSegments[binding.segmentIndex].profilePathIndex;
                        if(pathIndex < activeProfilePathFlags.size()) {
                            activeProfilePathFlags[pathIndex] = 1U;
                        }
                    }
                    const std::size_t firstSample = profileOffset + binding.firstSampleIndex;
                    const std::size_t secondSample = profileOffset + binding.secondSampleIndex;
                    if(firstSample >= thickness.size() || secondSample >= thickness.size()) {
                        ++axisymmetricMappedInvalidCount;
                        continue;
                    }
                    const float interpolation = std::clamp(binding.interpolation, 0.0f, 1.0f);
                    const float mappedValue = (1.0f - interpolation) * thickness[firstSample]
                        + interpolation * thickness[secondSample];
                    if(std::abs(mappedValue) <= 1.0e-9f) {
                        ++axisymmetricMappedZeroCount;
                    } else {
                        ++axisymmetricMappedNonzeroCount;
                    }
                }
                axisymmetricActiveSegmentCount = static_cast<std::size_t>(std::count(
                    activeSegmentFlags.begin(), activeSegmentFlags.end(), 1U));
                axisymmetricActiveProfilePathCount = static_cast<std::size_t>(std::count(
                    activeProfilePathFlags.begin(), activeProfilePathFlags.end(), 1U));
                reportProgress(
                    execution,
                    0.985,
                    (std::string(axisymmetricMappingCacheHit
                        ? "Axisymmetric mapping cache hit"
                        : "Axisymmetric mapping built")
                        + "; active bindings="
                        + std::to_string(axisymmetricActiveBindingCount)
                        + "/" + std::to_string(fullVertexBindings->size())
                        + " ("
                        + std::to_string(fullVertexBindings->empty()
                            ? 0.0
                            : 100.0 * static_cast<double>(axisymmetricActiveBindingCount)
                                / static_cast<double>(fullVertexBindings->size()))
                        + "%), segments=" + std::to_string(axisymmetricActiveSegmentCount)
                        + ", profilePaths=" + std::to_string(axisymmetricActiveProfilePathCount)
                        + ", mappedZero=" + std::to_string(axisymmetricMappedZeroCount)
                        + ", mappedNonzero=" + std::to_string(axisymmetricMappedNonzeroCount)
                        + ", mappedInvalid=" + std::to_string(axisymmetricMappedInvalidCount)
                        + ", selection r=["
                        + std::to_string(profile.selectionMinimum.x()) + ","
                        + std::to_string(profile.selectionMaximum.x()) + "], z=["
                        + std::to_string(profile.selectionMinimum.y()) + ","
                        + std::to_string(profile.selectionMaximum.y())
                        + "], polygonPoints="
                        + std::to_string(profile.selectionPolygon.size())).c_str());
                std::vector<float> expandedThickness;
                expandAxisymmetricProfileThickness(
                    *fullVertexBindings,
                    thickness,
                    profileOffset,
                    expandedThickness);
                thickness = std::move(expandedThickness);
                axisymmetricMappingMilliseconds = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - mappingStart).count();
            }
            ThicknessPredictionResult result;
            result.field.resizeFromWorkpiece(task.workpiece);
            for(std::size_t i = 0; i < thickness.size(); ++i) {
                result.field.results[i].thickness = static_cast<double>(thickness[i])
                    * kMillimetersToMeters;
            }
            result.field.updateErrors();
            result.metrics = ThicknessMetricsCalculator::calculate(
                result.field,
                task.options.base);
            result.warnings = std::move(predictionWarnings);
            result.warnings.push_back(
                "Spatial influence filtering was used; values below the Gaussian cutoff were omitted.");
            if(axisymmetricProfileApplied) {
                result.warnings.push_back(
                    "Axisymmetric profile mapping was performed after GPU prediction in "
                    + std::to_string(axisymmetricMappingMilliseconds) + " ms.");
            }
            const double totalMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - gridStart).count();
            result.timing.valid = true;
            result.timing.spatialFiltering = true;
            result.timing.spatialGridCacheHit = spatialGridCacheHit;
            result.timing.axisymmetricMappingCacheHit = axisymmetricMappingCacheHit;
            result.timing.manualSpatialGridCellSize = grid.manualCellSize;
            result.timing.periodicMappingMilliseconds = mappingMilliseconds;
            result.timing.axisymmetricMappingMilliseconds = axisymmetricMappingMilliseconds;
            result.timing.spatialGridMilliseconds = gridMilliseconds;
            result.timing.bvhMilliseconds = bvhMilliseconds;
            result.timing.uploadMilliseconds = uploadMilliseconds;
            result.timing.dispatchMilliseconds = dispatchMilliseconds;
            result.timing.pureGpuMilliseconds = gpuMilliseconds;
            result.timing.readbackMilliseconds = downloadMilliseconds;
            result.timing.backendTotalMilliseconds = totalMilliseconds;
            result.timing.spatialGridCellSizeMeters = grid.cellSize;
            result.timing.predictionVertexCount = predictionVertexCount;
            result.timing.spatialInputVertexCount = spatialInputVertexCount;
            result.timing.spatialSkippedVertexCount = spatialSkippedVertexCount;
            result.timing.sprayPointCount = allSpraySamples.size();
            result.timing.spatialGridCellCount = grid.cellCount();
            result.timing.spatialGridCandidateCellPairs = grid.candidateCellPairs;
            result.timing.spatialGridCandidateVertexPairs = selectedCandidatePairs;
            result.timing.axisymmetricBindingCount = axisymmetricBindingCount;
            result.timing.axisymmetricActiveBindingCount = axisymmetricActiveBindingCount;
            result.timing.axisymmetricMappedZeroCount = axisymmetricMappedZeroCount;
            result.timing.axisymmetricMappedNonzeroCount = axisymmetricMappedNonzeroCount;
            result.timing.axisymmetricMappedInvalidCount = axisymmetricMappedInvalidCount;
            result.timing.axisymmetricActiveSegmentCount = axisymmetricActiveSegmentCount;
            result.timing.axisymmetricActiveProfilePathCount = axisymmetricActiveProfilePathCount;
            result.timing.spatialGridDimensionX = grid.dimensions.x();
            result.timing.spatialGridDimensionY = grid.dimensions.y();
            result.timing.spatialGridDimensionZ = grid.dimensions.z();
            result.timing.adaptiveBatchSize = batchSelector.selectedBatch();
            reportProgress(
                execution,
                1.0,
                (std::string("Spatial GPU prediction completed; grid ")
                    + std::to_string(gridMilliseconds) + " ms, BVH "
                    + std::to_string(bvhMilliseconds) + " ms, upload "
                    + std::to_string(uploadMilliseconds) + " ms, dispatch "
                    + std::to_string(dispatchMilliseconds) + " ms, pure GPU "
                    + std::to_string(gpuMilliseconds) + " ms, readback "
                    + std::to_string(downloadMilliseconds) + " ms, total "
                    + std::to_string(totalMilliseconds) + " ms, candidate ratio "
                    + std::to_string(candidateRatio * 100.0)
                    + "%, adaptive batch "
                    + std::to_string(batchSelector.selectedBatch())).c_str());
            return result;
        }

        void validateTask(const ThicknessPredictionTask& task)
        {
            if(task.model != ThicknessModelKind::PaperGaussian) {
                throw std::runtime_error("The selected GPU thickness model is not supported.");
            }
            if(task.workpiece.samples.empty()) {
                throw std::runtime_error("Thickness prediction requires surface samples.");
            }
            if(task.workpiece.triangleIndices.empty()) {
                throw std::runtime_error("GPU BVH prediction requires workpiece triangle indices.");
            }
            if(task.trajectory.empty()) {
                throw std::runtime_error("Thickness prediction requires a spray trajectory.");
            }
            if(task.options.base.trajectorySamplingMode
                    == TrajectorySamplingMode::ResampleByTimeStep
                && task.options.base.timeStep <= 0.0) {
                throw std::runtime_error("Thickness prediction time step must be positive.");
            }
            if(task.options.axisymmetricProfile.enabled) {
                if(!task.options.spatialFiltering.enabled) {
                    throw std::runtime_error(
                        "Axisymmetric profile prediction requires spatial influence filtering.");
                }
                if(task.options.axisymmetricProfile.predictionSamples.empty()
                    || task.options.axisymmetricProfile.sampleSegments.empty()) {
                    throw std::runtime_error(
                        "Axisymmetric profile prediction inputs are incomplete.");
                }
            }
            if(glGetString(GL_VERSION) == nullptr || glDispatchCompute == nullptr) {
                throw std::runtime_error("An OpenGL 4.3 compute context is not current.");
            }
        }
    }

    namespace
    {
        constexpr std::size_t kOnlineStatisticsGroups = 64;
        // std430: ten consecutive doubles, no trailing struct padding.
        struct OnlinePartialStatistics
        {
            double count, minimum, maximum, mean, m2;
            double errorSum, maxAbsError, covered, under, over;
        };
        static_assert(sizeof(OnlinePartialStatistics) == 80, "GPU statistics layout");

        constexpr const char* kOnlineDisplayCopy = R"glsl(
#version 430 core
layout(local_size_x = 256) in;
layout(std430, binding = 0) readonly buffer Thickness { float thickness[]; };
layout(std430, binding = 1) readonly buffer Mapping { uint indices[]; };
layout(std430, binding = 2) writeonly buffer Display { float display[]; };
uniform int count;
void main() {
    uint i = gl_GlobalInvocationID.x;
    if (i < uint(count)) display[i] = thickness[indices[i]];
}
)glsl";

        constexpr const char* kOnlineStatistics = R"glsl(
#version 430 core
layout(local_size_x = 256) in;
layout(std430, binding = 0) readonly buffer Thickness { float thickness[]; };
layout(std430, binding = 1) readonly buffer Targets { double targets[]; };
struct Statistics {
    double count, minimum, maximum, mean, m2;
    double errorSum, maxAbsError, covered, under, over;
};
layout(std430, binding = 2) writeonly buffer Partials { Statistics partials[]; };
uniform int count;
uniform bool fullStatistics;
uniform double coverageTolerance;
uniform double overCoatTolerance;
shared Statistics localStats[256];
Statistics mergeStats(Statistics a, Statistics b) {
    if (b.count == 0.0) return a;
    if (a.count == 0.0) return b;
    double n = a.count + b.count;
    if (!fullStatistics) {
        a.count = n;
        a.minimum = min(a.minimum, b.minimum);
        a.maximum = max(a.maximum, b.maximum);
        return a;
    }
    double delta = b.mean - a.mean;
    a.m2 += b.m2 + delta * delta * a.count * b.count / n;
    a.mean += delta * b.count / n;
    a.count = n;
    a.minimum = min(a.minimum, b.minimum);
    a.maximum = max(a.maximum, b.maximum);
    a.errorSum += b.errorSum;
    a.maxAbsError = max(a.maxAbsError, b.maxAbsError);
    a.covered += b.covered;
    a.under += b.under;
    a.over += b.over;
    return a;
}
void main() {
    Statistics s = Statistics(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for (uint i = gl_GlobalInvocationID.x; i < uint(count);
         i += gl_NumWorkGroups.x * gl_WorkGroupSize.x) {
        float raw = thickness[i];
        if (isnan(raw) || isinf(raw)) continue;
        double value = double(raw) * 0.001lf;
        if (!fullStatistics) {
            if (s.count == 0.0) { s.minimum = value; s.maximum = value; }
            s.minimum = min(s.minimum, value);
            s.maximum = max(s.maximum, value);
            s.count += 1.0;
            continue;
        }
        double target = targets[i];
        double error = value - target;
        double lower = target - coverageTolerance;
        double upper = target + overCoatTolerance;
        Statistics sampleValue = Statistics(1.0, value, value, value, 0.0,
            error, abs(error), value >= lower && value <= upper ? 1.0 : 0.0,
            value < lower ? 1.0 : 0.0, value > upper ? 1.0 : 0.0);
        s = mergeStats(s, sampleValue);
    }
    uint lane = gl_LocalInvocationID.x;
    localStats[lane] = s;
    barrier();
    for (uint stride = 128; stride > 0; stride >>= 1) {
        if (lane < stride) localStats[lane] = mergeStats(localStats[lane], localStats[lane + stride]);
        barrier();
    }
    if (lane == 0) partials[gl_WorkGroupID.x] = localStats[0];
}
)glsl";
    }

    struct OpenGLThicknessPredictionBackend::Impl
    {
        BackendGpuResources resources;
        GpuCapabilities capabilities;
        bool capabilitiesReady{ false };
        std::shared_ptr<ThicknessPredictionTask> onlineTask;
        std::shared_ptr<const ThicknessBvh> onlineBvh;
        std::uint64_t onlineGeometryHash = 0;
        OnlineThicknessSnapshot onlineLastStatistics;
        std::vector<float> onlineThickness;
        std::vector<double> onlineTargets;
        std::vector<std::unique_ptr<GpuElapsedQuery>> onlineQueries;
        std::size_t onlineSprayPointCount{ 0 };
        std::unique_ptr<GpuBuffer> onlineDisplayIndices;
        std::unique_ptr<GpuBuffer> onlineTargetBuffer;
        std::unique_ptr<GpuBuffer> onlineStatisticsBuffer;
        std::unique_ptr<OpenGLComputeProgram> onlineDisplayProgram;
        std::unique_ptr<OpenGLComputeProgram> onlineStatisticsProgram;
        std::unique_ptr<GpuElapsedQuery> onlineDisplayQuery;
        std::unique_ptr<GpuElapsedQuery> onlineStatisticsQuery;
        std::size_t onlineDisplayCount = 0;

        const GpuCapabilities& deviceCapabilities()
        {
            if(!capabilitiesReady) {
                capabilities = queryGpuCapabilities();
                capabilitiesReady = true;
            }
            return capabilities;
        }
    };

    OpenGLThicknessPredictionBackend::OpenGLThicknessPredictionBackend()
        : m_impl(std::make_unique<Impl>())
    {
    }

    OpenGLThicknessPredictionBackend::~OpenGLThicknessPredictionBackend() = default;

    void OpenGLThicknessPredictionBackend::beginOnline(ThicknessPredictionTask task)
    {
        if(task.model != ThicknessModelKind::PaperGaussian
            || task.workpiece.samples.empty()
            || task.workpiece.triangleIndices.empty()
            || task.options.periodicLocal.enabled
            || task.options.axisymmetricProfile.enabled
            || task.options.spatialFiltering.enabled) {
            throw std::runtime_error("Online prediction requires a complete triangular workpiece and Paper Gaussian model.");
        }
        if(glGetString(GL_VERSION) == nullptr || glDispatchCompute == nullptr) {
            throw std::runtime_error("An OpenGL 4.3 compute context is not current.");
        }

        const auto geometryHash = hashWorkpieceGeometry(task.workpiece);
        const bool reuseGeometry = m_impl->onlineTask && m_impl->onlineGeometryHash == geometryHash;
        const std::size_t vertexCount = task.workpiece.samples.size();
        // Also used by motion integration when occlusion is disabled.
        const auto bvh = reuseGeometry ? m_impl->onlineBvh : cachedBvh(task.workpiece, {});
        if(task.options.enableBvhOcclusion && bvh->empty()) {
            throw std::runtime_error("Failed to build the online workpiece BVH.");
        }
        const LegacyGpuBvhNode dummyNode{};
        const std::uint32_t dummyIndex = 0;
        std::vector<float> thickness(vertexCount, 0.0f);
        std::vector<float> historyTau(vertexCount, 0.0f);
        std::vector<float> lastUpdateTime(vertexCount, 0.0f);
        std::vector<float> historyFactor(vertexCount, 1.0f);
        BackendGpuResources& resources = m_impl->resources;
        if(!reuseGeometry) {
            const auto surfaceData = makeSurfaceData(task.workpiece);
            const auto surfacePositions = makeSurfacePositionData(task.workpiece);
            const LegacyGpuBvh legacyBvh = makeLegacyGpuBvh(*bvh);
            resources.buffer(0).upload(0, surfaceData.data(),
                surfaceData.size() * sizeof(surfaceData.front()), GL_STATIC_DRAW);
            resources.buffer(3).upload(3, task.workpiece.triangleIndices.data(),
                task.workpiece.triangleIndices.size() * sizeof(std::uint32_t), GL_STATIC_DRAW);
            resources.buffer(4).upload(4,
                legacyBvh.nodes.empty() ? &dummyNode : legacyBvh.nodes.data(),
                legacyBvh.nodes.empty() ? sizeof(dummyNode)
                    : legacyBvh.nodes.size() * sizeof(LegacyGpuBvhNode), GL_STATIC_DRAW);
            resources.buffer(5).upload(5,
                legacyBvh.leafTriangleIndices.empty() ? &dummyIndex
                    : legacyBvh.leafTriangleIndices.data(),
                legacyBvh.leafTriangleIndices.empty() ? sizeof(dummyIndex)
                    : legacyBvh.leafTriangleIndices.size() * sizeof(std::uint32_t), GL_STATIC_DRAW);
            resources.buffer(10).upload(10, surfacePositions.data(),
                surfacePositions.size() * sizeof(surfacePositions.front()), GL_STATIC_DRAW);
        }
        resources.buffer(1).upload(1, thickness.data(),
            thickness.size() * sizeof(float), GL_DYNAMIC_COPY);
        resources.buffer(6).upload(6, historyTau.data(),
            historyTau.size() * sizeof(float), GL_DYNAMIC_COPY);
        resources.buffer(7).upload(7, lastUpdateTime.data(),
            lastUpdateTime.size() * sizeof(float), GL_DYNAMIC_COPY);
        resources.buffer(8).upload(8, historyFactor.data(),
            historyFactor.size() * sizeof(float), GL_DYNAMIC_COPY);
        resources.buffer(9).upload(9, &dummyIndex, sizeof(dummyIndex), GL_STATIC_DRAW);
        checkOpenGlErrors("online input upload");
        // The targets stay fixed for this session. Read their compact array
        // during conversion instead of streaming the entire surface sample array.
        m_impl->onlineTargets.resize(task.workpiece.samples.size());
        for(std::size_t index = 0; index < task.workpiece.samples.size(); ++index) {
            m_impl->onlineTargets[index] = task.workpiece.samples[index].targetThickness;
        }
        m_impl->onlineTask = std::make_shared<ThicknessPredictionTask>(std::move(task));
        m_impl->onlineBvh = bvh;
        m_impl->onlineGeometryHash = geometryHash;
        m_impl->onlineLastStatistics = {};
        m_impl->onlineDisplayCount = 0;
        // Compile before the first physical interval arrives.
        m_impl->resources.program(false, m_impl->onlineTask->options.enableBvhOcclusion,
            m_impl->onlineTask->options.enableHistoryCorrection);
        m_impl->onlineThickness = std::move(thickness);
        m_impl->onlineSprayPointCount = 0;
    }

    std::function<double(const Eigen::Vector3d&)>
        OpenGLThicknessPredictionBackend::onlineSurfaceDistanceQuery() const
    {
        const auto task = m_impl->onlineTask;
        const auto bvh = m_impl->onlineBvh;
        return [task, bvh](const Eigen::Vector3d& position) {
            return bvh->surfaceDistance(task->workpiece, position);
        };
    }

    void OpenGLThicknessPredictionBackend::copyOnlineVisibilityGeometry(
        const std::array<unsigned int, 4>& buffers)
    {
        if(!m_impl->onlineTask) throw std::runtime_error("Online geometry is not prepared.");
        GLint previousRead = 0, previousWrite = 0;
        glGetIntegerv(GL_COPY_READ_BUFFER_BINDING, &previousRead);
        glGetIntegerv(GL_COPY_WRITE_BUFFER_BINDING, &previousWrite);
        const std::array<unsigned int, 4> bindings{ 3, 4, 5, 10 };
        for(std::size_t i = 0; i < bindings.size(); ++i) {
            m_impl->resources.buffer(bindings[i]).bind(bindings[i]);
            GLint source = 0;
            glGetIntegeri_v(GL_SHADER_STORAGE_BUFFER_BINDING, bindings[i], &source);
            glBindBuffer(GL_COPY_READ_BUFFER, static_cast<GLuint>(source));
            const auto bytes = m_impl->resources.buffer(bindings[i]).size();
            glBindBuffer(GL_COPY_WRITE_BUFFER, buffers[i]);
            glBufferData(GL_COPY_WRITE_BUFFER, static_cast<GLsizeiptr>(bytes), nullptr, GL_STATIC_COPY);
            glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0,
                static_cast<GLsizeiptr>(bytes));
        }
        glBindBuffer(GL_COPY_READ_BUFFER, static_cast<GLuint>(previousRead));
        glBindBuffer(GL_COPY_WRITE_BUFFER, static_cast<GLuint>(previousWrite));
        checkOpenGlErrors("online visibility geometry copy");
    }

    ThicknessPredictionResult OpenGLThicknessPredictionBackend::appendOnline(
        const spraytrajectory::SprayTrajectory& trajectory)
    {
        ThicknessPredictionResult result;
        appendOnline(trajectory, result);
        return result;
    }

    void OpenGLThicknessPredictionBackend::appendOnline(
        const spraytrajectory::SprayTrajectory& trajectory, ThicknessPredictionResult& result)
    {
        appendOnlineImpl(trajectory, &result, nullptr);
    }

    void OpenGLThicknessPredictionBackend::appendOnline(
        const spraytrajectory::SprayTrajectory& trajectory, OnlineThicknessSnapshot& snapshot)
    {
        appendOnlineImpl(trajectory, nullptr, &snapshot);
    }

    void OpenGLThicknessPredictionBackend::setOnlineDisplayMapping(
        const std::vector<std::uint32_t>& sampleIndices)
    {
        if(!m_impl->onlineTask || sampleIndices.empty()) {
            throw std::runtime_error("Online display requires a model and vertex mapping.");
        }
        for(const auto index : sampleIndices) {
            if(index >= m_impl->onlineTask->workpiece.samples.size()) {
                throw std::runtime_error("Online display mapping references an invalid sample.");
            }
        }
        m_impl->onlineDisplayIndices = std::make_unique<GpuBuffer>();
        m_impl->onlineTargetBuffer = std::make_unique<GpuBuffer>();
        m_impl->onlineStatisticsBuffer = std::make_unique<GpuBuffer>();
        m_impl->onlineDisplayIndices->upload(0, sampleIndices.data(),
            sampleIndices.size() * sizeof(std::uint32_t), GL_STATIC_DRAW);
        m_impl->onlineTargetBuffer->upload(0, m_impl->onlineTargets.data(),
            m_impl->onlineTargets.size() * sizeof(double), GL_STATIC_DRAW);
        const std::array<OnlinePartialStatistics, kOnlineStatisticsGroups> empty{};
        m_impl->onlineStatisticsBuffer->upload(0, empty.data(), sizeof(empty), GL_DYNAMIC_READ);
        m_impl->onlineDisplayCount = sampleIndices.size();
        if(!m_impl->onlineDisplayProgram) {
            m_impl->onlineDisplayProgram = std::make_unique<OpenGLComputeProgram>(
                kOnlineDisplayCopy, "Online display gather");
        }
        if(!m_impl->onlineStatisticsProgram) {
            m_impl->onlineStatisticsProgram = std::make_unique<OpenGLComputeProgram>(
                kOnlineStatistics, "Online thickness statistics");
        }
        if(!m_impl->onlineDisplayQuery) m_impl->onlineDisplayQuery = std::make_unique<GpuElapsedQuery>();
        if(!m_impl->onlineStatisticsQuery) m_impl->onlineStatisticsQuery = std::make_unique<GpuElapsedQuery>();
        checkOpenGlErrors("online display mapping upload");
    }

    void OpenGLThicknessPredictionBackend::setOnlineToolDirections(
        const Eigen::Vector3d& sprayDirectionLocal, const Eigen::Vector3d& powderFeedDirectionLocal)
    {
        if(!m_impl->onlineTask) {
            throw std::runtime_error("Online thickness prediction has not been started.");
        }
        if(!sprayDirectionLocal.allFinite() || !powderFeedDirectionLocal.allFinite()
            || sprayDirectionLocal.squaredNorm() < 1.0e-12
            || powderFeedDirectionLocal.squaredNorm() < 1.0e-12) {
            throw std::runtime_error("Online tool directions must be finite and nonzero.");
        }
        const Eigen::Vector3d spray = sprayDirectionLocal.normalized();
        const Eigen::Vector3d powder = powderFeedDirectionLocal.normalized();
        if(spray.cross(powder).squaredNorm() < 1.0e-12) {
            throw std::runtime_error("Online powder feed direction must not be parallel to the spray direction.");
        }
        m_impl->onlineTask->tool.sprayDirectionLocal = spray;
        m_impl->onlineTask->tool.powderFeedDirectionLocal = powder;
    }

    void OpenGLThicknessPredictionBackend::appendOnlineGpu(
        const spraytrajectory::SprayTrajectory& trajectory, unsigned int displayBuffer,
        OnlineThicknessSnapshot& snapshot, const std::function<bool()>& canceled, bool fullStatistics)
    {
        if(displayBuffer == 0 || m_impl->onlineDisplayCount == 0) {
            throw std::runtime_error("Online GPU display buffer or mapping is unavailable.");
        }
        appendOnlineImpl(trajectory, nullptr, &snapshot, displayBuffer, canceled, fullStatistics);
    }

    void OpenGLThicknessPredictionBackend::appendOnlineImpl(
        const spraytrajectory::SprayTrajectory& trajectory,
        ThicknessPredictionResult* result, OnlineThicknessSnapshot* snapshot,
        unsigned int displayBuffer, const std::function<bool()>& canceled, bool fullStatistics)
    {
        if(!m_impl->onlineTask) {
            throw std::runtime_error("Online thickness prediction has not been started.");
        }
        const auto start = std::chrono::steady_clock::now();
        ThicknessPredictionTask& task = *m_impl->onlineTask;
        auto& thickness = snapshot ? snapshot->thicknessMillimeters : m_impl->onlineThickness;
        if(displayBuffer) thickness.clear();
        else thickness.resize(task.workpiece.samples.size());
        auto& timing = snapshot ? snapshot->timing : result->timing;
        timing = {};
        if(snapshot) {
            snapshot->residentVertexCount = displayBuffer ? task.workpiece.samples.size() : 0;
            snapshot->finiteVertexCount = 0;
            snapshot->varianceSquareMeters = 0.0;
        }
        task.trajectory = trajectory;
        const auto spraySamples = makePeriodicSpraySamples(task);
        task.trajectory = spraytrajectory::SprayTrajectory();
        const GpuCapabilities& capabilities = m_impl->deviceCapabilities();
        OpenGLComputeProgram& program = m_impl->resources.program(
            false, task.options.enableBvhOcclusion,
            task.options.enableHistoryCorrection);
        program.use();
        // Display gather/reduction use the same binding points in this context.
        for(unsigned int binding = 0; binding <= 10; ++binding) {
            m_impl->resources.buffer(binding).bind(binding);
        }
        configureProgram(program, task,
            static_cast<int>(task.workpiece.samples.size()),
            static_cast<int>(task.workpiece.samples.size()), false);
        const int sprayPointCountLocation = program.uniformLocation("sprayPointCount");
        const std::size_t maximumBatch = std::max<std::size_t>(1,
            static_cast<std::size_t>(capabilities.maximumBlockSize)
                / sizeof(SprayGpuSample));
        double uploadMilliseconds = 0.0;
        double dispatchMilliseconds = 0.0;
        double gpuMilliseconds = 0.0;
        double readbackMilliseconds = 0.0;
        std::size_t queryCount = 0;
        for(std::size_t begin = 0; begin < spraySamples.size();) {
            const auto uploadStart = std::chrono::steady_clock::now();
            const std::size_t count = std::min(maximumBatch, spraySamples.size() - begin);
            std::vector<SprayGpuSample> gpuSamples;
            gpuSamples.reserve(count);
            for(std::size_t index = begin; index < begin + count; ++index) {
                gpuSamples.push_back(makeGpuSpraySample(spraySamples[index]));
            }
            m_impl->resources.buffer(2).upload(2, gpuSamples.data(),
                gpuSamples.size() * sizeof(SprayGpuSample), GL_DYNAMIC_DRAW);
            glUniform1i(sprayPointCountLocation, static_cast<int>(count));
            const auto dispatchStart = std::chrono::steady_clock::now();
            uploadMilliseconds += std::chrono::duration<double, std::milli>(
                dispatchStart - uploadStart).count();
            if(queryCount == m_impl->onlineQueries.size()) {
                m_impl->onlineQueries.push_back(std::make_unique<GpuElapsedQuery>());
            }
            GpuElapsedQuery& query = *m_impl->onlineQueries[queryCount++];
            query.begin();
            glDispatchCompute(
                static_cast<GLuint>((task.workpiece.samples.size() + 255) / 256), 1, 1);
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);
            // Defer reading the timer until the required thickness download has
            // completed. Do not introduce a separate GPU wait before readback.
            query.end();
            dispatchMilliseconds += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - dispatchStart).count();
            begin += count;
        }
        if(displayBuffer) {
            auto& copyQuery = *m_impl->onlineDisplayQuery;
            auto& statisticsQuery = *m_impl->onlineStatisticsQuery;
            auto& copyProgram = *m_impl->onlineDisplayProgram;
            copyProgram.use();
            m_impl->resources.buffer(1).bind(0);
            m_impl->onlineDisplayIndices->bind(1);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, displayBuffer);
            copyProgram.setInt("count", static_cast<int>(m_impl->onlineDisplayCount));
            copyQuery.begin();
            glDispatchCompute(static_cast<GLuint>((m_impl->onlineDisplayCount + 255) / 256), 1, 1);
            glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);
            copyQuery.end();

            auto& statisticsProgram = *m_impl->onlineStatisticsProgram;
            statisticsProgram.use();
            m_impl->onlineTargetBuffer->bind(1);
            m_impl->onlineStatisticsBuffer->bind(2);
            statisticsProgram.setInt("count", static_cast<int>(task.workpiece.samples.size()));
            statisticsProgram.setInt("fullStatistics", fullStatistics ? 1 : 0);
            glUniform1d(statisticsProgram.uniformLocation("coverageTolerance"), task.options.base.coverageTolerance);
            glUniform1d(statisticsProgram.uniformLocation("overCoatTolerance"), task.options.base.overCoatTolerance);
            statisticsQuery.begin();
            glDispatchCompute(static_cast<GLuint>(kOnlineStatisticsGroups), 1, 1);
            glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
            statisticsQuery.end();

            // Wait only on the worker, and publish an already completed immutable
            // frame. The GUI never waits for prediction or reads an unfinished field.
            const auto waitStart = std::chrono::steady_clock::now();
            GLsync fence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
            if(!fence) throw std::runtime_error("Failed to create the online GPU completion fence.");
            glFlush();
            GLenum status = GL_TIMEOUT_EXPIRED;
            while(status == GL_TIMEOUT_EXPIRED) {
                if(canceled && canceled()) {
                    glDeleteSync(fence);
                    throw std::runtime_error("Online GPU frame canceled.");
                }
                status = glClientWaitSync(fence, 0, 1000000);
            }
            glDeleteSync(fence);
            if(status == GL_WAIT_FAILED) throw std::runtime_error("Online GPU completion wait failed.");
            timing.gpuCompletionWaitMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - waitStart).count();
            timing.gpuDisplayCopyMilliseconds = copyQuery.readMilliseconds();
            timing.gpuStatisticsMilliseconds = statisticsQuery.readMilliseconds();

            const auto readStart = std::chrono::steady_clock::now();
            std::array<OnlinePartialStatistics, kOnlineStatisticsGroups> partials{};
            m_impl->onlineStatisticsBuffer->download(partials.data(), sizeof(partials));
            timing.statisticsReadbackBytes = sizeof(partials);
            readbackMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - readStart).count();
            const auto mergeStart = std::chrono::steady_clock::now();
            OnlinePartialStatistics total{};
            for(const auto& p : partials) {
                if(p.count == 0.0) continue;
                if(total.count == 0.0) { total = p; continue; }
                const double count = total.count + p.count;
                const double delta = p.mean - total.mean;
                total.m2 += p.m2 + delta * delta * total.count * p.count / count;
                total.mean += delta * p.count / count;
                total.count = count;
                total.minimum = std::min(total.minimum, p.minimum);
                total.maximum = std::max(total.maximum, p.maximum);
                total.errorSum += p.errorSum;
                total.maxAbsError = std::max(total.maxAbsError, p.maxAbsError);
                total.covered += p.covered;
                total.under += p.under;
                total.over += p.over;
            }
            snapshot->metrics = {};
            if(total.count > 0.0) {
                auto& metrics = snapshot->metrics;
                metrics.minThickness = total.minimum;
                metrics.maxThickness = total.maximum;
                metrics.averageThickness = total.mean;
                metrics.meanError = total.errorSum / total.count;
                metrics.maxAbsError = total.maxAbsError;
                metrics.coverageRatio = total.covered / total.count;
                metrics.underCoatedRatio = total.under / total.count;
                metrics.overCoatedRatio = total.over / total.count;
                snapshot->finiteVertexCount = static_cast<std::size_t>(total.count);
                snapshot->varianceSquareMeters = std::max(0.0, total.m2 / total.count);
            }
            timing.resultConversionMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - mergeStart).count();
            if(fullStatistics) {
                m_impl->onlineLastStatistics.metrics = snapshot->metrics;
                m_impl->onlineLastStatistics.varianceSquareMeters = snapshot->varianceSquareMeters;
            } else {
                const auto minimum = snapshot->metrics.minThickness;
                const auto maximum = snapshot->metrics.maxThickness;
                snapshot->metrics = m_impl->onlineLastStatistics.metrics;
                snapshot->metrics.minThickness = minimum;
                snapshot->metrics.maxThickness = maximum;
                snapshot->varianceSquareMeters = m_impl->onlineLastStatistics.varianceSquareMeters;
            }
            timing.gpuResidentDisplay = true;
        } else {
            const auto readbackStart = std::chrono::steady_clock::now();
            // Read directly into the released frame buffer. Full and compact
            // callers may alternate, including a snapshot with no new spray.
            m_impl->resources.buffer(1).download(
                thickness.data(), thickness.size() * sizeof(float));
            readbackMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - readbackStart).count();
            timing.thicknessReadbackBytes = thickness.size() * sizeof(float);
        }
        m_impl->onlineSprayPointCount += spraySamples.size();
        const auto timerReadStart = std::chrono::steady_clock::now();
        for(std::size_t query = 0; query < queryCount; ++query) {
            gpuMilliseconds += m_impl->onlineQueries[query]->readMilliseconds();
        }
        const double timerReadMilliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - timerReadStart).count();
        checkOpenGlErrors("online prediction dispatch");

        // Compact frames avoid writing four result fields per vertex. The same
        // metric rules and double-precision unit conversion serve both paths.
        const auto conversionStart = std::chrono::steady_clock::now();
        const auto collect = [&](auto store) {
            std::size_t blocks = 1;
#ifdef _MSC_VER
            constexpr std::size_t maximumBlocks = 8;
            // The MSVC standard library reuses the Windows thread pool. Keep
            // work coarse, bounded, and leave CPU capacity for the GUI thread.
            static const std::size_t cpuBudget = std::max(1u, std::thread::hardware_concurrency() / 2);
            blocks = std::min({ maximumBlocks, cpuBudget,
                std::max<std::size_t>(1, thickness.size() / 65536) });
            std::array<std::size_t, maximumBlocks> blockIndices{ 0, 1, 2, 3, 4, 5, 6, 7 };
#endif
            std::vector<ThicknessMetricsAccumulator> partials(
                blocks, ThicknessMetricsAccumulator(task.options.base));
            const auto convertBlock = [&](std::size_t block) {
                const std::size_t begin = thickness.size() * block / blocks;
                const std::size_t end = thickness.size() * (block + 1) / blocks;
                ThicknessMetricsAccumulator local(task.options.base);
                for(std::size_t index = begin; index < end; ++index) {
                    ThicknessSampleResult sample;
                    sample.sampleIndex = index;
                    sample.targetThickness = m_impl->onlineTargets[index];
                    sample.thickness = static_cast<double>(thickness[index]) * kMillimetersToMeters;
                    sample.error = sample.thickness - sample.targetThickness;
                    local.add(sample);
                    store(index, sample);
                }
                partials[block] = local;
            };
#ifdef _MSC_VER
            if(blocks > 1) {
                std::for_each(std::execution::par, blockIndices.begin(),
                    blockIndices.begin() + blocks, convertBlock);
            } else
#endif
            {
                convertBlock(0);
            }
            ThicknessMetricsAccumulator metrics(task.options.base);
            for(const auto& partial : partials) metrics.merge(partial);
            return metrics.metrics();
        };
        if(displayBuffer) {
            // Metrics were reduced on the GPU; no per-vertex CPU conversion.
        } else if(snapshot) {
            snapshot->metrics = collect([](std::size_t, const ThicknessSampleResult&) {});
        } else {
            result->field.results.resize(thickness.size());
            result->metrics = collect([&](std::size_t index, const ThicknessSampleResult& sample) {
                result->field.results[index] = sample;
            });
        }
        if(!displayBuffer) {
            timing.resultConversionMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - conversionStart).count();
        }
        timing.gpuTimerReadMilliseconds = timerReadMilliseconds;
        timing.valid = true;
        timing.predictionVertexCount = task.workpiece.samples.size();
        timing.sprayPointCount = m_impl->onlineSprayPointCount;
        timing.uploadMilliseconds = uploadMilliseconds;
        timing.dispatchMilliseconds = dispatchMilliseconds;
        timing.pureGpuMilliseconds = gpuMilliseconds;
        timing.readbackMilliseconds = readbackMilliseconds;
        timing.backendTotalMilliseconds =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
    }

    void OpenGLThicknessPredictionBackend::endOnline()
    {
        m_impl->onlineTask.reset();
        m_impl->onlineBvh.reset();
        m_impl->onlineGeometryHash = 0;
        m_impl->onlineThickness.clear();
        m_impl->onlineTargets.clear();
        m_impl->onlineSprayPointCount = 0;
        m_impl->onlineDisplayCount = 0;
        m_impl->onlineDisplayIndices.reset();
        m_impl->onlineTargetBuffer.reset();
        m_impl->onlineStatisticsBuffer.reset();
    }

    ThicknessPredictionResult OpenGLThicknessPredictionBackend::predict(
        const ThicknessPredictionTask& task,
        const ThicknessPredictionExecution& execution)
    {
        // Offline dispatches reuse these bindings and invalidate prepared online geometry.
        endOnline();
        const auto totalStart = std::chrono::steady_clock::now();
        validateTask(task);
        checkOpenGlErrors("prediction start");
        const GpuCapabilities& capabilities = m_impl->deviceCapabilities();
        reportProgress(
            execution,
            0.02,
            (std::string("Preparing surface and trajectory data; GPU renderer=")
                + capabilities.renderer + ", SSBO block="
                + std::to_string(capabilities.maximumBlockSize)
                + ", SSBO bindings="
                + std::to_string(capabilities.maximumStorageBufferBindings)).c_str());
        sprayworkpiece::WorkpieceModel computationWorkpiece = task.workpiece;
        if(task.options.axisymmetricProfile.enabled) {
            computationWorkpiece.samples.insert(
                computationWorkpiece.samples.end(),
                task.options.axisymmetricProfile.predictionSamples.begin(),
                task.options.axisymmetricProfile.predictionSamples.end());
        }
        const auto surfaceData = makeSurfaceData(computationWorkpiece);
        const auto surfacePositionData = makeSurfacePositionData(computationWorkpiece);
        const auto previewSpraySamples = makePeriodicSpraySamples(task);
        std::vector<SprayGpuSample> allSpraySamples;
        allSpraySamples.reserve(previewSpraySamples.size());
        for(const PeriodicSpraySample& sample : previewSpraySamples) {
            allSpraySamples.push_back(makeGpuSpraySample(sample));
        }
        if(allSpraySamples.empty()) {
            throw std::runtime_error("Trajectory sampling produced no active spray intervals.");
        }
        if(task.options.spatialFiltering.enabled) {
            try {
                return predictSpatialFiltered(
                    task,
                    computationWorkpiece,
                    execution,
                    surfaceData,
                    surfacePositionData,
                    previewSpraySamples,
                    allSpraySamples,
                    m_impl->resources,
                    m_impl->deviceCapabilities());
            } catch(const std::exception& exception) {
                if(!task.options.spatialFiltering.fallbackToFullPrediction) {
                    throw;
                }
                reportProgress(
                    execution,
                    0.04,
                    (std::string("Spatial filtering unavailable; falling back to the original algorithm: ")
                        + exception.what()).c_str());
                ThicknessPredictionTask fallbackTask = task;
                fallbackTask.options.spatialFiltering.enabled = false;
                ThicknessPredictionResult fallbackResult = predict(
                    fallbackTask,
                    execution);
                fallbackResult.warnings.insert(
                    fallbackResult.warnings.begin(),
                    std::string("Spatial filtering fell back to the original algorithm: ")
                        + exception.what());
                return fallbackResult;
            }
        }
        std::vector<SprayGpuSample> spraySamples = allSpraySamples;
        std::vector<std::uint32_t> predictionVertexIndices(task.workpiece.samples.size());
        std::iota(predictionVertexIndices.begin(), predictionVertexIndices.end(), 0U);
        PeriodicSectorReduction periodicReduction;
        bool periodicReductionApplied = false;
        double mappingMilliseconds = 0.0;
        std::vector<std::string> predictionWarnings;
        if(task.options.periodicLocal.enabled) {
            reportProgress(execution, 0.04, "Building periodic base-sector mapping");
            const auto mappingStart = std::chrono::steady_clock::now();
            bool cpuMappingFallback = false;
            std::string gpuMappingFailure;
            PeriodicGpuMappingCallbacks mappingCallbacks;
            mappingCallbacks.canceled = [&execution]() {
                return canceled(execution);
            };
            mappingCallbacks.progress = [&execution](
                std::size_t current,
                std::size_t total,
                const char* phase) {
                const double fraction = total > 0
                    ? static_cast<double>(current) / static_cast<double>(total)
                    : 1.0;
                const double progress = 0.04 + 0.05 * fraction;
                const std::string message = std::string("Periodic ") + phase + " ("
                    + std::to_string(current) + "/" + std::to_string(total) + ")";
                reportProgress(execution, progress, message.c_str());
            };
            try {
                periodicReduction = buildPeriodicSectorReductionGpu(
                    task.workpiece,
                    task.options.periodicLocal,
                    mappingCallbacks);
            } catch(const std::exception& exception) {
                gpuMappingFailure = exception.what();
            }
            if(canceled(execution)) {
                ThicknessPredictionResult result;
                result.warnings.push_back("Thickness prediction was canceled.");
                return result;
            }
            if(!gpuMappingFailure.empty() || !periodicReduction.valid()) {
                if(gpuMappingFailure.empty()) {
                    gpuMappingFailure = periodicReduction.failureReason;
                }
                reportProgress(
                    execution,
                    0.045,
                    (std::string("GPU periodic mapping unavailable; using CPU fallback: ")
                        + gpuMappingFailure).c_str());
                cpuMappingFallback = true;
                periodicReduction = buildPeriodicSectorReduction(
                    task.workpiece, task.options.periodicLocal);
            }
            mappingMilliseconds = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - mappingStart).count();
            if(periodicReduction.valid()) {
                reportProgress(
                    execution,
                    0.09,
                    (std::string(periodicReduction.mappingCacheHit
                        ? "Periodic mapping cache hit"
                        : (cpuMappingFallback
                            ? "Periodic CPU fallback mapping built"
                            : "Periodic GPU mapping built"))
                        + " in " + std::to_string(mappingMilliseconds) + " ms").c_str());
                predictionVertexIndices = periodicReduction.predictionVertexIndices;
                std::vector<std::size_t> selectedSprayIndices;
                if(task.options.periodicLocal.reduceTrajectory) {
                    selectedSprayIndices = selectPeriodicSpraySampleIndices(
                        previewSpraySamples,
                        periodicReduction,
                        task.options);
                } else {
                    selectedSprayIndices.resize(previewSpraySamples.size());
                    std::iota(selectedSprayIndices.begin(), selectedSprayIndices.end(), 0U);
                }
                spraySamples.clear();
                spraySamples.reserve(selectedSprayIndices.size());
                for(const std::size_t index : selectedSprayIndices) {
                    if(index < allSpraySamples.size()) {
                        spraySamples.push_back(allSpraySamples[index]);
                    }
                }
                periodicReductionApplied = true;
                predictionWarnings.push_back(
                    "Periodic local prediction assumes rotationally periodic geometry, trajectory, occlusion, and thermal history.");
            } else if(task.options.periodicLocal.fallbackToFullPrediction) {
                predictionWarnings.push_back(
                    std::string("Periodic local prediction fell back to the full model: ")
                    + periodicReduction.failureReason);
            } else {
                throw std::runtime_error(periodicReduction.failureReason);
            }
        }
        reportProgress(
            execution,
            0.095,
            (std::string("Prepared ") + std::to_string(predictionVertexIndices.size())
                + "/" + std::to_string(task.workpiece.samples.size())
                + " prediction vertices and " + std::to_string(spraySamples.size())
                + "/" + std::to_string(allSpraySamples.size())
                + " active trajectory intervals using "
                + (task.options.base.trajectorySamplingMode == TrajectorySamplingMode::OriginalPoints
                    ? "original points"
                    : "time-step resampling")).c_str());

        if(canceled(execution)) {
            return {};
        }
        reportProgress(execution, 0.10, "Building thickness BVH");
        const ThicknessBvhProgress bvhProgress = [&execution](
            const char* phase,
            std::size_t current,
            std::size_t total) {
                const double fraction = total > 0
                    ? static_cast<double>(current) / static_cast<double>(total)
                    : 1.0;
                const double progress = 0.10 + 0.05 * fraction;
                const std::string message = std::string("BVH ") + phase + " ("
                    + std::to_string(current) + "/" + std::to_string(total) + ")";
                reportProgress(execution, progress, message.c_str());
            };
        const auto bvhStart = std::chrono::steady_clock::now();
        const std::shared_ptr<const ThicknessBvh> bvh =
            task.options.enableBvhOcclusion
            ? cachedBvh(task.workpiece, bvhProgress)
            : std::make_shared<ThicknessBvh>();
        const double bvhMilliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - bvhStart).count();
        reportProgress(execution, 0.15, "Thickness BVH ready");
        if(task.options.enableBvhOcclusion && bvh->empty()) {
            throw std::runtime_error("Failed to build the workpiece BVH.");
        }

        OpenGLComputeProgram& program = m_impl->resources.program(
            false,
            task.options.enableBvhOcclusion,
            task.options.enableHistoryCorrection);
        program.use();
        configureProgram(
            program,
            task,
            static_cast<int>(task.workpiece.samples.size()),
            static_cast<int>(predictionVertexIndices.size()),
            periodicReductionApplied);
        checkOpenGlErrors("compute program setup");

        const std::size_t vertexCount = task.workpiece.samples.size();
        const std::size_t predictionVertexCount = predictionVertexIndices.size();
        std::vector<float> thickness(vertexCount, 0.0f);
        std::vector<float> historyTau(vertexCount, 0.0f);
        std::vector<float> lastUpdateTime(
            vertexCount,
            static_cast<float>(allSpraySamples.front().time[0]));
        std::vector<float> historyFactor(vertexCount, 1.0f);
        const std::uint32_t dummyIndex = 0;
        const LegacyGpuBvh legacyBvh = makeLegacyGpuBvh(*bvh);
        const LegacyGpuBvhNode dummyNode{};

        GpuBuffer& surfaceBuffer = m_impl->resources.buffer(0);
        GpuBuffer& thicknessBuffer = m_impl->resources.buffer(1);
        GpuBuffer& sprayBuffer = m_impl->resources.buffer(2);
        GpuBuffer& indexBuffer = m_impl->resources.buffer(3);
        GpuBuffer& bvhBuffer = m_impl->resources.buffer(4);
        GpuBuffer& orderBuffer = m_impl->resources.buffer(5);
        GpuBuffer& historyBuffer = m_impl->resources.buffer(6);
        GpuBuffer& lastTimeBuffer = m_impl->resources.buffer(7);
        GpuBuffer& factorBuffer = m_impl->resources.buffer(8);
        GpuBuffer& predictionVertexBuffer = m_impl->resources.buffer(9);
        GpuBuffer& surfacePositionBuffer = m_impl->resources.buffer(10);
        const auto uploadStart = std::chrono::steady_clock::now();
        surfaceBuffer.upload(0, surfaceData.data(), surfaceData.size() * sizeof(surfaceData.front()), GL_STATIC_DRAW);
        thicknessBuffer.upload(1, thickness.data(), thickness.size() * sizeof(float), GL_DYNAMIC_COPY);
        indexBuffer.upload(3, task.workpiece.triangleIndices.data(),
            task.workpiece.triangleIndices.size() * sizeof(std::uint32_t), GL_STATIC_DRAW);
        bvhBuffer.upload(4, legacyBvh.nodes.empty() ? &dummyNode : legacyBvh.nodes.data(),
            legacyBvh.nodes.empty() ? sizeof(dummyNode) : legacyBvh.nodes.size() * sizeof(LegacyGpuBvhNode), GL_STATIC_DRAW);
        orderBuffer.upload(5, legacyBvh.leafTriangleIndices.empty()
                ? &dummyIndex : legacyBvh.leafTriangleIndices.data(),
            legacyBvh.leafTriangleIndices.empty()
                ? sizeof(dummyIndex)
                : legacyBvh.leafTriangleIndices.size() * sizeof(std::uint32_t),
            GL_STATIC_DRAW);
        historyBuffer.upload(6, historyTau.data(), historyTau.size() * sizeof(float), GL_DYNAMIC_COPY);
        lastTimeBuffer.upload(7, lastUpdateTime.data(), lastUpdateTime.size() * sizeof(float), GL_DYNAMIC_COPY);
        factorBuffer.upload(8, historyFactor.data(), historyFactor.size() * sizeof(float), GL_DYNAMIC_COPY);
        predictionVertexBuffer.upload(9,
            predictionVertexIndices.data(),
            predictionVertexIndices.size() * sizeof(std::uint32_t),
            GL_STATIC_DRAW);
        surfacePositionBuffer.upload(
            10,
            surfacePositionData.data(),
            surfacePositionData.size() * sizeof(surfacePositionData.front()),
            GL_STATIC_DRAW);
        checkOpenGlErrors("input buffer upload");
        const double uploadMilliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - uploadStart).count();

        const int maximumBlockSize = capabilities.maximumBlockSize;
        const std::size_t maximumSamples = maximumBlockSize > 0
            ? static_cast<std::size_t>(maximumBlockSize) / sizeof(SprayGpuSample)
            : spraySamples.size();
        const std::size_t initialBatch = computeAdaptiveBvhBatchSize(
            predictionVertexCount,
            spraySamples.size(),
            sizeof(SprayGpuSample),
            maximumBlockSize);
        const std::size_t requestedBatch = task.options.trajectoryBatchSize;
        const std::size_t maximumBatch = std::max<std::size_t>(1, std::min({
            requestedBatch == 0 ? spraySamples.size() : requestedBatch,
            maximumSamples,
            spraySamples.size()}));
        const std::uint64_t batchProfileKey = makeBatchProfileKey(
            capabilities,
            false,
            task.options.enableBvhOcclusion,
            task.options.enableHistoryCorrection,
            predictionVertexCount,
            1.0);
        AdaptiveBatchSelector batchSelector(
            batchProfileKey,
            maximumBatch,
            initialBatch);
        std::vector<SprayGpuSample> batchStorage(maximumBatch);
        sprayBuffer.upload(2, batchStorage.data(), batchStorage.size() * sizeof(SprayGpuSample), GL_DYNAMIC_DRAW);

        const int sprayPointCountLocation = program.uniformLocation("sprayPointCount");
        const auto dispatchStart = std::chrono::steady_clock::now();
        GpuElapsedQuery gpuElapsedQuery;
        gpuElapsedQuery.begin();
        reportProgress(
            execution,
            0.20,
            (std::string("Starting adaptive GPU thickness batches; initial=")
                + std::to_string(batchSelector.next(spraySamples.size()))
                + ", maximum=" + std::to_string(maximumBatch)
                + (batchSelector.wasCached() ? ", profile cache hit" : ", calibrating")
                + ", renderer=" + capabilities.renderer).c_str());
        std::size_t begin = 0;
        std::size_t batch = 0;
        while(begin < spraySamples.size()) {
            if(canceled(execution)) {
                ThicknessPredictionResult result;
                result.warnings.push_back("Thickness prediction was canceled.");
                return result;
            }
            const std::size_t count = batchSelector.next(
                spraySamples.size() - begin);
            const std::size_t completedSprayPoints = begin + count;
            const std::uint64_t completedDistributionPoints =
                static_cast<std::uint64_t>(completedSprayPoints)
                * static_cast<std::uint64_t>(predictionVertexCount);
            const std::string batchStartMessage =
                std::string("GPU batch ") + std::to_string(batch + 1) + ": "
                + std::to_string(count) + " spray points x "
                + std::to_string(predictionVertexCount) + " vertices";
            const bool reportBatchProgress = batch < 3
                || completedSprayPoints == spraySamples.size()
                || completedSprayPoints * 100 / spraySamples.size()
                    != begin * 100 / spraySamples.size();
            if(reportBatchProgress) {
                reportProgress(
                    execution,
                    0.20 + 0.75 * static_cast<double>(begin)
                        / static_cast<double>(spraySamples.size()),
                    batchStartMessage.c_str());
            }
            const bool measuringBatch = batchSelector.measuring();
            const auto batchStart = std::chrono::steady_clock::now();
            sprayBuffer.update(spraySamples.data() + begin, count * sizeof(SprayGpuSample));
            glUniform1i(sprayPointCountLocation, static_cast<int>(count));
            checkOpenGlErrors("batch input upload");
            glDispatchCompute(
                static_cast<unsigned int>((predictionVertexCount + 255) / 256),
                1,
                1);
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
            // Keep the legacy predictor's command-queue pacing. Most batches
            // are flushed asynchronously; every eighth batch establishes a
            // completed GPU boundary before more work is queued.
            if(measuringBatch || (batch + 1) % 8 == 0
                || completedSprayPoints == spraySamples.size()) {
                glFinish();
            } else {
                glFlush();
            }
            checkOpenGlErrors("batch dispatch");
            if(measuringBatch) {
                const double batchMilliseconds =
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - batchStart).count();
                batchSelector.record(
                    count,
                    static_cast<std::uint64_t>(count)
                        * static_cast<std::uint64_t>(predictionVertexCount),
                    batchMilliseconds);
            }
            const std::string batchDoneMessage =
                std::string("GPU batch ") + std::to_string(batch + 1)
                + " completed; distributed "
                + std::to_string(completedSprayPoints) + "/"
                + std::to_string(spraySamples.size()) + " spray points, "
                + std::to_string(completedDistributionPoints)
                + " vertex-distribution pairs";
            if(reportBatchProgress) {
                reportProgress(
                    execution,
                    0.20 + 0.75 * static_cast<double>(completedSprayPoints)
                        / static_cast<double>(spraySamples.size()),
                    batchDoneMessage.c_str());
            }
            begin = completedSprayPoints;
            ++batch;
        }
        batchSelector.finalize();

        const double gpuMilliseconds = gpuElapsedQuery.endAndReadMilliseconds();

        const double dispatchMilliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - dispatchStart).count();

        const auto downloadStart = std::chrono::steady_clock::now();
        thicknessBuffer.download(thickness.data(), thickness.size() * sizeof(float));
        checkOpenGlErrors("result readback");
        if(periodicReductionApplied) {
            std::vector<float> expandedThickness;
            expandPeriodicSectorThickness(
                periodicReduction, thickness, expandedThickness);
            thickness = std::move(expandedThickness);
        }
        ThicknessPredictionResult result;
        result.field.resizeFromWorkpiece(task.workpiece);
        for(std::size_t i = 0; i < thickness.size(); ++i) {
            result.field.results[i].thickness = static_cast<double>(thickness[i]) * kMillimetersToMeters;
        }
        result.field.updateErrors();
        result.metrics = ThicknessMetricsCalculator::calculate(result.field, task.options.base);
        result.warnings = std::move(predictionWarnings);
        const double downloadMilliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - downloadStart).count();
        const double totalMilliseconds = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - totalStart).count();
        result.timing.valid = true;
        result.timing.periodicMappingMilliseconds = mappingMilliseconds;
        result.timing.bvhMilliseconds = bvhMilliseconds;
        result.timing.uploadMilliseconds = uploadMilliseconds;
        result.timing.dispatchMilliseconds = dispatchMilliseconds;
        result.timing.pureGpuMilliseconds = gpuMilliseconds;
        result.timing.readbackMilliseconds = downloadMilliseconds;
        result.timing.backendTotalMilliseconds = totalMilliseconds;
        result.timing.predictionVertexCount = predictionVertexCount;
        result.timing.sprayPointCount = spraySamples.size();
        result.timing.adaptiveBatchSize = batchSelector.selectedBatch();
        reportProgress(
            execution,
            1.0,
            (std::string("GPU thickness prediction completed; BVH ")
                + std::to_string(bvhMilliseconds) + " ms, dispatch "
                + std::to_string(dispatchMilliseconds) + " ms, readback/finalize "
                + std::to_string(downloadMilliseconds) + " ms, upload "
                + std::to_string(uploadMilliseconds) + " ms, pure GPU "
                + std::to_string(gpuMilliseconds) + " ms, total "
                + std::to_string(totalMilliseconds) + " ms, adaptive batch "
                + std::to_string(batchSelector.selectedBatch())).c_str());
        return result;
    }
}
