#include <SprayThicknessPredictionOpenGL/ThicknessBvh.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <limits>
#include <numeric>
#include <queue>
#include <cmath>

namespace spraythickness::opengl
{
    double ThicknessBvh::surfaceDistance(const sprayworkpiece::WorkpieceModel& workpiece,
        const Eigen::Vector3d& position) const
    {
        if(empty()) return std::numeric_limits<double>::infinity();
        const auto triangleDistanceSquared = [&](const Eigen::Vector3d& a,
            const Eigen::Vector3d& b, const Eigen::Vector3d& c) {
            const Eigen::Vector3d ab = b - a, ac = c - a, ap = position - a;
            const Eigen::Vector3d normal = ab.cross(ac);
            const double areaSquared = normal.squaredNorm();
            if(areaSquared > 1.0e-30) {
                const double u = ap.cross(ac).dot(normal) / areaSquared;
                const double v = ab.cross(ap).dot(normal) / areaSquared;
                if(u >= 0.0 && v >= 0.0 && u + v <= 1.0)
                    return std::pow(ap.dot(normal), 2) / areaSquared;
            }
            const auto edgeDistance = [&](const Eigen::Vector3d& from, const Eigen::Vector3d& to) {
                const Eigen::Vector3d edge = to - from;
                const double length = edge.squaredNorm();
                const double t = length > 0.0
                    ? std::clamp((position - from).dot(edge) / length, 0.0, 1.0) : 0.0;
                return (position - from - t * edge).squaredNorm();
            };
            return std::min({ edgeDistance(a, b), edgeDistance(b, c), edgeDistance(c, a) });
        };
        const auto boundDistance = [&](std::int32_t index) {
            const auto& node = nodes[index];
            double distance = 0.0;
            for(int axis = 0; axis < 3; ++axis) {
                const double delta = std::max({ 0.0,
                    node.minimum[axis] * 0.001 - position[axis],
                    position[axis] - node.maximum[axis] * 0.001 });
                distance += delta * delta;
            }
            return distance;
        };
        using Entry = std::pair<double, std::int32_t>;
        std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> pending;
        pending.emplace(boundDistance(0), 0);
        double best = std::numeric_limits<double>::infinity();
        while(!pending.empty()) {
            const auto entry = pending.top();
            pending.pop();
            if(entry.first >= best) break;
            const auto& node = nodes[entry.second];
            if(node.triangleCount > 0) {
                for(int i = 0; i < node.triangleCount; ++i) {
                    const auto triangle = triangleOrder[node.firstTriangle + i] * 3;
                    const auto& indices = workpiece.triangleIndices;
                    best = std::min(best, triangleDistanceSquared(
                        workpiece.samples[indices[triangle]].position,
                        workpiece.samples[indices[triangle + 1]].position,
                        workpiece.samples[indices[triangle + 2]].position));
                }
            } else {
                for(const auto child : { node.leftChild, node.rightChild }) {
                    if(child < 0) continue;
                    const double distance = boundDistance(child);
                    if(distance < best) pending.emplace(distance, child);
                }
            }
        }
        return std::sqrt(best);
    }

    namespace
    {
        struct TriangleBounds
        {
            Eigen::Vector3f minimum;
            Eigen::Vector3f maximum;
            Eigen::Vector3f centroid;
        };

        std::array<float, 4> asGpuPoint(const Eigen::Vector3f& value)
        {
            return { value.x(), value.y(), value.z(), 0.0f };
        }

        class Builder
        {
        public:
            Builder(
                const sprayworkpiece::WorkpieceModel& workpiece,
                std::size_t maximumLeafTriangles,
                const ThicknessBvhProgress& progress)
                : m_workpiece(workpiece)
                , m_maximumLeafTriangles(std::max<std::size_t>(1, maximumLeafTriangles))
                , m_progress(progress)
            {
                const std::size_t triangleCount = workpiece.triangleIndices.size() / 3;
                m_triangleCount = triangleCount;
                m_bounds.reserve(triangleCount);
                for(std::size_t triangle = 0; triangle < triangleCount; ++triangle) {
                    m_bounds.push_back(calculateTriangleBounds(triangle));
                    reportProgress("triangle bounds", triangle + 1, triangleCount);
                }
                m_triangles.resize(triangleCount);
                std::iota(m_triangles.begin(), m_triangles.end(), 0U);
            }

            ThicknessBvh build()
            {
                if(m_triangles.empty()) {
                    return {};
                }
                buildNode(0, m_triangles.size());
                m_result.triangleOrder = std::move(m_triangles);
                return std::move(m_result);
            }

        private:
            TriangleBounds calculateTriangleBounds(std::size_t triangle) const
            {
                const auto& indices = m_workpiece.triangleIndices;
                const Eigen::Vector3f a = samplePosition(indices[triangle * 3]);
                const Eigen::Vector3f b = samplePosition(indices[triangle * 3 + 1]);
                const Eigen::Vector3f c = samplePosition(indices[triangle * 3 + 2]);
                constexpr float epsilon = 1.0e-6f;
                return {
                    a.cwiseMin(b).cwiseMin(c).array() - epsilon,
                    a.cwiseMax(b).cwiseMax(c).array() + epsilon,
                    (a + b + c) / 3.0f};
            }

            Eigen::Vector3f samplePosition(std::uint32_t index) const
            {
                return m_workpiece.samples[index].position.cast<float>() * 1000.0f;
            }

            std::int32_t buildNode(std::size_t begin, std::size_t end)
            {
                const std::int32_t nodeIndex = static_cast<std::int32_t>(m_result.nodes.size());
                m_result.nodes.emplace_back();
                ++m_nodesBuilt;
                reportProgress(
                    "BVH nodes",
                    m_nodesBuilt,
                    std::max<std::size_t>(1, m_triangleCount * 2 - 1));

                Eigen::Vector3f minimum = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
                Eigen::Vector3f maximum = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
                Eigen::Vector3f centroidMinimum = minimum;
                Eigen::Vector3f centroidMaximum = maximum;
                for(std::size_t i = begin; i < end; ++i) {
                    const TriangleBounds& bounds = m_bounds[m_triangles[i]];
                    minimum = minimum.cwiseMin(bounds.minimum);
                    maximum = maximum.cwiseMax(bounds.maximum);
                    centroidMinimum = centroidMinimum.cwiseMin(bounds.centroid);
                    centroidMaximum = centroidMaximum.cwiseMax(bounds.centroid);
                }

                GpuBvhNode node;
                node.minimum = asGpuPoint(minimum);
                node.maximum = asGpuPoint(maximum);
                const std::size_t count = end - begin;
                if(count <= m_maximumLeafTriangles) {
                    node.firstTriangle = static_cast<std::int32_t>(begin);
                    node.triangleCount = static_cast<std::int32_t>(count);
                    m_result.nodes[nodeIndex] = node;
                    return nodeIndex;
                }

                std::size_t split = 0;
                Eigen::Index splitAxis = 0;
                int splitBin = -1;
                if(!findSahSplit(
                    begin,
                    end,
                    centroidMinimum,
                    centroidMaximum,
                    splitAxis,
                    splitBin,
                    split)) {
                    Eigen::Index fallbackAxis = 0;
                    (centroidMaximum - centroidMinimum).maxCoeff(&fallbackAxis);
                    splitAxis = fallbackAxis;
                    split = begin + count / 2;
                }

                std::sort(
                    m_triangles.begin() + begin,
                    m_triangles.begin() + end,
                    [&](std::uint32_t lhs, std::uint32_t rhs) {
                        return m_bounds[lhs].centroid[splitAxis]
                            < m_bounds[rhs].centroid[splitAxis];
                    });

                if(splitBin >= 0) {
                    const float minimum = centroidMinimum[splitAxis];
                    const float extent = centroidMaximum[splitAxis] - minimum;
                    const std::size_t binCount = kSahBinCount;
                    const auto binOf = [&](std::uint32_t triangle) {
                        const float normalized = extent > 1.0e-12f
                            ? (m_bounds[triangle].centroid[splitAxis] - minimum) / extent
                            : 0.0f;
                        return std::min<std::size_t>(
                            binCount - 1,
                            static_cast<std::size_t>(std::max(
                                0.0f,
                                normalized * static_cast<float>(binCount))));
                    };
                    const auto first = m_triangles.begin() + begin;
                    const auto partition = std::find_if(
                        first,
                        m_triangles.begin() + end,
                        [&](std::uint32_t triangle) {
                            return static_cast<int>(binOf(triangle)) > splitBin;
                        });
                    split = static_cast<std::size_t>(partition - m_triangles.begin());
                    if(split <= begin || split >= end) {
                        split = begin + count / 2;
                    }
                }

                if(split <= begin || split >= end) {
                    split = begin + count / 2;
                }
                node.leftChild = buildNode(begin, split);
                node.rightChild = buildNode(split, end);
                m_result.nodes[nodeIndex] = node;
                return nodeIndex;
            }

            struct SahBin
            {
                Eigen::Vector3f minimum = Eigen::Vector3f::Constant(
                    std::numeric_limits<float>::max());
                Eigen::Vector3f maximum = Eigen::Vector3f::Constant(
                    std::numeric_limits<float>::lowest());
                std::size_t count = 0;
            };

            static constexpr std::size_t kSahBinCount = 16;

            static float surfaceArea(
                const Eigen::Vector3f& minimum,
                const Eigen::Vector3f& maximum)
            {
                const Eigen::Vector3f extent = (maximum - minimum).cwiseMax(0.0f);
                return 2.0f * (
                    extent.x() * extent.y()
                    + extent.x() * extent.z()
                    + extent.y() * extent.z());
            }

            bool findSahSplit(
                std::size_t begin,
                std::size_t end,
                const Eigen::Vector3f& centroidMinimum,
                const Eigen::Vector3f& centroidMaximum,
                Eigen::Index& bestAxis,
                int& bestBin,
                std::size_t& bestSplit) const
            {
                const std::size_t count = end - begin;
                float bestCost = std::numeric_limits<float>::max();
                bool found = false;
                for(Eigen::Index axis = 0; axis < 3; ++axis) {
                    const float minimum = centroidMinimum[axis];
                    const float extent = centroidMaximum[axis] - minimum;
                    if(extent <= 1.0e-12f) {
                        continue;
                    }

                    std::array<SahBin, kSahBinCount> bins;
                    for(std::size_t index = begin; index < end; ++index) {
                        const std::uint32_t triangle = m_triangles[index];
                        const float normalized =
                            (m_bounds[triangle].centroid[axis] - minimum) / extent;
                        const std::size_t bin = std::min<std::size_t>(
                            kSahBinCount - 1,
                            static_cast<std::size_t>(std::max(
                                0.0f,
                                normalized * static_cast<float>(kSahBinCount))));
                        SahBin& target = bins[bin];
                        target.minimum = target.minimum.cwiseMin(m_bounds[triangle].minimum);
                        target.maximum = target.maximum.cwiseMax(m_bounds[triangle].maximum);
                        ++target.count;
                    }

                    std::array<SahBin, kSahBinCount> prefix = bins;
                    std::array<SahBin, kSahBinCount> suffix = bins;
                    for(std::size_t bin = 1; bin < kSahBinCount; ++bin) {
                        prefix[bin].minimum = prefix[bin].minimum.cwiseMin(prefix[bin - 1].minimum);
                        prefix[bin].maximum = prefix[bin].maximum.cwiseMax(prefix[bin - 1].maximum);
                        prefix[bin].count += prefix[bin - 1].count;
                    }
                    for(std::size_t bin = kSahBinCount - 1; bin > 0; --bin) {
                        suffix[bin - 1].minimum = suffix[bin - 1].minimum.cwiseMin(suffix[bin].minimum);
                        suffix[bin - 1].maximum = suffix[bin - 1].maximum.cwiseMax(suffix[bin].maximum);
                        suffix[bin - 1].count += suffix[bin].count;
                    }

                    for(std::size_t bin = 0; bin + 1 < kSahBinCount; ++bin) {
                        if(prefix[bin].count == 0 || suffix[bin + 1].count == 0) {
                            continue;
                        }
                        const float cost = surfaceArea(prefix[bin].minimum, prefix[bin].maximum)
                            * static_cast<float>(prefix[bin].count)
                            + surfaceArea(suffix[bin + 1].minimum, suffix[bin + 1].maximum)
                            * static_cast<float>(suffix[bin + 1].count);
                        if(cost < bestCost) {
                            bestCost = cost;
                            bestAxis = axis;
                            bestBin = static_cast<int>(bin);
                            bestSplit = begin + prefix[bin].count;
                            found = true;
                        }
                    }
                }
                return found && bestSplit > begin && bestSplit < end;
            }

            const sprayworkpiece::WorkpieceModel& m_workpiece;
            std::size_t m_maximumLeafTriangles;
            ThicknessBvhProgress m_progress;
            std::size_t m_triangleCount{ 0 };
            std::size_t m_nodesBuilt{ 0 };
            std::size_t m_lastReportedBounds{ 0 };
            std::size_t m_lastReportedNodes{ 0 };
            std::vector<TriangleBounds> m_bounds;
            std::vector<std::uint32_t> m_triangles;
            ThicknessBvh m_result;

            void reportProgress(
                const char* phase,
                std::size_t current,
                std::size_t total)
            {
                if(!m_progress || total == 0) {
                    return;
                }
                const std::size_t interval = std::max<std::size_t>(1, total / 100);
                std::size_t& lastReported =
                    phase[0] == 't' ? m_lastReportedBounds : m_lastReportedNodes;
                if(current != total && current < lastReported + interval) {
                    return;
                }
                lastReported = current;
                m_progress(phase, current, total);
            }
        };
    }

    ThicknessBvh ThicknessBvhBuilder::build(
        const sprayworkpiece::WorkpieceModel& workpiece,
        std::size_t maximumLeafTriangles,
        const ThicknessBvhProgress& progress)
    {
        if(workpiece.triangleIndices.size() % 3 != 0) {
            return {};
        }
        for(const std::uint32_t index : workpiece.triangleIndices) {
            if(index >= workpiece.samples.size()) {
                return {};
            }
        }
        return Builder(workpiece, maximumLeafTriangles, progress).build();
    }
}
