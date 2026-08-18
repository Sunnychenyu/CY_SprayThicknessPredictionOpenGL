#pragma once

#include <WorkpieceCore/WorkpieceModel.h>

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace spraythickness::opengl
{
    struct alignas(16) GpuBvhNode
    {
        std::array<float, 4> minimum{};
        std::array<float, 4> maximum{};
        std::int32_t leftChild{ -1 };
        std::int32_t rightChild{ -1 };
        std::int32_t firstTriangle{ 0 };
        std::int32_t triangleCount{ 0 };
    };

    struct ThicknessBvh
    {
        std::vector<GpuBvhNode> nodes;
        std::vector<std::uint32_t> triangleOrder;

        bool empty() const { return nodes.empty(); }
    };

    using ThicknessBvhProgress = std::function<void(
        const char* phase,
        std::size_t current,
        std::size_t total)>;

    class ThicknessBvhBuilder
    {
    public:
        static ThicknessBvh build(
            const sprayworkpiece::WorkpieceModel& workpiece,
            std::size_t maximumLeafTriangles = 8,
            const ThicknessBvhProgress& progress = {});
    };
}
