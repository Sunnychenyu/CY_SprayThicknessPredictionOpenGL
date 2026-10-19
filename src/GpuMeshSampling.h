#pragma once

#include <SprayThicknessPrediction/ThicknessPrediction.h>
#include <SprayThicknessPrediction/MeshSamplingAssessment.h>
#include <SprayThicknessPredictionOpenGL/PeriodicSectorReduction.h>

#include <cstdint>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace spraythickness::opengl
{
    class OpenGLComputeProgram;

    class GpuMeshSampling
    {
    public:
        GpuMeshSampling();
        ~GpuMeshSampling();
        GpuMeshSampling(const GpuMeshSampling&) = delete;
        GpuMeshSampling& operator=(const GpuMeshSampling&) = delete;

        bool prepare(const ThicknessPredictionTask& task, std::size_t sprayCount,
            const ThicknessPredictionExecution& execution);
        bool active() const { return m_active; }
        void disable(const std::string& reason);
        void configure(const OpenGLComputeProgram& program, std::size_t sprayBase = 0) const;
        // Requires the predictor's geometry/BVH and complete spray SSBOs bound.
        // Called after ending the predictor's GL_TIME_ELAPSED query.
        std::shared_ptr<const MeshSamplingAssessment> finish(
            const ThicknessPredictionTask& task,
            const std::vector<PeriodicSpraySample>& sprays,
            const ThicknessPredictionExecution& execution);

    private:
        unsigned int m_buffer{ 0 };
        std::size_t m_bufferBytes{ 0 };
        std::uint64_t m_uploadedMappingKey{ 0 };
        bool m_uploadedMappingValid{ false };
        std::unique_ptr<OpenGLComputeProgram> m_probeProgram;
        std::uint64_t m_mappingKey{ 0 };
        bool m_mappingValid{ false };
        bool m_active{ false };
        std::vector<std::int32_t> m_predecessors;
        float m_momentScale{ 1.0f };
        MeshSamplingAssessment m_metadata;
    };
}
