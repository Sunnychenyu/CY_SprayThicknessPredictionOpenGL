#pragma once

#include <SprayThicknessPrediction/MeshSamplingAssessment.h>
#include <SprayThicknessPrediction/ThicknessPrediction.h>

namespace spraythickness::opengl
{
    // CPU reference and geometry-free aggregation of GPU footprint records.
    class MeshSamplingEvaluator
    {
    public:
        static MeshSamplingAssessment evaluate(
            const ThicknessPredictionTask& task,
            const ThicknessPredictionExecution& execution = {});
        static MeshSamplingAssessment summarize(
            const ThicknessPredictionTask& task,
            std::vector<MeshSamplingPointAssessment> points);
    };
}
