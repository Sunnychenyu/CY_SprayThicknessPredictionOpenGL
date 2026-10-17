#pragma once

#include <SprayThicknessPrediction/MeshSamplingAssessment.h>
#include <SprayThicknessPrediction/ThicknessPrediction.h>

namespace spraythickness::opengl
{
    // CPU-only, cancellable post-prediction diagnostic. Never changes thickness.
    class MeshSamplingEvaluator
    {
    public:
        static MeshSamplingAssessment evaluate(
            const ThicknessPredictionTask& task,
            const ThicknessPredictionExecution& execution = {});
    };
}
