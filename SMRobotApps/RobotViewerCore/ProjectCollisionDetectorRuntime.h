#pragma once

#include "ProjectRuntimeTypes.h"

#include <Collision/CollisionDebugDrawDesc.h>
#include <Collision/CollisionQueryOptions.h>
#include <Collision/CollisionResult.h>
#include <SimulationProject/ProjectDocument.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

struct ProjectCollisionDetectorRuntime
{
    std::string id;
    std::string name;
    std::string type;
    bool enabled = true;
    bool visible = true;
    collision::CollisionQueryOptions options;
    collision::CollisionVisualizationOptions visualization;
    collision::CollisionResult lastResult;
    bool hasResult = false;
    std::size_t effectiveIncludePairCount = 0;
    double lastCheckMs = 0.0;
    double lastDistanceMs = 0.0;
    double lastQueryMs = 0.0;
    std::uint64_t lastNearestQueryFrame = 0;
    std::string nearestState = "NotComputed";
    std::string nearestReason;
};

class ProjectCollisionDetectorBuilder
{
public:
    static std::vector<ProjectCollisionDetectorRuntime> build(
        const simulation_project::ProjectDocument& document,
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects);
};
