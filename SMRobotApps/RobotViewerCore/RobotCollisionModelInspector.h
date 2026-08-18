#pragma once

#include "ProjectRuntimeTypes.h"

#include <cstddef>
#include <string>
#include <vector>

struct RobotCollisionStat
{
    std::string name;
    std::size_t count = 0;
};

struct RobotCollisionModelVariantSummary
{
    std::string variantId;
    std::string label;
    std::string linkName;
    std::string role;
    std::string source;
    bool enabled = true;
    bool replaceOriginal = false;
    bool visibleInViewport = false;
    bool selectedInViewport = false;
    bool usedByActiveDetector = false;
    std::size_t elementCount = 0;
    std::size_t meshElementCount = 0;
    std::size_t meshVertexCount = 0;
    std::size_t meshTriangleCount = 0;
    std::size_t unresolvedMeshCount = 0;
    std::size_t primitiveCount = 0;
    std::vector<std::string> elementNames;
    std::vector<RobotCollisionStat> geometryTypes;
};

struct RobotCollisionLinkSummary
{
    std::string linkName;
    bool hasVisual = false;
    bool hasOriginalCollision = false;
    bool hasOverrideCollision = false;
    bool hasMeshCollision = false;
    bool hasSphereCover = false;
    std::size_t visualCount = 0;
    std::size_t originalCollisionCount = 0;
    std::size_t overrideCollisionCount = 0;
    std::size_t effectiveCollisionCount = 0;
    std::vector<RobotCollisionStat> geometryTypes;
    std::vector<RobotCollisionStat> roles;
    std::vector<RobotCollisionStat> sources;
    std::vector<RobotCollisionModelVariantSummary> variants;
};

struct RobotCollisionRobotSummary
{
    std::string robotId;
    std::string robotName;
    bool hasVisualOnlyLinks = false;
    bool hasCollisionReadyLinks = false;
    bool hasOverrideAppliedLinks = false;
    bool hasMeshCollision = false;
    bool hasSphereCover = false;
    std::size_t linkCount = 0;
    std::size_t visualLinkCount = 0;
    std::size_t visualOnlyLinkCount = 0;
    std::size_t collisionReadyLinkCount = 0;
    std::size_t overrideAppliedLinkCount = 0;
    std::size_t visualCount = 0;
    std::size_t originalCollisionCount = 0;
    std::size_t overrideCollisionCount = 0;
    std::size_t effectiveCollisionCount = 0;
    std::vector<RobotCollisionStat> geometryTypes;
    std::vector<RobotCollisionStat> roles;
    std::vector<RobotCollisionStat> sources;
    std::vector<RobotCollisionLinkSummary> links;
};

class RobotCollisionModelInspector
{
public:
    static RobotCollisionRobotSummary summarizeRobot(
        const RuntimeRobot& robot,
        const std::string& activeDetectorRole = std::string(),
        const std::string& activeDetectorSource = std::string(),
        const std::string& visibleVariantId = std::string());

    static RobotCollisionLinkSummary summarizeLink(
        const RuntimeRobot& robot,
        const std::string& linkName,
        const std::string& activeDetectorRole = std::string(),
        const std::string& activeDetectorSource = std::string(),
        const std::string& visibleVariantId = std::string());
};
