#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace robot_qt_viewer
{
    struct CollisionRuntimeStat
    {
        std::string name;
        std::size_t count = 0;
    };

    struct CollisionRuntimeModelVariantSummary
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
        std::vector<CollisionRuntimeStat> geometryTypes;
    };

    struct CollisionRuntimeLinkSummary
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
        std::vector<CollisionRuntimeStat> geometryTypes;
        std::vector<CollisionRuntimeStat> roles;
        std::vector<CollisionRuntimeStat> sources;
        std::vector<CollisionRuntimeModelVariantSummary> variants;
    };

    struct CollisionRuntimeRobotSummary
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
        std::vector<CollisionRuntimeStat> geometryTypes;
        std::vector<CollisionRuntimeStat> roles;
        std::vector<CollisionRuntimeStat> sources;
        std::vector<CollisionRuntimeLinkSummary> links;
    };

    struct CollisionRuntimeProxyRequest
    {
        std::string proxyType;
        std::string role = "PlanningProxy";
        double inflationMargin = 0.0;
        int maxSphereCount = 16;
        bool useVisualWhenCollisionMissing = true;
        bool useExistingCollisionAsInput = false;
    };

    struct CollisionRuntimeProxyQualitySummary
    {
        int requestedMaxSpheres = 0;
        int generatedSphereCount = 0;
        std::size_t inputPointCount = 0;
        std::size_t uncoveredPointCount = 0;
        double maxOutsideDistance = 0.0;
        double mainAxisLength = 0.0;
        double estimatedCrossSectionRadius = 0.0;
        double maxSphereRadius = 0.0;
        double maxRadiusToLinkLength = 0.0;
        double maxRadiusToCrossSectionRadius = 0.0;
        int oversizedSphereCount = 0;
        std::string recommendedShape;
        bool hasWarning = false;
        std::string warning;
    };

    struct CollisionRuntimeDetectorInfo
    {
        struct Vec3Info
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct ContactInfo
        {
            std::string bodyA;
            std::string bodyB;
            Vec3Info position;
            Vec3Info normal;
            double penetrationDepth = 0.0;
        };

        struct NearestInfo
        {
            bool valid = false;
            std::string state = "NotComputed";
            std::string reason;
            std::string bodyA;
            std::string bodyB;
            Vec3Info pointA;
            Vec3Info pointB;
            double distance = 0.0;
        };

        std::string id;
        std::string name;
        std::string type;
        bool enabled = true;
        bool visible = true;
        bool active = false;
        std::size_t includePairCount = 0;
        std::size_t effectiveIncludePairCount = 0;
        std::size_t contactCount = 0;
        bool inCollision = false;
        double minDistance = 0.0;
        double lastCheckMs = 0.0;
        double lastDistanceMs = 0.0;
        double lastQueryMs = 0.0;
        double frameRobotPoseMs = 0.0;
        double frameCollisionWorldUpdateMs = 0.0;
        double frameOverlayMs = 0.0;
        double frameOverlayHighlightMs = 0.0;
        double frameOverlayDebugBuildMs = 0.0;
        double frameOverlayVariantFilterMs = 0.0;
        double frameOverlayDebugSubmitMs = 0.0;
        double frameOverlayAuxFramesMs = 0.0;
        std::size_t frameOverlayDetectorCount = 0;
        std::size_t frameOverlayGeometryCount = 0;
        std::size_t frameOverlayContactCount = 0;
        std::size_t frameOverlayNearestCount = 0;
        std::size_t frameOverlayPrimitiveEstimate = 0;
        std::size_t frameOverlayLineEstimate = 0;
        std::string firstPairA;
        std::string firstPairB;
        bool hasResult = false;
        std::vector<ContactInfo> contacts;
        NearestInfo nearest;
    };
}
