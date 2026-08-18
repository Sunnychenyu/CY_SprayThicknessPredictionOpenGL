#include "RobotCollisionModelInspector.h"

#include <AssetCore/AssetManager.h>
#include <SimulationProject/CollisionModelSelectionIds.h>

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <system_error>
#include <utility>

namespace
{
    const char* geometryTypeName(robot::RobotGeometryType type)
    {
        switch(type) {
        case robot::RobotGeometryType::Box:
            return "Box";
        case robot::RobotGeometryType::Sphere:
            return "Sphere";
        case robot::RobotGeometryType::Cylinder:
            return "Cylinder";
        case robot::RobotGeometryType::Mesh:
            return "Mesh";
        default:
            return "Unknown";
        }
    }

    std::string normalizedRole(const std::string& role)
    {
        return role.empty() ? std::string("Exact") : role;
    }

    std::string normalizedSource(const std::string& source)
    {
        return source.empty() ? std::string("original") : source;
    }

    bool isOverrideSource(const std::string& source)
    {
        return !source.empty();
    }

    bool isSphereCover(const robot::RobotCollisionGeometry& geometry)
    {
        return geometry.type == robot::RobotGeometryType::Sphere &&
            normalizedRole(geometry.role) == "SphereCover";
    }

    bool isPrimitiveGeometry(robot::RobotGeometryType type)
    {
        return type == robot::RobotGeometryType::Box ||
            type == robot::RobotGeometryType::Sphere ||
            type == robot::RobotGeometryType::Cylinder;
    }

    std::string variantLabel(const std::string& source, const std::string& role)
    {
        if(source == "original") {
            return role == "Exact"
                ? std::string("Robot Collision Model")
                : std::string("Robot Collision Model ") + role;
        }
        if(source == simulation_project::kConvertFromVisualCollisionSource) {
            return "Convert from Visual";
        }
        if(source == simulation_project::kCoacdCollisionSource ||
            source == simulation_project::kCoacdVisualSource) {
            return "Generated from COACD";
        }
        if(source == "visualProxy") {
            return role == "SphereCover" ? std::string("Visual Sphere Cover") : std::string("Visual Proxy ") + role;
        }
        if(source == "collisionProxy") {
            return role == "SphereCover" ? std::string("Collision Sphere Cover") : std::string("Collision Proxy ") + role;
        }
        return role == "SphereCover" ? std::string("Manual Sphere Cover") : std::string("Manual Override ") + role;
    }

    std::string elementNameForSignature(const robot::RobotCollisionGeometry& geometry)
    {
        if(!geometry.partUid.empty()) {
            return geometry.partUid;
        }
        return geometryTypeName(geometry.type);
    }

    std::string stableHashHex(const std::vector<std::string>& values)
    {
        std::uint64_t hash = 1469598103934665603ull;
        for(const std::string& value : values) {
            for(char c : value) {
                hash ^= static_cast<unsigned char>(c);
                hash *= 1099511628211ull;
            }
            hash ^= 0xffu;
            hash *= 1099511628211ull;
        }

        std::ostringstream text;
        text << std::hex << std::setw(16) << std::setfill('0') << hash;
        return text.str();
    }

    std::string makeVariantId(
        const std::string& robotId,
        const std::string& linkName,
        const std::string& source,
        const std::string& role,
        const std::vector<std::string>& elementNames)
    {
        return robotId + "|" + linkName + "|" + source + "|" + role + "|" + stableHashHex(elementNames);
    }

    void finalizeVariantIds(
        RobotCollisionLinkSummary& summary,
        const std::string& robotId,
        const std::string& visibleVariantId)
    {
        for(RobotCollisionModelVariantSummary& variant : summary.variants) {
            variant.variantId = makeVariantId(
                robotId,
                summary.linkName,
                variant.source,
                variant.role,
                variant.elementNames);
            variant.visibleInViewport = !visibleVariantId.empty() && variant.variantId == visibleVariantId;
            variant.selectedInViewport = variant.visibleInViewport;
        }
    }

    void addStat(std::vector<RobotCollisionStat>& stats, const std::string& name)
    {
        auto it = std::find_if(
            stats.begin(),
            stats.end(),
            [&](const RobotCollisionStat& stat) {
                return stat.name == name;
            });

        if(it == stats.end()) {
            RobotCollisionStat stat;
            stat.name = name;
            stat.count = 1;
            stats.push_back(std::move(stat));
            return;
        }

        ++it->count;
    }

    void mergeStats(std::vector<RobotCollisionStat>& target, const std::vector<RobotCollisionStat>& source)
    {
        for(const RobotCollisionStat& stat : source) {
            for(std::size_t i = 0; i < stat.count; ++i) {
                addStat(target, stat.name);
            }
        }
    }

    void addMeshComplexity(
        const robot::RobotCollisionGeometry& geometry,
        RobotCollisionModelVariantSummary& variant)
    {
        if(geometry.type != robot::RobotGeometryType::Mesh || geometry.meshPath.empty()) {
            return;
        }
        ++variant.meshElementCount;

        std::error_code error;
        const std::filesystem::path meshPath = std::filesystem::u8path(geometry.meshPath);
        if(!std::filesystem::is_regular_file(meshPath, error)) {
            ++variant.unresolvedMeshCount;
            return;
        }

        const std::shared_ptr<assetcore::ModelDesc> modelDesc =
            assetcore::AssetManager::instance().tryLoadModel(
                geometry.meshPath,
                static_cast<float>(geometry.meshScale.x()));
        if(!modelDesc) {
            ++variant.unresolvedMeshCount;
            return;
        }

        for(const assetcore::SubMeshDesc& subMesh : modelDesc->subMeshes()) {
            variant.meshVertexCount += subMesh.geometry.positions.size();
            variant.meshTriangleCount += !subMesh.geometry.indices.empty()
                ? subMesh.geometry.indices.size() / 3
                : subMesh.geometry.positions.size() / 3;
        }
    }

    void addGeometryComplexity(
        const robot::RobotCollisionGeometry& geometry,
        RobotCollisionModelVariantSummary& variant)
    {
        if(geometry.type == robot::RobotGeometryType::Mesh) {
            addMeshComplexity(geometry, variant);
            return;
        }
        if(isPrimitiveGeometry(geometry.type)) {
            ++variant.primitiveCount;
        }
    }

    std::vector<std::string> orderedLinkNames(const robot::RobotModel& model)
    {
        std::vector<std::string> names = model.linkNames;
        if(!names.empty()) {
            return names;
        }

        names.reserve(model.links.size());
        for(const auto& item : model.links) {
            names.push_back(item.first);
        }
        std::sort(names.begin(), names.end());
        return names;
    }

    RobotCollisionModelVariantSummary& findOrCreateVariant(
        RobotCollisionLinkSummary& summary,
        const std::string& robotId,
        const std::string& role,
        const std::string& source,
        const std::string& activeDetectorRole,
        const std::string& activeDetectorSource)
    {
        auto it = std::find_if(
            summary.variants.begin(),
            summary.variants.end(),
            [&](const RobotCollisionModelVariantSummary& variant) {
                return variant.source == source && variant.role == role;
            });

        if(it != summary.variants.end()) {
            return *it;
        }

        RobotCollisionModelVariantSummary variant;
        variant.variantId = makeVariantId(robotId, summary.linkName, source, role, variant.elementNames);
        variant.label = variantLabel(source, role);
        variant.linkName = summary.linkName;
        variant.role = role;
        variant.source = source;
        variant.usedByActiveDetector = !activeDetectorRole.empty() &&
            role == normalizedRole(activeDetectorRole) &&
            (activeDetectorSource.empty() || source == normalizedSource(activeDetectorSource));
        summary.variants.push_back(std::move(variant));
        return summary.variants.back();
    }

    void appendVisualMeshVariant(
        RobotCollisionLinkSummary& summary,
        const robot::RobotLink& link,
        const std::string& robotId,
        const std::string& activeDetectorRole,
        const std::string& activeDetectorSource)
    {
        if(link.visuals.empty()) {
            return;
        }

        RobotCollisionModelVariantSummary& variant = findOrCreateVariant(
            summary,
            robotId,
            "Exact",
            simulation_project::kConvertFromVisualCollisionSource,
            activeDetectorRole,
            activeDetectorSource);

        for(std::size_t visualIndex = 0; visualIndex < link.visuals.size(); ++visualIndex) {
            const robot::RobotVisual& visual = link.visuals[visualIndex];
            if(visual.meshPath.empty()) {
                continue;
            }

            robot::RobotCollisionGeometry geometry;
            geometry.partUid = visual.partUid.empty()
                ? summary.linkName + "_visual_collision_" + std::to_string(visualIndex + 1)
                : visual.partUid;
            geometry.type = robot::RobotGeometryType::Mesh;
            geometry.meshPath = visual.meshPath;
            geometry.meshScale = Eigen::Vector3d::Ones() * static_cast<double>(visual.meshScale);
            geometry.role = "Exact";
            geometry.source = simulation_project::kConvertFromVisualCollisionSource;

            ++variant.elementCount;
            variant.elementNames.push_back(elementNameForSignature(geometry));
            addStat(variant.geometryTypes, geometryTypeName(geometry.type));
            addGeometryComplexity(geometry, variant);
        }
    }
}

RobotCollisionRobotSummary RobotCollisionModelInspector::summarizeRobot(
    const RuntimeRobot& robot,
    const std::string& activeDetectorRole,
    const std::string& activeDetectorSource,
    const std::string& visibleVariantId)
{
    RobotCollisionRobotSummary summary;
    summary.robotId = robot.documentId;
    summary.robotName = robot.name.empty() ? robot.model.name : robot.name;

    const std::vector<std::string> linkNames = orderedLinkNames(robot.model);
    summary.linkCount = linkNames.size();
    summary.links.reserve(linkNames.size());

    for(const std::string& linkName : linkNames) {
        RobotCollisionLinkSummary linkSummary = summarizeLink(
            robot,
            linkName,
            activeDetectorRole,
            activeDetectorSource,
            visibleVariantId);

        if(linkSummary.hasVisual) {
            ++summary.visualLinkCount;
        }
        if(linkSummary.hasVisual && linkSummary.effectiveCollisionCount == 0) {
            ++summary.visualOnlyLinkCount;
            summary.hasVisualOnlyLinks = true;
        }
        if(linkSummary.effectiveCollisionCount > 0) {
            ++summary.collisionReadyLinkCount;
            summary.hasCollisionReadyLinks = true;
        }
        if(linkSummary.hasOverrideCollision) {
            ++summary.overrideAppliedLinkCount;
            summary.hasOverrideAppliedLinks = true;
        }

        summary.visualCount += linkSummary.visualCount;
        summary.originalCollisionCount += linkSummary.originalCollisionCount;
        summary.overrideCollisionCount += linkSummary.overrideCollisionCount;
        summary.effectiveCollisionCount += linkSummary.effectiveCollisionCount;
        summary.hasMeshCollision = summary.hasMeshCollision || linkSummary.hasMeshCollision;
        summary.hasSphereCover = summary.hasSphereCover || linkSummary.hasSphereCover;
        mergeStats(summary.geometryTypes, linkSummary.geometryTypes);
        mergeStats(summary.roles, linkSummary.roles);
        mergeStats(summary.sources, linkSummary.sources);
        summary.links.push_back(std::move(linkSummary));
    }

    return summary;
}

RobotCollisionLinkSummary RobotCollisionModelInspector::summarizeLink(
    const RuntimeRobot& robot,
    const std::string& linkName,
    const std::string& activeDetectorRole,
    const std::string& activeDetectorSource,
    const std::string& visibleVariantId)
{
    RobotCollisionLinkSummary summary;
    summary.linkName = linkName;

    const auto linkIt = robot.model.links.find(linkName);
    if(linkIt == robot.model.links.end()) {
        return summary;
    }

    const robot::RobotLink& link = linkIt->second;
    summary.visualCount = link.visuals.size();
    summary.hasVisual = summary.visualCount > 0;
    appendVisualMeshVariant(
        summary,
        link,
        robot.documentId,
        activeDetectorRole,
        activeDetectorSource);

    for(const robot::RobotCollisionGeometry& geometry : link.collisions) {
        if(!geometry.enabled) {
            continue;
        }

        ++summary.effectiveCollisionCount;
        if(isOverrideSource(geometry.source)) {
            ++summary.overrideCollisionCount;
            summary.hasOverrideCollision = true;
        } else {
            ++summary.originalCollisionCount;
            summary.hasOriginalCollision = true;
        }

        summary.hasMeshCollision = summary.hasMeshCollision ||
            geometry.type == robot::RobotGeometryType::Mesh;
        summary.hasSphereCover = summary.hasSphereCover || isSphereCover(geometry);

        addStat(summary.geometryTypes, geometryTypeName(geometry.type));
        const std::string role = normalizedRole(geometry.role);
        const std::string source = normalizedSource(geometry.source);
        addStat(summary.roles, role);
        addStat(summary.sources, source);

        RobotCollisionModelVariantSummary& variant =
            findOrCreateVariant(summary, robot.documentId, role, source, activeDetectorRole, activeDetectorSource);
        ++variant.elementCount;
        variant.elementNames.push_back(elementNameForSignature(geometry));
        addStat(variant.geometryTypes, geometryTypeName(geometry.type));
        addGeometryComplexity(geometry, variant);
    }

    finalizeVariantIds(summary, robot.documentId, visibleVariantId);
    return summary;
}
