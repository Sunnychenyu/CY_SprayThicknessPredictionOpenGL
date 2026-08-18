#include "CollisionLinkModelsController.h"

#include <AssetCore/AssetManager.h>
#include <SimulationProject/AssetResolver.h>
#include <SimulationProject/CollisionModelSelectionIds.h>

#include <QStringList>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <memory>
#include <sstream>
#include <system_error>
#include <utility>

namespace
{
    using CollisionRuntimeDetectorInfo = robot_qt_viewer::CollisionRuntimeDetectorInfo;
    using CollisionRuntimeLinkSummary = robot_qt_viewer::CollisionRuntimeLinkSummary;
    using CollisionRuntimeModelVariantSummary = robot_qt_viewer::CollisionRuntimeModelVariantSummary;
    using CollisionRuntimeRobotSummary = robot_qt_viewer::CollisionRuntimeRobotSummary;
    using CollisionRuntimeStat = robot_qt_viewer::CollisionRuntimeStat;

    std::string normalizedCollisionRole(const std::string& role)
    {
        return role.empty() ? std::string("Exact") : role;
    }

    std::string normalizedCollisionSource(const std::string& source)
    {
        return source.empty() ? std::string("original") : source;
    }

    QString collisionStatsText(const std::vector<CollisionRuntimeStat>& stats)
    {
        if(stats.empty()) {
            return "none";
        }
        QStringList parts;
        for(const CollisionRuntimeStat& stat : stats) {
            parts << QString("%1:%2")
                .arg(QString::fromStdString(stat.name))
                .arg(static_cast<qulonglong>(stat.count));
        }
        return parts.join(", ");
    }

    QString primaryCollisionTypeText(const std::vector<CollisionRuntimeStat>& stats)
    {
        if(stats.empty()) {
            return "none";
        }
        const CollisionRuntimeStat* best = &stats.front();
        for(const CollisionRuntimeStat& stat : stats) {
            if(stat.count > best->count) {
                best = &stat;
            }
        }
        if(stats.size() == 1) {
            return QString::fromStdString(best->name);
        }
        return QString("%1+%2")
            .arg(QString::fromStdString(best->name))
            .arg(static_cast<qulonglong>(stats.size() - 1));
    }

    QString shortCollisionSourceText(const std::string& source)
    {
        if(source == "original") {
            return "original";
        }
        if(source == simulation_project::kCoacdCollisionSource ||
            source == simulation_project::kCoacdVisualSource) {
            return "coacd";
        }
        if(source == "visualProxy") {
            return "visual";
        }
        if(source == "collisionProxy") {
            return "proxy";
        }
        return QString::fromStdString(source.empty() ? std::string("override") : source);
    }

    QString collisionVariantStateText(
        const CollisionRuntimeModelVariantSummary& variant,
        const QString& activeModelId)
    {
        QStringList states;
        if(!activeModelId.isEmpty() &&
            (variant.variantId == activeModelId.toStdString() ||
                (simulation_project::isConvertFromVisualCollisionModelId(activeModelId.toStdString()) &&
                    variant.source == simulation_project::kConvertFromVisualCollisionSource))) {
            states << "Current";
        }
        if(variant.visibleInViewport) {
            states << "Shown";
        }
        if(variant.usedByActiveDetector) {
            states << "Used";
        }
        return states.isEmpty() ? "-" : states.join("+");
    }

    QString activeLinkModelId(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId,
        const std::string& linkName)
    {
        for(const simulation_project::RobotLinkCollisionModelSelectionDesc& selection :
            document.collision.robotLinkModelSelections) {
            if(selection.robotId == robotId && selection.linkName == linkName) {
                return QString::fromStdString(
                    simulation_project::normalizeRobotLinkCollisionModelId(selection.activeModelId));
            }
        }
        return QString();
    }

    QString variantPolicyText(const CollisionRuntimeModelVariantSummary* variant)
    {
        if(variant == nullptr) {
            return "none";
        }
        const bool hasMesh = std::any_of(
            variant->geometryTypes.begin(),
            variant->geometryTypes.end(),
            [](const CollisionRuntimeStat& stat) {
                return stat.name == "Mesh" && stat.count > 0;
            });
        if(variant->source == "original") {
            return hasMesh ? QString("exact mesh") : QString("original exact");
        }
        if(variant->source == "collisionProxy" || variant->source == "visualProxy") {
            return "explicit proxy";
        }
        if(variant->source == simulation_project::kCoacdCollisionSource ||
            variant->source == simulation_project::kCoacdVisualSource) {
            return "generated coacd";
        }
        return "manual override";
    }

    const simulation_project::CollisionDetectorDesc* findDetector(
        const simulation_project::ProjectDocument& document,
        const QString& detectorId)
    {
        const std::string id = detectorId.toStdString();
        auto it = std::find_if(
            document.collision.detectors.begin(),
            document.collision.detectors.end(),
            [&](const simulation_project::CollisionDetectorDesc& detector) {
                return detector.id == id;
            });
        return it == document.collision.detectors.end() ? nullptr : &(*it);
    }

    const simulation_project::RobotCollisionOverrideDesc* findCollisionOverride(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId)
    {
        auto it = std::find_if(
            document.collision.robotOverrides.begin(),
            document.collision.robotOverrides.end(),
            [&](const simulation_project::RobotCollisionOverrideDesc& collisionOverride) {
                return collisionOverride.robotId == robotId;
            });
        return it == document.collision.robotOverrides.end() ? nullptr : &(*it);
    }

    const CollisionRuntimeLinkSummary* findLinkSummary(
        const CollisionRuntimeRobotSummary* robotSummary,
        const std::string& linkName)
    {
        if(robotSummary == nullptr) {
            return nullptr;
        }
        for(const CollisionRuntimeLinkSummary& candidate : robotSummary->links) {
            if(candidate.linkName == linkName) {
                return &candidate;
            }
        }
        return nullptr;
    }

    QString activeSourceForDetector(const simulation_project::CollisionDetectorDesc* detector)
    {
        return detector != nullptr && !detector->geometrySource.empty()
            ? QString::fromStdString(normalizedCollisionSource(detector->geometrySource))
            : QString();
    }

    QString targetDisplayName(const QString& selectedRobotId, const QString& selectedLinkName)
    {
        if(selectedRobotId.isEmpty() || selectedLinkName.isEmpty()) {
            return QString();
        }
        return QString("Link: %1.%2").arg(selectedRobotId, selectedLinkName);
    }

    QString importedOrVisualSourceLabel(const CollisionRuntimeLinkSummary* linkSummary)
    {
        if(linkSummary == nullptr) {
            return "No source";
        }
        if(linkSummary->originalCollisionCount > 0 || linkSummary->hasOriginalCollision) {
            return "Imported Collision Model";
        }
        if(linkSummary->visualCount > 0 || linkSummary->hasVisual) {
            return "Convert from Visual";
        }
        return "No collision source";
    }

    QString importedOrVisualSourceDetail(const CollisionRuntimeLinkSummary* linkSummary)
    {
        if(linkSummary == nullptr) {
            return "Select a robot link to inspect its collision source.";
        }
        if(linkSummary->originalCollisionCount > 0 || linkSummary->hasOriginalCollision) {
            return "The robot importer reported an original collision model. The current runtime summary does not distinguish URDF from Simscape.";
        }
        if(linkSummary->visualCount > 0 || linkSummary->hasVisual) {
            return "No imported collision model was reported for this link; visual geometry is used as the collision basis.";
        }
        return "No visual or collision geometry was reported for this link.";
    }

    QString sourceLabelForVariant(const CollisionRuntimeModelVariantSummary& variant)
    {
        if(variant.source == "original") {
            return "Robot Collision Model";
        }
        if(variant.source == simulation_project::kCoacdCollisionSource ||
            variant.source == simulation_project::kCoacdVisualSource) {
            return "Application generated assets";
        }
        if(variant.source == "visualProxy" ||
            variant.source == simulation_project::kConvertFromVisualCollisionSource) {
            return variant.source == simulation_project::kConvertFromVisualCollisionSource
                ? QString("Visual Geometry")
                : QString("Project / system.json");
        }
        return "Project / system.json";
    }

    QString variantLabelText(const CollisionRuntimeModelVariantSummary& variant)
    {
        if(variant.source == "original") {
            return "Robot Collision Model";
        }
        if(variant.source == simulation_project::kConvertFromVisualCollisionSource) {
            return "Convert from Visual";
        }
        if(variant.source == simulation_project::kCoacdCollisionSource ||
            variant.source == simulation_project::kCoacdVisualSource) {
            return "Generated from COACD";
        }
        return "Defined in Project";
    }

    struct CollisionComplexityMetrics
    {
        std::size_t parts = 0;
        std::size_t meshParts = 0;
        std::size_t meshVertices = 0;
        std::size_t meshTriangles = 0;
        std::size_t unresolvedMeshes = 0;
        std::size_t primitives = 0;
        std::vector<CollisionRuntimeStat> primitiveTypes;
    };

    QString countText(std::size_t count)
    {
        return QString::number(static_cast<qulonglong>(count));
    }

    QString meshCountText(std::size_t count, std::size_t unresolvedMeshes)
    {
        if(unresolvedMeshes > 0 && count == 0) {
            return QString("N/A (%1 unloaded)").arg(static_cast<qulonglong>(unresolvedMeshes));
        }
        if(unresolvedMeshes > 0) {
            return QString("%1 (+%2 unloaded)")
                .arg(static_cast<qulonglong>(count))
                .arg(static_cast<qulonglong>(unresolvedMeshes));
        }
        return countText(count);
    }

    QVector<CollisionLinkModelMetricRowView> complexityRows(const CollisionComplexityMetrics& metrics)
    {
        auto row = [](const QString& label, const QString& value) {
            CollisionLinkModelMetricRowView result;
            result.label = label;
            result.value = value;
            return result;
        };

        const QString primitiveTypes = collisionStatsText(metrics.primitiveTypes);
        QVector<CollisionLinkModelMetricRowView> rows;
        rows.push_back(row("Parts", countText(metrics.parts)));
        rows.push_back(row("Mesh parts", countText(metrics.meshParts)));
        rows.push_back(row("Mesh vertices", meshCountText(metrics.meshVertices, metrics.unresolvedMeshes)));
        rows.push_back(row("Mesh triangles", meshCountText(metrics.meshTriangles, metrics.unresolvedMeshes)));
        rows.push_back(row("Primitives", countText(metrics.primitives)));
        rows.push_back(row("Primitive types", primitiveTypes));
        return rows;
    }

    bool isMeshTypeText(const QString& type)
    {
        return type.compare("Mesh", Qt::CaseInsensitive) == 0;
    }

    bool isPrimitiveTypeText(const QString& type)
    {
        return type.compare("Box", Qt::CaseInsensitive) == 0 ||
            type.compare("Sphere", Qt::CaseInsensitive) == 0 ||
            type.compare("Cylinder", Qt::CaseInsensitive) == 0;
    }

    void addRuntimeStat(std::vector<CollisionRuntimeStat>& stats, const QString& name)
    {
        const std::string text = name.toStdString();
        auto it = std::find_if(
            stats.begin(),
            stats.end(),
            [&](const CollisionRuntimeStat& stat) {
                return stat.name == text;
            });
        if(it == stats.end()) {
            CollisionRuntimeStat stat;
            stat.name = text;
            stat.count = 1;
            stats.push_back(std::move(stat));
            return;
        }
        ++it->count;
    }

    std::filesystem::path ancestorWithDataDirectory(std::filesystem::path start)
    {
        std::error_code error;
        start = start.empty()
            ? std::filesystem::current_path(error)
            : std::filesystem::weakly_canonical(start, error);
        if(error) {
            start = start.lexically_normal();
        }

        for(std::filesystem::path cursor = start; !cursor.empty(); cursor = cursor.parent_path()) {
            if(std::filesystem::is_directory(cursor / "data", error)) {
                return cursor;
            }
            if(cursor == cursor.parent_path()) {
                break;
            }
        }
        return start;
    }

    std::filesystem::path resolveSourceRoot(const std::filesystem::path& projectBasePath)
    {
        const std::filesystem::path fromProjectBase = ancestorWithDataDirectory(projectBasePath);
        std::error_code error;
        if(std::filesystem::is_directory(fromProjectBase / "data", error)) {
            return fromProjectBase;
        }

        const std::filesystem::path currentPath = std::filesystem::current_path(error);
        const std::filesystem::path fromCurrent = ancestorWithDataDirectory(currentPath);
        if(std::filesystem::is_directory(fromCurrent / "data", error)) {
            return fromCurrent;
        }

        return projectBasePath.empty() ? currentPath : projectBasePath;
    }

    std::string pathToUtf8(const std::filesystem::path& path)
    {
        return path.generic_u8string();
    }

    simulation_project::AssetResolveContext makeResolveContext(
        const std::filesystem::path& projectBasePath,
        const simulation_project::ProjectDocument& document)
    {
        simulation_project::AssetResolveContext context;
        context.projectBasePath = projectBasePath.empty()
            ? std::filesystem::current_path()
            : projectBasePath;
        context.sourceRootPath = resolveSourceRoot(context.projectBasePath);
        context.dataRootPath = context.sourceRootPath / "data";
        context.appRootPath = std::filesystem::current_path();
        context.assetSearchPaths = document.assetSearchPaths;
        return context;
    }

    std::filesystem::path resolveStoredAssetPath(
        const simulation_project::AssetResolveContext& context,
        const std::string& storedPath)
    {
        return storedPath.empty()
            ? std::filesystem::path()
            : simulation_project::AssetResolver::resolveProjectPath(context, storedPath);
    }

    void appendMeshAssetMetrics(
        CollisionComplexityMetrics& metrics,
        const std::filesystem::path& meshPath,
        double scale)
    {
        ++metrics.parts;
        ++metrics.meshParts;

        std::error_code error;
        if(meshPath.empty() || !std::filesystem::is_regular_file(meshPath, error)) {
            ++metrics.unresolvedMeshes;
            return;
        }

        const std::shared_ptr<assetcore::ModelDesc> modelDesc =
            assetcore::AssetManager::instance().tryLoadModel(
                pathToUtf8(meshPath),
                static_cast<float>(scale));
        if(!modelDesc) {
            ++metrics.unresolvedMeshes;
            return;
        }

        for(const assetcore::SubMeshDesc& subMesh : modelDesc->subMeshes()) {
            metrics.meshVertices += subMesh.geometry.positions.size();
            metrics.meshTriangles += !subMesh.geometry.indices.empty()
                ? subMesh.geometry.indices.size() / 3
                : subMesh.geometry.positions.size() / 3;
        }
    }

    void appendStoredMeshAssetMetrics(
        CollisionComplexityMetrics& metrics,
        const simulation_project::AssetResolveContext& context,
        const std::string& storedPath,
        double scale)
    {
        appendMeshAssetMetrics(metrics, resolveStoredAssetPath(context, storedPath), scale);
    }

    void appendObjectElementMetrics(
        CollisionComplexityMetrics& metrics,
        const simulation_project::ObjectCollisionElementOverrideDesc& element,
        const simulation_project::AssetResolveContext& context)
    {
        if(!element.enabled) {
            return;
        }

        const QString type = QString::fromStdString(element.type.empty() ? std::string("Box") : element.type);
        if(isMeshTypeText(type)) {
            appendStoredMeshAssetMetrics(metrics, context, element.meshPath, element.meshScale.x);
            return;
        }

        ++metrics.parts;
        if(isPrimitiveTypeText(type)) {
            ++metrics.primitives;
            addRuntimeStat(metrics.primitiveTypes, type);
        }
    }

    QVector<CollisionLinkModelMetricRowView> collisionVariantComplexityRows(
        const CollisionRuntimeModelVariantSummary& variant,
        const CollisionRuntimeLinkSummary* linkSummary)
    {
        (void)linkSummary;

        CollisionComplexityMetrics metrics;
        metrics.parts = variant.elementCount;
        metrics.meshParts = variant.meshElementCount;
        metrics.meshVertices = variant.meshVertexCount;
        metrics.meshTriangles = variant.meshTriangleCount;
        metrics.unresolvedMeshes = variant.unresolvedMeshCount;
        metrics.primitives = variant.primitiveCount;
        for(const CollisionRuntimeStat& stat : variant.geometryTypes) {
            const QString type = QString::fromStdString(stat.name);
            if(!isPrimitiveTypeText(type)) {
                continue;
            }
            CollisionRuntimeStat primitiveStat;
            primitiveStat.name = stat.name;
            primitiveStat.count = stat.count;
            metrics.primitiveTypes.push_back(std::move(primitiveStat));
        }
        return complexityRows(metrics);
    }

    bool isPreferredOriginalVariant(const CollisionLinkModelVariantItemView& item)
    {
        return item.source == "original" &&
            (item.role.isEmpty() || item.role == "Exact");
    }

    bool hasVisualCandidate(const QVector<CollisionLinkModelVariantItemView>& variants)
    {
        for(const CollisionLinkModelVariantItemView& item : variants) {
            if(item.source == simulation_project::kConvertFromVisualCollisionSource) {
                return true;
            }
        }
        return false;
    }

    bool isProjectDefinedCandidate(const CollisionLinkModelVariantItemView& item)
    {
        return item.source != simulation_project::kConvertFromVisualCollisionSource &&
            item.source != "original";
    }

    int variantDisplayRank(const CollisionLinkModelVariantItemView& item)
    {
        if(item.source == simulation_project::kConvertFromVisualCollisionSource) {
            return 0;
        }
        if(item.source == "original") {
            return 1;
        }
        if(item.source == simulation_project::kCoacdCollisionSource ||
            item.source == simulation_project::kCoacdVisualSource) {
            return 2;
        }
        return 3;
    }

    const simulation_project::ObjectCollisionOverrideDesc* findObjectCollisionOverride(
        const simulation_project::ProjectDocument& document,
        const std::string& objectId)
    {
        auto it = std::find_if(
            document.collision.objectOverrides.begin(),
            document.collision.objectOverrides.end(),
            [&](const simulation_project::ObjectCollisionOverrideDesc& collisionOverride) {
                return collisionOverride.objectId == objectId;
            });
        return it == document.collision.objectOverrides.end() ? nullptr : &(*it);
    }

    bool isCoacdSource(const std::string& source)
    {
        return simulation_project::isCoacdCollisionSource(source);
    }

    bool objectOverrideHasElements(const simulation_project::ObjectCollisionOverrideDesc* collisionOverride)
    {
        return collisionOverride != nullptr &&
            std::any_of(
                collisionOverride->elements.begin(),
                collisionOverride->elements.end(),
                [](const simulation_project::ObjectCollisionElementOverrideDesc& element) {
                    return element.enabled && !isCoacdSource(element.source);
                });
    }

    QString objectOverrideTypeLabel(const simulation_project::ObjectCollisionOverrideDesc* collisionOverride)
    {
        if(collisionOverride == nullptr) {
            return "Override";
        }
        QStringList types;
        for(const simulation_project::ObjectCollisionElementOverrideDesc& element : collisionOverride->elements) {
            if(!element.enabled || isCoacdSource(element.source)) {
                continue;
            }
            const QString type = QString::fromStdString(element.type.empty() ? std::string("override") : element.type);
            if(!types.contains(type, Qt::CaseInsensitive)) {
                types << type;
            }
        }
        if(types.isEmpty()) {
            return "Override";
        }
        return types.size() == 1 ? types.front() : QString("%1+%2").arg(types.front()).arg(types.size() - 1);
    }

    const simulation_project::SceneObjectDesc* findSceneObject(
        const simulation_project::ProjectDocument& document,
        const std::string& objectId)
    {
        auto it = std::find_if(
            document.objects.begin(),
            document.objects.end(),
            [&](const simulation_project::SceneObjectDesc& object) {
                return object.id == objectId;
            });
        return it == document.objects.end() ? nullptr : &(*it);
    }

    const simulation_project::AttachmentAssetDesc* findAttachmentAsset(
        const simulation_project::ProjectDocument& document,
        const std::string& assetId)
    {
        auto it = std::find_if(
            document.attachmentAssets.begin(),
            document.attachmentAssets.end(),
            [&](const simulation_project::AttachmentAssetDesc& asset) {
                return asset.id == assetId;
            });
        return it == document.attachmentAssets.end() ? nullptr : &(*it);
    }

    const simulation_project::MountedAttachmentDesc* findMountedAttachment(
        const simulation_project::ProjectDocument& document,
        const std::string& attachmentId)
    {
        auto it = std::find_if(
            document.mountedAttachments.begin(),
            document.mountedAttachments.end(),
            [&](const simulation_project::MountedAttachmentDesc& attachment) {
                return attachment.id == attachmentId;
            });
        return it == document.mountedAttachments.end() ? nullptr : &(*it);
    }

    QVector<CollisionLinkModelMetricRowView> objectVisualComplexityRows(
        const simulation_project::ProjectDocument& document,
        const QString& targetId,
        const QString& selectedAttachmentId,
        const std::filesystem::path& projectBasePath)
    {
        CollisionComplexityMetrics metrics;
        const simulation_project::AssetResolveContext context = makeResolveContext(projectBasePath, document);
        if(selectedAttachmentId.isEmpty()) {
            const simulation_project::SceneObjectDesc* object =
                findSceneObject(document, targetId.toStdString());
            if(object != nullptr) {
                appendStoredMeshAssetMetrics(metrics, context, object->sourcePath, object->visualScale);
            }
        } else {
            const simulation_project::MountedAttachmentDesc* attachment =
                findMountedAttachment(document, selectedAttachmentId.toStdString());
            const simulation_project::AttachmentAssetDesc* asset = attachment != nullptr
                ? findAttachmentAsset(document, attachment->assetId)
                : nullptr;
            if(asset != nullptr) {
                appendStoredMeshAssetMetrics(metrics, context, asset->visualPath, asset->visualScale);
            }
        }
        return complexityRows(metrics);
    }

    QVector<CollisionLinkModelMetricRowView> unresolvedLinkVisualComplexityRows()
    {
        CollisionComplexityMetrics metrics;
        metrics.parts = 1;
        metrics.meshParts = 1;
        metrics.unresolvedMeshes = 1;
        return complexityRows(metrics);
    }

    QVector<CollisionLinkModelMetricRowView> objectProjectDefinedComplexityRows(
        const simulation_project::ObjectCollisionOverrideDesc* collisionOverride,
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& projectBasePath)
    {
        CollisionComplexityMetrics metrics;
        if(collisionOverride != nullptr) {
            const simulation_project::AssetResolveContext context = makeResolveContext(projectBasePath, document);
            for(const simulation_project::ObjectCollisionElementOverrideDesc& element : collisionOverride->elements) {
                if(isCoacdSource(element.source)) {
                    continue;
                }
                appendObjectElementMetrics(metrics, element, context);
            }
        }
        return complexityRows(metrics);
    }

    std::string normalizedObjectElementRole(
        const simulation_project::ObjectCollisionElementOverrideDesc& element)
    {
        return normalizedCollisionRole(element.role);
    }

    std::string normalizedObjectElementSource(
        const simulation_project::ObjectCollisionElementOverrideDesc& element)
    {
        return normalizedCollisionSource(element.source);
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

    QString makeObjectVariantId(
        const QString& targetId,
        const std::string& source,
        const std::string& role,
        const std::vector<std::string>& elementIds)
    {
        return QString("%1|%2|%3|%4")
            .arg(targetId)
            .arg(QString::fromStdString(source))
            .arg(QString::fromStdString(role))
            .arg(QString::fromStdString(stableHashHex(elementIds)));
    }

    QString activeObjectModelId(
        const simulation_project::ProjectDocument& document,
        const QString& targetId,
        const simulation_project::ObjectCollisionOverrideDesc* collisionOverride)
    {
        const std::string objectId = targetId.toStdString();
        for(const simulation_project::ObjectCollisionModelSelectionDesc& selection :
            document.collision.objectModelSelections) {
            if(selection.objectId == objectId && !selection.activeModelId.empty()) {
                return QString::fromStdString(
                    simulation_project::normalizeObjectCollisionModelId(selection.activeModelId));
            }
        }

        if(collisionOverride != nullptr &&
            collisionOverride->replaceOriginalCollisions &&
            objectOverrideHasElements(collisionOverride)) {
            return simulation_project::kDefinedInProjectCollisionModelId;
        }
        return simulation_project::kConvertFromVisualCollisionModelId;
    }

    QVector<CollisionLinkModelMetricRowView> objectFilteredOverrideComplexityRows(
        const simulation_project::ObjectCollisionOverrideDesc* collisionOverride,
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& projectBasePath,
        const std::string& source,
        const std::string& role)
    {
        CollisionComplexityMetrics metrics;
        if(collisionOverride != nullptr) {
            const simulation_project::AssetResolveContext context = makeResolveContext(projectBasePath, document);
            for(const simulation_project::ObjectCollisionElementOverrideDesc& element : collisionOverride->elements) {
                if(!element.enabled ||
                    normalizedObjectElementSource(element) != source ||
                    normalizedObjectElementRole(element) != role) {
                    continue;
                }
                appendObjectElementMetrics(metrics, element, context);
            }
        }
        return complexityRows(metrics);
    }

    void appendObjectCoacdVariants(
        CollisionLinkModelsViewModel& view,
        const simulation_project::ProjectDocument& document,
        const simulation_project::ObjectCollisionOverrideDesc* collisionOverride,
        const QString& targetId,
        const QString& activeModelId,
        const std::filesystem::path& projectBasePath)
    {
        if(collisionOverride == nullptr) {
            return;
        }

        struct Group
        {
            std::string source;
            std::string role;
            std::vector<std::string> elementIds;
            QString typeLabel;
        };

        std::vector<Group> groups;
        for(const simulation_project::ObjectCollisionElementOverrideDesc& element : collisionOverride->elements) {
            if(!element.enabled) {
                continue;
            }
            const std::string source = normalizedObjectElementSource(element);
            if(!isCoacdSource(source)) {
                continue;
            }
            const std::string role = normalizedObjectElementRole(element);
            auto it = std::find_if(
                groups.begin(),
                groups.end(),
                [&](const Group& group) {
                    return group.source == source && group.role == role;
                });
            if(it == groups.end()) {
                Group group;
                group.source = source;
                group.role = role;
                group.typeLabel = QString::fromStdString(element.type.empty() ? std::string("Mesh") : element.type);
                groups.push_back(std::move(group));
                it = std::prev(groups.end());
            }
            it->elementIds.push_back(element.id);
        }

        for(const Group& group : groups) {
            CollisionLinkModelVariantItemView item;
            item.label = "Generated from COACD";
            item.sourceLabel = "Application generated assets";
            item.typeLabel = group.typeLabel;
            item.roleLabel = QString::fromStdString(group.role);
            item.detail = QString("Use COACD generated object collision parts.");
            item.tooltip = item.detail;
            item.complexityRows = objectFilteredOverrideComplexityRows(
                collisionOverride,
                document,
                projectBasePath,
                group.source,
                group.role);
            item.role = QString::fromStdString(group.role);
            item.source = QString::fromStdString(group.source);
            item.variantId = makeObjectVariantId(targetId, group.source, group.role, group.elementIds);
            item.current = item.variantId == activeModelId;
            view.variants.push_back(std::move(item));
        }
    }

    QString objectCollisionTargetId(const QString& selectedObjectId, const QString& selectedAttachmentId)
    {
        return !selectedAttachmentId.isEmpty() ? selectedAttachmentId : selectedObjectId;
    }

    QString objectCollisionEntityKind(const QString& selectedAttachmentId)
    {
        return selectedAttachmentId.isEmpty() ? QString("Object") : QString("Attachment");
    }

    QString objectCollisionTargetDisplayName(const QString& targetId, const QString& selectedAttachmentId)
    {
        if(targetId.isEmpty()) {
            return QString();
        }
        return QString("%1: %2").arg(objectCollisionEntityKind(selectedAttachmentId), targetId);
    }

    CollisionLinkModelsSummaryView buildObjectSummary(
        const simulation_project::ProjectDocument& document,
        const QString& selectedObjectId,
        const QString& selectedAttachmentId,
        const QString& qualityMessage)
    {
        CollisionLinkModelsSummaryView view;
        const QString targetId = objectCollisionTargetId(selectedObjectId, selectedAttachmentId);
        const std::string objectId = targetId.toStdString();
        const simulation_project::ObjectCollisionOverrideDesc* collisionOverride =
            findObjectCollisionOverride(document, objectId);
        const bool hasProjectDefined = objectOverrideHasElements(collisionOverride);
        const QString activeModelId = activeObjectModelId(document, targetId, collisionOverride);

        view.target.valid = !targetId.isEmpty();
        view.target.entityKind = objectCollisionEntityKind(selectedAttachmentId);
        view.target.displayName = targetId.isEmpty()
            ? QString("No tree node selected")
            : objectCollisionTargetDisplayName(targetId, selectedAttachmentId);
        view.target.stableId = targetId;
        view.originalSource.label = "Convert from Visual";
        view.originalSource.detail = selectedAttachmentId.isEmpty()
            ? QString("Use the scene object's visual mesh as the collision basis.")
            : QString("Use the mounted attachment visual mesh as the collision basis.");
        view.originalSource.sourceKind = "visual";
        view.replaceOriginal = activeModelId == simulation_project::kDefinedInProjectCollisionModelId;

        QStringList status;
        status << QString("variants %1").arg(hasProjectDefined ? 2 : 1);
        status << QString("current %1").arg(
            activeModelId == simulation_project::kDefinedInProjectCollisionModelId
                ? QString("Defined in Project")
                : (simulation_project::isConvertFromVisualCollisionModelId(activeModelId.toStdString())
                    ? QString("Convert from Visual")
                    : QString("Generated from COACD")));
        status << QString("preview %1").arg("none");
        if(!qualityMessage.isEmpty()) {
            status << QString("quality warning");
        }
        view.statusText = targetId.isEmpty() ? QString("No configurable object is selected.") : status.join(" | ");
        return view;
    }

    CollisionLinkModelsViewModel buildObjectViewModel(
        const simulation_project::ProjectDocument& document,
        const QString& selectedObjectId,
        const QString& selectedAttachmentId,
        const QString& previousVariantId,
        const QString& qualityMessage,
        const std::filesystem::path& projectBasePath)
    {
        CollisionLinkModelsViewModel view;
        const QString targetId = objectCollisionTargetId(selectedObjectId, selectedAttachmentId);
        if(targetId.isEmpty()) {
            view.summary = buildObjectSummary(document, selectedObjectId, selectedAttachmentId, qualityMessage);
            CollisionLinkModelVariantItemView item;
            item.label = "No object selected";
            item.detail = "Open the left project tree context menu on an object or mounted attachment.";
            item.enabled = false;
            view.variants.push_back(std::move(item));
            return view;
        }

        const simulation_project::ObjectCollisionOverrideDesc* collisionOverride =
            findObjectCollisionOverride(document, targetId.toStdString());
        const bool hasProjectDefined = objectOverrideHasElements(collisionOverride);
        const QString activeModelId = activeObjectModelId(document, targetId, collisionOverride);

        view.summary = buildObjectSummary(document, selectedObjectId, selectedAttachmentId, qualityMessage);

        CollisionLinkModelVariantItemView visual;
        visual.label = "Convert from Visual";
        visual.sourceLabel = "Visual Mesh";
        visual.typeLabel = "Visual Mesh";
        visual.roleLabel = "Exact";
        visual.detail = selectedAttachmentId.isEmpty()
            ? QString("Use the scene object's visual mesh as the collision model.")
            : QString("Use the mounted attachment visual mesh as the collision model.");
        visual.tooltip = visual.detail;
        visual.complexityRows = objectVisualComplexityRows(
            document,
            targetId,
            selectedAttachmentId,
            projectBasePath);
        visual.role = "Exact";
        visual.source = simulation_project::kConvertFromVisualCollisionSource;
        visual.variantId = simulation_project::kConvertFromVisualCollisionModelId;
        visual.current = simulation_project::isConvertFromVisualCollisionModelId(activeModelId.toStdString());
        view.variants.push_back(std::move(visual));

        if(hasProjectDefined) {
            CollisionLinkModelVariantItemView projectDefined;
            projectDefined.label = "Defined in Project";
            projectDefined.sourceLabel = "Project / system.json";
            projectDefined.typeLabel = objectOverrideTypeLabel(collisionOverride);
            projectDefined.roleLabel = "Exact";
            projectDefined.detail = QString("Use project-defined object collision override elements.");
            projectDefined.tooltip = projectDefined.detail;
            projectDefined.complexityRows = objectProjectDefinedComplexityRows(
                collisionOverride,
                document,
                projectBasePath);
            projectDefined.role = "Exact";
            projectDefined.source = simulation_project::kDefinedInProjectCollisionSource;
            projectDefined.variantId = simulation_project::kDefinedInProjectCollisionModelId;
            projectDefined.current = activeModelId == simulation_project::kDefinedInProjectCollisionModelId;
            view.variants.push_back(std::move(projectDefined));
        }

        appendObjectCoacdVariants(
            view,
            document,
            collisionOverride,
            targetId,
            activeModelId,
            projectBasePath);

        int targetIndex = -1;
        int currentIndex = -1;
        for(int i = 0; i < view.variants.size(); ++i) {
            if(view.variants[i].variantId == previousVariantId) {
                targetIndex = i;
            }
            if(view.variants[i].current) {
                currentIndex = i;
            }
        }
        if(targetIndex < 0) {
            targetIndex = currentIndex >= 0 ? currentIndex : 0;
        }
        if(targetIndex >= 0 && targetIndex < view.variants.size()) {
            view.variants[targetIndex].selected = true;
        }
        return view;
    }
}

CollisionLinkModelsViewModel CollisionLinkModelsController::buildViewModel(
    const simulation_project::ProjectDocument& document,
    const QString& selectedRobotId,
    const QString& selectedLinkName,
    const QString& selectedObjectId,
    const QString& selectedAttachmentId,
    const QString& activeDetectorId,
    const QString& previousVariantId,
    const QString& visibleVariantId,
    const QString& qualityMessage,
    const std::filesystem::path& projectBasePath,
    const robot_qt_viewer::CollisionRuntimeRobotSummary* robotSummary,
    const std::vector<robot_qt_viewer::CollisionRuntimeDetectorInfo>& runtimeDetectors)
{
    CollisionLinkModelsViewModel view;
    if(!selectedObjectId.isEmpty() || !selectedAttachmentId.isEmpty()) {
        return buildObjectViewModel(
            document,
            selectedObjectId,
            selectedAttachmentId,
            previousVariantId,
            qualityMessage,
            projectBasePath);
    }

    const std::string robotId = selectedRobotId.toStdString();
    const std::string linkName = selectedLinkName.toStdString();
    if(robotId.empty() || linkName.empty() || robotSummary == nullptr) {
        view.summary.target.entityKind = "Link";
        view.summary.target.displayName = "Select a robot link";
        view.summary.originalSource.label = "No source";
        view.summary.originalSource.detail = "Select a robot link to configure its collision model.";
        view.summary.statusText = "Select a robot link";

        CollisionLinkModelVariantItemView variant;
        variant.label = "Select a robot link";
        variant.detail = "Open the left project tree context menu on a robot link.";
        variant.enabled = false;
        view.variants.push_back(std::move(variant));
        return view;
    }

    view.summary = buildSummary(
        document,
        selectedRobotId,
        selectedLinkName,
        selectedObjectId,
        selectedAttachmentId,
        activeDetectorId,
        qualityMessage,
        robotSummary,
        runtimeDetectors);

    const simulation_project::CollisionDetectorDesc* activeDetector =
        findDetector(document, activeDetectorId);
    const QString activeRole = activeDetector != nullptr
        ? QString::fromStdString(activeDetector->geometryRole)
        : QString();
    const QString activeSource = activeSourceForDetector(activeDetector);
    const QString activeModelId = activeLinkModelId(document, robotId, linkName);

    const CollisionRuntimeLinkSummary* linkSummary = findLinkSummary(robotSummary, linkName);
    if(linkSummary != nullptr) {
        for(const CollisionRuntimeModelVariantSummary& variant : linkSummary->variants) {
            CollisionLinkModelVariantItemView item;
            item.label = variantLabelText(variant);
            item.sourceLabel = sourceLabelForVariant(variant);
            item.typeLabel = primaryCollisionTypeText(variant.geometryTypes);
            item.roleLabel = QString::fromStdString(normalizedCollisionRole(variant.role));
            item.detail = QString("elements %1 | source %2")
                .arg(static_cast<qulonglong>(variant.elementCount))
                .arg(QString::fromStdString(variant.source.empty() ? std::string("override") : variant.source));
            item.complexityRows = collisionVariantComplexityRows(variant, linkSummary);
            item.role = QString::fromStdString(normalizedCollisionRole(variant.role));
            item.source = QString::fromStdString(normalizedCollisionSource(variant.source));
            item.variantId = QString::fromStdString(variant.variantId);
            item.current = !activeModelId.isEmpty() &&
                (item.variantId == activeModelId ||
                    (simulation_project::isConvertFromVisualCollisionModelId(activeModelId.toStdString()) &&
                        item.source == simulation_project::kConvertFromVisualCollisionSource));
            item.tooltip = QString("label: %1\nstate: %2\nrole: %3\nsource: %4\nelements: %5\ntypes: %6\nid: %7")
                .arg(item.label)
                .arg(collisionVariantStateText(variant, activeModelId))
                .arg(item.role)
                .arg(item.source)
                .arg(static_cast<qulonglong>(variant.elementCount))
                .arg(collisionStatsText(variant.geometryTypes))
                .arg(item.variantId);
            view.variants.push_back(std::move(item));
        }

        if(linkSummary->hasVisual && !hasVisualCandidate(view.variants)) {
            CollisionLinkModelVariantItemView item;
            item.label = "Convert from Visual";
            item.sourceLabel = "Visual";
            item.typeLabel = "Visual Mesh";
            item.roleLabel = "Exact";
            item.detail = "Use the selected link visual geometry as the collision model basis.";
            item.tooltip = item.detail;
            item.complexityRows = unresolvedLinkVisualComplexityRows();
            item.role = "Exact";
            item.source = simulation_project::kConvertFromVisualCollisionSource;
            item.variantId = simulation_project::kConvertFromVisualCollisionModelId;
            item.current = !activeModelId.isEmpty() &&
                simulation_project::isConvertFromVisualCollisionModelId(activeModelId.toStdString());
            view.variants.prepend(std::move(item));
        }
    }

    if(view.variants.isEmpty()) {
        CollisionLinkModelVariantItemView item;
        item.label = "No collision model variants";
        item.detail = "This link has no collision model variant in the current runtime summary.";
        item.enabled = false;
        view.variants.push_back(std::move(item));
    } else {
        std::stable_sort(
            view.variants.begin(),
            view.variants.end(),
            [](const CollisionLinkModelVariantItemView& lhs,
               const CollisionLinkModelVariantItemView& rhs) {
                return variantDisplayRank(lhs) < variantDisplayRank(rhs);
            });
        int targetIndex = -1;
        int currentIndex = -1;
        int visibleIndex = -1;
        int activeIndex = -1;
        for(int i = 0; i < view.variants.size(); ++i) {
            const CollisionLinkModelVariantItemView& item = view.variants[i];
            if(item.variantId == previousVariantId) {
                targetIndex = i;
            }
            if(!activeModelId.isEmpty() &&
                (item.variantId == activeModelId ||
                    (simulation_project::isConvertFromVisualCollisionModelId(activeModelId.toStdString()) &&
                        item.source == simulation_project::kConvertFromVisualCollisionSource))) {
                currentIndex = i;
            }
            if(!visibleVariantId.isEmpty() && item.variantId == visibleVariantId) {
                visibleIndex = i;
            }
            if(activeIndex < 0 &&
                item.role == activeRole &&
                (activeSource.isEmpty() || item.source == activeSource)) {
                activeIndex = i;
            }
        }
        if(currentIndex < 0 && activeModelId.isEmpty()) {
            for(int i = 0; i < view.variants.size(); ++i) {
                if(isProjectDefinedCandidate(view.variants[i])) {
                    currentIndex = i;
                    break;
                }
            }
            for(int i = 0; i < view.variants.size(); ++i) {
                if(currentIndex < 0 && isPreferredOriginalVariant(view.variants[i])) {
                    currentIndex = i;
                    break;
                }
            }
            if(currentIndex < 0) {
                currentIndex = 0;
            }
            view.variants[currentIndex].current = true;
        }
        if(targetIndex < 0) {
            targetIndex = currentIndex >= 0
                ? currentIndex
                : (visibleIndex >= 0 ? visibleIndex : (activeIndex >= 0 ? activeIndex : 0));
        }
        if(targetIndex >= 0 && targetIndex < view.variants.size()) {
            view.variants[targetIndex].selected = true;
        }
    }

    return view;
}

CollisionLinkModelsSummaryView CollisionLinkModelsController::buildSummary(
    const simulation_project::ProjectDocument& document,
    const QString& selectedRobotId,
    const QString& selectedLinkName,
    const QString& selectedObjectId,
    const QString& selectedAttachmentId,
    const QString& activeDetectorId,
    const QString& qualityMessage,
    const robot_qt_viewer::CollisionRuntimeRobotSummary* robotSummary,
    const std::vector<robot_qt_viewer::CollisionRuntimeDetectorInfo>& runtimeDetectors)
{
    CollisionLinkModelsSummaryView view;
    if(!selectedObjectId.isEmpty() || !selectedAttachmentId.isEmpty()) {
        return buildObjectSummary(document, selectedObjectId, selectedAttachmentId, qualityMessage);
    }

    if(selectedRobotId.isEmpty() || selectedLinkName.isEmpty() || robotSummary == nullptr) {
        view.target.entityKind = "Link";
        view.target.displayName = "Select a robot link";
        view.originalSource.label = "No source";
        view.originalSource.detail = "Select a robot link to configure its collision model.";
        view.statusText = "No configurable link is selected.";
        return view;
    }

    const simulation_project::CollisionDetectorDesc* activeDetector =
        findDetector(document, activeDetectorId);
    const QString activeRole = activeDetector != nullptr
        ? QString::fromStdString(activeDetector->geometryRole)
        : QString();
    const QString activeSource = activeSourceForDetector(activeDetector);
    const CollisionRuntimeLinkSummary* linkSummary =
        findLinkSummary(robotSummary, selectedLinkName.toStdString());
    const QString activeModelId =
        activeLinkModelId(document, selectedRobotId.toStdString(), selectedLinkName.toStdString());

    view.target.valid = linkSummary != nullptr;
    view.target.entityKind = "Link";
    view.target.displayName = targetDisplayName(selectedRobotId, selectedLinkName);
    view.target.stableId = QString("%1:%2").arg(selectedRobotId, selectedLinkName);
    view.originalSource.label = importedOrVisualSourceLabel(linkSummary);
    view.originalSource.detail = importedOrVisualSourceDetail(linkSummary);
    view.originalSource.sourceKind = linkSummary != nullptr &&
            (linkSummary->originalCollisionCount > 0 || linkSummary->hasOriginalCollision)
        ? QString("imported")
        : QString("visual");

    if(linkSummary != nullptr) {
        const CollisionRuntimeModelVariantSummary* currentVariant = nullptr;
        const CollisionRuntimeModelVariantSummary* shownVariant = nullptr;
        const CollisionRuntimeModelVariantSummary* detectorVariant = nullptr;
        for(const CollisionRuntimeModelVariantSummary& variant : linkSummary->variants) {
            if(!activeModelId.isEmpty() &&
                variant.variantId == activeModelId.toStdString() &&
                currentVariant == nullptr) {
                currentVariant = &variant;
            }
            if(variant.visibleInViewport && shownVariant == nullptr) {
                shownVariant = &variant;
            }
            if(!activeRole.isEmpty() &&
                variant.role == activeRole.toStdString() &&
                (activeSource.isEmpty() || variant.source == activeSource.toStdString()) &&
                detectorVariant == nullptr) {
                detectorVariant = &variant;
            }
        }

        QStringList status;
        status << QString("variants %1").arg(static_cast<qulonglong>(linkSummary->variants.size()));
        status << QString("current %1")
            .arg(currentVariant != nullptr
                ? QString::fromStdString(currentVariant->label)
                : (activeModelId.isEmpty() ? QString("default") : QString("missing")));
        status << QString("preview %1")
            .arg(shownVariant != nullptr ? QString::fromStdString(shownVariant->label) : QString("none"));
        if(detectorVariant != nullptr) {
            status << QString("detector %1").arg(QString::fromStdString(detectorVariant->label));
        }
        if(!qualityMessage.isEmpty()) {
            status << QString("quality warning");
        }
        view.statusText = status.join(" | ");
    } else {
        view.statusText = "Selected link is not present in the collision runtime summary.";
    }

    const simulation_project::RobotCollisionOverrideDesc* collisionOverride =
        findCollisionOverride(document, selectedRobotId.toStdString());
    view.replaceOriginal = collisionOverride != nullptr && collisionOverride->replaceOriginalCollisions;
    return view;
}
