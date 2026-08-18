#include "CollisionResultsController.h"

#include <algorithm>

namespace
{
    using CollisionRuntimeDetectorInfo = robot_qt_viewer::CollisionRuntimeDetectorInfo;

    const simulation_project::CollisionDetectorDesc* findCollisionDetector(
        const simulation_project::ProjectDocument& document,
        const std::string& detectorId)
    {
        auto it = std::find_if(
            document.collision.detectors.begin(),
            document.collision.detectors.end(),
            [&](const simulation_project::CollisionDetectorDesc& detector) {
                return detector.id == detectorId;
            });
        return it == document.collision.detectors.end() ? nullptr : &(*it);
    }

    QString vec3Text(const CollisionRuntimeDetectorInfo::Vec3Info& value)
    {
        return QString("(%1, %2, %3)")
            .arg(value.x, 0, 'g', 5)
            .arg(value.y, 0, 'g', 5)
            .arg(value.z, 0, 'g', 5);
    }

    CollisionResultsTableRow messageRow(const QString& message)
    {
        CollisionResultsTableRow row;
        row.cells << message;
        row.spanColumns = true;
        return row;
    }

    QString pairSourcesText(const simulation_project::CollisionDetectorDesc& detector)
    {
        if(detector.pairGenerators.empty()) {
            return "legacy targets";
        }

        QString pairSources;
        for(const simulation_project::CollisionPairGeneratorDesc& generator : detector.pairGenerators) {
            if(!pairSources.isEmpty()) {
                pairSources += "\n";
            }
            if(generator.type == "LinkLink") {
                pairSources += QString("LinkLink: %1.%2 <-> %3.%4")
                    .arg(QString::fromStdString(generator.robotA))
                    .arg(QString::fromStdString(generator.linkA))
                    .arg(QString::fromStdString(generator.robotB))
                    .arg(QString::fromStdString(generator.linkB));
            } else if(generator.type == "SceneAll") {
                pairSources += "SceneAll Debug: all collision primitives";
            } else if(generator.type == "RobotRobot") {
                pairSources += QString("RobotRobot: %1 <-> %2")
                    .arg(QString::fromStdString(generator.robotA))
                    .arg(QString::fromStdString(generator.robotB));
            } else if(generator.type == "RobotObject") {
                pairSources += QString("RobotObject: %1 <-> %2")
                    .arg(QString::fromStdString(generator.robotId))
                    .arg(QString::fromStdString(generator.objectId));
            } else {
                pairSources += QString::fromStdString(generator.type);
            }
        }
        return pairSources;
    }

    const CollisionRuntimeDetectorInfo* runtimeInfoForDetector(
        const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors,
        const std::string& detectorId)
    {
        const auto runtimeIt = std::find_if(
            runtimeDetectors.begin(),
            runtimeDetectors.end(),
            [&](const CollisionRuntimeDetectorInfo& runtimeDetector) {
                return runtimeDetector.id == detectorId;
            });
        return runtimeIt == runtimeDetectors.end() ? nullptr : &(*runtimeIt);
    }

    QString runtimeIdsText(const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors)
    {
        QStringList runtimeIds;
        for(const CollisionRuntimeDetectorInfo& runtimeDetector : runtimeDetectors) {
            runtimeIds << QString::fromStdString(runtimeDetector.id);
        }
        return runtimeIds.isEmpty() ? "none" : runtimeIds.join(", ");
    }

    QString activeRuntimeDetectorId(const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors)
    {
        for(const CollisionRuntimeDetectorInfo& runtimeDetector : runtimeDetectors) {
            if(runtimeDetector.active) {
                return QString::fromStdString(runtimeDetector.id);
            }
        }
        return "none";
    }

    QString runtimeStateText(const CollisionRuntimeDetectorInfo* runtimeInfo)
    {
        if(runtimeInfo == nullptr) {
            return "no runtime result";
        }
        if(!runtimeInfo->hasResult) {
            return "not evaluated";
        }
        if(runtimeInfo->inCollision) {
            return QString("Collision (%1 contacts)")
                .arg(static_cast<qulonglong>(runtimeInfo->contactCount));
        }
        return QString("Clear (%1 contacts)")
            .arg(static_cast<qulonglong>(runtimeInfo->contactCount));
    }

    QString msText(double value)
    {
        return QString("%1 ms").arg(value, 8, 'f', 4);
    }

    QString distanceTimingText(const CollisionRuntimeDetectorInfo& runtimeInfo)
    {
        if(runtimeInfo.lastDistanceMs > 0.0) {
            return msText(runtimeInfo.lastDistanceMs);
        }
        if(runtimeInfo.nearest.valid) {
            return "cached";
        }
        if(runtimeInfo.nearest.state == "Disabled") {
            return "disabled";
        }
        return "not run";
    }

    QString percentText(double value, double total)
    {
        if(value <= 0.0 || total <= 0.0) {
            return "-";
        }
        const double percent = 100.0 * value / total;
        return QString("%1%").arg(percent, 5, 'f', 1);
    }

    QString queryShareText(double value, double total)
    {
        return percentText(value, total);
    }

    double frameTimingTotal(const CollisionRuntimeDetectorInfo& runtimeInfo)
    {
        return runtimeInfo.lastCheckMs +
            runtimeInfo.lastDistanceMs +
            runtimeInfo.frameRobotPoseMs +
            runtimeInfo.frameCollisionWorldUpdateMs +
            runtimeInfo.frameOverlayMs;
    }

    QString distanceTimingState(
        const CollisionRuntimeDetectorInfo& runtimeInfo,
        const simulation_project::CollisionDetectorDesc* detector)
    {
        if(detector != nullptr && !detector->nearestPoints && !detector->distance) {
            return "disabled";
        }
        if(runtimeInfo.lastDistanceMs > 0.0) {
            return runtimeInfo.nearest.valid ? "nearest returned" : "distance query";
        }
        if(runtimeInfo.nearest.valid) {
            return "cached";
        }
        if(!runtimeInfo.nearest.state.empty()) {
            return QString::fromStdString(runtimeInfo.nearest.state);
        }
        return "not run";
    }

    CollisionTimingTableRow makeTimingRow(
        const QString& stage,
        const QString& last,
        const QString& queryShare,
        const QString& frameShare,
        const QString& state,
        const QString& tooltip)
    {
        CollisionTimingTableRow row;
        row.stage = stage;
        row.last = last;
        row.queryShare = queryShare;
        row.frameShare = frameShare;
        row.state = state;
        row.tooltip = tooltip;
        return row;
    }

    CollisionSummaryRow makeSummaryRow(
        const QString& metric,
        const QString& value,
        const QString& share,
        const QString& tooltip)
    {
        CollisionSummaryRow row;
        row.metric = metric;
        row.value = value;
        row.share = share;
        row.tooltip = tooltip;
        return row;
    }

    void appendUnavailableSummaryRows(CollisionResultsViewModel& viewModel)
    {
        viewModel.summaryRows << makeSummaryRow(
            "Total Time",
            "-",
            "-",
            "Total time spent by the active collision detector query in the latest frame.");
        viewModel.summaryRows << makeSummaryRow(
            "Check Time",
            "-",
            "-",
            "Collision overlap/contact query time in the latest frame.");
        viewModel.summaryRows << makeSummaryRow(
            "Distance",
            "-",
            "-",
            "Distance or nearest-point query time when enabled.");
        viewModel.summaryRows << makeSummaryRow(
            "Overhead Time",
            "-",
            "-",
            "Viewport debug visualization cost. This is display cost, not collision query cost.");
    }

    CollisionOverlayTimingRow makeOverlayTimingRow(
        const QString& step,
        const QString& last,
        const QString& tooltip)
    {
        CollisionOverlayTimingRow row;
        row.step = step;
        row.last = last;
        row.tooltip = tooltip;
        return row;
    }

    QString overlayStateText(const CollisionRuntimeDetectorInfo& runtimeInfo)
    {
        return QString("geometry %1, contacts %2, nearest %3, primitives %4, lines %5")
            .arg(static_cast<qulonglong>(runtimeInfo.frameOverlayGeometryCount))
            .arg(static_cast<qulonglong>(runtimeInfo.frameOverlayContactCount))
            .arg(static_cast<qulonglong>(runtimeInfo.frameOverlayNearestCount))
            .arg(static_cast<qulonglong>(runtimeInfo.frameOverlayPrimitiveEstimate))
            .arg(static_cast<qulonglong>(runtimeInfo.frameOverlayLineEstimate));
    }

    void buildTimingViews(
        CollisionResultsViewModel& viewModel,
        const CollisionRuntimeDetectorInfo* runtimeInfo,
        const simulation_project::CollisionDetectorDesc* detector)
    {
        if(runtimeInfo == nullptr) {
            appendUnavailableSummaryRows(viewModel);
            return;
        }

        const double frameTotal = frameTimingTotal(*runtimeInfo);
        viewModel.summaryRows << makeSummaryRow(
            "Total Time",
            msText(runtimeInfo->lastQueryMs),
            percentText(runtimeInfo->lastQueryMs, frameTotal),
            "Total time spent by the active collision detector query in the latest frame.");
        viewModel.summaryRows << makeSummaryRow(
            "Check Time",
            msText(runtimeInfo->lastCheckMs),
            percentText(runtimeInfo->lastCheckMs, frameTotal),
            "Collision overlap/contact query time in the latest frame.");
        viewModel.summaryRows << makeSummaryRow(
            "Distance",
            distanceTimingText(*runtimeInfo),
            percentText(runtimeInfo->lastDistanceMs, frameTotal),
            "Distance or nearest-point query time when enabled.");
        viewModel.summaryRows << makeSummaryRow(
            "Overhead Time",
            msText(runtimeInfo->frameOverlayMs),
            percentText(runtimeInfo->frameOverlayMs, frameTotal),
            "Viewport debug visualization cost. This is display cost, not collision query cost.");

        viewModel.timingRows << makeTimingRow(
            "Collision Check",
            msText(runtimeInfo->lastCheckMs),
            queryShareText(runtimeInfo->lastCheckMs, runtimeInfo->lastQueryMs),
            percentText(runtimeInfo->lastCheckMs, frameTotal),
            detector != nullptr && detector->contacts ? "contacts on" : "contacts off",
            "Runs collision overlap/contact query for the active detector. Contact and normal extraction are currently included in this timing.");

        viewModel.timingRows << makeTimingRow(
            "Distance / Nearest",
            distanceTimingText(*runtimeInfo),
            queryShareText(runtimeInfo->lastDistanceMs, runtimeInfo->lastQueryMs),
            percentText(runtimeInfo->lastDistanceMs, frameTotal),
            distanceTimingState(*runtimeInfo, detector),
            "Runs distance or nearest-point query when enabled. It may be skipped when nearest/distance is disabled or already represented by the current collision result.");

        viewModel.timingRows << makeTimingRow(
            "Robot Pose Update",
            msText(runtimeInfo->frameRobotPoseMs),
            "-",
            percentText(runtimeInfo->frameRobotPoseMs, frameTotal),
            "active",
            "Updates robot joint/link transforms for this frame before collision objects are synchronized.");

        viewModel.timingRows << makeTimingRow(
            "Collision World Update",
            msText(runtimeInfo->frameCollisionWorldUpdateMs),
            "-",
            percentText(runtimeInfo->frameCollisionWorldUpdateMs, frameTotal),
            "active",
            "Synchronizes collision object transforms from the latest robot/object/tool poses before querying.");

        viewModel.timingRows << makeTimingRow(
            "Viewport Overlay",
            msText(runtimeInfo->frameOverlayMs),
            "-",
            percentText(runtimeInfo->frameOverlayMs, frameTotal),
            overlayStateText(*runtimeInfo),
            "Builds and submits debug visualization such as collision geometry, contacts, nearest points, and highlights. This is display cost, not collision query cost.");

        viewModel.overlayTimingRows << makeOverlayTimingRow(
            "Highlight",
            msText(runtimeInfo->frameOverlayHighlightMs),
            "Applies temporary visual highlights for collision pairs and colliding objects.");
        viewModel.overlayTimingRows << makeOverlayTimingRow(
            "Build Debug Draw",
            msText(runtimeInfo->frameOverlayDebugBuildMs),
            "Builds debug geometry for collision shapes, contacts, and nearest point markers.");
        viewModel.overlayTimingRows << makeOverlayTimingRow(
            "Variant Filter",
            msText(runtimeInfo->frameOverlayVariantFilterMs),
            "Filters debug geometry to the collision model variant currently shown in the viewport.");
        viewModel.overlayTimingRows << makeOverlayTimingRow(
            "Submit Draw",
            msText(runtimeInfo->frameOverlayDebugSubmitMs),
            "Submits prepared debug primitives and lines to the viewport render queues.");
        viewModel.overlayTimingRows << makeOverlayTimingRow(
            "Aux Frames",
            msText(runtimeInfo->frameOverlayAuxFramesMs),
            "Draws auxiliary frame markers and other small overlay helpers.");
    }

    void buildTableViews(
        CollisionResultsViewModel& viewModel,
        const QString& detectorId,
        const simulation_project::CollisionDetectorDesc* detector,
        bool viewportAvailable,
        const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors)
    {
        viewModel.contacts.normalColumnVisible =
            detector == nullptr || detector->visualization.showNormals;

        if(detectorId.isEmpty() || !viewportAvailable) {
            const QString message = detectorId.isEmpty()
                ? "No detector selected."
                : "Viewport is not available.";
            viewModel.contacts.rows << messageRow(message);
            viewModel.nearest.rows << messageRow(message);
            return;
        }

        const CollisionRuntimeDetectorInfo* runtimeInfo =
            runtimeInfoForDetector(runtimeDetectors, detectorId.toStdString());
        if(runtimeInfo == nullptr) {
            const QString message = QString("Runtime detector missing for %1. Runtime detectors: %2")
                .arg(detectorId)
                .arg(runtimeIdsText(runtimeDetectors));
            viewModel.contacts.rows << messageRow(message);
            viewModel.nearest.rows << messageRow(message);
            return;
        }

        if(!runtimeInfo->hasResult) {
            QString message = QString("Runtime detector exists but has no query result yet: %1").arg(detectorId);
            if(detector != nullptr && !detector->enabled) {
                message = QString("Detector is disabled: %1").arg(detectorId);
            } else if(runtimeInfo->effectiveIncludePairCount == 0 && runtimeInfo->includePairCount > 0) {
                message = QString("Runtime detector has zero effective pairs after role/filter checks: %1").arg(detectorId);
            }
            viewModel.contacts.rows << messageRow(message);
            viewModel.nearest.rows << messageRow(message);
            return;
        }

        if(detector != nullptr && !detector->visualization.showContacts) {
            viewModel.contacts.rows << messageRow("Contact display is disabled for this detector.");
        } else if(runtimeInfo->contacts.empty()) {
            viewModel.contacts.rows << messageRow("No contacts reported.");
        } else {
            for(const CollisionRuntimeDetectorInfo::ContactInfo& contact : runtimeInfo->contacts) {
                CollisionResultsTableRow row;
                row.cells << QString::fromStdString(contact.bodyA)
                          << QString::fromStdString(contact.bodyB)
                          << vec3Text(contact.position)
                          << vec3Text(contact.normal)
                          << QString::number(contact.penetrationDepth, 'g', 5);
                viewModel.contacts.rows << row;
            }
        }

        if(detector != nullptr && !detector->nearestPoints && !detector->distance) {
            viewModel.nearest.rows << messageRow("Nearest points are disabled for this detector.");
        } else if(runtimeInfo->nearest.valid) {
            CollisionResultsTableRow row;
            row.cells << QString::fromStdString(runtimeInfo->nearest.bodyA)
                      << QString::fromStdString(runtimeInfo->nearest.bodyB)
                      << vec3Text(runtimeInfo->nearest.pointA)
                      << vec3Text(runtimeInfo->nearest.pointB)
                      << QString::number(runtimeInfo->nearest.distance, 'g', 5);
            viewModel.nearest.rows << row;
        } else {
            QString message = QString("No nearest points reported. state: %1")
                .arg(QString::fromStdString(runtimeInfo->nearest.state));
            if(!runtimeInfo->nearest.reason.empty()) {
                message += QString(" (%1)").arg(QString::fromStdString(runtimeInfo->nearest.reason));
            }
            viewModel.nearest.rows << messageRow(message);
        }
    }
}

CollisionResultsViewModel CollisionResultsController::buildViewModel(
    const simulation_project::ProjectDocument& document,
    const QString& detectorId,
    bool viewportAvailable,
    const std::vector<robot_qt_viewer::CollisionRuntimeDetectorInfo>& runtimeDetectors,
    const QString& markedPairRobotA,
    const QString& markedPairLinkA)
{
    CollisionResultsViewModel viewModel;

    if(detectorId.isEmpty()) {
        viewModel.detailsText = "No collision detector selected";
        appendUnavailableSummaryRows(viewModel);
        buildTableViews(viewModel, detectorId, nullptr, viewportAvailable, runtimeDetectors);
        return viewModel;
    }

    const simulation_project::CollisionDetectorDesc* detector =
        findCollisionDetector(document, detectorId.toStdString());
    if(detector == nullptr) {
        viewModel.detailsText = "Selected collision detector is not in project data";
        appendUnavailableSummaryRows(viewModel);
        viewModel.contacts.rows << messageRow("Selected collision detector is not in project data.");
        viewModel.nearest.rows << messageRow("Selected collision detector is not in project data.");
        return viewModel;
    }

    int robotTargets = 0;
    int objectTargets = 0;
    int groupTargets = 0;
    for(const simulation_project::CollisionDetectorTargetDesc& target : detector->targets) {
        if(!target.robotId.empty()) {
            ++robotTargets;
        } else if(!target.objectId.empty()) {
            ++objectTargets;
        } else if(!target.objectGroupId.empty()) {
            ++groupTargets;
        }
    }

    const CollisionRuntimeDetectorInfo* runtimeInfo =
        runtimeInfoForDetector(runtimeDetectors, detector->id);
    const QString runtimeState = runtimeStateText(runtimeInfo);
    const bool sceneAll = detector->type == "SceneAll";
    const QString queryScope = sceneAll
        ? "all scene collision objects"
        : "selected collision pairs";
    const QString resolvedPairs = sceneAll
        ? "all pairs are resolved by the collision broad phase"
        : runtimeInfo != nullptr
            ? QString::number(static_cast<qulonglong>(runtimeInfo->includePairCount))
            : "not available";
    const QString effectivePairs = sceneAll
        ? "all pairs are resolved by the collision broad phase"
        : runtimeInfo != nullptr
            ? QString::number(static_cast<qulonglong>(runtimeInfo->effectiveIncludePairCount))
            : "not available";
    const QString queryPolicy = detector->queryPolicy.empty()
        ? "Legacy Pair Generators"
        : QString::fromStdString(detector->queryPolicy);
    const QString selectionSetA = detector->selectionSetAId.empty()
        ? "none"
        : QString::fromStdString(detector->selectionSetAId);
    const QString selectionSetB = detector->selectionSetBId.empty()
        ? "none"
        : QString::fromStdString(detector->selectionSetBId);
    const bool globalCollisionGeometry =
        document.collision.visualization.showCollisionGeometry;
    const QString contactsReturned = runtimeInfo != nullptr && runtimeInfo->hasResult
        ? QString::number(static_cast<qulonglong>(runtimeInfo->contactCount))
        : "not available";
    const QString nearestReturned = runtimeInfo != nullptr && runtimeInfo->hasResult
        ? (runtimeInfo->nearest.valid ? "true" : "false")
        : "not available";
    QString nearestState = "not available";
    QString nearestStateShort = "not available";
    if(runtimeInfo != nullptr && runtimeInfo->hasResult) {
        nearestStateShort = QString::fromStdString(runtimeInfo->nearest.state);
        nearestState = nearestStateShort;
        if(!runtimeInfo->nearest.reason.empty()) {
            nearestState += QString(": %1").arg(QString::fromStdString(runtimeInfo->nearest.reason));
        }
    }
    const QString resultSummary = runtimeInfo != nullptr && runtimeInfo->hasResult
        ? QString("contacts=%1, nearest=%2")
              .arg(static_cast<qulonglong>(runtimeInfo->contactCount))
              .arg(runtimeInfo->nearest.valid ? "true" : "false")
        : "not evaluated";

    QString runtimeWarning;
    if(!viewportAvailable) {
        runtimeWarning = "viewport is not available";
    } else if(runtimeInfo == nullptr) {
        runtimeWarning = QString("runtime detector missing; available: %1")
            .arg(runtimeIdsText(runtimeDetectors));
    } else if(!runtimeInfo->hasResult) {
        runtimeWarning = "runtime detector exists but has no query result yet";
    }

    QStringList resultLines;
    resultLines << QString("Status: %1").arg(runtimeState);
    if(runtimeInfo != nullptr && runtimeInfo->hasResult) {
        resultLines << QString("Collision: %1").arg(runtimeInfo->inCollision ? "Yes" : "No");
        resultLines << QString("Contacts: %1").arg(static_cast<qulonglong>(runtimeInfo->contactCount));
        if(detector->nearestPoints || detector->distance) {
            if(runtimeInfo->nearest.valid) {
                resultLines << QString("Distance: %1 m").arg(runtimeInfo->nearest.distance, 0, 'g', 5);
                resultLines << QString("Nearest pair: %1 <-> %2")
                    .arg(QString::fromStdString(runtimeInfo->nearest.bodyA))
                    .arg(QString::fromStdString(runtimeInfo->nearest.bodyB));
            } else {
                QString nearestLine = QString("Distance: %1")
                    .arg(QString::fromStdString(runtimeInfo->nearest.state));
                if(!runtimeInfo->nearest.reason.empty()) {
                    nearestLine += QString(" (%1)").arg(QString::fromStdString(runtimeInfo->nearest.reason));
                }
                resultLines << nearestLine;
            }
        } else {
            resultLines << "Distance: disabled";
        }
    }
    resultLines << QString("Pairs checked: %1").arg(effectivePairs);
    resultLines << QString("Scope: %1").arg(queryScope);
    if(!runtimeWarning.isEmpty()) {
        resultLines << QString("Runtime warning: %1").arg(runtimeWarning);
    }

    buildTimingViews(viewModel, runtimeInfo, detector);

    QStringList debugLines;
    if(runtimeInfo != nullptr) {
        if(runtimeInfo->frameOverlayMs > 16.0) {
            debugLines << "Performance: viewport overlay is above 16 ms; hide Collision Geometry for smoother motion";
        }
        debugLines << QString("Overlay load: detectors %1, geometry %2, contacts %3, nearest %4, primitives %5, lines %6")
            .arg(static_cast<qulonglong>(runtimeInfo->frameOverlayDetectorCount))
            .arg(static_cast<qulonglong>(runtimeInfo->frameOverlayGeometryCount))
            .arg(static_cast<qulonglong>(runtimeInfo->frameOverlayContactCount))
            .arg(static_cast<qulonglong>(runtimeInfo->frameOverlayNearestCount))
            .arg(static_cast<qulonglong>(runtimeInfo->frameOverlayPrimitiveEstimate))
            .arg(static_cast<qulonglong>(runtimeInfo->frameOverlayLineEstimate));
    }
    debugLines << QString("Nearest state: %1").arg(nearestStateShort);
    debugLines << QString("Resolved pairs: %1").arg(resolvedPairs);
    debugLines << QString("Effective pairs: %1").arg(effectivePairs);
    if(!runtimeWarning.isEmpty()) {
        debugLines << QString("Runtime warning: %1").arg(runtimeWarning);
    }
    debugLines << QString("active runtime detector: %1").arg(activeRuntimeDetectorId(runtimeDetectors));
    debugLines << QString("id: %1").arg(QString::fromStdString(detector->id));
    debugLines << QString("name: %1").arg(QString::fromStdString(detector->name.empty() ? detector->id : detector->name));
    debugLines << "lookup key: id";
    debugLines << "name use: display/search only";
    debugLines << QString("type: %1").arg(QString::fromStdString(detector->type));
    debugLines << QString("query policy: %1").arg(queryPolicy);
    debugLines << QString("selection set A: %1").arg(selectionSetA);
    debugLines << QString("selection set B: %1").arg(selectionSetB);
    debugLines << QString("enabled: %1").arg(detector->enabled ? "true" : "false");
    debugLines << QString("detector overlay: %1").arg(detector->visualization.visible ? "on" : "off");
    debugLines << QString("detector geometry: %1").arg(detector->visualization.showCollisionGeometry ? "on" : "off");
    debugLines << QString("global collision geometry: %1").arg(globalCollisionGeometry ? "on" : "off");
    debugLines << QString("contacts display: %1").arg(detector->visualization.showContacts ? "on" : "off");
    debugLines << QString("contact query: %1").arg(detector->contacts ? "true" : "false");
    debugLines << QString("contacts returned: %1").arg(contactsReturned);
    debugLines << QString("nearest requested: %1").arg((detector->nearestPoints || detector->distance) ? "true" : "false");
    debugLines << QString("nearest returned: %1").arg(nearestReturned);
    debugLines << QString("nearest state: %1").arg(nearestState);
    debugLines << QString("distance threshold: %1").arg(detector->distanceThreshold, 0, 'g', 5);
    debugLines << QString("state: %1").arg(runtimeState);
    debugLines << QString("role: %1").arg(QString::fromStdString(detector->geometryRole));
    debugLines << QString("source: %1").arg(QString::fromStdString(detector->geometrySource.empty() ? std::string("any") : detector->geometrySource));
    debugLines << QString("role policy: robot %1, environment own role").arg(QString::fromStdString(detector->geometryRole));
    debugLines << QString("result summary: %1").arg(resultSummary);
    debugLines << QString("max contacts: %1").arg(detector->maxContacts);
    debugLines << QString("targets: %1 robots, %2 objects, %3 groups").arg(robotTargets).arg(objectTargets).arg(groupTargets);
    if(!sceneAll) {
        const QString pairA = markedPairRobotA.isEmpty()
            ? "not set"
            : QString("%1.%2").arg(markedPairRobotA, markedPairLinkA);
        debugLines << QString("marked link A shortcut: %1").arg(pairA);
        if(runtimeInfo != nullptr && runtimeInfo->includePairCount == 0) {
            debugLines << "0 pair hints:";
            debugLines << "  - robot/object has no collision geometry";
            debugLines << "  - selected link name does not exist";
            debugLines << "  - object collision is disabled";
            debugLines << "  - object group is empty";
            debugLines << "  - pair filters removed every pair";
        } else if(runtimeInfo != nullptr && runtimeInfo->includePairCount > 0 &&
                runtimeInfo->hasResult && runtimeInfo->contactCount == 0 && !runtimeInfo->nearest.valid) {
            debugLines << "empty result hints:";
            debugLines << "  - distance or nearest point output is disabled";
            debugLines << "  - all included pairs are outside the active distance query result";
            debugLines << "  - the active collision backend cannot compute this geometry pair";
        }
    }
    debugLines << "pair sources:";
    debugLines << pairSourcesText(*detector);

    viewModel.detailsText = resultLines.join('\n');
    viewModel.debugText = debugLines.join('\n');
    buildTableViews(viewModel, detectorId, detector, viewportAvailable, runtimeDetectors);
    return viewModel;
}
