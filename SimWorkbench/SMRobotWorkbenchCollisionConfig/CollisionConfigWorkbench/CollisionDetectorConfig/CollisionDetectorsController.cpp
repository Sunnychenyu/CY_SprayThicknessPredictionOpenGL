#include "CollisionDetectorsController.h"

#include <algorithm>
#include <QStringList>
#include <unordered_map>
#include <utility>

namespace
{
    QString detectorDisplayName(const simulation_project::CollisionDetectorDesc& detector)
    {
        return QString::fromStdString(detector.name.empty() ? detector.id : detector.name);
    }

    QString selectionSetDisplayName(const simulation_project::CollisionSelectionSetDesc& selectionSet)
    {
        return QString::fromStdString(selectionSet.name.empty() ? selectionSet.id : selectionSet.name);
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
}

QVector<CollisionDetectorListItemView> CollisionDetectorsController::buildDetectorListItems(
    const simulation_project::ProjectDocument& document,
    const std::vector<robot_qt_viewer::CollisionRuntimeDetectorInfo>& runtimeDetectors)
{
    std::unordered_map<std::string, robot_qt_viewer::CollisionRuntimeDetectorInfo> runtimeById;
    for(const robot_qt_viewer::CollisionRuntimeDetectorInfo& detector : runtimeDetectors) {
        runtimeById[detector.id] = detector;
    }

    QVector<CollisionDetectorListItemView> items;
    for(const simulation_project::CollisionDetectorDesc& detector : document.collision.detectors) {
        const auto runtimeIt = runtimeById.find(detector.id);
        const bool visible = detector.visualization.visible;
        const std::size_t includePairCount = runtimeIt != runtimeById.end() ? runtimeIt->second.includePairCount : 0;
        const std::size_t contactCount = runtimeIt != runtimeById.end() ? runtimeIt->second.contactCount : 0;
        const bool hasResult = runtimeIt != runtimeById.end() && runtimeIt->second.hasResult;
        const bool inCollision = runtimeIt != runtimeById.end() && runtimeIt->second.inCollision;

        const QString displayName = detectorDisplayName(detector);
        QString text = displayName;

        QStringList tooltipLines;
        tooltipLines << displayName;
        tooltipLines << QString("id: %1").arg(QString::fromStdString(detector.id));
        tooltipLines << QString("type: %1").arg(QString::fromStdString(detector.type));
        if(detector.type == "SceneAll") {
            tooltipLines << "scope: all";
        } else if(includePairCount == 0) {
            tooltipLines << "resolved pairs: 0";
        } else {
            tooltipLines << QString("resolved pairs: %1").arg(static_cast<qulonglong>(includePairCount));
        }
        if(hasResult) {
            tooltipLines << QString("contacts: %1").arg(static_cast<qulonglong>(contactCount));
            tooltipLines << QString("state: %1").arg(inCollision ? "collision" : "clear");
        } else {
            tooltipLines << "result: not available";
        }

        CollisionDetectorListItemView item;
        item.id = QString::fromStdString(detector.id);
        item.text = text;
        item.tooltip = tooltipLines.join('\n');
        item.checked = detector.enabled;
        item.selected = visible;
        items.push_back(std::move(item));
    }

    if(items.isEmpty()) {
        CollisionDetectorListItemView item;
        item.text = "No collision detectors";
        item.enabled = false;
        items.push_back(std::move(item));
    }

    return items;
}

CollisionDetectorPropertiesView CollisionDetectorsController::buildProperties(
    const simulation_project::ProjectDocument& document,
    const QString& detectorId)
{
    CollisionDetectorPropertiesView view;
    for(const simulation_project::CollisionSelectionSetDesc& selectionSet : document.collision.selectionSets) {
        CollisionDetectorSelectionSetOptionView option;
        option.id = QString::fromStdString(selectionSet.id);
        option.text = selectionSetDisplayName(selectionSet);
        view.selectionSets.push_back(std::move(option));
    }

    const simulation_project::CollisionDetectorDesc* detector = findDetector(document, detectorId);
    if(detector == nullptr) {
        return view;
    }

    view.hasDetector = true;
    view.id = QString::fromStdString(detector->id);
    view.name = QString::fromStdString(detector->name);
    view.enabled = detector->enabled;
    view.visible = detector->visualization.visible;
    view.geometry = detector->visualization.showCollisionGeometry;
    view.contacts = detector->contacts;
    view.normals = detector->visualization.showNormals;
    view.nearest = detector->nearestPoints || detector->distance;
    view.maxContacts = static_cast<double>(detector->maxContacts);
    view.distanceThreshold = detector->distanceThreshold;
    view.role = QString::fromStdString(detector->geometryRole);
    view.policy = detector->queryPolicy.empty()
        ? QString("Legacy")
        : QString::fromStdString(detector->queryPolicy);
    view.setA = QString::fromStdString(detector->selectionSetAId);
    view.setB = QString::fromStdString(detector->selectionSetBId);
    return view;
}
