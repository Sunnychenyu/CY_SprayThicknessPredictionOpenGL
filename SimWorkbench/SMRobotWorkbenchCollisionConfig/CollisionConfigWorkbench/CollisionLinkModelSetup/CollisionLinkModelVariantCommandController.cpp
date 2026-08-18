#include "CollisionLinkModelVariantCommandController.h"

#include "CollisionLinkModelDocumentFacade.h"
#include "RobotQtViewerViewportServices.h"

#include <SimulationProject/CollisionModelSelectionIds.h>
#include <SimulationProject/ProjectDocumentService.h>

namespace
{
    CollisionLinkModelVariantCommandResult makeFailure(const QString& message)
    {
        CollisionLinkModelVariantCommandResult result;
        result.success = false;
        result.message = message;
        return result;
    }
}

CollisionLinkModelVariantCommandResult CollisionLinkModelVariantCommandController::showVariantOnly(
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& robotId,
    const QString& linkName,
    const QString& variantId)
{
    if(viewportServices == nullptr) {
        return makeFailure("Viewport is not available.");
    }
    if(robotId.isEmpty() || linkName.isEmpty() || variantId.isEmpty()) {
        return makeFailure("Select a collision model variant first.");
    }
    if(!viewportServices->setVisibleRobotCollisionVariant(robotId, linkName, variantId)) {
        return makeFailure("Failed to show selected collision model variant.");
    }

    CollisionLinkModelVariantCommandResult result;
    result.success = true;
    result.message = "Showing only the selected collision model variant.";
    return result;
}

CollisionLinkModelVariantCommandResult CollisionLinkModelVariantCommandController::useVariantInDetector(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& detectorId,
    const QString& role,
    const QString& source)
{
    if(role.isEmpty()) {
        return makeFailure("Selected collision model variant has no detector role.");
    }
    if(source.isEmpty()) {
        return makeFailure("Selected collision model variant has no source.");
    }
    if(detectorId.isEmpty()) {
        return makeFailure("Select an active collision detector first.");
    }

    CollisionLinkModelDocumentFacade documentFacade(document);
    simulation_project::CollisionDetectorDesc* detector =
        documentFacade.findDetector(detectorId.toStdString());
    if(detector == nullptr) {
        return makeFailure("Selected collision detector is not in the project.");
    }

    detector->geometryRole = role.toStdString();
    detector->geometrySource = source.toStdString();

    bool updatedRuntime = viewportServices == nullptr;
    if(viewportServices != nullptr) {
        updatedRuntime = viewportServices->updateCollisionDetectorRuntimeOptions(*detector);
        if(updatedRuntime) {
            viewportServices->setActiveCollisionDetector(detectorId);
        }
    }

    CollisionLinkModelVariantCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.runtimeUpdateFailed = viewportServices != nullptr && !updatedRuntime;
    result.message = QString("Active detector variant: %1 / %2").arg(role, source);
    return result;
}

CollisionLinkModelVariantCommandResult CollisionLinkModelVariantCommandController::setCurrentLinkModel(
    simulation_project::ProjectDocumentService& service,
    const QString& robotId,
    const QString& linkName,
    const QString& variantId)
{
    if(robotId.isEmpty() || linkName.isEmpty()) {
        return makeFailure("Select a robot link first.");
    }
    if(variantId.isEmpty()) {
        return makeFailure("Select a collision model variant first.");
    }

    bool changed = false;
    std::string error;
    if(!service.setActiveRobotLinkCollisionModel(
           robotId.toStdString(),
           linkName.toStdString(),
           variantId.toStdString(),
           &changed,
           &error)) {
        return makeFailure(error.empty()
            ? QString("Failed to set current collision model.")
            : QString::fromStdString(error));
    }

    CollisionLinkModelVariantCommandResult result;
    result.success = true;
    result.projectChanged = changed;
    result.message = changed
        ? QString("Current link collision model: %1").arg(variantId)
        : QString("Selected collision model is already current.");
    return result;
}

CollisionLinkModelVariantCommandResult CollisionLinkModelVariantCommandController::setCurrentObjectModel(
    simulation_project::ProjectDocumentService& service,
    const QString& objectId,
    const QString& variantId)
{
    if(objectId.isEmpty()) {
        return makeFailure("Select an object or attachment first.");
    }
    if(variantId.isEmpty()) {
        return makeFailure("Select a collision model variant first.");
    }

    bool changed = false;
    std::string error;
    if(!service.setActiveObjectCollisionModel(
           objectId.toStdString(),
           variantId.toStdString(),
           &changed,
           &error)) {
        return makeFailure(error.empty()
            ? QString("Failed to set current object collision model.")
            : QString::fromStdString(error));
    }

    CollisionLinkModelVariantCommandResult result;
    result.success = true;
    result.projectChanged = changed;
    result.message = changed
        ? QString("Current object collision model: %1").arg(variantId)
        : QString("Selected collision model is already current.");
    return result;
}
