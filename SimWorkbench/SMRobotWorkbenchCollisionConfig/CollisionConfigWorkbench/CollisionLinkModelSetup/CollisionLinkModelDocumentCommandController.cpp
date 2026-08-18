#include "CollisionLinkModelDocumentCommandController.h"

#include "CollisionLinkModelDocumentFacade.h"

namespace
{
    CollisionLinkModelDocumentCommandResult makeFailure(const QString& message)
    {
        CollisionLinkModelDocumentCommandResult result;
        result.success = false;
        result.message = message;
        return result;
    }
}

CollisionLinkModelDocumentCommandResult CollisionLinkModelDocumentCommandController::addBoxElement(
    simulation_project::ProjectDocument& document,
    const QString& robotId,
    const QString& linkName)
{
    if(robotId.isEmpty() || linkName.isEmpty()) {
        return makeFailure("Select a robot link first.");
    }

    CollisionLinkModelDocumentFacade documentFacade(document);
    documentFacade.addBoxElement(robotId.toStdString(), linkName.toStdString());

    CollisionLinkModelDocumentCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.selectedSource = "viewer";
    result.selectedRole = "PlanningProxy";
    result.message = QString("Added box collision to %1.%2").arg(robotId, linkName);
    return result;
}

CollisionLinkModelDocumentCommandResult CollisionLinkModelDocumentCommandController::setReplaceOriginal(
    simulation_project::ProjectDocument& document,
    const QString& robotId,
    bool replaceOriginal)
{
    if(robotId.isEmpty()) {
        return makeFailure("Select a robot first.");
    }

    CollisionLinkModelDocumentFacade documentFacade(document);
    if(!documentFacade.setReplaceOriginal(robotId.toStdString(), replaceOriginal)) {
        CollisionLinkModelDocumentCommandResult result;
        result.success = true;
        result.projectChanged = false;
        result.message = QString("Replace original collisions: %1").arg(replaceOriginal ? "on" : "off");
        return result;
    }

    CollisionLinkModelDocumentCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.message = QString("Replace original collisions: %1").arg(replaceOriginal ? "on" : "off");
    return result;
}

CollisionLinkModelDocumentCommandResult CollisionLinkModelDocumentCommandController::removeElement(
    simulation_project::ProjectDocument& document,
    const QString& robotId,
    const QString& linkName,
    const QString& elementId)
{
    if(elementId.isEmpty()) {
        return makeFailure("No collision element selected.");
    }

    CollisionLinkModelDocumentFacade documentFacade(document);
    const bool removed = documentFacade.removeElement(
        robotId.toStdString(),
        linkName.toStdString(),
        elementId.toStdString());
    if(!removed) {
        return makeFailure("Selected collision element is not in the project.");
    }

    CollisionLinkModelDocumentCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.message = QString("Removed collision element %1").arg(elementId);
    return result;
}

bool CollisionLinkModelDocumentCommandController::hasCollisionOverrides(
    simulation_project::ProjectDocument& document,
    const QString& robotId)
{
    if(robotId.isEmpty()) {
        return false;
    }

    CollisionLinkModelDocumentFacade documentFacade(document);
    return documentFacade.hasOverrideElements(robotId.toStdString());
}
