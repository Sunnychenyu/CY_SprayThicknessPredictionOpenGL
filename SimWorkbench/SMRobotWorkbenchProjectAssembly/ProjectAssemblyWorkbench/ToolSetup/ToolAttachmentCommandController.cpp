#include "ToolAttachmentCommandController.h"

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectAttachmentCommands.h>

namespace
{
    ToolAttachmentCommandResult makeError(const QString& message)
    {
        ToolAttachmentCommandResult result;
        result.success = false;
        result.message = message;
        return result;
    }

    ToolAttachmentCommandResult fromProjectCommandResult(
        const simulation_project::ProjectCommandResult& commandResult)
    {
        if(!commandResult.success) {
            return makeError(QString::fromStdString(commandResult.message));
        }

        ToolAttachmentCommandResult result;
        result.success = true;
        result.projectChanged = commandResult.projectChanged;
        result.message = QString::fromStdString(commandResult.message);
        return result;
    }
}

ToolAttachmentCommandResult ToolAttachmentCommandController::applyMountedAttachmentUpdate(
    simulation_project::ProjectDocument& document,
    const simulation_project::MountedAttachmentDesc& attachment)
{
    simulation_project::ProjectAttachmentCommands commands(document);
    return fromProjectCommandResult(commands.applyMountedAttachmentUpdate(attachment));
}

ToolAttachmentCommandResult ToolAttachmentCommandController::applyAttachmentAssetUpdate(
    simulation_project::ProjectDocument& document,
    const std::string& assetId,
    const simulation_project::AttachmentAssetDesc& asset)
{
    simulation_project::ProjectAttachmentCommands commands(document);
    return fromProjectCommandResult(commands.applyAttachmentAssetUpdate(assetId, asset));
}
