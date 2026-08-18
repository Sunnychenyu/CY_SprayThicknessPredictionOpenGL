#pragma once

#include <QString>

#include <string>

namespace simulation_project
{
    struct AttachmentAssetDesc;
    struct MountedAttachmentDesc;
    struct ProjectDocument;
}

struct ToolAttachmentCommandResult
{
    bool success = false;
    bool projectChanged = false;
    QString message;
};

class ToolAttachmentCommandController
{
public:
    static ToolAttachmentCommandResult applyMountedAttachmentUpdate(
        simulation_project::ProjectDocument& document,
        const simulation_project::MountedAttachmentDesc& attachment);

    static ToolAttachmentCommandResult applyAttachmentAssetUpdate(
        simulation_project::ProjectDocument& document,
        const std::string& assetId,
        const simulation_project::AttachmentAssetDesc& asset);
};
