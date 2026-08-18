#pragma once

#include "ToolSetupViewModel.h"

#include <QString>

namespace simulation_project
{
    struct ProjectDocument;
}

ToolSetupPanelView buildToolSetupPanelView(
    const simulation_project::ProjectDocument& document,
    const QString& selectedRobotId,
    const QString& selectedLinkName,
    const QString& preferredMountId,
    const QString& previousMountId,
    const QString& preferredAttachmentId,
    const QString& previousAttachmentId,
    const QString& preferredAssetId,
    const QString& previousAssetId);

