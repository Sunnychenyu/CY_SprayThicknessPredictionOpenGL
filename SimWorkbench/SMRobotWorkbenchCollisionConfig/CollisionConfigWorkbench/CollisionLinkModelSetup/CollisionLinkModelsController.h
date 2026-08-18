#pragma once

#include "CollisionLinkModelsViewModel.h"

#include "CollisionRuntimeViewModel.h"
#include <SimulationProject/ProjectDocument.h>

#include <QString>

#include <filesystem>
#include <vector>

class CollisionLinkModelsController
{
public:
    static CollisionLinkModelsViewModel buildViewModel(
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
        const std::vector<robot_qt_viewer::CollisionRuntimeDetectorInfo>& runtimeDetectors);

    static CollisionLinkModelsSummaryView buildSummary(
        const simulation_project::ProjectDocument& document,
        const QString& selectedRobotId,
        const QString& selectedLinkName,
        const QString& selectedObjectId,
        const QString& selectedAttachmentId,
        const QString& activeDetectorId,
        const QString& qualityMessage,
        const robot_qt_viewer::CollisionRuntimeRobotSummary* robotSummary,
        const std::vector<robot_qt_viewer::CollisionRuntimeDetectorInfo>& runtimeDetectors);
};
