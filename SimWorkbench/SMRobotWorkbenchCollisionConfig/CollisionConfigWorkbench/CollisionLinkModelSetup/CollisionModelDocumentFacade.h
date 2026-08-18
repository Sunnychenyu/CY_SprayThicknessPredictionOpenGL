#pragma once

#include "CollisionLinkModelsViewModel.h"
#include "CollisionRuntimeViewModel.h"

#include <QString>

#include <filesystem>
#include <vector>

namespace simulation_project
{
    struct ProjectDocument;
}

namespace robot_qt_viewer
{
    struct CollisionModelRuntimeContext
    {
        QString role;
        QString source;
    };

    class CollisionModelDocumentFacade
    {
    public:
        explicit CollisionModelDocumentFacade(const simulation_project::ProjectDocument& document);

        CollisionModelRuntimeContext runtimeContext(const QString& activeDetectorId) const;

        CollisionLinkModelsViewModel buildViewModel(
            const QString& selectedRobotId,
            const QString& selectedLinkName,
            const QString& selectedObjectId,
            const QString& selectedAttachmentId,
            const QString& activeDetectorId,
            const QString& previousVariantId,
            const QString& visibleVariantId,
            const QString& qualityMessage,
            const std::filesystem::path& projectBasePath,
            const CollisionRuntimeRobotSummary* robotSummary,
            const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors) const;

        CollisionLinkModelsSummaryView buildSummary(
            const QString& selectedRobotId,
            const QString& selectedLinkName,
            const QString& selectedObjectId,
            const QString& selectedAttachmentId,
            const QString& activeDetectorId,
            const QString& qualityMessage,
            const CollisionRuntimeRobotSummary* robotSummary,
            const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors) const;

    private:
        const simulation_project::ProjectDocument& m_document;
    };
}
