#pragma once

#include "CollisionLinkModelsCommandController.h"
#include "ProjectSessionWorkflowController.h"

#include <QString>

#include <filesystem>
#include <string>

namespace simulation_project
{
    struct ProjectDocument;
    class ProjectSession;
}

namespace robot_qt_viewer
{
    class CollisionWorkbenchServices;

    class CollisionExportDocumentFacade
    {
    public:
        CollisionExportDocumentFacade(
            const simulation_project::ProjectDocument& document,
            CollisionWorkbenchServices& appServices);

        bool projectRequiresSaveAs() const;
        ProjectSessionWorkflowResult saveProject(const QString& sourceId);
        bool robotHasCollisionOverrides(const QString& robotId) const;
        std::filesystem::path robotSourcePath(const QString& robotId) const;
        CollisionLinkModelsCommandResult saveSidecar(
            const QString& robotId,
            const std::filesystem::path& sidecarPath,
            const std::string& portableSidecarPath);
        CollisionLinkModelsCommandResult exportUrdf(
            const QString& robotId,
            const std::filesystem::path& sourceUrdfPath,
            const std::filesystem::path& outputUrdfPath);

    private:
        const simulation_project::ProjectDocument& m_document;
        CollisionWorkbenchServices& m_appServices;
    };
}
