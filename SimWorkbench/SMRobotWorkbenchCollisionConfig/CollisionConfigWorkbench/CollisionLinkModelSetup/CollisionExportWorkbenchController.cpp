#include "CollisionExportWorkbenchController.h"

#include "CollisionExportDocumentFacade.h"
#include "CollisionWorkbenchServices.h"

#include <utility>

namespace robot_qt_viewer
{
    CollisionExportWorkbenchController::CollisionExportWorkbenchController(
        CollisionWorkbenchServices& appServices,
        CollisionExportDocumentFacade& documentFacade,
        Callbacks callbacks)
        : m_appServices(appServices)
        , m_documentFacade(documentFacade)
        , m_callbacks(std::move(callbacks))
    {
    }

    void CollisionExportWorkbenchController::saveOverridesToProject()
    {
        if(m_documentFacade.projectRequiresSaveAs()) {
            showStatus("Project needs Save As before collision overrides can be saved.", 3000);
            return;
        }

        const ProjectSessionWorkflowResult result =
            m_documentFacade.saveProject(QStringLiteral("saveCollisionOverridesToProject"));
        showStatus(result.message, result.success ? 3000 : 5000);
    }

    bool CollisionExportWorkbenchController::selectedRobotHasCollisionOverrides() const
    {
        if(m_appServices.selectedRobotId().isEmpty()) {
            return false;
        }
        return m_documentFacade.robotHasCollisionOverrides(m_appServices.selectedRobotId());
    }

    std::filesystem::path CollisionExportWorkbenchController::selectedRobotSourcePath() const
    {
        if(m_appServices.selectedRobotId().isEmpty()) {
            return {};
        }
        return m_documentFacade.robotSourcePath(m_appServices.selectedRobotId());
    }

    void CollisionExportWorkbenchController::saveOverridesAsSidecar(
        const std::filesystem::path& sidecarPath,
        const std::string& portableSidecarPath)
    {
        if(m_appServices.selectedRobotId().isEmpty()) {
            showStatus("Select a robot first.", 3000);
            return;
        }

        if(!selectedRobotHasCollisionOverrides()) {
            showStatus("Selected robot has no collision overrides.", 3000);
            return;
        }

        const CollisionLinkModelsCommandResult result =
            m_documentFacade.saveSidecar(
                m_appServices.selectedRobotId(),
                sidecarPath,
                portableSidecarPath);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }

        showStatus(result.message, 5000);
    }

    void CollisionExportWorkbenchController::exportRobotUrdfWithCollision(
        const std::filesystem::path& sourceUrdfPath,
        const std::filesystem::path& outputUrdfPath)
    {
        if(m_appServices.selectedRobotId().isEmpty()) {
            showStatus("Select a robot first.", 3000);
            return;
        }

        if(!selectedRobotHasCollisionOverrides()) {
            showStatus("Selected robot has no collision overrides.", 3000);
            return;
        }

        const CollisionLinkModelsCommandResult result =
            m_documentFacade.exportUrdf(
                m_appServices.selectedRobotId(),
                sourceUrdfPath,
                outputUrdfPath);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }

        showStatus(result.message, 5000);
    }

    void CollisionExportWorkbenchController::showStatus(const QString& message, int timeoutMs) const
    {
        if(m_callbacks.statusMessage) {
            m_callbacks.statusMessage(message, timeoutMs);
        }
    }
}
