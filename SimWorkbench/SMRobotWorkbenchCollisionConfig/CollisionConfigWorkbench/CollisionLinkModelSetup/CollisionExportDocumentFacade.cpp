#include "CollisionExportDocumentFacade.h"

#include "CollisionWorkbenchServices.h"
#include "CollisionLinkModelDocumentCommandController.h"

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectSession.h>

#include <algorithm>

namespace
{
    const simulation_project::RobotDesc* findRobotDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId)
    {
        const auto it = std::find_if(
            document.robots.begin(),
            document.robots.end(),
            [&](const simulation_project::RobotDesc& robot) {
                return robot.id == robotId;
            });
        return it == document.robots.end() ? nullptr : &(*it);
    }
}

namespace robot_qt_viewer
{
    CollisionExportDocumentFacade::CollisionExportDocumentFacade(
        const simulation_project::ProjectDocument& document,
        CollisionWorkbenchServices& appServices)
        : m_document(document)
        , m_appServices(appServices)
    {
    }

    bool CollisionExportDocumentFacade::projectRequiresSaveAs() const
    {
        return m_appServices.session().requiresSaveAs() || m_appServices.session().path().empty();
    }

    ProjectSessionWorkflowResult CollisionExportDocumentFacade::saveProject(const QString& sourceId)
    {
        return m_appServices.saveProject(m_appServices.session().path(), false, sourceId);
    }

    bool CollisionExportDocumentFacade::robotHasCollisionOverrides(const QString& robotId) const
    {
        if(robotId.isEmpty()) {
            return false;
        }
        const auto it = std::find_if(
            m_document.collision.robotOverrides.begin(),
            m_document.collision.robotOverrides.end(),
            [&](const simulation_project::RobotCollisionOverrideDesc& collisionOverride) {
                return collisionOverride.robotId == robotId.toStdString();
            });
        return it != m_document.collision.robotOverrides.end() && !it->elements.empty();
    }

    std::filesystem::path CollisionExportDocumentFacade::robotSourcePath(const QString& robotId) const
    {
        if(robotId.isEmpty()) {
            return {};
        }
        const simulation_project::RobotDesc* robot = findRobotDesc(m_document, robotId.toStdString());
        return robot != nullptr ? std::filesystem::path(robot->sourcePath) : std::filesystem::path();
    }

    CollisionLinkModelsCommandResult CollisionExportDocumentFacade::saveSidecar(
        const QString& robotId,
        const std::filesystem::path& sidecarPath,
        const std::string& portableSidecarPath)
    {
        CollisionLinkModelsCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionExport"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelsCommandController::saveSidecar(
                    service.document(),
                    m_appServices,
                    robotId,
                    sidecarPath,
                    portableSidecarPath);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionLinkModelsCommandResult CollisionExportDocumentFacade::exportUrdf(
        const QString& robotId,
        const std::filesystem::path& sourceUrdfPath,
        const std::filesystem::path& outputUrdfPath)
    {
        CollisionLinkModelsCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionExport"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelsCommandController::exportUrdf(
                    service.document(),
                    m_appServices,
                    robotId,
                    sourceUrdfPath,
                    outputUrdfPath);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }
}
