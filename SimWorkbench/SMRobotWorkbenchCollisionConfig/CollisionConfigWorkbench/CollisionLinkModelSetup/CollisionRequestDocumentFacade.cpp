#include "CollisionRequestDocumentFacade.h"

#include "CollisionWorkbenchServices.h"

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectSession.h>

namespace robot_qt_viewer
{
    CollisionRequestDocumentFacade::CollisionRequestDocumentFacade(
        const simulation_project::ProjectDocument& document,
        CollisionWorkbenchServices& appServices)
        : m_document(document)
        , m_appServices(appServices)
    {
    }

    CollisionLinkModelVariantCommandResult CollisionRequestDocumentFacade::useVariantInDetector(
        RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        const QString& role,
        const QString& source)
    {
        CollisionLinkModelVariantCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelVariantCommandController::useVariantInDetector(
                    service.document(),
                    viewportServices,
                    detectorId,
                    role,
                    source);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionLinkModelVariantCommandResult CollisionRequestDocumentFacade::setCurrentLinkCollisionModel(
        const QString& robotId,
        const QString& linkName,
        const QString& variantId)
    {
        CollisionLinkModelVariantCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                result = CollisionLinkModelVariantCommandController::setCurrentLinkModel(
                    service,
                    robotId,
                    linkName,
                    variantId);
                changed = result.projectChanged;
                if(!result.success) {
                    error = result.message.toStdString();
                }
                return result.success;
            });
        return result;
    }

    CollisionLinkModelVariantCommandResult CollisionRequestDocumentFacade::setCurrentObjectCollisionModel(
        const QString& objectId,
        const QString& variantId)
    {
        CollisionLinkModelVariantCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                result = CollisionLinkModelVariantCommandController::setCurrentObjectModel(
                    service,
                    objectId,
                    variantId);
                changed = result.projectChanged;
                if(!result.success) {
                    error = result.message.toStdString();
                }
                return result.success;
            });
        return result;
    }

    CollisionLinkModelDocumentCommandResult CollisionRequestDocumentFacade::setReplaceOriginal(
        const QString& robotId,
        bool replaceOriginal)
    {
        CollisionLinkModelDocumentCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelDocumentCommandController::setReplaceOriginal(
                    service.document(),
                    robotId,
                    replaceOriginal);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionLinkModelDocumentCommandResult CollisionRequestDocumentFacade::addBoxElement(
        const QString& robotId,
        const QString& linkName)
    {
        CollisionLinkModelDocumentCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelDocumentCommandController::addBoxElement(
                    service.document(),
                    robotId,
                    linkName);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionLinkModelsCommandResult CollisionRequestDocumentFacade::generateFromVisual(
        RobotQtViewerViewportServices* viewportServices,
        const QString& robotId,
        const QString& linkName,
        const CollisionRuntimeProxyRequest& request,
        bool replaceOriginal)
    {
        CollisionLinkModelsCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelsCommandController::generateFromVisual(
                    service.document(),
                    viewportServices,
                    robotId,
                    linkName,
                    request,
                    replaceOriginal);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionLinkModelsCommandResult CollisionRequestDocumentFacade::generateFromExistingCollision(
        RobotQtViewerViewportServices* viewportServices,
        const QString& robotId,
        const QString& linkName,
        const CollisionRuntimeProxyRequest& request,
        bool replaceOriginal)
    {
        CollisionLinkModelsCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelsCommandController::generateFromExistingCollision(
                    service.document(),
                    viewportServices,
                    robotId,
                    linkName,
                    request,
                    replaceOriginal);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionLinkModelsCommandResult CollisionRequestDocumentFacade::generateRobotFromExistingCollision(
        RobotQtViewerViewportServices* viewportServices,
        const QString& robotId,
        const CollisionRuntimeProxyRequest& request,
        bool replaceOriginal)
    {
        CollisionLinkModelsCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelsCommandController::generateRobotFromExistingCollision(
                    service.document(),
                    viewportServices,
                    robotId,
                    request,
                    replaceOriginal);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionLinkModelsCommandResult CollisionRequestDocumentFacade::generateMissingFromVisual(
        RobotQtViewerViewportServices* viewportServices,
        const QString& robotId,
        const CollisionRuntimeProxyRequest& request,
        bool replaceOriginal)
    {
        CollisionLinkModelsCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelsCommandController::generateMissingFromVisual(
                    service.document(),
                    viewportServices,
                    robotId,
                    request,
                    replaceOriginal);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionLinkModelsCommandResult CollisionRequestDocumentFacade::generateCoacdForLink(
        RobotQtViewerViewportServices* viewportServices,
        const QString& robotId,
        const QString& linkName)
    {
        CollisionLinkModelsCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelsCommandController::generateCoacdForLink(
                    service.document(),
                    viewportServices,
                    robotId,
                    linkName);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionLinkModelsCommandResult CollisionRequestDocumentFacade::generateCoacdForObject(
        RobotQtViewerViewportServices* viewportServices,
        const QString& objectId)
    {
        CollisionLinkModelsCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelsCommandController::generateCoacdForObject(
                    service.document(),
                    viewportServices,
                    objectId);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionLinkModelDocumentCommandResult CollisionRequestDocumentFacade::removeElement(
        const QString& robotId,
        const QString& linkName,
        const QString& elementId)
    {
        CollisionLinkModelDocumentCommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionRequest"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionLinkModelDocumentCommandController::removeElement(
                    service.document(),
                    robotId,
                    linkName,
                    elementId);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }
}
