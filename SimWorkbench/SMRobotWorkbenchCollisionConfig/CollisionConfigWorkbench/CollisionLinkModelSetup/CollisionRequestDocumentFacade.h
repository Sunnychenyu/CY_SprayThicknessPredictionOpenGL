#pragma once

#include "CollisionLinkModelDocumentCommandController.h"
#include "CollisionLinkModelVariantCommandController.h"
#include "CollisionLinkModelsCommandController.h"

namespace simulation_project
{
    struct ProjectDocument;
    class ProjectSession;
}

namespace robot_qt_viewer
{
    class CollisionWorkbenchServices;

    class CollisionRequestDocumentFacade
    {
    public:
        CollisionRequestDocumentFacade(
            const simulation_project::ProjectDocument& document,
            CollisionWorkbenchServices& appServices);

        CollisionLinkModelVariantCommandResult useVariantInDetector(
            RobotQtViewerViewportServices* viewportServices,
            const QString& detectorId,
            const QString& role,
            const QString& source);

        CollisionLinkModelVariantCommandResult setCurrentLinkCollisionModel(
            const QString& robotId,
            const QString& linkName,
            const QString& variantId);

        CollisionLinkModelVariantCommandResult setCurrentObjectCollisionModel(
            const QString& objectId,
            const QString& variantId);

        CollisionLinkModelDocumentCommandResult setReplaceOriginal(
            const QString& robotId,
            bool replaceOriginal);

        CollisionLinkModelDocumentCommandResult addBoxElement(
            const QString& robotId,
            const QString& linkName);

        CollisionLinkModelsCommandResult generateFromVisual(
            RobotQtViewerViewportServices* viewportServices,
            const QString& robotId,
            const QString& linkName,
            const CollisionRuntimeProxyRequest& request,
            bool replaceOriginal);

        CollisionLinkModelsCommandResult generateFromExistingCollision(
            RobotQtViewerViewportServices* viewportServices,
            const QString& robotId,
            const QString& linkName,
            const CollisionRuntimeProxyRequest& request,
            bool replaceOriginal);

        CollisionLinkModelsCommandResult generateRobotFromExistingCollision(
            RobotQtViewerViewportServices* viewportServices,
            const QString& robotId,
            const CollisionRuntimeProxyRequest& request,
            bool replaceOriginal);

        CollisionLinkModelsCommandResult generateCoacdForLink(
            RobotQtViewerViewportServices* viewportServices,
            const QString& robotId,
            const QString& linkName);

        CollisionLinkModelsCommandResult generateCoacdForObject(
            RobotQtViewerViewportServices* viewportServices,
            const QString& objectId);

        CollisionLinkModelsCommandResult generateMissingFromVisual(
            RobotQtViewerViewportServices* viewportServices,
            const QString& robotId,
            const CollisionRuntimeProxyRequest& request,
            bool replaceOriginal);

        CollisionLinkModelDocumentCommandResult removeElement(
            const QString& robotId,
            const QString& linkName,
            const QString& elementId);

    private:
        const simulation_project::ProjectDocument& m_document;
        CollisionWorkbenchServices& m_appServices;
    };
}
