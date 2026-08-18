#pragma once

#include "CollisionRuntimeViewModel.h"

#include <SimulationProject/ProjectDocument.h>

#include <QString>

#include <filesystem>

namespace robot_qt_viewer
{
    class CollisionWorkbenchServices;
    class RobotQtViewerViewportServices;
}

struct CollisionLinkModelsCommandResult
{
    bool success = false;
    bool projectChanged = false;
    bool runtimeUpdateFailed = false;
    int generatedCount = 0;
    QString selectedSource;
    QString selectedRole;
    QString qualityMessage;
    QString message;
};

class CollisionLinkModelsCommandController
{
public:
    static CollisionLinkModelsCommandResult generateFromVisual(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& robotId,
        const QString& linkName,
        const robot_qt_viewer::CollisionRuntimeProxyRequest& request,
        bool replaceOriginal);

    static CollisionLinkModelsCommandResult generateFromExistingCollision(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& robotId,
        const QString& linkName,
        const robot_qt_viewer::CollisionRuntimeProxyRequest& request,
        bool replaceOriginal);

    static CollisionLinkModelsCommandResult generateRobotFromExistingCollision(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& robotId,
        const robot_qt_viewer::CollisionRuntimeProxyRequest& request,
        bool replaceOriginal);

    static CollisionLinkModelsCommandResult generateCoacdForLink(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& robotId,
        const QString& linkName);

    static CollisionLinkModelsCommandResult generateCoacdForObject(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& objectId);

    static CollisionLinkModelsCommandResult generateMissingFromVisual(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& robotId,
        const robot_qt_viewer::CollisionRuntimeProxyRequest& request,
        bool replaceOriginal);

    static CollisionLinkModelsCommandResult saveSidecar(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::CollisionWorkbenchServices& appServices,
        const QString& robotId,
        const std::filesystem::path& sidecarPath,
        const std::string& portableSidecarPath);

    static CollisionLinkModelsCommandResult exportUrdf(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::CollisionWorkbenchServices& appServices,
        const QString& robotId,
        const std::filesystem::path& sourceUrdfPath,
        const std::filesystem::path& outputUrdfPath);
};
