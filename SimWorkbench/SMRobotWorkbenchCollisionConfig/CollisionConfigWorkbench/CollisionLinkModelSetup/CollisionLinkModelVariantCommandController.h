#pragma once

#include <SimulationProject/ProjectDocument.h>

#include <QString>

namespace simulation_project
{
    class ProjectDocumentService;
}

namespace robot_qt_viewer
{
    class RobotQtViewerViewportServices;
}

struct CollisionLinkModelVariantCommandResult
{
    bool success = false;
    bool projectChanged = false;
    bool runtimeUpdateFailed = false;
    QString message;
};

class CollisionLinkModelVariantCommandController
{
public:
    static CollisionLinkModelVariantCommandResult showVariantOnly(
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& robotId,
        const QString& linkName,
        const QString& variantId);

    static CollisionLinkModelVariantCommandResult useVariantInDetector(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        const QString& role,
        const QString& source);

    static CollisionLinkModelVariantCommandResult setCurrentLinkModel(
        simulation_project::ProjectDocumentService& service,
        const QString& robotId,
        const QString& linkName,
        const QString& variantId);

    static CollisionLinkModelVariantCommandResult setCurrentObjectModel(
        simulation_project::ProjectDocumentService& service,
        const QString& objectId,
        const QString& variantId);
};
