#pragma once

#include "CollisionDetectorsViewModel.h"

#include <SimulationProject/ProjectDocument.h>

#include <QString>
#include <QVector>

namespace robot_qt_viewer
{
    class RobotQtViewerViewportServices;
}

struct CollisionDetectorCommandResult
{
    bool success = false;
    bool projectChanged = false;
    bool runtimeUpdateFailed = false;
    QString message;
};

class CollisionDetectorCommandController
{
public:
    static CollisionDetectorCommandResult setDetectorEnabled(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        bool enabled);

    static CollisionDetectorCommandResult applyDetectorProperties(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        const CollisionDetectorPropertiesView& properties);

    static CollisionDetectorCommandResult applyDetectorQueryContract(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        const CollisionDetectorQueryContractView& contract);

    static CollisionDetectorCommandResult showOnlyDetectors(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QVector<QString>& detectorIds);

    static CollisionDetectorCommandResult removeDetector(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId);

    static CollisionDetectorCommandResult bindDetectorPairGenerators(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        const QVector<CollisionDetectorDraftMemberView>& setA,
        const QVector<CollisionDetectorDraftMemberView>& setB);

    static CollisionDetectorCommandResult removeDetectorPairGenerators(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId,
        const QVector<int>& generatorIndexes);

    static CollisionDetectorCommandResult clearDetectorPairScope(
        simulation_project::ProjectDocument& document,
        robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
        const QString& detectorId);
};
