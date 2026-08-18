#pragma once

#include "CollisionDetectorsViewModel.h"

#include "CollisionRuntimeViewModel.h"
#include <SimulationProject/ProjectDocument.h>

#include <QString>

#include <vector>

class CollisionDetectorsController
{
public:
    static QVector<CollisionDetectorListItemView> buildDetectorListItems(
        const simulation_project::ProjectDocument& document,
        const std::vector<robot_qt_viewer::CollisionRuntimeDetectorInfo>& runtimeDetectors);

    static CollisionDetectorPropertiesView buildProperties(
        const simulation_project::ProjectDocument& document,
        const QString& detectorId);
};
