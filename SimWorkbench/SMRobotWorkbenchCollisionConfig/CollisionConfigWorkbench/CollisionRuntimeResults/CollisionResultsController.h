#pragma once

#include "CollisionResultsViewModel.h"

#include "CollisionRuntimeViewModel.h"
#include <SimulationProject/ProjectDocument.h>

#include <QString>

#include <vector>

class CollisionResultsController
{
public:
    static CollisionResultsViewModel buildViewModel(
        const simulation_project::ProjectDocument& document,
        const QString& detectorId,
        bool viewportAvailable,
        const std::vector<robot_qt_viewer::CollisionRuntimeDetectorInfo>& runtimeDetectors,
        const QString& markedPairRobotA,
        const QString& markedPairLinkA);
};
