#include "CollisionResultDocumentFacade.h"

#include "CollisionResultsController.h"

#include <SimulationProject/ProjectDocument.h>

namespace robot_qt_viewer
{
    CollisionResultDocumentFacade::CollisionResultDocumentFacade(
        const simulation_project::ProjectDocument& document)
        : m_document(document)
    {
    }

    CollisionResultsViewModel CollisionResultDocumentFacade::buildViewModel(
        const QString& detectorId,
        bool viewportAvailable,
        const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors,
        const QString& markedPairRobotA,
        const QString& markedPairLinkA) const
    {
        return CollisionResultsController::buildViewModel(
            m_document,
            detectorId,
            viewportAvailable,
            runtimeDetectors,
            markedPairRobotA,
            markedPairLinkA);
    }
}
