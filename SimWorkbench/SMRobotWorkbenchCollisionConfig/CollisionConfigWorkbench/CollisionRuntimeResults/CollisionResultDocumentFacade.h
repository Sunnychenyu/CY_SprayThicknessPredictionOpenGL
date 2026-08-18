#pragma once

#include "CollisionResultsViewModel.h"
#include "CollisionRuntimeViewModel.h"

#include <QString>

#include <vector>

namespace simulation_project
{
    struct ProjectDocument;
}

namespace robot_qt_viewer
{
    class CollisionResultDocumentFacade
    {
    public:
        explicit CollisionResultDocumentFacade(const simulation_project::ProjectDocument& document);

        CollisionResultsViewModel buildViewModel(
            const QString& detectorId,
            bool viewportAvailable,
            const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors,
            const QString& markedPairRobotA,
            const QString& markedPairLinkA) const;

    private:
        const simulation_project::ProjectDocument& m_document;
    };
}
