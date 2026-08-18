#pragma once

#include "CollisionLegacyPairsViewModel.h"
#include "CollisionPairWorkflowController.h"

#include <QVector>

namespace simulation_project
{
    struct ProjectDocument;
    class ProjectSession;
}

namespace robot_qt_viewer
{
    class CollisionWorkbenchServices;

    class CollisionLegacyPairDocumentFacade
    {
    public:
        CollisionLegacyPairDocumentFacade(
            const simulation_project::ProjectDocument& document,
            CollisionWorkbenchServices& appServices);

        QVector<CollisionLegacyPairItemView> buildPairItems() const;
        bool canAutoPairAll() const;
        bool syncRobotObjectPairs();
        CollisionPairWorkflowResult setAllRobotObjectPairsEnabled(bool enabled);
        CollisionPairWorkflowResult setRobotObjectPairEnabled(
            const QString& robotId,
            const QString& objectId,
            bool enabled);

    private:
        const simulation_project::ProjectDocument& m_document;
        CollisionWorkbenchServices& m_appServices;
    };
}
