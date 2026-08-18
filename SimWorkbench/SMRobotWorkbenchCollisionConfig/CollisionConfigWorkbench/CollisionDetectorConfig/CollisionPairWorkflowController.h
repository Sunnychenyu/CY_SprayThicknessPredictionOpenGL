#pragma once

#include <QString>

namespace simulation_project
{
    struct ProjectDocument;
}

namespace robot_qt_viewer
{
    struct CollisionPairWorkflowResult
    {
        bool success = false;
        QString message;
    };

    class CollisionPairWorkflowController
    {
    public:
        bool syncRobotObjectPairs(simulation_project::ProjectDocument& document) const;
        CollisionPairWorkflowResult setAllRobotObjectPairsEnabled(
            simulation_project::ProjectDocument& document,
            bool enabled) const;
        CollisionPairWorkflowResult setRobotObjectPairEnabled(
            simulation_project::ProjectDocument& document,
            const QString& robotId,
            const QString& objectId,
            bool enabled) const;
    };
}
