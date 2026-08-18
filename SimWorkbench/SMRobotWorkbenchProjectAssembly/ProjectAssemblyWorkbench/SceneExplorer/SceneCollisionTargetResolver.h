#pragma once

#include "SceneExplorerViewModel.h"

#include <SimulationProject/CollisionTargetRef.h>

#include <QString>

namespace robot_qt_viewer
{
    bool sceneExplorerNodeKindCanBeCollisionTarget(SceneExplorerNodeKind kind);
    bool sceneExplorerNodeKindCanConfigureCollisionModel(SceneExplorerNodeKind kind);
    bool sceneExplorerNodeKindIsPrimaryCollisionTarget(SceneExplorerNodeKind kind);

    bool sceneExplorerNodeToCollisionTarget(
        const SceneExplorerNodeRef& node,
        simulation_project::CollisionTargetRef& target,
        QString* errorMessage = nullptr);

    bool sceneExplorerNodeToCollisionSelectionMember(
        const SceneExplorerNodeRef& node,
        simulation_project::CollisionSelectionSetMemberDesc& member,
        QString* errorMessage = nullptr);
}
