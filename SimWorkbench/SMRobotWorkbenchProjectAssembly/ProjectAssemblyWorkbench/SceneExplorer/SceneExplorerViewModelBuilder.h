#pragma once

#include "SceneExplorerViewModel.h"

#include <SimulationProject/ProjectDocument.h>

namespace robot_qt_viewer
{
    SceneExplorerViewModel buildSceneExplorerViewModel(
        const simulation_project::ProjectDocument& document,
        const QVector<SceneExplorerRobotRuntimeView>& robots,
        const QVector<SceneExplorerObjectRuntimeView>& objects,
        const SceneExplorerNodeRef& selectedNode = SceneExplorerNodeRef(),
        RobotQtViewerViewportInteractionMode interactionMode = RobotQtViewerViewportInteractionMode::Browse);

    SceneExplorerViewModel buildSceneExplorerViewModel(
        const simulation_project::ProjectDocument& document,
        const QVector<SceneExplorerRobotRuntimeView>& robots,
        const QVector<SceneExplorerObjectRuntimeView>& objects,
        const SceneExplorerNodeRef& selectedNode,
        const SceneExplorerViewOptions& options);
}

