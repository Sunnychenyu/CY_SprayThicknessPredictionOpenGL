#pragma once

#include "SceneExplorerViewModel.h"

#include <QString>

namespace simulation_project
{
    struct ProjectDocument;
}

namespace robot_qt_viewer
{
    enum class SceneSelectionIntentKind
    {
        None,
        Clear,
        SelectRobotLink,
        SelectRobotJoint,
        SelectRobotMount,
        SelectObjectFrame,
        SelectMountedAttachment,
        SelectSceneObject,
        SelectToolAsset
    };

    struct SceneSelectionIntent
    {
        SceneSelectionIntentKind kind = SceneSelectionIntentKind::None;
        QString itemId;
        QString robotId;
        QString linkName;
        QString jointName;
        QString mountId;
        QString statusMessage;
    };

    struct SceneRobotSelectionContext
    {
        QString robotId;
        QString linkName;
        QString mountId;
    };

    class SceneSelectionController
    {
    public:
        static SceneSelectionIntent intentFromNode(
            const simulation_project::ProjectDocument& document,
            const SceneExplorerNodeRef& node);
        static SceneExplorerNodeRef nodeFromViewportPick(
            const QString& kind,
            const QString& robotId,
            const QString& linkName,
            const QString& robotMountId,
            const QString& mountedAttachmentId,
            const QString& sceneObjectId);
        static SceneRobotSelectionContext robotSelectionContext(
            const simulation_project::ProjectDocument& document,
            const QString& robotId,
            const QString& preferredLinkName,
            const QString& preferredMountId);
        static QString preferredMountIdForRobot(
            const simulation_project::ProjectDocument& document,
            const QString& robotId,
            const QString& preferredLinkName,
            const QString& preferredMountId);
    };
}
