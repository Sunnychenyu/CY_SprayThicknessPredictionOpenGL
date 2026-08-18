#pragma once

#include <QString>

namespace robot_qt_viewer
{
    enum class RobotQtViewerWorkbenchKind
    {
        Browse,
        Motion,
        ToolSetup,
        Collision,
        TrajectoryPlanning,
        SprayProcess,
        CoatingAnalysis,
        DigitalTwin
    };

    enum class RobotQtViewerWorkbenchDomain
    {
        ProjectAssembly,
        RobotRun,
        CollisionConfig,
        TrajectoryPlanning,
        SprayProcess,
        CoatingAnalysis,
        DigitalTwin
    };

    enum class RobotQtViewerRightPanelKind
    {
        Status,
        SceneSelection,
        Motion,
        MotionPlanning,
        ProjectAssembly,
        CollisionConfig,
        CoatingAnalysis
    };

    enum class RobotQtViewerViewportInteractionMode
    {
        Browse,
        SelectRobot,
        SelectLink,
        SelectMount,
        SelectAttachment,
        EditTransformPreview,
        EditCollisionProxy,
        SelectCollisionTarget
    };

    struct RobotQtViewerTaskSession
    {
        RobotQtViewerWorkbenchKind workbench = RobotQtViewerWorkbenchKind::Browse;
        RobotQtViewerViewportInteractionMode viewportMode = RobotQtViewerViewportInteractionMode::Browse;
        QString taskId;
        QString targetRobotId;
        QString targetLinkName;
        QString targetObjectId;
        QString targetMountId;
        QString targetAttachmentId;
        QString targetCollisionDetectorId;
        bool dirty = false;
        bool canExit = true;
    };

    struct RobotQtViewerWorkbenchDescriptor
    {
        RobotQtViewerWorkbenchKind kind = RobotQtViewerWorkbenchKind::Browse;
        RobotQtViewerWorkbenchDomain domain = RobotQtViewerWorkbenchDomain::ProjectAssembly;
        RobotQtViewerRightPanelKind rightPanel = RobotQtViewerRightPanelKind::SceneSelection;
        RobotQtViewerViewportInteractionMode defaultViewportMode =
            RobotQtViewerViewportInteractionMode::Browse;
        QString id;
        QString displayName;
        QString rightPanelTitle;
        bool projectAssemblyTreeProjection = true;
        bool collisionTreeProjection = false;
        bool projectAssemblyActions = true;
        bool collisionActions = false;
    };

    QString robotQtViewerWorkbenchName(RobotQtViewerWorkbenchKind kind);
    const RobotQtViewerWorkbenchDescriptor& robotQtViewerWorkbenchDescriptor(
        RobotQtViewerWorkbenchKind kind);
    QString robotQtViewerViewportInteractionModeName(RobotQtViewerViewportInteractionMode mode);

    class RobotQtViewerWorkbenchManager
    {
    public:
        RobotQtViewerWorkbenchKind activeWorkbench() const;
        const RobotQtViewerWorkbenchDescriptor& activeDescriptor() const;
        RobotQtViewerViewportInteractionMode viewportMode() const;
        const RobotQtViewerTaskSession& session() const;

        bool enterWorkbench(RobotQtViewerWorkbenchKind kind, const QString& sourceId = QString());
        bool exitToBrowse(const QString& sourceId = QString());
        bool canExitActiveWorkbench() const;
        void setSessionDirty(bool dirty);
        void setSessionCanExit(bool canExit);
        void setViewportMode(RobotQtViewerViewportInteractionMode mode);
        void setSessionTarget(
            const QString& robotId,
            const QString& linkName = QString(),
            const QString& mountId = QString(),
            const QString& attachmentId = QString());

    private:
        RobotQtViewerTaskSession m_session;
    };
}

