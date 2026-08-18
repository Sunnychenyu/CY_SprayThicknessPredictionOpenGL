#include "RobotQtViewerWorkbench.h"

namespace robot_qt_viewer
{
    namespace
    {
        const RobotQtViewerWorkbenchDescriptor kBrowseDescriptor{
            RobotQtViewerWorkbenchKind::Browse,
            RobotQtViewerWorkbenchDomain::ProjectAssembly,
            RobotQtViewerRightPanelKind::SceneSelection,
            RobotQtViewerViewportInteractionMode::Browse,
            QStringLiteral("projectAssemblyBrowse"),
            QStringLiteral("Project Assembly"),
            QStringLiteral("Scene Edit Panel"),
            true,
            false,
            true,
            false
        };

        const RobotQtViewerWorkbenchDescriptor kMotionDescriptor{
            RobotQtViewerWorkbenchKind::Motion,
            RobotQtViewerWorkbenchDomain::RobotRun,
            RobotQtViewerRightPanelKind::Motion,
            RobotQtViewerViewportInteractionMode::SelectRobot,
            QStringLiteral("robotRun"),
            QStringLiteral("Robot Run"),
            QStringLiteral("Motion Panel"),
            true,
            false,
            false,
            false
        };

        const RobotQtViewerWorkbenchDescriptor kToolSetupDescriptor{
            RobotQtViewerWorkbenchKind::ToolSetup,
            RobotQtViewerWorkbenchDomain::ProjectAssembly,
            RobotQtViewerRightPanelKind::ProjectAssembly,
            RobotQtViewerViewportInteractionMode::SelectMount,
            QStringLiteral("projectAssembly"),
            QStringLiteral("Project Assembly"),
            QStringLiteral("Mount Frame Editor"),
            true,
            false,
            true,
            false
        };

        const RobotQtViewerWorkbenchDescriptor kCollisionDescriptor{
            RobotQtViewerWorkbenchKind::Collision,
            RobotQtViewerWorkbenchDomain::CollisionConfig,
            RobotQtViewerRightPanelKind::CollisionConfig,
            RobotQtViewerViewportInteractionMode::SelectCollisionTarget,
            QStringLiteral("collisionConfig"),
            QStringLiteral("Collision Config"),
            QStringLiteral("Collision Detector Configuration"),
            false,
            true,
            false,
            true
        };

        const RobotQtViewerWorkbenchDescriptor kTrajectoryPlanningDescriptor{
            RobotQtViewerWorkbenchKind::TrajectoryPlanning,
            RobotQtViewerWorkbenchDomain::TrajectoryPlanning,
            RobotQtViewerRightPanelKind::MotionPlanning,
            RobotQtViewerViewportInteractionMode::Browse,
            QStringLiteral("motionPlanning"),
            QStringLiteral("Motion Planning"),
            QStringLiteral("Motion Planning"),
            true,
            false,
            false,
            false
        };

        const RobotQtViewerWorkbenchDescriptor kSprayProcessDescriptor{
            RobotQtViewerWorkbenchKind::SprayProcess,
            RobotQtViewerWorkbenchDomain::SprayProcess,
            RobotQtViewerRightPanelKind::Status,
            RobotQtViewerViewportInteractionMode::Browse,
            QStringLiteral("sprayProcess"),
            QStringLiteral("Spray Process"),
            QStringLiteral("Spray Process"),
            true,
            false,
            false,
            false
        };

        const RobotQtViewerWorkbenchDescriptor kDigitalTwinDescriptor{
            RobotQtViewerWorkbenchKind::DigitalTwin,
            RobotQtViewerWorkbenchDomain::DigitalTwin,
            RobotQtViewerRightPanelKind::Status,
            RobotQtViewerViewportInteractionMode::Browse,
            QStringLiteral("digitalTwin"),
            QStringLiteral("Digital Twin"),
            QStringLiteral("Digital Twin"),
            true,
            false,
            false,
            false
        };

        const RobotQtViewerWorkbenchDescriptor kCoatingAnalysisDescriptor{
            RobotQtViewerWorkbenchKind::CoatingAnalysis,
            RobotQtViewerWorkbenchDomain::CoatingAnalysis,
            RobotQtViewerRightPanelKind::CoatingAnalysis,
            RobotQtViewerViewportInteractionMode::Browse,
            QStringLiteral("coatingAnalysis"),
            QStringLiteral("Coating Analysis"),
            QStringLiteral("Coating Analysis"),
            true,
            false,
            false,
            false
        };
    }

    QString robotQtViewerWorkbenchName(RobotQtViewerWorkbenchKind kind)
    {
        return robotQtViewerWorkbenchDescriptor(kind).displayName;
    }

    const RobotQtViewerWorkbenchDescriptor& robotQtViewerWorkbenchDescriptor(
        RobotQtViewerWorkbenchKind kind)
    {
        switch(kind) {
        case RobotQtViewerWorkbenchKind::Browse:
            return kBrowseDescriptor;
        case RobotQtViewerWorkbenchKind::Motion:
            return kMotionDescriptor;
        case RobotQtViewerWorkbenchKind::ToolSetup:
            return kToolSetupDescriptor;
        case RobotQtViewerWorkbenchKind::Collision:
            return kCollisionDescriptor;
        case RobotQtViewerWorkbenchKind::TrajectoryPlanning:
            return kTrajectoryPlanningDescriptor;
        case RobotQtViewerWorkbenchKind::SprayProcess:
            return kSprayProcessDescriptor;
        case RobotQtViewerWorkbenchKind::CoatingAnalysis:
            return kCoatingAnalysisDescriptor;
        case RobotQtViewerWorkbenchKind::DigitalTwin:
            return kDigitalTwinDescriptor;
        }
        return kBrowseDescriptor;
    }

    QString robotQtViewerViewportInteractionModeName(RobotQtViewerViewportInteractionMode mode)
    {
        switch(mode) {
        case RobotQtViewerViewportInteractionMode::Browse:
            return QStringLiteral("Browse");
        case RobotQtViewerViewportInteractionMode::SelectRobot:
            return QStringLiteral("SelectRobot");
        case RobotQtViewerViewportInteractionMode::SelectLink:
            return QStringLiteral("SelectLink");
        case RobotQtViewerViewportInteractionMode::SelectMount:
            return QStringLiteral("SelectMount");
        case RobotQtViewerViewportInteractionMode::SelectAttachment:
            return QStringLiteral("SelectAttachment");
        case RobotQtViewerViewportInteractionMode::EditTransformPreview:
            return QStringLiteral("EditTransformPreview");
        case RobotQtViewerViewportInteractionMode::EditCollisionProxy:
            return QStringLiteral("EditCollisionProxy");
        case RobotQtViewerViewportInteractionMode::SelectCollisionTarget:
            return QStringLiteral("SelectCollisionTarget");
        }
        return QStringLiteral("Browse");
    }

    RobotQtViewerWorkbenchKind RobotQtViewerWorkbenchManager::activeWorkbench() const
    {
        return m_session.workbench;
    }

    const RobotQtViewerWorkbenchDescriptor& RobotQtViewerWorkbenchManager::activeDescriptor() const
    {
        return robotQtViewerWorkbenchDescriptor(m_session.workbench);
    }

    RobotQtViewerViewportInteractionMode RobotQtViewerWorkbenchManager::viewportMode() const
    {
        return m_session.viewportMode;
    }

    const RobotQtViewerTaskSession& RobotQtViewerWorkbenchManager::session() const
    {
        return m_session;
    }

    bool RobotQtViewerWorkbenchManager::enterWorkbench(
        RobotQtViewerWorkbenchKind kind,
        const QString& sourceId)
    {
        if(!canExitActiveWorkbench()) {
            return false;
        }

        m_session = RobotQtViewerTaskSession{};
        const RobotQtViewerWorkbenchDescriptor& descriptor =
            robotQtViewerWorkbenchDescriptor(kind);
        m_session.workbench = kind;
        m_session.taskId = sourceId.isEmpty() ? descriptor.id : sourceId;
        m_session.viewportMode = descriptor.defaultViewportMode;
        return true;
    }

    bool RobotQtViewerWorkbenchManager::exitToBrowse(const QString& sourceId)
    {
        return enterWorkbench(RobotQtViewerWorkbenchKind::Browse, sourceId);
    }

    bool RobotQtViewerWorkbenchManager::canExitActiveWorkbench() const
    {
        return m_session.canExit;
    }

    void RobotQtViewerWorkbenchManager::setSessionDirty(bool dirty)
    {
        m_session.dirty = dirty;
    }

    void RobotQtViewerWorkbenchManager::setSessionCanExit(bool canExit)
    {
        m_session.canExit = canExit;
    }

    void RobotQtViewerWorkbenchManager::setViewportMode(RobotQtViewerViewportInteractionMode mode)
    {
        m_session.viewportMode = mode;
    }

    void RobotQtViewerWorkbenchManager::setSessionTarget(
        const QString& robotId,
        const QString& linkName,
        const QString& mountId,
        const QString& attachmentId)
    {
        m_session.targetRobotId = robotId;
        m_session.targetLinkName = linkName;
        m_session.targetMountId = mountId;
        m_session.targetAttachmentId = attachmentId;
    }
}

