#pragma once

#include "RobotQtViewerEvents.h"
#include "RobotQtViewerWorkbench.h"
#include "SceneExplorerViewModel.h"
#include "SceneSelectionController.h"
#include "SceneTreeIntentController.h"

#include <QObject>
#include <QHash>
#include <QPoint>
#include <QSet>
#include <QString>
#include <QStringList>
#include <QVector>

class SceneExplorerWidget;
class SceneExplorerTaskWidget;
class QWidget;

namespace robot_qt_viewer
{
    class RobotQtViewerDocumentContext;
    class RobotQtViewerViewportServices;
    class SceneEntityWorkflowController;

    class SceneExplorerModuleController : public QObject
    {
        Q_OBJECT

    public:
        SceneExplorerModuleController(
            SceneExplorerWidget& widget,
            RobotQtViewerDocumentContext& context,
            QObject* parent = nullptr);

        void setRobotRuntime(
            const QString& robotId,
            const QString& robotName,
            const QStringList& links,
            const QStringList& joints,
            const QStringList& movableJoints,
            const QStringList& movableJointTypes);
        void setSceneObjectRuntime(const QString& objectId, const QString& objectName);
        void clearRuntime();
        void refreshViewModel();
        void setTaskWidget(SceneExplorerTaskWidget* taskWidget);
        void setSceneEntityWorkflow(SceneEntityWorkflowController* workflow);
        void setViewportServices(RobotQtViewerViewportServices* viewportServices);
        void setViewportInteractionMode(RobotQtViewerViewportInteractionMode mode);
        void setWorkbenchDescriptor(const RobotQtViewerWorkbenchDescriptor& descriptor);
        void handleEvent(const RobotQtViewerEvent& event);
        SceneExplorerNodeRef currentNode() const;
        bool selectNode(const SceneExplorerNodeRef& node);
        QHash<QString, QStringList> robotLinksByRobotId() const;
        SceneSelectionIntent selectionIntentForNode(const SceneExplorerNodeRef& node) const;
        SceneRobotSelectionContext robotSelectionContext(
            const QString& robotId,
            const QString& preferredLinkName,
            const QString& preferredMountId) const;
        void focusTransformTask(const SceneExplorerNodeRef& node);
        void createObjectFrameForObject(const QString& objectId);
        bool resolvePendingTransformPreviewIfTargetChanges(
            const SceneExplorerNodeRef& nextNode,
            QWidget* parentWidget);
        bool linkFrameVisible(const QString& robotId, const QString& linkName) const;
        bool toggleLinkFrameVisible(const QString& robotId, const QString& linkName);

    signals:
        void nodeActivated(const robot_qt_viewer::SceneExplorerNodeRef& node, int column);
        void statusMessageRequested(const QString& message, int timeoutMs);
        void contextMenuActionRequested(
            const robot_qt_viewer::SceneTreeIntentController::ContextMenuAction& action);

    private:
        bool nodeSelectableForCurrentMode(const SceneExplorerNodeRef& node) const;
        void handleNodeActivated(const SceneExplorerNodeRef& node, int column);
        void handleTransformPreviewChanged(
            const SceneExplorerNodeRef& target,
            const simulation_project::TransformDesc& transform);
        void applyTransformChange(
            const SceneExplorerNodeRef& target,
            const simulation_project::TransformDesc& transform);
        void cancelTransformChange(const SceneExplorerNodeRef& target);
        void handleObjectFrameVisibilityChanged(const SceneExplorerNodeRef& target, bool visible);
        bool sameTransformTarget(const SceneExplorerNodeRef& lhs, const SceneExplorerNodeRef& rhs) const;
        bool hasPendingTransformPreviewFor(const SceneExplorerNodeRef& target) const;
        bool resolvePendingTransformPreview(QWidget* parentWidget);
        void discardPendingTransformPreview();
        void publishTaskStateChanged(const QString& sourceId);
        void mutateViewportPreview(
            const RobotQtViewerViewportPreviewPayload& preview,
            const QString& sourceId);
        bool objectFrameTaskActive() const;
        void mergeActiveObjectFrameViewportState(RobotQtViewerViewportPreviewPayload& preview) const;
        void reassertActiveObjectFrameTask(const QString& sourceId);
        void captureObjectFrameEditSnapshot(
            const QString& objectId,
            const QString& frameId,
            bool newFrame = false);
        void clearObjectFrameEditSnapshot();
        bool restoreObjectFrameDraft(const QString& sourceId);
        void showContextMenu(const QPoint& pos);
        QString linkFrameKey(const QString& robotId, const QString& linkName) const;

        SceneExplorerWidget& m_widget;
        SceneExplorerTaskWidget* m_taskWidget = nullptr;
        RobotQtViewerDocumentContext& m_context;
        SceneEntityWorkflowController* m_sceneEntityWorkflow = nullptr;
        RobotQtViewerViewportServices* m_viewportServices = nullptr;
        QVector<SceneExplorerRobotRuntimeView> m_robots;
        QVector<SceneExplorerObjectRuntimeView> m_objects;
        RobotQtViewerViewportInteractionMode m_interactionMode = RobotQtViewerViewportInteractionMode::Browse;
        RobotQtViewerWorkbenchDescriptor m_workbenchDescriptor;
        bool m_hasPendingTransformPreview = false;
        SceneExplorerNodeRef m_pendingTransformTarget;
        simulation_project::TransformDesc m_pendingTransform;
        SceneExplorerObjectFrameMode m_objectFrameMode = SceneExplorerObjectFrameMode::Selection;
        bool m_hasObjectFrameEditSnapshot = false;
        bool m_objectFrameEditSnapshotIsNew = false;
        bool m_hasObjectFrameRollbackDocument = false;
        bool m_objectFrameRollbackDirty = false;
        QString m_activeObjectFrameObjectId;
        QString m_activeObjectFrameId;
        QString m_objectFrameDraftSourceObjectId;
        simulation_project::ObjectFrameDesc m_objectFrameEditSnapshot;
        simulation_project::ProjectDocument m_objectFrameRollbackDocument;
        QSet<QString> m_visibleLinkFrameKeys;
    };
}
