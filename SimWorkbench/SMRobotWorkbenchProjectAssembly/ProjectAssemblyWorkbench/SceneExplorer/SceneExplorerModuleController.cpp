#include "SceneExplorerModuleController.h"

#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerSelectionModel.h"
#include "RobotQtViewerViewportServices.h"
#include "RobotQtViewerViewportPreviewState.h"
#include "SceneExplorerViewModelBuilder.h"
#include "SceneSelectionController.h"
#include "SceneExplorerTaskWidget.h"
#include "SceneExplorerWidget.h"
#include "SceneEntityWorkflowController.h"

#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectSession.h>

#include <QAction>
#include <QByteArray>
#include <QMenu>
#include <QMessageBox>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include <cmath>
#include <algorithm>
#include <cstddef>

namespace robot_qt_viewer
{
    namespace
    {
        bool transformForNode(
            const simulation_project::ProjectDocument& document,
            const SceneExplorerNodeRef& node,
            simulation_project::TransformDesc& transform)
        {
            if(node.kind == SceneExplorerNodeKind::Robot) {
                for(const simulation_project::RobotDesc& robot : document.robots) {
                    if(robot.id == node.id.toStdString()) {
                        transform = robot.baseTransform;
                        return true;
                    }
                }
                return false;
            }

            if(node.kind == SceneExplorerNodeKind::Object) {
                for(const simulation_project::SceneObjectDesc& object : document.objects) {
                    if(object.id == node.id.toStdString()) {
                        transform = object.transform;
                        return true;
                    }
                }
            }
            if(node.kind == SceneExplorerNodeKind::ObjectFrame) {
                for(const simulation_project::SceneObjectDesc& object : document.objects) {
                    if(object.id != node.id.toStdString()) {
                        continue;
                    }
                    for(const simulation_project::ObjectFrameDesc& frame : object.objectFrames) {
                        if(frame.id == node.linkName.toStdString()) {
                            transform = frame.objectToFrame;
                            return true;
                        }
                    }
                    return false;
                }
            }
            if(node.kind == SceneExplorerNodeKind::PointCloud) {
                for(const simulation_project::PointCloudDesc& pointCloud : document.pointClouds) {
                    if(pointCloud.id == node.id.toStdString()) {
                        transform = pointCloud.transform;
                        return true;
                    }
                }
            }
            return false;
        }

        std::string qStringToUtf8(const QString& text)
        {
            const QByteArray bytes = text.toUtf8();
            return std::string(bytes.constData(), static_cast<std::size_t>(bytes.size()));
        }

        const simulation_project::ObjectFrameDesc* findObjectFrameDesc(
            const simulation_project::ProjectDocument& document,
            const QString& objectId,
            const QString& frameId)
        {
            const std::string objectIdValue = qStringToUtf8(objectId);
            const std::string frameIdValue = qStringToUtf8(frameId);
            for(const simulation_project::SceneObjectDesc& object : document.objects) {
                if(object.id != objectIdValue) {
                    continue;
                }
                for(const simulation_project::ObjectFrameDesc& frame : object.objectFrames) {
                    if(frame.id == frameIdValue) {
                        return &frame;
                    }
                }
                return nullptr;
            }
            return nullptr;
        }

        bool nearlyEqual(double lhs, double rhs)
        {
            return std::abs(lhs - rhs) <= 1.0e-9;
        }

        bool sameTransform(
            const simulation_project::TransformDesc& lhs,
            const simulation_project::TransformDesc& rhs)
        {
            return nearlyEqual(lhs.x, rhs.x) &&
                nearlyEqual(lhs.y, rhs.y) &&
                nearlyEqual(lhs.z, rhs.z) &&
                nearlyEqual(lhs.roll, rhs.roll) &&
                nearlyEqual(lhs.pitch, rhs.pitch) &&
                nearlyEqual(lhs.yaw, rhs.yaw);
        }
    }

    SceneExplorerModuleController::SceneExplorerModuleController(
        SceneExplorerWidget& widget,
        RobotQtViewerDocumentContext& context,
        QObject* parent)
        : QObject(parent)
        , m_widget(widget)
        , m_context(context)
    {
        connect(&m_widget, &SceneExplorerWidget::nodeActivated,
            this, &SceneExplorerModuleController::handleNodeActivated);
        connect(&m_widget, &SceneExplorerWidget::treeContextMenuRequested,
            this, &SceneExplorerModuleController::showContextMenu);
    }

    void SceneExplorerModuleController::setRobotRuntime(
        const QString& robotId,
        const QString& robotName,
        const QStringList& links,
        const QStringList& joints,
        const QStringList& movableJoints,
        const QStringList& movableJointTypes)
    {
        for(SceneExplorerRobotRuntimeView& robot : m_robots) {
            if(robot.id == robotId) {
                robot.name = robotName;
                robot.links = links;
                robot.joints = joints;
                robot.movableJoints = movableJoints;
                robot.movableJointTypes = movableJointTypes;
                m_context.documentController().publishRobotRuntimeChanged(QStringLiteral("sceneExplorerRuntime"));
                return;
            }
        }

        SceneExplorerRobotRuntimeView robot;
        robot.id = robotId;
        robot.name = robotName;
        robot.links = links;
        robot.joints = joints;
        robot.movableJoints = movableJoints;
        robot.movableJointTypes = movableJointTypes;
        m_robots.push_back(robot);
        m_context.documentController().publishRobotRuntimeChanged(QStringLiteral("sceneExplorerRuntime"));
    }

    void SceneExplorerModuleController::setSceneObjectRuntime(const QString& objectId, const QString& objectName)
    {
        for(SceneExplorerObjectRuntimeView& object : m_objects) {
            if(object.id == objectId) {
                object.name = objectName;
                m_context.documentController().publishRobotRuntimeChanged(QStringLiteral("sceneExplorerRuntime"));
                return;
            }
        }

        SceneExplorerObjectRuntimeView object;
        object.id = objectId;
        object.name = objectName;
        m_objects.push_back(object);
        m_context.documentController().publishRobotRuntimeChanged(QStringLiteral("sceneExplorerRuntime"));
    }

    void SceneExplorerModuleController::clearRuntime()
    {
        m_robots.clear();
        m_objects.clear();
        m_context.documentController().publishRobotRuntimeChanged(QStringLiteral("sceneExplorerRuntimeClear"));
    }

    void SceneExplorerModuleController::refreshViewModel()
    {
        SceneExplorerViewModel viewModel = buildSceneExplorerViewModel(
            m_context.document(),
            m_robots,
            m_objects,
            m_widget.currentNode(),
            SceneExplorerViewOptions{
                m_interactionMode,
                sceneExplorerTreeProjectionForWorkbench(m_workbenchDescriptor)
            });
        if(viewModel.objectFrameEditorVisible &&
            viewModel.transformTarget.kind == SceneExplorerNodeKind::ObjectFrame &&
            viewModel.transformTarget.id == m_activeObjectFrameObjectId &&
            viewModel.transformTarget.linkName == m_activeObjectFrameId) {
            viewModel.objectFrameMode = m_objectFrameMode;
        }
        if(viewModel.transformEditorVisible && hasPendingTransformPreviewFor(viewModel.transformTarget)) {
            viewModel.transform = m_pendingTransform;
            viewModel.transformEditorDirty = true;
            if(viewModel.transformTarget.kind == SceneExplorerNodeKind::ObjectFrame &&
                !m_pendingTransformTarget.name.isEmpty()) {
                viewModel.objectFrameName = m_pendingTransformTarget.name;
            }
        }
        m_widget.setDocumentView(viewModel);
        if(m_taskWidget != nullptr) {
            m_taskWidget->setDocumentView(viewModel);
        }
    }

    void SceneExplorerModuleController::setTaskWidget(SceneExplorerTaskWidget* taskWidget)
    {
        m_taskWidget = taskWidget;
        if(m_taskWidget == nullptr) {
            return;
        }
        connect(m_taskWidget, &SceneExplorerTaskWidget::transformPreviewChanged,
            this, &SceneExplorerModuleController::handleTransformPreviewChanged);
        connect(m_taskWidget, &SceneExplorerTaskWidget::transformApplyRequested,
            this, &SceneExplorerModuleController::applyTransformChange);
        connect(m_taskWidget, &SceneExplorerTaskWidget::transformCancelRequested,
            this, &SceneExplorerModuleController::cancelTransformChange);
        connect(m_taskWidget, &SceneExplorerTaskWidget::objectFrameVisibilityChanged,
            this, &SceneExplorerModuleController::handleObjectFrameVisibilityChanged);
        refreshViewModel();
    }

    void SceneExplorerModuleController::setSceneEntityWorkflow(SceneEntityWorkflowController* workflow)
    {
        m_sceneEntityWorkflow = workflow;
    }

    void SceneExplorerModuleController::setViewportServices(RobotQtViewerViewportServices* viewportServices)
    {
        m_viewportServices = viewportServices;
    }

    void SceneExplorerModuleController::setViewportInteractionMode(RobotQtViewerViewportInteractionMode mode)
    {
        if(m_interactionMode == mode) {
            return;
        }
        m_interactionMode = mode;
        publishTaskStateChanged(QStringLiteral("sceneExplorerInteractionMode"));
    }

    void SceneExplorerModuleController::setWorkbenchDescriptor(
        const RobotQtViewerWorkbenchDescriptor& descriptor)
    {
        m_workbenchDescriptor = descriptor;
        setViewportInteractionMode(descriptor.defaultViewportMode);
        refreshViewModel();
    }

    void SceneExplorerModuleController::handleEvent(const RobotQtViewerEvent& event)
    {
        switch(event.kind) {
        case RobotQtViewerEventKind::ProjectDocumentChanged:
        case RobotQtViewerEventKind::ViewportReloaded:
        case RobotQtViewerEventKind::RobotRuntimeChanged:
        case RobotQtViewerEventKind::AttachmentChanged:
        case RobotQtViewerEventKind::TaskStateChanged:
            refreshViewModel();
            break;
        default:
            break;
        }
    }

    SceneExplorerNodeRef SceneExplorerModuleController::currentNode() const
    {
        return m_widget.currentNode();
    }

    bool SceneExplorerModuleController::selectNode(const SceneExplorerNodeRef& node)
    {
        return m_widget.selectNode(node);
    }

    QHash<QString, QStringList> SceneExplorerModuleController::robotLinksByRobotId() const
    {
        QHash<QString, QStringList> linksByRobot;
        for(const SceneExplorerRobotRuntimeView& robot : m_robots) {
            linksByRobot.insert(robot.id, robot.links);
        }
        return linksByRobot;
    }

    SceneSelectionIntent SceneExplorerModuleController::selectionIntentForNode(
        const SceneExplorerNodeRef& node) const
    {
        return SceneSelectionController::intentFromNode(m_context.document(), node);
    }

    SceneRobotSelectionContext SceneExplorerModuleController::robotSelectionContext(
        const QString& robotId,
        const QString& preferredLinkName,
        const QString& preferredMountId) const
    {
        return SceneSelectionController::robotSelectionContext(
            m_context.document(),
            robotId,
            preferredLinkName,
            preferredMountId);
    }

    void SceneExplorerModuleController::focusTransformTask(const SceneExplorerNodeRef& node)
    {
        if(node.kind != SceneExplorerNodeKind::Robot &&
            node.kind != SceneExplorerNodeKind::Object &&
            node.kind != SceneExplorerNodeKind::ObjectFrame &&
            node.kind != SceneExplorerNodeKind::PointCloud) {
            emit statusMessageRequested(
                QStringLiteral("Select a robot, scene object, object frame, or point cloud transform."),
                3000);
            return;
        }

        if(node.kind == SceneExplorerNodeKind::Object ||
            node.kind == SceneExplorerNodeKind::ObjectFrame) {
            m_interactionMode = RobotQtViewerViewportInteractionMode::EditTransformPreview;
            if(node.kind == SceneExplorerNodeKind::ObjectFrame) {
                m_objectFrameMode = SceneExplorerObjectFrameMode::Edit;
                captureObjectFrameEditSnapshot(node.id, node.linkName);
                m_context.selectionModel().selectObjectFrame(
                    node.id,
                    node.linkName,
                    QStringLiteral("sceneExplorerObjectFrameEditTask"));
                RobotQtViewerViewportPreviewPayload preview;
                preview.focusObjectFrameObject = true;
                preview.focusObjectFrameObjectId = node.id;
                mutateViewportPreview(preview, QStringLiteral("sceneExplorerTransformTask"));
            }
        }
        publishTaskStateChanged(QStringLiteral("sceneExplorerTransformTask"));
        emit statusMessageRequested(QStringLiteral("Transform editor is ready."), 2500);
    }

    void SceneExplorerModuleController::createObjectFrameForObject(const QString& objectId)
    {
        if(objectId.isEmpty()) {
            emit statusMessageRequested(QStringLiteral("Select an object before adding an object frame."), 3000);
            return;
        }

        const std::string objectIdUtf8 = objectId.toStdString();
        simulation_project::ObjectFrameDesc frame;
        m_objectFrameRollbackDocument = m_context.document();
        m_objectFrameRollbackDirty = m_context.projectSession().isDirty();
        m_hasObjectFrameRollbackDocument = true;
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("createObjectFrame"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                const simulation_project::SceneObjectDesc* object =
                    service.findSceneObject(objectIdUtf8);
                if(object == nullptr) {
                    error = "Object not found.";
                    return false;
                }

                const QString baseId = objectId + QStringLiteral("_frame");
                QString frameId = baseId;
                int suffix = 1;
                while(service.findObjectFrame(objectIdUtf8, frameId.toStdString()) != nullptr) {
                    frameId = QString("%1_%2").arg(baseId).arg(suffix++);
                }

                frame.id = frameId.toStdString();
                frame.name = frame.id;
                if(!service.addObjectFrame(objectIdUtf8, frame, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            clearObjectFrameEditSnapshot();
            emit statusMessageRequested(
                QString("Create object frame failed: %1").arg(mutationResult.message),
                5000);
            return;
        }

        const QString frameId = QString::fromStdString(frame.id);
        m_objectFrameMode = SceneExplorerObjectFrameMode::Create;
        m_objectFrameDraftSourceObjectId = objectId;
        m_activeObjectFrameObjectId = objectId;
        m_activeObjectFrameId = frameId;
        captureObjectFrameEditSnapshot(objectId, frameId, true);
        m_context.selectionModel().selectObjectFrame(
            objectId,
            frameId,
            QStringLiteral("createObjectFrame"));
        m_interactionMode = RobotQtViewerViewportInteractionMode::EditTransformPreview;
        SceneExplorerNodeRef node;
        node.kind = SceneExplorerNodeKind::ObjectFrame;
        node.id = objectId;
        node.name = frameId;
        node.linkName = frameId;
        selectNode(node);
        m_hasPendingTransformPreview = true;
        m_pendingTransformTarget = node;
        m_pendingTransform = frame.objectToFrame;
        RobotQtViewerViewportPreviewPayload preview;
        preview.upsertPreviewObjectFrame = true;
        preview.objectFrameObjectId = objectId;
        preview.objectFrame = frame;
        preview.previewObjectFrameTransform = true;
        preview.previewObjectFrameObjectId = objectId;
        preview.previewObjectFrameId = frameId;
        preview.objectFrameTransform = frame.objectToFrame;
        preview.focusObjectFrameObject = true;
        preview.focusObjectFrameObjectId = objectId;
        mutateViewportPreview(preview, QStringLiteral("createObjectFrame"));
        publishTaskStateChanged(QStringLiteral("createObjectFrame"));
        emit statusMessageRequested(QString("Created object frame: %1").arg(frameId), 3000);
    }

    bool SceneExplorerModuleController::resolvePendingTransformPreviewIfTargetChanges(
        const SceneExplorerNodeRef& nextNode,
        QWidget* parentWidget)
    {
        if(!m_hasPendingTransformPreview || sameTransformTarget(m_pendingTransformTarget, nextNode)) {
            return true;
        }
        const bool resolved = resolvePendingTransformPreview(parentWidget);
        if(!resolved) {
            reassertActiveObjectFrameTask(QStringLiteral("sceneExplorerKeepObjectFrameTask"));
        }
        return resolved;
    }

    bool SceneExplorerModuleController::linkFrameVisible(
        const QString& robotId,
        const QString& linkName) const
    {
        return !robotId.isEmpty() &&
            !linkName.isEmpty() &&
            m_visibleLinkFrameKeys.contains(linkFrameKey(robotId, linkName));
    }

    bool SceneExplorerModuleController::toggleLinkFrameVisible(
        const QString& robotId,
        const QString& linkName)
    {
        if(robotId.isEmpty() || linkName.isEmpty()) {
            return false;
        }

        const QString key = linkFrameKey(robotId, linkName);
        if(m_visibleLinkFrameKeys.contains(key)) {
            m_visibleLinkFrameKeys.remove(key);
            return false;
        }

        m_visibleLinkFrameKeys.insert(key);
        return true;
    }

    bool SceneExplorerModuleController::nodeSelectableForCurrentMode(const SceneExplorerNodeRef& node) const
    {
        return sceneExplorerNodeSelectableForMode(node.kind, m_interactionMode);
    }

    void SceneExplorerModuleController::handleNodeActivated(const SceneExplorerNodeRef& node, int column)
    {
        if(!nodeSelectableForCurrentMode(node)) {
            emit statusMessageRequested(QStringLiteral("This item is not selectable in the current task."), 2500);
            refreshViewModel();
            return;
        }
        emit nodeActivated(node, column);
        refreshViewModel();
    }

    void SceneExplorerModuleController::handleTransformPreviewChanged(
        const SceneExplorerNodeRef& target,
        const simulation_project::TransformDesc& transform)
    {
        if(target.id.isEmpty()) {
            return;
        }
        if(m_hasPendingTransformPreview && !sameTransformTarget(m_pendingTransformTarget, target)) {
            discardPendingTransformPreview();
        }
        m_hasPendingTransformPreview = true;
        m_pendingTransformTarget = target;
        m_pendingTransform = transform;
        RobotQtViewerViewportPreviewPayload preview;
        if(target.kind == SceneExplorerNodeKind::Robot) {
            preview.previewRobotBaseTransform = true;
            preview.robotBaseRobotId = target.id;
            preview.robotBaseTransform = transform;
        } else if(target.kind == SceneExplorerNodeKind::ObjectFrame) {
            preview.previewObjectFrameTransform = true;
            preview.previewObjectFrameObjectId = target.id;
            preview.previewObjectFrameId = target.linkName;
            preview.objectFrameTransform = transform;
        } else if(target.kind == SceneExplorerNodeKind::Object ||
            target.kind == SceneExplorerNodeKind::PointCloud) {
            preview.previewSceneObjectTransform = true;
            preview.sceneObjectId = target.id;
            preview.sceneObjectTransform = transform;
        } else {
            return;
        }
        mutateViewportPreview(preview, QStringLiteral("sceneExplorerTransformPreview"));
    }

    void SceneExplorerModuleController::applyTransformChange(
        const SceneExplorerNodeRef& target,
        const simulation_project::TransformDesc& transform)
    {
        if(target.id.isEmpty()) {
            emit statusMessageRequested(QStringLiteral("Scene transform workflow is not available."), 4000);
            return;
        }
        if(target.kind != SceneExplorerNodeKind::ObjectFrame && m_sceneEntityWorkflow == nullptr) {
            emit statusMessageRequested(QStringLiteral("Scene transform workflow is not available."), 4000);
            return;
        }

        SceneEntityMutationResult result;
        if(target.kind == SceneExplorerNodeKind::Robot) {
            result = m_sceneEntityWorkflow->setRobotBaseTransform(target.id, transform);
        } else if(target.kind == SceneExplorerNodeKind::Object) {
            result = m_sceneEntityWorkflow->setSceneObjectTransform(target.id, transform);
        } else if(target.kind == SceneExplorerNodeKind::PointCloud) {
            result = m_sceneEntityWorkflow->setPointCloudTransform(target.id, transform);
        } else if(target.kind == SceneExplorerNodeKind::ObjectFrame) {
            if(target.name.trimmed().isEmpty()) {
                emit statusMessageRequested(QStringLiteral("Frame name cannot be empty."), 4000);
                return;
            }
            m_interactionMode = RobotQtViewerViewportInteractionMode::Browse;
            simulation_project::ObjectFrameDesc updated;
            const std::string objectId = target.id.toStdString();
            const std::string frameId = target.linkName.toStdString();
            const std::string frameName = qStringToUtf8(target.name.trimmed());
            const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
                QStringLiteral("applyObjectFrameTransform"),
                ProjectDirtyPolicy::UserEdit,
                [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                    const simulation_project::ObjectFrameDesc* existing =
                        service.findObjectFrame(objectId, frameId);
                    if(existing == nullptr) {
                        error = "Object frame not found.";
                        return false;
                    }
                    updated = *existing;
                    updated.name = frameName;
                    updated.objectToFrame = transform;
                    if(updated.name == existing->name &&
                        sameTransform(updated.objectToFrame, existing->objectToFrame)) {
                        changed = false;
                        return true;
                    }
                    if(!service.updateObjectFrame(objectId, frameId, updated, &error)) {
                        return false;
                    }
                    changed = true;
                    return true;
                });
            if(!mutationResult.success) {
                emit statusMessageRequested(
                    QString("Object frame update failed: %1").arg(mutationResult.message),
                    5000);
                return;
            }
            RobotQtViewerViewportPreviewPayload preview;
            preview.clearObjectFrameObjectFocus = true;
            preview.upsertPreviewObjectFrame = true;
            preview.objectFrameObjectId = target.id;
            preview.objectFrame = updated;
            result.success = true;
            if(mutationResult.changed) {
                m_context.selectionModel().selectObjectFrame(
                    target.id,
                    target.linkName,
                    QStringLiteral("applyObjectFrameTransform"));
                result.message = QString("Updated object frame: %1").arg(QString::fromStdString(updated.name));
            } else {
                result.message = QString("Object frame unchanged: %1").arg(target.name.trimmed());
            }
            mutateViewportPreview(preview, QStringLiteral("applyObjectFrameTransform"));
            m_objectFrameMode = SceneExplorerObjectFrameMode::Selection;
            clearObjectFrameEditSnapshot();
        } else {
            emit statusMessageRequested(
                QStringLiteral("Select a robot, scene object, object frame, or point cloud transform."),
                3000);
            return;
        }

        if(!result.success) {
            emit statusMessageRequested(result.message, 5000);
            return;
        }

        if((target.kind == SceneExplorerNodeKind::Object ||
               target.kind == SceneExplorerNodeKind::PointCloud) &&
            m_viewportServices != nullptr) {
            m_viewportServices->commitSceneObjectTransform(target.id, transform);
        }

        if(target.kind == SceneExplorerNodeKind::Object) {
            m_interactionMode = RobotQtViewerViewportInteractionMode::Browse;
            publishTaskStateChanged(QStringLiteral("sceneExplorerApplyObjectTransform"));
        }
        if(hasPendingTransformPreviewFor(target)) {
            m_hasPendingTransformPreview = false;
            m_pendingTransformTarget = SceneExplorerNodeRef();
            m_pendingTransform = simulation_project::TransformDesc();
        }
        refreshViewModel();
        emit statusMessageRequested(result.message, 3000);
    }

    void SceneExplorerModuleController::cancelTransformChange(const SceneExplorerNodeRef& target)
    {
        if(target.kind == SceneExplorerNodeKind::ObjectFrame && m_objectFrameEditSnapshotIsNew) {
            restoreObjectFrameDraft(QStringLiteral("cancelObjectFrameDraft"));
            emit statusMessageRequested(QStringLiteral("Object frame creation canceled."), 2500);
            return;
        }

        simulation_project::TransformDesc transform;
        if(transformForNode(m_context.document(), target, transform)) {
            RobotQtViewerViewportPreviewPayload preview;
            if(target.kind == SceneExplorerNodeKind::Robot) {
                preview.previewRobotBaseTransform = true;
                preview.robotBaseRobotId = target.id;
                preview.robotBaseTransform = transform;
            } else if(target.kind == SceneExplorerNodeKind::ObjectFrame) {
                preview.previewObjectFrameTransform = true;
                preview.previewObjectFrameObjectId = target.id;
                preview.previewObjectFrameId = target.linkName;
                preview.objectFrameTransform = transform;
                preview.clearObjectFrameObjectFocus = true;
                if(const simulation_project::ObjectFrameDesc* frame =
                       findObjectFrameDesc(m_context.document(), target.id, target.linkName)) {
                    preview.upsertPreviewObjectFrame = true;
                    preview.objectFrameObjectId = target.id;
                    preview.objectFrame = *frame;
                }
            } else if(target.kind == SceneExplorerNodeKind::Object ||
                target.kind == SceneExplorerNodeKind::PointCloud) {
                preview.previewSceneObjectTransform = true;
                preview.sceneObjectId = target.id;
                preview.sceneObjectTransform = transform;
            } else {
                return;
            }
            mutateViewportPreview(preview, QStringLiteral("sceneExplorerCancelTransform"));
        }
        if(target.kind == SceneExplorerNodeKind::Object ||
            target.kind == SceneExplorerNodeKind::ObjectFrame) {
            m_interactionMode = RobotQtViewerViewportInteractionMode::Browse;
        }
        if(target.kind == SceneExplorerNodeKind::ObjectFrame) {
            m_objectFrameMode = SceneExplorerObjectFrameMode::Selection;
            clearObjectFrameEditSnapshot();
        }
        if(hasPendingTransformPreviewFor(target)) {
            m_hasPendingTransformPreview = false;
            m_pendingTransformTarget = SceneExplorerNodeRef();
            m_pendingTransform = simulation_project::TransformDesc();
        }
        publishTaskStateChanged(QStringLiteral("sceneExplorerCancelTransform"));
        emit statusMessageRequested(QStringLiteral("Transform edits canceled."), 2500);
    }

    void SceneExplorerModuleController::handleObjectFrameVisibilityChanged(
        const SceneExplorerNodeRef& target,
        bool visible)
    {
        if(target.kind != SceneExplorerNodeKind::ObjectFrame ||
            target.id.isEmpty() ||
            target.linkName.isEmpty()) {
            emit statusMessageRequested(QStringLiteral("Select an object frame first."), 3000);
            refreshViewModel();
            return;
        }

        const std::string objectId = target.id.toStdString();
        const std::string frameId = target.linkName.toStdString();
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("setObjectFrameVisibility"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                const simulation_project::ObjectFrameDesc* existing =
                    service.findObjectFrame(objectId, frameId);
                if(existing == nullptr) {
                    error = "Object frame not found: " + frameId;
                    return false;
                }
                if(existing->visible == visible) {
                    changed = false;
                    return true;
                }
                if(!service.setObjectFrameVisibility(objectId, frameId, visible, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            emit statusMessageRequested(
                QString("Object frame visibility update failed: %1").arg(mutationResult.message),
                5000);
            refreshViewModel();
            return;
        }

        if(const simulation_project::ObjectFrameDesc* frame =
               findObjectFrameDesc(m_context.document(), target.id, target.linkName)) {
            RobotQtViewerViewportPreviewPayload preview;
            preview.upsertPreviewObjectFrame = true;
            preview.objectFrameObjectId = target.id;
            preview.objectFrame = *frame;
            mutateViewportPreview(preview, QStringLiteral("setObjectFrameVisibility"));
        }

        emit statusMessageRequested(
            visible
                ? QStringLiteral("Object frame display enabled.")
                : QStringLiteral("Object frame display disabled."),
            2500);
    }

    bool SceneExplorerModuleController::sameTransformTarget(
        const SceneExplorerNodeRef& lhs,
        const SceneExplorerNodeRef& rhs) const
    {
        return lhs.kind == rhs.kind && lhs.id == rhs.id && lhs.linkName == rhs.linkName;
    }

    bool SceneExplorerModuleController::hasPendingTransformPreviewFor(const SceneExplorerNodeRef& target) const
    {
        return m_hasPendingTransformPreview && sameTransformTarget(m_pendingTransformTarget, target);
    }

    bool SceneExplorerModuleController::resolvePendingTransformPreview(QWidget* parentWidget)
    {
        if(!m_hasPendingTransformPreview) {
            return true;
        }

        const QMessageBox::StandardButton button = QMessageBox::question(
            parentWidget,
            QStringLiteral("Unsaved Transform Changes"),
            QStringLiteral("Save transform changes before changing selection?"),
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
            QMessageBox::Save);
        if(button == QMessageBox::Cancel) {
            return false;
        }

        const SceneExplorerNodeRef target = m_pendingTransformTarget;
        const simulation_project::TransformDesc transform = m_pendingTransform;
        if(button == QMessageBox::Discard) {
            discardPendingTransformPreview();
            publishTaskStateChanged(QStringLiteral("sceneExplorerDiscardTransform"));
            emit statusMessageRequested(QStringLiteral("Transform edits discarded."), 2500);
            return true;
        }

        applyTransformChange(target, transform);
        return !m_hasPendingTransformPreview;
    }

    void SceneExplorerModuleController::discardPendingTransformPreview()
    {
        if(!m_hasPendingTransformPreview) {
            return;
        }

        const SceneExplorerNodeRef target = m_pendingTransformTarget;
        if(target.kind == SceneExplorerNodeKind::ObjectFrame && m_objectFrameEditSnapshotIsNew) {
            restoreObjectFrameDraft(QStringLiteral("discardObjectFrameDraft"));
            return;
        }

        m_hasPendingTransformPreview = false;
        m_pendingTransformTarget = SceneExplorerNodeRef();
        m_pendingTransform = simulation_project::TransformDesc();

        simulation_project::TransformDesc transform;
        if(transformForNode(m_context.document(), target, transform)) {
            RobotQtViewerViewportPreviewPayload preview;
            if(target.kind == SceneExplorerNodeKind::Robot) {
                preview.previewRobotBaseTransform = true;
                preview.robotBaseRobotId = target.id;
                preview.robotBaseTransform = transform;
            } else if(target.kind == SceneExplorerNodeKind::ObjectFrame) {
                preview.previewObjectFrameTransform = true;
                preview.previewObjectFrameObjectId = target.id;
                preview.previewObjectFrameId = target.linkName;
                preview.objectFrameTransform = transform;
                preview.clearObjectFrameObjectFocus = true;
                if(const simulation_project::ObjectFrameDesc* frame =
                       findObjectFrameDesc(m_context.document(), target.id, target.linkName)) {
                    preview.upsertPreviewObjectFrame = true;
                    preview.objectFrameObjectId = target.id;
                    preview.objectFrame = *frame;
                }
            } else if(target.kind == SceneExplorerNodeKind::Object ||
                target.kind == SceneExplorerNodeKind::PointCloud) {
                preview.previewSceneObjectTransform = true;
                preview.sceneObjectId = target.id;
                preview.sceneObjectTransform = transform;
            } else {
                return;
            }
            mutateViewportPreview(preview, QStringLiteral("sceneExplorerDiscardTransform"));
        }
        if(target.kind == SceneExplorerNodeKind::Object ||
            target.kind == SceneExplorerNodeKind::ObjectFrame) {
            m_interactionMode = RobotQtViewerViewportInteractionMode::Browse;
        }
    }

    void SceneExplorerModuleController::publishTaskStateChanged(const QString& sourceId)
    {
        RobotQtViewerEvent event;
        event.kind = RobotQtViewerEventKind::TaskStateChanged;
        event.sourceId = sourceId;
        m_context.eventHub().publish(event);
    }

    void SceneExplorerModuleController::mutateViewportPreview(
        const RobotQtViewerViewportPreviewPayload& preview,
        const QString& sourceId)
    {
        RobotQtViewerViewportPreviewPayload mergedPreview = preview;
        mergeActiveObjectFrameViewportState(mergedPreview);
        m_context.viewportPreviewState().mutate(mergedPreview, sourceId);
    }

    bool SceneExplorerModuleController::objectFrameTaskActive() const
    {
        return (m_objectFrameMode == SceneExplorerObjectFrameMode::Create ||
                   m_objectFrameMode == SceneExplorerObjectFrameMode::Edit) &&
            !m_activeObjectFrameObjectId.isEmpty() &&
            !m_activeObjectFrameId.isEmpty();
    }

    void SceneExplorerModuleController::mergeActiveObjectFrameViewportState(
        RobotQtViewerViewportPreviewPayload& preview) const
    {
        if(!objectFrameTaskActive() || preview.clearObjectFrameObjectFocus) {
            return;
        }

        if(!preview.focusObjectFrameObject) {
            preview.focusObjectFrameObject = true;
            preview.focusObjectFrameObjectId = m_activeObjectFrameObjectId;
        }
        if(!preview.previewObjectFrameTransform) {
            preview.previewObjectFrameTransform = true;
            preview.previewObjectFrameObjectId = m_activeObjectFrameObjectId;
            preview.previewObjectFrameId = m_activeObjectFrameId;
            preview.objectFrameTransform = m_hasPendingTransformPreview &&
                    m_pendingTransformTarget.kind == SceneExplorerNodeKind::ObjectFrame &&
                    m_pendingTransformTarget.id == m_activeObjectFrameObjectId &&
                    m_pendingTransformTarget.linkName == m_activeObjectFrameId
                ? m_pendingTransform
                : m_objectFrameEditSnapshot.objectToFrame;
        }
    }

    void SceneExplorerModuleController::reassertActiveObjectFrameTask(const QString& sourceId)
    {
        if(!objectFrameTaskActive()) {
            return;
        }

        SceneExplorerNodeRef node;
        node.kind = SceneExplorerNodeKind::ObjectFrame;
        node.id = m_activeObjectFrameObjectId;
        node.name = m_activeObjectFrameId;
        node.linkName = m_activeObjectFrameId;
        selectNode(node);
        m_context.selectionModel().selectObjectFrame(
            m_activeObjectFrameObjectId,
            m_activeObjectFrameId,
            sourceId);

        RobotQtViewerViewportPreviewPayload preview;
        mergeActiveObjectFrameViewportState(preview);
        m_context.viewportPreviewState().mutate(preview, sourceId);
    }

    void SceneExplorerModuleController::captureObjectFrameEditSnapshot(
        const QString& objectId,
        const QString& frameId,
        bool newFrame)
    {
        const simulation_project::SceneObjectDesc* object = nullptr;
        for(const simulation_project::SceneObjectDesc& candidate : m_context.document().objects) {
            if(QString::fromStdString(candidate.id) == objectId) {
                object = &candidate;
                break;
            }
        }
        if(object == nullptr) {
            clearObjectFrameEditSnapshot();
            return;
        }

        const std::string frameIdValue = frameId.toStdString();
        const auto frameIt = std::find_if(
            object->objectFrames.begin(),
            object->objectFrames.end(),
            [&](const simulation_project::ObjectFrameDesc& frame) {
                return frame.id == frameIdValue;
            });
        if(frameIt == object->objectFrames.end()) {
            clearObjectFrameEditSnapshot();
            return;
        }

        m_hasObjectFrameEditSnapshot = true;
        m_objectFrameEditSnapshotIsNew = newFrame;
        m_activeObjectFrameObjectId = objectId;
        m_activeObjectFrameId = frameId;
        m_objectFrameEditSnapshot = *frameIt;
    }

    void SceneExplorerModuleController::clearObjectFrameEditSnapshot()
    {
        m_hasObjectFrameEditSnapshot = false;
        m_objectFrameEditSnapshotIsNew = false;
        m_hasObjectFrameRollbackDocument = false;
        m_objectFrameRollbackDirty = false;
        m_activeObjectFrameObjectId.clear();
        m_activeObjectFrameId.clear();
        m_objectFrameDraftSourceObjectId.clear();
        m_objectFrameEditSnapshot = simulation_project::ObjectFrameDesc();
        m_objectFrameRollbackDocument = simulation_project::ProjectDocument();
    }

    bool SceneExplorerModuleController::restoreObjectFrameDraft(const QString& sourceId)
    {
        const QString sourceObjectId = m_objectFrameDraftSourceObjectId;
        if(m_hasObjectFrameRollbackDocument) {
            m_context.documentController().restoreProjectSnapshot(
                sourceId,
                m_objectFrameRollbackDocument,
                m_objectFrameRollbackDirty);
        }

        RobotQtViewerViewportPreviewPayload preview;
        preview.clearObjectFrameObjectFocus = true;
        mutateViewportPreview(preview, sourceId);

        m_hasPendingTransformPreview = false;
        m_pendingTransformTarget = SceneExplorerNodeRef();
        m_pendingTransform = simulation_project::TransformDesc();
        m_interactionMode = RobotQtViewerViewportInteractionMode::Browse;
        m_objectFrameMode = SceneExplorerObjectFrameMode::Selection;
        clearObjectFrameEditSnapshot();
        if(!sourceObjectId.isEmpty()) {
            SceneExplorerNodeRef objectNode;
            objectNode.kind = SceneExplorerNodeKind::Object;
            objectNode.id = sourceObjectId;
            objectNode.name = sourceObjectId;
            selectNode(objectNode);
            m_context.selectionModel().selectSceneObject(sourceObjectId, sourceId);
        }
        publishTaskStateChanged(sourceId);
        refreshViewModel();
        return true;
    }

    void SceneExplorerModuleController::showContextMenu(const QPoint& pos)
    {
        QTreeWidget* tree = m_widget.treeWidget();
        if(tree == nullptr) {
            return;
        }

        QTreeWidgetItem* item = SceneTreeIntentController::treeItemAtOrCurrent(tree, pos);
        if(item == nullptr) {
            return;
        }

        tree->setCurrentItem(item);
        if(!resolvePendingTransformPreviewIfTargetChanges(m_widget.currentNode(), tree)) {
            refreshViewModel();
            return;
        }
        if(!nodeSelectableForCurrentMode(m_widget.currentNode())) {
            emit statusMessageRequested(QStringLiteral("This item is not selectable in the current task."), 2500);
            return;
        }
        SceneTreeIntentController::ContextMenuModel model =
            SceneTreeIntentController::contextMenuModelFromItem(
                item,
                !m_context.document().collision.selectionSets.empty(),
                sceneExplorerActionScopeForWorkbench(m_workbenchDescriptor));
        if(model.actions.empty()) {
            return;
        }

        for(SceneTreeIntentController::ContextMenuActionView& actionView : model.actions) {
            if(actionView.action.kind == SceneTreeIntentController::ContextMenuActionKind::ShowLinkFrame) {
                actionView.checkable = true;
                actionView.checked = linkFrameVisible(actionView.action.node.id, actionView.action.node.linkName);
            }
        }

        QMenu menu(tree);
        for(const SceneTreeIntentController::ContextMenuActionView& actionView : model.actions) {
            if(actionView.separatorBefore && !menu.actions().empty()) {
                menu.addSeparator();
            }
            QAction* action = menu.addAction(actionView.label);
            action->setEnabled(actionView.enabled);
            action->setCheckable(actionView.checkable);
            action->setChecked(actionView.checked);
            connect(action, &QAction::triggered, this, [this, actionView]() {
                emit contextMenuActionRequested(actionView.action);
            });
        }
        menu.exec(tree->viewport()->mapToGlobal(pos));
    }

    QString SceneExplorerModuleController::linkFrameKey(
        const QString& robotId,
        const QString& linkName) const
    {
        return robotId + QStringLiteral("\n") + linkName;
    }
}
