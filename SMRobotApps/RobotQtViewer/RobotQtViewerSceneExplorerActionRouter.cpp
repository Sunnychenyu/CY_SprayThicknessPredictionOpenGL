#include "RobotQtViewerSceneExplorerActionRouter.h"

#include "CollisionWorkbenchModuleController.h"
#include "SceneCollisionTargetResolver.h"
#include "SceneExplorerModuleController.h"
#include "ToolSetupModuleController.h"

#include <utility>

namespace robot_qt_viewer
{
    void RobotQtViewerSceneExplorerActionRouter::setParentWidget(QWidget* parentWidget)
    {
        m_parentWidget = parentWidget;
    }

    void RobotQtViewerSceneExplorerActionRouter::setSceneExplorerController(
        SceneExplorerModuleController* controller)
    {
        m_sceneExplorerController = controller;
    }

    void RobotQtViewerSceneExplorerActionRouter::setToolSetupController(ToolSetupModuleController* controller)
    {
        m_toolSetupController = controller;
    }

    void RobotQtViewerSceneExplorerActionRouter::setCollisionWorkbenchController(
        CollisionWorkbenchModuleController* controller)
    {
        m_collisionWorkbenchController = controller;
    }

    void RobotQtViewerSceneExplorerActionRouter::setEnterToolSetupWorkbenchCallback(VoidCallback callback)
    {
        m_enterToolSetupWorkbenchCallback = std::move(callback);
    }

    void RobotQtViewerSceneExplorerActionRouter::setDeleteSelectedEntityCallback(VoidCallback callback)
    {
        m_deleteSelectedEntityCallback = std::move(callback);
    }

    void RobotQtViewerSceneExplorerActionRouter::setReloadViewportCallback(VoidCallback callback)
    {
        m_reloadViewportCallback = std::move(callback);
    }

    void RobotQtViewerSceneExplorerActionRouter::setSelectRobotContextCallback(RobotLinkCallback callback)
    {
        m_selectRobotContextCallback = std::move(callback);
    }

    void RobotQtViewerSceneExplorerActionRouter::setSelectObjectContextCallback(ObjectCallback callback)
    {
        m_selectObjectContextCallback = std::move(callback);
    }

    void RobotQtViewerSceneExplorerActionRouter::handleAction(
        const SceneTreeIntentController::ContextMenuAction& action,
        StatusCallback statusCallback)
    {
        const auto showStatus = [&](const QString& message, int timeoutMs) {
            if(statusCallback) {
                statusCallback(message, timeoutMs);
            }
        };

        switch(action.kind) {
        case SceneTreeIntentController::ContextMenuActionKind::ConfigureRobotFlange:
            if(m_toolSetupController == nullptr) {
                showStatus(QStringLiteral("Frame Editor is not available."), 3000);
                return;
            }
            if(m_enterToolSetupWorkbenchCallback) {
                m_enterToolSetupWorkbenchCallback();
            }
            if(action.node.kind == SceneExplorerNodeKind::RobotMount && m_sceneExplorerController != nullptr) {
                const SceneSelectionIntent intent = m_sceneExplorerController->selectionIntentForNode(action.node);
                if(intent.kind != SceneSelectionIntentKind::SelectRobotMount) {
                    showStatus(QStringLiteral("Mount frame is not available."), 3000);
                    return;
                }
                m_toolSetupController->focusRobotMountTask(intent.robotId, intent.linkName, intent.mountId);
            } else if(action.node.kind == SceneExplorerNodeKind::Link && !action.node.linkName.isEmpty()) {
                m_toolSetupController->focusRobotMountTask(action.node.id, action.node.linkName);
                m_toolSetupController->createRobotMountForSelectedLink();
            } else {
                m_toolSetupController->focusRobotMountTask(
                    action.node.id,
                    action.node.linkName.isEmpty() && !action.robotLinks.isEmpty()
                        ? action.robotLinks.first()
                        : action.node.linkName);
            }
            break;
        case SceneTreeIntentController::ContextMenuActionKind::MoveRobotBase:
            if(m_sceneExplorerController != nullptr) {
                m_sceneExplorerController->focusTransformTask(action.node);
            }
            break;
        case SceneTreeIntentController::ContextMenuActionKind::DeleteRobot:
            if(m_deleteSelectedEntityCallback) {
                m_deleteSelectedEntityCallback();
            }
            break;
        case SceneTreeIntentController::ContextMenuActionKind::MoveSceneObject:
            if(m_sceneExplorerController != nullptr) {
                m_sceneExplorerController->focusTransformTask(action.node);
            }
            break;
        case SceneTreeIntentController::ContextMenuActionKind::AddObjectFrame:
            if(m_sceneExplorerController != nullptr) {
                m_sceneExplorerController->createObjectFrameForObject(action.node.id);
            }
            break;
        case SceneTreeIntentController::ContextMenuActionKind::EditObjectFrame:
            if(m_sceneExplorerController != nullptr) {
                m_sceneExplorerController->focusTransformTask(action.node);
            }
            break;
        case SceneTreeIntentController::ContextMenuActionKind::BindItemToMount:
            if(m_toolSetupController == nullptr) {
                showStatus(QStringLiteral("Frame Editor is not available."), 3000);
                return;
            }
            if(m_enterToolSetupWorkbenchCallback) {
                m_enterToolSetupWorkbenchCallback();
            }
            m_toolSetupController->focusObjectBindingTask(action.node.id);
            break;
        case SceneTreeIntentController::ContextMenuActionKind::BindObjectToMount:
            if(m_toolSetupController == nullptr) {
                showStatus(QStringLiteral("Frame Editor is not available."), 3000);
                return;
            }
            if(m_enterToolSetupWorkbenchCallback) {
                m_enterToolSetupWorkbenchCallback();
            }
            m_toolSetupController->focusObjectBindingTask(
                QString(),
                action.node.id,
                action.node.kind == SceneExplorerNodeKind::ObjectFrame
                    ? action.node.linkName
                    : QString());
            break;
        case SceneTreeIntentController::ContextMenuActionKind::CreateToolAssetFromObject:
            if(m_toolSetupController != nullptr) {
                m_toolSetupController->createToolAssetFromSceneObject(action.node.id);
            }
            break;
        case SceneTreeIntentController::ContextMenuActionKind::UnbindMountedAttachment:
            if(m_toolSetupController == nullptr) {
                showStatus(QStringLiteral("Frame Editor is not available."), 3000);
                return;
            }
            m_toolSetupController->unbindMountedAttachment(action.node.id);
            break;
        case SceneTreeIntentController::ContextMenuActionKind::DeleteObject:
        case SceneTreeIntentController::ContextMenuActionKind::DeletePointCloud:
            if(m_deleteSelectedEntityCallback) {
                m_deleteSelectedEntityCallback();
            }
            break;
        case SceneTreeIntentController::ContextMenuActionKind::ShowLinkFrame:
            showStatus(QStringLiteral("Link frame display is handled by the main window."), 3000);
            break;
        case SceneTreeIntentController::ContextMenuActionKind::ConfigureCollisionModel:
            if(!sceneExplorerNodeKindCanConfigureCollisionModel(action.node.kind)) {
                showStatus(QStringLiteral("Select a robot, link, object, or attachment to configure collision geometry."), 3000);
                return;
            }
            if(m_collisionWorkbenchController == nullptr) {
                showStatus(QStringLiteral("Collision configuration is not available."), 3000);
                return;
            }
            if((action.node.kind == SceneExplorerNodeKind::Robot ||
                    action.node.kind == SceneExplorerNodeKind::Link) &&
                m_selectRobotContextCallback) {
                const QString linkName = !action.node.linkName.isEmpty()
                    ? action.node.linkName
                    : (!action.robotLinks.isEmpty() ? action.robotLinks.first() : QString());
                if(linkName.isEmpty()) {
                    showStatus(QStringLiteral("Select a robot link to configure collision geometry."), 3000);
                    return;
                }
                m_selectRobotContextCallback(action.node.id, linkName);
            } else if(action.node.kind == SceneExplorerNodeKind::Object && m_selectObjectContextCallback) {
                m_selectObjectContextCallback(action.node.id);
            } else if(action.node.kind == SceneExplorerNodeKind::ToolAttachment &&
                m_toolSetupController != nullptr) {
                m_toolSetupController->selectToolAttachmentById(action.node.id.toStdString());
            }
            m_collisionWorkbenchController->showCollisionModelConfiguration();
            showStatus(QStringLiteral("Collision model configuration opened."), 3000);
            break;
        case SceneTreeIntentController::ContextMenuActionKind::AddToDetectorSetA:
            if(!action.hasCollisionMember) {
                showStatus(QStringLiteral("Select a robot, link, object, or attachment."), 3000);
                return;
            }
            if(m_collisionWorkbenchController != nullptr) {
                m_collisionWorkbenchController->addMemberToDetectorDraftSet(
                    QStringLiteral("A"),
                    action.displayName,
                    action.robotLinks,
                    action.collisionMember);
            }
            break;
        case SceneTreeIntentController::ContextMenuActionKind::AddToDetectorSetB:
            if(!action.hasCollisionMember) {
                showStatus(QStringLiteral("Select a robot, link, object, or attachment."), 3000);
                return;
            }
            if(m_collisionWorkbenchController != nullptr) {
                m_collisionWorkbenchController->addMemberToDetectorDraftSet(
                    QStringLiteral("B"),
                    action.displayName,
                    action.robotLinks,
                    action.collisionMember);
            }
            break;
        }
    }
}
