#include "SceneCollisionTargetResolver.h"

namespace robot_qt_viewer
{
    bool sceneExplorerNodeKindCanBeCollisionTarget(SceneExplorerNodeKind kind)
    {
        return kind == SceneExplorerNodeKind::Robot ||
            kind == SceneExplorerNodeKind::Link ||
            kind == SceneExplorerNodeKind::Object ||
            kind == SceneExplorerNodeKind::PointCloud ||
            kind == SceneExplorerNodeKind::ToolAttachment;
    }

    bool sceneExplorerNodeKindCanConfigureCollisionModel(SceneExplorerNodeKind kind)
    {
        return kind == SceneExplorerNodeKind::Robot ||
            kind == SceneExplorerNodeKind::Link ||
            kind == SceneExplorerNodeKind::Object ||
            kind == SceneExplorerNodeKind::ToolAttachment;
    }

    bool sceneExplorerNodeKindIsPrimaryCollisionTarget(SceneExplorerNodeKind kind)
    {
        return kind == SceneExplorerNodeKind::Link ||
            kind == SceneExplorerNodeKind::Object ||
            kind == SceneExplorerNodeKind::PointCloud ||
            kind == SceneExplorerNodeKind::ToolAttachment;
    }

    bool sceneExplorerNodeToCollisionTarget(
        const SceneExplorerNodeRef& node,
        simulation_project::CollisionTargetRef& target,
        QString* errorMessage)
    {
        target = simulation_project::CollisionTargetRef();
        if(node.id.isEmpty()) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral("Selected item has no collision target id.");
            }
            return false;
        }

        if(node.kind == SceneExplorerNodeKind::Object) {
            target = simulation_project::makeSceneObjectCollisionTarget(node.id.toStdString());
        } else if(node.kind == SceneExplorerNodeKind::PointCloud) {
            target = simulation_project::makePointCloudCollisionTarget(node.id.toStdString());
        } else if(node.kind == SceneExplorerNodeKind::ToolAttachment) {
            target = simulation_project::makeAttachmentCollisionTarget(node.id.toStdString());
        } else if(node.kind == SceneExplorerNodeKind::Robot) {
            target = simulation_project::makeRobotCollisionTarget(node.id.toStdString());
        } else if(node.kind == SceneExplorerNodeKind::Link) {
            if(node.linkName.isEmpty()) {
                if(errorMessage != nullptr) {
                    *errorMessage = QStringLiteral("Selected robot link has no link name.");
                }
                return false;
            }
            target = simulation_project::makeRobotLinkCollisionTarget(
                node.id.toStdString(),
                node.linkName.toStdString());
        } else {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral("Select a robot, link, object, point cloud, or attachment.");
            }
            return false;
        }

        if(!simulation_project::isValidCollisionTarget(target)) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral("Selected item has no valid collision target.");
            }
            return false;
        }
        return true;
    }

    bool sceneExplorerNodeToCollisionSelectionMember(
        const SceneExplorerNodeRef& node,
        simulation_project::CollisionSelectionSetMemberDesc& member,
        QString* errorMessage)
    {
        member = simulation_project::CollisionSelectionSetMemberDesc();
        simulation_project::CollisionTargetRef target;
        if(!sceneExplorerNodeToCollisionTarget(node, target, errorMessage)) {
            return false;
        }

        member = simulation_project::collisionSelectionSetMemberFromTarget(target);
        if(member.robotId.empty() && member.objectId.empty() && member.attachmentId.empty()) {
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral("Selected item has no collision member id.");
            }
            return false;
        }
        return true;
    }
}
