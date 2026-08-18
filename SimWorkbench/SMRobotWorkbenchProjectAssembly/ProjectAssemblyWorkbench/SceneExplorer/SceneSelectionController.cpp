#include "SceneSelectionController.h"

#include <SimulationProject/ProjectDocument.h>

#include <algorithm>

namespace robot_qt_viewer
{
    namespace
    {
        const simulation_project::RobotMountDesc* findRobotMountDesc(
            const simulation_project::ProjectDocument& document,
            const std::string& mountId)
        {
            const auto it = std::find_if(
                document.robotMounts.begin(),
                document.robotMounts.end(),
                [&](const simulation_project::RobotMountDesc& mount) {
                    return mount.id == mountId;
                });
            return it == document.robotMounts.end() ? nullptr : &(*it);
        }

        const simulation_project::RobotMountDesc* resolveRobotMountNode(
            const simulation_project::ProjectDocument& document,
            const SceneExplorerNodeRef& node)
        {
            const simulation_project::RobotMountDesc* mount =
                findRobotMountDesc(document, node.id.toStdString());
            if(mount != nullptr || node.linkName.isEmpty()) {
                return mount;
            }

            const std::string linkName = node.linkName.toStdString();
            const std::string nodeName = node.name.toStdString();
            const std::string nodeId = node.id.toStdString();
            const simulation_project::RobotMountDesc* match = nullptr;
            for(const simulation_project::RobotMountDesc& candidate : document.robotMounts) {
                if(candidate.linkName != linkName) {
                    continue;
                }
                if(candidate.id != nodeName &&
                    candidate.name != nodeName &&
                    candidate.name != nodeId) {
                    continue;
                }
                if(match != nullptr) {
                    return nullptr;
                }
                match = &candidate;
            }
            return match;
        }

        const simulation_project::AttachmentAssetDesc* findAttachmentAssetDesc(
            const simulation_project::ProjectDocument& document,
            const std::string& assetId)
        {
            const auto it = std::find_if(
                document.attachmentAssets.begin(),
                document.attachmentAssets.end(),
                [&](const simulation_project::AttachmentAssetDesc& asset) {
                    return asset.id == assetId;
                });
            return it == document.attachmentAssets.end() ? nullptr : &(*it);
        }
    }

    SceneSelectionIntent SceneSelectionController::intentFromNode(
        const simulation_project::ProjectDocument& document,
        const SceneExplorerNodeRef& node)
    {
        SceneSelectionIntent intent;
        intent.itemId = node.id;

        if(node.id.isEmpty()) {
            intent.kind = SceneSelectionIntentKind::Clear;
            return intent;
        }

        if(node.kind == SceneExplorerNodeKind::Object ||
            node.kind == SceneExplorerNodeKind::PointCloud) {
            intent.kind = SceneSelectionIntentKind::SelectSceneObject;
            intent.statusMessage = node.kind == SceneExplorerNodeKind::PointCloud
                ? QString("Selected point cloud %1").arg(node.id)
                : QString("Selected object %1").arg(node.id);
            return intent;
        }

        if(node.kind == SceneExplorerNodeKind::ObjectFrame) {
            intent.kind = SceneSelectionIntentKind::SelectObjectFrame;
            intent.itemId = node.id;
            intent.linkName = node.linkName;
            intent.statusMessage = QString("Selected object frame %1.%2").arg(node.id, node.linkName);
            return intent;
        }

        if(node.kind == SceneExplorerNodeKind::ToolAsset) {
            intent.kind = SceneSelectionIntentKind::SelectToolAsset;
            intent.statusMessage = QString("Selected attachment asset %1").arg(node.id);
            return intent;
        }

        if(node.kind == SceneExplorerNodeKind::Joint) {
            intent.kind = SceneSelectionIntentKind::SelectRobotJoint;
            intent.robotId = node.id;
            intent.jointName = node.name;
            intent.itemId = node.name;
            intent.statusMessage = QString("Selected joint %1.%2").arg(node.id, node.name);
            return intent;
        }

        if(node.kind == SceneExplorerNodeKind::RobotMount) {
            const simulation_project::RobotMountDesc* mount = resolveRobotMountNode(document, node);
            if(mount == nullptr) {
                intent.kind = SceneSelectionIntentKind::None;
                intent.statusMessage = QString("Mount frame not found: %1").arg(node.id);
                return intent;
            }
            intent.kind = SceneSelectionIntentKind::SelectRobotMount;
            intent.itemId = QString::fromStdString(mount->id);
            intent.robotId = QString::fromStdString(mount->robotId);
            intent.linkName = QString::fromStdString(mount->linkName);
            intent.mountId = intent.itemId;
            intent.statusMessage = QString("Selected mount frame %1").arg(intent.mountId);
            return intent;
        }

        if(node.kind == SceneExplorerNodeKind::ToolAttachment) {
            intent.kind = SceneSelectionIntentKind::SelectMountedAttachment;
            intent.itemId = node.id;
            intent.statusMessage = QString("Selected attachment %1").arg(node.id);
            return intent;
        }

        intent.kind = SceneSelectionIntentKind::SelectRobotLink;
        intent.robotId = node.id;
        intent.linkName = node.linkName;
        intent.statusMessage = node.linkName.isEmpty()
            ? QString("Selected %1").arg(node.id)
            : QString("Selected %1.%2").arg(node.id, node.linkName);
        return intent;
    }

    SceneExplorerNodeRef SceneSelectionController::nodeFromViewportPick(
        const QString& kind,
        const QString& robotId,
        const QString& linkName,
        const QString& robotMountId,
        const QString& mountedAttachmentId,
        const QString& sceneObjectId)
    {
        SceneExplorerNodeRef node;
        if(kind == QStringLiteral("robot")) {
            node.kind = SceneExplorerNodeKind::Robot;
            node.id = robotId;
            node.name = robotId;
        } else if(kind == QStringLiteral("robotLink")) {
            node.kind = SceneExplorerNodeKind::Link;
            node.id = robotId;
            node.name = robotId;
            node.linkName = linkName;
        } else if(kind == QStringLiteral("robotMount")) {
            node.kind = SceneExplorerNodeKind::RobotMount;
            node.id = robotMountId;
            node.name = robotMountId;
            node.linkName = linkName;
        } else if(kind == QStringLiteral("mountedAttachment")) {
            node.kind = SceneExplorerNodeKind::ToolAttachment;
            node.id = mountedAttachmentId;
            node.name = mountedAttachmentId;
            node.linkName = linkName;
        } else if(kind == QStringLiteral("sceneObject")) {
            node.kind = SceneExplorerNodeKind::Object;
            node.id = sceneObjectId;
            node.name = sceneObjectId;
        } else if(kind == QStringLiteral("pointCloud")) {
            node.kind = SceneExplorerNodeKind::PointCloud;
            node.id = sceneObjectId;
            node.name = sceneObjectId;
        }
        return node;
    }

    QString SceneSelectionController::preferredMountIdForRobot(
        const simulation_project::ProjectDocument& document,
        const QString& robotId,
        const QString& preferredLinkName,
        const QString& preferredMountId)
    {
        if(robotId.isEmpty()) {
            return QString();
        }

        const std::string robotIdValue = robotId.toStdString();
        const simulation_project::RobotMountDesc* preferredMount =
            findRobotMountDesc(document, preferredMountId.toStdString());
        if(preferredMount != nullptr && preferredMount->robotId == robotIdValue) {
            return preferredMountId;
        }

        if(!preferredLinkName.isEmpty()) {
            const std::string linkName = preferredLinkName.toStdString();
            for(const simulation_project::RobotMountDesc& mount : document.robotMounts) {
                if(mount.robotId == robotIdValue && mount.linkName == linkName) {
                    return QString::fromStdString(mount.id);
                }
            }
        }

        for(const simulation_project::MountedAttachmentDesc& attachment : document.mountedAttachments) {
            const simulation_project::RobotMountDesc* mount =
                findRobotMountDesc(document, attachment.mountFrameId);
            if(attachment.enabled &&
                attachment.visible &&
                mount != nullptr &&
                mount->robotId == robotIdValue) {
                return QString::fromStdString(mount->id);
            }
        }

        for(const simulation_project::MountedAttachmentDesc& attachment : document.mountedAttachments) {
            const simulation_project::AttachmentAssetDesc* asset =
                findAttachmentAssetDesc(document, attachment.assetId);
            const simulation_project::RobotMountDesc* mount =
                findRobotMountDesc(document, attachment.mountFrameId);
            if(asset != nullptr &&
                asset->assetKind != "sensor" &&
                attachment.enabled &&
                attachment.visible &&
                mount != nullptr &&
                mount->robotId == robotIdValue) {
                return QString::fromStdString(mount->id);
            }
        }

        for(const simulation_project::RobotMountDesc& mount : document.robotMounts) {
            if(mount.robotId == robotIdValue) {
                return QString::fromStdString(mount.id);
            }
        }

        return QString();
    }

    SceneRobotSelectionContext SceneSelectionController::robotSelectionContext(
        const simulation_project::ProjectDocument& document,
        const QString& robotId,
        const QString& preferredLinkName,
        const QString& preferredMountId)
    {
        SceneRobotSelectionContext context;
        context.robotId = robotId;
        context.linkName = preferredLinkName;
        context.mountId = preferredMountIdForRobot(document, robotId, preferredLinkName, preferredMountId);

        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(document, context.mountId.toStdString());
        if(!preferredMountId.isEmpty() && mount != nullptr) {
            context.linkName = QString::fromStdString(mount->linkName);
        }

        return context;
    }
}
