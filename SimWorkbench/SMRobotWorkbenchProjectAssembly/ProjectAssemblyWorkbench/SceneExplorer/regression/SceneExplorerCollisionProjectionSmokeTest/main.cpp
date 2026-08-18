#include "SceneCollisionTargetResolver.h"
#include "SceneExplorerViewModelBuilder.h"
#include "SceneTreeIntentController.h"

#include <QTreeWidgetItem>

#include <SimulationProject/ProjectDocument.h>

#include <cstdlib>
#include <iostream>
#include <string>

namespace
{
    struct Checks
    {
        int failures = 0;

        void require(bool condition, const std::string& message)
        {
            if(condition) {
                return;
            }
            ++failures;
            std::cerr << "FAIL: " << message << '\n';
        }
    };

    simulation_project::ProjectDocument makeDocument()
    {
        simulation_project::ProjectDocument document;

        simulation_project::RobotDesc robot;
        robot.id = "red4600";
        robot.name = "Red 4600";
        document.robots.push_back(robot);

        simulation_project::RobotMountDesc mount;
        mount.id = "tool_mount";
        mount.name = "Tool Mount";
        mount.robotId = robot.id;
        mount.linkName = "Link6";
        document.robotMounts.push_back(mount);

        simulation_project::AttachmentAssetDesc asset;
        asset.id = "tool_asset";
        asset.name = "420 Tool Asset";
        asset.visualPath = "420_tool.obj";
        document.attachmentAssets.push_back(asset);

        simulation_project::MountedAttachmentDesc attachment;
        attachment.id = "420_tool";
        attachment.name = "420 Tool";
        attachment.mountFrameId = mount.id;
        attachment.assetId = asset.id;
        document.mountedAttachments.push_back(attachment);

        simulation_project::SceneObjectDesc object;
        object.id = "workpiece";
        object.name = "Workpiece";
        simulation_project::ObjectFrameDesc frame;
        frame.id = "tcp_like_frame";
        frame.name = "Object Frame";
        object.objectFrames.push_back(frame);
        document.objects.push_back(object);

        simulation_project::PointCloudDesc pointCloud;
        pointCloud.id = "scan_cloud";
        pointCloud.name = "Scan Cloud";
        pointCloud.sourcePath = "scan.pcd";
        document.pointClouds.push_back(pointCloud);

        return document;
    }

    robot_qt_viewer::SceneExplorerNodeRef node(
        robot_qt_viewer::SceneExplorerNodeKind kind,
        const QString& id,
        const QString& name = QString(),
        const QString& linkName = QString())
    {
        robot_qt_viewer::SceneExplorerNodeRef ref;
        ref.kind = kind;
        ref.id = id;
        ref.name = name;
        ref.linkName = linkName;
        return ref;
    }

    QTreeWidgetItem makeTreeItem(const robot_qt_viewer::SceneExplorerNodeRef& ref)
    {
        QTreeWidgetItem item;
        item.setData(0, robot_qt_viewer::kSceneExplorerRoleId, ref.id);
        item.setData(0, robot_qt_viewer::kSceneExplorerRoleName, ref.name);
        item.setData(0, robot_qt_viewer::kSceneExplorerRoleLink, ref.linkName);
        item.setData(0, robot_qt_viewer::kSceneExplorerRoleType,
            robot_qt_viewer::sceneExplorerNodeTypeName(ref.kind));
        item.setText(0, ref.name.isEmpty() ? ref.id : ref.name);
        return item;
    }
}

int main()
{
    Checks checks;
    const simulation_project::ProjectDocument document = makeDocument();

    QVector<robot_qt_viewer::SceneExplorerRobotRuntimeView> robots;
    robot_qt_viewer::SceneExplorerRobotRuntimeView robot;
    robot.id = QStringLiteral("red4600");
    robot.name = QStringLiteral("Red 4600");
    robot.links = QStringList{ QStringLiteral("Link1"), QStringLiteral("Link6") };
    robot.joints = QStringList{ QStringLiteral("J1"), QStringLiteral("J6") };
    robots.push_back(robot);

    QVector<robot_qt_viewer::SceneExplorerObjectRuntimeView> objects;
    robot_qt_viewer::SceneExplorerObjectRuntimeView object;
    object.id = QStringLiteral("workpiece");
    object.name = QStringLiteral("Workpiece");
    objects.push_back(object);

    robot_qt_viewer::SceneExplorerViewOptions options;
    options.interactionMode = robot_qt_viewer::RobotQtViewerViewportInteractionMode::SelectCollisionTarget;
    options.treeProjection = robot_qt_viewer::SceneExplorerTreeProjection::CollisionConfig;

    const robot_qt_viewer::SceneExplorerViewModel view =
        robot_qt_viewer::buildSceneExplorerViewModel(
            document,
            robots,
            objects,
            robot_qt_viewer::SceneExplorerNodeRef(),
            options);

    bool hasMountGroup = false;
    bool hasRobotMountNode = false;
    bool hasObjectFrameNode = false;
    bool hasToolAttachmentUnderLink6 = false;
    for(const robot_qt_viewer::SceneExplorerNodeView& viewNode : view.nodes) {
        if(viewNode.text.startsWith(QStringLiteral("Mounts"))) {
            hasMountGroup = true;
        }
        if(viewNode.ref.kind == robot_qt_viewer::SceneExplorerNodeKind::RobotMount) {
            hasRobotMountNode = true;
        }
        if(viewNode.ref.kind == robot_qt_viewer::SceneExplorerNodeKind::ObjectFrame) {
            hasObjectFrameNode = true;
        }
        if(viewNode.ref.kind == robot_qt_viewer::SceneExplorerNodeKind::ToolAttachment &&
            viewNode.ref.id == QStringLiteral("420_tool") &&
            viewNode.parentNodeId.endsWith(QStringLiteral(":links:Link6"))) {
            hasToolAttachmentUnderLink6 = true;
        }
    }

    checks.require(!hasMountGroup, "collision projection hides the standalone Mounts group");
    checks.require(!hasRobotMountNode, "collision projection hides robot mount frame nodes");
    checks.require(!hasObjectFrameNode, "collision projection hides object frame nodes");
    checks.require(hasToolAttachmentUnderLink6, "collision projection preserves mounted attachment under Link6");

    simulation_project::CollisionSelectionSetMemberDesc member;
    checks.require(
        !robot_qt_viewer::sceneExplorerNodeToCollisionSelectionMember(
            node(robot_qt_viewer::SceneExplorerNodeKind::RobotMount, QStringLiteral("tool_mount")),
            member),
        "robot mount frame is not a collision selection member");
    checks.require(
        !robot_qt_viewer::sceneExplorerNodeToCollisionSelectionMember(
            node(
                robot_qt_viewer::SceneExplorerNodeKind::ObjectFrame,
                QStringLiteral("workpiece"),
                QStringLiteral("Object Frame"),
                QStringLiteral("tcp_like_frame")),
            member),
        "object frame is not a collision selection member");
    checks.require(
        robot_qt_viewer::sceneExplorerNodeToCollisionSelectionMember(
            node(
                robot_qt_viewer::SceneExplorerNodeKind::Link,
                QStringLiteral("red4600"),
                QStringLiteral("Red 4600"),
                QStringLiteral("Link6")),
            member) &&
            member.robotId == "red4600" &&
            member.linkName == "Link6",
        "robot link converts to a collision selection member");
    checks.require(
        robot_qt_viewer::sceneExplorerNodeToCollisionSelectionMember(
            node(robot_qt_viewer::SceneExplorerNodeKind::ToolAttachment, QStringLiteral("420_tool")),
            member) &&
            member.attachmentId == "420_tool",
        "mounted attachment converts to a collision selection member");
    checks.require(
        robot_qt_viewer::sceneExplorerNodeToCollisionSelectionMember(
            node(robot_qt_viewer::SceneExplorerNodeKind::PointCloud, QStringLiteral("scan_cloud")),
            member) &&
            member.objectId == "scan_cloud",
        "point cloud converts to a collision selection member through the object-compatible field");

    checks.require(
        robot_qt_viewer::sceneExplorerNodeKindCanConfigureCollisionModel(
            robot_qt_viewer::SceneExplorerNodeKind::Robot),
        "robot node can enter collision model configuration");
    checks.require(
        robot_qt_viewer::sceneExplorerNodeKindCanConfigureCollisionModel(
            robot_qt_viewer::SceneExplorerNodeKind::Link),
        "link node can enter collision model configuration");
    checks.require(
        robot_qt_viewer::sceneExplorerNodeKindCanConfigureCollisionModel(
            robot_qt_viewer::SceneExplorerNodeKind::Object),
        "object node can enter collision model configuration");
    checks.require(
        robot_qt_viewer::sceneExplorerNodeKindCanConfigureCollisionModel(
            robot_qt_viewer::SceneExplorerNodeKind::ToolAttachment),
        "mounted attachment can enter collision model configuration");
    checks.require(
        !robot_qt_viewer::sceneExplorerNodeKindCanConfigureCollisionModel(
            robot_qt_viewer::SceneExplorerNodeKind::RobotMount),
        "robot mount cannot enter collision model configuration");
    checks.require(
        !robot_qt_viewer::sceneExplorerNodeKindCanConfigureCollisionModel(
            robot_qt_viewer::SceneExplorerNodeKind::ObjectFrame),
        "object frame cannot enter collision model configuration");
    checks.require(
        !robot_qt_viewer::sceneExplorerNodeKindCanConfigureCollisionModel(
            robot_qt_viewer::SceneExplorerNodeKind::PointCloud),
        "point cloud cannot enter collision model configuration");

    QTreeWidgetItem mountItem =
        makeTreeItem(node(robot_qt_viewer::SceneExplorerNodeKind::RobotMount, QStringLiteral("tool_mount")));
    const robot_qt_viewer::SceneTreeIntentController::ContextMenuModel mountMenu =
        robot_qt_viewer::SceneTreeIntentController::contextMenuModelFromItem(
            &mountItem,
            true,
            robot_qt_viewer::SceneExplorerActionScope::CollisionConfig);
    checks.require(mountMenu.actions.empty(), "robot mount has no collision context menu actions");

    QTreeWidgetItem linkItem = makeTreeItem(node(
        robot_qt_viewer::SceneExplorerNodeKind::Link,
        QStringLiteral("red4600"),
        QStringLiteral("Red 4600"),
        QStringLiteral("Link6")));
    const robot_qt_viewer::SceneTreeIntentController::ContextMenuModel linkMenu =
        robot_qt_viewer::SceneTreeIntentController::contextMenuModelFromItem(
            &linkItem,
            true,
            robot_qt_viewer::SceneExplorerActionScope::CollisionConfig);
    bool linkHasConfigure = false;
    bool linkHasSetA = false;
    bool linkHasSetB = false;
    for(const robot_qt_viewer::SceneTreeIntentController::ContextMenuActionView& action : linkMenu.actions) {
        linkHasConfigure = linkHasConfigure ||
            action.action.kind ==
                robot_qt_viewer::SceneTreeIntentController::ContextMenuActionKind::ConfigureCollisionModel;
        linkHasSetA = linkHasSetA ||
            action.action.kind ==
                robot_qt_viewer::SceneTreeIntentController::ContextMenuActionKind::AddToDetectorSetA;
        linkHasSetB = linkHasSetB ||
            action.action.kind ==
                robot_qt_viewer::SceneTreeIntentController::ContextMenuActionKind::AddToDetectorSetB;
    }
    checks.require(linkHasConfigure, "robot link has Configure Collision Model action");
    checks.require(linkHasSetA, "robot link has Add to Detector Set A action");
    checks.require(linkHasSetB, "robot link has Add to Detector Set B action");

    if(checks.failures != 0) {
        std::cerr << checks.failures << " check(s) failed.\n";
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
