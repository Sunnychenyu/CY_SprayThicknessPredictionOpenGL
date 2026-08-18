#include "SceneExplorerViewModelBuilder.h"

#include <QHash>
#include <QStringList>

#include <array>
#include <cmath>

namespace
{
    QString conciseText(const std::string& text, int maxChars)
    {
        QString value = QString::fromStdString(text);
        if(maxChars <= 3 || value.size() <= maxChars) {
            return value;
        }
        return value.left(maxChars - 3) + "...";
    }

    QString displayNameWithId(const QString& name, const QString& id)
    {
        return name == id || id.isEmpty()
            ? name
            : QString("%1 (%2)").arg(name, id);
    }

    robot_qt_viewer::SceneExplorerNodeView makeNode(
        const QString& nodeId,
        const QString& parentNodeId,
        robot_qt_viewer::SceneExplorerNodeKind kind,
        const QString& text,
        const QString& id = QString(),
        const QString& name = QString(),
        const QString& linkName = QString(),
        const QString& toolTip = QString(),
        bool expanded = false)
    {
        robot_qt_viewer::SceneExplorerNodeView node;
        node.nodeId = nodeId;
        node.parentNodeId = parentNodeId;
        node.ref.kind = kind;
        node.ref.id = id;
        node.ref.name = name;
        node.ref.linkName = linkName;
        node.text = text;
        node.toolTip = toolTip;
        node.expanded = expanded;
        return node;
    }

    QString taskSummaryForMode(robot_qt_viewer::RobotQtViewerViewportInteractionMode mode)
    {
        switch(mode) {
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::SelectMount:
            return QStringLiteral("Frame Editor: select links or mount frames.");
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::SelectAttachment:
            return QStringLiteral("Frame Editor: select a mounted attachment.");
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::SelectCollisionTarget:
            return QStringLiteral("Collision: select collision targets.");
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::Browse:
            return QStringLiteral("Browse project scene.");
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::SelectRobot:
            return QStringLiteral("Select a robot.");
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::SelectLink:
            return QStringLiteral("Select a robot link.");
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::EditTransformPreview:
            return QStringLiteral("Edit transform preview.");
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::EditCollisionProxy:
            return QStringLiteral("Edit collision proxy.");
        }
        return QStringLiteral("Browse project scene.");
    }

    bool projectAssemblyProjection(robot_qt_viewer::SceneExplorerTreeProjection projection)
    {
        return projection == robot_qt_viewer::SceneExplorerTreeProjection::ProjectAssembly;
    }

    void applyTaskState(
        robot_qt_viewer::SceneExplorerNodeView& node,
        robot_qt_viewer::RobotQtViewerViewportInteractionMode mode)
    {
        node.taskSelectable = robot_qt_viewer::sceneExplorerNodeSelectableForMode(node.ref.kind, mode);
        node.taskHighlighted = robot_qt_viewer::sceneExplorerNodeHighlightedForMode(node.ref.kind, mode);
        if(!node.taskSelectable) {
            node.taskHint = QStringLiteral("Not selectable in the current task.");
        } else if(node.taskHighlighted) {
            node.taskHint = QStringLiteral("Primary selectable item for the current task.");
        }
    }

    const simulation_project::RobotMountDesc* findRobotMount(
        const simulation_project::ProjectDocument& document,
        const std::string& mountId)
    {
        for(const simulation_project::RobotMountDesc& mount : document.robotMounts) {
            if(mount.id == mountId) {
                return &mount;
            }
        }
        return nullptr;
    }

    const simulation_project::RobotDesc* findRobot(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId)
    {
        for(const simulation_project::RobotDesc& robot : document.robots) {
            if(robot.id == robotId) {
                return &robot;
            }
        }
        return nullptr;
    }

    const simulation_project::SceneObjectDesc* findSceneObject(
        const simulation_project::ProjectDocument& document,
        const std::string& objectId)
    {
        for(const simulation_project::SceneObjectDesc& object : document.objects) {
            if(object.id == objectId) {
                return &object;
            }
        }
        return nullptr;
    }

    const simulation_project::ObjectFrameDesc* findObjectFrame(
        const simulation_project::SceneObjectDesc& object,
        const std::string& frameId)
    {
        for(const simulation_project::ObjectFrameDesc& frame : object.objectFrames) {
            if(frame.id == frameId) {
                return &frame;
            }
        }
        return nullptr;
    }

    const simulation_project::AttachmentAssetDesc* findAttachmentAsset(
        const simulation_project::ProjectDocument& document,
        const std::string& assetId)
    {
        for(const simulation_project::AttachmentAssetDesc& asset : document.attachmentAssets) {
            if(asset.id == assetId) {
                return &asset;
            }
        }
        return nullptr;
    }

    const simulation_project::SceneObjectDesc* findAttachmentSourceObject(
        const simulation_project::ProjectDocument& document,
        const simulation_project::MountedAttachmentDesc& attachment)
    {
        if(!attachment.sourceObjectId.empty()) {
            return findSceneObject(document, attachment.sourceObjectId);
        }

        const simulation_project::AttachmentAssetDesc* asset =
            findAttachmentAsset(document, attachment.assetId);
        if(asset == nullptr || asset->visualPath.empty()) {
            return nullptr;
        }

        const simulation_project::SceneObjectDesc* firstMatch = nullptr;
        for(const simulation_project::SceneObjectDesc& object : document.objects) {
            if(object.sourcePath != asset->visualPath) {
                continue;
            }
            if(firstMatch == nullptr) {
                firstMatch = &object;
            }
            if(!object.visible) {
                return &object;
            }
        }
        return firstMatch;
    }

    bool hiddenObjectMountedAsAttachment(
        const simulation_project::ProjectDocument& document,
        const simulation_project::SceneObjectDesc& object)
    {
        if(object.visible || object.sourcePath.empty()) {
            return false;
        }
        for(const simulation_project::MountedAttachmentDesc& attachment : document.mountedAttachments) {
            const simulation_project::AttachmentAssetDesc* asset = nullptr;
            for(const simulation_project::AttachmentAssetDesc& candidate : document.attachmentAssets) {
                if(candidate.id == attachment.assetId) {
                    asset = &candidate;
                    break;
                }
            }
            if(asset != nullptr && asset->visualPath == object.sourcePath) {
                return true;
            }
        }
        return false;
    }

    const simulation_project::PointCloudDesc* findPointCloud(
        const simulation_project::ProjectDocument& document,
        const std::string& pointCloudId)
    {
        for(const simulation_project::PointCloudDesc& pointCloud : document.pointClouds) {
            if(pointCloud.id == pointCloudId) {
                return &pointCloud;
            }
        }
        return nullptr;
    }

    std::array<double, 16> transformMatrixValues(const simulation_project::TransformDesc& transform)
    {
        const double cr = std::cos(transform.roll);
        const double sr = std::sin(transform.roll);
        const double cp = std::cos(transform.pitch);
        const double sp = std::sin(transform.pitch);
        const double cy = std::cos(transform.yaw);
        const double sy = std::sin(transform.yaw);

        return {
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            transform.x,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            transform.y,
            -sp,
            cp * sr,
            cp * cr,
            transform.z,
            0.0,
            0.0,
            0.0,
            1.0
        };
    }

    QString matrixRowText(const std::array<double, 16>& values, int row)
    {
        QStringList columns;
        columns.reserve(4);
        for(int column = 0; column < 4; ++column) {
            columns.push_back(QString::number(values[static_cast<std::size_t>(row * 4 + column)], 'f', 6));
        }
        return QStringLiteral("  [ ") + columns.join(QStringLiteral(", ")) + QStringLiteral(" ]");
    }

    QString displayMatrixRowText(const std::array<double, 16>& values, int row)
    {
        QStringList columns;
        columns.reserve(4);
        for(int column = 0; column < 4; ++column) {
            columns.push_back(QString::number(values[static_cast<std::size_t>(row * 4 + column)], 'f', 4));
        }
        return QStringLiteral("    ") + columns.join(QStringLiteral("        "));
    }

    QString displayTransformMatrixText(const simulation_project::TransformDesc& transform)
    {
        const std::array<double, 16> values = transformMatrixValues(transform);
        QStringList rows;
        rows.reserve(4);
        for(int row = 0; row < 4; ++row) {
            rows.push_back(displayMatrixRowText(values, row));
        }
        return rows.join(QLatin1Char('\n'));
    }

    QString compactMatrixRowText(const std::array<double, 16>& values, int row)
    {
        QStringList columns;
        columns.reserve(4);
        for(int column = 0; column < 4; ++column) {
            columns.push_back(QString::number(values[static_cast<std::size_t>(row * 4 + column)], 'f', 3));
        }
        return QStringLiteral("[ ") + columns.join(QStringLiteral("  ")) + QStringLiteral(" ]");
    }

    QString compactMatrixText(const std::array<double, 16>& values)
    {
        QStringList rows;
        rows.reserve(4);
        for(int row = 0; row < 4; ++row) {
            rows.push_back(compactMatrixRowText(values, row));
        }
        return rows.join(QLatin1Char('\n'));
    }

    QString transformMatrixText(const simulation_project::TransformDesc& transform)
    {
        return compactMatrixText(transformMatrixValues(transform));
    }

    QString inverseTransformMatrixText(const simulation_project::TransformDesc& transform)
    {
        const std::array<double, 16> matrix = transformMatrixValues(transform);
        std::array<double, 16> inverse = {
            matrix[0],
            matrix[4],
            matrix[8],
            -(matrix[0] * matrix[3] + matrix[4] * matrix[7] + matrix[8] * matrix[11]),
            matrix[1],
            matrix[5],
            matrix[9],
            -(matrix[1] * matrix[3] + matrix[5] * matrix[7] + matrix[9] * matrix[11]),
            matrix[2],
            matrix[6],
            matrix[10],
            -(matrix[2] * matrix[3] + matrix[6] * matrix[7] + matrix[10] * matrix[11]),
            0.0,
            0.0,
            0.0,
            1.0
        };
        return compactMatrixText(inverse);
    }

    const simulation_project::MountedAttachmentDesc* findObjectBindingAttachment(
        const simulation_project::ProjectDocument& document,
        const std::string& mountId)
    {
        for(const simulation_project::MountedAttachmentDesc& attachment : document.mountedAttachments) {
            if(attachment.mountFrameId == mountId && !attachment.sourceObjectId.empty()) {
                return &attachment;
            }
        }
        return nullptr;
    }

    void applyMountBindingDiagram(
        robot_qt_viewer::SceneExplorerViewModel& view,
        const simulation_project::ProjectDocument& document,
        const simulation_project::RobotMountDesc& mount)
    {
        view.mountBindingDiagram.visible = true;
        view.mountBindingDiagram.hasObjectBinding = false;
        view.mountBindingDiagram.bindingName = QStringLiteral("No bound object");
        view.mountBindingDiagram.mountLinkName = QString::fromStdString(mount.linkName);
        view.mountBindingDiagram.mountFrameName = QString::fromStdString(mount.name.empty() ? mount.id : mount.name);
        view.mountBindingDiagram.mountTransformText = transformMatrixText(mount.linkToMount);
        view.mountBindingDiagram.objectFrameName = QStringLiteral("Object Frame");
        view.mountBindingDiagram.objectName = QStringLiteral("Object");
        view.mountBindingDiagram.objectFrameInverseTransformText =
            transformMatrixText(simulation_project::TransformDesc());

        const simulation_project::MountedAttachmentDesc* attachment =
            findObjectBindingAttachment(document, mount.id);
        if(attachment == nullptr) {
            return;
        }

        const simulation_project::SceneObjectDesc* object =
            findSceneObject(document, attachment->sourceObjectId);
        if(object == nullptr) {
            return;
        }

        view.mountBindingDiagram.hasObjectBinding = true;
        view.mountBindingDiagram.bindingName = QString::fromStdString(
            attachment->name.empty() ? attachment->id : attachment->name);
        view.mountBindingDiagram.objectName = QString::fromStdString(
            object->name.empty() ? object->id : object->name);

        if(attachment->sourceObjectFrameId.empty()) {
            view.mountBindingDiagram.objectFrameName = QStringLiteral("Object origin");
            view.mountBindingDiagram.objectFrameInverseTransformText =
                transformMatrixText(simulation_project::TransformDesc());
            return;
        }

        const simulation_project::ObjectFrameDesc* frame =
            findObjectFrame(*object, attachment->sourceObjectFrameId);
        if(frame == nullptr) {
            view.mountBindingDiagram.objectFrameName =
                QString::fromStdString(attachment->sourceObjectFrameId);
            view.mountBindingDiagram.objectFrameInverseTransformText =
                transformMatrixText(simulation_project::TransformDesc());
            return;
        }

        view.mountBindingDiagram.objectFrameName =
            QString::fromStdString(frame->name.empty() ? frame->id : frame->name);
        view.mountBindingDiagram.objectFrameInverseTransformText =
            inverseTransformMatrixText(frame->objectToFrame);
    }

    void applyTransformEditor(
        robot_qt_viewer::SceneExplorerViewModel& view,
        const simulation_project::ProjectDocument& document,
        const robot_qt_viewer::SceneExplorerNodeRef& selectedNode,
        robot_qt_viewer::RobotQtViewerViewportInteractionMode interactionMode)
    {
        if(selectedNode.kind == robot_qt_viewer::SceneExplorerNodeKind::Robot) {
            const simulation_project::RobotDesc* robot = findRobot(document, selectedNode.id.toStdString());
            if(robot == nullptr) {
                return;
            }
            view.transformEditorVisible = true;
            view.transformEditorEnabled = true;
            view.transformEditorTitle = QString("Robot base transform: %1").arg(selectedNode.id);
            view.transformTarget = selectedNode;
            view.transform = robot->baseTransform;
            return;
        }

        if(selectedNode.kind == robot_qt_viewer::SceneExplorerNodeKind::Object &&
            interactionMode == robot_qt_viewer::RobotQtViewerViewportInteractionMode::EditTransformPreview) {
            const simulation_project::SceneObjectDesc* object =
                findSceneObject(document, selectedNode.id.toStdString());
            if(object == nullptr) {
                return;
            }
            view.transformEditorVisible = true;
            view.transformEditorEnabled = true;
            view.transformEditorTitle = QString("Scene object transform: %1").arg(selectedNode.id);
            view.transformTarget = selectedNode;
            view.transform = object->transform;
            return;
        }

        if(selectedNode.kind == robot_qt_viewer::SceneExplorerNodeKind::ObjectFrame &&
            interactionMode == robot_qt_viewer::RobotQtViewerViewportInteractionMode::EditTransformPreview) {
            const simulation_project::SceneObjectDesc* object =
                findSceneObject(document, selectedNode.id.toStdString());
            if(object == nullptr) {
                return;
            }
            const simulation_project::ObjectFrameDesc* frame =
                findObjectFrame(*object, selectedNode.linkName.toStdString());
            if(frame == nullptr) {
                return;
            }
            view.transformEditorVisible = true;
            view.transformEditorEnabled = true;
            view.objectFrameEditorVisible = true;
            view.objectFrameMode = robot_qt_viewer::SceneExplorerObjectFrameMode::Edit;
            view.transformEditorTitle = QStringLiteral("Frame Transform");
            view.transformTarget = selectedNode;
            view.transformTarget.name = QString::fromStdString(frame->name.empty() ? frame->id : frame->name);
            view.objectFrameObjectName = QString::fromStdString(object->name.empty() ? object->id : object->name);
            view.objectFrameName = view.transformTarget.name;
            view.objectFrameVisible = frame->visible;
            view.objectFrameVisibilityControlVisible = true;
            view.objectFrameVisibilityTarget = selectedNode;
            view.objectFrameDiagram.visible = true;
            view.objectFrameDiagram.objectName = view.objectFrameObjectName;
            view.objectFrameDiagram.objectFrameName = view.objectFrameName;
            view.objectFrameDiagram.objectToFrameTransformText =
                transformMatrixText(frame->objectToFrame);
            view.transform = frame->objectToFrame;
            return;
        }

        if(selectedNode.kind == robot_qt_viewer::SceneExplorerNodeKind::PointCloud) {
            const simulation_project::PointCloudDesc* pointCloud =
                findPointCloud(document, selectedNode.id.toStdString());
            if(pointCloud == nullptr) {
                return;
            }
            view.transformEditorVisible = true;
            view.transformEditorEnabled = true;
            view.transformEditorTitle = QString("Point cloud transform: %1").arg(selectedNode.id);
            view.transformTarget = selectedNode;
            view.transform = pointCloud->transform;
        }
    }

    void applySelectionDetails(
        robot_qt_viewer::SceneExplorerViewModel& view,
        const simulation_project::ProjectDocument& document,
        const robot_qt_viewer::SceneExplorerNodeRef& selectedNode)
    {
        view.selectedNode = selectedNode;
        if(selectedNode.kind == robot_qt_viewer::SceneExplorerNodeKind::Unknown) {
            view.selectionTitle = QStringLiteral("Scene Selection");
            view.selectionDetails.push_back(QStringLiteral("No scene item selected."));
            return;
        }

        switch(selectedNode.kind) {
        case robot_qt_viewer::SceneExplorerNodeKind::Robot:
            view.selectionTitle = QString("Robot: %1").arg(selectedNode.name.isEmpty() ? selectedNode.id : selectedNode.name);
            view.selectionDetails.push_back(QString("id: %1").arg(selectedNode.id));
            for(const simulation_project::RobotDesc& robot : document.robots) {
                if(QString::fromStdString(robot.id) == selectedNode.id) {
                    view.selectionDetails.push_back(QString("source: %1").arg(QString::fromStdString(robot.sourcePath)));
                    view.selectionDetails.push_back(QString("visible: %1").arg(robot.visible ? QStringLiteral("true") : QStringLiteral("false")));
                    view.selectionDetails.push_back(QString("collision: %1").arg(robot.collisionEnabled ? QStringLiteral("enabled") : QStringLiteral("disabled")));
                    break;
                }
            }
            break;
        case robot_qt_viewer::SceneExplorerNodeKind::Link: {
            view.selectionTitle = QString("Link: %1").arg(selectedNode.linkName);
            view.selectionDetails.push_back(QString("robot: %1").arg(selectedNode.id));
            int mountCount = 0;
            for(const simulation_project::RobotMountDesc& mount : document.robotMounts) {
                if(QString::fromStdString(mount.robotId) == selectedNode.id &&
                    QString::fromStdString(mount.linkName) == selectedNode.linkName) {
                    ++mountCount;
                }
            }
            view.selectionDetails.push_back(QString("mounts: %1").arg(mountCount));
            break;
        }
        case robot_qt_viewer::SceneExplorerNodeKind::Joint:
            view.selectionTitle = QString("Joint: %1").arg(selectedNode.name);
            view.selectionDetails.push_back(QString("robot: %1").arg(selectedNode.id));
            break;
        case robot_qt_viewer::SceneExplorerNodeKind::RobotMount:
            view.selectionTitle = QString("Mount frame: %1").arg(selectedNode.name.isEmpty() ? selectedNode.id : selectedNode.name);
            for(const simulation_project::RobotMountDesc& mount : document.robotMounts) {
                if(QString::fromStdString(mount.id) == selectedNode.id) {
                    applyMountBindingDiagram(view, document, mount);
                    break;
                }
            }
            break;
        case robot_qt_viewer::SceneExplorerNodeKind::ObjectFrame:
            view.selectionTitle = QString("Object frame: %1").arg(
                selectedNode.name.isEmpty() ? selectedNode.linkName : selectedNode.name);
            for(const simulation_project::SceneObjectDesc& object : document.objects) {
                if(QString::fromStdString(object.id) == selectedNode.id) {
                    const simulation_project::ObjectFrameDesc* frame =
                        findObjectFrame(object, selectedNode.linkName.toStdString());
                    view.objectFrameObjectName = QString::fromStdString(object.name.empty() ? object.id : object.name);
                    view.objectFrameName = frame != nullptr
                        ? QString::fromStdString(frame->name.empty() ? frame->id : frame->name)
                        : selectedNode.name;
                    view.selectionDetails.push_back(QString("id: %1").arg(selectedNode.linkName));
                    view.selectionDetails.push_back(QString("object: %1").arg(view.objectFrameObjectName));
                    view.selectionDetails.push_back(QString("frame: %1").arg(view.objectFrameName));
                    if(frame != nullptr) {
                        view.objectFrameDiagram.visible = true;
                        view.objectFrameDiagram.objectName = view.objectFrameObjectName;
                        view.objectFrameDiagram.objectFrameName = view.objectFrameName;
                        view.objectFrameDiagram.objectToFrameTransformText =
                            transformMatrixText(frame->objectToFrame);
                        view.objectFrameVisible = frame->visible;
                        view.readOnlyTransformMatrixVisible = true;
                        view.readOnlyTransformTitle = QString("Object -> Object Frame: %1")
                            .arg(view.objectFrameName);
                        view.readOnlyTransformMatrixText = displayTransformMatrixText(frame->objectToFrame);
                        const std::array<double, 16> matrix = transformMatrixValues(frame->objectToFrame);
                        view.selectionDetails.push_back(QStringLiteral("objectToFrame 4x4 matrix:"));
                        for(int row = 0; row < 4; ++row) {
                            view.selectionDetails.push_back(matrixRowText(matrix, row));
                        }
                    }
                    break;
                }
            }
            break;
        case robot_qt_viewer::SceneExplorerNodeKind::ToolAttachment:
            view.selectionTitle = QString("Attachment: %1").arg(selectedNode.name.isEmpty() ? selectedNode.id : selectedNode.name);
            view.selectionDetails.push_back(QString("id: %1").arg(selectedNode.id));
            view.selectionDetails.push_back(QString("link: %1").arg(selectedNode.linkName));
            break;
        case robot_qt_viewer::SceneExplorerNodeKind::Object:
            view.selectionTitle = QString("Object: %1").arg(selectedNode.name.isEmpty() ? selectedNode.id : selectedNode.name);
            view.selectionDetails.push_back(QString("id: %1").arg(selectedNode.id));
            for(const simulation_project::SceneObjectDesc& object : document.objects) {
                if(QString::fromStdString(object.id) == selectedNode.id) {
                    view.selectionDetails.push_back(QString("source: %1").arg(QString::fromStdString(object.sourcePath)));
                    view.selectionDetails.push_back(QString("visual scale: %1").arg(object.visualScale));
                    view.selectionDetails.push_back(QString("collision scale: %1").arg(object.collisionScale));
                    view.readOnlyTransformMatrixVisible = true;
                    view.readOnlyTransformTitle = QString("Scene object transform: %1").arg(selectedNode.id);
                    view.readOnlyTransformMatrixText = displayTransformMatrixText(object.transform);
                    break;
                }
            }
            break;
        case robot_qt_viewer::SceneExplorerNodeKind::PointCloud:
            view.selectionTitle = QString("Point cloud: %1").arg(selectedNode.name.isEmpty() ? selectedNode.id : selectedNode.name);
            view.selectionDetails.push_back(QString("id: %1").arg(selectedNode.id));
            for(const simulation_project::PointCloudDesc& pointCloud : document.pointClouds) {
                if(QString::fromStdString(pointCloud.id) == selectedNode.id) {
                    view.pointCloudScaleVisible = true;
                    view.pointCloudScale = pointCloud.scale;
                    view.selectionDetails.push_back(QString("source: %1").arg(QString::fromStdString(pointCloud.sourcePath)));
                    view.selectionDetails.push_back(QString("format: %1").arg(QString::fromStdString(pointCloud.format)));
                    view.selectionDetails.push_back(QString("scale: %1").arg(pointCloud.scale));
                    view.selectionDetails.push_back(QString("point size: %1").arg(pointCloud.visualization.pointSize));
                    break;
                }
            }
            break;
        case robot_qt_viewer::SceneExplorerNodeKind::ToolAsset:
            view.selectionTitle = QString("Attachment asset: %1").arg(selectedNode.name.isEmpty() ? selectedNode.id : selectedNode.name);
            view.selectionDetails.push_back(QString("id: %1").arg(selectedNode.id));
            break;
        case robot_qt_viewer::SceneExplorerNodeKind::Group:
            view.selectionTitle = selectedNode.name.isEmpty() ? QStringLiteral("Scene group") : selectedNode.name;
            break;
        case robot_qt_viewer::SceneExplorerNodeKind::Unknown:
            break;
        }
    }

}

namespace robot_qt_viewer
{
    SceneExplorerViewModel buildSceneExplorerViewModel(
        const simulation_project::ProjectDocument& document,
        const QVector<SceneExplorerRobotRuntimeView>& robots,
        const QVector<SceneExplorerObjectRuntimeView>& objects,
        const SceneExplorerNodeRef& selectedNode,
        RobotQtViewerViewportInteractionMode interactionMode)
    {
        SceneExplorerViewOptions options;
        options.interactionMode = interactionMode;
        options.treeProjection = SceneExplorerTreeProjection::ProjectAssembly;
        return buildSceneExplorerViewModel(document, robots, objects, selectedNode, options);
    }

    SceneExplorerViewModel buildSceneExplorerViewModel(
        const simulation_project::ProjectDocument& document,
        const QVector<SceneExplorerRobotRuntimeView>& robots,
        const QVector<SceneExplorerObjectRuntimeView>& objects,
        const SceneExplorerNodeRef& selectedNode,
        const SceneExplorerViewOptions& options)
    {
        SceneExplorerViewModel view;
        const RobotQtViewerViewportInteractionMode interactionMode = options.interactionMode;
        const bool showAssemblyRelations = projectAssemblyProjection(options.treeProjection);
        const bool showCollisionProjection =
            options.treeProjection == SceneExplorerTreeProjection::CollisionConfig;
        view.taskSummary = taskSummaryForMode(interactionMode);

        const QString robotsGroupId = QStringLiteral("group:robots");
        view.nodes.push_back(makeNode(
            robotsGroupId,
            QString(),
            SceneExplorerNodeKind::Group,
            QStringLiteral("Robots"),
            QString(),
            QStringLiteral("Robots"),
            QString(),
            QString(),
            true));

        for(const SceneExplorerRobotRuntimeView& robot : robots) {
            const QString robotNodeId = QStringLiteral("robot:") + robot.id;
            view.nodes.push_back(makeNode(
                robotNodeId,
                robotsGroupId,
                SceneExplorerNodeKind::Robot,
                displayNameWithId(robot.name, robot.id),
                robot.id,
                robot.name,
                QString(),
                QString(),
                true));

            QVector<const simulation_project::RobotMountDesc*> robotMounts;
            QHash<QString, QVector<const simulation_project::RobotMountDesc*>> mountsByLink;
            QHash<QString, QVector<const simulation_project::MountedAttachmentDesc*>> attachmentsByMount;
            for(const simulation_project::RobotMountDesc& mount : document.robotMounts) {
                if(QString::fromStdString(mount.robotId) == robot.id) {
                    robotMounts.push_back(&mount);
                    mountsByLink[QString::fromStdString(mount.linkName)].push_back(&mount);
                }
            }
            for(const simulation_project::MountedAttachmentDesc& attachment : document.mountedAttachments) {
                const simulation_project::RobotMountDesc* mount = findRobotMount(document, attachment.mountFrameId);
                if(mount != nullptr && QString::fromStdString(mount->robotId) == robot.id) {
                    attachmentsByMount[QString::fromStdString(mount->id)].push_back(&attachment);
                }
            }

            const auto addAttachmentNodes =
                [&](const QString& mountNodeId, const simulation_project::RobotMountDesc& mount) {
            const QString mountId = QString::fromStdString(mount.id);
            const QVector<const simulation_project::MountedAttachmentDesc*> attachments =
                attachmentsByMount.value(mountId);
            for(const simulation_project::MountedAttachmentDesc* attachment : attachments) {
                const QString attachmentId = QString::fromStdString(attachment->id);
                        QString attachmentName = QString::fromStdString(
                            attachment->name.empty() ? attachment->id : attachment->name);
                        if(!attachment->enabled) {
                            attachmentName += " [disabled]";
                        }
                if(!attachment->visible) {
                    attachmentName += " [hidden]";
                }
                const simulation_project::SceneObjectDesc* sourceObject =
                    findAttachmentSourceObject(document, *attachment);
                const QString attachmentNodeId =
                    mountNodeId + QStringLiteral(":attachment:") + attachmentId;
                if(showAssemblyRelations && !attachment->sourceObjectId.empty() && sourceObject != nullptr) {
                    const QString sourceObjectId = QString::fromStdString(sourceObject->id);
                    const QString sourceObjectName = QString::fromStdString(
                        sourceObject->name.empty() ? sourceObject->id : sourceObject->name);
                    const QString sourceFrameId = QString::fromStdString(attachment->sourceObjectFrameId);
                    QString sourceFrameName = QStringLiteral("Object origin");
                    if(!sourceFrameId.isEmpty()) {
                        const simulation_project::ObjectFrameDesc* frame =
                            findObjectFrame(*sourceObject, attachment->sourceObjectFrameId);
                        sourceFrameName = frame != nullptr
                            ? QString::fromStdString(frame->name.empty() ? frame->id : frame->name)
                            : sourceFrameId;
                    }
                    const QString sourceFrameNodeId =
                        mountNodeId + QStringLiteral(":sourceFrame:") + attachmentId;
                    view.nodes.push_back(makeNode(
                        sourceFrameNodeId,
                        mountNodeId,
                        SceneExplorerNodeKind::Group,
                        sourceFrameName,
                        QString(),
                        sourceFrameName,
                        QString::fromStdString(mount.linkName),
                        QString("bound object: %1\nframe: %2")
                            .arg(sourceObjectId, sourceFrameId.isEmpty()
                                ? QStringLiteral("Object origin")
                                : sourceFrameId),
                        true));
                    view.nodes.push_back(makeNode(
                        attachmentNodeId,
                        sourceFrameNodeId,
                        SceneExplorerNodeKind::ToolAttachment,
                        displayNameWithId(sourceObjectName, attachmentId),
                        attachmentId,
                        sourceObjectName,
                        QString::fromStdString(mount.linkName),
                        QString("id: %1\nasset id: %2\nmount id: %3\nsource object: %4")
                            .arg(attachmentId)
                            .arg(QString::fromStdString(attachment->assetId))
                            .arg(QString::fromStdString(attachment->mountFrameId))
                            .arg(sourceObjectId),
                        false));
                    continue;
                }
                view.nodes.push_back(makeNode(
                    attachmentNodeId,
                    mountNodeId,
                    SceneExplorerNodeKind::ToolAttachment,
                    QString("%1 (%2)")
                        .arg(attachmentName, conciseText(attachment->id, 32)),
                    attachmentId,
                    attachmentName,
                    QString::fromStdString(mount.linkName),
                    QString("id: %1\nasset id: %2\nmount id: %3")
                        .arg(attachmentId)
                        .arg(QString::fromStdString(attachment->assetId))
                        .arg(QString::fromStdString(attachment->mountFrameId)),
                    showAssemblyRelations && sourceObject != nullptr && !sourceObject->objectFrames.empty()));
                if(!showAssemblyRelations || sourceObject == nullptr) {
                    continue;
                }
                const QString sourceObjectId = QString::fromStdString(sourceObject->id);
                for(const simulation_project::ObjectFrameDesc& frame : sourceObject->objectFrames) {
                    const QString frameId = QString::fromStdString(frame.id);
                    const QString frameName = QString::fromStdString(frame.name.empty() ? frame.id : frame.name);
                    view.nodes.push_back(makeNode(
                        attachmentNodeId + QStringLiteral(":frame:") + frameId,
                        attachmentNodeId,
                        SceneExplorerNodeKind::ObjectFrame,
                        frameName,
                        sourceObjectId,
                        frameName,
                        frameId,
                        QString("bound object: %1\nframe: %2").arg(sourceObjectId, frameId)));
                }
            }
        };

        const auto addMountNode =
            [&](const QString& nodeId,
                const QString& parentNodeId,
                const simulation_project::RobotMountDesc& mount,
                const QString& text,
                bool includeAttachments) {
            const QString mountId = QString::fromStdString(mount.id);
            const QString mountName = QString::fromStdString(mount.name.empty() ? mount.id : mount.name);
            view.nodes.push_back(makeNode(
                        nodeId,
                        parentNodeId,
                        SceneExplorerNodeKind::RobotMount,
                        text,
                        mountId,
                        mountName,
                        QString::fromStdString(mount.linkName),
                QString("id: %1\nlink: %2")
                    .arg(mountId)
                    .arg(QString::fromStdString(mount.linkName)),
                includeAttachments && !attachmentsByMount.value(mountId).empty()));
            if(includeAttachments) {
                addAttachmentNodes(nodeId, mount);
            }
        };

            const QString linksGroupId = robotNodeId + QStringLiteral(":links");
            view.nodes.push_back(makeNode(
                linksGroupId,
                robotNodeId,
                SceneExplorerNodeKind::Group,
                QString("Links (%1)").arg(robot.links.size()),
                robot.id,
                robot.name,
                QString(),
                QString(),
                true));
            for(const QString& link : robot.links) {
                const QString linkNodeId = linksGroupId + QStringLiteral(":") + link;
                const QVector<const simulation_project::RobotMountDesc*> linkMounts = mountsByLink.value(link);
                int linkAttachmentCount = 0;
                for(const simulation_project::RobotMountDesc* mount : linkMounts) {
                    if(mount == nullptr) {
                        continue;
                    }
                    linkAttachmentCount += attachmentsByMount.value(QString::fromStdString(mount->id)).size();
                }
                QString linkText = link;
                if(showAssemblyRelations && !linkMounts.empty()) {
                    linkText = QString("%1 (%2 mounts)").arg(link).arg(linkMounts.size());
                } else if(showCollisionProjection && linkAttachmentCount > 0) {
                    linkText = QString("%1 (%2 attachments)").arg(link).arg(linkAttachmentCount);
                }
                view.nodes.push_back(makeNode(
                    linkNodeId,
                    linksGroupId,
                    SceneExplorerNodeKind::Link,
                    linkText,
                    robot.id,
                    robot.name,
                    link,
                    QString(),
                    showAssemblyRelations ? !linkMounts.empty() : linkAttachmentCount > 0));
                for(const simulation_project::RobotMountDesc* mount : linkMounts) {
                    if(mount == nullptr) {
                        continue;
                    }
                    if(showAssemblyRelations) {
                        const QString mountId = QString::fromStdString(mount->id);
                        const QString mountName = QString::fromStdString(mount->name.empty() ? mount->id : mount->name);
                        addMountNode(
                            linkNodeId + QStringLiteral(":mount:") + mountId,
                            linkNodeId,
                            *mount,
                            mountName,
                            true);
                    } else {
                        addAttachmentNodes(linkNodeId, *mount);
                    }
                }
            }

            const QString jointsGroupId = robotNodeId + QStringLiteral(":joints");
            view.nodes.push_back(makeNode(
                jointsGroupId,
                robotNodeId,
                SceneExplorerNodeKind::Group,
                QString("Joints (%1)").arg(robot.joints.size()),
                robot.id,
                robot.name,
                QString()));
            for(const QString& joint : robot.joints) {
                view.nodes.push_back(makeNode(
                    jointsGroupId + QStringLiteral(":") + joint,
                    jointsGroupId,
                    SceneExplorerNodeKind::Joint,
                    joint,
                    robot.id,
                    joint,
                    QString(),
                    QString("robot: %1\njoint: %2").arg(robot.id, joint)));
            }

            if(showAssemblyRelations) {
                const QString mountsGroupId = robotNodeId + QStringLiteral(":mounts");
                view.nodes.push_back(makeNode(
                    mountsGroupId,
                    robotNodeId,
                    SceneExplorerNodeKind::Group,
                    QString("Mounts (%1)").arg(robotMounts.size()),
                    robot.id,
                    robot.name,
                    QString(),
                    QString(),
                    !robotMounts.empty()));
                for(const simulation_project::RobotMountDesc* mount : robotMounts) {
                    const QString mountId = QString::fromStdString(mount->id);
                    const QString mountName = QString::fromStdString(mount->name.empty() ? mount->id : mount->name);
                    const QString linkName = QString::fromStdString(mount->linkName);
                    addMountNode(
                        mountsGroupId + QStringLiteral(":") + mountId,
                        mountsGroupId,
                        *mount,
                        QString("%1 [%2]").arg(mountName, linkName),
                        false);
                }
            }
        }

        const QString objectsGroupId = QStringLiteral("group:objects");
        view.nodes.push_back(makeNode(
            objectsGroupId,
            QString(),
            SceneExplorerNodeKind::Group,
            QStringLiteral("Objects"),
            QString(),
            QStringLiteral("Objects"),
            QString(),
            QString(),
            !objects.empty()));
        for(const SceneExplorerObjectRuntimeView& object : objects) {
            const simulation_project::SceneObjectDesc* objectDesc =
                findSceneObject(document, object.id.toStdString());
            const bool boundHiddenObject =
                objectDesc != nullptr && hiddenObjectMountedAsAttachment(document, *objectDesc);
            if(boundHiddenObject) {
                continue;
            }
            const bool hasFrames =
                showAssemblyRelations && objectDesc != nullptr && !objectDesc->objectFrames.empty();
            const QString objectNodeId = objectsGroupId + QStringLiteral(":") + object.id;
            QString objectText = displayNameWithId(object.name, object.id);
            if(objectDesc != nullptr && !objectDesc->visible) {
                objectText += QStringLiteral(" [hidden]");
            }
            view.nodes.push_back(makeNode(
                objectNodeId,
                objectsGroupId,
                SceneExplorerNodeKind::Object,
                hasFrames
                    ? QString("%1 (%2 frames)").arg(objectText).arg(objectDesc->objectFrames.size())
                    : objectText,
                object.id,
                object.name,
                QString(),
                boundHiddenObject
                    ? QStringLiteral("This source object is hidden because it is bound as a mounted attachment.")
                    : QString(),
                hasFrames));
            if(objectDesc == nullptr) {
                continue;
            }
            if(!showAssemblyRelations) {
                continue;
            }
            for(const simulation_project::ObjectFrameDesc& frame : objectDesc->objectFrames) {
                const QString frameId = QString::fromStdString(frame.id);
                const QString frameName = QString::fromStdString(frame.name.empty() ? frame.id : frame.name);
                view.nodes.push_back(makeNode(
                    objectNodeId + QStringLiteral(":frame:") + frameId,
                    objectNodeId,
                    SceneExplorerNodeKind::ObjectFrame,
                    frameName,
                    object.id,
                    frameName,
                    frameId,
                    QString("object: %1\nframe: %2").arg(object.id, frameId)));
            }
        }

        const QString pointCloudsGroupId = QStringLiteral("group:pointClouds");
        view.nodes.push_back(makeNode(
            pointCloudsGroupId,
            QString(),
            SceneExplorerNodeKind::Group,
            QStringLiteral("Point Clouds"),
            QString(),
            QStringLiteral("Point Clouds"),
            QString(),
            QString(),
            !document.pointClouds.empty()));
        for(const simulation_project::PointCloudDesc& pointCloud : document.pointClouds) {
            const QString pointCloudId = QString::fromStdString(pointCloud.id);
            const QString pointCloudName = QString::fromStdString(pointCloud.name.empty() ? pointCloud.id : pointCloud.name);
            QString text = pointCloudName == pointCloudId
                ? pointCloudName
                : QString("%1 (%2)").arg(pointCloudName, conciseText(pointCloud.id, 32));
            if(!pointCloud.visualization.visible) {
                text += QStringLiteral(" [hidden]");
            }
            if(pointCloud.collision.enabled) {
                text += QStringLiteral(" [collision]");
            }
            view.nodes.push_back(makeNode(
                pointCloudsGroupId + QStringLiteral(":") + pointCloudId,
                pointCloudsGroupId,
                SceneExplorerNodeKind::PointCloud,
                text,
                pointCloudId,
                pointCloudName,
                QString(),
                QString("id: %1\nsource: %2\nformat: %3")
                    .arg(pointCloudId)
                    .arg(QString::fromStdString(pointCloud.sourcePath))
                    .arg(QString::fromStdString(pointCloud.format))));
        }

        if(showAssemblyRelations) {
            const QString assetsGroupId = QStringLiteral("group:toolAssets");
            int toolAssetCount = 0;
            for(const simulation_project::AttachmentAssetDesc& asset : document.attachmentAssets) {
                if(asset.assetKind != "sensor") {
                    ++toolAssetCount;
                }
            }
            view.nodes.push_back(makeNode(
                assetsGroupId,
                QString(),
                SceneExplorerNodeKind::Group,
                QStringLiteral("Tool Assets"),
                QString(),
                QStringLiteral("Tool Assets"),
                QString(),
                QString(),
                toolAssetCount > 0));
            for(const simulation_project::AttachmentAssetDesc& asset : document.attachmentAssets) {
                if(asset.assetKind == "sensor") {
                    continue;
                }
                const QString assetId = QString::fromStdString(asset.id);
                const QString assetName = QString::fromStdString(asset.name.empty() ? asset.id : asset.name);
                view.nodes.push_back(makeNode(
                    assetsGroupId + QStringLiteral(":") + assetId,
                    assetsGroupId,
                    SceneExplorerNodeKind::ToolAsset,
                    assetName == assetId ? assetName : QString("%1 (%2)").arg(assetName, conciseText(asset.id, 32)),
                    assetId,
                    assetName,
                    QString(),
                    QString("id: %1\nvisual: %2").arg(assetId, QString::fromStdString(asset.visualPath))));
            }
        }

        for(SceneExplorerNodeView& node : view.nodes) {
            applyTaskState(node, interactionMode);
            const bool sameJointName = selectedNode.kind != SceneExplorerNodeKind::Joint ||
                node.ref.name == selectedNode.name;
            if(node.ref.kind == selectedNode.kind &&
                node.ref.id == selectedNode.id &&
                node.ref.linkName == selectedNode.linkName &&
                sameJointName) {
                view.selectedNodeId = node.nodeId;
            }
        }
        applySelectionDetails(view, document, selectedNode);
        applyTransformEditor(view, document, selectedNode, interactionMode);
        return view;
    }
}

