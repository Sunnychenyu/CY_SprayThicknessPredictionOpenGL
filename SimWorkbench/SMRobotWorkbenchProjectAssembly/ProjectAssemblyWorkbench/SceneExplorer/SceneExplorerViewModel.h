#pragma once

#include "RobotQtViewerWorkbench.h"

#include <QVector>
#include <QString>
#include <QStringList>

#include <Qt>

#include <SimulationProject/ProjectDocument.h>

namespace robot_qt_viewer
{
    inline constexpr int kSceneExplorerRoleId = Qt::UserRole;
    inline constexpr int kSceneExplorerRoleName = Qt::UserRole + 1;
    inline constexpr int kSceneExplorerRoleType = Qt::UserRole + 2;
    inline constexpr int kSceneExplorerRoleLink = Qt::UserRole + 3;
    inline constexpr int kSceneExplorerRoleTaskSelectable = Qt::UserRole + 4;
    inline constexpr int kSceneExplorerRoleTaskHighlighted = Qt::UserRole + 5;

    inline const QString kSceneExplorerNodeGroup = QStringLiteral("group");
    inline const QString kSceneExplorerNodeRobot = QStringLiteral("robot");
    inline const QString kSceneExplorerNodeObject = QStringLiteral("object");
    inline const QString kSceneExplorerNodePointCloud = QStringLiteral("pointCloud");
    inline const QString kSceneExplorerNodeLink = QStringLiteral("link");
    inline const QString kSceneExplorerNodeJoint = QStringLiteral("joint");
    inline const QString kSceneExplorerNodeRobotMount = QStringLiteral("robotMount");
    inline const QString kSceneExplorerNodeObjectFrame = QStringLiteral("objectFrame");
    inline const QString kSceneExplorerNodeToolAttachment = QStringLiteral("toolAttachment");
    inline const QString kSceneExplorerNodeToolAsset = QStringLiteral("toolAsset");

    enum class SceneExplorerNodeKind
    {
        Unknown,
        Group,
        Robot,
        Object,
        PointCloud,
        Link,
        Joint,
        RobotMount,
        ObjectFrame,
        ToolAttachment,
        ToolAsset
    };

    enum class SceneExplorerTreeProjection
    {
        ProjectAssembly,
        CollisionConfig
    };

    enum class SceneExplorerActionScope
    {
        ProjectAssembly,
        CollisionConfig
    };

    struct SceneExplorerViewOptions
    {
        RobotQtViewerViewportInteractionMode interactionMode =
            RobotQtViewerViewportInteractionMode::Browse;
        SceneExplorerTreeProjection treeProjection =
            SceneExplorerTreeProjection::ProjectAssembly;
    };

    struct SceneExplorerNodeRef
    {
        SceneExplorerNodeKind kind = SceneExplorerNodeKind::Unknown;
        QString id;
        QString name;
        QString linkName;
    };

    struct SceneExplorerNodeView
    {
        SceneExplorerNodeRef ref;
        QString nodeId;
        QString parentNodeId;
        QString text;
        QString toolTip;
        bool expanded = false;
        bool taskSelectable = true;
        bool taskHighlighted = false;
        QString taskHint;
    };

    struct SceneExplorerMountBindingDiagramView
    {
        bool visible = false;
        bool hasObjectBinding = false;
        QString bindingName;
        QString mountLinkName;
        QString mountFrameName;
        QString objectFrameName;
        QString objectName;
        QString mountTransformText;
        QString objectFrameInverseTransformText;
    };

    enum class SceneExplorerObjectFrameMode
    {
        Selection,
        Create,
        Edit
    };

    struct SceneExplorerObjectFrameDiagramView
    {
        bool visible = false;
        QString objectName;
        QString objectFrameName;
        QString objectToFrameTransformText;
    };

    struct SceneExplorerViewModel
    {
        QVector<SceneExplorerNodeView> nodes;
        QString selectedNodeId;
        QString taskSummary;
        SceneExplorerNodeRef selectedNode;
        QString selectionTitle;
        QStringList selectionDetails;
        SceneExplorerMountBindingDiagramView mountBindingDiagram;
        bool readOnlyTransformMatrixVisible = false;
        QString readOnlyTransformTitle;
        QString readOnlyTransformMatrixText;
        bool pointCloudScaleVisible = false;
        double pointCloudScale = 1.0;
        SceneExplorerObjectFrameMode objectFrameMode = SceneExplorerObjectFrameMode::Selection;
        SceneExplorerObjectFrameDiagramView objectFrameDiagram;
        bool objectFrameEditorVisible = false;
        QString objectFrameObjectName;
        QString objectFrameName;
        bool objectFrameVisible = false;
        bool objectFrameVisibilityControlVisible = false;
        SceneExplorerNodeRef objectFrameVisibilityTarget;
        bool transformEditorVisible = false;
        bool transformEditorEnabled = false;
        bool transformEditorDirty = false;
        QString transformEditorTitle;
        SceneExplorerNodeRef transformTarget;
        simulation_project::TransformDesc transform;
    };

    struct SceneExplorerRobotRuntimeView
    {
        QString id;
        QString name;
        QStringList links;
        QStringList joints;
        QStringList movableJoints;
        QStringList movableJointTypes;
    };

    struct SceneExplorerObjectRuntimeView
    {
        QString id;
        QString name;
    };

    SceneExplorerNodeKind sceneExplorerNodeKindFromType(const QString& type);
    QString sceneExplorerNodeTypeName(SceneExplorerNodeKind kind);
    bool sceneExplorerNodeSelectableForMode(
        SceneExplorerNodeKind kind,
        RobotQtViewerViewportInteractionMode mode);
    bool sceneExplorerNodeHighlightedForMode(
        SceneExplorerNodeKind kind,
        RobotQtViewerViewportInteractionMode mode);
    SceneExplorerTreeProjection sceneExplorerTreeProjectionForWorkbench(
        const RobotQtViewerWorkbenchDescriptor& descriptor);
    SceneExplorerActionScope sceneExplorerActionScopeForWorkbench(
        const RobotQtViewerWorkbenchDescriptor& descriptor);
}
