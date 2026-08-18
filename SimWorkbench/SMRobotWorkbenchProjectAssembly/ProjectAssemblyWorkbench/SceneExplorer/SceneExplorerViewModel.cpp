#include "SceneExplorerViewModel.h"

#include "SceneCollisionTargetResolver.h"

namespace robot_qt_viewer
{
    SceneExplorerNodeKind sceneExplorerNodeKindFromType(const QString& type)
    {
        if(type == kSceneExplorerNodeGroup) {
            return SceneExplorerNodeKind::Group;
        }
        if(type == kSceneExplorerNodeRobot) {
            return SceneExplorerNodeKind::Robot;
        }
        if(type == kSceneExplorerNodeObject) {
            return SceneExplorerNodeKind::Object;
        }
        if(type == kSceneExplorerNodePointCloud) {
            return SceneExplorerNodeKind::PointCloud;
        }
        if(type == kSceneExplorerNodeLink) {
            return SceneExplorerNodeKind::Link;
        }
        if(type == kSceneExplorerNodeJoint) {
            return SceneExplorerNodeKind::Joint;
        }
        if(type == kSceneExplorerNodeRobotMount) {
            return SceneExplorerNodeKind::RobotMount;
        }
        if(type == kSceneExplorerNodeObjectFrame) {
            return SceneExplorerNodeKind::ObjectFrame;
        }
        if(type == kSceneExplorerNodeToolAttachment) {
            return SceneExplorerNodeKind::ToolAttachment;
        }
        if(type == kSceneExplorerNodeToolAsset) {
            return SceneExplorerNodeKind::ToolAsset;
        }
        return SceneExplorerNodeKind::Unknown;
    }

    QString sceneExplorerNodeTypeName(SceneExplorerNodeKind kind)
    {
        switch(kind) {
        case SceneExplorerNodeKind::Robot:
            return kSceneExplorerNodeRobot;
        case SceneExplorerNodeKind::Object:
            return kSceneExplorerNodeObject;
        case SceneExplorerNodeKind::PointCloud:
            return kSceneExplorerNodePointCloud;
        case SceneExplorerNodeKind::Link:
            return kSceneExplorerNodeLink;
        case SceneExplorerNodeKind::Joint:
            return kSceneExplorerNodeJoint;
        case SceneExplorerNodeKind::RobotMount:
            return kSceneExplorerNodeRobotMount;
        case SceneExplorerNodeKind::ObjectFrame:
            return kSceneExplorerNodeObjectFrame;
        case SceneExplorerNodeKind::ToolAttachment:
            return kSceneExplorerNodeToolAttachment;
        case SceneExplorerNodeKind::ToolAsset:
            return kSceneExplorerNodeToolAsset;
        case SceneExplorerNodeKind::Group:
        case SceneExplorerNodeKind::Unknown:
            return kSceneExplorerNodeGroup;
        }
        return kSceneExplorerNodeGroup;
    }

    bool sceneExplorerNodeSelectableForMode(
        SceneExplorerNodeKind kind,
        RobotQtViewerViewportInteractionMode mode)
    {
        switch(mode) {
        case RobotQtViewerViewportInteractionMode::SelectRobot:
            return kind == SceneExplorerNodeKind::Group ||
                kind == SceneExplorerNodeKind::Robot;
        case RobotQtViewerViewportInteractionMode::SelectLink:
            return kind == SceneExplorerNodeKind::Group ||
                kind == SceneExplorerNodeKind::Robot ||
                kind == SceneExplorerNodeKind::Link ||
                kind == SceneExplorerNodeKind::Joint;
        case RobotQtViewerViewportInteractionMode::SelectMount:
            return kind == SceneExplorerNodeKind::Group ||
                kind == SceneExplorerNodeKind::Robot ||
                kind == SceneExplorerNodeKind::Link ||
                kind == SceneExplorerNodeKind::Joint ||
                kind == SceneExplorerNodeKind::RobotMount ||
                kind == SceneExplorerNodeKind::ToolAttachment ||
                kind == SceneExplorerNodeKind::Object ||
                kind == SceneExplorerNodeKind::ObjectFrame ||
                kind == SceneExplorerNodeKind::PointCloud ||
                kind == SceneExplorerNodeKind::ToolAsset;
        case RobotQtViewerViewportInteractionMode::SelectAttachment:
            return kind == SceneExplorerNodeKind::Group ||
                kind == SceneExplorerNodeKind::Robot ||
                kind == SceneExplorerNodeKind::RobotMount ||
                kind == SceneExplorerNodeKind::ToolAttachment;
        case RobotQtViewerViewportInteractionMode::SelectCollisionTarget:
            return kind == SceneExplorerNodeKind::Group ||
                sceneExplorerNodeKindCanBeCollisionTarget(kind);
        case RobotQtViewerViewportInteractionMode::Browse:
        case RobotQtViewerViewportInteractionMode::EditTransformPreview:
        case RobotQtViewerViewportInteractionMode::EditCollisionProxy:
            return true;
        }
        return true;
    }

    bool sceneExplorerNodeHighlightedForMode(
        SceneExplorerNodeKind kind,
        RobotQtViewerViewportInteractionMode mode)
    {
        switch(mode) {
        case RobotQtViewerViewportInteractionMode::SelectRobot:
            return kind == SceneExplorerNodeKind::Robot;
        case RobotQtViewerViewportInteractionMode::SelectLink:
            return kind == SceneExplorerNodeKind::Link;
        case RobotQtViewerViewportInteractionMode::SelectMount:
            return kind == SceneExplorerNodeKind::RobotMount ||
                kind == SceneExplorerNodeKind::ToolAttachment;
        case RobotQtViewerViewportInteractionMode::SelectAttachment:
            return kind == SceneExplorerNodeKind::ToolAttachment;
        case RobotQtViewerViewportInteractionMode::SelectCollisionTarget:
            return sceneExplorerNodeKindIsPrimaryCollisionTarget(kind);
        case RobotQtViewerViewportInteractionMode::Browse:
        case RobotQtViewerViewportInteractionMode::EditTransformPreview:
        case RobotQtViewerViewportInteractionMode::EditCollisionProxy:
            return false;
        }
        return false;
    }

    SceneExplorerTreeProjection sceneExplorerTreeProjectionForWorkbench(
        const RobotQtViewerWorkbenchDescriptor& descriptor)
    {
        return descriptor.collisionTreeProjection
            ? SceneExplorerTreeProjection::CollisionConfig
            : SceneExplorerTreeProjection::ProjectAssembly;
    }

    SceneExplorerActionScope sceneExplorerActionScopeForWorkbench(
        const RobotQtViewerWorkbenchDescriptor& descriptor)
    {
        return descriptor.collisionActions
            ? SceneExplorerActionScope::CollisionConfig
            : SceneExplorerActionScope::ProjectAssembly;
    }
}
