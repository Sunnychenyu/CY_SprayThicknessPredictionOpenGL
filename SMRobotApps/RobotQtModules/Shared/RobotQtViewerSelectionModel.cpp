#include "RobotQtViewerSelectionModel.h"

namespace robot_qt_viewer
{
    RobotQtViewerSelectionModel::RobotQtViewerSelectionModel(RobotQtViewerEventHub& eventHub)
        : m_eventHub(eventHub)
    {
    }

    const RobotQtViewerSelectionState& RobotQtViewerSelectionModel::state() const
    {
        return m_state;
    }

    RobotQtViewerSelectionPayload RobotQtViewerSelectionModel::payload() const
    {
        RobotQtViewerSelectionPayload value;
        value.robotId = m_state.robotId;
        value.linkName = m_state.linkName;
        value.objectId = m_state.objectId;
        value.objectFrameId = m_objectFrameId;
        value.mountId = m_state.mountId;
        value.attachmentId = m_state.attachmentId;
        value.assetId = m_toolAssetId;
        value.jointName = m_jointName;
        value.collisionDetectorId = m_collisionDetectorId;
        value.collisionPairRobotA = m_collisionPairRobotA;
        value.collisionPairLinkA = m_collisionPairLinkA;
        return value;
    }

    void RobotQtViewerSelectionModel::clear(const QString& sourceId)
    {
        m_state = {};
        m_toolAssetId.clear();
        m_objectFrameId.clear();
        m_jointName.clear();
        publish(sourceId);
    }

    void RobotQtViewerSelectionModel::selectRobotLink(
        const QString& robotId,
        const QString& linkName,
        const QString& sourceId)
    {
        m_state = {};
        m_state.kind = RobotQtViewerSelectionKind::RobotLink;
        m_state.robotId = robotId;
        m_state.linkName = linkName;
        m_toolAssetId.clear();
        m_objectFrameId.clear();
        m_jointName.clear();
        publish(sourceId);
    }

    void RobotQtViewerSelectionModel::selectRobotMount(
        const QString& robotId,
        const QString& linkName,
        const QString& mountId,
        const QString& sourceId)
    {
        m_state = {};
        m_state.kind = RobotQtViewerSelectionKind::RobotMount;
        m_state.robotId = robotId;
        m_state.linkName = linkName;
        m_state.mountId = mountId;
        m_toolAssetId.clear();
        m_objectFrameId.clear();
        m_jointName.clear();
        publish(sourceId);
    }

    void RobotQtViewerSelectionModel::selectMountedAttachment(
        const QString& attachmentId,
        const QString& robotId,
        const QString& linkName,
        const QString& mountId,
        const QString& sourceId)
    {
        m_state = {};
        m_state.kind = RobotQtViewerSelectionKind::MountedAttachment;
        m_state.attachmentId = attachmentId;
        m_state.robotId = robotId;
        m_state.linkName = linkName;
        m_state.mountId = mountId;
        m_toolAssetId.clear();
        m_objectFrameId.clear();
        m_jointName.clear();
        publish(sourceId);
    }

    void RobotQtViewerSelectionModel::selectSceneObject(
        const QString& objectId,
        const QString& sourceId)
    {
        m_state = {};
        m_state.kind = RobotQtViewerSelectionKind::SceneObject;
        m_state.objectId = objectId;
        m_toolAssetId.clear();
        m_objectFrameId.clear();
        m_jointName.clear();
        publish(sourceId);
    }

    void RobotQtViewerSelectionModel::selectObjectFrame(
        const QString& objectId,
        const QString& frameId,
        const QString& sourceId)
    {
        m_state = {};
        m_state.kind = RobotQtViewerSelectionKind::SceneObject;
        m_state.objectId = objectId;
        m_toolAssetId.clear();
        m_objectFrameId = frameId;
        m_jointName.clear();
        publish(sourceId);
    }

    void RobotQtViewerSelectionModel::selectRobotJoint(
        const QString& robotId,
        const QString& jointName,
        const QString& sourceId)
    {
        m_state = {};
        m_state.kind = RobotQtViewerSelectionKind::RobotLink;
        m_state.robotId = robotId;
        m_toolAssetId.clear();
        m_objectFrameId.clear();
        m_jointName = jointName;
        publish(sourceId);
    }

    void RobotQtViewerSelectionModel::selectToolAsset(
        const QString& assetId,
        const QString& sourceId)
    {
        m_state = {};
        m_toolAssetId = assetId;
        m_objectFrameId.clear();
        m_jointName.clear();
        publish(sourceId);
    }

    void RobotQtViewerSelectionModel::setCollisionDetector(
        const QString& detectorId,
        const QString& sourceId)
    {
        m_collisionDetectorId = detectorId;
        publish(sourceId);
    }

    void RobotQtViewerSelectionModel::setCollisionPairA(
        const QString& robotId,
        const QString& linkName,
        const QString& sourceId)
    {
        m_collisionPairRobotA = robotId;
        m_collisionPairLinkA = linkName;
        publish(sourceId);
    }

    void RobotQtViewerSelectionModel::publish(const QString& sourceId)
    {
        RobotQtViewerEvent event;
        event.kind = RobotQtViewerEventKind::SelectionChanged;
        event.sourceId = sourceId;
        event.selection = payload();
        m_eventHub.publish(event);
    }
}
