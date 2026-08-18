#include "InspectorContext.h"

void InspectorContext::clearSelection()
{
    clearEntitySelection();
}

void InspectorContext::setRobotLink(
    const QString& robotId,
    const QString& linkName)
{
    clearEntitySelection();
    m_selectedRobotId = robotId;
    m_selectedLinkName = linkName;
}

void InspectorContext::setRobotMount(
    const QString& robotId,
    const QString& linkName,
    const QString& mountId)
{
    setRobotLink(robotId, linkName);
    m_selectedRobotMountId = mountId;
}

void InspectorContext::setObject(const QString& objectId)
{
    clearEntitySelection();
    m_selectedObjectId = objectId;
}

void InspectorContext::setToolAsset(const QString& assetId)
{
    clearEntitySelection();
    m_selectedToolAssetId = assetId;
}

void InspectorContext::setToolAttachment(
    const QString& robotId,
    const QString& linkName,
    const QString& mountId,
    const QString& attachmentId)
{
    setRobotMount(robotId, linkName, mountId);
    m_selectedToolAttachmentId = attachmentId;
}

void InspectorContext::setActiveCollisionDetectorId(const QString& detectorId)
{
    m_activeCollisionDetectorId = detectorId;
}

void InspectorContext::setMarkedCollisionPairA(
    const QString& robotId,
    const QString& linkName)
{
    m_markedCollisionPairRobotA = robotId;
    m_markedCollisionPairLinkA = linkName;
}

const QString& InspectorContext::selectedRobotId() const
{
    return m_selectedRobotId;
}

const QString& InspectorContext::selectedLinkName() const
{
    return m_selectedLinkName;
}

const QString& InspectorContext::selectedObjectId() const
{
    return m_selectedObjectId;
}

const QString& InspectorContext::selectedRobotMountId() const
{
    return m_selectedRobotMountId;
}

const QString& InspectorContext::selectedToolAssetId() const
{
    return m_selectedToolAssetId;
}

const QString& InspectorContext::selectedToolAttachmentId() const
{
    return m_selectedToolAttachmentId;
}

const QString& InspectorContext::activeCollisionDetectorId() const
{
    return m_activeCollisionDetectorId;
}

const QString& InspectorContext::markedCollisionPairRobotA() const
{
    return m_markedCollisionPairRobotA;
}

const QString& InspectorContext::markedCollisionPairLinkA() const
{
    return m_markedCollisionPairLinkA;
}

void InspectorContext::clearEntitySelection()
{
    m_selectedRobotId.clear();
    m_selectedLinkName.clear();
    m_selectedObjectId.clear();
    m_selectedRobotMountId.clear();
    m_selectedToolAssetId.clear();
    m_selectedToolAttachmentId.clear();
}
