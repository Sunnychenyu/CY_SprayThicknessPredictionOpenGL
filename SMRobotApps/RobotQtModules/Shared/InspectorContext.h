#pragma once

#include <QString>

class InspectorContext
{
public:
    void clearSelection();

    void setRobotLink(
        const QString& robotId,
        const QString& linkName);
    void setRobotMount(
        const QString& robotId,
        const QString& linkName,
        const QString& mountId);
    void setObject(const QString& objectId);
    void setToolAsset(const QString& assetId);
    void setToolAttachment(
        const QString& robotId,
        const QString& linkName,
        const QString& mountId,
        const QString& attachmentId);

    void setActiveCollisionDetectorId(const QString& detectorId);
    void setMarkedCollisionPairA(
        const QString& robotId,
        const QString& linkName);

    const QString& selectedRobotId() const;
    const QString& selectedLinkName() const;
    const QString& selectedObjectId() const;
    const QString& selectedRobotMountId() const;
    const QString& selectedToolAssetId() const;
    const QString& selectedToolAttachmentId() const;
    const QString& activeCollisionDetectorId() const;
    const QString& markedCollisionPairRobotA() const;
    const QString& markedCollisionPairLinkA() const;

private:
    void clearEntitySelection();

    QString m_selectedRobotId;
    QString m_selectedLinkName;
    QString m_selectedObjectId;
    QString m_selectedRobotMountId;
    QString m_selectedToolAssetId;
    QString m_selectedToolAttachmentId;
    QString m_activeCollisionDetectorId;
    QString m_markedCollisionPairRobotA;
    QString m_markedCollisionPairLinkA;
};
