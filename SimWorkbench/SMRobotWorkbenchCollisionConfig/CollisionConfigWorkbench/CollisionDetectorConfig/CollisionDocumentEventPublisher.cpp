#include "CollisionDocumentEventPublisher.h"

#include "RobotQtViewerDocumentController.h"

namespace robot_qt_viewer
{
    CollisionDocumentEventPublisher::CollisionDocumentEventPublisher(
        RobotQtViewerDocumentController& documentController)
        : m_documentController(documentController)
    {
    }

    void CollisionDocumentEventPublisher::publishCollisionChanged(
        const QString& sourceId,
        const QString& detectorId) const
    {
        RobotQtViewerCollisionPayload payload;
        payload.detectorId = detectorId;
        m_documentController.publishCollisionChanged(payload, sourceId);
        m_documentController.publishDocumentChanged(sourceId, false);
    }
}
