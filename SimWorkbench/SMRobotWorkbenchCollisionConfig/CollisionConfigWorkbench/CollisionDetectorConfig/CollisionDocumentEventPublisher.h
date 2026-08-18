#pragma once

#include <QString>

namespace robot_qt_viewer
{
    class RobotQtViewerDocumentController;

    class CollisionDocumentEventPublisher
    {
    public:
        explicit CollisionDocumentEventPublisher(RobotQtViewerDocumentController& documentController);

        void publishCollisionChanged(
            const QString& sourceId,
            const QString& detectorId = QString()) const;

    private:
        RobotQtViewerDocumentController& m_documentController;
    };
}
