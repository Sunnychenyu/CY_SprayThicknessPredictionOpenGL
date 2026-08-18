#pragma once

#include "RobotQtViewerEvents.h"

#include <QString>

namespace robot_qt_viewer
{
    class RobotQtViewerEventHub;

    class RobotQtViewerViewportPreviewState
    {
    public:
        explicit RobotQtViewerViewportPreviewState(RobotQtViewerEventHub& eventHub);

        const RobotQtViewerViewportPreviewPayload& lastMutation() const;

        void mutate(
            const RobotQtViewerViewportPreviewPayload& mutation,
            const QString& sourceId = QString());
        void clearTaskPreview(const QString& sourceId = QString());

    private:
        void publish(const QString& sourceId);

        RobotQtViewerEventHub& m_eventHub;
        RobotQtViewerViewportPreviewPayload m_lastMutation;
    };
}
