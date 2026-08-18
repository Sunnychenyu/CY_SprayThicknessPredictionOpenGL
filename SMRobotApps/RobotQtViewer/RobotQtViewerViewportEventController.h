#pragma once

#include "RobotQtViewerEvents.h"

#include <QObject>
#include <QString>

#include <functional>

namespace robot_qt_viewer
{
    class RobotQtViewerViewportServices;
    class RobotQtViewerViewportPreviewState;

    class RobotQtViewerViewportEventController : public QObject
    {
    public:
        using LinkFrameVisibleQuery = std::function<bool(const QString&, const QString&)>;

        RobotQtViewerViewportEventController(
            RobotQtViewerViewportServices& viewportServices,
            const RobotQtViewerViewportPreviewState& viewportPreviewState,
            LinkFrameVisibleQuery linkFrameVisible,
            QObject* parent = nullptr);

        void handleEvent(const RobotQtViewerEvent& event);

    private:
        void applySelection(const RobotQtViewerSelectionPayload& selection);
        void applyViewportPreview(const RobotQtViewerViewportPreviewPayload& preview);

        RobotQtViewerViewportServices& m_viewportServices;
        const RobotQtViewerViewportPreviewState& m_viewportPreviewState;
        LinkFrameVisibleQuery m_linkFrameVisible;
    };
}
