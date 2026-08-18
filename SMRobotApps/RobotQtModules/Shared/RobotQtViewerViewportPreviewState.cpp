#include "RobotQtViewerViewportPreviewState.h"

#include "RobotQtViewerEventHub.h"

namespace robot_qt_viewer
{
    RobotQtViewerViewportPreviewState::RobotQtViewerViewportPreviewState(
        RobotQtViewerEventHub& eventHub)
        : m_eventHub(eventHub)
    {
    }

    const RobotQtViewerViewportPreviewPayload& RobotQtViewerViewportPreviewState::lastMutation() const
    {
        return m_lastMutation;
    }

    void RobotQtViewerViewportPreviewState::mutate(
        const RobotQtViewerViewportPreviewPayload& mutation,
        const QString& sourceId)
    {
        m_lastMutation = mutation;
        publish(sourceId);
    }

    void RobotQtViewerViewportPreviewState::clearTaskPreview(const QString& sourceId)
    {
        RobotQtViewerViewportPreviewPayload mutation;
        mutation.setActivePreviewRobotMount = true;
        mutation.activePreviewRobotMountId = QString();
        mutation.setRobotMountFrameVisibility = true;
        mutation.selectedLinkFrameVisible = false;
        mutation.mountFrameVisible = false;
        mutation.clearMountFrameLinkFocus = true;
        mutation.clearObjectFrameObjectFocus = true;
        mutation.clearMountedAttachmentFocus = true;
        mutation.clearObjectCollisionModelVariantPreview = true;
        mutation.setActiveMountedAttachment = true;
        mutation.activeMountedAttachmentId = QString();
        mutate(mutation, sourceId);
    }

    void RobotQtViewerViewportPreviewState::publish(const QString& sourceId)
    {
        RobotQtViewerEvent event;
        event.kind = RobotQtViewerEventKind::ViewportPreviewChanged;
        event.sourceId = sourceId;
        m_eventHub.publish(event);
    }
}
