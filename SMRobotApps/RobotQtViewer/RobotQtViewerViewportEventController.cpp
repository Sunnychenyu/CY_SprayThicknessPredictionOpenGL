#include "RobotQtViewerViewportEventController.h"

#include "RobotQtViewerViewportPreviewState.h"
#include "RobotQtViewerViewportServices.h"

#include <utility>

namespace robot_qt_viewer
{
    RobotQtViewerViewportEventController::RobotQtViewerViewportEventController(
        RobotQtViewerViewportServices& viewportServices,
        const RobotQtViewerViewportPreviewState& viewportPreviewState,
        LinkFrameVisibleQuery linkFrameVisible,
        QObject* parent)
        : QObject(parent)
        , m_viewportServices(viewportServices)
        , m_viewportPreviewState(viewportPreviewState)
        , m_linkFrameVisible(std::move(linkFrameVisible))
    {
    }

    void RobotQtViewerViewportEventController::handleEvent(const RobotQtViewerEvent& event)
    {
        switch(event.kind) {
        case RobotQtViewerEventKind::SelectionChanged:
            applySelection(event.selection);
            break;
        case RobotQtViewerEventKind::ViewportPreviewChanged:
            applyViewportPreview(m_viewportPreviewState.lastMutation());
            break;
        case RobotQtViewerEventKind::ViewportReloaded:
            if(!event.viewport.reloadSucceeded) {
                applySelection(RobotQtViewerSelectionPayload());
            }
            break;
        default:
            break;
        }
    }

    void RobotQtViewerViewportEventController::applySelection(
        const RobotQtViewerSelectionPayload& selection)
    {
        m_viewportServices.setActiveToolFrameRobot(selection.robotId);
        if(selection.attachmentId.isEmpty()) {
            m_viewportServices.setActiveMountedAttachment(QString());
        }
        if(selection.objectFrameId.isEmpty()) {
            m_viewportServices.clearObjectFrameObjectFocus();
            m_viewportServices.selectObjectFrame(QString(), QString());
        }

        if(selection.jointName.isEmpty()) {
            m_viewportServices.selectRobotJointFrame(QString(), QString());
        }

        if(!selection.objectFrameId.isEmpty()) {
            m_viewportServices.setActivePreviewRobotMount(QString());
            m_viewportServices.setRobotMountFrameVisibility(false, false);
            m_viewportServices.clearObjectFrameObjectFocus();
            m_viewportServices.selectObjectFrame(selection.objectId, selection.objectFrameId);
            return;
        }

        if(!selection.jointName.isEmpty()) {
            m_viewportServices.setActivePreviewRobotMount(QString());
            m_viewportServices.setRobotMountFrameVisibility(false, false);
            m_viewportServices.selectRobotJointFrame(selection.robotId, selection.jointName);
            return;
        }

        if(!selection.attachmentId.isEmpty()) {
            m_viewportServices.setActivePreviewRobotMount(QString());
            m_viewportServices.setRobotMountFrameVisibility(false, false);
            m_viewportServices.selectMountedAttachment(selection.attachmentId);
            m_viewportServices.setActiveMountedAttachment(selection.attachmentId);
            return;
        }

        if(!selection.mountId.isEmpty()) {
            m_viewportServices.selectRobotMount(selection.robotId, selection.linkName, selection.mountId);
            m_viewportServices.setActivePreviewRobotMount(QString());
            m_viewportServices.setRobotMountFrameVisibility(false, false);
            return;
        }

        if(!selection.objectId.isEmpty()) {
            m_viewportServices.setActivePreviewRobotMount(QString());
            m_viewportServices.setRobotMountFrameVisibility(false, false);
            m_viewportServices.selectSceneObject(selection.objectId);
            return;
        }

        if(!selection.robotId.isEmpty()) {
            const bool showLinkFrame =
                m_linkFrameVisible && m_linkFrameVisible(selection.robotId, selection.linkName);
            m_viewportServices.selectRobotLink(selection.robotId, selection.linkName);
            m_viewportServices.setActivePreviewRobotMount(QString());
            m_viewportServices.setRobotMountFrameVisibility(showLinkFrame, false);
            return;
        }

        m_viewportServices.selectRobotLink(QString(), QString());
        m_viewportServices.setActivePreviewRobotMount(QString());
        m_viewportServices.setRobotMountFrameVisibility(false, false);
    }

    void RobotQtViewerViewportEventController::applyViewportPreview(
        const RobotQtViewerViewportPreviewPayload& preview)
    {
        if(preview.removePreviewRobotMount) {
            m_viewportServices.removePreviewRobotMount(preview.removePreviewRobotMountId);
        }
        if(preview.upsertPreviewRobotMount) {
            m_viewportServices.upsertPreviewRobotMount(preview.robotMount);
        }
        if(preview.previewRobotMountTransform) {
            m_viewportServices.previewRobotMountTransform(
                preview.previewRobotMountId,
                preview.robotMountTransform);
        }
        if(preview.setActivePreviewRobotMount) {
            m_viewportServices.setActivePreviewRobotMount(preview.activePreviewRobotMountId);
        }
        if(preview.setRobotMountFrameVisibility) {
            m_viewportServices.setRobotMountFrameVisibility(
                preview.selectedLinkFrameVisible,
                preview.mountFrameVisible);
        }
        if(preview.upsertPreviewObjectFrame) {
            m_viewportServices.upsertPreviewObjectFrame(
                preview.objectFrameObjectId,
                preview.objectFrame);
        }
        if(preview.previewRobotBaseTransform) {
            m_viewportServices.previewRobotBaseTransform(
                preview.robotBaseRobotId,
                preview.robotBaseTransform);
        }
        if(preview.previewObjectFrameTransform) {
            m_viewportServices.previewObjectFrameTransform(
                preview.previewObjectFrameObjectId,
                preview.previewObjectFrameId,
                preview.objectFrameTransform);
        }
        if(preview.previewSceneObjectTransform) {
            m_viewportServices.previewSceneObjectTransform(
                preview.sceneObjectId,
                preview.sceneObjectTransform);
        }
        if(preview.clearObjectFrameObjectFocus) {
            m_viewportServices.clearObjectFrameObjectFocus();
        }
        if(preview.focusObjectFrameObject) {
            m_viewportServices.focusObjectFrameObject(preview.focusObjectFrameObjectId);
        }
        if(preview.clearMountFrameLinkFocus) {
            m_viewportServices.clearMountFrameLinkFocus();
        }
        if(preview.focusMountFrameLink) {
            m_viewportServices.focusMountFrameLink(
                preview.focusMountFrameRobotId,
                preview.focusMountFrameLinkName);
        }
        if(preview.clearMountedAttachmentFocus) {
            m_viewportServices.clearMountedAttachmentFocus();
        }
        if(preview.focusMountedAttachment) {
            m_viewportServices.focusMountedAttachment(preview.focusMountedAttachmentId);
        }
        if(preview.clearObjectCollisionModelVariantPreview) {
            m_viewportServices.clearObjectCollisionModelVariantPreview();
        }
        if(preview.previewObjectCollisionModelVariant) {
            m_viewportServices.previewObjectCollisionModelVariant(
                preview.previewObjectCollisionModelObjectId,
                preview.previewObjectCollisionModelVariantId);
        }
        if(preview.setActiveMountedAttachment) {
            m_viewportServices.setActiveMountedAttachment(preview.activeMountedAttachmentId);
        }
        if(preview.setToolFrameVisibility) {
            m_viewportServices.setToolFrameVisibility(preview.toolFrameVisibility);
        }
        if(preview.setPinnedRobotMountFrames) {
            m_viewportServices.setPinnedRobotMountFrames(preview.pinnedRobotMountFrameIds);
        }
    }
}
