#include "CollisionModelWorkbenchController.h"

#include "CollisionWorkbenchServices.h"
#include "CollisionLinkModelsWidget.h"
#include "CollisionModelDocumentFacade.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerViewportPreviewState.h"
#include "RobotQtViewerViewportServices.h"

#include <SimulationProject/ProjectSession.h>

#include <filesystem>
#include <utility>
#include <vector>

namespace
{
    using CollisionRuntimeDetectorInfo = robot_qt_viewer::CollisionRuntimeDetectorInfo;
    using CollisionRuntimeRobotSummary = robot_qt_viewer::CollisionRuntimeRobotSummary;

    std::filesystem::path projectBasePath(const simulation_project::ProjectSession& session)
    {
        return session.path().empty()
            ? std::filesystem::current_path()
            : session.path().parent_path();
    }
}

namespace robot_qt_viewer
{
    CollisionModelWorkbenchController::CollisionModelWorkbenchController(
        CollisionLinkModelsWidget& widget,
        RobotQtViewerDocumentContext& context,
        CollisionWorkbenchServices& appServices,
        CollisionModelDocumentFacade& documentFacade,
        std::function<void(bool, bool)> setLinkPairActionsEnabled)
        : m_widget(widget)
        , m_context(context)
        , m_appServices(appServices)
        , m_documentFacade(documentFacade)
        , m_setLinkPairActionsEnabled(std::move(setLinkPairActionsEnabled))
    {
    }

    void CollisionModelWorkbenchController::refreshElementList(
        const QString& activeDetectorId,
        const QString& qualityMessage)
    {
        if(m_refreshingElementList) {
            return;
        }

        m_refreshingElementList = true;
        const QString previousVariantId = m_widget.currentVariantId();

        QString visibleVariantId;
        CollisionRuntimeRobotSummary robotSummary;
        const CollisionRuntimeRobotSummary* robotSummaryPtr = nullptr;
        std::vector<CollisionRuntimeDetectorInfo> runtimeDetectors;
        const QString selectedObjectId = m_appServices.selectedObjectId();
        const QString selectedAttachmentId = m_appServices.selectedToolAttachmentId();
        if(m_context.viewportServices() != nullptr &&
            !m_appServices.selectedRobotId().isEmpty() &&
            !m_appServices.selectedLinkName().isEmpty()) {
            visibleVariantId = m_context.viewportServices()->visibleRobotCollisionVariant(
                m_appServices.selectedRobotId(),
                m_appServices.selectedLinkName());
            robotSummary = m_context.viewportServices()->robotCollisionSummary(
                m_appServices.selectedRobotId());
            robotSummaryPtr = &robotSummary;
            runtimeDetectors = m_context.viewportServices()->collisionRuntimeDetectors();
        }

        m_widget.setViewModel(
            m_documentFacade.buildViewModel(
                m_appServices.selectedRobotId(),
                m_appServices.selectedLinkName(),
                selectedObjectId,
                selectedAttachmentId,
                activeDetectorId,
                previousVariantId,
                visibleVariantId,
                qualityMessage,
                projectBasePath(m_appServices.session()),
                robotSummaryPtr,
                runtimeDetectors));

        const bool hasVariant = m_widget.hasCurrentVariant();
        m_widget.setVariantActionsEnabled(hasVariant, hasVariant);
        refreshSummary(activeDetectorId, qualityMessage);

        const bool hasLink = selectedObjectId.isEmpty() &&
            selectedAttachmentId.isEmpty() &&
            !m_appServices.selectedRobotId().isEmpty() &&
            !m_appServices.selectedLinkName().isEmpty();
        const bool hasObjectOrAttachment = !selectedObjectId.isEmpty() || !selectedAttachmentId.isEmpty();
        const bool hasRobotTarget = selectedObjectId.isEmpty() &&
            selectedAttachmentId.isEmpty() &&
            !m_appServices.selectedRobotId().isEmpty();
        m_widget.setContextActionsEnabled(hasRobotTarget, hasLink);
        m_widget.setTaskExitEnabled(hasLink || hasObjectOrAttachment);
        if(m_setLinkPairActionsEnabled) {
            m_setLinkPairActionsEnabled(
                hasLink,
                hasLink &&
                    !m_appServices.collisionPairRobotA().isEmpty() &&
                    !m_appServices.collisionPairLinkA().isEmpty());
        }
        m_refreshingElementList = false;
    }

    void CollisionModelWorkbenchController::refreshSummary(
        const QString& activeDetectorId,
        const QString& qualityMessage)
    {
        (void)activeDetectorId;

        CollisionRuntimeRobotSummary robotSummary;
        const CollisionRuntimeRobotSummary* robotSummaryPtr = nullptr;
        std::vector<CollisionRuntimeDetectorInfo> runtimeDetectors;
        const QString selectedObjectId = m_appServices.selectedObjectId();
        const QString selectedAttachmentId = m_appServices.selectedToolAttachmentId();
        if(m_context.viewportServices() != nullptr &&
            !m_appServices.selectedRobotId().isEmpty() &&
            !m_appServices.selectedLinkName().isEmpty()) {
            robotSummary = m_context.viewportServices()->robotCollisionSummary(
                m_appServices.selectedRobotId());
            robotSummaryPtr = &robotSummary;
            runtimeDetectors = m_context.viewportServices()->collisionRuntimeDetectors();
        }

        m_widget.setSummary(
            m_documentFacade.buildSummary(
                m_appServices.selectedRobotId(),
                m_appServices.selectedLinkName(),
                selectedObjectId,
                selectedAttachmentId,
                activeDetectorId,
                qualityMessage,
                robotSummaryPtr,
                runtimeDetectors));
    }

    void CollisionModelWorkbenchController::focusSelectedTargetInViewport()
    {
        const QString selectedObjectId = m_appServices.selectedObjectId();
        const QString selectedAttachmentId = m_appServices.selectedToolAttachmentId();
        const bool hasLink = selectedObjectId.isEmpty() &&
            selectedAttachmentId.isEmpty() &&
            !m_appServices.selectedRobotId().isEmpty() &&
            !m_appServices.selectedLinkName().isEmpty();

        RobotQtViewerViewportPreviewPayload preview;
        if(!selectedAttachmentId.isEmpty()) {
            preview.focusMountedAttachment = true;
            preview.focusMountedAttachmentId = selectedAttachmentId;
            preview.setActiveMountedAttachment = true;
            preview.activeMountedAttachmentId = selectedAttachmentId;
        } else if(!selectedObjectId.isEmpty()) {
            preview.focusObjectFrameObject = true;
            preview.focusObjectFrameObjectId = selectedObjectId;
        } else if(hasLink) {
            preview.setRobotMountFrameVisibility = true;
            preview.selectedLinkFrameVisible = true;
            preview.mountFrameVisible = false;
            preview.focusMountFrameLink = true;
            preview.focusMountFrameRobotId = m_appServices.selectedRobotId();
            preview.focusMountFrameLinkName = m_appServices.selectedLinkName();
        }

        if(preview.focusMountedAttachment || preview.focusObjectFrameObject || preview.focusMountFrameLink) {
            m_context.viewportPreviewState().mutate(preview, QStringLiteral("collisionModelConfigurationFocus"));
        }
    }
}
