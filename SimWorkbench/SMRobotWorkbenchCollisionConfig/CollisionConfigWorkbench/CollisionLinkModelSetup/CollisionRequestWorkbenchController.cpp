#include "CollisionRequestWorkbenchController.h"

#include "CollisionWorkbenchServices.h"
#include "CollisionLinkModelsWidget.h"
#include "CollisionQualityMessageState.h"
#include "CollisionRequestDocumentFacade.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerViewportPreviewState.h"
#include "RobotQtViewerViewportServices.h"

#include <SimulationProject/CollisionModelSelectionIds.h>

#include <utility>

namespace robot_qt_viewer
{
    CollisionRequestWorkbenchController::CollisionRequestWorkbenchController(
        CollisionLinkModelsWidget& widget,
        RobotQtViewerDocumentContext& context,
        CollisionWorkbenchServices& appServices,
        CollisionRequestDocumentFacade& documentFacade,
        CollisionQualityMessageState& qualityMessageState,
        Callbacks callbacks)
        : m_widget(widget)
        , m_context(context)
        , m_appServices(appServices)
        , m_documentFacade(documentFacade)
        , m_qualityMessageState(qualityMessageState)
        , m_callbacks(std::move(callbacks))
    {
    }

    void CollisionRequestWorkbenchController::handleVariantSelectionChanged()
    {
        const bool hasVariant = m_widget.hasCurrentVariant();
        m_widget.setVariantActionsEnabled(hasVariant, hasVariant);

        if(hasVariant &&
            (!m_callbacks.isCollisionModelConfigurationActive ||
                m_callbacks.isCollisionModelConfigurationActive()) &&
            m_context.viewportServices() != nullptr) {
            previewVariant(m_widget.currentVariantId(), m_widget.currentVariantSource());
        }
        refreshModelSummary(QString());
    }

    void CollisionRequestWorkbenchController::showSelectedVariantOnly()
    {
        if(!m_widget.hasCurrentVariant() ||
            m_appServices.selectedRobotId().isEmpty() ||
            m_appServices.selectedLinkName().isEmpty()) {
            showStatus("Select a collision model variant first.", 3000);
            return;
        }

        const QString variantId = m_widget.currentVariantId();
        if(variantId.isEmpty()) {
            showStatus("Selected collision model variant has no id.", 3000);
            return;
        }

        const CollisionLinkModelVariantCommandResult result =
            CollisionLinkModelVariantCommandController::showVariantOnly(
                m_context.viewportServices(),
                m_appServices.selectedRobotId(),
                m_appServices.selectedLinkName(),
                variantId);
        if(!result.success) {
            showStatus(result.message, 3000);
            return;
        }

        refreshElementList(QString());
        showStatus(result.message, 3000);
    }

    void CollisionRequestWorkbenchController::setSelectedVariantCurrent()
    {
        if(!m_widget.hasCurrentVariant()) {
            showStatus("Select a collision model variant first.", 3000);
            return;
        }

        const QString selectedVariantId = m_widget.currentVariantId();
        const QString selectedVariantSource = m_widget.currentVariantSource();
        const QString selectedAttachmentId = m_appServices.selectedToolAttachmentId();
        const QString selectedObjectId = !selectedAttachmentId.isEmpty()
            ? selectedAttachmentId
            : m_appServices.selectedObjectId();
        CollisionLinkModelVariantCommandResult result;
        if(!selectedObjectId.isEmpty()) {
            result = m_documentFacade.setCurrentObjectCollisionModel(
                selectedObjectId,
                selectedVariantId);
        } else if(!m_appServices.selectedRobotId().isEmpty() &&
            !m_appServices.selectedLinkName().isEmpty()) {
            result = m_documentFacade.setCurrentLinkCollisionModel(
                m_appServices.selectedRobotId(),
                m_appServices.selectedLinkName(),
                selectedVariantId);
        } else {
            showStatus("Select a collision model target first.", 3000);
            return;
        }
        if(!result.success) {
            showStatus(result.message, 3000);
            return;
        }

        refreshElementList(QString());
        refreshModelSummary(QString());
        const QString refreshSource = !selectedObjectId.isEmpty()
            ? QStringLiteral("setCurrentObjectCollisionModelVariant")
            : QStringLiteral("setCurrentCollisionModelVariant");
        if(!m_appServices.refreshViewportCollisionConfiguration(refreshSource)) {
            m_appServices.reloadViewport(refreshSource);
        }
        previewVariant(selectedVariantId, selectedVariantSource);
        showStatus(result.message, 3000);
    }

    void CollisionRequestWorkbenchController::cancelSelectedVariantChange()
    {
        const QString appliedId = m_widget.appliedVariantId();
        const QString appliedSource = m_widget.appliedVariantSource();
        if(appliedId.isEmpty()) {
            clearVariantPreview();
            refreshElementList(QString());
            refreshModelSummary(QString());
            showStatus("Collision model selection canceled.", 2000);
            return;
        }

        if(!m_widget.selectAppliedVariant()) {
            previewVariant(appliedId, appliedSource);
        }
        refreshModelSummary(QString());
        showStatus("Collision model selection canceled.", 2000);
    }

    void CollisionRequestWorkbenchController::clearRobotVariantPreview()
    {
        if(m_context.viewportServices() != nullptr &&
            !m_previewRobotId.isEmpty() &&
            !m_previewLinkName.isEmpty()) {
            m_context.viewportServices()->setVisibleRobotCollisionVariant(
                m_previewRobotId,
                m_previewLinkName,
                QString());
        }
        m_previewRobotId.clear();
        m_previewLinkName.clear();
    }

    void CollisionRequestWorkbenchController::clearVariantPreview()
    {
        clearRobotVariantPreview();
        if(m_context.viewportServices() != nullptr) {
            m_context.viewportServices()->clearObjectCollisionModelVariantPreview();
        }
    }

    void CollisionRequestWorkbenchController::useSelectedVariantInActiveDetector()
    {
        if(!m_widget.hasCurrentVariant()) {
            showStatus("Select a collision model variant first.", 3000);
            return;
        }

        const QString role = m_widget.currentVariantRole();
        if(role.isEmpty()) {
            showStatus("Selected collision model variant has no detector role.", 3000);
            return;
        }
        const QString source = m_widget.currentVariantSource();
        if(source.isEmpty()) {
            showStatus("Selected collision model variant has no source.", 3000);
            return;
        }

        const QString detectorId = activeDetectorId();
        if(detectorId.isEmpty()) {
            showStatus("Select an active collision detector first.", 3000);
            return;
        }

        if(!m_callbacks.setCurrentDetectorRole || !m_callbacks.setCurrentDetectorRole(role)) {
            showStatus(QString("Detector role is not available: %1").arg(role), 5000);
            return;
        }

        const CollisionLinkModelVariantCommandResult result =
            m_documentFacade.useVariantInDetector(
                m_context.viewportServices(),
                detectorId,
                role,
                source);
        if(!result.success) {
            showStatus(result.message, 3000);
            refreshDetectorProperties();
            return;
        }

        if(result.runtimeUpdateFailed) {
            showStatus("Runtime detector update failed; rebuilding viewport scene.", 3000);
            m_appServices.reloadViewport(QStringLiteral("useSelectedCollisionVariantInActiveDetector"));
        }
        m_appServices.setActiveCollisionDetectorContext(detectorId);

        showStatus(result.message, 3000);
    }

    void CollisionRequestWorkbenchController::selectVariantBySourceRole(
        const QString& source,
        const QString& role)
    {
        if(m_widget.selectVariantBySourceRole(source, role)) {
            handleVariantSelectionChanged();
        }
    }

    void CollisionRequestWorkbenchController::previewVariant(
        const QString& variantId,
        const QString& source)
    {
        (void)source;
        if(m_context.viewportServices() == nullptr ||
            (m_appServices.selectedRobotId().isEmpty() &&
                m_appServices.selectedObjectId().isEmpty() &&
                m_appServices.selectedToolAttachmentId().isEmpty())) {
            return;
        }

        if(!m_appServices.selectedToolAttachmentId().isEmpty()) {
            clearRobotVariantPreview();
            RobotQtViewerViewportPreviewPayload preview;
            preview.focusMountedAttachment = true;
            preview.focusMountedAttachmentId = m_appServices.selectedToolAttachmentId();
            preview.setActiveMountedAttachment = true;
            preview.activeMountedAttachmentId = m_appServices.selectedToolAttachmentId();
            preview.previewObjectCollisionModelVariant = true;
            preview.previewObjectCollisionModelObjectId = m_appServices.selectedToolAttachmentId();
            preview.previewObjectCollisionModelVariantId = variantId;
            m_context.viewportPreviewState().mutate(preview, QStringLiteral("collisionObjectVariantPreview"));
            return;
        }

        if(!m_appServices.selectedObjectId().isEmpty()) {
            clearRobotVariantPreview();
            RobotQtViewerViewportPreviewPayload preview;
            preview.focusObjectFrameObject = true;
            preview.focusObjectFrameObjectId = m_appServices.selectedObjectId();
            preview.previewObjectCollisionModelVariant = true;
            preview.previewObjectCollisionModelObjectId = m_appServices.selectedObjectId();
            preview.previewObjectCollisionModelVariantId = variantId;
            m_context.viewportPreviewState().mutate(preview, QStringLiteral("collisionObjectVariantPreview"));
            return;
        }

        if(m_appServices.selectedRobotId().isEmpty() ||
            m_appServices.selectedLinkName().isEmpty()) {
            return;
        }

        const QString robotId = m_appServices.selectedRobotId();
        const QString linkName = m_appServices.selectedLinkName();
        if(m_previewRobotId != robotId || m_previewLinkName != linkName) {
            clearRobotVariantPreview();
        }

        if(m_context.viewportServices()->setVisibleRobotCollisionVariant(
               robotId,
               linkName,
               variantId)) {
            m_previewRobotId = robotId;
            m_previewLinkName = linkName;
        }
    }

    void CollisionRequestWorkbenchController::setReplaceOriginal(bool replaceOriginal, bool updating)
    {
        if(updating || m_appServices.selectedRobotId().isEmpty()) {
            return;
        }

        const CollisionLinkModelDocumentCommandResult result =
            m_documentFacade.setReplaceOriginal(
                m_appServices.selectedRobotId(),
                replaceOriginal);
        if(!result.success) {
            showStatus(result.message, 3000);
            return;
        }
        if(!result.projectChanged) {
            return;
        }

        m_appServices.reloadViewport(QStringLiteral("handleCollisionReplaceOriginalChanged"));
        showStatus(result.message, 3000);
    }

    void CollisionRequestWorkbenchController::addBoxElement()
    {
        const CollisionLinkModelDocumentCommandResult result =
            m_documentFacade.addBoxElement(
                m_appServices.selectedRobotId(),
                m_appServices.selectedLinkName());
        if(!result.success) {
            showStatus(result.message, 3000);
            return;
        }

        m_appServices.reloadViewport(QStringLiteral("addBoxCollisionElement"));
        selectVariantBySourceRole(result.selectedSource, result.selectedRole);
        showStatus(result.message, 3000);
    }

    void CollisionRequestWorkbenchController::generateFromVisual(const QString& proxyType)
    {
        if(m_appServices.selectedRobotId().isEmpty() || m_appServices.selectedLinkName().isEmpty()) {
            showStatus("Select a robot link first.", 3000);
            return;
        }
        if(m_context.viewportServices() == nullptr) {
            return;
        }

        const QString selectedRobotId = m_appServices.selectedRobotId();
        const QString selectedLinkName = m_appServices.selectedLinkName();

        const CollisionRuntimeProxyRequest request =
            m_widget.proxyRequest(proxyType, "PlanningProxy", false);
        const bool replaceOriginal = m_widget.replaceOriginal();
        const CollisionLinkModelsCommandResult result =
            m_documentFacade.generateFromVisual(
                m_context.viewportServices(),
                selectedRobotId,
                selectedLinkName,
                request,
                replaceOriginal);
        if(!result.success) {
            showStatus(result.message, 4000);
            return;
        }

        m_qualityMessageState.setMessage(result.qualityMessage);
        m_appServices.reloadViewport(QStringLiteral("generateCollisionProxyFromVisual"));
        selectGeneratedVariant(result.selectedSource, result.selectedRole, true);
        showStatus(result.message, 3000);
    }

    void CollisionRequestWorkbenchController::generateFromExistingCollision(const QString& proxyType)
    {
        if(m_appServices.selectedRobotId().isEmpty() || m_appServices.selectedLinkName().isEmpty()) {
            showStatus("Select a robot link first.", 3000);
            return;
        }
        if(m_context.viewportServices() == nullptr) {
            return;
        }

        const std::string fallbackRole = proxyType == "sphereCover"
            ? std::string("SphereCover")
            : std::string("PlanningProxy");
        const CollisionRuntimeProxyRequest request =
            m_widget.proxyRequest(proxyType, fallbackRole, true);
        const bool replaceOriginal = m_widget.replaceOriginal();
        const CollisionLinkModelsCommandResult result =
            m_documentFacade.generateFromExistingCollision(
                m_context.viewportServices(),
                m_appServices.selectedRobotId(),
                m_appServices.selectedLinkName(),
                request,
                replaceOriginal);
        if(!result.success) {
            showStatus(result.message, 4000);
            return;
        }

        m_qualityMessageState.setMessage(result.qualityMessage);
        m_appServices.reloadViewport(QStringLiteral("generateCollisionProxyFromExistingCollision"));
        selectGeneratedVariant(result.selectedSource, result.selectedRole, true);
        showStatus(result.message, 3000);
    }

    void CollisionRequestWorkbenchController::generateRobotFromExistingCollision(const QString& proxyType)
    {
        if(m_appServices.selectedRobotId().isEmpty()) {
            showStatus("Select a robot first.", 3000);
            return;
        }
        if(m_context.viewportServices() == nullptr) {
            return;
        }

        const QString selectedRobotId = m_appServices.selectedRobotId();

        const std::string fallbackRole = proxyType == "sphereCover"
            ? std::string("SphereCover")
            : std::string("PlanningProxy");
        const CollisionRuntimeProxyRequest request =
            m_widget.proxyRequest(proxyType, fallbackRole, true);
        const bool replaceOriginal = m_widget.replaceOriginal();
        const CollisionLinkModelsCommandResult result =
            m_documentFacade.generateRobotFromExistingCollision(
                m_context.viewportServices(),
                selectedRobotId,
                request,
                replaceOriginal);
        if(!result.success) {
            showStatus(result.message, 4000);
            return;
        }

        m_qualityMessageState.setMessage(result.qualityMessage);
        m_appServices.reloadViewport(QStringLiteral("generateRobotCollisionProxyFromExistingCollision"));
        selectGeneratedVariant(result.selectedSource, result.selectedRole, false);
        showStatus(result.message, 3000);
    }

    void CollisionRequestWorkbenchController::generateMissingFromVisual()
    {
        if(m_appServices.selectedRobotId().isEmpty()) {
            showStatus("Select a robot first.", 3000);
            return;
        }
        if(m_context.viewportServices() == nullptr) {
            return;
        }

        const QString selectedRobotId = m_appServices.selectedRobotId();
        const CollisionRuntimeProxyRequest request =
            m_widget.proxyRequest("box", "PlanningProxy", false);
        const bool replaceOriginal = m_widget.replaceOriginal();
        const CollisionLinkModelsCommandResult result =
            m_documentFacade.generateMissingFromVisual(
                m_context.viewportServices(),
                selectedRobotId,
                request,
                replaceOriginal);
        if(!result.success) {
            showStatus(result.message, 4000);
            return;
        }

        m_qualityMessageState.setMessage(result.qualityMessage);
        m_appServices.reloadViewport(QStringLiteral("generateMissingCollisionProxiesFromVisual"));
        selectGeneratedVariant(result.selectedSource, result.selectedRole, false);
        showStatus(result.message, 3000);
    }

    void CollisionRequestWorkbenchController::generateCoacd()
    {
        if(m_context.viewportServices() == nullptr) {
            showStatus("Viewport is not available.", 3000);
            return;
        }

        const QString selectedAttachmentId = m_appServices.selectedToolAttachmentId();
        const QString selectedObjectId = !selectedAttachmentId.isEmpty()
            ? selectedAttachmentId
            : m_appServices.selectedObjectId();
        CollisionLinkModelsCommandResult result;
        QString reloadSource;
        if(!selectedObjectId.isEmpty()) {
            result = m_documentFacade.generateCoacdForObject(
                m_context.viewportServices(),
                selectedObjectId);
            reloadSource = QStringLiteral("generateObjectCoacdCollisionModel");
        } else if(!m_appServices.selectedRobotId().isEmpty() &&
            !m_appServices.selectedLinkName().isEmpty()) {
            result = m_documentFacade.generateCoacdForLink(
                m_context.viewportServices(),
                m_appServices.selectedRobotId(),
                m_appServices.selectedLinkName());
            reloadSource = QStringLiteral("generateRobotCoacdCollisionModel");
        } else {
            showStatus("Select a robot link, object, or attachment first.", 3000);
            return;
        }

        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }

        m_qualityMessageState.setMessage(result.qualityMessage);
        m_appServices.reloadViewport(reloadSource);
        refreshElementList(result.qualityMessage);
        refreshModelSummary(result.qualityMessage);
        selectGeneratedVariant(result.selectedSource, result.selectedRole, false);
        showStatus(result.message, 3000);
    }

    void CollisionRequestWorkbenchController::removeSelectedElement()
    {
        const QString elementId = m_widget.currentElementId();
        if(elementId.isEmpty()) {
            showStatus("No collision element selected.", 3000);
            return;
        }

        const CollisionLinkModelDocumentCommandResult result =
            m_documentFacade.removeElement(
                m_appServices.selectedRobotId(),
                m_appServices.selectedLinkName(),
                elementId);
        if(!result.success) {
            showStatus(result.message, 3000);
            return;
        }

        m_appServices.reloadViewport(QStringLiteral("removeSelectedCollisionElement"));
        showStatus(result.message, 3000);
    }

    QString CollisionRequestWorkbenchController::activeDetectorId() const
    {
        return m_callbacks.activeDetectorId ? m_callbacks.activeDetectorId() : QString();
    }

    void CollisionRequestWorkbenchController::showStatus(const QString& message, int timeoutMs) const
    {
        if(m_callbacks.statusMessage) {
            m_callbacks.statusMessage(message, timeoutMs);
        }
    }

    void CollisionRequestWorkbenchController::refreshElementList(const QString& qualityMessage) const
    {
        if(m_callbacks.refreshElementList) {
            m_callbacks.refreshElementList(qualityMessage);
        }
    }

    void CollisionRequestWorkbenchController::refreshModelSummary(const QString& qualityMessage) const
    {
        if(m_callbacks.refreshModelSummary) {
            m_callbacks.refreshModelSummary(qualityMessage);
        }
    }

    void CollisionRequestWorkbenchController::refreshDetectorList() const
    {
        if(m_callbacks.refreshDetectorList) {
            m_callbacks.refreshDetectorList();
        }
    }

    void CollisionRequestWorkbenchController::refreshDetectorDetails() const
    {
        if(m_callbacks.refreshDetectorDetails) {
            m_callbacks.refreshDetectorDetails();
        }
    }

    void CollisionRequestWorkbenchController::refreshDetectorProperties() const
    {
        if(m_callbacks.refreshDetectorProperties) {
            m_callbacks.refreshDetectorProperties();
        }
    }

    void CollisionRequestWorkbenchController::selectGeneratedVariant(
        const QString& source,
        const QString& role,
        bool useInDetector)
    {
        if(source.isEmpty() && role.isEmpty()) {
            return;
        }

        selectVariantBySourceRole(source, role);
        if(useInDetector) {
            useSelectedVariantInActiveDetector();
        }
    }
}
