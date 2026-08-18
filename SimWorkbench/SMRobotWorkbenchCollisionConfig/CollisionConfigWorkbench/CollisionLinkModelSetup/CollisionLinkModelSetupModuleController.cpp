#include "CollisionLinkModelSetupModuleController.h"

#include "CollisionExportDocumentFacade.h"
#include "CollisionExportWorkbenchController.h"
#include "CollisionWorkbenchServices.h"
#include "CollisionLinkModelsWidget.h"
#include "CollisionModelDocumentFacade.h"
#include "CollisionModelWorkbenchController.h"
#include "CollisionQualityMessageState.h"
#include "CollisionRequestDocumentFacade.h"
#include "CollisionRequestWorkbenchController.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerViewportPreviewState.h"

#include <QObject>

#include <utility>

namespace robot_qt_viewer
{
    CollisionLinkModelSetupModuleController::CollisionLinkModelSetupModuleController(
        CollisionLinkModelsWidget& widget,
        RobotQtViewerDocumentContext& context,
        CollisionWorkbenchServices& appServices,
        Callbacks callbacks)
        : m_widget(widget)
        , m_context(context)
        , m_appServices(appServices)
        , m_callbacks(std::move(callbacks))
        , m_modelDocument(std::make_unique<CollisionModelDocumentFacade>(m_appServices.document()))
        , m_modelWorkbench(std::make_unique<CollisionModelWorkbenchController>(
              m_widget,
              m_context,
              m_appServices,
              *m_modelDocument,
              [this](bool canMarkLinkA, bool canCreateLinkLink) {
                  if(m_callbacks.setLinkPairActionsEnabled) {
                      m_callbacks.setLinkPairActionsEnabled(canMarkLinkA, canCreateLinkLink);
                  }
              }))
        , m_exportDocument(std::make_unique<CollisionExportDocumentFacade>(
              m_appServices.document(),
              m_appServices))
        , m_exportWorkbench(std::make_unique<CollisionExportWorkbenchController>(
              m_appServices,
              *m_exportDocument,
              CollisionExportWorkbenchController::Callbacks{
                  [this](const QString& message, int timeoutMs) {
                      if(m_callbacks.statusMessage) {
                          m_callbacks.statusMessage(message, timeoutMs);
                      }
                  }}))
        , m_qualityMessageState(std::make_unique<CollisionQualityMessageState>())
        , m_requestDocument(std::make_unique<CollisionRequestDocumentFacade>(
              m_appServices.document(),
              m_appServices))
        , m_requestWorkbench(std::make_unique<CollisionRequestWorkbenchController>(
              m_widget,
              m_context,
              m_appServices,
              *m_requestDocument,
              *m_qualityMessageState,
              CollisionRequestWorkbenchController::Callbacks{
                  [this]() {
                      return m_callbacks.activeDetectorId ? m_callbacks.activeDetectorId() : QString();
                  },
                  [this](const QString& message, int timeoutMs) {
                      if(m_callbacks.statusMessage) {
                          m_callbacks.statusMessage(message, timeoutMs);
                      }
                  },
                  [this](const QString& qualityMessage) {
                      if(m_callbacks.refreshElementList) {
                          m_callbacks.refreshElementList(qualityMessage);
                      }
                  },
                  [this](const QString& qualityMessage) {
                      if(m_callbacks.refreshModelSummary) {
                          m_callbacks.refreshModelSummary(qualityMessage);
                      }
                  },
                  [this]() {
                      if(m_callbacks.refreshDetectorList) {
                          m_callbacks.refreshDetectorList();
                      }
                  },
                  [this]() {
                      if(m_callbacks.refreshDetectorDetails) {
                          m_callbacks.refreshDetectorDetails();
                      }
                  },
                  [this]() {
                      if(m_callbacks.refreshDetectorProperties) {
                          m_callbacks.refreshDetectorProperties();
                      }
                  },
                  [this](const QString& role) {
                      return m_callbacks.setCurrentDetectorRole
                          ? m_callbacks.setCurrentDetectorRole(role)
                          : false;
                  },
                  [this]() {
                      return m_collisionModelConfigurationActive;
                  }}))
    {
    }

    CollisionLinkModelSetupModuleController::~CollisionLinkModelSetupModuleController() = default;

    CollisionModelWorkbenchController& CollisionLinkModelSetupModuleController::modelWorkbench()
    {
        return *m_modelWorkbench;
    }

    void CollisionLinkModelSetupModuleController::connectPanelSignals(QObject& receiver)
    {
        QObject::connect(&m_widget, &CollisionLinkModelsWidget::variantSelectionChanged,
            &receiver, [this]() {
                handleVariantSelectionChanged();
            });
        QObject::connect(&m_widget, &CollisionLinkModelsWidget::setCurrentVariantRequested,
            &receiver, [this]() {
                setSelectedVariantCurrent();
            });
        QObject::connect(&m_widget, &CollisionLinkModelsWidget::generateCoacdRequested,
            &receiver, [this]() {
                generateCoacd();
            });
        QObject::connect(&m_widget, &CollisionLinkModelsWidget::applyConfigurationRequested,
            &receiver, [this]() {
                applyCollisionModelConfiguration();
            });
        QObject::connect(&m_widget, &CollisionLinkModelsWidget::cancelConfigurationRequested,
            &receiver, [this]() {
                cancelCollisionModelConfiguration();
            });
    }

    void CollisionLinkModelSetupModuleController::refreshElementList(
        const QString& activeDetectorId,
        const QString& qualityMessage)
    {
        m_modelWorkbench->refreshElementList(activeDetectorId, qualityMessage);
        m_requestWorkbench->handleVariantSelectionChanged();
    }

    void CollisionLinkModelSetupModuleController::refreshModelSummary(
        const QString& activeDetectorId,
        const QString& qualityMessage)
    {
        m_modelWorkbench->refreshSummary(activeDetectorId, qualityMessage);
    }

    void CollisionLinkModelSetupModuleController::enterCollisionModelViewportFocus()
    {
        m_modelWorkbench->focusSelectedTargetInViewport();
    }

    void CollisionLinkModelSetupModuleController::handleVariantSelectionChanged()
    {
        m_requestWorkbench->handleVariantSelectionChanged();
    }

    void CollisionLinkModelSetupModuleController::showSelectedVariantOnly()
    {
        m_requestWorkbench->showSelectedVariantOnly();
    }

    void CollisionLinkModelSetupModuleController::setSelectedVariantCurrent()
    {
        m_requestWorkbench->setSelectedVariantCurrent();
    }

    void CollisionLinkModelSetupModuleController::clearCollisionModelViewportFocus(const QString& sourceId)
    {
        m_requestWorkbench->clearVariantPreview();
        RobotQtViewerViewportPreviewPayload mutation;
        mutation.setRobotMountFrameVisibility = true;
        mutation.selectedLinkFrameVisible = false;
        mutation.mountFrameVisible = false;
        mutation.clearMountFrameLinkFocus = true;
        mutation.clearObjectFrameObjectFocus = true;
        mutation.clearMountedAttachmentFocus = true;
        mutation.clearObjectCollisionModelVariantPreview = true;
        m_context.viewportPreviewState().mutate(mutation, sourceId);
    }

    void CollisionLinkModelSetupModuleController::applyCollisionModelConfiguration()
    {
        clearCollisionModelViewportFocus(QStringLiteral("applyCollisionModelConfiguration"));
        if(m_callbacks.exitCollisionModelConfiguration) {
            m_callbacks.exitCollisionModelConfiguration();
        }
    }

    void CollisionLinkModelSetupModuleController::cancelCollisionModelConfiguration()
    {
        m_requestWorkbench->cancelSelectedVariantChange();
        clearCollisionModelViewportFocus(QStringLiteral("cancelCollisionModelConfiguration"));
        if(m_callbacks.exitCollisionModelConfiguration) {
            m_callbacks.exitCollisionModelConfiguration();
        }
    }

    void CollisionLinkModelSetupModuleController::cancelCollisionModelConfigurationForHandoff(const QString& sourceId)
    {
        m_requestWorkbench->cancelSelectedVariantChange();
        clearCollisionModelViewportFocus(sourceId);
    }

    void CollisionLinkModelSetupModuleController::useSelectedVariantInActiveDetector()
    {
        m_requestWorkbench->useSelectedVariantInActiveDetector();
    }

    void CollisionLinkModelSetupModuleController::selectVariantBySourceRole(const QString& source, const QString& role)
    {
        m_requestWorkbench->selectVariantBySourceRole(source, role);
    }

    void CollisionLinkModelSetupModuleController::setReplaceOriginal(bool replaceOriginal, bool updating)
    {
        m_requestWorkbench->setReplaceOriginal(replaceOriginal, updating);
    }

    void CollisionLinkModelSetupModuleController::addBoxElement()
    {
        m_requestWorkbench->addBoxElement();
    }

    void CollisionLinkModelSetupModuleController::generateFromVisual(const QString& proxyType)
    {
        m_requestWorkbench->generateFromVisual(proxyType);
    }

    void CollisionLinkModelSetupModuleController::generateFromExistingCollision(const QString& proxyType)
    {
        m_requestWorkbench->generateFromExistingCollision(proxyType);
    }

    void CollisionLinkModelSetupModuleController::generateRobotFromExistingCollision(const QString& proxyType)
    {
        m_requestWorkbench->generateRobotFromExistingCollision(proxyType);
    }

    void CollisionLinkModelSetupModuleController::generateCoacd()
    {
        m_requestWorkbench->generateCoacd();
    }

    void CollisionLinkModelSetupModuleController::generateMissingFromVisual()
    {
        m_requestWorkbench->generateMissingFromVisual();
    }

    void CollisionLinkModelSetupModuleController::removeSelectedElement()
    {
        m_requestWorkbench->removeSelectedElement();
    }

    void CollisionLinkModelSetupModuleController::saveOverridesToProject()
    {
        m_exportWorkbench->saveOverridesToProject();
    }

    bool CollisionLinkModelSetupModuleController::selectedRobotHasCollisionOverrides() const
    {
        return m_exportWorkbench->selectedRobotHasCollisionOverrides();
    }

    std::filesystem::path CollisionLinkModelSetupModuleController::selectedRobotSourcePath() const
    {
        return m_exportWorkbench->selectedRobotSourcePath();
    }

    void CollisionLinkModelSetupModuleController::saveOverridesAsSidecar(
        const std::filesystem::path& sidecarPath,
        const std::string& portableSidecarPath)
    {
        m_exportWorkbench->saveOverridesAsSidecar(sidecarPath, portableSidecarPath);
    }

    void CollisionLinkModelSetupModuleController::exportRobotUrdfWithCollision(
        const std::filesystem::path& sourceUrdfPath,
        const std::filesystem::path& outputUrdfPath)
    {
        m_exportWorkbench->exportRobotUrdfWithCollision(sourceUrdfPath, outputUrdfPath);
    }

    void CollisionLinkModelSetupModuleController::setCollisionModelConfigurationActive(bool active)
    {
        if(m_collisionModelConfigurationActive && !active) {
            clearCollisionModelViewportFocus(QStringLiteral("collisionModelConfigurationInactive"));
        }
        m_collisionModelConfigurationActive = active;
    }
}
