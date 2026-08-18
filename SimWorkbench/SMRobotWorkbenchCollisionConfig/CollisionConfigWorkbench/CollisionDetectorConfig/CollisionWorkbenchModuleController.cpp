#include "CollisionWorkbenchModuleController.h"

#include "CollisionDetectorConfigModuleController.h"
#include "CollisionDocumentEventPublisher.h"
#include "CollisionWorkbenchEventCoordinator.h"
#include "CollisionWorkbenchServices.h"
#include "CollisionWorkbenchPanel.h"
#include "CollisionLinkModelSetupModuleController.h"
#include "RobotQtViewerDocumentContext.h"

#include <utility>

namespace robot_qt_viewer
{
    CollisionWorkbenchModuleController::CollisionWorkbenchModuleController(
        CollisionWorkbenchPanel& panel,
        RobotQtViewerDocumentContext& context,
        CollisionWorkbenchServices& appServices,
        QObject* parent)
        : QObject(parent)
        , m_panel(panel)
        , m_context(context)
        , m_appServices(appServices)
        , m_documentEvents(std::make_unique<CollisionDocumentEventPublisher>(m_context.documentController()))
        , m_linkModelSetupController(std::make_unique<CollisionLinkModelSetupModuleController>(
              *m_panel.linkModelsWidget(),
              m_context,
              m_appServices,
              CollisionLinkModelSetupModuleController::Callbacks{
                  [this]() {
                      return isUpdating();
                  },
                  [this]() {
                      return currentCollisionDetectorId();
                  },
                  [this](const QString& qualityMessage) {
                      refreshCollisionElementList(qualityMessage);
                  },
                  [this](const QString& qualityMessage) {
                      refreshCollisionModelSummary(qualityMessage);
                  },
                  [this]() {
                      refreshCollisionDetectorList();
                  },
                  [this]() {
                      refreshCollisionDetectorDetails();
                  },
                  [this]() {
                      refreshCollisionDetectorPropertyEditors();
                  },
                  [this](bool canMarkLinkA, bool canCreateLinkLink) {
                      m_panel.setLinkPairActionsEnabled(canMarkLinkA, canCreateLinkLink);
                  },
                  [this](const QString& role) {
                      return m_panel.setCurrentDetectorRole(role);
                  },
                  [this](const QString& message, int timeoutMs) {
                      emit statusMessageRequested(message, timeoutMs);
                  },
                  [this]() {
                      emit saveCollisionOverridesSidecarRequested();
                  },
                  [this]() {
                      emit exportCollisionUrdfRequested();
                  },
                  [this]() {
                      showDetectorConfiguration();
                  }}))
        , m_detectorConfigController(std::make_unique<CollisionDetectorConfigModuleController>(
              m_panel,
              m_context,
              m_appServices,
              *m_documentEvents,
              CollisionDetectorConfigModuleController::Callbacks{
                  [this]() {
                      return isUpdating();
                  },
                  [this]() {
                      refreshCollisionSelectionSetList();
                  },
                  [this]() {
                      refreshCollisionDetectorList();
                  },
                  [this]() {
                      refreshCollisionDetectorPropertyEditors();
                  },
                  [this]() {
                      refreshCollisionDetectorDetails();
                  },
                  [this](const QString& qualityMessage) {
                      refreshCollisionElementList(qualityMessage);
                  },
                  [this]() {
                      emit viewportReloadRequested();
                  },
                  [this](const QString& message, int timeoutMs) {
                      emit statusMessageRequested(message, timeoutMs);
                  }},
              m_updating))
    {
        m_linkModelSetupController->connectPanelSignals(*this);
        m_detectorConfigController->connectPanelSignals(*this);
    }

    CollisionWorkbenchModuleController::~CollisionWorkbenchModuleController() = default;

    void CollisionWorkbenchModuleController::handleEvent(const RobotQtViewerEvent& event)
    {
        CollisionWorkbenchEventCoordinator::dispatchEvent(
            event,
            CollisionWorkbenchEventCoordinator::Handlers{
                [this]() {
                    emit refreshInspectorRequested();
                },
                [this]() {
                    emit refreshSelectionDependentViewsRequested();
                }});
    }

    bool CollisionWorkbenchModuleController::isUpdating() const
    {
        return m_updating;
    }

    bool CollisionWorkbenchModuleController::isCollisionModelConfigurationActive() const
    {
        return m_collisionModelConfigurationActive;
    }

    bool CollisionWorkbenchModuleController::canHandoffCollisionModelConfiguration() const
    {
        return true;
    }

    QString CollisionWorkbenchModuleController::currentCollisionSelectionSetId() const
    {
        return m_detectorConfigController->currentSelectionSetId();
    }

    QString CollisionWorkbenchModuleController::currentCollisionDetectorId() const
    {
        return m_detectorConfigController->currentDetectorId();
    }

    void CollisionWorkbenchModuleController::refreshInspector(const QString& qualityMessage)
    {
        refreshCollisionSelectionSetList();
        refreshCollisionDetectorList();
        refreshCollisionElementList(qualityMessage);
        refreshCollisionPairList();
    }

    void CollisionWorkbenchModuleController::refreshCollisionPairList()
    {
        m_detectorConfigController->refreshPairList();
    }

    void CollisionWorkbenchModuleController::refreshCollisionDetectorList()
    {
        m_detectorConfigController->refreshDetectorList();
    }

    void CollisionWorkbenchModuleController::refreshCollisionSelectionSetList()
    {
        m_detectorConfigController->refreshSelectionSetList();
    }

    void CollisionWorkbenchModuleController::refreshCollisionSelectionSetMemberList()
    {
        m_detectorConfigController->refreshSelectionSetMemberList();
    }

    void CollisionWorkbenchModuleController::refreshCollisionDetectorPropertyEditors()
    {
        m_detectorConfigController->refreshDetectorPropertyEditors();
    }

    void CollisionWorkbenchModuleController::refreshCollisionDetectorDetails()
    {
        // Runtime result details are shown by run/analysis modules, not CollisionConfigMode.
    }

    void CollisionWorkbenchModuleController::refreshCollisionElementList(const QString& qualityMessage)
    {
        m_linkModelSetupController->refreshElementList(currentCollisionDetectorId(), qualityMessage);
    }

    void CollisionWorkbenchModuleController::refreshCollisionModelSummary(const QString& qualityMessage)
    {
        m_linkModelSetupController->refreshModelSummary(currentCollisionDetectorId(), qualityMessage);
    }

    bool CollisionWorkbenchModuleController::syncLegacyRobotObjectPairs()
    {
        return m_detectorConfigController->syncLegacyRobotObjectPairs();
    }

    void CollisionWorkbenchModuleController::autoPairAllRobotObjects()
    {
        m_detectorConfigController->autoPairAllRobotObjects();
    }

    void CollisionWorkbenchModuleController::handleLegacyPairEnabledChanged(
        const QString& robotId,
        const QString& objectId,
        bool enabled)
    {
        m_detectorConfigController->setLegacyPairEnabled(robotId, objectId, enabled);
    }

    void CollisionWorkbenchModuleController::previewCollisionSelectionSetMember()
    {
        m_detectorConfigController->previewSelectionSetMember();
    }

    void CollisionWorkbenchModuleController::addCollisionSelectionSet()
    {
        m_detectorConfigController->addSelectionSet();
    }

    void CollisionWorkbenchModuleController::renameSelectedCollisionSelectionSet()
    {
        m_detectorConfigController->renameSelectedSelectionSet();
    }

    void CollisionWorkbenchModuleController::removeSelectedCollisionSelectionSet()
    {
        m_detectorConfigController->removeSelectedSelectionSet();
    }

    void CollisionWorkbenchModuleController::removeSelectedCollisionSelectionSetMember()
    {
        m_detectorConfigController->removeSelectedSelectionSetMember();
    }

    void CollisionWorkbenchModuleController::handleCollisionDetectorEnabledChanged(const QString& detectorId, bool enabled)
    {
        m_detectorConfigController->setDetectorEnabled(detectorId, enabled);
    }

    void CollisionWorkbenchModuleController::handleCollisionDetectorSelectionChanged()
    {
        m_detectorConfigController->handleDetectorSelectionChanged();
    }

    void CollisionWorkbenchModuleController::handleCollisionDetectorPropertyChanged()
    {
        m_detectorConfigController->applyDetectorProperties();
    }

    void CollisionWorkbenchModuleController::previewCollisionDetectorSelection()
    {
        m_detectorConfigController->previewDetectorSelection();
    }

    void CollisionWorkbenchModuleController::showSelectedCollisionDetectors()
    {
        m_detectorConfigController->showSelectedDetectors();
    }

    void CollisionWorkbenchModuleController::removeSelectedCollisionDetector()
    {
        m_detectorConfigController->removeSelectedDetector();
    }

    void CollisionWorkbenchModuleController::addCollisionDetectorFromTaskPanel()
    {
        m_detectorConfigController->addDetectorFromTaskPanel();
    }

    void CollisionWorkbenchModuleController::editSelectedCollisionDetector()
    {
        m_detectorConfigController->editSelectedDetector();
    }

    void CollisionWorkbenchModuleController::addSceneAllCollisionDetector()
    {
        m_detectorConfigController->addSceneAllDetector();
    }

    void CollisionWorkbenchModuleController::showDetectorConfiguration()
    {
        m_collisionModelConfigurationActive = false;
        m_linkModelSetupController->setCollisionModelConfigurationActive(false);
        m_panel.showDetectorConfiguration();
        refreshInspector(QString());
    }

    void CollisionWorkbenchModuleController::showCollisionModelConfiguration()
    {
        if(m_collisionModelConfigurationActive) {
            cancelCollisionModelConfigurationForHandoff(QStringLiteral("showCollisionModelConfigurationHandoff"));
        }
        m_collisionModelConfigurationActive = true;
        m_linkModelSetupController->setCollisionModelConfigurationActive(true);
        m_panel.showCollisionModelConfiguration();
        refreshCollisionElementList(QString());
        refreshCollisionModelSummary(QString());
        m_linkModelSetupController->enterCollisionModelViewportFocus();
        m_linkModelSetupController->handleVariantSelectionChanged();
    }

    void CollisionWorkbenchModuleController::cancelCollisionModelConfigurationForHandoff(const QString& sourceId)
    {
        if(!m_collisionModelConfigurationActive) {
            return;
        }
        m_collisionModelConfigurationActive = false;
        m_linkModelSetupController->setCollisionModelConfigurationActive(false);
        m_linkModelSetupController->cancelCollisionModelConfigurationForHandoff(sourceId);
    }

    void CollisionWorkbenchModuleController::handleCollisionVariantSelectionChanged()
    {
        if(!m_collisionModelConfigurationActive) {
            return;
        }
        m_linkModelSetupController->handleVariantSelectionChanged();
    }

    void CollisionWorkbenchModuleController::showSelectedCollisionVariantOnly()
    {
        m_linkModelSetupController->showSelectedVariantOnly();
    }

    void CollisionWorkbenchModuleController::useSelectedCollisionVariantInActiveDetector()
    {
        m_linkModelSetupController->useSelectedVariantInActiveDetector();
    }

    void CollisionWorkbenchModuleController::selectCollisionVariantBySourceRole(const QString& source, const QString& role)
    {
        m_linkModelSetupController->selectVariantBySourceRole(source, role);
    }

    void CollisionWorkbenchModuleController::handleCollisionReplaceOriginalChanged(bool replaceOriginal)
    {
        m_linkModelSetupController->setReplaceOriginal(replaceOriginal, isUpdating());
    }

    void CollisionWorkbenchModuleController::addBoxCollisionElement()
    {
        m_linkModelSetupController->addBoxElement();
    }

    void CollisionWorkbenchModuleController::generateCollisionProxyFromVisual(const QString& proxyType)
    {
        m_linkModelSetupController->generateFromVisual(proxyType);
    }

    void CollisionWorkbenchModuleController::generateCollisionProxyFromExistingCollision(const QString& proxyType)
    {
        m_linkModelSetupController->generateFromExistingCollision(proxyType);
    }

    void CollisionWorkbenchModuleController::generateRobotCollisionProxyFromExistingCollision(const QString& proxyType)
    {
        m_linkModelSetupController->generateRobotFromExistingCollision(proxyType);
    }

    void CollisionWorkbenchModuleController::generateCollisionCoacd()
    {
        m_linkModelSetupController->generateCoacd();
    }

    void CollisionWorkbenchModuleController::generateMissingCollisionProxiesFromVisual()
    {
        m_linkModelSetupController->generateMissingFromVisual();
    }

    void CollisionWorkbenchModuleController::removeSelectedCollisionElement()
    {
        m_linkModelSetupController->removeSelectedElement();
    }

    void CollisionWorkbenchModuleController::saveOverridesToProject()
    {
        m_linkModelSetupController->saveOverridesToProject();
    }

    bool CollisionWorkbenchModuleController::selectedRobotHasCollisionOverrides() const
    {
        return m_linkModelSetupController->selectedRobotHasCollisionOverrides();
    }

    std::filesystem::path CollisionWorkbenchModuleController::selectedRobotSourcePath() const
    {
        return m_linkModelSetupController->selectedRobotSourcePath();
    }

    void CollisionWorkbenchModuleController::saveOverridesAsSidecar(
        const std::filesystem::path& sidecarPath,
        const std::string& portableSidecarPath)
    {
        m_linkModelSetupController->saveOverridesAsSidecar(sidecarPath, portableSidecarPath);
    }

    void CollisionWorkbenchModuleController::exportRobotUrdfWithCollision(
        const std::filesystem::path& sourceUrdfPath,
        const std::filesystem::path& outputUrdfPath)
    {
        m_linkModelSetupController->exportRobotUrdfWithCollision(sourceUrdfPath, outputUrdfPath);
    }

    void CollisionWorkbenchModuleController::addMemberToNewSelectionSet(
        const QString& defaultName,
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        m_detectorConfigController->addMemberToNewSelectionSet(defaultName, member);
    }

    void CollisionWorkbenchModuleController::addMemberToExistingSelectionSet(
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        m_detectorConfigController->addMemberToExistingSelectionSet(member);
    }

    void CollisionWorkbenchModuleController::addMemberToDetectorDraftSet(
        const QString& side,
        const QString& displayName,
        const QStringList& robotLinks,
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        m_detectorConfigController->addMemberToDetectorDraftSet(
            side,
            displayName,
            robotLinks,
            member);
    }

}
