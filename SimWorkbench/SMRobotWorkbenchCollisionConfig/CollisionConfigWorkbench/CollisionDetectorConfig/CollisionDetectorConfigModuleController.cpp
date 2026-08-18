#include "CollisionDetectorConfigModuleController.h"

#include "CollisionDetectorWorkbenchController.h"
#include "CollisionDetectorWorkbenchDocumentFacade.h"
#include "CollisionDocumentEventPublisher.h"
#include "CollisionWorkbenchServices.h"
#include "CollisionWorkbenchPanel.h"
#include "CollisionLegacyPairDocumentFacade.h"
#include "CollisionLegacyPairWorkbenchController.h"
#include "CollisionSelectionSetDocumentFacade.h"
#include "CollisionSelectionSetViewController.h"
#include "CollisionSelectionSetWorkbenchController.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerViewportServices.h"

#include <QObject>

#include <utility>
#include <vector>

namespace
{
    using CollisionRuntimeDetectorInfo = robot_qt_viewer::CollisionRuntimeDetectorInfo;
}

namespace robot_qt_viewer
{
    CollisionDetectorConfigModuleController::CollisionDetectorConfigModuleController(
        CollisionWorkbenchPanel& panel,
        RobotQtViewerDocumentContext& context,
        CollisionWorkbenchServices& appServices,
        CollisionDocumentEventPublisher& documentEvents,
        Callbacks callbacks,
        bool& updating)
        : m_panel(panel)
        , m_context(context)
        , m_appServices(appServices)
        , m_documentEvents(documentEvents)
        , m_callbacks(std::move(callbacks))
        , m_updating(updating)
        , m_detectorDocument(std::make_unique<CollisionDetectorWorkbenchDocumentFacade>(
              m_appServices.document(),
              m_appServices))
        , m_detectorWorkbench(std::make_unique<CollisionDetectorWorkbenchController>(
              m_panel,
              m_context,
              m_appServices,
              *m_detectorDocument,
              CollisionDetectorWorkbenchController::Callbacks{
                  [this]() {
                      return isUpdating();
                  },
                  [this]() {
                      if(m_callbacks.refreshDetectorList) {
                          m_callbacks.refreshDetectorList();
                      }
                  },
                  [this]() {
                      if(m_callbacks.refreshDetectorProperties) {
                          m_callbacks.refreshDetectorProperties();
                      }
                  },
                  [this]() {
                      if(m_callbacks.refreshDetectorDetails) {
                          m_callbacks.refreshDetectorDetails();
                      }
                  },
                  [this](const QString& qualityMessage) {
                      if(m_callbacks.refreshElementList) {
                          m_callbacks.refreshElementList(qualityMessage);
                      }
                  },
                  [this](const QString& sourceId, const QString& detectorId) {
                      m_documentEvents.publishCollisionChanged(sourceId, detectorId);
                  },
                  [this](const QString& message, int timeoutMs) {
                      showStatus(message, timeoutMs);
                  }}))
        , m_legacyPairDocument(std::make_unique<CollisionLegacyPairDocumentFacade>(
              m_appServices.document(),
              m_appServices))
        , m_legacyPairWorkbench(std::make_unique<CollisionLegacyPairWorkbenchController>(
              m_panel,
              *m_legacyPairDocument,
              CollisionLegacyPairWorkbenchController::Callbacks{
                  [this]() {
                      return isUpdating();
                  },
                  [this]() {
                      if(m_callbacks.viewportReload) {
                          m_callbacks.viewportReload();
                      }
                  },
                  [this](const QString& sourceId) {
                      m_documentEvents.publishCollisionChanged(sourceId);
                  },
                  [this](const QString& message, int timeoutMs) {
                      showStatus(message, timeoutMs);
                  }}))
        , m_selectionSetDocument(std::make_unique<CollisionSelectionSetDocumentFacade>(
              m_appServices.document(),
              m_appServices))
        , m_selectionSetView(std::make_unique<CollisionSelectionSetViewController>(
              *m_panel.selectionSetsWidget(),
              m_context,
              *m_selectionSetDocument,
              CollisionSelectionSetViewController::Callbacks{
                  [this](const QString& message, int timeoutMs) {
                      showStatus(message, timeoutMs);
                  }}))
        , m_selectionSetWorkbench(std::make_unique<CollisionSelectionSetWorkbenchController>(
              m_panel,
              *m_selectionSetDocument,
              CollisionSelectionSetWorkbenchController::Callbacks{
                  [this]() {
                      if(m_callbacks.refreshSelectionSetList) {
                          m_callbacks.refreshSelectionSetList();
                      }
                  },
                  [this](const QString& sourceId) {
                      m_documentEvents.publishCollisionChanged(sourceId);
                  },
                  [this](const QString& message, int timeoutMs) {
                      showStatus(message, timeoutMs);
                  }}))
    {
    }

    CollisionDetectorConfigModuleController::~CollisionDetectorConfigModuleController() = default;

    CollisionDetectorWorkbenchDocumentFacade& CollisionDetectorConfigModuleController::detectorDocument()
    {
        return *m_detectorDocument;
    }

    CollisionLegacyPairWorkbenchController& CollisionDetectorConfigModuleController::legacyPairWorkbench()
    {
        return *m_legacyPairWorkbench;
    }

    CollisionSelectionSetViewController& CollisionDetectorConfigModuleController::selectionSetView()
    {
        return *m_selectionSetView;
    }

    void CollisionDetectorConfigModuleController::connectPanelSignals(QObject& receiver)
    {
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::selectionSetSelectionChanged,
            &receiver, [this]() {
                refreshSelectionSetMemberList();
                refreshDetectorPropertyEditors();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::selectionSetMemberSelectionChanged,
            &receiver, [this]() {
                m_panel.setRemoveSelectionSetMemberEnabled(m_panel.currentSelectionSetMemberIndex() >= 0);
                previewSelectionSetMember();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::addSelectionSetRequested,
            &receiver, [this]() {
                addSelectionSet();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::renameSelectionSetRequested,
            &receiver, [this]() {
                renameSelectedSelectionSet();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::removeSelectionSetRequested,
            &receiver, [this]() {
                removeSelectedSelectionSet();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::removeSelectionSetMemberRequested,
            &receiver, [this]() {
                removeSelectedSelectionSetMember();
            });

        QObject::connect(&m_panel, &CollisionWorkbenchPanel::detectorEnabledChanged,
            &receiver, [this](const QString& detectorId, bool enabled) {
                setDetectorEnabled(detectorId, enabled);
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::detectorSelectionChanged,
            &receiver, [this]() {
                handleDetectorSelectionChanged();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::detectorClicked,
            &receiver, [this](const QString& detectorId) {
                if(detectorId.isEmpty()) {
                    return;
                }

                RobotQtViewerViewportServices* viewportServices = m_context.viewportServices();
                if(viewportServices != nullptr && viewportServices->setActiveCollisionDetector(detectorId)) {
                    m_appServices.setActiveCollisionDetectorContext(detectorId);
                    showStatus(QString("Active collision detector: %1").arg(detectorId), 3000);
                    if(m_callbacks.refreshDetectorDetails) {
                        m_callbacks.refreshDetectorDetails();
                    }
                }
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::detectorPropertyChanged,
            &receiver, [this]() {
                applyDetectorProperties();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::addDetectorRequested,
            &receiver, [this]() {
                addDetectorFromTaskPanel();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::editDetectorRequested,
            &receiver, [this]() {
                editSelectedDetector();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::showSelectedDetectorsRequested,
            &receiver, [this]() {
                showSelectedDetectors();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::removeDetectorRequested,
            &receiver, [this]() {
                removeSelectedDetector();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::bindDetectorDraftSetsRequested,
            &receiver, [this]() {
                m_detectorWorkbench->bindDraftSetsToDetector();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::removeDetectorPairGeneratorsRequested,
            &receiver, [this](const QVector<int>& generatorIndexes) {
                m_detectorWorkbench->removePairGenerators(generatorIndexes);
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::clearDetectorPairScopeRequested,
            &receiver, [this]() {
                m_detectorWorkbench->clearPairScope();
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::detectorPairScopePreviewRequested,
            &receiver, [this](
                const QString& robotAId,
                const QString& linkAName,
                const QString& objectAId,
                const QString& attachmentAId,
                const QString& robotBId,
                const QString& linkBName,
                const QString& objectBId,
                const QString& attachmentBId) {
                m_detectorWorkbench->previewPairScope(
                    robotAId,
                    linkAName,
                    objectAId,
                    attachmentAId,
                    robotBId,
                    linkBName,
                    objectBId,
                    attachmentBId);
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::detectorDraftMemberPreviewRequested,
            &receiver, [this](
                const QString& robotId,
                const QString& linkName,
                const QString& objectId,
                const QString& attachmentId) {
                m_detectorWorkbench->previewDraftMember(robotId, linkName, objectId, attachmentId);
            });

        QObject::connect(&m_panel, &CollisionWorkbenchPanel::legacyPairEnabledChanged,
            &receiver, [this](const QString& robotId, const QString& objectId, bool enabled) {
                setLegacyPairEnabled(robotId, objectId, enabled);
            });
        QObject::connect(&m_panel, &CollisionWorkbenchPanel::autoPairAllRequested,
            &receiver, [this]() {
                autoPairAllRobotObjects();
            });
    }

    QString CollisionDetectorConfigModuleController::currentSelectionSetId() const
    {
        return m_selectionSetView->currentSelectionSetId();
    }

    QString CollisionDetectorConfigModuleController::currentDetectorId() const
    {
        return m_panel.currentDetectorId();
    }

    void CollisionDetectorConfigModuleController::refreshPairList()
    {
        m_updating = true;
        m_legacyPairWorkbench->refreshPairList();
        m_updating = false;
    }

    void CollisionDetectorConfigModuleController::refreshDetectorList()
    {
        const QString previousId = currentDetectorId();
        std::vector<CollisionRuntimeDetectorInfo> detectors;
        if(m_context.viewportServices() != nullptr) {
            detectors = m_context.viewportServices()->collisionRuntimeDetectors();
        }

        m_updating = true;
        m_panel.setDetectors(
            m_detectorDocument->detectorListItems(detectors),
            previousId);

        const QString currentId = currentDetectorId();
        m_panel.setDetectorActionsEnabled(
            m_detectorDocument->hasSceneEntities(),
            !currentId.isEmpty(),
            !currentId.isEmpty() && m_detectorDocument->hasMultipleDetectors());

        m_appServices.setActiveCollisionDetectorContext(currentDetectorId());
        m_updating = false;
        refreshDetectorPropertyEditors();
        if(m_callbacks.refreshDetectorDetails) {
            m_callbacks.refreshDetectorDetails();
        }
    }

    void CollisionDetectorConfigModuleController::refreshSelectionSetList()
    {
        const QString previousId = currentSelectionSetId();
        const bool wasUpdating = m_updating;
        m_updating = true;
        m_selectionSetView->refreshList(previousId);
        refreshDetectorPropertyEditors();
        m_updating = wasUpdating;
    }

    void CollisionDetectorConfigModuleController::refreshSelectionSetMemberList()
    {
        m_selectionSetView->refreshMemberList();
    }

    void CollisionDetectorConfigModuleController::refreshDetectorPropertyEditors()
    {
        const bool wasUpdating = m_updating;
        m_updating = true;
        m_panel.setDetectorProperties(m_detectorDocument->detectorProperties(currentDetectorId()));
        std::vector<CollisionRuntimeDetectorInfo> detectors;
        if(m_context.viewportServices() != nullptr) {
            detectors = m_context.viewportServices()->collisionRuntimeDetectors();
        }
        m_panel.setDetectorPairs(m_detectorDocument->detectorPairs(currentDetectorId(), detectors));
        m_updating = wasUpdating;
    }

    bool CollisionDetectorConfigModuleController::syncLegacyRobotObjectPairs()
    {
        return m_legacyPairWorkbench->syncRobotObjectPairs();
    }

    void CollisionDetectorConfigModuleController::autoPairAllRobotObjects()
    {
        m_legacyPairWorkbench->autoPairAllRobotObjects();
    }

    void CollisionDetectorConfigModuleController::setLegacyPairEnabled(
        const QString& robotId,
        const QString& objectId,
        bool enabled)
    {
        m_legacyPairWorkbench->setPairEnabled(robotId, objectId, enabled);
    }

    void CollisionDetectorConfigModuleController::previewSelectionSetMember()
    {
        m_selectionSetView->previewMember(isUpdating());
    }

    void CollisionDetectorConfigModuleController::addSelectionSet()
    {
        m_selectionSetWorkbench->addSelectionSet();
    }

    void CollisionDetectorConfigModuleController::renameSelectedSelectionSet()
    {
        m_selectionSetWorkbench->renameSelectedSelectionSet();
    }

    void CollisionDetectorConfigModuleController::removeSelectedSelectionSet()
    {
        m_selectionSetWorkbench->removeSelectedSelectionSet();
    }

    void CollisionDetectorConfigModuleController::removeSelectedSelectionSetMember()
    {
        m_selectionSetWorkbench->removeSelectedSelectionSetMember();
    }

    void CollisionDetectorConfigModuleController::setDetectorEnabled(const QString& detectorId, bool enabled)
    {
        m_detectorWorkbench->setDetectorEnabled(detectorId, enabled);
    }

    void CollisionDetectorConfigModuleController::handleDetectorSelectionChanged()
    {
        m_detectorWorkbench->handleSelectionChanged();
    }

    void CollisionDetectorConfigModuleController::applyDetectorProperties()
    {
        m_detectorWorkbench->applyDetectorProperties();
    }

    void CollisionDetectorConfigModuleController::previewDetectorSelection()
    {
        m_detectorWorkbench->previewSelection();
    }

    void CollisionDetectorConfigModuleController::showSelectedDetectors()
    {
        m_detectorWorkbench->showSelectedDetectors();
    }

    void CollisionDetectorConfigModuleController::removeSelectedDetector()
    {
        m_detectorWorkbench->removeSelectedDetector();
    }

    void CollisionDetectorConfigModuleController::addDetectorFromTaskPanel()
    {
        m_detectorWorkbench->addDetectorFromTaskPanel();
    }

    void CollisionDetectorConfigModuleController::editSelectedDetector()
    {
        m_detectorWorkbench->editSelectedDetector();
    }

    void CollisionDetectorConfigModuleController::addSceneAllDetector()
    {
        m_detectorWorkbench->addSceneAllDetector();
    }

    void CollisionDetectorConfigModuleController::addMemberToNewSelectionSet(
        const QString& defaultName,
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        m_selectionSetWorkbench->addMemberToNewSelectionSet(defaultName, member);
    }

    void CollisionDetectorConfigModuleController::addMemberToExistingSelectionSet(
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        m_selectionSetWorkbench->addMemberToExistingSelectionSet(member);
    }

    void CollisionDetectorConfigModuleController::addMemberToDetectorDraftSet(
        const QString& side,
        const QString& displayName,
        const QStringList& robotLinks,
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        m_detectorWorkbench->addSelectionMemberToDraftSet(
            side,
            displayName,
            robotLinks,
            member);
    }

    bool CollisionDetectorConfigModuleController::isUpdating() const
    {
        return m_callbacks.isUpdating ? m_callbacks.isUpdating() : false;
    }

    void CollisionDetectorConfigModuleController::showStatus(const QString& message, int timeoutMs) const
    {
        if(m_callbacks.statusMessage) {
            m_callbacks.statusMessage(message, timeoutMs);
        }
    }
}
