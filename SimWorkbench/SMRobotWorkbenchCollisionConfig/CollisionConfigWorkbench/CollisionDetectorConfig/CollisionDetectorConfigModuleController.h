#pragma once

#include <QString>
#include <QStringList>

#include <functional>
#include <memory>

class CollisionWorkbenchPanel;
class QObject;

namespace simulation_project
{
    struct CollisionSelectionSetMemberDesc;
}

namespace robot_qt_viewer
{
    class CollisionDetectorWorkbenchController;
    class CollisionDetectorWorkbenchDocumentFacade;
    class CollisionDocumentEventPublisher;
    class CollisionWorkbenchServices;
    class CollisionLegacyPairDocumentFacade;
    class CollisionLegacyPairWorkbenchController;
    class CollisionSelectionSetDocumentFacade;
    class CollisionSelectionSetViewController;
    class CollisionSelectionSetWorkbenchController;
    class RobotQtViewerDocumentContext;

    class CollisionDetectorConfigModuleController
    {
    public:
        struct Callbacks
        {
            std::function<bool()> isUpdating;
            std::function<void()> refreshSelectionSetList;
            std::function<void()> refreshDetectorList;
            std::function<void()> refreshDetectorProperties;
            std::function<void()> refreshDetectorDetails;
            std::function<void(const QString&)> refreshElementList;
            std::function<void()> viewportReload;
            std::function<void(const QString&, int)> statusMessage;
        };

        CollisionDetectorConfigModuleController(
            CollisionWorkbenchPanel& panel,
            RobotQtViewerDocumentContext& context,
            CollisionWorkbenchServices& appServices,
            CollisionDocumentEventPublisher& documentEvents,
            Callbacks callbacks,
            bool& updating);
        ~CollisionDetectorConfigModuleController();

        CollisionDetectorWorkbenchDocumentFacade& detectorDocument();
        CollisionLegacyPairWorkbenchController& legacyPairWorkbench();
        CollisionSelectionSetViewController& selectionSetView();

        void connectPanelSignals(QObject& receiver);
        QString currentSelectionSetId() const;
        QString currentDetectorId() const;
        void refreshPairList();
        void refreshDetectorList();
        void refreshSelectionSetList();
        void refreshSelectionSetMemberList();
        void refreshDetectorPropertyEditors();
        bool syncLegacyRobotObjectPairs();
        void autoPairAllRobotObjects();
        void setLegacyPairEnabled(const QString& robotId, const QString& objectId, bool enabled);
        void previewSelectionSetMember();
        void addSelectionSet();
        void renameSelectedSelectionSet();
        void removeSelectedSelectionSet();
        void removeSelectedSelectionSetMember();
        void setDetectorEnabled(const QString& detectorId, bool enabled);
        void handleDetectorSelectionChanged();
        void applyDetectorProperties();
        void previewDetectorSelection();
        void showSelectedDetectors();
        void removeSelectedDetector();
        void addDetectorFromTaskPanel();
        void editSelectedDetector();
        void addSceneAllDetector();
        void addMemberToNewSelectionSet(
            const QString& defaultName,
            const simulation_project::CollisionSelectionSetMemberDesc& member);
        void addMemberToExistingSelectionSet(
            const simulation_project::CollisionSelectionSetMemberDesc& member);
        void addMemberToDetectorDraftSet(
            const QString& side,
            const QString& displayName,
            const QStringList& robotLinks,
            const simulation_project::CollisionSelectionSetMemberDesc& member);

    private:
        bool isUpdating() const;
        void showStatus(const QString& message, int timeoutMs) const;

        CollisionWorkbenchPanel& m_panel;
        RobotQtViewerDocumentContext& m_context;
        CollisionWorkbenchServices& m_appServices;
        CollisionDocumentEventPublisher& m_documentEvents;
        Callbacks m_callbacks;
        bool& m_updating;
        std::unique_ptr<CollisionDetectorWorkbenchDocumentFacade> m_detectorDocument;
        std::unique_ptr<CollisionDetectorWorkbenchController> m_detectorWorkbench;
        std::unique_ptr<CollisionLegacyPairDocumentFacade> m_legacyPairDocument;
        std::unique_ptr<CollisionLegacyPairWorkbenchController> m_legacyPairWorkbench;
        std::unique_ptr<CollisionSelectionSetDocumentFacade> m_selectionSetDocument;
        std::unique_ptr<CollisionSelectionSetViewController> m_selectionSetView;
        std::unique_ptr<CollisionSelectionSetWorkbenchController> m_selectionSetWorkbench;
    };
}
