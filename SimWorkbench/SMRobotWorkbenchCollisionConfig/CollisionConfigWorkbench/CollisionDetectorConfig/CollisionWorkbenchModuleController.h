#pragma once

#include "CollisionWorkbenchServices.h"
#include "RobotQtViewerEvents.h"

#include <QObject>
#include <QString>
#include <QStringList>

#include <filesystem>
#include <memory>

class CollisionWorkbenchPanel;

namespace simulation_project
{
    struct CollisionDetectorDesc;
    struct CollisionSelectionSetMemberDesc;
}

namespace robot_qt_viewer
{
    class CollisionDocumentEventPublisher;
    class CollisionDetectorConfigModuleController;
    class CollisionLinkModelSetupModuleController;
    class RobotQtViewerDocumentContext;

    class CollisionWorkbenchModuleController : public QObject
    {
        Q_OBJECT

    public:
        CollisionWorkbenchModuleController(
            CollisionWorkbenchPanel& panel,
            RobotQtViewerDocumentContext& context,
            CollisionWorkbenchServices& appServices,
            QObject* parent = nullptr);
        ~CollisionWorkbenchModuleController() override;

        void handleEvent(const RobotQtViewerEvent& event);
        bool isUpdating() const;
        bool isCollisionModelConfigurationActive() const;
        bool canHandoffCollisionModelConfiguration() const;
        QString currentCollisionSelectionSetId() const;
        QString currentCollisionDetectorId() const;
        void refreshInspector(const QString& qualityMessage);
        void refreshCollisionPairList();
        void refreshCollisionDetectorList();
        void refreshCollisionSelectionSetList();
        void refreshCollisionSelectionSetMemberList();
        void refreshCollisionDetectorPropertyEditors();
        void refreshCollisionDetectorDetails();
        void refreshCollisionElementList(const QString& qualityMessage);
        void refreshCollisionModelSummary(const QString& qualityMessage);
        bool syncLegacyRobotObjectPairs();
        void previewCollisionSelectionSetMember();
        void addCollisionSelectionSet();
        void renameSelectedCollisionSelectionSet();
        void removeSelectedCollisionSelectionSet();
        void removeSelectedCollisionSelectionSetMember();
        void handleCollisionDetectorEnabledChanged(const QString& detectorId, bool enabled);
        void handleCollisionDetectorSelectionChanged();
        void handleCollisionDetectorPropertyChanged();
        void previewCollisionDetectorSelection();
        void showSelectedCollisionDetectors();
        void removeSelectedCollisionDetector();
        void addCollisionDetectorFromTaskPanel();
        void editSelectedCollisionDetector();
        void addSceneAllCollisionDetector();
        void showDetectorConfiguration();
        void showCollisionModelConfiguration();
        void cancelCollisionModelConfigurationForHandoff(const QString& sourceId);
        void handleCollisionVariantSelectionChanged();
        void showSelectedCollisionVariantOnly();
        void useSelectedCollisionVariantInActiveDetector();
        void selectCollisionVariantBySourceRole(const QString& source, const QString& role);
        void handleCollisionReplaceOriginalChanged(bool replaceOriginal);
        void addBoxCollisionElement();
        void generateCollisionProxyFromVisual(const QString& proxyType);
        void generateCollisionProxyFromExistingCollision(const QString& proxyType);
        void generateRobotCollisionProxyFromExistingCollision(const QString& proxyType);
        void generateCollisionCoacd();
        void generateMissingCollisionProxiesFromVisual();
        void removeSelectedCollisionElement();
        void saveOverridesToProject();
        bool selectedRobotHasCollisionOverrides() const;
        std::filesystem::path selectedRobotSourcePath() const;
        void saveOverridesAsSidecar(
            const std::filesystem::path& sidecarPath,
            const std::string& portableSidecarPath);
        void exportRobotUrdfWithCollision(
            const std::filesystem::path& sourceUrdfPath,
            const std::filesystem::path& outputUrdfPath);
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

    signals:
        void refreshInspectorRequested();
        void refreshSelectionDependentViewsRequested();
        void saveCollisionOverridesSidecarRequested();
        void exportCollisionUrdfRequested();
        void viewportReloadRequested();
        void statusMessageRequested(const QString& message, int timeoutMs);

    private:
        void autoPairAllRobotObjects();
        void handleLegacyPairEnabledChanged(const QString& robotId, const QString& objectId, bool enabled);

        CollisionWorkbenchPanel& m_panel;
        RobotQtViewerDocumentContext& m_context;
        CollisionWorkbenchServices& m_appServices;
        std::unique_ptr<CollisionDocumentEventPublisher> m_documentEvents;
        std::unique_ptr<CollisionLinkModelSetupModuleController> m_linkModelSetupController;
        std::unique_ptr<CollisionDetectorConfigModuleController> m_detectorConfigController;
        bool m_updating = false;
        bool m_collisionModelConfigurationActive = false;
    };
}
