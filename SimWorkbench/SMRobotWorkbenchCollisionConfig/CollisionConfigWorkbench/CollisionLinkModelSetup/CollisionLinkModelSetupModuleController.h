#pragma once

#include <QString>

#include <filesystem>
#include <functional>
#include <memory>
#include <string>

class CollisionLinkModelsWidget;
class QObject;

namespace robot_qt_viewer
{
    class CollisionExportDocumentFacade;
    class CollisionExportWorkbenchController;
    class CollisionWorkbenchServices;
    class CollisionModelDocumentFacade;
    class CollisionModelWorkbenchController;
    class CollisionQualityMessageState;
    class CollisionRequestDocumentFacade;
    class CollisionRequestWorkbenchController;
    class RobotQtViewerDocumentContext;

    class CollisionLinkModelSetupModuleController
    {
    public:
        struct Callbacks
        {
            std::function<bool()> isUpdating;
            std::function<QString()> activeDetectorId;
            std::function<void(const QString&)> refreshElementList;
            std::function<void(const QString&)> refreshModelSummary;
            std::function<void()> refreshDetectorList;
            std::function<void()> refreshDetectorDetails;
            std::function<void()> refreshDetectorProperties;
            std::function<void(bool, bool)> setLinkPairActionsEnabled;
            std::function<bool(const QString&)> setCurrentDetectorRole;
            std::function<void(const QString&, int)> statusMessage;
            std::function<void()> saveSidecarRequested;
            std::function<void()> exportUrdfRequested;
            std::function<void()> exitCollisionModelConfiguration;
        };

        CollisionLinkModelSetupModuleController(
            CollisionLinkModelsWidget& widget,
            RobotQtViewerDocumentContext& context,
            CollisionWorkbenchServices& appServices,
            Callbacks callbacks);
        ~CollisionLinkModelSetupModuleController();

        CollisionModelWorkbenchController& modelWorkbench();

        void connectPanelSignals(QObject& receiver);
        void refreshElementList(const QString& activeDetectorId, const QString& qualityMessage);
        void refreshModelSummary(const QString& activeDetectorId, const QString& qualityMessage);
        void enterCollisionModelViewportFocus();
        void handleVariantSelectionChanged();
        void showSelectedVariantOnly();
        void setSelectedVariantCurrent();
        void applyCollisionModelConfiguration();
        void cancelCollisionModelConfiguration();
        void cancelCollisionModelConfigurationForHandoff(const QString& sourceId);
        void useSelectedVariantInActiveDetector();
        void selectVariantBySourceRole(const QString& source, const QString& role);
        void setReplaceOriginal(bool replaceOriginal, bool updating);
        void addBoxElement();
        void generateFromVisual(const QString& proxyType);
        void generateFromExistingCollision(const QString& proxyType);
        void generateRobotFromExistingCollision(const QString& proxyType);
        void generateCoacd();
        void generateMissingFromVisual();
        void removeSelectedElement();
        void saveOverridesToProject();
        bool selectedRobotHasCollisionOverrides() const;
        std::filesystem::path selectedRobotSourcePath() const;
        void saveOverridesAsSidecar(
            const std::filesystem::path& sidecarPath,
            const std::string& portableSidecarPath);
        void exportRobotUrdfWithCollision(
            const std::filesystem::path& sourceUrdfPath,
            const std::filesystem::path& outputUrdfPath);
        void setCollisionModelConfigurationActive(bool active);

    private:
        void clearCollisionModelViewportFocus(const QString& sourceId);

        CollisionLinkModelsWidget& m_widget;
        RobotQtViewerDocumentContext& m_context;
        CollisionWorkbenchServices& m_appServices;
        Callbacks m_callbacks;
        std::unique_ptr<CollisionModelDocumentFacade> m_modelDocument;
        std::unique_ptr<CollisionModelWorkbenchController> m_modelWorkbench;
        std::unique_ptr<CollisionExportDocumentFacade> m_exportDocument;
        std::unique_ptr<CollisionExportWorkbenchController> m_exportWorkbench;
        std::unique_ptr<CollisionQualityMessageState> m_qualityMessageState;
        std::unique_ptr<CollisionRequestDocumentFacade> m_requestDocument;
        std::unique_ptr<CollisionRequestWorkbenchController> m_requestWorkbench;
        bool m_collisionModelConfigurationActive = false;
    };
}
