#pragma once

#include <QMainWindow>
#include <QHash>
#include <QString>
#include <QStringList>
#include <QVector>

#include <SimulationProject/ProjectSession.h>

#include <memory>

#include "RobotQtViewerAppController.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerDocumentViewRegistry.h"
#include "RobotQtViewerEventHub.h"
#include "RobotQtViewerLanguage.h"
#include "RobotQtViewerSelectionModel.h"
#include "RobotQtViewerViewportPreviewState.h"
#include "RobotQtViewerWorkbench.h"
#include "RobotQtViewerWorkbenchPackageRegistry.h"
#include "RobotQtViewerWindowConfig.h"
#include "SceneTreeIntentController.h"
#include "SceneExplorerViewModel.h"

#include <filesystem>

class QLabel;
class QLineEdit;
class QAction;
class QDockWidget;
class QEvent;
class QMenu;
class QPushButton;
class QPoint;
class QStackedWidget;
class QTreeWidget;
class QWidget;
class CollisionWorkbenchPanel;
class CollisionRuntimeResultsWidget;
class MotionControlWidget;
class MotionPlanningEditorWidget;
class RobotViewport;
class SceneExplorerTaskWidget;
class SceneExplorerWidget;
class StatusPanelWidget;
class ToolSetupWidget;

namespace robot_qt_viewer
{
    class CollisionWorkbenchModuleController;
    class CoatingAnalysisModuleController;
    class CoatingAnalysisPanel;
    class MotionControlModuleController;
    class MotionPlanningModuleController;
    class RobotQtViewerCollisionWorkbenchServicesAdapter;
    class RobotQtViewerSceneExplorerActionRouter;
    class RobotQtViewerToolbarController;
    class RobotQtViewerToolSetupAppServicesAdapter;
    class RobotQtViewerViewportEventController;
    class RobotQtViewerViewportPresentationController;
    class RobotQtViewerViewportServicesAdapter;
    class SceneExplorerModuleController;
    class ThicknessLegendWidget;
    class ToolSetupModuleController;
}

namespace smrobotgen2
{
namespace sdk
{
    class IRobotLoader;
    class IRobotModel;
    class IRobotSdk;
}
}

class MainWindow : public QMainWindow
{
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;
    bool openProjectPathForProfiling(
        const std::filesystem::path& path,
        int exitDelayMs = -1,
        int repeatCount = 1,
        bool enableCollisionAfterLoad = false,
        const QString& profileGenerateObjectCoacdId = QString());

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    void applyIndustrialStyle();
    void applyTheme(const QString& themeName);
    void applyLanguage(const QString& languageName);
    QString uiText(const QString& key) const;
    void retranslateUi();
    void createActions();
    void createCameraViewOverlay();
    void showCameraViewPalette();
    void updateCameraViewOverlayGeometry();
    void setViewportPresentationMode(bool active);
    void updateViewportPresentationAction();
    void updateThicknessLegendOverlayGeometry();
    void createPanels();
    void applyInitialPanelLayout();
    void enterWorkbench(robot_qt_viewer::RobotQtViewerWorkbenchKind kind, const QString& sourceId);
    void updateWorkbenchActions();
    void setRobotRunDetailsRequested(bool requested);
    void updateRobotRunDetailsDockVisibility();
    void updateTaskPanel();
    void newProject();
    void openProject();
    void saveProject();
    void saveProjectAs();
    void saveProjectAsV3();
    void importRobotPackage();
    void exportSelectedRobotPackage();
    void loadRobot();
    void importRobot();
    void importObject();
    void importPointCloud();
    void deleteSelectedRobot();
    void renamePointCloud(
        const robot_qt_viewer::SceneTreeIntentController::ContextMenuAction& action);
    void handleSceneTreeContextMenuAction(
        const robot_qt_viewer::SceneTreeIntentController::ContextMenuAction& action);
    void addRobotLinksToTree(
        const QString& robotId,
        const QString& robotName,
        const QStringList& links,
        const QStringList& joints,
        const QStringList& movableJoints,
        const QStringList& movableJointTypes);
    void addSceneObjectToTree(const QString& objectId, const QString& objectName);
    void refreshSceneExplorerViewModel();
    bool resolveToolSetupPendingChanges(bool restoreEditorTarget = true);
    bool handleCollisionModelConfigurationNodeActivated(
        const robot_qt_viewer::SceneExplorerNodeRef& node);
    bool selectCollisionModelConfigurationTarget(
        const robot_qt_viewer::SceneExplorerNodeRef& node,
        const QString& sourceId);
    QString collisionModelConfigurationLinkName(
        const robot_qt_viewer::SceneExplorerNodeRef& node) const;
    void rememberCollisionModelConfigurationTarget(
        const robot_qt_viewer::SceneExplorerNodeRef& node);
    bool currentSceneExplorerNodeIsLink() const;
    void focusSceneExplorerMountFrame(
        const QString& robotId,
        const QString& linkName,
        const QString& mountId);
    void handleSceneExplorerNodeActivated(const robot_qt_viewer::SceneExplorerNodeRef& node, int column);
    void selectRobotContext(
        const QString& robotId,
        const QString& preferredLinkName = QString(),
        const QString& preferredMountId = QString());
    void clearInspectorSelectionContext();
    void setActiveCollisionDetectorContext(const QString& detectorId);
    void setMarkedCollisionPairAContext(const QString& robotId, const QString& linkName);
    void refreshSelectedLinkMaterialSummary();
    void connectCollisionWorkbenchPanel();
    void refreshCollisionWorkbench();
    void publishCollisionChanged(
        const QString& sourceId,
        const QString& detectorId = QString());
    bool collisionUiUpdating() const;
    void refreshCollisionPairList();
    void refreshCollisionDetectorList();
    void refreshCollisionDetectorPropertyEditors();
    void refreshCollisionSelectionSetList();
    void refreshCollisionSelectionSetMemberList();
    QString currentCollisionSelectionSetId() const;
    QString currentCollisionDetectorId() const;
    void refreshCollisionDetectorDetails();
    void refreshCollisionElementList();
    void refreshCollisionModelSummary();
    void saveCollisionOverridesToProject();
    void saveCollisionOverridesAsSidecar();
    void exportRobotUrdfWithCollision();
    void updateRobotPanel(const smrobotgen2::sdk::IRobotModel& model);
    void saveViewportImage();
    bool saveProjectToPath(const std::filesystem::path& path, bool saveAsV3 = false);
    bool reloadViewportProject();
    std::string makePortableAssetPath(const std::filesystem::path& assetPath) const;
    QTreeWidget* robotTree() const;

private:
    RobotViewport* m_viewport = nullptr;
    std::unique_ptr<robot_qt_viewer::RobotQtViewerViewportServicesAdapter> m_viewportServices;
    std::unique_ptr<robot_qt_viewer::RobotQtViewerToolSetupAppServicesAdapter> m_toolSetupServices;
    std::unique_ptr<robot_qt_viewer::RobotQtViewerCollisionWorkbenchServicesAdapter> m_collisionWorkbenchServices;
    std::unique_ptr<robot_qt_viewer::RobotQtViewerSceneExplorerActionRouter> m_sceneExplorerActionRouter;
    std::unique_ptr<robot_qt_viewer::RobotQtViewerViewportPresentationController>
        m_viewportPresentationController;
    robot_qt_viewer::RobotQtViewerViewportEventController* m_viewportEventController = nullptr;
    robot_qt_viewer::LanguageKind m_language = robot_qt_viewer::LanguageKind::English;
    QMenu* m_fileMenu = nullptr;
    QMenu* m_viewMenu = nullptr;
    QMenu* m_windowMenu = nullptr;
    QMenu* m_themeMenu = nullptr;
    QMenu* m_languageMenu = nullptr;
    QMenu* m_cameraViewMenu = nullptr;
    QDockWidget* m_sceneExplorerDock = nullptr;
    QDockWidget* m_taskPanelDock = nullptr;
    QDockWidget* m_bottomPanelDock = nullptr;
    robot_qt_viewer::RobotQtViewerToolbarController* m_toolbarController = nullptr;
    QHash<QString, QAction*> m_themeActions;
    QHash<QString, QAction*> m_languageActions;
    QHash<QString, QAction*> m_cameraViewActions;
    QWidget* m_cameraViewOverlay = nullptr;
    robot_qt_viewer::ThicknessLegendWidget* m_thicknessLegendOverlay = nullptr;
    QAction* m_loadRobotAction = nullptr;
    QAction* m_newProjectAction = nullptr;
    QAction* m_openProjectAction = nullptr;
    QAction* m_saveProjectAction = nullptr;
    QAction* m_saveProjectAsAction = nullptr;
    QAction* m_saveProjectAsV3Action = nullptr;
    QAction* m_importRobotPackageAction = nullptr;
    QAction* m_exportRobotPackageAction = nullptr;
    QAction* m_saveCollisionOverridesAction = nullptr;
    QAction* m_saveCollisionSidecarAction = nullptr;
    QAction* m_exportCollisionUrdfAction = nullptr;
    QAction* m_importRobotAction = nullptr;
    QAction* m_importObjectAction = nullptr;
    QAction* m_importPointCloudAction = nullptr;
    QAction* m_deleteRobotAction = nullptr;
    QAction* m_saveImageAction = nullptr;
    QAction* m_resetCameraAction = nullptr;
    QAction* m_viewportPresentationAction = nullptr;
    QAction* m_collisionGeometryAction = nullptr;
    QAction* m_collisionQueriesAction = nullptr;
    QAction* m_robotRunDetailsAction = nullptr;
    QAction* m_browseWorkbenchAction = nullptr;
    QAction* m_motionWorkbenchAction = nullptr;
    QAction* m_toolSetupWorkbenchAction = nullptr;
    QAction* m_collisionWorkbenchAction = nullptr;
    QAction* m_trajectoryPlanningWorkbenchAction = nullptr;
    QAction* m_sprayProcessWorkbenchAction = nullptr;
    QAction* m_coatingAnalysisWorkbenchAction = nullptr;
    QAction* m_digitalTwinWorkbenchAction = nullptr;
    QLabel* m_statusLabel = nullptr;
    SceneExplorerWidget* m_sceneExplorerWidget = nullptr;
    SceneExplorerTaskWidget* m_sceneExplorerTaskWidget = nullptr;
    robot_qt_viewer::SceneExplorerModuleController* m_sceneExplorerController = nullptr;
    StatusPanelWidget* m_statusPanelWidget = nullptr;
    MotionControlWidget* m_motionControlWidget = nullptr;
    CollisionRuntimeResultsWidget* m_robotRunCollisionDetailsWidget = nullptr;
    robot_qt_viewer::MotionControlModuleController* m_motionControlController = nullptr;
    MotionPlanningEditorWidget* m_motionPlanningWidget = nullptr;
    robot_qt_viewer::MotionPlanningModuleController* m_motionPlanningController = nullptr;
    ToolSetupWidget* m_toolSetupWidget = nullptr;
    robot_qt_viewer::ToolSetupModuleController* m_toolSetupController = nullptr;
    CollisionWorkbenchPanel* m_collisionWorkbenchPanel = nullptr;
    robot_qt_viewer::CollisionWorkbenchModuleController* m_collisionWorkbenchController = nullptr;
    robot_qt_viewer::CoatingAnalysisPanel* m_coatingAnalysisPanel = nullptr;
    robot_qt_viewer::CoatingAnalysisModuleController* m_coatingAnalysisController = nullptr;
    QStackedWidget* m_taskPanelStack = nullptr;
    QWidget* m_sceneExplorerTaskPanel = nullptr;
    QWidget* m_motionTaskPanel = nullptr;
    QWidget* m_motionPlanningTaskPanel = nullptr;
    QWidget* m_toolSetupTaskPanel = nullptr;
    QWidget* m_collisionTaskPanel = nullptr;
    QWidget* m_coatingAnalysisTaskPanel = nullptr;
    QString m_collisionLastQualityMessage;
    robot_qt_viewer::SceneExplorerNodeRef m_collisionModelConfigurationNode;
    bool m_hasCollisionModelConfigurationNode = false;
    int m_collisionResultRefreshFrame = 0;
    smrobotgen2::sdk::IRobotSdk* m_sdk = nullptr;
    smrobotgen2::sdk::IRobotLoader* m_robotLoader = nullptr;
    smrobotgen2::sdk::IRobotModel* m_robotModel = nullptr;
    simulation_project::ProjectSession m_projectSession;
    robot_qt_viewer::RobotQtViewerWindowConfig m_windowConfig;
    robot_qt_viewer::RobotQtViewerEventHub m_eventHub;
    robot_qt_viewer::RobotQtViewerDocumentViewRegistry m_documentViewRegistry;
    robot_qt_viewer::RobotQtViewerDocumentController m_documentController;
    robot_qt_viewer::RobotQtViewerSelectionModel m_selectionModel;
    robot_qt_viewer::RobotQtViewerViewportPreviewState m_viewportPreviewState;
    robot_qt_viewer::RobotQtViewerWorkbenchPackageRegistry m_workbenchPackageRegistry;
    robot_qt_viewer::RobotQtViewerWorkbenchManager m_workbenchManager;
    robot_qt_viewer::RobotQtViewerDocumentContext m_documentContext;
    robot_qt_viewer::RobotQtViewerAppController m_appController;
    std::filesystem::path& m_projectPath;
    bool& m_projectDirty;
    bool& m_projectRequiresSaveAs;
    bool m_robotRunDetailsRequested = false;
};
