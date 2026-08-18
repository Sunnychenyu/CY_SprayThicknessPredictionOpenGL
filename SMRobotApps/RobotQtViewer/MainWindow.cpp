#include "MainWindow.h"

#include "CollisionWorkbenchModuleController.h"
#include "CollisionWorkbenchPanel.h"
#include "CollisionRuntimeResultsWidget.h"
#include "CoatingAnalysisModuleController.h"
#include "CoatingAnalysisPanel.h"
#include "CoatingAnalysisInfoPanel.h"
#include "CoatingAnalysisTreePanel.h"
#include "CoatingAnalysisVisibilityBar.h"
#include "MotionControlModuleController.h"
#include "MotionControlWidget.h"
#include "MotionPlanningEditorWidget.h"
#include "MotionPlanningModuleController.h"
#include "RobotQtViewerLanguage.h"
#include "RobotQtViewerSceneExplorerActionRouter.h"
#include "RobotQtViewerTheme.h"
#include "RobotQtViewerToolbarController.h"
#include "RobotQtViewerCollisionWorkbenchServicesAdapter.h"
#include "RobotQtViewerToolSetupAppServicesAdapter.h"
#include "RobotQtViewerViewportEventController.h"
#include "RobotQtViewerViewportServicesAdapter.h"
#include "ProjectAssemblyDialogService.h"
#include "CollisionConfigDialogService.h"
#include "RobotQtWidgetUtils.h"
#include "RobotViewport.h"
#include "SceneExplorerModuleController.h"
#include "SceneCollisionTargetResolver.h"
#include "SceneExplorerTaskWidget.h"
#include "SceneExplorerWidget.h"
#include "SceneSelectionController.h"
#include "SceneTreeIntentController.h"
#include "StatusPanelWidget.h"
#include "ThicknessLegendWidget.h"
#include "ToolSetupModuleController.h"
#include "ToolSetupWidget.h"

#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectIo.h>
#include <SimulationProject/RuntimePaths.h>

#include <RobotSDK/IRobotLoader.h>
#include <RobotSDK/IRobotModel.h>
#include <RobotSDK/IRobotSdk.h>
#include <RobotSDK/RobotSdkApi.h>

#include <CustomLog/CustomLog.h>

#include <QAction>
#include <QAbstractSpinBox>
#include <QActionGroup>
#include <QApplication>
#include <QByteArray>
#include <QCursor>
#include <QDockWidget>
#include <QEvent>
#include <QFileDialog>
#include <QFrame>
#include <QGridLayout>
#include <QHash>
#include <QHBoxLayout>
#include <QImage>
#include <QLabel>
#include <QLayout>
#include <QList>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPoint>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSplitter>
#include <QStackedWidget>
#include <QStatusBar>
#include <QStringList>
#include <QTimer>
#include <QToolButton>
#include <QToolTip>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidgetAction>
#include <QWidget>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cstdlib>
#include <data_path.h>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <system_error>
#include <unordered_set>
#include <vector>

#ifndef ROBOT_QT_VIEWER_DEFAULT_PROJECT_PATH
#define ROBOT_QT_VIEWER_DEFAULT_PROJECT_PATH "projects/420.v3.scene.json"
#endif

namespace
{
    using robot_qt_viewer::configureInspectorButton;
    using robot_qt_viewer::makeHorizontallyCompressible;

    double elapsedMilliseconds(const std::chrono::steady_clock::time_point& start)
    {
        const auto elapsed = std::chrono::steady_clock::now() - start;
        return std::chrono::duration<double, std::milli>(elapsed).count();
    }

    simulation_project::ColorDesc viewportBackgroundForTheme(
        robot_qt_viewer::ThemeKind theme)
    {
        switch(theme) {
        case robot_qt_viewer::ThemeKind::Light:
            return simulation_project::ColorDesc{ 0.91, 0.93, 0.95, 1.0 };
        case robot_qt_viewer::ThemeKind::Dark:
            return simulation_project::ColorDesc{ 0.043, 0.059, 0.078, 1.0 };
        case robot_qt_viewer::ThemeKind::Modern:
        default:
            return simulation_project::ColorDesc{ 0.067, 0.082, 0.102, 1.0 };
        }
    }

    std::string sourceTypeFromPath(const QString& path)
    {
        return path.endsWith(".xml", Qt::CaseInsensitive) ? "simscape" : "urdf";
    }

    std::filesystem::path weakCanonicalPath(const std::filesystem::path& path)
    {
        std::error_code error;
        std::filesystem::path canonical = std::filesystem::weakly_canonical(path, error);
        return error ? path.lexically_normal() : canonical;
    }

    std::string lowercaseAscii(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        return value;
    }

    std::string pathToUtf8(const std::filesystem::path& path)
    {
        return path.generic_u8string();
    }

    std::filesystem::path pathFromUtf8(const std::string& path)
    {
        return std::filesystem::u8path(path);
    }

    ProjectSceneInteractionMode toProjectSceneInteractionMode(
        robot_qt_viewer::RobotQtViewerViewportInteractionMode mode)
    {
        switch(mode) {
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::Browse:
            return ProjectSceneInteractionMode::Browse;
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::SelectRobot:
            return ProjectSceneInteractionMode::SelectRobot;
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::SelectLink:
            return ProjectSceneInteractionMode::SelectLink;
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::SelectMount:
            return ProjectSceneInteractionMode::SelectMount;
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::SelectAttachment:
            return ProjectSceneInteractionMode::SelectAttachment;
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::EditTransformPreview:
            return ProjectSceneInteractionMode::EditTransformPreview;
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::EditCollisionProxy:
            return ProjectSceneInteractionMode::EditCollisionProxy;
        case robot_qt_viewer::RobotQtViewerViewportInteractionMode::SelectCollisionTarget:
            return ProjectSceneInteractionMode::SelectCollisionTarget;
        }
        return ProjectSceneInteractionMode::Browse;
    }

    std::string normalizedCollisionRole(const std::string& role)
    {
        return role.empty() ? std::string("Exact") : role;
    }

    bool pathIsInside(const std::filesystem::path& path, const std::filesystem::path& root)
    {
        const std::string pathText = lowercaseAscii(pathToUtf8(weakCanonicalPath(path)));
        std::string rootText = lowercaseAscii(pathToUtf8(weakCanonicalPath(root)));
        if(rootText.empty()) {
            return false;
        }
        if(rootText.back() != '/') {
            rootText.push_back('/');
        }
        return pathText == rootText.substr(0, rootText.size() - 1) ||
            pathText.rfind(rootText, 0) == 0;
    }

    std::string relativePathString(
        const std::filesystem::path& path,
        const std::filesystem::path& root)
    {
        if(!pathIsInside(path, root)) {
            return {};
        }

        std::filesystem::path relative = weakCanonicalPath(path).lexically_relative(weakCanonicalPath(root));
        if(relative.empty()) {
            return {};
        }
        return pathToUtf8(relative);
    }

    std::string makePortableAssetPathForProject(
        const std::filesystem::path& assetPath,
        const std::filesystem::path& projectPath)
    {
        const std::filesystem::path canonicalAsset = weakCanonicalPath(assetPath);
        const std::filesystem::path dataRoot = weakCanonicalPath(
            simulation_project::RuntimePaths::dataRoot());

        std::string dataRelative = relativePathString(canonicalAsset, dataRoot);
        if(!dataRelative.empty()) {
            return pathToUtf8(std::filesystem::path("data") / pathFromUtf8(dataRelative));
        }

        const char* assetRootEnv = std::getenv("SMROBOT_ASSET_DIR");
        if(assetRootEnv != nullptr) {
            std::string envRelative = relativePathString(canonicalAsset, std::filesystem::path(assetRootEnv));
            if(!envRelative.empty()) {
                return envRelative;
            }
        }

        if(!projectPath.empty()) {
            const std::filesystem::path projectDir = weakCanonicalPath(projectPath.parent_path());
            std::string projectRelative = relativePathString(canonicalAsset, projectDir);
            if(!projectRelative.empty()) {
                return projectRelative;
            }
        }

        return pathToUtf8(canonicalAsset);
    }

    std::filesystem::path resolveStoredAssetPathForSave(
        const std::string& storedPath,
        const std::filesystem::path& currentProjectPath)
    {
        std::filesystem::path candidate = pathFromUtf8(storedPath);
        if(candidate.is_absolute()) {
            return candidate;
        }

        if(!currentProjectPath.empty()) {
            const std::filesystem::path fromCurrentProject = currentProjectPath.parent_path() / candidate;
            if(std::filesystem::exists(fromCurrentProject)) {
                return fromCurrentProject;
            }
        }

        const std::filesystem::path fromAppRoot =
            simulation_project::RuntimePaths::applicationRoot() / candidate;
        if(std::filesystem::exists(fromAppRoot)) {
            return fromAppRoot;
        }

        const std::filesystem::path sourceRoot = simulation_project::RuntimePaths::sourceRoot();
        const std::filesystem::path fromSourceRoot = sourceRoot / candidate;
        if(!sourceRoot.empty() && std::filesystem::exists(fromSourceRoot)) {
            return fromSourceRoot;
        }

        return candidate;
    }

    std::string makePortableStoredAssetPathForSave(
        const std::string& storedPath,
        const std::filesystem::path& currentProjectPath,
        const std::filesystem::path& targetProjectPath)
    {
        const std::filesystem::path assetPath = resolveStoredAssetPathForSave(storedPath, currentProjectPath);
        if(assetPath.is_relative() && !std::filesystem::exists(assetPath)) {
            return storedPath;
        }
        return makePortableAssetPathForProject(assetPath, targetProjectPath);
    }

    QString dialogPathFromFilesystem(const std::filesystem::path& path)
    {
        return QString::fromStdWString(path.wstring());
    }

    QString defaultProjectDialogPath(const std::filesystem::path& currentProjectPath)
    {
        if(!currentProjectPath.empty()) {
            return dialogPathFromFilesystem(currentProjectPath.parent_path());
        }
        return dialogPathFromFilesystem(
            simulation_project::RuntimePaths::configRoot() / "projects");
    }

    bool confirmProjectReplacement(
        QWidget* parent,
        const QString& title,
        const std::function<bool(bool)>& resolvePendingChanges,
        const std::function<void()>& saveProject,
        const std::function<bool()>& isProjectDirty)
    {
        if(resolvePendingChanges && !resolvePendingChanges(false)) {
            return false;
        }
        if(!isProjectDirty || !isProjectDirty()) {
            return true;
        }

        const QMessageBox::StandardButton choice = QMessageBox::warning(
            parent,
            title,
            QStringLiteral("The current project has unsaved changes. Save them before continuing?"),
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
            QMessageBox::Save);
        if(choice == QMessageBox::Cancel) {
            return false;
        }
        if(choice == QMessageBox::Discard) {
            return true;
        }

        if(saveProject) {
            saveProject();
        }
        return isProjectDirty ? !isProjectDirty() : true;
    }

    bool syncLegacyRobotObjectPairsPreservingDirty(
        robot_qt_viewer::CollisionWorkbenchModuleController* controller,
        simulation_project::ProjectSession& session,
        robot_qt_viewer::RobotQtViewerDocumentController& documentController,
        const QString& sourceId)
    {
        if(controller == nullptr) {
            return false;
        }

        const bool previousDirty = session.isDirty();
        const bool changed = controller->syncLegacyRobotObjectPairs();
        if(session.isDirty() != previousDirty) {
            session.setDirty(previousDirty);
            documentController.publishDirtyChanged(sourceId);
        }
        return changed;
    }

    constexpr double kPi = 3.14159265358979323846;

    double toDegrees(double radians)
    {
        return radians * 180.0 / kPi;
    }

    simulation_project::TransformDesc attachmentAssetTcpTransform(
        const simulation_project::AttachmentAssetDesc* asset)
    {
        if(asset == nullptr) {
            return simulation_project::TransformDesc();
        }
        for(const simulation_project::AttachmentFunctionalFrameDesc& frame : asset->functionalFrames) {
            if(frame.primary || frame.frameType == "tcp") {
                return frame.assetMountToFrame;
            }
        }
        return simulation_project::TransformDesc();
    }

    void setAttachmentAssetTcpTransform(
        simulation_project::AttachmentAssetDesc& asset,
        const simulation_project::TransformDesc& transform)
    {
        for(simulation_project::AttachmentFunctionalFrameDesc& frame : asset.functionalFrames) {
            if(frame.primary || frame.frameType == "tcp") {
                frame.assetMountToFrame = transform;
                frame.primary = true;
                return;
            }
        }
        simulation_project::AttachmentFunctionalFrameDesc frame;
        frame.id = asset.id + ".tcp";
        frame.name = "TCP";
        frame.frameType = "tcp";
        frame.assetMountToFrame = transform;
        frame.primary = true;
        asset.functionalFrames.push_back(frame);
    }

    QString collisionSelectionSetMemberText(const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        const std::string attachmentId = !member.attachmentId.empty()
            ? member.attachmentId
            : std::string();
        if(!attachmentId.empty()) {
            return QString("attachment: %1").arg(QString::fromStdString(attachmentId));
        }
        if(!member.objectId.empty()) {
            return QString("object: %1").arg(QString::fromStdString(member.objectId));
        }
        if(!member.robotId.empty() && !member.linkName.empty()) {
            return QString("link: %1.%2")
                .arg(QString::fromStdString(member.robotId), QString::fromStdString(member.linkName));
        }
        if(!member.robotId.empty()) {
            return QString("robot: %1").arg(QString::fromStdString(member.robotId));
        }
        return "invalid member";
    }

    bool sameCollisionSelectionSetMember(
        const simulation_project::CollisionSelectionSetMemberDesc& a,
        const simulation_project::CollisionSelectionSetMemberDesc& b)
    {
        return a.robotId == b.robotId &&
            a.linkName == b.linkName &&
            a.objectId == b.objectId &&
            a.attachmentId == b.attachmentId;
    }

    QString cameraViewIconResourcePath(const QString& id)
    {
        return QStringLiteral(":/RobotQtViewer/icons/ribbon/camera_view_%1.png").arg(id);
    }

    QToolButton* makeCameraViewButton(QAction* action, QWidget* parent)
    {
        auto* button = new QToolButton(parent);
        if(action != nullptr) {
            button->setDefaultAction(action);
        }
        button->setAutoRaise(true);
        button->setToolButtonStyle(Qt::ToolButtonIconOnly);
        button->setIconSize(QSize(34, 34));
        button->setFixedSize(QSize(42, 42));
        return button;
    }

}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , m_windowConfig(robot_qt_viewer::defaultRobotQtViewerWindowConfig())
    , m_eventHub(this)
    , m_documentViewRegistry(m_eventHub, m_windowConfig)
    , m_documentController(m_projectSession, m_eventHub, this)
    , m_selectionModel(m_eventHub)
    , m_viewportPreviewState(m_eventHub)
    , m_workbenchPackageRegistry(robot_qt_viewer::defaultRobotQtViewerWorkbenchPackageRegistry())
    , m_documentContext(
          m_projectSession,
          m_documentController,
          m_selectionModel,
          m_viewportPreviewState,
          m_eventHub)
    , m_appController(m_documentContext)
    , m_projectPath(m_projectSession.path())
    , m_projectDirty(m_projectSession.dirtyFlag())
    , m_projectRequiresSaveAs(m_projectSession.requiresSaveAsFlag())
{
    m_language = robot_qt_viewer::LanguageManager::savedLanguage();
    setWindowTitle(uiText("window.title"));
    applyIndustrialStyle();

    m_sdk = createRobotSdk();
    if(m_sdk != nullptr) {
        m_robotLoader = m_sdk->createRobotLoader();
    }

    m_viewport = new RobotViewport(this);
    m_viewport->setDefaultBackgroundColor(viewportBackgroundForTheme(
        robot_qt_viewer::ThemeManager::savedTheme()));
    m_viewportServices = std::make_unique<robot_qt_viewer::RobotQtViewerViewportServicesAdapter>(*m_viewport);
    m_viewportEventController = new robot_qt_viewer::RobotQtViewerViewportEventController(
        *m_viewportServices,
        m_viewportPreviewState,
        [this](const QString& robotId, const QString& linkName) {
            return m_sceneExplorerController != nullptr &&
                m_sceneExplorerController->linkFrameVisible(robotId, linkName);
        },
        this);
    m_documentViewRegistry.registerModule(QStringLiteral("viewport"), m_viewportEventController,
        [this](const robot_qt_viewer::RobotQtViewerEvent& event) {
            m_viewportEventController->handleEvent(event);
        });
    m_toolSetupServices =
        std::make_unique<robot_qt_viewer::RobotQtViewerToolSetupAppServicesAdapter>(m_appController);
    m_collisionWorkbenchServices =
        std::make_unique<robot_qt_viewer::RobotQtViewerCollisionWorkbenchServicesAdapter>(m_appController);
    m_sceneExplorerActionRouter =
        std::make_unique<robot_qt_viewer::RobotQtViewerSceneExplorerActionRouter>();
    m_sceneExplorerActionRouter->setParentWidget(this);
    m_sceneExplorerActionRouter->setEnterToolSetupWorkbenchCallback([this]() {
        enterWorkbench(robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup, QStringLiteral("configureRobotFlange"));
    });
    m_sceneExplorerActionRouter->setDeleteSelectedEntityCallback([this]() {
        deleteSelectedRobot();
    });
    m_sceneExplorerActionRouter->setReloadViewportCallback([this]() {
        reloadViewportProject();
    });
    m_sceneExplorerActionRouter->setSelectRobotContextCallback(
        [this](const QString& robotId, const QString& linkName) {
            selectRobotContext(robotId, linkName);
        });
    m_sceneExplorerActionRouter->setSelectObjectContextCallback(
        [this](const QString& objectId) {
            selectRobotContext(QString());
            m_appController.setObjectInspectorContext(objectId);
            m_selectionModel.selectSceneObject(objectId, QStringLiteral("sceneExplorerCollisionModel"));
        });
    m_documentContext.setViewportServices(m_viewportServices.get());
    setCentralWidget(m_viewport);
    connect(m_viewport, &RobotViewport::robotLinksAvailable, this, &MainWindow::addRobotLinksToTree);
    connect(m_viewport, &RobotViewport::sceneObjectAvailable, this, &MainWindow::addSceneObjectToTree);
    connect(m_viewport, &RobotViewport::scenePicked, this,
        [this](
            const QString& kind,
            const QString& robotId,
            const QString& linkName,
            const QString& robotMountId,
            const QString& mountedAttachmentId,
            const QString& sceneObjectId) {
            const robot_qt_viewer::SceneExplorerNodeRef node =
                robot_qt_viewer::SceneSelectionController::nodeFromViewportPick(
                    kind,
                    robotId,
                    linkName,
                    robotMountId,
                    mountedAttachmentId,
                    sceneObjectId);
            if(node.kind != robot_qt_viewer::SceneExplorerNodeKind::Unknown) {
                handleSceneExplorerNodeActivated(node, 0);
            }
        });
    connect(m_viewport, &RobotViewport::sceneSelectionCleared, this, [this]() {
        m_selectionModel.clear(QStringLiteral("viewportBlankClick"));
    });
    connect(m_viewport, &RobotViewport::robotStateUpdated, this, [this]() {
        if(m_motionControlController != nullptr) {
            m_motionControlController->handleRobotStateUpdated();
        }
        ++m_collisionResultRefreshFrame;
        if((m_collisionResultRefreshFrame % 6) == 0) {
            refreshCollisionDetectorDetails();
        }
    });

    createActions();
    createPanels();

    const std::filesystem::path defaultProjectPath =
        simulation_project::RuntimePaths::configRoot() / ROBOT_QT_VIEWER_DEFAULT_PROJECT_PATH;
    const robot_qt_viewer::ProjectSessionWorkflowResult startupResult =
        m_appController.loadStartupProject(defaultProjectPath, QStringLiteral("startup"));
    if(startupResult.success) {
        syncLegacyRobotObjectPairsPreservingDirty(
            m_collisionWorkbenchController,
            m_projectSession,
            m_documentController,
            QStringLiteral("startupSyncLegacyPairs"));
        reloadViewportProject();
    } else {
        statusBar()->showMessage(QString("Default project load failed: %1").arg(startupResult.message), 5000);
    }

    const char* version = getSdkVersion();
    m_statusLabel = new QLabel(QString("RobotSDK %1").arg(version ? version : "unknown"), this);
    statusBar()->addPermanentWidget(m_statusLabel);
    statusBar()->showMessage(uiText("status.ready"));
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event)
{
    if(watched == m_viewport &&
        (event->type() == QEvent::Resize || event->type() == QEvent::Show)) {
        updateCameraViewOverlayGeometry();
        updateThicknessLegendOverlayGeometry();
        updateCoatingVisibilityOverlayGeometry();
    }
    return QMainWindow::eventFilter(watched, event);
}

MainWindow::~MainWindow()
{
    if(m_sdk != nullptr) {
        if(m_robotModel != nullptr) {
            m_sdk->destroyRobotModel(m_robotModel);
            m_robotModel = nullptr;
        }

        if(m_robotLoader != nullptr) {
            m_sdk->destroyRobotLoader(m_robotLoader);
            m_robotLoader = nullptr;
        }
    }

    destroyRobotSdk(m_sdk);
    m_sdk = nullptr;
}

void MainWindow::applyIndustrialStyle()
{
    if(qApp != nullptr) {
        const robot_qt_viewer::ThemeKind theme = robot_qt_viewer::ThemeManager::savedTheme();
        robot_qt_viewer::ThemeManager::apply(*qApp, theme);
        robot_qt_viewer::ThemeManager::applyNativeWindowFrame(*this, theme);
    }
}

void MainWindow::applyTheme(const QString& themeName)
{
    if(qApp == nullptr) {
        return;
    }

    const robot_qt_viewer::ThemeKind theme =
        robot_qt_viewer::ThemeManager::themeFromName(themeName);
    robot_qt_viewer::ThemeManager::apply(*qApp, theme);
    robot_qt_viewer::ThemeManager::applyNativeWindowFrame(*this, theme);
    if(m_viewport != nullptr) {
        m_viewport->setDefaultBackgroundColor(viewportBackgroundForTheme(theme));
    }
    robot_qt_viewer::ThemeManager::save(theme);
}

void MainWindow::applyLanguage(const QString& languageName)
{
    m_language = robot_qt_viewer::LanguageManager::languageFromName(languageName);
    robot_qt_viewer::LanguageManager::save(m_language);
    retranslateUi();
}

QString MainWindow::uiText(const QString& key) const
{
    return robot_qt_viewer::LanguageManager::text(m_language, key);
}

void MainWindow::retranslateUi()
{
    setWindowTitle(uiText("window.title"));

    if(m_fileMenu != nullptr) {
        m_fileMenu->setTitle(uiText("menu.file"));
    }
    if(m_viewMenu != nullptr) {
        m_viewMenu->setTitle(uiText("menu.view"));
    }
    if(m_windowMenu != nullptr) {
        m_windowMenu->setTitle(uiText("menu.window"));
    }
    if(m_themeMenu != nullptr) {
        m_themeMenu->setTitle(uiText("menu.theme"));
    }
    if(m_languageMenu != nullptr) {
        m_languageMenu->setTitle(uiText("menu.language"));
    }
    if(m_cameraViewMenu != nullptr) {
        m_cameraViewMenu->setTitle(uiText("menu.cameraViews"));
    }
    if(m_projectionModeMenu != nullptr) {
        m_projectionModeMenu->setTitle(uiText("menu.projectionMode"));
    }
    if(m_bottomPanelDock != nullptr) {
        m_bottomPanelDock->setWindowTitle(uiText("action.robotRunDetails"));
    }
    if(m_toolbarController != nullptr) {
        robot_qt_viewer::RobotQtViewerToolbarTexts texts;
        texts.toolbarTitle = uiText("toolbar.simulation");
        texts.sceneGroup = uiText("toolbar.sceneEdit");
        texts.robotEditGroup = uiText("toolbar.robotEdit");
        texts.viewGroup = uiText("toolbar.view");
        texts.workbenchGroup = uiText("toolbar.modes");
        m_toolbarController->retranslate(texts);
    }

    auto setAction = [this](QAction* action, const QString& text) {
        if(m_toolbarController != nullptr) {
            m_toolbarController->setActionTextAndToolTip(action, text);
            return;
        }
        if(action != nullptr) {
            action->setText(text);
            action->setToolTip(text);
            action->setStatusTip(text);
        }
    };

    for(auto it = m_themeActions.begin(); it != m_themeActions.end(); ++it) {
        it.value()->setText(uiText(QString("theme.%1").arg(it.key())));
    }
    for(auto it = m_languageActions.begin(); it != m_languageActions.end(); ++it) {
        const robot_qt_viewer::LanguageKind actionLanguage =
            robot_qt_viewer::LanguageManager::languageFromName(it.key());
        it.value()->setText(uiText(QString("language.%1").arg(it.key())));
        it.value()->setChecked(actionLanguage == m_language);
    }

    if(m_loadRobotAction != nullptr) {
        setAction(m_loadRobotAction, uiText("action.loadRobot"));
    }
    if(m_newProjectAction != nullptr) {
        setAction(m_newProjectAction, uiText("action.newProject"));
    }
    if(m_openProjectAction != nullptr) {
        setAction(m_openProjectAction, uiText("action.openProject"));
    }
    if(m_saveProjectAction != nullptr) {
        setAction(m_saveProjectAction, uiText("action.saveProject"));
    }
    if(m_saveProjectAsAction != nullptr) {
        setAction(m_saveProjectAsAction, uiText("action.saveProjectAs"));
    }
    if(m_saveProjectAsV3Action != nullptr) {
        setAction(m_saveProjectAsV3Action, uiText("action.saveProjectAsV3"));
    }
    if(m_importRobotPackageAction != nullptr) {
        setAction(m_importRobotPackageAction, uiText("action.importRobotPackage"));
    }
    if(m_exportRobotPackageAction != nullptr) {
        setAction(m_exportRobotPackageAction, uiText("action.exportRobotPackage"));
    }
    if(m_saveCollisionOverridesAction != nullptr) {
        setAction(m_saveCollisionOverridesAction, uiText("action.saveCollisionOverrides"));
    }
    if(m_saveCollisionSidecarAction != nullptr) {
        setAction(m_saveCollisionSidecarAction, uiText("action.saveCollisionSidecar"));
    }
    if(m_exportCollisionUrdfAction != nullptr) {
        setAction(m_exportCollisionUrdfAction, uiText("action.exportCollisionUrdf"));
    }
    if(m_importRobotAction != nullptr) {
        setAction(m_importRobotAction, uiText("action.importRobot"));
    }
    if(m_importObjectAction != nullptr) {
        setAction(m_importObjectAction, uiText("action.importObject"));
    }
    if(m_importPointCloudAction != nullptr) {
        setAction(m_importPointCloudAction, m_language == robot_qt_viewer::LanguageKind::Chinese
            ? QString::fromWCharArray(L"\u5bfc\u5165\u70b9\u4e91")
            : QStringLiteral("Import Point Cloud"));
    }
    if(m_deleteRobotAction != nullptr) {
        setAction(m_deleteRobotAction, uiText("action.deleteSelectedItem"));
    }
    if(m_saveImageAction != nullptr) {
        setAction(m_saveImageAction, uiText("action.saveImage"));
    }
    if(m_resetCameraAction != nullptr) {
        setAction(m_resetCameraAction, uiText("action.viewOrientation"));
    }
    for(auto it = m_cameraViewActions.begin(); it != m_cameraViewActions.end(); ++it) {
        setAction(it.value(), uiText(QString("action.cameraView.%1").arg(it.key())));
    }
    if(m_perspectiveProjectionAction != nullptr) {
        setAction(m_perspectiveProjectionAction, uiText("action.projectionMode.perspective"));
    }
    if(m_orthographicProjectionAction != nullptr) {
        setAction(m_orthographicProjectionAction, uiText("action.projectionMode.orthographic"));
    }
    if(m_collisionGeometryAction != nullptr) {
        setAction(m_collisionGeometryAction, uiText("action.collisionGeometry"));
    }
    if(m_collisionQueriesAction != nullptr) {
        setAction(m_collisionQueriesAction, uiText("action.collisionQueries"));
    }
    if(m_robotRunDetailsAction != nullptr) {
        setAction(m_robotRunDetailsAction, uiText("action.robotRunDetails"));
    }
    if(m_browseWorkbenchAction != nullptr) {
        setAction(m_browseWorkbenchAction, uiText("action.projectAssemblyMode"));
    }
    if(m_motionWorkbenchAction != nullptr) {
        setAction(m_motionWorkbenchAction, uiText("action.robotRunMode"));
    }
    if(m_toolSetupWorkbenchAction != nullptr) {
        setAction(m_toolSetupWorkbenchAction, uiText("action.addLinkMount"));
    }
    if(m_collisionWorkbenchAction != nullptr) {
        setAction(m_collisionWorkbenchAction, uiText("action.collisionConfigMode"));
    }
    if(m_trajectoryPlanningWorkbenchAction != nullptr) {
        setAction(m_trajectoryPlanningWorkbenchAction, uiText("action.trajectoryPlanningMode"));
    }
    if(m_sprayProcessWorkbenchAction != nullptr) {
        setAction(m_sprayProcessWorkbenchAction, uiText("action.sprayProcessMode"));
    }
    if(m_coatingAnalysisWorkbenchAction != nullptr) {
        setAction(m_coatingAnalysisWorkbenchAction, uiText("action.coatingAnalysisMode"));
    }
    if(m_digitalTwinWorkbenchAction != nullptr) {
        setAction(m_digitalTwinWorkbenchAction, uiText("action.digitalTwinMode"));
    }
}

void MainWindow::createActions()
{
    m_fileMenu = menuBar()->addMenu(QString());
    m_viewMenu = menuBar()->addMenu(QString());
    m_windowMenu = menuBar()->addMenu(QString());
    m_themeMenu = m_viewMenu->addMenu(QString());
    m_languageMenu = m_viewMenu->addMenu(QString());

    QActionGroup* themeGroup = new QActionGroup(this);
    themeGroup->setExclusive(true);
    const robot_qt_viewer::ThemeKind savedTheme = robot_qt_viewer::ThemeManager::savedTheme();
    for(const QString& themeName : { QString("Modern"), QString("Dark"), QString("Light") }) {
        QAction* themeAction = m_themeMenu->addAction(QString());
        themeAction->setCheckable(true);
        themeAction->setActionGroup(themeGroup);
        themeAction->setChecked(
            robot_qt_viewer::ThemeManager::themeFromName(themeName) == savedTheme);
        connect(themeAction, &QAction::triggered, this, [this, themeName]() {
            applyTheme(themeName);
        });
        m_themeActions.insert(themeName, themeAction);
    }

    QActionGroup* languageGroup = new QActionGroup(this);
    languageGroup->setExclusive(true);
    for(const QString& languageName : { QString("English"), QString("Chinese") }) {
        QAction* languageAction = m_languageMenu->addAction(QString());
        languageAction->setCheckable(true);
        languageAction->setActionGroup(languageGroup);
        languageAction->setChecked(
            robot_qt_viewer::LanguageManager::languageFromName(languageName) == m_language);
        connect(languageAction, &QAction::triggered, this, [this, languageName]() {
            applyLanguage(languageName);
        });
        m_languageActions.insert(languageName, languageAction);
    }

    m_loadRobotAction = new QAction(this);
    connect(m_loadRobotAction, &QAction::triggered, this, &MainWindow::importRobot);

    m_newProjectAction = new QAction(this);
    connect(m_newProjectAction, &QAction::triggered, this, &MainWindow::newProject);

    m_openProjectAction = new QAction(this);
    connect(m_openProjectAction, &QAction::triggered, this, &MainWindow::openProject);

    m_saveProjectAction = new QAction(this);
    connect(m_saveProjectAction, &QAction::triggered, this, &MainWindow::saveProject);

    m_saveProjectAsAction = new QAction(this);
    connect(m_saveProjectAsAction, &QAction::triggered, this, &MainWindow::saveProjectAs);

    m_saveProjectAsV3Action = new QAction(this);
    connect(m_saveProjectAsV3Action, &QAction::triggered, this, &MainWindow::saveProjectAsV3);

    m_importRobotPackageAction = new QAction(this);
    connect(m_importRobotPackageAction, &QAction::triggered, this, &MainWindow::importRobotPackage);

    m_exportRobotPackageAction = new QAction(this);
    connect(m_exportRobotPackageAction, &QAction::triggered, this, &MainWindow::exportSelectedRobotPackage);

    m_saveCollisionOverridesAction = new QAction(this);
    connect(m_saveCollisionOverridesAction, &QAction::triggered, this, &MainWindow::saveCollisionOverridesToProject);

    m_saveCollisionSidecarAction = new QAction(this);
    connect(m_saveCollisionSidecarAction, &QAction::triggered, this, &MainWindow::saveCollisionOverridesAsSidecar);

    m_exportCollisionUrdfAction = new QAction(this);
    connect(m_exportCollisionUrdfAction, &QAction::triggered, this, &MainWindow::exportRobotUrdfWithCollision);

    m_importRobotAction = new QAction(this);
    connect(m_importRobotAction, &QAction::triggered, this, &MainWindow::importRobot);

    m_importObjectAction = new QAction(this);
    connect(m_importObjectAction, &QAction::triggered, this, &MainWindow::importObject);

    m_importPointCloudAction = new QAction(this);
    connect(m_importPointCloudAction, &QAction::triggered, this, &MainWindow::importPointCloud);

    m_deleteRobotAction = new QAction(this);
    connect(m_deleteRobotAction, &QAction::triggered, this, &MainWindow::deleteSelectedRobot);

    m_saveImageAction = new QAction(this);
    connect(m_saveImageAction, &QAction::triggered, this, [this]() {
        saveViewportImage();
    });

    m_resetCameraAction = new QAction(this);
    connect(m_resetCameraAction, &QAction::triggered, this, &MainWindow::showCameraViewPalette);

    m_cameraViewMenu = new QMenu(this);
    auto addCameraViewAction = [this](const QString& id, ProjectSceneCameraView view) {
        QAction* action = m_cameraViewMenu->addAction(QString());
        action->setIcon(QIcon(cameraViewIconResourcePath(id)));
        connect(action, &QAction::triggered, this, [this, view]() {
            m_viewport->setCameraView(view);
        });
        m_cameraViewActions.insert(id, action);
    };
    addCameraViewAction(QStringLiteral("home"), ProjectSceneCameraView::Home);
    addCameraViewAction(QStringLiteral("isometric"), ProjectSceneCameraView::Isometric);
    m_cameraViewMenu->addSeparator();
    addCameraViewAction(QStringLiteral("front"), ProjectSceneCameraView::Front);
    addCameraViewAction(QStringLiteral("back"), ProjectSceneCameraView::Back);
    addCameraViewAction(QStringLiteral("left"), ProjectSceneCameraView::Left);
    addCameraViewAction(QStringLiteral("right"), ProjectSceneCameraView::Right);
    addCameraViewAction(QStringLiteral("top"), ProjectSceneCameraView::Top);
    addCameraViewAction(QStringLiteral("bottom"), ProjectSceneCameraView::Bottom);

    m_projectionModeMenu = new QMenu(this);
    auto* projectionGroup = new QActionGroup(this);
    projectionGroup->setExclusive(true);
    m_perspectiveProjectionAction = new QAction(this);
    m_perspectiveProjectionAction->setCheckable(true);
    m_perspectiveProjectionAction->setActionGroup(projectionGroup);
    m_perspectiveProjectionAction->setChecked(true);
    connect(m_perspectiveProjectionAction, &QAction::triggered, this, [this]() {
        if(m_viewport != nullptr) {
            m_viewport->setProjectionMode(ProjectSceneProjectionMode::Perspective);
        }
    });

    m_orthographicProjectionAction = new QAction(this);
    m_orthographicProjectionAction->setCheckable(true);
    m_orthographicProjectionAction->setActionGroup(projectionGroup);
    connect(m_orthographicProjectionAction, &QAction::triggered, this, [this]() {
        if(m_viewport != nullptr) {
            m_viewport->setProjectionMode(ProjectSceneProjectionMode::Orthographic);
        }
    });
    m_projectionModeMenu->addAction(m_perspectiveProjectionAction);
    m_projectionModeMenu->addAction(m_orthographicProjectionAction);

    m_collisionGeometryAction = new QAction(this);
    m_collisionGeometryAction->setCheckable(true);
    m_collisionGeometryAction->setChecked(false);
    connect(m_collisionGeometryAction, &QAction::toggled, this, [this](bool visible) {
        m_appController.setCollisionGeometryVisible(visible, QStringLiteral("collisionGeometryVisibility"));
        m_viewport->setCollisionGeometryVisible(visible);
    });

    m_collisionQueriesAction = new QAction(this);
    m_collisionQueriesAction->setCheckable(true);
    m_collisionQueriesAction->setChecked(false);
    connect(m_collisionQueriesAction, &QAction::toggled, this, [this](bool enabled) {
        m_viewport->setCollisionQueriesEnabled(enabled);
        if(m_statusLabel != nullptr) {
            m_statusLabel->setText(enabled
                ? QStringLiteral("Collision detection enabled")
                : QStringLiteral("Collision detection paused"));
        }
    });

    m_robotRunDetailsAction = new QAction(this);
    m_robotRunDetailsAction->setCheckable(true);
    m_robotRunDetailsAction->setChecked(false);
    connect(m_robotRunDetailsAction, &QAction::toggled, this, [this](bool checked) {
        setRobotRunDetailsRequested(checked);
    });

    auto* workbenchGroup = new QActionGroup(this);
    workbenchGroup->setExclusive(true);

    m_browseWorkbenchAction = new QAction(this);
    m_browseWorkbenchAction->setCheckable(true);
    m_browseWorkbenchAction->setActionGroup(workbenchGroup);
    m_browseWorkbenchAction->setChecked(true);
    connect(m_browseWorkbenchAction, &QAction::triggered, this, [this]() {
        enterWorkbench(robot_qt_viewer::RobotQtViewerWorkbenchKind::Browse, QStringLiteral("browseAction"));
    });

    m_motionWorkbenchAction = new QAction(this);
    m_motionWorkbenchAction->setCheckable(true);
    m_motionWorkbenchAction->setActionGroup(workbenchGroup);
    connect(m_motionWorkbenchAction, &QAction::triggered, this, [this]() {
        enterWorkbench(robot_qt_viewer::RobotQtViewerWorkbenchKind::Motion, QStringLiteral("motionAction"));
    });

    m_toolSetupWorkbenchAction = new QAction(this);
    connect(m_toolSetupWorkbenchAction, &QAction::triggered, this, [this]() {
        if(!currentSceneExplorerNodeIsLink()) {
            return;
        }
        if(!resolveToolSetupPendingChanges()) {
            updateWorkbenchActions();
            return;
        }
        if(m_workbenchManager.activeWorkbench() != robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup) {
            enterWorkbench(robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup, QStringLiteral("addLinkMountAction"));
        }
        if(m_toolSetupController != nullptr) {
            m_toolSetupController->createRobotMountForSelectedLink();
        }
    });

    m_collisionWorkbenchAction = new QAction(this);
    m_collisionWorkbenchAction->setCheckable(true);
    m_collisionWorkbenchAction->setActionGroup(workbenchGroup);
    connect(m_collisionWorkbenchAction, &QAction::triggered, this, [this]() {
        enterWorkbench(robot_qt_viewer::RobotQtViewerWorkbenchKind::Collision, QStringLiteral("collisionAction"));
    });

    m_trajectoryPlanningWorkbenchAction = new QAction(this);
    m_trajectoryPlanningWorkbenchAction->setCheckable(true);
    m_trajectoryPlanningWorkbenchAction->setActionGroup(workbenchGroup);
    connect(m_trajectoryPlanningWorkbenchAction, &QAction::triggered, this, [this]() {
        enterWorkbench(
            robot_qt_viewer::RobotQtViewerWorkbenchKind::TrajectoryPlanning,
            QStringLiteral("motionPlanningAction"));
    });

    m_sprayProcessWorkbenchAction = new QAction(this);
    m_sprayProcessWorkbenchAction->setCheckable(true);
    m_sprayProcessWorkbenchAction->setActionGroup(workbenchGroup);
    connect(m_sprayProcessWorkbenchAction, &QAction::triggered, this, [this]() {
        enterWorkbench(
            robot_qt_viewer::RobotQtViewerWorkbenchKind::SprayProcess,
            QStringLiteral("sprayProcessAction"));
    });

    m_coatingAnalysisWorkbenchAction = new QAction(this);
    m_coatingAnalysisWorkbenchAction->setCheckable(true);
    m_coatingAnalysisWorkbenchAction->setActionGroup(workbenchGroup);
    connect(m_coatingAnalysisWorkbenchAction, &QAction::triggered, this, [this]() {
        enterWorkbench(
            robot_qt_viewer::RobotQtViewerWorkbenchKind::CoatingAnalysis,
            QStringLiteral("coatingAnalysisAction"));
    });

    m_digitalTwinWorkbenchAction = new QAction(this);
    m_digitalTwinWorkbenchAction->setCheckable(true);
    m_digitalTwinWorkbenchAction->setActionGroup(workbenchGroup);
    connect(m_digitalTwinWorkbenchAction, &QAction::triggered, this, [this]() {
        enterWorkbench(
            robot_qt_viewer::RobotQtViewerWorkbenchKind::DigitalTwin,
            QStringLiteral("digitalTwinAction"));
    });

    m_fileMenu->addAction(m_newProjectAction);
    m_fileMenu->addAction(m_openProjectAction);
    m_fileMenu->addAction(m_saveProjectAction);
    m_fileMenu->addAction(m_saveProjectAsAction);
    m_fileMenu->addAction(m_saveProjectAsV3Action);
    m_fileMenu->addSeparator();
    m_fileMenu->addAction(m_importRobotPackageAction);
    m_fileMenu->addAction(m_exportRobotPackageAction);
    m_fileMenu->addSeparator();
    m_fileMenu->addAction(m_saveCollisionOverridesAction);
    m_fileMenu->addAction(m_saveCollisionSidecarAction);
    m_fileMenu->addAction(m_exportCollisionUrdfAction);
    m_fileMenu->addSeparator();
    m_fileMenu->addAction(m_importRobotAction);
    m_fileMenu->addAction(m_importObjectAction);
    m_fileMenu->addAction(m_importPointCloudAction);
    m_fileMenu->addAction(m_deleteRobotAction);
    m_fileMenu->addSeparator();
    m_fileMenu->addAction(m_loadRobotAction);
    m_fileMenu->addAction(m_saveImageAction);
    m_viewMenu->addAction(m_resetCameraAction);
    m_viewMenu->addMenu(m_cameraViewMenu);
    m_viewMenu->addMenu(m_projectionModeMenu);
    m_viewMenu->addAction(m_collisionGeometryAction);
    m_viewMenu->addAction(m_collisionQueriesAction);

    m_toolbarController = new robot_qt_viewer::RobotQtViewerToolbarController(*this, this);
    robot_qt_viewer::RobotQtViewerToolbarActions toolbarActions;
    toolbarActions.newProject = m_newProjectAction;
    toolbarActions.openProject = m_openProjectAction;
    toolbarActions.saveProject = m_saveProjectAction;
    toolbarActions.saveProjectAs = m_saveProjectAsAction;
    toolbarActions.saveCollisionOverrides = m_saveCollisionOverridesAction;
    toolbarActions.importRobot = m_importRobotAction;
    toolbarActions.importObject = m_importObjectAction;
    toolbarActions.deleteSelectedItem = m_deleteRobotAction;
    toolbarActions.saveImage = m_saveImageAction;
    toolbarActions.resetCamera = m_resetCameraAction;
    toolbarActions.collisionGeometry = m_collisionGeometryAction;
    toolbarActions.collisionQueries = m_collisionQueriesAction;
    toolbarActions.browseWorkbench = m_browseWorkbenchAction;
    toolbarActions.motionWorkbench = m_motionWorkbenchAction;
    toolbarActions.toolSetupWorkbench = m_toolSetupWorkbenchAction;
    toolbarActions.collisionWorkbench = m_collisionWorkbenchAction;
    toolbarActions.trajectoryPlanningWorkbench = m_trajectoryPlanningWorkbenchAction;
    toolbarActions.sprayProcessWorkbench = m_sprayProcessWorkbenchAction;
    toolbarActions.coatingAnalysisWorkbench = m_coatingAnalysisWorkbenchAction;
    toolbarActions.digitalTwinWorkbench = m_digitalTwinWorkbenchAction;
    m_toolbarController->build(toolbarActions);
    createCameraViewOverlay();

    retranslateUi();
    updateWorkbenchActions();
}

void MainWindow::showCameraViewPalette()
{
    QMenu palette(this);
    palette.setObjectName(QStringLiteral("RobotQtViewerCameraViewPalette"));
    palette.setStyleSheet(QStringLiteral(
        "QMenu#RobotQtViewerCameraViewPalette { padding: 8px; }"
        "QToolButton { min-width: 58px; max-width: 58px; min-height: 58px; max-height: 58px; "
        "border: 1px solid transparent; border-radius: 4px; padding: 4px; }"
        "QToolButton:hover { background: rgba(70, 138, 180, 45); border-color: rgba(70, 138, 180, 130); }"
        "QToolButton:pressed { background: rgba(70, 138, 180, 90); }"));

    auto* container = new QWidget(&palette);
    auto* grid = new QGridLayout(container);
    grid->setContentsMargins(4, 4, 4, 4);
    grid->setHorizontalSpacing(4);
    grid->setVerticalSpacing(4);

    auto addButton = [&](int row, int column, QAction* action) {
        auto* button = new QToolButton(container);
        if(action != nullptr) {
            button->setDefaultAction(action);
        }
        button->setAutoRaise(true);
        button->setToolButtonStyle(Qt::ToolButtonIconOnly);
        button->setIconSize(QSize(42, 42));
        connect(button, &QToolButton::clicked, &palette, &QMenu::close);
        grid->addWidget(button, row, column);
    };

    addButton(0, 0, m_cameraViewActions.value(QStringLiteral("home")));
    addButton(0, 1, m_cameraViewActions.value(QStringLiteral("top")));
    addButton(0, 2, m_cameraViewActions.value(QStringLiteral("isometric")));
    addButton(1, 0, m_cameraViewActions.value(QStringLiteral("left")));
    addButton(1, 1, m_cameraViewActions.value(QStringLiteral("front")));
    addButton(1, 2, m_cameraViewActions.value(QStringLiteral("right")));
    addButton(2, 0, m_cameraViewActions.value(QStringLiteral("back")));
    addButton(2, 1, m_cameraViewActions.value(QStringLiteral("bottom")));

    auto* widgetAction = new QWidgetAction(&palette);
    widgetAction->setDefaultWidget(container);
    palette.addAction(widgetAction);
    palette.exec(QCursor::pos());
}

void MainWindow::createCameraViewOverlay()
{
    if(m_viewport == nullptr || m_cameraViewOverlay != nullptr) {
        return;
    }

    auto* overlay = new QFrame(m_viewport);
    overlay->setObjectName(QStringLiteral("RobotQtViewerCameraViewOverlay"));
    overlay->setAttribute(Qt::WA_StyledBackground, true);
    overlay->setStyleSheet(QStringLiteral(
        "QFrame#RobotQtViewerCameraViewOverlay {"
        "background: rgba(248, 250, 252, 218);"
        "border: 1px solid rgba(110, 122, 132, 160);"
        "border-radius: 4px;"
        "}"
        "QToolButton { border: 1px solid transparent; border-radius: 3px; padding: 2px; }"
        "QToolButton:hover { background: rgba(70, 138, 180, 45); border-color: rgba(70, 138, 180, 130); }"
        "QToolButton:pressed { background: rgba(70, 138, 180, 90); }"));

    auto* overlayLayout = new QHBoxLayout(overlay);
    overlayLayout->setContentsMargins(4, 4, 4, 4);
    overlayLayout->setSpacing(0);
    overlayLayout->addWidget(makeCameraViewButton(m_resetCameraAction, overlay));

    m_cameraViewOverlay = overlay;
    m_viewport->installEventFilter(this);
    updateCameraViewOverlayGeometry();
    m_cameraViewOverlay->show();
}

void MainWindow::updateCameraViewOverlayGeometry()
{
    if(m_viewport == nullptr || m_cameraViewOverlay == nullptr) {
        return;
    }

    m_cameraViewOverlay->adjustSize();
    const int margin = 14;
    const QSize size = m_cameraViewOverlay->sizeHint();
    const int x = std::max(margin, m_viewport->width() - size.width() - margin);
    m_cameraViewOverlay->move(x, margin);
    m_cameraViewOverlay->raise();
}

void MainWindow::updateThicknessLegendOverlayGeometry()
{
    if(m_viewport == nullptr || m_thicknessLegendOverlay == nullptr) {
        return;
    }

    const int margin = 16;
    const int desiredHeight = std::max(320, std::min(520, m_viewport->height() * 3 / 5));
    const int availableHeight = std::max(1, m_viewport->height() - margin * 2);
    m_thicknessLegendOverlay->resize(
        m_thicknessLegendOverlay->width(),
        std::min(desiredHeight, availableHeight));
    const int viewportX = std::max(0,
        m_viewport->width() - m_thicknessLegendOverlay->width() - margin);
    const int viewportY = std::max(0, (m_viewport->height() - m_thicknessLegendOverlay->height()) / 2);
    const QPoint overlayPosition = m_viewport->mapTo(this, QPoint(viewportX, viewportY));
    m_thicknessLegendOverlay->move(overlayPosition);
    m_thicknessLegendOverlay->raise();
}

void MainWindow::enterWorkbench(
    robot_qt_viewer::RobotQtViewerWorkbenchKind kind,
    const QString& sourceId)
{
    const robot_qt_viewer::RobotQtViewerWorkbenchKind previousKind =
        m_workbenchManager.activeWorkbench();
    if(!m_workbenchPackageRegistry.hasMode(kind)) {
        statusBar()->showMessage(
            QString("Workbench package is not available: %1")
                .arg(robot_qt_viewer::robotQtViewerWorkbenchName(kind)),
            3000);
        updateWorkbenchActions();
        return;
    }

    if(m_workbenchManager.activeWorkbench() == robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup &&
        kind != robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup &&
        !resolveToolSetupPendingChanges()) {
        updateWorkbenchActions();
        return;
    }

    if(!m_workbenchManager.enterWorkbench(kind, sourceId)) {
        statusBar()->showMessage("Finish or cancel the current task first.", 3000);
        updateWorkbenchActions();
        return;
    }

    if(m_coatingAnalysisController != nullptr && previousKind != kind) {
        if(previousKind == robot_qt_viewer::RobotQtViewerWorkbenchKind::CoatingAnalysis) {
            m_coatingAnalysisController->deactivate();
        }
        if(kind == robot_qt_viewer::RobotQtViewerWorkbenchKind::CoatingAnalysis) {
            m_coatingAnalysisController->activate();
        }
    }

    if(previousKind != kind && m_collisionWorkbenchController != nullptr &&
        (previousKind == robot_qt_viewer::RobotQtViewerWorkbenchKind::Collision ||
            kind == robot_qt_viewer::RobotQtViewerWorkbenchKind::Collision)) {
        m_collisionWorkbenchController->showDetectorConfiguration();
    }

    robot_qt_viewer::RobotQtViewerEvent taskEvent;
    taskEvent.kind = robot_qt_viewer::RobotQtViewerEventKind::TaskStateChanged;
    taskEvent.sourceId = sourceId;
    m_eventHub.publish(taskEvent);
    m_viewportPreviewState.clearTaskPreview(sourceId);

    updateWorkbenchActions();
    if(m_sceneExplorerController != nullptr) {
        m_sceneExplorerController->setWorkbenchDescriptor(m_workbenchManager.activeDescriptor());
    }
    if(m_viewport != nullptr) {
        m_viewport->setInteractionMode(toProjectSceneInteractionMode(m_workbenchManager.viewportMode()));
        if(previousKind != kind &&
            kind == robot_qt_viewer::RobotQtViewerWorkbenchKind::Browse) {
            m_viewport->focusFullScene(0.3);
        }
    }
    updateTaskPanel();
    updateRobotRunDetailsDockVisibility();
    updateCoatingAnalysisVisibility();
    robot_qt_viewer::RobotQtViewerEvent event;
    event.kind = robot_qt_viewer::RobotQtViewerEventKind::StatusMessageRequested;
    event.sourceId = sourceId;
    event.message = QString("Workbench: %1").arg(robot_qt_viewer::robotQtViewerWorkbenchName(kind));
    m_eventHub.publish(event);
}

void MainWindow::updateWorkbenchActions()
{
    const robot_qt_viewer::RobotQtViewerWorkbenchKind kind = m_workbenchManager.activeWorkbench();
    if(m_browseWorkbenchAction != nullptr) {
        m_browseWorkbenchAction->setChecked(kind == robot_qt_viewer::RobotQtViewerWorkbenchKind::Browse);
    }
    if(m_motionWorkbenchAction != nullptr) {
        m_motionWorkbenchAction->setEnabled(
            m_workbenchPackageRegistry.hasMode(robot_qt_viewer::RobotQtViewerWorkbenchKind::Motion));
        m_motionWorkbenchAction->setChecked(kind == robot_qt_viewer::RobotQtViewerWorkbenchKind::Motion);
    }
    if(m_toolSetupWorkbenchAction != nullptr) {
        m_toolSetupWorkbenchAction->setEnabled(
            currentSceneExplorerNodeIsLink() &&
            m_workbenchPackageRegistry.hasMode(robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup));
    }
    if(m_collisionWorkbenchAction != nullptr) {
        m_collisionWorkbenchAction->setEnabled(
            m_workbenchPackageRegistry.hasMode(robot_qt_viewer::RobotQtViewerWorkbenchKind::Collision));
        m_collisionWorkbenchAction->setChecked(kind == robot_qt_viewer::RobotQtViewerWorkbenchKind::Collision);
    }
    if(m_trajectoryPlanningWorkbenchAction != nullptr) {
        m_trajectoryPlanningWorkbenchAction->setEnabled(
            m_workbenchPackageRegistry.hasMode(
                robot_qt_viewer::RobotQtViewerWorkbenchKind::TrajectoryPlanning));
        m_trajectoryPlanningWorkbenchAction->setChecked(
            kind == robot_qt_viewer::RobotQtViewerWorkbenchKind::TrajectoryPlanning);
    }
    if(m_sprayProcessWorkbenchAction != nullptr) {
        m_sprayProcessWorkbenchAction->setEnabled(
            m_workbenchPackageRegistry.hasMode(robot_qt_viewer::RobotQtViewerWorkbenchKind::SprayProcess));
        m_sprayProcessWorkbenchAction->setChecked(
            kind == robot_qt_viewer::RobotQtViewerWorkbenchKind::SprayProcess);
    }
    if(m_coatingAnalysisWorkbenchAction != nullptr) {
        m_coatingAnalysisWorkbenchAction->setEnabled(
            m_workbenchPackageRegistry.hasMode(robot_qt_viewer::RobotQtViewerWorkbenchKind::CoatingAnalysis));
        m_coatingAnalysisWorkbenchAction->setChecked(
            kind == robot_qt_viewer::RobotQtViewerWorkbenchKind::CoatingAnalysis);
    }
    if(m_digitalTwinWorkbenchAction != nullptr) {
        m_digitalTwinWorkbenchAction->setEnabled(
            m_workbenchPackageRegistry.hasMode(robot_qt_viewer::RobotQtViewerWorkbenchKind::DigitalTwin));
        m_digitalTwinWorkbenchAction->setChecked(
            kind == robot_qt_viewer::RobotQtViewerWorkbenchKind::DigitalTwin);
    }
}

void MainWindow::setRobotRunDetailsRequested(bool requested)
{
    m_robotRunDetailsRequested = requested;
    if(m_robotRunDetailsAction != nullptr && m_robotRunDetailsAction->isChecked() != requested) {
        QSignalBlocker blocker(m_robotRunDetailsAction);
        m_robotRunDetailsAction->setChecked(requested);
    }
    updateRobotRunDetailsDockVisibility();
}

void MainWindow::updateRobotRunDetailsDockVisibility()
{
    if(m_bottomPanelDock == nullptr) {
        return;
    }

    const bool inRobotRunMode =
        m_workbenchManager.activeWorkbench() == robot_qt_viewer::RobotQtViewerWorkbenchKind::Motion;
    const bool visible = m_robotRunDetailsRequested && inRobotRunMode;
    m_bottomPanelDock->setVisible(visible);
    if(m_robotRunDetailsAction != nullptr) {
        QSignalBlocker blocker(m_robotRunDetailsAction);
        m_robotRunDetailsAction->setEnabled(inRobotRunMode);
        m_robotRunDetailsAction->setChecked(visible);
    }
}

void MainWindow::updateCoatingAnalysisVisibility()
{
    const bool coatingActive =
        m_workbenchManager.activeWorkbench()
        == robot_qt_viewer::RobotQtViewerWorkbenchKind::CoatingAnalysis;
    if(m_sceneExplorerDock != nullptr) {
        m_sceneExplorerDock->setVisible(!coatingActive);
    }
    if(m_coatingAnalysisTreeDock != nullptr) {
        m_coatingAnalysisTreeDock->setVisible(coatingActive);
    }
    if(m_coatingAnalysisVisibilityBar != nullptr) {
        m_coatingAnalysisVisibilityBar->setVisible(coatingActive);
        if(coatingActive) {
            updateCoatingVisibilityOverlayGeometry();
        }
    }
}

void MainWindow::updateCoatingVisibilityOverlayGeometry()
{
    if(m_viewport == nullptr || m_coatingAnalysisVisibilityBar == nullptr) {
        return;
    }

    m_coatingAnalysisVisibilityBar->adjustSize();
    const int margin = 10;
    const QSize size = m_coatingAnalysisVisibilityBar->sizeHint();
    const int x = std::max(margin, (m_viewport->width() - size.width()) / 2);
    const int y = std::max(margin, m_viewport->height() - size.height() - margin);
    m_coatingAnalysisVisibilityBar->move(x, y);
    m_coatingAnalysisVisibilityBar->raise();
}

void MainWindow::updateTaskPanel()
{
    if(m_taskPanelStack == nullptr) {
        return;
    }

    const robot_qt_viewer::RobotQtViewerWorkbenchDescriptor& descriptor =
        m_workbenchManager.activeDescriptor();
    QString panelTitle = descriptor.rightPanelTitle;
    switch(descriptor.rightPanel) {
    case robot_qt_viewer::RobotQtViewerRightPanelKind::SceneSelection:
        if(m_sceneExplorerController != nullptr &&
            m_sceneExplorerController->currentNode().kind != robot_qt_viewer::SceneExplorerNodeKind::Unknown &&
            m_sceneExplorerTaskPanel != nullptr) {
            if(m_sceneExplorerController->currentNode().kind == robot_qt_viewer::SceneExplorerNodeKind::ObjectFrame) {
                panelTitle = QStringLiteral("Object Frame Editor");
            } else if(m_sceneExplorerController->currentNode().kind ==
                robot_qt_viewer::SceneExplorerNodeKind::RobotMount) {
                panelTitle = QStringLiteral("Mount Frame");
            }
            m_taskPanelStack->setCurrentWidget(m_sceneExplorerTaskPanel);
        } else if(m_statusPanelWidget != nullptr) {
            m_taskPanelStack->setCurrentWidget(m_statusPanelWidget);
        }
        break;
    case robot_qt_viewer::RobotQtViewerRightPanelKind::Motion:
        if(m_motionTaskPanel != nullptr) {
            m_taskPanelStack->setCurrentWidget(m_motionTaskPanel);
        }
        break;
    case robot_qt_viewer::RobotQtViewerRightPanelKind::MotionPlanning:
        if(m_motionPlanningTaskPanel != nullptr) {
            m_taskPanelStack->setCurrentWidget(m_motionPlanningTaskPanel);
        }
        break;
    case robot_qt_viewer::RobotQtViewerRightPanelKind::ProjectAssembly:
        if(m_toolSetupTaskPanel != nullptr) {
            m_taskPanelStack->setCurrentWidget(m_toolSetupTaskPanel);
        }
        break;
    case robot_qt_viewer::RobotQtViewerRightPanelKind::CollisionConfig:
        if(m_collisionTaskPanel != nullptr) {
            m_taskPanelStack->setCurrentWidget(m_collisionTaskPanel);
        }
        break;
    case robot_qt_viewer::RobotQtViewerRightPanelKind::CoatingAnalysis:
        if(m_coatingAnalysisTaskPanel != nullptr) {
            m_taskPanelStack->setCurrentWidget(m_coatingAnalysisTaskPanel);
        }
        break;
    case robot_qt_viewer::RobotQtViewerRightPanelKind::Status:
        if(m_statusPanelWidget != nullptr) {
            m_taskPanelStack->setCurrentWidget(m_statusPanelWidget);
        }
        break;
    }
    if(m_taskPanelDock != nullptr) {
        m_taskPanelDock->setWindowTitle(panelTitle);
    }
}

void MainWindow::newProject()
{
    if(!confirmProjectReplacement(
        this,
        QStringLiteral("New project"),
        [this](bool restoreEditorTarget) {
            return resolveToolSetupPendingChanges(restoreEditorTarget);
        },
        [this]() {
            saveProject();
        },
        [this]() {
            return m_projectDirty;
        })) {
        statusBar()->showMessage(QStringLiteral("Project replacement canceled."), 3000);
        return;
    }

    const robot_qt_viewer::ProjectSessionWorkflowResult result =
        m_appController.resetProject(QStringLiteral("newProject"));
    if(result.shouldReloadViewport) {
        reloadViewportProject();
    }
    refreshCollisionPairList();
    statusBar()->showMessage(result.message, 3000);
}

void MainWindow::openProject()
{
    const QString fileName = QFileDialog::getOpenFileName(
        this,
        "Open project",
        defaultProjectDialogPath(m_projectPath),
        "System Project (*.sys.json);;Scene Project (*.scene.v3.json *.scene.json);;JSON Files (*.json);;All Files (*.*)");

    if(fileName.isEmpty()) {
        return;
    }

    if(!confirmProjectReplacement(
        this,
        QStringLiteral("Open project"),
        [this](bool restoreEditorTarget) {
            return resolveToolSetupPendingChanges(restoreEditorTarget);
        },
        [this]() {
            saveProject();
        },
        [this]() {
            return m_projectDirty;
        })) {
        statusBar()->showMessage(QStringLiteral("Open project canceled."), 3000);
        return;
    }

    const std::filesystem::path path = fileName.toStdWString();
    const robot_qt_viewer::ProjectSessionWorkflowResult result =
        m_appController.loadProject(path, QStringLiteral("openProject"));
    if(!result.success) {
        statusBar()->showMessage(result.message, 5000);
        return;
    }

    if(m_collisionWorkbenchController != nullptr) {
        syncLegacyRobotObjectPairsPreservingDirty(
            m_collisionWorkbenchController,
            m_projectSession,
            m_documentController,
            QStringLiteral("openProjectSyncLegacyPairs"));
    }
    if(result.shouldReloadViewport) {
        reloadViewportProject();
    }
    statusBar()->showMessage(result.message, 5000);
}

bool MainWindow::openProjectPathForProfiling(
    const std::filesystem::path& path,
    int exitDelayMs,
    int repeatCount,
    bool enableCollisionAfterLoad,
    const QString& profileGenerateObjectCoacdId)
{
    const int remainingRepeats = repeatCount > 1 ? repeatCount : 1;
    std::cout << "\n"
        << "+------------------------------------------------------------------------------+\n"
        << "| RobotQtViewer Project Load Profile                                           |\n"
        << "+------------------------------------------------------------------------------+\n"
        << "| project: " << path.generic_u8string() << "\n"
        << "| repeatRemaining: " << remainingRepeats << "\n"
        << "+------------------------------------------------------------------------------+\n";

    const robot_qt_viewer::ProjectSessionWorkflowResult result =
        m_appController.loadProject(path, QStringLiteral("profileProject"));
    if(!result.success) {
        std::cout << "| result : FAILED - " << result.message.toStdString() << "\n"
            << "+------------------------------------------------------------------------------+\n";
        statusBar()->showMessage(result.message, 5000);
        return false;
    }

    if(m_collisionWorkbenchController != nullptr) {
        syncLegacyRobotObjectPairsPreservingDirty(
            m_collisionWorkbenchController,
            m_projectSession,
            m_documentController,
            QStringLiteral("profileProjectSyncLegacyPairs"));
    }
    if(result.shouldReloadViewport) {
        reloadViewportProject();
    }
    if(!profileGenerateObjectCoacdId.isEmpty()) {
        const auto coacdStart = std::chrono::steady_clock::now();
        const QString guiAttachmentPrefix = QStringLiteral("gui-attachment:");
        const bool useGuiAttachmentPath =
            profileGenerateObjectCoacdId.startsWith(guiAttachmentPrefix);
        bool generated = false;
        std::size_t generatedPartCount = 0;
        QString profileTarget = profileGenerateObjectCoacdId;
        if(useGuiAttachmentPath) {
            profileTarget = profileGenerateObjectCoacdId.mid(guiAttachmentPrefix.size());
            if(m_toolSetupController != nullptr) {
                m_toolSetupController->selectToolAttachmentById(profileTarget.toStdString());
            }
            if(m_collisionWorkbenchServices != nullptr) {
                std::cout << "| Profile GUI selection        | attachment="
                    << m_collisionWorkbenchServices->selectedToolAttachmentId().toStdString()
                    << " robot=" << m_collisionWorkbenchServices->selectedRobotId().toStdString()
                    << " link=" << m_collisionWorkbenchServices->selectedLinkName().toStdString()
                    << "\n";
            }
            if(m_collisionWorkbenchController != nullptr) {
                m_collisionWorkbenchController->showCollisionModelConfiguration();
                m_collisionWorkbenchController->generateCollisionCoacd();
            }
            const QString statusMessage = statusBar()->currentMessage();
            generated = statusMessage.startsWith(QStringLiteral("Generated "));
            bool generatedPartCountOk = false;
            const int parsedGeneratedPartCount =
                statusMessage.section(QLatin1Char(' '), 1, 1).toInt(&generatedPartCountOk);
            if(generatedPartCountOk && parsedGeneratedPartCount >= 0) {
                generatedPartCount = static_cast<std::size_t>(parsedGeneratedPartCount);
            }
            std::cout << "| Profile GUI COACD status     | "
                << statusMessage.toStdString() << "\n";
        } else {
            std::vector<simulation_project::ObjectCollisionElementOverrideDesc> elements;
            generated =
                m_viewportServices != nullptr &&
                m_viewportServices->generateObjectCollisionCoacdFromVisual(
                    profileTarget,
                    elements);
            generatedPartCount = elements.size();
        }
        const auto coacdMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - coacdStart).count();
        std::cout << "| Profile object COACD         | "
            << std::right << std::setw(10) << coacdMs
            << " ms | object=" << profileTarget.toStdString()
            << " ok=" << (generated ? "true" : "false")
            << " parts=" << generatedPartCount
            << "\n";
    }
    if(enableCollisionAfterLoad) {
        const auto collisionStart = std::chrono::steady_clock::now();
        const bool changed = m_viewport->setCollisionQueriesEnabled(true);
        const auto collisionMs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - collisionStart).count();
        std::cout << "| Profile enable collision       | "
            << std::right << std::setw(10) << collisionMs
            << " ms | changed=" << (changed ? "true" : "false") << "\n";
        if(m_collisionQueriesAction != nullptr) {
            QSignalBlocker blocker(m_collisionQueriesAction);
            m_collisionQueriesAction->setChecked(true);
        }
    }
    statusBar()->showMessage(result.message, 5000);

    if(remainingRepeats > 1) {
        QTimer::singleShot(
            0,
            this,
            [this, path, exitDelayMs, remainingRepeats, enableCollisionAfterLoad, profileGenerateObjectCoacdId]() {
            openProjectPathForProfiling(
                path,
                exitDelayMs,
                remainingRepeats - 1,
                enableCollisionAfterLoad,
                profileGenerateObjectCoacdId);
        });
    } else if(exitDelayMs >= 0) {
        QTimer::singleShot(exitDelayMs, qApp, &QCoreApplication::quit);
    }
    return true;
}

void MainWindow::saveProject()
{
    if(m_projectRequiresSaveAs || m_projectPath.empty()) {
        saveProjectAs();
        return;
    }
    saveProjectToPath(m_projectPath);
}

void MainWindow::saveProjectAs()
{
    const QString fileName = QFileDialog::getSaveFileName(
        this,
        "Save project",
        m_projectPath.empty() ? defaultProjectDialogPath(m_projectPath) : QString::fromStdWString(m_projectPath.wstring()),
        "System Project (*.sys.json);;Scene Project (*.scene.v3.json *.scene.json);;JSON Files (*.json);;All Files (*.*)");

    if(fileName.isEmpty()) {
        return;
    }

    saveProjectToPath(std::filesystem::path(fileName.toStdWString()));
}

void MainWindow::saveProjectAsV3()
{
    const QString fileName = QFileDialog::getSaveFileName(
        this,
        "Save project as system",
        m_projectPath.empty() ? defaultProjectDialogPath(m_projectPath) : QString::fromStdWString(m_projectPath.wstring()),
        "System Project (*.sys.json);;v3 Scene Project (*.scene.v3.json);;JSON Files (*.json);;All Files (*.*)");

    if(fileName.isEmpty()) {
        return;
    }

    saveProjectToPath(std::filesystem::path(fileName.toStdWString()), true);
}

void MainWindow::importRobotPackage()
{
    const QString fileName = robot_qt_viewer::ProjectAssemblyDialogService::selectRobotPackageForImport(
        this,
        defaultProjectDialogPath(m_projectPath));

    if(fileName.isEmpty()) {
        return;
    }

    std::unordered_set<std::string> existingRobotIds;
    for(const simulation_project::RobotDesc& robot : m_appController.document().robots) {
        existingRobotIds.insert(robot.id);
    }

    const std::filesystem::path packagePath(fileName.toStdWString());
    const robot_qt_viewer::ProjectMutationResult mutationResult = m_documentController.mutateProject(
        QStringLiteral("importRobotPackage"),
        robot_qt_viewer::ProjectDirtyPolicy::UserEdit,
        [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& errorMessage) {
            if(!simulation_project::importRobotPackageDocument(packagePath, service.document(), &errorMessage)) {
                return false;
            }
            changed = true;
            return true;
        });
    if(!mutationResult.success) {
        statusBar()->showMessage(QString("Import robot package failed: %1").arg(mutationResult.message), 8000);
        return;
    }

    QString importedRobotId;
    for(const simulation_project::RobotDesc& robot : m_appController.document().robots) {
        if(existingRobotIds.find(robot.id) == existingRobotIds.end()) {
            importedRobotId = QString::fromStdString(robot.id);
            break;
        }
    }

    reloadViewportProject();
    if(!importedRobotId.isEmpty()) {
        selectRobotContext(importedRobotId);
    }
    statusBar()->showMessage(QString("Imported robot package: %1").arg(fileName), 5000);
}

void MainWindow::exportSelectedRobotPackage()
{
    const QString robotId = m_appController.selectedRobotId();
    if(robotId.isEmpty()) {
        statusBar()->showMessage("Select a robot before exporting a robot package.", 4000);
        return;
    }

    std::filesystem::path suggestedPath;
    if(!m_projectPath.empty()) {
        suggestedPath = m_projectPath.parent_path() / (robotId.toStdString() + ".rbt.json");
    }

    const QString fileName = robot_qt_viewer::ProjectAssemblyDialogService::selectRobotPackageForExport(
        this,
        suggestedPath.empty() ? defaultProjectDialogPath(m_projectPath) : QString::fromStdWString(suggestedPath.wstring()));

    if(fileName.isEmpty()) {
        return;
    }

    std::string errorMessage;
    if(!simulation_project::saveRobotPackageDocument(
           std::filesystem::path(fileName.toStdWString()),
           m_appController.document(),
           robotId.toStdString(),
           &errorMessage)) {
        statusBar()->showMessage(QString("Export robot package failed: %1").arg(QString::fromStdString(errorMessage)), 8000);
        return;
    }

    statusBar()->showMessage(QString("Exported robot package: %1").arg(fileName), 5000);
}

bool MainWindow::saveProjectToPath(const std::filesystem::path& path, bool saveAsV3)
{
    const robot_qt_viewer::ProjectSessionWorkflowResult result =
        m_appController.saveProject(path, saveAsV3, QStringLiteral("saveProject"));
    if(!result.success) {
        statusBar()->showMessage(result.message, 5000);
        return false;
    }
    statusBar()->showMessage(result.message, 3000);
    return true;
}

void MainWindow::importRobot()
{
    const QString fileName = robot_qt_viewer::ProjectAssemblyDialogService::selectRobotForImport(this);

    if(fileName.isEmpty()) {
        return;
    }

    const std::filesystem::path path = fileName.toStdWString();
    const robot_qt_viewer::SceneEntityImportResult importResult =
        m_appController.sceneEntityWorkflow().importRobotFromPath(path, sourceTypeFromPath(fileName));
    if(!importResult.success) {
        statusBar()->showMessage(importResult.message, 8000);
        return;
    }
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->syncLegacyRobotObjectPairs();
    }
    QApplication::setOverrideCursor(Qt::WaitCursor);
    statusBar()->showMessage(QString("Importing %1...").arg(fileName));
    QApplication::processEvents();
    bool loaded = false;
    QString importError;
    try {
        loaded = reloadViewportProject();
    } catch(const std::exception& e) {
        importError = QString::fromLocal8Bit(e.what());
        statusBar()->showMessage(QString("Import failed: %1").arg(importError), 8000);
        LOG_ERROR("rs2026") << "Import robot failed: " << e.what();
    } catch(...) {
        importError = "unknown error";
        statusBar()->showMessage("Import failed: unknown error", 8000);
        LOG_ERROR("rs2026") << "Import robot failed: unknown error";
    }
    QApplication::restoreOverrideCursor();
    if(!loaded) {
        if(importError.isEmpty() && m_viewport != nullptr && !m_viewport->lastError().isEmpty()) {
            importError = m_viewport->lastError();
        }
        m_appController.sceneEntityWorkflow().restoreImportState(importResult);
        if(m_collisionWorkbenchController != nullptr) {
            m_collisionWorkbenchController->syncLegacyRobotObjectPairs();
        }
        reloadViewportProject();
        const QString error = !importError.isEmpty()
            ? importError
            : QString("Failed to rebuild viewport scene.");
        statusBar()->showMessage(QString("Import failed: %1").arg(error), 8000);
        return;
    }
    m_documentController.publishDocumentChanged(QStringLiteral("importRobot"), false);
    statusBar()->showMessage(QString("Imported %1").arg(fileName), 5000);
}

void MainWindow::importObject()
{
    const QString fileName = robot_qt_viewer::ProjectAssemblyDialogService::selectObjectForImport(this);

    if(fileName.isEmpty()) {
        return;
    }

    const std::filesystem::path path = fileName.toStdWString();
    const robot_qt_viewer::SceneEntityImportResult importResult =
        m_appController.sceneEntityWorkflow().importSceneObjectFromPath(path, "workpiece");
    if(!importResult.success) {
        statusBar()->showMessage(importResult.message, 8000);
        return;
    }
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->syncLegacyRobotObjectPairs();
    }
    QApplication::setOverrideCursor(Qt::WaitCursor);
    statusBar()->showMessage(QString("Importing object %1...").arg(fileName));
    QApplication::processEvents();
    bool loaded = false;
    QString importError;
    try {
        loaded = reloadViewportProject();
    } catch(const std::exception& e) {
        importError = QString::fromLocal8Bit(e.what());
        statusBar()->showMessage(QString("Import object failed: %1").arg(importError), 8000);
        LOG_ERROR("rs2026") << "Import object failed: " << e.what();
    } catch(...) {
        importError = "unknown error";
        statusBar()->showMessage("Import object failed: unknown error", 8000);
        LOG_ERROR("rs2026") << "Import object failed: unknown error";
    }
    QApplication::restoreOverrideCursor();
    if(!loaded) {
        if(importError.isEmpty() && m_viewport != nullptr && !m_viewport->lastError().isEmpty()) {
            importError = m_viewport->lastError();
        }
        m_appController.sceneEntityWorkflow().restoreImportState(importResult);
        if(m_collisionWorkbenchController != nullptr) {
            m_collisionWorkbenchController->syncLegacyRobotObjectPairs();
        }
        reloadViewportProject();
        const QString error = !importError.isEmpty()
            ? importError
            : QString("Failed to rebuild viewport scene.");
        statusBar()->showMessage(QString("Import object failed: %1").arg(error), 8000);
        return;
    }
    m_documentController.publishDocumentChanged(QStringLiteral("importObject"), false);
    statusBar()->showMessage(QString("Imported object %1 from %2").arg(
        importResult.entityId,
        importResult.storedPath), 5000);
}

void MainWindow::importPointCloud()
{
    const QString fileName = robot_qt_viewer::ProjectAssemblyDialogService::selectPointCloudForImport(this);

    if(fileName.isEmpty()) {
        return;
    }

    const std::filesystem::path path = fileName.toStdWString();
    const robot_qt_viewer::SceneEntityImportResult importResult =
        m_appController.sceneEntityWorkflow().importPointCloudFromPath(path, "pcd");
    if(!importResult.success) {
        statusBar()->showMessage(importResult.message, 8000);
        return;
    }
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->syncLegacyRobotObjectPairs();
    }
    QApplication::setOverrideCursor(Qt::WaitCursor);
    statusBar()->showMessage(QString("Importing point cloud %1...").arg(fileName));
    QApplication::processEvents();
    bool loaded = false;
    QString importError;
    try {
        loaded = reloadViewportProject();
    } catch(const std::exception& e) {
        importError = QString::fromLocal8Bit(e.what());
        statusBar()->showMessage(QString("Import point cloud failed: %1").arg(importError), 8000);
        LOG_ERROR("rs2026") << "Import point cloud failed: " << e.what();
    } catch(...) {
        importError = "unknown error";
        statusBar()->showMessage("Import point cloud failed: unknown error", 8000);
        LOG_ERROR("rs2026") << "Import point cloud failed: unknown error";
    }
    QApplication::restoreOverrideCursor();
    if(!loaded) {
        if(importError.isEmpty() && m_viewport != nullptr && !m_viewport->lastError().isEmpty()) {
            importError = m_viewport->lastError();
        }
        m_appController.sceneEntityWorkflow().restoreImportState(importResult);
        if(m_collisionWorkbenchController != nullptr) {
            m_collisionWorkbenchController->syncLegacyRobotObjectPairs();
        }
        reloadViewportProject();
        const QString error = !importError.isEmpty()
            ? importError
            : QString("Failed to rebuild viewport scene.");
        statusBar()->showMessage(QString("Import point cloud failed: %1").arg(error), 8000);
        return;
    }
    m_documentController.publishDocumentChanged(QStringLiteral("importPointCloud"), false);
    statusBar()->showMessage(QString("Imported point cloud %1 from %2").arg(
        importResult.entityId,
        importResult.storedPath), 5000);
}

void MainWindow::deleteSelectedRobot()
{
    if(m_sceneExplorerController == nullptr) {
        statusBar()->showMessage("No robot selected.", 3000);
        return;
    }

    const robot_qt_viewer::SceneExplorerNodeRef node = m_sceneExplorerController->currentNode();
    if(node.id.isEmpty()) {
        statusBar()->showMessage("No scene item selected.", 3000);
        return;
    }

    robot_qt_viewer::SceneEntityKind entityKind = robot_qt_viewer::SceneEntityKind::Robot;
    if(node.kind == robot_qt_viewer::SceneExplorerNodeKind::Robot) {
        entityKind = robot_qt_viewer::SceneEntityKind::Robot;
    } else if(node.kind == robot_qt_viewer::SceneExplorerNodeKind::Object) {
        entityKind = robot_qt_viewer::SceneEntityKind::Object;
    } else if(node.kind == robot_qt_viewer::SceneExplorerNodeKind::PointCloud) {
        entityKind = robot_qt_viewer::SceneEntityKind::PointCloud;
    } else {
        statusBar()->showMessage("Select a robot, object, or point cloud to delete.", 3000);
        return;
    }

    if(!robot_qt_viewer::ProjectAssemblyDialogService::confirmDelete(this, node.kind, node.id)) {
        return;
    }

    const robot_qt_viewer::SceneEntityDeleteResult deleteResult =
        m_appController.sceneEntityWorkflow().deleteEntity(entityKind, node.id);
    if(!deleteResult.success) {
        statusBar()->showMessage(deleteResult.message, 3000);
        return;
    }

    if(entityKind == robot_qt_viewer::SceneEntityKind::Robot) {
        reloadViewportProject();
    } else if(m_viewportServices != nullptr) {
        if(m_viewportServices->removeSceneObject(node.id)) {
            m_viewportServices->rebuildCollisionDetectorsFromDocument(m_appController.document());
        }
    }
    statusBar()->showMessage(deleteResult.message, 4000);
}

void MainWindow::renamePointCloud(
    const robot_qt_viewer::SceneTreeIntentController::ContextMenuAction& action)
{
    if(action.node.id.isEmpty()) {
        statusBar()->showMessage("No point cloud selected.", 3000);
        return;
    }

    bool ok = false;
    const QString currentName = action.displayName.isEmpty()
        ? action.node.name
        : action.displayName;
    const QString newName = robot_qt_viewer::ProjectAssemblyDialogService::requestPointCloudName(
        this,
        currentName,
        &ok);
    if(!ok) {
        return;
    }

    const robot_qt_viewer::SceneEntityMutationResult result =
        m_appController.sceneEntityWorkflow().renamePointCloud(action.node.id, newName);
    if(!result.success) {
        statusBar()->showMessage(result.message, 5000);
        return;
    }

    statusBar()->showMessage(result.message, 3000);
}

void MainWindow::handleSceneTreeContextMenuAction(
    const robot_qt_viewer::SceneTreeIntentController::ContextMenuAction& action)
{
    if(m_sceneExplorerController != nullptr &&
        !m_sceneExplorerController->resolvePendingTransformPreviewIfTargetChanges(action.node, this)) {
        refreshSceneExplorerViewModel();
        return;
    }

    const bool sameEditedMount =
        action.node.kind == robot_qt_viewer::SceneExplorerNodeKind::RobotMount &&
        m_toolSetupWidget != nullptr &&
        action.node.id == m_toolSetupWidget->currentMountId();
    const bool switchingFromFrameEditor =
        m_workbenchManager.activeWorkbench() == robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup &&
        !sameEditedMount;
    if(switchingFromFrameEditor && !resolveToolSetupPendingChanges(false)) {
        refreshSceneExplorerViewModel();
        return;
    }
    if(switchingFromFrameEditor && action.node.kind != robot_qt_viewer::SceneExplorerNodeKind::RobotMount) {
        enterWorkbench(robot_qt_viewer::RobotQtViewerWorkbenchKind::Browse, QStringLiteral("sceneExplorerSelection"));
    }

    if(action.kind == robot_qt_viewer::SceneTreeIntentController::ContextMenuActionKind::RenamePointCloud) {
        renamePointCloud(action);
        return;
    }
    if(action.kind == robot_qt_viewer::SceneTreeIntentController::ContextMenuActionKind::ShowLinkFrame) {
        const bool visible =
            m_sceneExplorerController != nullptr
                ? m_sceneExplorerController->toggleLinkFrameVisible(action.node.id, action.node.linkName)
                : false;
        selectRobotContext(action.node.id, action.node.linkName);
        robot_qt_viewer::RobotQtViewerViewportPreviewPayload preview;
        preview.setRobotMountFrameVisibility = true;
        preview.selectedLinkFrameVisible = visible;
        preview.mountFrameVisible = false;
        m_viewportPreviewState.mutate(preview, QStringLiteral("sceneExplorerShowLinkFrame"));
        statusBar()->showMessage(
            QString("%1 link frame: %2.%3")
                .arg(visible ? QStringLiteral("Showing") : QStringLiteral("Hiding"),
                    action.node.id,
                    action.node.linkName),
            3000);
        return;
    }
    if(m_sceneExplorerActionRouter == nullptr) {
        return;
    }
    m_sceneExplorerActionRouter->handleAction(
        action,
        [this](const QString& message, int timeoutMs) {
            statusBar()->showMessage(message, timeoutMs);
        });
    if(action.kind ==
            robot_qt_viewer::SceneTreeIntentController::ContextMenuActionKind::ConfigureCollisionModel &&
        robot_qt_viewer::sceneExplorerNodeKindCanConfigureCollisionModel(action.node.kind)) {
        rememberCollisionModelConfigurationTarget(action.node);
    }
}

void MainWindow::createPanels()
{
    QDockWidget* robotDock = new QDockWidget("Scene Explorer", this);
    m_sceneExplorerDock = robotDock;
    m_sceneExplorerWidget = new SceneExplorerWidget(robotDock);
    m_sceneExplorerController = new robot_qt_viewer::SceneExplorerModuleController(
        *m_sceneExplorerWidget,
        m_documentContext,
        this);
    m_sceneExplorerController->setSceneEntityWorkflow(&m_appController.sceneEntityWorkflow());
    m_sceneExplorerController->setViewportServices(m_viewportServices.get());
    connect(m_sceneExplorerController, &robot_qt_viewer::SceneExplorerModuleController::nodeActivated,
        this, &MainWindow::handleSceneExplorerNodeActivated);
    connect(m_sceneExplorerWidget, &SceneExplorerWidget::nodeDoubleActivated,
        this, [this](const robot_qt_viewer::SceneExplorerNodeRef& node, int) {
            if(m_sceneExplorerController == nullptr) {
                return;
            }
            if(node.kind == robot_qt_viewer::SceneExplorerNodeKind::RobotMount) {
                robot_qt_viewer::SceneTreeIntentController::ContextMenuAction action;
                action.kind =
                    robot_qt_viewer::SceneTreeIntentController::ContextMenuActionKind::ConfigureRobotFlange;
                action.node = node;
                handleSceneTreeContextMenuAction(action);
                return;
            }
            if(node.kind != robot_qt_viewer::SceneExplorerNodeKind::Object &&
                node.kind != robot_qt_viewer::SceneExplorerNodeKind::ObjectFrame) {
                return;
            }
            if(m_workbenchManager.activeWorkbench() == robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup &&
                !resolveToolSetupPendingChanges(false)) {
                refreshSceneExplorerViewModel();
                return;
            }
            if(m_workbenchManager.activeWorkbench() == robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup) {
                enterWorkbench(
                    robot_qt_viewer::RobotQtViewerWorkbenchKind::Browse,
                    QStringLiteral("sceneExplorerTransformDoubleClick"));
            }
            if(!m_sceneExplorerController->resolvePendingTransformPreviewIfTargetChanges(node, this)) {
                refreshSceneExplorerViewModel();
                return;
            }
            if(node.kind == robot_qt_viewer::SceneExplorerNodeKind::Object) {
                const robot_qt_viewer::SceneSelectionIntent intent =
                    m_sceneExplorerController->selectionIntentForNode(node);
                if(intent.kind != robot_qt_viewer::SceneSelectionIntentKind::SelectSceneObject) {
                    return;
                }
                selectRobotContext(QString());
                m_appController.setObjectInspectorContext(intent.itemId);
                m_selectionModel.selectSceneObject(intent.itemId, QStringLiteral("sceneExplorerObjectEdit"));
                m_sceneExplorerController->focusTransformTask(node);
                statusBar()->showMessage(intent.statusMessage, 3000);
                updateTaskPanel();
                return;
            }
            if(node.kind == robot_qt_viewer::SceneExplorerNodeKind::ObjectFrame) {
                const robot_qt_viewer::SceneSelectionIntent intent =
                    m_sceneExplorerController->selectionIntentForNode(node);
                if(intent.kind != robot_qt_viewer::SceneSelectionIntentKind::SelectObjectFrame) {
                    return;
                }
                selectRobotContext(QString());
                m_appController.setObjectInspectorContext(intent.itemId);
                m_selectionModel.selectObjectFrame(
                    intent.itemId,
                    intent.linkName,
                    QStringLiteral("sceneExplorerObjectFrameEdit"));
                m_sceneExplorerController->focusTransformTask(node);
                statusBar()->showMessage(intent.statusMessage, 3000);
                updateTaskPanel();
                return;
            }
        });
    connect(m_sceneExplorerController, &robot_qt_viewer::SceneExplorerModuleController::contextMenuActionRequested,
        this, &MainWindow::handleSceneTreeContextMenuAction);
    connect(m_sceneExplorerController, &robot_qt_viewer::SceneExplorerModuleController::statusMessageRequested,
        this, [this](const QString& message, int timeoutMs) {
            statusBar()->showMessage(message, timeoutMs);
        });
    m_sceneExplorerController->setWorkbenchDescriptor(m_workbenchManager.activeDescriptor());
    if(m_sceneExplorerActionRouter != nullptr) {
        m_sceneExplorerActionRouter->setSceneExplorerController(m_sceneExplorerController);
    }
    if(m_viewport != nullptr) {
        m_viewport->setInteractionMode(toProjectSceneInteractionMode(m_workbenchManager.viewportMode()));
    }
    m_documentViewRegistry.registerModule(QStringLiteral("sceneExplorer"), m_sceneExplorerController,
        [this](const robot_qt_viewer::RobotQtViewerEvent& event) {
            m_sceneExplorerController->handleEvent(event);
            if(event.kind == robot_qt_viewer::RobotQtViewerEventKind::SelectionChanged) {
                refreshSelectedLinkMaterialSummary();
            }
        });
    robotDock->setWidget(m_sceneExplorerWidget);
    robotDock->setMinimumWidth(320);
    addDockWidget(Qt::LeftDockWidgetArea, robotDock);

    QDockWidget* resultDock = new QDockWidget("Scene Edit Panel", this);
    m_taskPanelDock = resultDock;
    QWidget* resultPanel = new QWidget(resultDock);
    resultPanel->setMinimumWidth(0);
    QVBoxLayout* resultLayout = new QVBoxLayout(resultPanel);
    resultLayout->setContentsMargins(12, 10, 12, 10);
    resultLayout->setSpacing(8);

    m_taskPanelStack = new QStackedWidget(resultPanel);
    m_taskPanelStack->setMinimumWidth(0);
    makeHorizontallyCompressible(m_taskPanelStack);

    auto* sceneExplorerTaskPanel = new QScrollArea(m_taskPanelStack);
    m_sceneExplorerTaskPanel = sceneExplorerTaskPanel;
    makeHorizontallyCompressible(sceneExplorerTaskPanel);
    sceneExplorerTaskPanel->setWidgetResizable(true);
    sceneExplorerTaskPanel->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    sceneExplorerTaskPanel->setFrameShape(QFrame::NoFrame);
    m_sceneExplorerTaskWidget = new SceneExplorerTaskWidget(sceneExplorerTaskPanel);
    sceneExplorerTaskPanel->setWidget(m_sceneExplorerTaskWidget);
    if(m_sceneExplorerController != nullptr) {
        m_sceneExplorerController->setTaskWidget(m_sceneExplorerTaskWidget);
    }

    auto* motionTaskPanel = new QScrollArea(m_taskPanelStack);
    m_motionTaskPanel = motionTaskPanel;
    makeHorizontallyCompressible(motionTaskPanel);
    motionTaskPanel->setWidgetResizable(true);
    motionTaskPanel->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    motionTaskPanel->setFrameShape(QFrame::NoFrame);

    m_motionControlWidget = new MotionControlWidget(motionTaskPanel);
    m_motionControlController = new robot_qt_viewer::MotionControlModuleController(
        *m_motionControlWidget,
        m_documentContext,
        *m_viewportServices,
        this);
    connect(m_motionControlController, &robot_qt_viewer::MotionControlModuleController::statusMessageRequested,
        this, [this](const QString& message, int timeoutMs) {
            statusBar()->showMessage(message, timeoutMs);
        });
    connect(m_motionControlController, &robot_qt_viewer::MotionControlModuleController::collisionQueriesEnabledChanged,
        this, [this](bool enabled) {
            if(m_collisionQueriesAction != nullptr) {
                QSignalBlocker blocker(m_collisionQueriesAction);
                m_collisionQueriesAction->setChecked(enabled);
            }
            if(m_statusLabel != nullptr) {
                m_statusLabel->setText(enabled
                    ? QStringLiteral("Collision detection enabled")
                    : QStringLiteral("Collision detection paused"));
            }
        });
    connect(m_motionControlController, &robot_qt_viewer::MotionControlModuleController::collisionGeometryVisibleChanged,
        this, [this](bool visible) {
            if(m_collisionGeometryAction != nullptr) {
                QSignalBlocker blocker(m_collisionGeometryAction);
                m_collisionGeometryAction->setChecked(visible);
            }
        });
    QDockWidget* bottomDock = new QDockWidget(uiText("action.robotRunDetails"), this);
    m_bottomPanelDock = bottomDock;
    constexpr int robotRunDetailsExpandedHeight = 280;
    constexpr int robotRunDetailsCollapsedHeight = 36;
    auto* robotRunDetailsTitleBar = new QWidget(bottomDock);
    auto* robotRunDetailsTitleLayout = new QHBoxLayout(robotRunDetailsTitleBar);
    robotRunDetailsTitleLayout->setContentsMargins(8, 2, 4, 2);
    robotRunDetailsTitleLayout->setSpacing(4);
    auto* robotRunDetailsTitle = new QLabel(uiText("action.robotRunDetails"), robotRunDetailsTitleBar);
    robotRunDetailsTitle->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    auto* robotRunDetailsCollapseButton = new QToolButton(robotRunDetailsTitleBar);
    robotRunDetailsCollapseButton->setText(QStringLiteral("-"));
    robotRunDetailsCollapseButton->setToolTip(QStringLiteral("Collapse Robot Run Details"));
    robotRunDetailsCollapseButton->setAutoRaise(true);
    auto* robotRunDetailsCloseButton = new QToolButton(robotRunDetailsTitleBar);
    robotRunDetailsCloseButton->setText(QStringLiteral("x"));
    robotRunDetailsCloseButton->setToolTip(QStringLiteral("Close Robot Run Details"));
    robotRunDetailsCloseButton->setAutoRaise(true);
    robotRunDetailsTitleLayout->addWidget(robotRunDetailsTitle, 1);
    robotRunDetailsTitleLayout->addWidget(robotRunDetailsCollapseButton);
    robotRunDetailsTitleLayout->addWidget(robotRunDetailsCloseButton);
    bottomDock->setTitleBarWidget(robotRunDetailsTitleBar);
    m_robotRunCollisionDetailsWidget = new CollisionRuntimeResultsWidget(bottomDock);
    m_robotRunCollisionDetailsWidget->setDisplayMode(CollisionResultsWidget::DisplayMode::Full);
    m_robotRunCollisionDetailsWidget->setMinimumHeight(robotRunDetailsExpandedHeight - 32);
    bottomDock->setWidget(m_robotRunCollisionDetailsWidget);
    bottomDock->setMinimumHeight(robotRunDetailsExpandedHeight);
    bottomDock->resize(bottomDock->width(), robotRunDetailsExpandedHeight);
    bottomDock->setFeatures(
        QDockWidget::DockWidgetClosable |
        QDockWidget::DockWidgetMovable |
        QDockWidget::DockWidgetFloatable);
    bottomDock->setProperty("robotRunDetailsCollapsed", false);
    connect(robotRunDetailsCollapseButton, &QToolButton::clicked, this,
        [bottomDock,
         robotRunDetailsCollapseButton,
         robotRunDetailsExpandedHeight,
         robotRunDetailsCollapsedHeight]() {
            const bool collapsed = bottomDock->property("robotRunDetailsCollapsed").toBool();
            if(collapsed) {
                if(QWidget* dockWidget = bottomDock->widget()) {
                    dockWidget->setVisible(true);
                }
                bottomDock->setMinimumHeight(robotRunDetailsExpandedHeight);
                bottomDock->setMaximumHeight(16777215);
                bottomDock->resize(bottomDock->width(), robotRunDetailsExpandedHeight);
                robotRunDetailsCollapseButton->setText(QStringLiteral("-"));
                robotRunDetailsCollapseButton->setToolTip(QStringLiteral("Collapse Robot Run Details"));
            } else {
                if(QWidget* dockWidget = bottomDock->widget()) {
                    dockWidget->setVisible(false);
                }
                bottomDock->setMinimumHeight(robotRunDetailsCollapsedHeight);
                bottomDock->setMaximumHeight(robotRunDetailsCollapsedHeight);
                bottomDock->resize(bottomDock->width(), robotRunDetailsCollapsedHeight);
                robotRunDetailsCollapseButton->setText(QStringLiteral("+"));
                robotRunDetailsCollapseButton->setToolTip(QStringLiteral("Restore Robot Run Details"));
            }
            bottomDock->setProperty("robotRunDetailsCollapsed", !collapsed);
        });
    connect(robotRunDetailsCloseButton, &QToolButton::clicked, bottomDock, &QDockWidget::hide);
    bottomDock->setVisible(false);
    addDockWidget(Qt::BottomDockWidgetArea, bottomDock);
    connect(bottomDock, &QDockWidget::visibilityChanged, this, [this](bool visible) {
        if(visible || !m_robotRunDetailsRequested ||
            m_workbenchManager.activeWorkbench() != robot_qt_viewer::RobotQtViewerWorkbenchKind::Motion) {
            return;
        }
        setRobotRunDetailsRequested(false);
    });
    m_motionControlController->setCollisionDetailsWidget(
        m_robotRunCollisionDetailsWidget->resultsWidget());
    m_documentViewRegistry.registerModule(QStringLiteral("motion"), m_motionControlController,
        [this](const robot_qt_viewer::RobotQtViewerEvent& event) {
            m_motionControlController->handleEvent(event);
        });
    motionTaskPanel->setWidget(m_motionControlWidget);

    auto* motionPlanningTaskPanel = new QScrollArea(m_taskPanelStack);
    m_motionPlanningTaskPanel = motionPlanningTaskPanel;
    makeHorizontallyCompressible(motionPlanningTaskPanel);
    motionPlanningTaskPanel->setWidgetResizable(true);
    motionPlanningTaskPanel->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    motionPlanningTaskPanel->setFrameShape(QFrame::NoFrame);
    m_motionPlanningWidget = new MotionPlanningEditorWidget(motionPlanningTaskPanel);
    m_motionPlanningController = new robot_qt_viewer::MotionPlanningModuleController(
        *m_motionPlanningWidget,
        m_documentContext,
        this);
    connect(m_motionPlanningController,
        &robot_qt_viewer::MotionPlanningModuleController::statusMessageRequested,
        this,
        [this](const QString& message, int timeoutMs) {
            statusBar()->showMessage(message, timeoutMs);
        });
    m_documentViewRegistry.registerModule(
        QStringLiteral("motionPlanning"),
        m_motionPlanningController,
        [this](const robot_qt_viewer::RobotQtViewerEvent& event) {
            m_motionPlanningController->handleEvent(event);
        });
    motionPlanningTaskPanel->setWidget(m_motionPlanningWidget);

    QWidget* collisionTaskPanel = new QWidget(m_taskPanelStack);
    QVBoxLayout* collisionLayout = new QVBoxLayout(collisionTaskPanel);
    collisionLayout->setContentsMargins(10, 10, 10, 10);
    collisionLayout->setSpacing(8);

    auto* toolTaskPanel = new QScrollArea(m_taskPanelStack);
    m_toolSetupTaskPanel = toolTaskPanel;
    makeHorizontallyCompressible(toolTaskPanel);
    toolTaskPanel->setWidgetResizable(true);
    toolTaskPanel->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    toolTaskPanel->setFrameShape(QFrame::NoFrame);

    m_toolSetupWidget = new ToolSetupWidget(toolTaskPanel);
    m_toolSetupController = new robot_qt_viewer::ToolSetupModuleController(
        *m_toolSetupWidget,
        m_documentContext,
        *m_toolSetupServices,
        this);
    connect(m_toolSetupController, &robot_qt_viewer::ToolSetupModuleController::mountFrameFocusRequested,
        this, &MainWindow::focusSceneExplorerMountFrame);
    connect(m_toolSetupController, &robot_qt_viewer::ToolSetupModuleController::linkFocusRequested,
        this, [this](const QString& robotId, const QString& linkName) {
            selectRobotContext(robotId, linkName);
            updateTaskPanel();
        });
    connect(m_toolSetupController, &robot_qt_viewer::ToolSetupModuleController::selectionDependentViewsRefreshRequested,
        this, [this]() {
            refreshSelectedLinkMaterialSummary();
        });
    connect(m_toolSetupController, &robot_qt_viewer::ToolSetupModuleController::statusMessageRequested,
        this, [this](const QString& message, int timeoutMs) {
            statusBar()->showMessage(message, timeoutMs);
        });
    connect(m_toolSetupController, &robot_qt_viewer::ToolSetupModuleController::viewModelRefreshed,
        this, [this]() {
            if(m_toolSetupController != nullptr) {
                m_toolSetupController->updateToolFrameVisibility();
            }
        });
    connect(m_toolSetupController, &robot_qt_viewer::ToolSetupModuleController::taskDirtyChanged,
        this, [this](bool dirty) {
            if(m_workbenchManager.activeWorkbench() != robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup) {
                return;
            }
            m_workbenchManager.setSessionDirty(dirty);
            m_workbenchManager.setSessionCanExit(!dirty);
        });
    connect(m_toolSetupController, &robot_qt_viewer::ToolSetupModuleController::taskExitRequested,
        this, [this]() {
            enterWorkbench(robot_qt_viewer::RobotQtViewerWorkbenchKind::Browse, QStringLiteral("toolSetupExit"));
            if(m_sceneExplorerController == nullptr) {
                return;
            }
            const robot_qt_viewer::SceneExplorerNodeRef node = m_sceneExplorerController->currentNode();
            if(node.kind != robot_qt_viewer::SceneExplorerNodeKind::RobotMount) {
                updateTaskPanel();
                return;
            }
            const robot_qt_viewer::SceneSelectionIntent intent =
                m_sceneExplorerController->selectionIntentForNode(node);
            if(intent.kind == robot_qt_viewer::SceneSelectionIntentKind::SelectRobotMount) {
                selectRobotContext(intent.robotId, intent.linkName, intent.mountId);
            }
            updateTaskPanel();
        });
    m_documentViewRegistry.registerModule(QStringLiteral("toolSetup"), m_toolSetupController,
        [this](const robot_qt_viewer::RobotQtViewerEvent& event) {
            const QString eventRobotId = !event.selection.robotId.isEmpty()
                ? event.selection.robotId
                : m_appController.selectedRobotId();
            const QString eventLinkName = !event.selection.linkName.isEmpty()
                ? event.selection.linkName
                : m_appController.selectedLinkName();
            m_toolSetupController->handleEvent(event, eventRobotId, eventLinkName);
        });
    if(m_sceneExplorerActionRouter != nullptr) {
        m_sceneExplorerActionRouter->setToolSetupController(m_toolSetupController);
    }

    toolTaskPanel->setWidget(m_toolSetupWidget);

    m_collisionWorkbenchPanel = new CollisionWorkbenchPanel(collisionTaskPanel);
    m_collisionWorkbenchController = new robot_qt_viewer::CollisionWorkbenchModuleController(
        *m_collisionWorkbenchPanel,
        m_documentContext,
        *m_collisionWorkbenchServices,
        this);
    connect(m_collisionWorkbenchController, &robot_qt_viewer::CollisionWorkbenchModuleController::statusMessageRequested,
        this, [this](const QString& message, int timeoutMs) {
            statusBar()->showMessage(message, timeoutMs);
        });
    connectCollisionWorkbenchPanel();
    m_documentViewRegistry.registerModule(QStringLiteral("collisionWorkbench"), m_collisionWorkbenchController,
        [this](const robot_qt_viewer::RobotQtViewerEvent& event) {
            m_collisionWorkbenchController->handleEvent(event);
        });
    if(m_sceneExplorerActionRouter != nullptr) {
        m_sceneExplorerActionRouter->setCollisionWorkbenchController(m_collisionWorkbenchController);
    }
    collisionLayout->addWidget(m_collisionWorkbenchPanel, 1);

    m_statusPanelWidget = new StatusPanelWidget(m_taskPanelStack);
    m_documentViewRegistry.registerModule(QStringLiteral("status"), m_statusPanelWidget,
        [this](const robot_qt_viewer::RobotQtViewerEvent& event) {
            if(event.kind == robot_qt_viewer::RobotQtViewerEventKind::StatusMessageRequested) {
                statusBar()->showMessage(event.message, event.timeoutMs > 0 ? event.timeoutMs : 2500);
                m_statusPanelWidget->setStatusItems({ event.message });
            } else if(event.kind == robot_qt_viewer::RobotQtViewerEventKind::ProjectOpened) {
                m_statusPanelWidget->setStatusItems({ QStringLiteral("Project opened") });
            } else if(event.kind == robot_qt_viewer::RobotQtViewerEventKind::ProjectSaved) {
                m_statusPanelWidget->setStatusItems({ QStringLiteral("Project saved") });
            }
        });

    auto* coatingAnalysisScrollArea = new QScrollArea(m_taskPanelStack);
    m_coatingAnalysisTaskPanel = coatingAnalysisScrollArea;
    makeHorizontallyCompressible(coatingAnalysisScrollArea);
    coatingAnalysisScrollArea->setWidgetResizable(true);
    coatingAnalysisScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    coatingAnalysisScrollArea->setFrameShape(QFrame::NoFrame);
    m_coatingAnalysisPanel = new robot_qt_viewer::CoatingAnalysisPanel(coatingAnalysisScrollArea);

    QDockWidget* coatingTreeDock = new QDockWidget(QStringLiteral("Coating Analysis"), this);
    m_coatingAnalysisTreeDock = coatingTreeDock;
    auto* coatingSplitter = new QSplitter(Qt::Vertical, coatingTreeDock);
    m_coatingAnalysisTreePanel = new robot_qt_viewer::CoatingAnalysisTreePanel(coatingSplitter);
    m_coatingAnalysisInfoPanel = new robot_qt_viewer::CoatingAnalysisInfoPanel(coatingSplitter);
    coatingSplitter->addWidget(m_coatingAnalysisTreePanel);
    coatingSplitter->addWidget(m_coatingAnalysisInfoPanel);
    coatingSplitter->setStretchFactor(0, 3);
    coatingSplitter->setStretchFactor(1, 2);
    coatingTreeDock->setWidget(coatingSplitter);
    coatingTreeDock->setMinimumWidth(300);
    addDockWidget(Qt::LeftDockWidgetArea, coatingTreeDock);
    coatingTreeDock->hide();

    m_coatingAnalysisVisibilityBar =
        new robot_qt_viewer::CoatingAnalysisVisibilityBar(m_viewport);
    m_coatingAnalysisVisibilityBar->setObjectName(
        QStringLiteral("RobotQtViewerCoatingVisibilityOverlay"));
    m_coatingAnalysisVisibilityBar->setAttribute(Qt::WA_StyledBackground, true);
    m_coatingAnalysisVisibilityBar->setStyleSheet(QStringLiteral(
        "QWidget#RobotQtViewerCoatingVisibilityOverlay {"
        "background: rgba(248, 250, 252, 232);"
        "border: 1px solid rgba(110, 122, 132, 160);"
        "border-radius: 6px;"
        "}"));
    updateCoatingVisibilityOverlayGeometry();
    m_coatingAnalysisVisibilityBar->hide();

    m_coatingAnalysisController = new robot_qt_viewer::CoatingAnalysisModuleController(
        *m_coatingAnalysisPanel,
        *m_coatingAnalysisTreePanel,
        *m_coatingAnalysisInfoPanel,
        *m_coatingAnalysisVisibilityBar,
        m_documentContext,
        this);
    m_thicknessLegendOverlay = new robot_qt_viewer::ThicknessLegendWidget(this);
    m_thicknessLegendOverlay->hide();
    connect(m_coatingAnalysisController,
        &robot_qt_viewer::CoatingAnalysisModuleController::thicknessLegendChanged,
        this,
        [this](bool visible, double minimumMicrometers, double maximumMicrometers) {
            if(m_thicknessLegendOverlay == nullptr) {
                return;
            }
            m_thicknessLegendOverlay->setRange(minimumMicrometers, maximumMicrometers);
            m_thicknessLegendOverlay->setVisible(visible);
            if(visible) {
                updateThicknessLegendOverlayGeometry();
            }
        });
    coatingAnalysisScrollArea->setWidget(m_coatingAnalysisPanel);
    connect(m_coatingAnalysisController,
        &robot_qt_viewer::CoatingAnalysisModuleController::statusMessageRequested,
        this,
        [this](const QString& message, int timeoutMs) {
            statusBar()->showMessage(message, timeoutMs);
        });
    connect(m_coatingAnalysisController,
        &robot_qt_viewer::CoatingAnalysisModuleController::thicknessToolTipRequested,
        this,
        [this](const QString& text, const QPoint& viewportPosition, bool visible) {
            if(!visible || text.isEmpty() || m_viewport == nullptr) {
                QToolTip::hideText();
                return;
            }
            QToolTip::showText(m_viewport->mapToGlobal(viewportPosition), text, m_viewport);
        });
    connect(m_viewport, &RobotViewport::surfaceScalarHovered,
        m_coatingAnalysisController,
        &robot_qt_viewer::CoatingAnalysisModuleController::handleSurfaceScalarHover);
    connect(m_viewport, &RobotViewport::rotationSurfacePicked,
        m_coatingAnalysisController,
        &robot_qt_viewer::CoatingAnalysisModuleController::handleRotationSurfacePicked);
    m_documentViewRegistry.registerModule(
        QStringLiteral("coatingAnalysis"),
        m_coatingAnalysisController,
        [this](const robot_qt_viewer::RobotQtViewerEvent& event) {
            m_coatingAnalysisController->handleEvent(event);
        });

    collisionTaskPanel->setLayout(collisionLayout);

    auto* collisionTaskScrollArea = new QScrollArea(m_taskPanelStack);
    m_collisionTaskPanel = collisionTaskScrollArea;
    makeHorizontallyCompressible(collisionTaskScrollArea);
    collisionTaskScrollArea->setWidgetResizable(true);
    collisionTaskScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    collisionTaskScrollArea->setFrameShape(QFrame::NoFrame);
    collisionTaskScrollArea->setWidget(collisionTaskPanel);

    m_taskPanelStack->addWidget(m_statusPanelWidget);
    m_taskPanelStack->addWidget(sceneExplorerTaskPanel);
    m_taskPanelStack->addWidget(motionTaskPanel);
    m_taskPanelStack->addWidget(motionPlanningTaskPanel);
    m_taskPanelStack->addWidget(toolTaskPanel);
    m_taskPanelStack->addWidget(collisionTaskScrollArea);
    m_taskPanelStack->addWidget(coatingAnalysisScrollArea);

    for(QPushButton* button : resultPanel->findChildren<QPushButton*>()) {
        configureInspectorButton(button);
    }
    for(QAbstractSpinBox* spin : resultPanel->findChildren<QAbstractSpinBox*>()) {
        makeHorizontallyCompressible(spin);
    }

    resultLayout->addWidget(m_taskPanelStack, 1);
    resultPanel->setLayout(resultLayout);
    resultDock->setWidget(resultPanel);
    resultDock->setMinimumWidth(300);
    addDockWidget(Qt::RightDockWidgetArea, resultDock);
    m_windowMenu->addAction(m_sceneExplorerDock->toggleViewAction());
    m_windowMenu->addAction(m_taskPanelDock->toggleViewAction());
    if(m_coatingAnalysisTreeDock != nullptr) {
        m_windowMenu->addAction(m_coatingAnalysisTreeDock->toggleViewAction());
    }
    m_windowMenu->addAction(m_robotRunDetailsAction);
    applyInitialPanelLayout();
    QTimer::singleShot(0, this, [this]() {
        applyInitialPanelLayout();
    });
    m_documentController.publishDocumentChanged(QStringLiteral("createPanelsInitialView"), false);
    updateWorkbenchActions();
    updateTaskPanel();
    updateRobotRunDetailsDockVisibility();
    updateCoatingAnalysisVisibility();
}

void MainWindow::applyInitialPanelLayout()
{
    if(m_sceneExplorerDock == nullptr || m_taskPanelDock == nullptr) {
        return;
    }
    resizeDocks({ m_sceneExplorerDock, m_taskPanelDock }, { 399, 479 }, Qt::Horizontal);
}

void MainWindow::addRobotLinksToTree(
    const QString& robotId,
    const QString& robotName,
    const QStringList& links,
    const QStringList& joints,
    const QStringList& movableJoints,
    const QStringList& movableJointTypes)
{
    if(m_motionControlController != nullptr) {
        m_motionControlController->setRobotRuntime(robotId, movableJoints, movableJointTypes);
    }

    if(m_sceneExplorerController != nullptr) {
        m_sceneExplorerController->setRobotRuntime(
            robotId,
            robotName,
            links,
            joints,
            movableJoints,
            movableJointTypes);
    }

    if(m_appController.selectedRobotId().isEmpty()) {
        selectRobotContext(robotId);
    }
}

void MainWindow::addSceneObjectToTree(const QString& objectId, const QString& objectName)
{
    if(m_sceneExplorerController != nullptr) {
        m_sceneExplorerController->setSceneObjectRuntime(objectId, objectName);
    }
}

void MainWindow::refreshSceneExplorerViewModel()
{
    if(m_sceneExplorerController == nullptr) {
        return;
    }

    m_sceneExplorerController->refreshViewModel();
}

bool MainWindow::resolveToolSetupPendingChanges(bool restoreEditorTarget)
{
    if(m_toolSetupController == nullptr || !m_toolSetupController->hasPendingTaskChanges()) {
        return true;
    }
    const bool resolved = m_toolSetupController->resolvePendingTaskChanges(this, restoreEditorTarget);
    updateWorkbenchActions();
    return resolved;
}

bool MainWindow::currentSceneExplorerNodeIsLink() const
{
    if(m_sceneExplorerController == nullptr) {
        return false;
    }
    return m_sceneExplorerController->currentNode().kind == robot_qt_viewer::SceneExplorerNodeKind::Link;
}

void MainWindow::focusSceneExplorerMountFrame(
    const QString& robotId,
    const QString& linkName,
    const QString& mountId)
{
    if(m_sceneExplorerController == nullptr || mountId.isEmpty()) {
        return;
    }

    refreshSceneExplorerViewModel();
    robot_qt_viewer::SceneExplorerNodeRef node;
    node.kind = robot_qt_viewer::SceneExplorerNodeKind::RobotMount;
    node.id = mountId;
    node.linkName = linkName;
    m_sceneExplorerController->selectNode(node);
    updateWorkbenchActions();
}

void MainWindow::handleSceneExplorerNodeActivated(const robot_qt_viewer::SceneExplorerNodeRef& node, int)
{
    if(m_viewport == nullptr || m_sceneExplorerController == nullptr) {
        return;
    }

    if(handleCollisionModelConfigurationNodeActivated(node)) {
        return;
    }

    const bool sameEditedMount =
        node.kind == robot_qt_viewer::SceneExplorerNodeKind::RobotMount &&
        m_toolSetupWidget != nullptr &&
        node.id == m_toolSetupWidget->currentMountId();
    const bool switchingFromFrameEditor =
        m_workbenchManager.activeWorkbench() == robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup &&
        !sameEditedMount;
    if(switchingFromFrameEditor && !resolveToolSetupPendingChanges(false)) {
        refreshSceneExplorerViewModel();
        return;
    }
    if(switchingFromFrameEditor && node.kind != robot_qt_viewer::SceneExplorerNodeKind::RobotMount) {
        enterWorkbench(robot_qt_viewer::RobotQtViewerWorkbenchKind::Browse, QStringLiteral("sceneExplorerSelection"));
    }
    if(!m_sceneExplorerController->resolvePendingTransformPreviewIfTargetChanges(node, this)) {
        refreshSceneExplorerViewModel();
        return;
    }

    const robot_qt_viewer::SceneSelectionIntent intent =
        m_sceneExplorerController->selectionIntentForNode(node);
    updateWorkbenchActions();
    switch(intent.kind) {
    case robot_qt_viewer::SceneSelectionIntentKind::Clear:
        selectRobotContext(QString());
        updateTaskPanel();
        return;
    case robot_qt_viewer::SceneSelectionIntentKind::SelectSceneObject:
        selectRobotContext(QString());
        m_appController.setObjectInspectorContext(intent.itemId);
        m_selectionModel.selectSceneObject(intent.itemId, QStringLiteral("sceneExplorer"));
        statusBar()->showMessage(intent.statusMessage, 3000);
        updateTaskPanel();
        return;
    case robot_qt_viewer::SceneSelectionIntentKind::SelectObjectFrame:
        if(m_sceneExplorerController != nullptr &&
            !m_sceneExplorerController->resolvePendingTransformPreviewIfTargetChanges(
                robot_qt_viewer::SceneExplorerNodeRef(),
                this)) {
            refreshSceneExplorerViewModel();
            return;
        }
        selectRobotContext(QString());
        if(m_sceneExplorerController != nullptr) {
            m_sceneExplorerController->setWorkbenchDescriptor(m_workbenchManager.activeDescriptor());
        }
        m_appController.setObjectInspectorContext(intent.itemId);
        m_selectionModel.selectObjectFrame(
            intent.itemId,
            intent.linkName,
            QStringLiteral("sceneExplorerObjectFrame"));
        statusBar()->showMessage(intent.statusMessage, 3000);
        updateTaskPanel();
        return;
    case robot_qt_viewer::SceneSelectionIntentKind::SelectToolAsset:
        selectRobotContext(QString());
        m_appController.setToolAssetInspectorContext(intent.itemId);
        m_selectionModel.selectToolAsset(intent.itemId, QStringLiteral("sceneExplorer"));
        statusBar()->showMessage(intent.statusMessage, 3000);
        updateTaskPanel();
        return;
    case robot_qt_viewer::SceneSelectionIntentKind::SelectRobotJoint:
        selectRobotContext(intent.robotId);
        m_selectionModel.selectRobotJoint(
            intent.robotId,
            intent.jointName,
            QStringLiteral("sceneExplorerRobotJoint"));
        statusBar()->showMessage(intent.statusMessage, 3000);
        updateTaskPanel();
        return;
    case robot_qt_viewer::SceneSelectionIntentKind::SelectRobotMount:
        if(m_workbenchManager.activeWorkbench() == robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup) {
            enterWorkbench(
                robot_qt_viewer::RobotQtViewerWorkbenchKind::Browse,
                QStringLiteral("sceneExplorerMountSelection"));
            if(m_workbenchManager.activeWorkbench() == robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup) {
                refreshSceneExplorerViewModel();
                return;
            }
        }
        selectRobotContext(intent.robotId, intent.linkName, intent.mountId);
        statusBar()->showMessage(intent.statusMessage, 3000);
        updateTaskPanel();
        return;
    case robot_qt_viewer::SceneSelectionIntentKind::SelectMountedAttachment:
        if(m_toolSetupController != nullptr) {
            m_toolSetupController->selectToolAttachmentById(intent.itemId.toStdString());
        }
        statusBar()->showMessage(intent.statusMessage, 3000);
        updateTaskPanel();
        return;
    case robot_qt_viewer::SceneSelectionIntentKind::SelectRobotLink:
        selectRobotContext(intent.robotId, intent.linkName);
        statusBar()->showMessage(intent.statusMessage, 3000);
        updateTaskPanel();
        return;
    case robot_qt_viewer::SceneSelectionIntentKind::None:
        if(!intent.statusMessage.isEmpty()) {
            statusBar()->showMessage(intent.statusMessage, 4000);
        }
        updateTaskPanel();
        return;
    }
}

bool MainWindow::handleCollisionModelConfigurationNodeActivated(
    const robot_qt_viewer::SceneExplorerNodeRef& node)
{
    if(m_workbenchManager.activeWorkbench() != robot_qt_viewer::RobotQtViewerWorkbenchKind::Collision ||
        m_collisionWorkbenchController == nullptr ||
        !m_collisionWorkbenchController->isCollisionModelConfigurationActive()) {
        return false;
    }

    if(!robot_qt_viewer::sceneExplorerNodeKindCanConfigureCollisionModel(node.kind)) {
        statusBar()->showMessage(
            QStringLiteral("Finish or cancel collision model configuration before selecting this item."),
            3000);
        if(m_hasCollisionModelConfigurationNode && m_sceneExplorerController != nullptr) {
            m_sceneExplorerController->selectNode(m_collisionModelConfigurationNode);
        }
        refreshSceneExplorerViewModel();
        return true;
    }

    if(!m_collisionWorkbenchController->canHandoffCollisionModelConfiguration()) {
        statusBar()->showMessage(QStringLiteral("Collision model configuration has pending changes."), 3000);
        if(m_hasCollisionModelConfigurationNode && m_sceneExplorerController != nullptr) {
            m_sceneExplorerController->selectNode(m_collisionModelConfigurationNode);
        }
        refreshSceneExplorerViewModel();
        return true;
    }

    m_collisionWorkbenchController->cancelCollisionModelConfigurationForHandoff(
        QStringLiteral("sceneExplorerCollisionModelTargetHandoff"));
    if(!selectCollisionModelConfigurationTarget(node, QStringLiteral("sceneExplorerCollisionModelTargetHandoff"))) {
        statusBar()->showMessage(
            QStringLiteral("Select a robot, link, object, or attachment to configure collision geometry."),
            3000);
        refreshSceneExplorerViewModel();
        return true;
    }

    rememberCollisionModelConfigurationTarget(node);
    m_collisionWorkbenchController->showCollisionModelConfiguration();
    statusBar()->showMessage(QStringLiteral("Collision model configuration switched."), 3000);
    updateWorkbenchActions();
    updateTaskPanel();
    refreshSceneExplorerViewModel();
    return true;
}

bool MainWindow::selectCollisionModelConfigurationTarget(
    const robot_qt_viewer::SceneExplorerNodeRef& node,
    const QString& sourceId)
{
    if(node.kind == robot_qt_viewer::SceneExplorerNodeKind::Robot ||
        node.kind == robot_qt_viewer::SceneExplorerNodeKind::Link) {
        const QString linkName = collisionModelConfigurationLinkName(node);
        if(node.id.isEmpty() || linkName.isEmpty()) {
            return false;
        }
        selectRobotContext(node.id, linkName);
        return true;
    }

    if(node.kind == robot_qt_viewer::SceneExplorerNodeKind::Object) {
        if(node.id.isEmpty()) {
            return false;
        }
        selectRobotContext(QString());
        m_appController.setObjectInspectorContext(node.id);
        m_selectionModel.selectSceneObject(node.id, sourceId);
        return true;
    }

    if(node.kind == robot_qt_viewer::SceneExplorerNodeKind::ToolAttachment) {
        if(node.id.isEmpty() || m_toolSetupController == nullptr) {
            return false;
        }
        m_toolSetupController->selectToolAttachmentById(node.id.toStdString());
        return true;
    }

    return false;
}

QString MainWindow::collisionModelConfigurationLinkName(
    const robot_qt_viewer::SceneExplorerNodeRef& node) const
{
    QString linkName = node.linkName;
    if(linkName.isEmpty() &&
        (node.kind == robot_qt_viewer::SceneExplorerNodeKind::Robot ||
            node.kind == robot_qt_viewer::SceneExplorerNodeKind::Link) &&
        m_sceneExplorerController != nullptr) {
        const QHash<QString, QStringList> robotLinksById =
            m_sceneExplorerController->robotLinksByRobotId();
        const QStringList robotLinks = robotLinksById.value(node.id);
        if(!robotLinks.isEmpty()) {
            linkName = robotLinks.first();
        }
    }
    return linkName;
}

void MainWindow::rememberCollisionModelConfigurationTarget(
    const robot_qt_viewer::SceneExplorerNodeRef& node)
{
    m_collisionModelConfigurationNode = node;
    m_hasCollisionModelConfigurationNode =
        robot_qt_viewer::sceneExplorerNodeKindCanConfigureCollisionModel(node.kind) &&
        !node.id.isEmpty();
}

void MainWindow::selectRobotContext(
    const QString& robotId,
    const QString& preferredLinkName,
    const QString& preferredMountId)
{
    robot_qt_viewer::SceneRobotSelectionContext selectionContext{
        robotId,
        preferredLinkName,
        preferredMountId
    };
    const bool explicitMountSelection = !preferredMountId.isEmpty();
    const bool explicitLinkSelection = !preferredLinkName.isEmpty() && preferredMountId.isEmpty();
    if(!explicitLinkSelection && m_sceneExplorerController != nullptr) {
        selectionContext =
            m_sceneExplorerController->robotSelectionContext(robotId, preferredLinkName, preferredMountId);
    }

    const QString mountId = selectionContext.mountId;
    const QString linkName = selectionContext.linkName;
    const QString selectedRobotId = selectionContext.robotId;
    if(!mountId.isEmpty()) {
        m_appController.setRobotMountInspectorContext(selectedRobotId, linkName, mountId);
        m_selectionModel.selectRobotMount(
            selectedRobotId,
            linkName,
            mountId,
            QStringLiteral("selectRobotContext"));
    } else {
        m_appController.setRobotLinkInspectorContext(selectedRobotId, linkName);
        m_selectionModel.selectRobotLink(selectedRobotId, linkName, QStringLiteral("selectRobotContext"));
    }

    refreshSelectedLinkMaterialSummary();
    if(explicitMountSelection &&
        m_toolSetupController != nullptr &&
        m_workbenchManager.activeWorkbench() == robot_qt_viewer::RobotQtViewerWorkbenchKind::ToolSetup) {
        m_toolSetupController->updateToolFrameVisibility();
    }
    updateWorkbenchActions();
}

void MainWindow::clearInspectorSelectionContext()
{
    m_appController.clearInspectorSelectionContext();
    m_selectionModel.clear(QStringLiteral("clearInspectorSelectionContext"));
}

void MainWindow::setActiveCollisionDetectorContext(const QString& detectorId)
{
    m_appController.setActiveCollisionDetectorContext(detectorId);
    m_selectionModel.setCollisionDetector(detectorId, QStringLiteral("collisionWorkbench"));
}

void MainWindow::setMarkedCollisionPairAContext(const QString& robotId, const QString& linkName)
{
    m_appController.setMarkedCollisionPairAContext(robotId, linkName);
    m_selectionModel.setCollisionPairA(robotId, linkName, QStringLiteral("collisionWorkbench"));
}

void MainWindow::refreshSelectedLinkMaterialSummary()
{
    if(m_sceneExplorerWidget == nullptr) {
        return;
    }

    if(m_appController.selectedRobotId().isEmpty() || m_appController.selectedLinkName().isEmpty()) {
        m_sceneExplorerWidget->setSummaryText(m_appController.selectedRobotId().isEmpty()
            ? "No robot selected"
            : QString("Robot: %1\nSelect a link to inspect material color.").arg(m_appController.selectedRobotId()));
        return;
    }

    const std::vector<ProjectScene::RobotLinkMaterialInfo> materials =
        m_viewport != nullptr
            ? m_viewport->robotLinkMaterials(m_appController.selectedRobotId(), m_appController.selectedLinkName())
            : std::vector<ProjectScene::RobotLinkMaterialInfo>();

    QStringList lines;
    lines << QString("Robot: %1").arg(m_appController.selectedRobotId());
    lines << QString("Link: %1").arg(m_appController.selectedLinkName());
    lines << "Material:";

    if(materials.empty()) {
        lines << "  no visual material information";
    } else {
        const int maxRows = std::min<int>(static_cast<int>(materials.size()), 4);
        for(int i = 0; i < maxRows; ++i) {
            const ProjectScene::RobotLinkMaterialInfo& info = materials[static_cast<std::size_t>(i)];
            const QString part = info.partUid.empty()
                ? QString("visual %1").arg(i + 1)
                : QString::fromUtf8(info.partUid.c_str());
            const QString mesh = info.meshPath.empty()
                ? QString("<none>")
                : QString::fromUtf8(info.meshPath.c_str());
            lines << QString("  %1").arg(part);
            lines << QString("    source: %1").arg(QString::fromUtf8(info.source.c_str()));
            lines << QString("    color: rgba(%1, %2, %3, %4)")
                .arg(info.r, 0, 'f', 3)
                .arg(info.g, 0, 'f', 3)
                .arg(info.b, 0, 'f', 3)
                .arg(info.a, 0, 'f', 3);
            lines << QString("    override: %1").arg(QString::fromUtf8(info.overrideState.c_str()));
            lines << QString("    mesh: %1").arg(mesh);
        }
        if(static_cast<int>(materials.size()) > maxRows) {
            lines << QString("  ... %1 more visual parts").arg(static_cast<int>(materials.size()) - maxRows);
        }
    }

    m_sceneExplorerWidget->setSummaryText(lines.join('\n'));
}

void MainWindow::connectCollisionWorkbenchPanel()
{
    if(m_collisionWorkbenchController == nullptr) {
        return;
    }

    connect(m_collisionWorkbenchController, &robot_qt_viewer::CollisionWorkbenchModuleController::refreshInspectorRequested,
        this, &MainWindow::refreshCollisionWorkbench);
    connect(m_collisionWorkbenchController, &robot_qt_viewer::CollisionWorkbenchModuleController::refreshSelectionDependentViewsRequested,
        this, [this]() {
            QTimer::singleShot(0, this, [this]() {
                if(m_collisionWorkbenchPanel == nullptr) {
                    return;
                }
                refreshCollisionElementList();
                refreshCollisionDetectorList();
            });
        });
    connect(m_collisionWorkbenchController, &robot_qt_viewer::CollisionWorkbenchModuleController::saveCollisionOverridesSidecarRequested, this, &MainWindow::saveCollisionOverridesAsSidecar);
    connect(m_collisionWorkbenchController, &robot_qt_viewer::CollisionWorkbenchModuleController::exportCollisionUrdfRequested, this, &MainWindow::exportRobotUrdfWithCollision);
    connect(m_collisionWorkbenchController, &robot_qt_viewer::CollisionWorkbenchModuleController::viewportReloadRequested, this, &MainWindow::reloadViewportProject);
    if(m_collisionWorkbenchPanel != nullptr) {
        connect(m_collisionWorkbenchPanel, &CollisionWorkbenchPanel::rightPanelTitleChanged,
            this, [this](const QString& title) {
                if(m_taskPanelDock != nullptr) {
                    m_taskPanelDock->setWindowTitle(title);
                }
            });
    }
}

void MainWindow::refreshCollisionWorkbench()
{
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->refreshInspector(m_collisionLastQualityMessage);
    }
}

void MainWindow::publishCollisionChanged(
    const QString& sourceId,
    const QString& detectorId)
{
    robot_qt_viewer::RobotQtViewerCollisionPayload payload;
    payload.detectorId = detectorId;
    m_documentController.publishCollisionChanged(payload, sourceId);
    m_documentController.publishDocumentChanged(sourceId, false);
}

bool MainWindow::collisionUiUpdating() const
{
    return m_collisionWorkbenchController != nullptr && m_collisionWorkbenchController->isUpdating();
}

void MainWindow::refreshCollisionPairList()
{
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->refreshCollisionPairList();
    }
}

void MainWindow::refreshCollisionDetectorList()
{
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->refreshCollisionDetectorList();
    }
}

void MainWindow::refreshCollisionSelectionSetList()
{
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->refreshCollisionSelectionSetList();
    }
}

void MainWindow::refreshCollisionDetectorPropertyEditors()
{
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->refreshCollisionDetectorPropertyEditors();
    }
}

void MainWindow::refreshCollisionSelectionSetMemberList()
{
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->refreshCollisionSelectionSetMemberList();
    }
}

QString MainWindow::currentCollisionSelectionSetId() const
{
    return m_collisionWorkbenchController != nullptr
        ? m_collisionWorkbenchController->currentCollisionSelectionSetId()
        : QString();
}

QString MainWindow::currentCollisionDetectorId() const
{
    const QString detectorId = m_collisionWorkbenchController != nullptr
        ? m_collisionWorkbenchController->currentCollisionDetectorId()
        : QString();
    return !detectorId.isEmpty() ? detectorId : m_appController.inspectorContext().activeCollisionDetectorId();
}

void MainWindow::refreshCollisionDetectorDetails()
{
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->refreshCollisionDetectorDetails();
    }
}

void MainWindow::refreshCollisionElementList()
{
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->refreshCollisionElementList(m_collisionLastQualityMessage);
    }
}

void MainWindow::refreshCollisionModelSummary()
{
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->refreshCollisionModelSummary(m_collisionLastQualityMessage);
    }
}

void MainWindow::saveCollisionOverridesToProject()
{
    if(m_projectRequiresSaveAs || m_projectPath.empty()) {
        saveProjectAs();
        return;
    }

    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->saveOverridesToProject();
    }
}

void MainWindow::saveCollisionOverridesAsSidecar()
{
    if(m_appController.selectedRobotId().isEmpty()) {
        statusBar()->showMessage("Select a robot first.", 3000);
        return;
    }

    if(m_collisionWorkbenchController == nullptr ||
        !m_collisionWorkbenchController->selectedRobotHasCollisionOverrides()) {
        statusBar()->showMessage("Selected robot has no collision overrides.", 3000);
        return;
    }

    const std::string robotId = m_appController.selectedRobotId().toStdString();
    const QString defaultName = m_projectPath.empty()
        ? QString("%1.collision.override.json").arg(m_appController.selectedRobotId())
        : QString::fromStdWString((m_projectPath.parent_path() /
              (robotId + ".collision.override.json")).wstring());

    const QString fileName = robot_qt_viewer::CollisionConfigDialogService::selectOverrideSidecarForSave(
        this,
        defaultName);

    if(fileName.isEmpty()) {
        return;
    }

    const std::filesystem::path path = fileName.toStdWString();
    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->saveOverridesAsSidecar(path, makePortableAssetPath(path));
    }
}

void MainWindow::exportRobotUrdfWithCollision()
{
    if(m_appController.selectedRobotId().isEmpty()) {
        statusBar()->showMessage("Select a robot first.", 3000);
        return;
    }

    if(m_collisionWorkbenchController == nullptr) {
        statusBar()->showMessage("Selected robot is not in project.", 3000);
        return;
    }

    const std::filesystem::path storedSourcePath = m_collisionWorkbenchController->selectedRobotSourcePath();
    if(storedSourcePath.empty()) {
        statusBar()->showMessage("Selected robot is not in project.", 3000);
        return;
    }
    if(QString::fromStdWString(storedSourcePath.extension().wstring()).compare(".urdf", Qt::CaseInsensitive) != 0) {
        statusBar()->showMessage("URDF collision export only supports URDF robots.", 4000);
        return;
    }

    if(!m_collisionWorkbenchController->selectedRobotHasCollisionOverrides()) {
        statusBar()->showMessage("Selected robot has no collision overrides.", 3000);
        return;
    }

    const std::filesystem::path sourcePath =
        resolveStoredAssetPathForSave(storedSourcePath.generic_u8string(), m_projectPath);
    const std::filesystem::path defaultPath =
        sourcePath.parent_path() / (sourcePath.stem().wstring() + L"_with_collision" + sourcePath.extension().wstring());

    const QString fileName = robot_qt_viewer::CollisionConfigDialogService::selectUrdfForExport(
        this,
        QString::fromStdWString(defaultPath.wstring()));

    if(fileName.isEmpty()) {
        return;
    }

    if(m_collisionWorkbenchController != nullptr) {
        m_collisionWorkbenchController->exportRobotUrdfWithCollision(
            sourcePath,
            std::filesystem::path(fileName.toStdWString()));
    }
}

void MainWindow::loadRobot()
{
    if(m_robotLoader == nullptr || m_sdk == nullptr) {
        statusBar()->showMessage("RobotSDK is not available.", 3000);
        return;
    }

    const QString fileName = robot_qt_viewer::ProjectAssemblyDialogService::selectRobotForImport(this);

    if(fileName.isEmpty()) {
        return;
    }

    smrobotgen2::sdk::RobotSourceType sourceType = smrobotgen2::sdk::RobotSourceType::Urdf;
    if(fileName.endsWith(".xml", Qt::CaseInsensitive)) {
        sourceType = smrobotgen2::sdk::RobotSourceType::Simscape;
    }

    const QByteArray localFileName = fileName.toLocal8Bit();
    smrobotgen2::sdk::LoadRobotResult result = m_robotLoader->load(
        sourceType,
        localFileName.constData());

    if(!smrobotgen2::sdk::succeeded(result.status) || result.model == nullptr) {
        const char* message = result.status.message != nullptr ? result.status.message : "Unknown error";
        statusBar()->showMessage(QString("Load failed: %1").arg(message), 5000);
        return;
    }

    if(m_robotModel != nullptr) {
        m_sdk->destroyRobotModel(m_robotModel);
    }
    m_robotModel = result.model;

    updateRobotPanel(*m_robotModel);
    m_viewport->setRobotSummary(m_robotModel->name(), m_robotModel->linkCount(), m_robotModel->jointCount());
    statusBar()->showMessage(QString("Loaded %1").arg(m_robotModel->name()), 5000);
}

void MainWindow::updateRobotPanel(const smrobotgen2::sdk::IRobotModel& model)
{
    if(m_sceneExplorerWidget != nullptr) {
        m_sceneExplorerWidget->setSummaryText(QString("Name: %1\nRoot: %2\nLinks: %3\nJoints: %4\nDOF: %5")
            .arg(model.name())
            .arg(model.rootLink())
            .arg(model.linkCount())
            .arg(model.jointCount())
            .arg(model.dof()));
    }

    if(m_statusPanelWidget == nullptr) {
        return;
    }

    QStringList statusItems;
    statusItems << QString("Loaded robot: %1").arg(model.name());
    statusItems << QString("Root link: %1").arg(model.rootLink());
    statusItems << QString("Links: %1").arg(model.linkCount());
    statusItems << QString("Joints: %1").arg(model.jointCount());
    statusItems << QString("DOF: %1").arg(model.dof());

    const std::size_t previewCount = model.linkCount() < 8 ? model.linkCount() : 8;
    for(std::size_t i = 0; i < previewCount; ++i) {
        statusItems << QString("Link %1: %2").arg(i).arg(model.linkName(i));
    }
    m_statusPanelWidget->setStatusItems(statusItems);
}

void MainWindow::saveViewportImage()
{
    QString selectedFilter;
    const QString fileName = QFileDialog::getSaveFileName(
        this,
        "Save viewport image",
        QString(),
        "PNG Image (*.png);;BMP Image (*.bmp)",
        &selectedFilter);

    if(fileName.isEmpty()) {
        return;
    }

    QString outputPath = fileName;
    if(!outputPath.endsWith(".png", Qt::CaseInsensitive) &&
        !outputPath.endsWith(".bmp", Qt::CaseInsensitive)) {
        outputPath += selectedFilter.contains("BMP", Qt::CaseInsensitive) ? ".bmp" : ".png";
    }

    const char* imageFormat = outputPath.endsWith(".bmp", Qt::CaseInsensitive) ? "BMP" : "PNG";
    const QImage image = m_viewport->grabFramebuffer();
    if(!image.isNull() && image.save(outputPath, imageFormat)) {
        statusBar()->showMessage(QString("Saved %1").arg(outputPath), 3000);
    } else {
        statusBar()->showMessage(QString("Failed to save %1").arg(outputPath), 3000);
    }
}

bool MainWindow::reloadViewportProject()
{
    const QString previousRobotId = m_appController.selectedRobotId();
    const QString previousLinkName = m_appController.selectedLinkName();
    const QString previousMountId = m_toolSetupWidget != nullptr
        ? m_toolSetupWidget->currentMountId()
        : QString();
    const QString previousAttachmentId = m_toolSetupWidget != nullptr
        ? m_toolSetupWidget->currentAttachmentId()
        : QString();
    const QString previousCollisionDetectorId = currentCollisionDetectorId();
    if(m_motionControlController != nullptr) {
        m_motionControlController->clearRuntime();
    }
    clearInspectorSelectionContext();

    if(m_sceneExplorerController != nullptr) {
        m_sceneExplorerController->clearRuntime();
    }

    const robot_qt_viewer::ViewportReloadWorkflowResult reloadResult =
        m_appController.reloadViewport(QStringLiteral("reloadViewportProject"));
    if(!reloadResult.success) {
        statusBar()->showMessage(reloadResult.errorMessage, 8000);
        return false;
    }
    if(!previousCollisionDetectorId.isEmpty()) {
        m_viewport->setActiveCollisionDetector(previousCollisionDetectorId);
        setActiveCollisionDetectorContext(previousCollisionDetectorId);
    }
    if(m_collisionGeometryAction != nullptr) {
        QSignalBlocker blocker(m_collisionGeometryAction);
        m_collisionGeometryAction->setChecked(reloadResult.showCollisionGeometry);
    }
    if(m_collisionQueriesAction != nullptr) {
        QSignalBlocker blocker(m_collisionQueriesAction);
        m_collisionQueriesAction->setChecked(false);
    }
    const bool restoreRobotContext =
        !previousRobotId.isEmpty() &&
        m_appController.hasRobot(previousRobotId);
    if(restoreRobotContext) {
        selectRobotContext(previousRobotId, previousLinkName, previousMountId);
        if(!previousAttachmentId.isEmpty()) {
            m_selectionModel.selectMountedAttachment(
                previousAttachmentId,
                previousRobotId,
                previousLinkName,
                previousMountId,
                QStringLiteral("reloadViewportProject"));
            robot_qt_viewer::RobotQtViewerViewportPreviewPayload preview;
            preview.setActiveMountedAttachment = true;
            preview.activeMountedAttachmentId = previousAttachmentId;
            m_viewportPreviewState.mutate(preview, QStringLiteral("reloadViewportProject"));
        }
    }
    m_viewport->update();
    LOG_DEBUG("rs2026") << "MainWindow reloadViewportProject: elapsedMs=" << reloadResult.elapsedMs
        << ", robots=" << reloadResult.robotCount
        << ", objects=" << reloadResult.objectCount
        << ", detectors=" << reloadResult.detectorCount;
    return true;
}

std::string MainWindow::makePortableAssetPath(const std::filesystem::path& assetPath) const
{
    return m_projectSession.makePortableAssetPath(assetPath);
}

QTreeWidget* MainWindow::robotTree() const
{
    return m_sceneExplorerWidget != nullptr ? m_sceneExplorerWidget->treeWidget() : nullptr;
}
