#include "RobotQtViewerToolbarController.h"

#include <QAction>
#include <QFrame>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QMainWindow>
#include <QMenu>
#include <QSize>
#include <QSizePolicy>
#include <QStyle>
#include <QToolBar>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>

namespace
{
    QString generatedIconResourcePath(const QString& actionId)
    {
        if(actionId == QStringLiteral("newProject")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/new_project.png");
        }
        if(actionId == QStringLiteral("openProject")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/open_project.png");
        }
        if(actionId == QStringLiteral("saveProject")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/save_project.png");
        }
        if(actionId == QStringLiteral("saveProjectAs")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/save_project.png");
        }
        if(actionId == QStringLiteral("saveCollisionOverrides")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/save_collision_overrides.png");
        }
        if(actionId == QStringLiteral("importRobot")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/import_robot.png");
        }
        if(actionId == QStringLiteral("importObject")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/import_object.png");
        }
        if(actionId == QStringLiteral("deleteSelectedItem")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/delete_selected.png");
        }
        if(actionId == QStringLiteral("saveImage")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/save_image.png");
        }
        if(actionId == QStringLiteral("resetCamera")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/camera_view_home.png");
        }
        if(actionId == QStringLiteral("cameraViewIsometric")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/camera_view_isometric.png");
        }
        if(actionId == QStringLiteral("cameraViewFront")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/camera_view_front.png");
        }
        if(actionId == QStringLiteral("cameraViewBack")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/camera_view_back.png");
        }
        if(actionId == QStringLiteral("cameraViewLeft")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/camera_view_left.png");
        }
        if(actionId == QStringLiteral("cameraViewRight")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/camera_view_right.png");
        }
        if(actionId == QStringLiteral("cameraViewTop")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/camera_view_top.png");
        }
        if(actionId == QStringLiteral("cameraViewBottom")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/camera_view_bottom.png");
        }
        if(actionId == QStringLiteral("collisionGeometry")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/collision_geometry.png");
        }
        if(actionId == QStringLiteral("browseWorkbench")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/browse_workbench.png");
        }
        if(actionId == QStringLiteral("projectAssemblyWorkbench")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/browse_workbench.png");
        }
        if(actionId == QStringLiteral("motionWorkbench")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/motion_workbench.png");
        }
        if(actionId == QStringLiteral("robotRunWorkbench")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/motion_workbench.png");
        }
        if(actionId == QStringLiteral("toolSetupWorkbench")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/tool_setup_workbench.png");
        }
        if(actionId == QStringLiteral("collisionWorkbench")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/collision_workbench.png");
        }
        if(actionId == QStringLiteral("collisionConfigWorkbench")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/collision_workbench.png");
        }
        if(actionId == QStringLiteral("trajectoryPlanningWorkbench") ||
            actionId == QStringLiteral("motionPlanningWorkbench")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/motion_planning_workbench.png");
        }
        if(actionId == QStringLiteral("sprayProcessWorkbench")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/spray_process_workbench.png");
        }
        if(actionId == QStringLiteral("coatingAnalysisWorkbench")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/coating_analysis_workbench.png");
        }
        if(actionId == QStringLiteral("digitalTwinWorkbench")) {
            return QStringLiteral(":/RobotQtViewer/icons/ribbon/digital_twin_workbench.png");
        }

        return QString();
    }

    QIcon generatedRibbonIcon(const QString& actionId)
    {
        const QString resourcePath = generatedIconResourcePath(actionId);
        return resourcePath.isEmpty() ? QIcon() : QIcon(resourcePath);
    }

    QIcon themedIcon(QWidget* widget, const QString& themeName, QStyle::StandardPixmap fallback)
    {
        const QIcon themeIcon = QIcon::fromTheme(themeName);
        if(!themeIcon.isNull()) {
            return themeIcon;
        }
        return widget != nullptr && widget->style() != nullptr
            ? widget->style()->standardIcon(fallback)
            : QIcon();
    }
}

namespace robot_qt_viewer
{
    RobotQtViewerToolbarController::RobotQtViewerToolbarController(QMainWindow& window, QObject* parent)
        : QObject(parent)
        , m_window(window)
    {
    }

    QToolBar* RobotQtViewerToolbarController::toolbar() const
    {
        return m_toolbar;
    }

    void RobotQtViewerToolbarController::build(const RobotQtViewerToolbarActions& actions)
    {
        if(m_toolbar == nullptr) {
            m_toolbar = m_window.addToolBar(QString());
            configureToolbar();
        }

        m_actions = makeActionMap(actions);
        renderModel(makeDefaultRobotQtViewerRibbonModel());
    }

    void RobotQtViewerToolbarController::retranslate(const RobotQtViewerToolbarTexts& texts)
    {
        if(m_toolbar != nullptr) {
            m_toolbar->setWindowTitle(texts.toolbarTitle);
        }

        const RobotQtViewerRibbonModel model = makeDefaultRobotQtViewerRibbonModel(texts);
        for(const RobotQtViewerRibbonPageSpec& page : model.pages) {
            for(const RobotQtViewerRibbonGroupSpec& group : page.groups) {
                QLabel* label = m_groupLabels.value(group.groupId, nullptr);
                if(label != nullptr) {
                    label->setText(group.title);
                }
            }
        }
    }

    void RobotQtViewerToolbarController::setActionTextAndToolTip(QAction* action, const QString& text)
    {
        if(action == nullptr) {
            return;
        }
        action->setText(text);
        action->setIconText(text);
        action->setToolTip(text);
        action->setStatusTip(text);
    }

    QLabel* RobotQtViewerToolbarController::makeGroupLabel(const QString& text, QWidget* parent)
    {
        auto* label = new QLabel(text, parent);
        label->setObjectName(QStringLiteral("RobotQtViewerRibbonGroupLabel"));
        label->setMargin(0);
        label->setAlignment(Qt::AlignCenter);
        return label;
    }

    void RobotQtViewerToolbarController::configureToolbar()
    {
        m_toolbar->setMovable(false);
        m_toolbar->setFloatable(false);
        m_toolbar->setAllowedAreas(Qt::TopToolBarArea);
        m_toolbar->setToolButtonStyle(Qt::ToolButtonIconOnly);
        m_toolbar->setIconSize(QSize(58, 58));
        m_toolbar->setStyleSheet(QStringLiteral(
            "QToolBar { spacing: 0px; padding: 0px; border-bottom: 1px solid palette(midlight); }"
            "QWidget#RobotQtViewerRibbonSurface { padding: 6px 10px 8px 10px; }"
            "QWidget#RobotQtViewerRibbonGroup { padding: 0px 8px 0px 8px; }"
            "QLabel#RobotQtViewerRibbonGroupLabel { color: palette(text); font-size: 13pt; "
            "font-weight: 700; padding: 2px 6px 6px 6px; }"
            "QToolButton { min-width: 68px; max-width: 68px; min-height: 68px; max-height: 68px; "
            "padding: 5px; border-radius: 5px; }"
            "QToolButton#RobotQtViewerRibbonCompactButton { min-width: 36px; max-width: 36px; "
            "min-height: 36px; max-height: 36px; padding: 3px; }"
            "QToolButton:hover { background: palette(alternate-base); }"
            "QToolButton:pressed, QToolButton:checked { background: palette(highlight); "
            "color: palette(highlighted-text); }"));
    }

    void RobotQtViewerToolbarController::renderModel(const RobotQtViewerRibbonModel& model)
    {
        if(m_toolbar == nullptr) {
            return;
        }

        m_toolbar->clear();
        m_groupLabels.clear();
        m_toolbar->setWindowTitle(model.toolbarTitle);

        auto* ribbonSurface = new QWidget(m_toolbar);
        ribbonSurface->setObjectName(QStringLiteral("RobotQtViewerRibbonSurface"));
        ribbonSurface->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

        auto* groupsLayout = new QHBoxLayout(ribbonSurface);
        groupsLayout->setContentsMargins(0, 0, 0, 0);
        groupsLayout->setSpacing(0);

        bool needsSeparator = false;
        for(const RobotQtViewerRibbonPageSpec& page : model.pages) {
            Q_UNUSED(page.title);
            for(const RobotQtViewerRibbonGroupSpec& group : page.groups) {
                if(needsSeparator) {
                    auto* separator = new QFrame(ribbonSurface);
                    separator->setFrameShape(QFrame::VLine);
                    separator->setFrameShadow(QFrame::Sunken);
                    groupsLayout->addWidget(separator);
                }

                auto* groupWidget = new QWidget(ribbonSurface);
                groupWidget->setObjectName(QStringLiteral("RobotQtViewerRibbonGroup"));
                groupWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);

                auto* groupLayout = new QVBoxLayout(groupWidget);
                groupLayout->setContentsMargins(0, 0, 0, 0);
                groupLayout->setSpacing(2);

                QLabel* label = makeGroupLabel(group.title, groupWidget);
                m_groupLabels.insert(group.groupId, label);
                groupLayout->addWidget(label);

                auto* buttonsLayout = new QHBoxLayout();
                buttonsLayout->setContentsMargins(0, 0, 0, 0);
                buttonsLayout->setSpacing(4);

                for(const RobotQtViewerRibbonActionSpec& actionSpec : group.actions) {
                    QAction* action = m_actions.value(actionSpec.actionId, nullptr);
                    if(action == nullptr) {
                        continue;
                    }
                    assignIcon(action, actionSpec);

                    auto* button = new QToolButton(groupWidget);
                    button->setDefaultAction(action);
                    if(actionSpec.compact) {
                        button->setObjectName(QStringLiteral("RobotQtViewerRibbonCompactButton"));
                    }
                    button->setToolButtonStyle(Qt::ToolButtonIconOnly);
                    button->setIconSize(actionSpec.compact ? QSize(28, 28) : QSize(58, 58));
                    button->setFixedSize(actionSpec.compact ? QSize(36, 36) : QSize(68, 68));
                    button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
                    buttonsLayout->addWidget(button);
                }

                groupLayout->addLayout(buttonsLayout);
                groupsLayout->addWidget(groupWidget);
                needsSeparator = true;
            }
        }

        groupsLayout->addStretch(1);
        m_toolbar->addWidget(ribbonSurface);
    }

    void RobotQtViewerToolbarController::assignIcon(
        QAction* action,
        const RobotQtViewerRibbonActionSpec& actionSpec) const
    {
        if(action == nullptr) {
            return;
        }
        const QIcon icon = generatedRibbonIcon(actionSpec.actionId);
        action->setIcon(!icon.isNull()
            ? icon
            : themedIcon(&m_window, actionSpec.themeIconName, actionSpec.fallbackIcon));
    }

    QHash<QString, QAction*> RobotQtViewerToolbarController::makeActionMap(
        const RobotQtViewerToolbarActions& actions) const
    {
        QHash<QString, QAction*> actionMap;
        actionMap.insert(QStringLiteral("newProject"), actions.newProject);
        actionMap.insert(QStringLiteral("openProject"), actions.openProject);
        actionMap.insert(QStringLiteral("saveProject"), actions.saveProject);
        actionMap.insert(QStringLiteral("saveProjectAs"), actions.saveProjectAs);
        actionMap.insert(QStringLiteral("saveCollisionOverrides"), actions.saveCollisionOverrides);
        actionMap.insert(QStringLiteral("importRobot"), actions.importRobot);
        actionMap.insert(QStringLiteral("importObject"), actions.importObject);
        actionMap.insert(QStringLiteral("deleteSelectedItem"), actions.deleteSelectedItem);
        actionMap.insert(QStringLiteral("saveImage"), actions.saveImage);
        actionMap.insert(QStringLiteral("resetCamera"), actions.resetCamera);
        actionMap.insert(QStringLiteral("collisionGeometry"), actions.collisionGeometry);
        actionMap.insert(QStringLiteral("collisionQueries"), actions.collisionQueries);
        actionMap.insert(QStringLiteral("browseWorkbench"), actions.browseWorkbench);
        actionMap.insert(QStringLiteral("projectAssemblyWorkbench"), actions.browseWorkbench);
        actionMap.insert(QStringLiteral("motionWorkbench"), actions.motionWorkbench);
        actionMap.insert(QStringLiteral("robotRunWorkbench"), actions.motionWorkbench);
        actionMap.insert(QStringLiteral("toolSetupWorkbench"), actions.toolSetupWorkbench);
        actionMap.insert(QStringLiteral("collisionWorkbench"), actions.collisionWorkbench);
        actionMap.insert(QStringLiteral("collisionConfigWorkbench"), actions.collisionWorkbench);
        actionMap.insert(QStringLiteral("trajectoryPlanningWorkbench"), actions.trajectoryPlanningWorkbench);
        actionMap.insert(QStringLiteral("motionPlanningWorkbench"), actions.trajectoryPlanningWorkbench);
        actionMap.insert(QStringLiteral("sprayProcessWorkbench"), actions.sprayProcessWorkbench);
        actionMap.insert(QStringLiteral("coatingAnalysisWorkbench"), actions.coatingAnalysisWorkbench);
        actionMap.insert(QStringLiteral("digitalTwinWorkbench"), actions.digitalTwinWorkbench);
        return actionMap;
    }
}
