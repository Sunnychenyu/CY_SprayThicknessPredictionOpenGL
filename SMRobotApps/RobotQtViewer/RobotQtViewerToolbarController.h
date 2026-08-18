#pragma once

#include "RobotQtViewerRibbonModel.h"

#include <QHash>
#include <QObject>
#include <QString>
#include <QVector>

class QAction;
class QLabel;
class QMainWindow;
class QToolBar;
class QWidget;

namespace robot_qt_viewer
{
    struct RobotQtViewerToolbarActions
    {
        QAction* newProject = nullptr;
        QAction* openProject = nullptr;
        QAction* saveProject = nullptr;
        QAction* saveProjectAs = nullptr;
        QAction* saveCollisionOverrides = nullptr;
        QAction* importRobot = nullptr;
        QAction* importObject = nullptr;
        QAction* deleteSelectedItem = nullptr;
        QAction* saveImage = nullptr;
        QAction* resetCamera = nullptr;
        QAction* collisionGeometry = nullptr;
        QAction* collisionQueries = nullptr;
        QAction* browseWorkbench = nullptr;
        QAction* motionWorkbench = nullptr;
        QAction* toolSetupWorkbench = nullptr;
        QAction* collisionWorkbench = nullptr;
        QAction* trajectoryPlanningWorkbench = nullptr;
        QAction* sprayProcessWorkbench = nullptr;
        QAction* coatingAnalysisWorkbench = nullptr;
        QAction* digitalTwinWorkbench = nullptr;
    };

    using RobotQtViewerToolbarTexts = RobotQtViewerRibbonTexts;

    class RobotQtViewerToolbarController : public QObject
    {
        Q_OBJECT

    public:
        explicit RobotQtViewerToolbarController(QMainWindow& window, QObject* parent = nullptr);

        QToolBar* toolbar() const;
        void build(const RobotQtViewerToolbarActions& actions);
        void retranslate(const RobotQtViewerToolbarTexts& texts);
        void setActionTextAndToolTip(QAction* action, const QString& text);

    private:
        QLabel* makeGroupLabel(const QString& text, QWidget* parent);
        void configureToolbar();
        void renderModel(const RobotQtViewerRibbonModel& model);
        void assignIcon(QAction* action, const RobotQtViewerRibbonActionSpec& actionSpec) const;
        QHash<QString, QAction*> makeActionMap(const RobotQtViewerToolbarActions& actions) const;

        QMainWindow& m_window;
        QToolBar* m_toolbar = nullptr;
        QHash<QString, QAction*> m_actions;
        QHash<QString, QLabel*> m_groupLabels;
    };
}
