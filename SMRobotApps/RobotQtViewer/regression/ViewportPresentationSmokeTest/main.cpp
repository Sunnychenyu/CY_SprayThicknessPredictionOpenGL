#include "RobotQtViewerViewportPresentationController.h"

#include <QApplication>
#include <QDockWidget>
#include <QMainWindow>
#include <QMenuBar>
#include <QStatusBar>
#include <QToolBar>
#include <QWidget>

#include <iostream>

namespace
{
    bool require(bool condition, const char* message)
    {
        if(condition) {
            return true;
        }

        std::cerr << message << '\n';
        return false;
    }
}

int main(int argc, char* argv[])
{
    QApplication application(argc, argv);

    QMainWindow window;
    window.resize(960, 640);
    auto* viewport = new QWidget(&window);
    window.setCentralWidget(viewport);
    window.menuBar()->addMenu(QStringLiteral("View"));
    window.statusBar()->showMessage(QStringLiteral("Ready"));
    QToolBar* toolBar = window.addToolBar(QStringLiteral("Ribbon"));

    auto* visibleDock = new QDockWidget(QStringLiteral("Visible"), &window);
    visibleDock->setWidget(new QWidget(visibleDock));
    window.addDockWidget(Qt::LeftDockWidgetArea, visibleDock);

    auto* hiddenDock = new QDockWidget(QStringLiteral("Hidden"), &window);
    hiddenDock->setWidget(new QWidget(hiddenDock));
    window.addDockWidget(Qt::RightDockWidgetArea, hiddenDock);
    hiddenDock->hide();

    window.show();
    application.processEvents();

    robot_qt_viewer::RobotQtViewerViewportPresentationController controller(window, *viewport);
    controller.setActive(true);
    application.processEvents();

    if(!require(controller.isActive(), "Presentation controller did not enter full screen.") ||
       !require(window.isFullScreen(), "Main window is not full screen.") ||
       !require(!window.menuBar()->isVisible(), "Menu bar remains visible in full screen.") ||
       !require(!window.statusBar()->isVisible(), "Status bar remains visible in full screen.") ||
       !require(!toolBar->isVisible(), "Toolbar remains visible in full screen.") ||
       !require(!visibleDock->isVisible(), "Visible dock remains visible in full screen.") ||
       !require(!hiddenDock->isVisible(), "Hidden dock became visible in full screen.")) {
        return 1;
    }

    controller.setActive(false);
    application.processEvents();

    if(!require(!controller.isActive(), "Presentation controller did not leave full screen.") ||
       !require(!window.isFullScreen(), "Main window remains full screen after restore.") ||
       !require(window.menuBar()->isVisible(), "Menu bar visibility was not restored.") ||
       !require(window.statusBar()->isVisible(), "Status bar visibility was not restored.") ||
       !require(toolBar->isVisible(), "Toolbar visibility was not restored.") ||
       !require(visibleDock->isVisible(), "Visible dock visibility was not restored.") ||
       !require(!hiddenDock->isVisible(), "Originally hidden dock was incorrectly shown.")) {
        return 1;
    }

    window.showMaximized();
    application.processEvents();
    controller.setActive(true);
    application.processEvents();
    controller.setActive(false);
    application.processEvents();

    if(!require(window.isMaximized(), "Maximized window state was not restored.")) {
        return 1;
    }

    std::cout << "RobotQtViewer viewport presentation smoke passed.\n";
    return 0;
}
