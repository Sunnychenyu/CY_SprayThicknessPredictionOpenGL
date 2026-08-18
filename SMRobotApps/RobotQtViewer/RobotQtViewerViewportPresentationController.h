#pragma once

#include <QByteArray>
#include <QPointer>
#include <QVector>
#include <QWidget>

class QMainWindow;

namespace robot_qt_viewer
{
    class RobotQtViewerViewportPresentationController
    {
    public:
        RobotQtViewerViewportPresentationController(QMainWindow& window, QWidget& viewport);

        bool isActive() const;
        void setActive(bool active);

    private:
        struct SavedWidgetVisibility
        {
            QPointer<QWidget> widget;
            bool visible = false;
        };

        void enter();
        void leave();
        void rememberAndHide(QWidget* widget);

        QMainWindow& m_window;
        QWidget& m_viewport;
        bool m_active = false;
        Qt::WindowStates m_savedWindowState = Qt::WindowNoState;
        QByteArray m_savedWindowGeometry;
        QVector<SavedWidgetVisibility> m_savedWidgetVisibility;
    };
}
