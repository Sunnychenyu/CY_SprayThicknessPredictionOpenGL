#pragma once

#include "RobotQtViewerEventHub.h"
#include "RobotQtViewerWindowConfig.h"

#include <QPointer>
#include <QString>
#include <QVector>

namespace robot_qt_viewer
{
    class RobotQtViewerDocumentViewRegistry
    {
    public:
        RobotQtViewerDocumentViewRegistry(
            RobotQtViewerEventHub& eventHub,
            const RobotQtViewerWindowConfig& windowConfig);

        void registerModule(
            const QString& moduleId,
            QObject* receiver,
            RobotQtViewerEventHub::EventHandler handler);
        void unregisterModule(QObject* receiver);

        int registeredModuleCount() const;

    private:
        struct RegisteredModule
        {
            QString moduleId;
            QPointer<QObject> receiver;
        };

        RobotQtViewerEventHub& m_eventHub;
        const RobotQtViewerWindowConfig& m_windowConfig;
        QVector<RegisteredModule> m_modules;
    };
}
