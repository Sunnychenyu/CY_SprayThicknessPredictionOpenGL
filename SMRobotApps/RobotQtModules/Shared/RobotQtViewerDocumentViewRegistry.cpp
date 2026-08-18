#include "RobotQtViewerDocumentViewRegistry.h"

#include <utility>

namespace robot_qt_viewer
{
    RobotQtViewerDocumentViewRegistry::RobotQtViewerDocumentViewRegistry(
        RobotQtViewerEventHub& eventHub,
        const RobotQtViewerWindowConfig& windowConfig)
        : m_eventHub(eventHub)
        , m_windowConfig(windowConfig)
    {
    }

    void RobotQtViewerDocumentViewRegistry::registerModule(
        const QString& moduleId,
        QObject* receiver,
        RobotQtViewerEventHub::EventHandler handler)
    {
        if(receiver == nullptr || !handler) {
            return;
        }

        unregisterModule(receiver);
        m_eventHub.subscribeModule(m_windowConfig, moduleId, receiver, std::move(handler));

        RegisteredModule module;
        module.moduleId = moduleId;
        module.receiver = receiver;
        m_modules.push_back(module);
    }

    void RobotQtViewerDocumentViewRegistry::unregisterModule(QObject* receiver)
    {
        if(receiver == nullptr) {
            return;
        }

        m_eventHub.unsubscribe(receiver);
        for(int i = m_modules.size() - 1; i >= 0; --i) {
            if(m_modules[i].receiver == receiver || m_modules[i].receiver.isNull()) {
                m_modules.removeAt(i);
            }
        }
    }

    int RobotQtViewerDocumentViewRegistry::registeredModuleCount() const
    {
        int count = 0;
        for(const RegisteredModule& module : m_modules) {
            if(!module.receiver.isNull()) {
                ++count;
            }
        }
        return count;
    }
}
