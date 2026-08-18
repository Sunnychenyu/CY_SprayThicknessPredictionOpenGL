#include "RobotQtViewerEventHub.h"

#include <utility>

namespace robot_qt_viewer
{
    RobotQtViewerEventHub::RobotQtViewerEventHub(QObject* parent)
        : QObject(parent)
    {
    }

    void RobotQtViewerEventHub::publish(const RobotQtViewerEvent& event)
    {
        const QVector<SubscriptionEntry> snapshot = m_subscriptions;
        for(const SubscriptionEntry& entry : snapshot) {
            if(entry.kind != event.kind || entry.receiver.isNull() || !entry.handler) {
                continue;
            }
            entry.handler(event);
        }
    }

    void RobotQtViewerEventHub::subscribe(
        RobotQtViewerEventKind kind,
        QObject* receiver,
        EventHandler handler,
        const QString& subscriberId)
    {
        if(receiver == nullptr || !handler) {
            return;
        }

        SubscriptionEntry entry;
        entry.kind = kind;
        entry.subscriberId = subscriberId;
        entry.receiver = receiver;
        entry.handler = std::move(handler);
        m_subscriptions.push_back(std::move(entry));

        QObject::connect(receiver, &QObject::destroyed, this, [this, receiver]() {
            unsubscribe(receiver);
        });
    }

    void RobotQtViewerEventHub::subscribeModule(
        const RobotQtViewerWindowConfig& config,
        const QString& subscriberId,
        QObject* receiver,
        EventHandler handler)
    {
        for(const RobotQtViewerModuleDesc& module : config.modules) {
            if(module.id != subscriberId || !module.enabled) {
                continue;
            }
            for(const RobotQtViewerSubscriptionDesc& subscription : module.subscriptions) {
                subscribe(subscription.eventKind, receiver, handler, subscription.subscriberId);
            }
            return;
        }
    }

    void RobotQtViewerEventHub::unsubscribe(QObject* receiver)
    {
        if(receiver == nullptr) {
            return;
        }

        for(int i = m_subscriptions.size() - 1; i >= 0; --i) {
            if(m_subscriptions[i].receiver == receiver || m_subscriptions[i].receiver.isNull()) {
                m_subscriptions.removeAt(i);
            }
        }
    }

    int RobotQtViewerEventHub::subscriptionCount() const
    {
        return m_subscriptions.size();
    }

    int RobotQtViewerEventHub::subscriptionCount(RobotQtViewerEventKind kind) const
    {
        int count = 0;
        for(const SubscriptionEntry& entry : m_subscriptions) {
            if(entry.kind == kind && !entry.receiver.isNull()) {
                ++count;
            }
        }
        return count;
    }
}
