#pragma once

#include "RobotQtViewerEvents.h"
#include "RobotQtViewerWindowConfig.h"

#include <QVector>
#include <QString>
#include <QObject>
#include <QPointer>

#include <functional>

namespace robot_qt_viewer
{
    class RobotQtViewerEventHub : public QObject
    {
    public:
        using EventHandler = std::function<void(const RobotQtViewerEvent&)>;

        explicit RobotQtViewerEventHub(QObject* parent = nullptr);

        void publish(const RobotQtViewerEvent& event);
        void subscribe(
            RobotQtViewerEventKind kind,
            QObject* receiver,
            EventHandler handler,
            const QString& subscriberId = QString());
        void subscribeModule(
            const RobotQtViewerWindowConfig& config,
            const QString& subscriberId,
            QObject* receiver,
            EventHandler handler);
        void unsubscribe(QObject* receiver);

        int subscriptionCount() const;
        int subscriptionCount(RobotQtViewerEventKind kind) const;

    private:
        struct SubscriptionEntry
        {
            RobotQtViewerEventKind kind = RobotQtViewerEventKind::ProjectDocumentChanged;
            QString subscriberId;
            QPointer<QObject> receiver;
            EventHandler handler;
        };

        QVector<SubscriptionEntry> m_subscriptions;
    };
}

