#pragma once

#include "RobotQtViewerEvents.h"

#include <QVector>
#include <QString>

class QJsonObject;

namespace robot_qt_viewer
{
    enum class RobotQtViewerDockArea
    {
        Left,
        Right,
        Top,
        Bottom,
        Central,
        Floating
    };

    enum class RobotQtViewerRefreshPolicy
    {
        Immediate,
        Deferred,
        Manual
    };

    struct RobotQtViewerSubscriptionDesc
    {
        QString subscriberId;
        RobotQtViewerEventKind eventKind = RobotQtViewerEventKind::ProjectDocumentChanged;
        RobotQtViewerRefreshPolicy refreshPolicy = RobotQtViewerRefreshPolicy::Immediate;
    };

    struct RobotQtViewerModuleDesc
    {
        QString id;
        QString type;
        RobotQtViewerDockArea dockArea = RobotQtViewerDockArea::Right;
        bool enabled = true;
        QVector<RobotQtViewerSubscriptionDesc> subscriptions;
    };

    struct RobotQtViewerWindowConfig
    {
        QVector<RobotQtViewerModuleDesc> modules;
    };

    QString robotQtViewerDockAreaName(RobotQtViewerDockArea area);
    bool robotQtViewerDockAreaFromName(const QString& name, RobotQtViewerDockArea& area);
    QString robotQtViewerRefreshPolicyName(RobotQtViewerRefreshPolicy policy);
    bool robotQtViewerRefreshPolicyFromName(const QString& name, RobotQtViewerRefreshPolicy& policy);

    RobotQtViewerWindowConfig defaultRobotQtViewerWindowConfig();
    bool robotQtViewerWindowConfigFromJson(
        const QJsonObject& json,
        RobotQtViewerWindowConfig& config,
        QString* errorMessage = nullptr);
}

