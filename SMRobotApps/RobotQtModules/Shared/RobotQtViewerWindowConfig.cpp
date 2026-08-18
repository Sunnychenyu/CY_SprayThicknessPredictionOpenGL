#include "RobotQtViewerWindowConfig.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QJsonValue>

#include <utility>

namespace
{
    void setError(QString* errorMessage, const QString& message)
    {
        if(errorMessage != nullptr) {
            *errorMessage = message;
        }
    }

    robot_qt_viewer::RobotQtViewerSubscriptionDesc makeSubscription(
        const QString& subscriberId,
        robot_qt_viewer::RobotQtViewerEventKind eventKind,
        robot_qt_viewer::RobotQtViewerRefreshPolicy policy =
            robot_qt_viewer::RobotQtViewerRefreshPolicy::Immediate)
    {
        robot_qt_viewer::RobotQtViewerSubscriptionDesc subscription;
        subscription.subscriberId = subscriberId;
        subscription.eventKind = eventKind;
        subscription.refreshPolicy = policy;
        return subscription;
    }

    robot_qt_viewer::RobotQtViewerModuleDesc makeModule(
        const QString& id,
        const QString& type,
        robot_qt_viewer::RobotQtViewerDockArea dockArea,
        QVector<robot_qt_viewer::RobotQtViewerSubscriptionDesc> subscriptions)
    {
        robot_qt_viewer::RobotQtViewerModuleDesc module;
        module.id = id;
        module.type = type;
        module.dockArea = dockArea;
        module.subscriptions = std::move(subscriptions);
        return module;
    }
}

namespace robot_qt_viewer
{
    QString robotQtViewerDockAreaName(RobotQtViewerDockArea area)
    {
        switch(area) {
        case RobotQtViewerDockArea::Left:
            return QStringLiteral("left");
        case RobotQtViewerDockArea::Right:
            return QStringLiteral("right");
        case RobotQtViewerDockArea::Top:
            return QStringLiteral("top");
        case RobotQtViewerDockArea::Bottom:
            return QStringLiteral("bottom");
        case RobotQtViewerDockArea::Central:
            return QStringLiteral("central");
        case RobotQtViewerDockArea::Floating:
            return QStringLiteral("floating");
        }
        return QStringLiteral("right");
    }

    bool robotQtViewerDockAreaFromName(const QString& name, RobotQtViewerDockArea& area)
    {
        const QString normalized = name.trimmed().toLower();
        if(normalized == QStringLiteral("left")) {
            area = RobotQtViewerDockArea::Left;
            return true;
        }
        if(normalized == QStringLiteral("right")) {
            area = RobotQtViewerDockArea::Right;
            return true;
        }
        if(normalized == QStringLiteral("top")) {
            area = RobotQtViewerDockArea::Top;
            return true;
        }
        if(normalized == QStringLiteral("bottom")) {
            area = RobotQtViewerDockArea::Bottom;
            return true;
        }
        if(normalized == QStringLiteral("central")) {
            area = RobotQtViewerDockArea::Central;
            return true;
        }
        if(normalized == QStringLiteral("floating")) {
            area = RobotQtViewerDockArea::Floating;
            return true;
        }
        return false;
    }

    QString robotQtViewerRefreshPolicyName(RobotQtViewerRefreshPolicy policy)
    {
        switch(policy) {
        case RobotQtViewerRefreshPolicy::Immediate:
            return QStringLiteral("immediate");
        case RobotQtViewerRefreshPolicy::Deferred:
            return QStringLiteral("deferred");
        case RobotQtViewerRefreshPolicy::Manual:
            return QStringLiteral("manual");
        }
        return QStringLiteral("immediate");
    }

    bool robotQtViewerRefreshPolicyFromName(const QString& name, RobotQtViewerRefreshPolicy& policy)
    {
        const QString normalized = name.trimmed().toLower();
        if(normalized == QStringLiteral("immediate")) {
            policy = RobotQtViewerRefreshPolicy::Immediate;
            return true;
        }
        if(normalized == QStringLiteral("deferred")) {
            policy = RobotQtViewerRefreshPolicy::Deferred;
            return true;
        }
        if(normalized == QStringLiteral("manual")) {
            policy = RobotQtViewerRefreshPolicy::Manual;
            return true;
        }
        return false;
    }

    RobotQtViewerWindowConfig defaultRobotQtViewerWindowConfig()
    {
        RobotQtViewerWindowConfig config;

        config.modules.push_back(makeModule(
            QStringLiteral("viewport"),
            QStringLiteral("Viewport"),
            RobotQtViewerDockArea::Central,
            {
                makeSubscription(QStringLiteral("viewport"), RobotQtViewerEventKind::SelectionChanged),
                makeSubscription(QStringLiteral("viewport"), RobotQtViewerEventKind::ViewportPreviewChanged),
                makeSubscription(QStringLiteral("viewport"), RobotQtViewerEventKind::ViewportReloaded)
            }));

        config.modules.push_back(makeModule(
            QStringLiteral("sceneExplorer"),
            QStringLiteral("SceneExplorer"),
            RobotQtViewerDockArea::Left,
            {
                makeSubscription(QStringLiteral("sceneExplorer"), RobotQtViewerEventKind::ProjectDocumentChanged),
                makeSubscription(QStringLiteral("sceneExplorer"), RobotQtViewerEventKind::ViewportReloaded),
                makeSubscription(QStringLiteral("sceneExplorer"), RobotQtViewerEventKind::RobotRuntimeChanged),
                makeSubscription(QStringLiteral("sceneExplorer"), RobotQtViewerEventKind::AttachmentChanged),
                makeSubscription(QStringLiteral("sceneExplorer"), RobotQtViewerEventKind::SelectionChanged),
                makeSubscription(QStringLiteral("sceneExplorer"), RobotQtViewerEventKind::TaskStateChanged)
            }));

        config.modules.push_back(makeModule(
            QStringLiteral("motion"),
            QStringLiteral("MotionControl"),
            RobotQtViewerDockArea::Right,
            {
                makeSubscription(QStringLiteral("motion"), RobotQtViewerEventKind::SelectionChanged),
                makeSubscription(QStringLiteral("motion"), RobotQtViewerEventKind::RobotRuntimeChanged),
                makeSubscription(QStringLiteral("motion"), RobotQtViewerEventKind::ProjectDocumentChanged),
                makeSubscription(QStringLiteral("motion"), RobotQtViewerEventKind::CollisionChanged),
                makeSubscription(QStringLiteral("motion"), RobotQtViewerEventKind::ViewportReloaded)
            }));

        config.modules.push_back(makeModule(
            QStringLiteral("toolSetup"),
            QStringLiteral("ToolSetup"),
            RobotQtViewerDockArea::Right,
            {
                makeSubscription(QStringLiteral("toolSetup"), RobotQtViewerEventKind::SelectionChanged),
                makeSubscription(QStringLiteral("toolSetup"), RobotQtViewerEventKind::AttachmentChanged),
                makeSubscription(QStringLiteral("toolSetup"), RobotQtViewerEventKind::ProjectDocumentChanged),
                makeSubscription(QStringLiteral("toolSetup"), RobotQtViewerEventKind::ViewportReloaded)
            }));

        config.modules.push_back(makeModule(
            QStringLiteral("collisionWorkbench"),
            QStringLiteral("CollisionWorkbench"),
            RobotQtViewerDockArea::Right,
            {
                makeSubscription(QStringLiteral("collisionWorkbench"), RobotQtViewerEventKind::SelectionChanged),
                makeSubscription(QStringLiteral("collisionWorkbench"), RobotQtViewerEventKind::ProjectDocumentChanged),
                makeSubscription(QStringLiteral("collisionWorkbench"), RobotQtViewerEventKind::CollisionChanged),
                makeSubscription(QStringLiteral("collisionWorkbench"), RobotQtViewerEventKind::ViewportReloaded),
                makeSubscription(QStringLiteral("collisionWorkbench"), RobotQtViewerEventKind::RobotRuntimeChanged)
            }));

        config.modules.push_back(makeModule(
            QStringLiteral("coatingAnalysis"),
            QStringLiteral("CoatingAnalysis"),
            RobotQtViewerDockArea::Right,
            {
                makeSubscription(QStringLiteral("coatingAnalysis"), RobotQtViewerEventKind::ProjectOpened),
                makeSubscription(QStringLiteral("coatingAnalysis"), RobotQtViewerEventKind::ProjectDocumentChanged),
                makeSubscription(QStringLiteral("coatingAnalysis"), RobotQtViewerEventKind::ViewportReloaded),
                makeSubscription(QStringLiteral("coatingAnalysis"), RobotQtViewerEventKind::SelectionChanged),
                makeSubscription(QStringLiteral("coatingAnalysis"), RobotQtViewerEventKind::CoatingAnalysisChanged)
            }));

        config.modules.push_back(makeModule(
            QStringLiteral("status"),
            QStringLiteral("StatusPanel"),
            RobotQtViewerDockArea::Right,
            {
                makeSubscription(QStringLiteral("status"), RobotQtViewerEventKind::ProjectOpened),
                makeSubscription(QStringLiteral("status"), RobotQtViewerEventKind::ProjectSaved),
                makeSubscription(QStringLiteral("status"), RobotQtViewerEventKind::StatusMessageRequested)
            }));

        return config;
    }

    bool robotQtViewerWindowConfigFromJson(
        const QJsonObject& json,
        RobotQtViewerWindowConfig& config,
        QString* errorMessage)
    {
        RobotQtViewerWindowConfig parsed;
        const QJsonValue modulesValue = json.value(QStringLiteral("modules"));
        if(!modulesValue.isArray()) {
            setError(errorMessage, QStringLiteral("RobotQtViewer window config requires a modules array."));
            return false;
        }

        const QJsonArray modules = modulesValue.toArray();
        for(const QJsonValue& moduleValue : modules) {
            if(!moduleValue.isObject()) {
                setError(errorMessage, QStringLiteral("RobotQtViewer module entry must be an object."));
                return false;
            }

            const QJsonObject moduleObject = moduleValue.toObject();
            RobotQtViewerModuleDesc module;
            module.id = moduleObject.value(QStringLiteral("id")).toString();
            module.type = moduleObject.value(QStringLiteral("type")).toString();
            module.enabled = moduleObject.value(QStringLiteral("enabled")).toBool(true);
            if(module.id.isEmpty() || module.type.isEmpty()) {
                setError(errorMessage, QStringLiteral("RobotQtViewer module id and type are required."));
                return false;
            }

            const QString dockName = moduleObject.value(QStringLiteral("dock")).toString(QStringLiteral("right"));
            if(!robotQtViewerDockAreaFromName(dockName, module.dockArea)) {
                setError(errorMessage, QStringLiteral("Unknown RobotQtViewer dock area: %1").arg(dockName));
                return false;
            }

            const QJsonValue subscriptionsValue = moduleObject.value(QStringLiteral("subscriptions"));
            if(subscriptionsValue.isArray()) {
                const QJsonArray subscriptions = subscriptionsValue.toArray();
                for(const QJsonValue& subscriptionValue : subscriptions) {
                    RobotQtViewerEventKind eventKind = RobotQtViewerEventKind::ProjectDocumentChanged;
                    RobotQtViewerRefreshPolicy policy = RobotQtViewerRefreshPolicy::Immediate;
                    QString eventName;
                    if(subscriptionValue.isString()) {
                        eventName = subscriptionValue.toString();
                    } else if(subscriptionValue.isObject()) {
                        const QJsonObject subscriptionObject = subscriptionValue.toObject();
                        eventName = subscriptionObject.value(QStringLiteral("event")).toString();
                        const QString policyName =
                            subscriptionObject.value(QStringLiteral("refresh")).toString(QStringLiteral("immediate"));
                        if(!robotQtViewerRefreshPolicyFromName(policyName, policy)) {
                            setError(errorMessage, QStringLiteral("Unknown RobotQtViewer refresh policy: %1").arg(policyName));
                            return false;
                        }
                    } else {
                        setError(errorMessage, QStringLiteral("RobotQtViewer subscription must be a string or object."));
                        return false;
                    }

                    if(!robotQtViewerEventKindFromName(eventName, eventKind)) {
                        setError(errorMessage, QStringLiteral("Unknown RobotQtViewer event kind: %1").arg(eventName));
                        return false;
                    }

                    module.subscriptions.push_back(makeSubscription(module.id, eventKind, policy));
                }
            }
            parsed.modules.push_back(std::move(module));
        }

        config = std::move(parsed);
        return true;
    }
}
