#pragma once

#include "RobotQtViewerEvents.h"

#include <QObject>
#include <QHash>
#include <QString>
#include <QStringList>

class MotionControlWidget;
class CollisionResultsWidget;

namespace robotruntime
{
    class IRobotRunService;
}

namespace robot_qt_viewer
{
    class CollisionResultDocumentFacade;
    class RobotQtViewerDocumentContext;

    class MotionControlModuleController : public QObject
    {
        Q_OBJECT

    public:
        MotionControlModuleController(
            MotionControlWidget& widget,
            RobotQtViewerDocumentContext& context,
            robotruntime::IRobotRunService& runService,
            QObject* parent = nullptr);

        bool isUpdating() const;
        QStringList movableJoints(const QString& robotId) const;
        void setAutoMotionChecked(bool checked);
        void setRobotRuntime(
            const QString& robotId,
            const QStringList& movableJoints,
            const QStringList& movableJointTypes);
        void setCollisionDetailsWidget(CollisionResultsWidget* widget);
        void clearRuntime();
        void selectRobot(const QString& robotId);
        void updateRowsFromScene();
        void refreshCollisionMonitor();
        void refreshTrajectories();
        void handleRobotStateUpdated();
        void handleEvent(const RobotQtViewerEvent& event);

    signals:
        void statusMessageRequested(const QString& message, int timeoutMs);
        void collisionQueriesEnabledChanged(bool enabled);
        void collisionGeometryVisibleChanged(bool visible);

    private:
        void handleJointDisplayValueChanged(
            const QString& jointName,
            const QString& jointType,
            double displayValue);
        void handleAutoMotionChanged();
        void applyCurrentPoseAsInitial();
        void handleCollisionDetectorSelectionChanged(const QString& detectorId);
        void handleCollisionMonitoringChanged(bool enabled);
        void handleCollisionGeometryVisibilityChanged(bool visible);
        void loadTrajectory(const QString& trajectoryId);
        void startTrajectory();
        void pauseTrajectory();
        void stopTrajectory();
        void stepTrajectory();
        void updateTrajectoryStatus();
        void refreshCollisionResults(const QString& detectorId);
        QString preferredCollisionDetectorId() const;

        MotionControlWidget& m_widget;
        RobotQtViewerDocumentContext& m_context;
        robotruntime::IRobotRunService& m_runService;
        CollisionResultsWidget* m_collisionDetailsWidget = nullptr;
        QString m_selectedRobotId;
        QString m_selectedCollisionDetectorId;
        QHash<QString, QStringList> m_robotMovableJoints;
        QHash<QString, QStringList> m_robotMovableJointTypes;
        bool m_updating = false;
    };
}
