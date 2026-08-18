#pragma once

#include "RobotQtViewerEvents.h"

#include <QObject>
#include <QString>

class MotionPlanningEditorWidget;

namespace robot_qt_viewer
{
    class RobotQtViewerDocumentContext;

    class MotionPlanningModuleController : public QObject
    {
        Q_OBJECT

    public:
        MotionPlanningModuleController(
            MotionPlanningEditorWidget& widget,
            RobotQtViewerDocumentContext& context,
            QObject* parent = nullptr);

        void handleEvent(const RobotQtViewerEvent& event);

    signals:
        void statusMessageRequested(const QString& message, int timeoutMs);
        void trajectoryPlanned(const QString& trajectoryId);

    private:
        void planTrajectory(
            const QString& startJoints,
            const QString& goalJoints,
            const QString& jointNames,
            double duration,
            int sampleCount);
        void setSelectedRobot(const QString& robotId);

        MotionPlanningEditorWidget& m_widget;
        RobotQtViewerDocumentContext& m_context;
        QString m_selectedRobotId;
    };
}
