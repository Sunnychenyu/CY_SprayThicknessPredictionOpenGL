#include "MotionPlanningModuleController.h"

#include "MotionPlanningEditorWidget.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerSelectionModel.h"

#include <MotionPlanningCore/MotionPlanning.h>
#include <SimulationProject/ProjectDocumentService.h>

#include <QStringList>

#include <algorithm>
#include <string>
#include <vector>

namespace
{
    bool parseJointVector(
        const QString& text,
        std::vector<double>& values,
        QString& error)
    {
        values.clear();
        const QStringList parts = text.split(',', Qt::SkipEmptyParts);
        for(const QString& part : parts) {
            bool ok = false;
            const double value = part.trimmed().toDouble(&ok);
            if(!ok) {
                error = QStringLiteral("Invalid joint value: %1").arg(part.trimmed());
                return false;
            }
            values.push_back(value);
        }
        if(values.empty()) {
            error = QStringLiteral("Enter at least one joint value.");
            return false;
        }
        return true;
    }
}

namespace robot_qt_viewer
{
    MotionPlanningModuleController::MotionPlanningModuleController(
        MotionPlanningEditorWidget& widget,
        RobotQtViewerDocumentContext& context,
        QObject* parent)
        : QObject(parent)
        , m_widget(widget)
        , m_context(context)
    {
        connect(&m_widget, &MotionPlanningEditorWidget::planRequested,
            this, &MotionPlanningModuleController::planTrajectory);
        setSelectedRobot(m_context.selectionModel().state().robotId);
    }

    void MotionPlanningModuleController::handleEvent(const RobotQtViewerEvent& event)
    {
        if(event.kind == RobotQtViewerEventKind::SelectionChanged) {
            setSelectedRobot(event.selection.robotId);
        } else if(event.kind == RobotQtViewerEventKind::ProjectOpened) {
            setSelectedRobot(m_context.selectionModel().state().robotId);
        }
    }

    void MotionPlanningModuleController::planTrajectory(
        const QString& startJoints,
        const QString& goalJoints,
        const QString& jointNames,
        double duration,
        int sampleCount)
    {
        if(m_selectedRobotId.isEmpty()) {
            m_widget.setResult(QStringLiteral("Select a robot before planning."), false);
            return;
        }

        motion_planning::MotionPlanningRequest request;
        request.robotId = m_selectedRobotId.toStdString();
        request.constraint.duration = duration;
        request.constraint.sampleCount = static_cast<std::size_t>(sampleCount);
        const QStringList jointNameParts = jointNames.split(',', Qt::SkipEmptyParts);
        for(const QString& jointName : jointNameParts) {
            request.jointNames.push_back(jointName.trimmed().toStdString());
        }

        QString parseError;
        if(!parseJointVector(startJoints, request.startJoints, parseError) ||
            !parseJointVector(goalJoints, request.goalJoints, parseError)) {
            m_widget.setResult(parseError, false);
            return;
        }

        const motion_planning::LinearJointMotionPlanner planner;
        const motion_planning::MotionPlanningResult planningResult = planner.plan(request);
        if(!planningResult.succeeded()) {
            const QString message = planningResult.diagnostics.empty()
                ? QStringLiteral("Motion planning failed.")
                : QString::fromStdString(planningResult.diagnostics.front().message);
            m_widget.setResult(message, false);
            emit statusMessageRequested(message, 5000);
            return;
        }

        motion_planning::StoredMotionPlan plan;
        plan.id = request.robotId + "_linear_plan";
        plan.name = plan.id;
        plan.robotId = request.robotId;
        plan.jointNames = request.jointNames;
        plan.trajectory = planningResult.trajectory;

        const ProjectMutationResult mutation = m_context.documentController().mutateProject(
            QStringLiteral("motionPlanning"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(!motion_planning::MotionPlanningProjectStore::upsertPlan(
                       service.document(), plan, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutation.success) {
            m_widget.setResult(mutation.message, false);
            emit statusMessageRequested(mutation.message, 5000);
            return;
        }

        const QString planId = QString::fromStdString(plan.id);
        const QString summary = QStringLiteral("Stored %1 points as %2")
            .arg(static_cast<int>(plan.trajectory.points.size()))
            .arg(planId);
        m_widget.setResult(summary, true);
        emit trajectoryPlanned(planId);
        emit statusMessageRequested(summary, 4000);
    }

    void MotionPlanningModuleController::setSelectedRobot(const QString& robotId)
    {
        m_selectedRobotId = robotId;
        m_widget.setRobotId(robotId);
        if(robotId.isEmpty()) {
            return;
        }
        const simulation_project::ProjectDocument& document = m_context.document();
        const auto robotIt = std::find_if(
            document.robots.begin(),
            document.robots.end(),
            [&](const simulation_project::RobotDesc& robot) {
                return robot.id == robotId.toStdString();
            });
        if(robotIt == document.robots.end() || robotIt->initialJoints.empty()) {
            return;
        }
        QStringList names;
        QStringList values;
        for(const simulation_project::JointValueDesc& joint : robotIt->initialJoints) {
            names.push_back(QString::fromStdString(joint.jointName));
            values.push_back(QString::number(joint.value, 'g', 12));
        }
        m_widget.setJointDefaults(names.join(QStringLiteral(", ")), values.join(QStringLiteral(", ")));
    }
}
