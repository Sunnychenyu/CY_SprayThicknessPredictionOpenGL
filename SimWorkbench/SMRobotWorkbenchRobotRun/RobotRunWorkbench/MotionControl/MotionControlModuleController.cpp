#include "MotionControlModuleController.h"

#include "CollisionResultDocumentFacade.h"
#include "CollisionResultsWidget.h"
#include "MotionControlWidget.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerViewportServices.h"

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectDocumentService.h>
#include <RobotRuntime/RobotRunService.h>
#include <MotionPlanningCore/MotionPlanning.h>

#include <algorithm>
#include <cstddef>
#include <vector>

namespace
{
    constexpr double kPi = 3.14159265358979323846;

    bool isRevoluteJointType(const QString& jointType)
    {
        return jointType.compare(QStringLiteral("revolute"), Qt::CaseInsensitive) == 0 ||
            jointType.compare(QStringLiteral("continuous"), Qt::CaseInsensitive) == 0;
    }

    double toDegrees(double radians)
    {
        return radians * 180.0 / kPi;
    }

    double toRadians(double degrees)
    {
        return degrees * kPi / 180.0;
    }

    simulation_project::JointValueDesc makeJointValue(const QString& jointName, double value)
    {
        simulation_project::JointValueDesc jointValue;
        jointValue.jointName = jointName.toStdString();
        jointValue.value = value;
        return jointValue;
    }

    bool sameJointValues(
        const std::vector<simulation_project::JointValueDesc>& lhs,
        const std::vector<simulation_project::JointValueDesc>& rhs)
    {
        if(lhs.size() != rhs.size()) {
            return false;
        }
        for(std::size_t index = 0; index < lhs.size(); ++index) {
            if(lhs[index].jointName != rhs[index].jointName ||
                lhs[index].value != rhs[index].value) {
                return false;
            }
        }
        return true;
    }
}

namespace robot_qt_viewer
{
    MotionControlModuleController::MotionControlModuleController(
        MotionControlWidget& widget,
        RobotQtViewerDocumentContext& context,
        robotruntime::IRobotRunService& runService,
        QObject* parent)
        : QObject(parent)
        , m_widget(widget)
        , m_context(context)
        , m_runService(runService)
    {
        connect(&m_widget, &MotionControlWidget::jointDisplayValueChanged,
            this, &MotionControlModuleController::handleJointDisplayValueChanged);
        connect(&m_widget, &MotionControlWidget::robotSelectionChanged,
            this, &MotionControlModuleController::handleRobotSelectionChanged);
        connect(&m_widget, &MotionControlWidget::autoMotionChanged,
            this, &MotionControlModuleController::handleAutoMotionChanged);
        connect(&m_widget, &MotionControlWidget::applyInitialPoseRequested,
            this, &MotionControlModuleController::applyCurrentPoseAsInitial);
        connect(&m_widget, &MotionControlWidget::collisionDetectorSelectionChanged,
            this, &MotionControlModuleController::handleCollisionDetectorSelectionChanged);
        connect(&m_widget, &MotionControlWidget::collisionMonitoringChanged,
            this, &MotionControlModuleController::handleCollisionMonitoringChanged);
        connect(&m_widget, &MotionControlWidget::collisionGeometryVisibilityChanged,
            this, &MotionControlModuleController::handleCollisionGeometryVisibilityChanged);
        connect(&m_widget, &MotionControlWidget::trajectoryLoadRequested,
            this, &MotionControlModuleController::loadTrajectory);
        connect(&m_widget, &MotionControlWidget::trajectoryStartRequested,
            this, &MotionControlModuleController::startTrajectory);
        connect(&m_widget, &MotionControlWidget::trajectoryPauseRequested,
            this, &MotionControlModuleController::pauseTrajectory);
        connect(&m_widget, &MotionControlWidget::trajectoryStopRequested,
            this, &MotionControlModuleController::stopTrajectory);
        connect(&m_widget, &MotionControlWidget::trajectoryStepRequested,
            this, &MotionControlModuleController::stepTrajectory);
    }

    bool MotionControlModuleController::isUpdating() const
    {
        return m_updating;
    }

    QStringList MotionControlModuleController::movableJoints(const QString& robotId) const
    {
        return m_robotMovableJoints.value(robotId);
    }

    void MotionControlModuleController::setAutoMotionChecked(bool checked)
    {
        if(!m_selectedRobotId.isEmpty()) {
            AutoMotionState state = m_autoMotionStates.value(m_selectedRobotId);
            state.enabled = checked;
            m_autoMotionStates[m_selectedRobotId] = state;
        }
        m_widget.setAutoMotionChecked(checked);
    }

    void MotionControlModuleController::setRobotRuntime(
        const QString& robotId,
        const QStringList& movableJoints,
        const QStringList& movableJointTypes)
    {
        m_robotMovableJoints[robotId] = movableJoints;
        m_robotMovableJointTypes[robotId] = movableJointTypes;
        if(!m_autoMotionStates.contains(robotId)) {
            m_autoMotionStates.insert(robotId, AutoMotionState{});
        }
        refreshRobotSelection();
        const QString targetRobotId = m_selectedRobotId.isEmpty()
            ? firstAvailableRobotId()
            : m_selectedRobotId;
        if(!targetRobotId.isEmpty()) {
            selectRobot(targetRobotId);
        }
    }

    void MotionControlModuleController::setCollisionDetailsWidget(CollisionResultsWidget* widget)
    {
        m_collisionDetailsWidget = widget;
        refreshCollisionResults(m_widget.currentCollisionDetectorId());
    }

    void MotionControlModuleController::clearRuntime()
    {
        stopAllAutoMotion();
        m_robotMovableJoints.clear();
        m_robotMovableJointTypes.clear();
        m_autoMotionStates.clear();
        refreshRobotSelection();
        selectRobot(QString());
    }

    void MotionControlModuleController::stopAllAutoMotion()
    {
        for(auto it = m_autoMotionStates.begin(); it != m_autoMotionStates.end(); ++it) {
            if(it.value().enabled) {
                m_runService.setAutoMotion(it.key().toStdString(), false, 0.0, 0.0);
                it.value().enabled = false;
            }
        }
        if(!m_selectedRobotId.isEmpty()) {
            const AutoMotionState state = m_autoMotionStates.value(m_selectedRobotId);
            m_widget.setAutoMotionControls(false, state.amplitude, state.speed);
        }
    }

    void MotionControlModuleController::selectRobot(const QString& robotId)
    {
        m_selectedRobotId = m_robotMovableJoints.contains(robotId) ? robotId : QString();
        m_updating = true;
        m_widget.setRobotId(m_selectedRobotId);
        m_widget.setJoints(
            m_robotMovableJoints.value(m_selectedRobotId),
            m_robotMovableJointTypes.value(m_selectedRobotId));
        const AutoMotionState state = m_autoMotionStates.value(m_selectedRobotId);
        m_widget.setAutoMotionControls(state.enabled, state.amplitude, state.speed);
        m_updating = false;
        updateRowsFromScene();
        refreshTrajectories();
    }

    void MotionControlModuleController::updateRowsFromScene()
    {
        const bool hasRobot = !m_selectedRobotId.isEmpty();
        const QStringList joints = m_robotMovableJoints.value(m_selectedRobotId);
        const QStringList types = m_robotMovableJointTypes.value(m_selectedRobotId);
        const bool hasJoints = hasRobot && !joints.empty();
        m_updating = true;

        for(int i = 0; i < joints.size(); ++i) {
            const QString jointName = joints.at(i);
            const QString jointType = i < types.size() ? types.at(i) : QString();
            bool valueOk = false;
            double value = 0.0;
            if(hasRobot) {
                valueOk = m_runService.jointValue(
                    m_selectedRobotId.toStdString(),
                    jointName.toStdString(),
                    value);
            }

            const double displayValue = isRevoluteJointType(jointType) ? toDegrees(value) : value;
            m_widget.setJointDisplayValue(jointName, displayValue, valueOk);
        }

        m_widget.setMotionActionsEnabled(hasJoints);
        m_updating = false;
    }

    void MotionControlModuleController::refreshCollisionMonitor()
    {
        RobotQtViewerViewportServices* viewportServices = m_context.viewportServices();
        const std::vector<CollisionRuntimeDetectorInfo> runtimeDetectors =
            viewportServices != nullptr
                ? viewportServices->collisionRuntimeDetectors()
                : std::vector<CollisionRuntimeDetectorInfo>();
        const bool collisionQueriesEnabled =
            viewportServices != nullptr && viewportServices->collisionQueriesEnabled();

        QVector<MotionControlWidget::CollisionDetectorItem> detectorItems;
        detectorItems.reserve(static_cast<int>(runtimeDetectors.size()));
        for(const CollisionRuntimeDetectorInfo& detector : runtimeDetectors) {
            MotionControlWidget::CollisionDetectorItem item;
            item.id = QString::fromStdString(detector.id);
            item.label = QString::fromStdString(detector.name.empty() ? detector.id : detector.name);
            item.enabled = detector.enabled && collisionQueriesEnabled;
            item.active = detector.active;
            detectorItems.push_back(item);
        }

        const QString preferredId = preferredCollisionDetectorId();
        m_updating = true;
        m_widget.setCollisionDetectors(detectorItems, preferredId);
        m_updating = false;

        const QString detectorId = m_widget.currentCollisionDetectorId();
        if(m_selectedCollisionDetectorId.isEmpty()) {
            m_selectedCollisionDetectorId = detectorId;
        }
        refreshCollisionResults(detectorId);
    }

    void MotionControlModuleController::refreshTrajectories()
    {
        QVector<MotionControlWidget::TrajectoryItem> items;
        for(const motion_planning::StoredMotionPlan& plan :
            motion_planning::MotionPlanningProjectStore::plans(m_context.document())) {
            if(!m_selectedRobotId.isEmpty() && plan.robotId != m_selectedRobotId.toStdString()) {
                continue;
            }
            MotionControlWidget::TrajectoryItem item;
            item.id = QString::fromStdString(plan.id);
            item.label = QString::fromStdString(plan.name.empty() ? plan.id : plan.name);
            items.push_back(item);
        }
        m_widget.setTrajectories(items);
        updateTrajectoryStatus();
    }

    void MotionControlModuleController::handleRobotStateUpdated()
    {
        if(m_autoMotionStates.value(m_selectedRobotId).enabled) {
            updateRowsFromScene();
        }
        refreshCollisionResults(m_widget.currentCollisionDetectorId());
    }

    void MotionControlModuleController::handleEvent(const RobotQtViewerEvent& event)
    {
        switch(event.kind) {
        case RobotQtViewerEventKind::SelectionChanged:
            break;
        case RobotQtViewerEventKind::RobotRuntimeChanged:
            updateRowsFromScene();
            refreshCollisionMonitor();
            break;
        case RobotQtViewerEventKind::ProjectDocumentChanged:
            refreshTrajectories();
            refreshCollisionMonitor();
            break;
        case RobotQtViewerEventKind::CollisionChanged:
        case RobotQtViewerEventKind::ViewportReloaded:
            refreshCollisionMonitor();
            break;
        default:
            break;
        }
    }

    void MotionControlModuleController::handleRobotSelectionChanged(const QString& robotId)
    {
        if(m_updating) {
            return;
        }
        selectRobot(robotId);
    }

    void MotionControlModuleController::loadTrajectory(const QString& trajectoryId)
    {
        if(trajectoryId.isEmpty()) {
            return;
        }
        const std::vector<motion_planning::StoredMotionPlan> plans =
            motion_planning::MotionPlanningProjectStore::plans(m_context.document());
        const auto planIt = std::find_if(
            plans.begin(),
            plans.end(),
            [&](const motion_planning::StoredMotionPlan& plan) {
                return plan.id == trajectoryId.toStdString();
            });
        if(planIt == plans.end()) {
            emit statusMessageRequested(QStringLiteral("Motion plan was not found."), 4000);
            return;
        }
        const robotruntime::RobotRunCommandResult result = m_runService.loadTrajectory(
            planIt->robotId,
            planIt->id,
            planIt->jointNames,
            planIt->trajectory);
        m_widget.setTrajectoryStatus(QString::fromStdString(result.message));
        emit statusMessageRequested(QString::fromStdString(result.message), result.success ? 3000 : 5000);
    }

    void MotionControlModuleController::startTrajectory()
    {
        const robotruntime::RobotRunCommandResult result = m_runService.startTrajectory();
        m_widget.setTrajectoryStatus(QString::fromStdString(result.message));
        emit statusMessageRequested(QString::fromStdString(result.message), result.success ? 3000 : 5000);
    }

    void MotionControlModuleController::pauseTrajectory()
    {
        const robotruntime::RobotRunCommandResult result = m_runService.pauseTrajectory();
        m_widget.setTrajectoryStatus(QString::fromStdString(result.message));
        emit statusMessageRequested(QString::fromStdString(result.message), result.success ? 3000 : 5000);
    }

    void MotionControlModuleController::stopTrajectory()
    {
        const robotruntime::RobotRunCommandResult result = m_runService.stopTrajectory();
        m_widget.setTrajectoryStatus(QString::fromStdString(result.message));
        emit statusMessageRequested(QString::fromStdString(result.message), result.success ? 3000 : 5000);
        if(result.success) {
            updateRowsFromScene();
        }
    }

    void MotionControlModuleController::stepTrajectory()
    {
        const robotruntime::RobotRunCommandResult result = m_runService.stepTrajectory(0.1);
        m_widget.setTrajectoryStatus(QString::fromStdString(result.message));
        emit statusMessageRequested(QString::fromStdString(result.message), result.success ? 2000 : 5000);
        if(result.success) {
            updateRowsFromScene();
            m_context.documentController().publishRobotRuntimeChanged(QStringLiteral("trajectoryStep"));
        }
    }

    void MotionControlModuleController::updateTrajectoryStatus()
    {
        const robotruntime::RobotRunExecutionSnapshot snapshot = m_runService.trajectorySnapshot();
        if(snapshot.state == robotruntime::RobotRunExecutionState::Idle) {
            m_widget.setTrajectoryStatus(QStringLiteral("No trajectory loaded"));
            return;
        }
        m_widget.setTrajectoryStatus(
            QStringLiteral("%1: %2 / %3 s")
                .arg(QString::fromStdString(snapshot.trajectoryId))
                .arg(snapshot.time, 0, 'f', 2)
                .arg(snapshot.duration, 0, 'f', 2));
    }

    void MotionControlModuleController::handleJointDisplayValueChanged(
        const QString& jointName,
        const QString& jointType,
        double displayValue)
    {
        if(m_updating || m_selectedRobotId.isEmpty() || jointName.isEmpty()) {
            return;
        }

        const bool revolute = isRevoluteJointType(jointType);
        const double internalValue = revolute ? toRadians(displayValue) : displayValue;

        if(m_widget.autoMotionChecked()) {
            m_widget.setAutoMotionChecked(false);
            AutoMotionState state = m_autoMotionStates.value(m_selectedRobotId);
            state.enabled = false;
            m_autoMotionStates[m_selectedRobotId] = state;
            m_runService.setAutoMotion(
                m_selectedRobotId.toStdString(), false, 0.0, 0.0);
        }

        const robotruntime::RobotRunCommandResult result = m_runService.setJointValue(
            m_selectedRobotId.toStdString(),
            jointName.toStdString(),
            internalValue);
        if(!result.success) {
            emit statusMessageRequested(QString::fromStdString(result.message), 5000);
        }
    }

    void MotionControlModuleController::handleAutoMotionChanged()
    {
        if(m_updating || m_selectedRobotId.isEmpty()) {
            return;
        }

        const bool enabled = m_widget.autoMotionChecked();
        double amplitude = m_widget.autoAmplitude();
        const double speed = m_widget.autoSpeed();
        const AutoMotionState previousState = m_autoMotionStates.value(m_selectedRobotId);

        if(m_widget.hasRevoluteJoint()) {
            amplitude = toRadians(amplitude);
        }

        const robotruntime::RobotRunCommandResult result = m_runService.setAutoMotion(
            m_selectedRobotId.toStdString(), enabled, amplitude, speed);
        if(!result.success) {
            m_widget.setAutoMotionControls(
                previousState.enabled,
                previousState.amplitude,
                previousState.speed);
            emit statusMessageRequested(QString::fromStdString(result.message), 5000);
            return;
        }

        AutoMotionState state;
        state.enabled = enabled;
        state.amplitude = m_widget.autoAmplitude();
        state.speed = speed;
        m_autoMotionStates[m_selectedRobotId] = state;
    }

    void MotionControlModuleController::applyCurrentPoseAsInitial()
    {
        if(m_selectedRobotId.isEmpty()) {
            return;
        }

        const auto robotIt = std::find_if(
            m_context.document().robots.begin(),
            m_context.document().robots.end(),
            [&](const simulation_project::RobotDesc& robot) {
                return robot.id == m_selectedRobotId.toStdString();
            });
        if(robotIt == m_context.document().robots.end()) {
            emit statusMessageRequested("Selected robot is not in project.", 3000);
            return;
        }

        const QStringList movableJoints = m_robotMovableJoints.value(m_selectedRobotId);
        std::vector<simulation_project::JointValueDesc> updatedInitialJoints = robotIt->initialJoints;
        updatedInitialJoints.erase(
            std::remove_if(
                updatedInitialJoints.begin(),
                updatedInitialJoints.end(),
                [&](const simulation_project::JointValueDesc& joint) {
                    return movableJoints.contains(QString::fromStdString(joint.jointName));
                }),
            updatedInitialJoints.end());
        for(const QString& jointName : movableJoints) {
            bool ok = false;
            double value = 0.0;
            ok = m_runService.jointValue(
                m_selectedRobotId.toStdString(),
                jointName.toStdString(),
                value);
            if(ok) {
                updatedInitialJoints.push_back(makeJointValue(jointName, value));
            }
        }

        const std::string robotId = m_selectedRobotId.toStdString();
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("motionInitialPose"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                simulation_project::ProjectDocument& document = service.document();
                auto mutableRobotIt = std::find_if(
                    document.robots.begin(),
                    document.robots.end(),
                    [&](const simulation_project::RobotDesc& robot) {
                        return robot.id == robotId;
                    });
                if(mutableRobotIt == document.robots.end()) {
                    error = "Selected robot is not in project.";
                    return false;
                }
                if(sameJointValues(mutableRobotIt->initialJoints, updatedInitialJoints)) {
                    changed = false;
                    return true;
                }
                mutableRobotIt->initialJoints = updatedInitialJoints;
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            emit statusMessageRequested(mutationResult.message, 5000);
            return;
        }
        emit statusMessageRequested(QString("Applied current pose to %1").arg(m_selectedRobotId), 3000);
    }

    void MotionControlModuleController::handleCollisionDetectorSelectionChanged(const QString& detectorId)
    {
        if(m_updating || detectorId.isEmpty()) {
            return;
        }

        m_selectedCollisionDetectorId = detectorId;
        RobotQtViewerViewportServices* viewportServices = m_context.viewportServices();
        if(viewportServices != nullptr) {
            viewportServices->setActiveCollisionDetector(detectorId);
            if(m_widget.collisionMonitoringChecked()) {
                viewportServices->setCollisionDetectorEnabled(detectorId, true);
                viewportServices->setCollisionQueriesEnabled(true);
                emit collisionQueriesEnabledChanged(true);
            }
        }
        refreshCollisionMonitor();
    }

    void MotionControlModuleController::handleCollisionMonitoringChanged(bool enabled)
    {
        const QString detectorId = m_widget.currentCollisionDetectorId();
        if(detectorId.isEmpty()) {
            return;
        }

        m_selectedCollisionDetectorId = detectorId;
        RobotQtViewerViewportServices* viewportServices = m_context.viewportServices();
        if(viewportServices == nullptr) {
            return;
        }

        viewportServices->setActiveCollisionDetector(detectorId);
        viewportServices->setCollisionDetectorEnabled(detectorId, enabled);
        viewportServices->setCollisionQueriesEnabled(enabled);
        if(!enabled) {
            viewportServices->setCollisionGeometryVisible(false);
            m_widget.setCollisionGeometryChecked(false);
            emit collisionGeometryVisibleChanged(false);
        }
        emit collisionQueriesEnabledChanged(enabled);
        refreshCollisionMonitor();
    }

    void MotionControlModuleController::handleCollisionGeometryVisibilityChanged(bool visible)
    {
        RobotQtViewerViewportServices* viewportServices = m_context.viewportServices();
        if(viewportServices == nullptr) {
            return;
        }

        if(visible && !m_widget.collisionMonitoringChecked()) {
            m_widget.setCollisionGeometryChecked(false);
            emit statusMessageRequested(QStringLiteral("Enable collision detection before showing collision models."), 3000);
            return;
        }

        viewportServices->setCollisionGeometryVisible(visible);
        emit collisionGeometryVisibleChanged(visible);
    }

    void MotionControlModuleController::refreshRobotSelection()
    {
        QVector<MotionControlWidget::RobotItem> items;
        QStringList appendedIds;

        for(const simulation_project::RobotDesc& robot : m_context.document().robots) {
            const QString robotId = QString::fromStdString(robot.id);
            if(robotId.isEmpty() || !m_robotMovableJoints.contains(robotId)) {
                continue;
            }

            MotionControlWidget::RobotItem item;
            item.id = robotId;
            const QString robotName = QString::fromStdString(robot.name);
            item.label = robotName.isEmpty() || robotName == robotId
                ? robotId
                : QStringLiteral("%1 (%2)").arg(robotName, robotId);
            items.push_back(item);
            appendedIds.push_back(robotId);
        }

        QStringList remainingIds = m_robotMovableJoints.keys();
        remainingIds.erase(
            std::remove_if(
                remainingIds.begin(),
                remainingIds.end(),
                [&](const QString& robotId) {
                    return appendedIds.contains(robotId);
                }),
            remainingIds.end());
        std::sort(remainingIds.begin(), remainingIds.end());

        for(const QString& robotId : remainingIds) {
            MotionControlWidget::RobotItem item;
            item.id = robotId;
            item.label = robotId;
            items.push_back(item);
        }

        m_widget.setRobots(items, m_selectedRobotId);
    }

    QString MotionControlModuleController::firstAvailableRobotId() const
    {
        for(const simulation_project::RobotDesc& robot : m_context.document().robots) {
            const QString robotId = QString::fromStdString(robot.id);
            if(m_robotMovableJoints.contains(robotId)) {
                return robotId;
            }
        }

        QStringList ids = m_robotMovableJoints.keys();
        std::sort(ids.begin(), ids.end());
        return ids.isEmpty() ? QString() : ids.front();
    }

    void MotionControlModuleController::refreshCollisionResults(const QString& detectorId)
    {
        CollisionResultsWidget* resultsWidget = m_widget.collisionResultsWidget();
        if(resultsWidget == nullptr && m_collisionDetailsWidget == nullptr) {
            return;
        }

        std::vector<CollisionRuntimeDetectorInfo> runtimeDetectors;
        if(m_context.viewportServices() != nullptr) {
            runtimeDetectors = m_context.viewportServices()->collisionRuntimeDetectors();
        }

        CollisionResultDocumentFacade facade(m_context.document());
        const CollisionResultsViewModel viewModel = facade.buildViewModel(
            detectorId,
            m_context.viewportServices() != nullptr,
            runtimeDetectors,
            QString(),
            QString());
        if(resultsWidget != nullptr) {
            resultsWidget->setResults(viewModel);
        }
        if(m_collisionDetailsWidget != nullptr) {
            m_collisionDetailsWidget->setResults(viewModel);
        }
    }

    QString MotionControlModuleController::preferredCollisionDetectorId() const
    {
        if(!m_selectedCollisionDetectorId.isEmpty()) {
            return m_selectedCollisionDetectorId;
        }
        return m_widget.currentCollisionDetectorId();
    }
}
