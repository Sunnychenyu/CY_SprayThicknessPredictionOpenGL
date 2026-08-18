#include "MotionControlWidget.h"

#include "CollisionResultsWidget.h"
#include "RobotQtWidgetUtils.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFrame>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSlider>
#include <QStringList>
#include <QVBoxLayout>

MotionControlWidget::MotionControlWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* motionLayout = new QVBoxLayout(this);
    motionLayout->setContentsMargins(10, 10, 10, 10);
    motionLayout->setSpacing(8);

    motionLayout->addWidget(robot_qt_viewer::makePanelTitle("Joint Control", this));
    m_jointRobotLabel = new QLabel("No robot selected", this);
    robot_qt_viewer::makeHorizontallyCompressible(m_jointRobotLabel);
    motionLayout->addWidget(m_jointRobotLabel);

    auto* autoLayout = new QGridLayout();
    autoLayout->setContentsMargins(0, 0, 0, 0);
    autoLayout->setSpacing(8);
    robot_qt_viewer::configureInspectorGrid(autoLayout);

    m_autoMotionCheck = new QCheckBox("Auto Motion", this);
    robot_qt_viewer::makeHorizontallyCompressible(m_autoMotionCheck);
    connect(m_autoMotionCheck, &QCheckBox::toggled, this, [this]() {
        if(!m_updatingUi) {
            emit autoMotionChanged();
        }
    });
    autoLayout->addWidget(m_autoMotionCheck, 0, 0, 1, 2);

    m_applyInitialPoseButton = new QPushButton("Apply as Initial Pose", this);
    robot_qt_viewer::makeHorizontallyCompressible(m_applyInitialPoseButton);
    connect(m_applyInitialPoseButton, &QPushButton::clicked, this, &MotionControlWidget::applyInitialPoseRequested);
    autoLayout->addWidget(m_applyInitialPoseButton, 0, 2);

    m_autoAmplitudeSpin = new QDoubleSpinBox(this);
    robot_qt_viewer::makeHorizontallyCompressible(m_autoAmplitudeSpin);
    m_autoAmplitudeSpin->setMinimumWidth(0);
    m_autoAmplitudeSpin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    m_autoAmplitudeSpin->setRange(0.0, 360.0);
    m_autoAmplitudeSpin->setDecimals(3);
    m_autoAmplitudeSpin->setSingleStep(5.0);
    m_autoAmplitudeSpin->setValue(30.0);
    m_autoAmplitudeSpin->setPrefix("Amp ");
    m_autoAmplitudeSpin->setSuffix(" deg/raw");
    m_autoAmplitudeSpin->setToolTip("Auto motion amplitude.");
    connect(m_autoAmplitudeSpin, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [this]() {
        if(!m_updatingUi) {
            emit autoMotionChanged();
        }
    });
    autoLayout->addWidget(m_autoAmplitudeSpin, 1, 0, 1, 2);

    m_autoSpeedSpin = new QDoubleSpinBox(this);
    robot_qt_viewer::makeHorizontallyCompressible(m_autoSpeedSpin);
    m_autoSpeedSpin->setMinimumWidth(0);
    m_autoSpeedSpin->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    m_autoSpeedSpin->setRange(0.0, 20.0);
    m_autoSpeedSpin->setDecimals(3);
    m_autoSpeedSpin->setSingleStep(0.1);
    m_autoSpeedSpin->setValue(1.0);
    m_autoSpeedSpin->setPrefix("Speed ");
    m_autoSpeedSpin->setToolTip("Auto motion speed.");
    connect(m_autoSpeedSpin, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [this]() {
        if(!m_updatingUi) {
            emit autoMotionChanged();
        }
    });
    autoLayout->addWidget(m_autoSpeedSpin, 1, 2);
    autoLayout->setColumnStretch(0, 1);
    autoLayout->setColumnStretch(1, 1);
    autoLayout->setColumnStretch(2, 1);
    motionLayout->addLayout(autoLayout);

    m_jointScrollArea = new QScrollArea(this);
    robot_qt_viewer::makeHorizontallyCompressible(m_jointScrollArea);
    m_jointScrollArea->setWidgetResizable(true);
    m_jointScrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_jointRowsWidget = new QWidget(m_jointScrollArea);
    m_jointRowsLayout = new QVBoxLayout(m_jointRowsWidget);
    m_jointRowsLayout->setContentsMargins(8, 8, 8, 8);
    m_jointRowsLayout->setSpacing(6);
    m_jointRowsLayout->addStretch(1);
    m_jointRowsWidget->setLayout(m_jointRowsLayout);
    m_jointScrollArea->setWidget(m_jointRowsWidget);
    motionLayout->addWidget(m_jointScrollArea, 1);

    motionLayout->addWidget(robot_qt_viewer::makePanelTitle("Trajectory Execution", this));
    auto* trajectoryLayout = new QGridLayout();
    trajectoryLayout->setContentsMargins(0, 0, 0, 0);
    trajectoryLayout->setSpacing(6);
    m_trajectoryCombo = new QComboBox(this);
    trajectoryLayout->addWidget(m_trajectoryCombo, 0, 0, 1, 3);
    m_trajectoryLoadButton = new QPushButton("Load", this);
    m_trajectoryStartButton = new QPushButton("Start", this);
    m_trajectoryPauseButton = new QPushButton("Pause", this);
    m_trajectoryStopButton = new QPushButton("Stop", this);
    m_trajectoryStepButton = new QPushButton("Step", this);
    trajectoryLayout->addWidget(m_trajectoryLoadButton, 1, 0);
    trajectoryLayout->addWidget(m_trajectoryStartButton, 1, 1);
    trajectoryLayout->addWidget(m_trajectoryPauseButton, 1, 2);
    trajectoryLayout->addWidget(m_trajectoryStopButton, 2, 0);
    trajectoryLayout->addWidget(m_trajectoryStepButton, 2, 1);
    m_trajectoryStatus = new QLabel("No trajectory loaded", this);
    m_trajectoryStatus->setWordWrap(true);
    trajectoryLayout->addWidget(m_trajectoryStatus, 3, 0, 1, 3);
    motionLayout->addLayout(trajectoryLayout);

    connect(m_trajectoryLoadButton, &QPushButton::clicked, this, [this]() {
        emit trajectoryLoadRequested(currentTrajectoryId());
    });
    connect(m_trajectoryStartButton, &QPushButton::clicked,
        this, &MotionControlWidget::trajectoryStartRequested);
    connect(m_trajectoryPauseButton, &QPushButton::clicked,
        this, &MotionControlWidget::trajectoryPauseRequested);
    connect(m_trajectoryStopButton, &QPushButton::clicked,
        this, &MotionControlWidget::trajectoryStopRequested);
    connect(m_trajectoryStepButton, &QPushButton::clicked,
        this, &MotionControlWidget::trajectoryStepRequested);

    m_collisionSeparator = new QFrame(this);
    m_collisionSeparator->setFrameShape(QFrame::HLine);
    m_collisionSeparator->setFrameShadow(QFrame::Sunken);
    motionLayout->addWidget(m_collisionSeparator);

    motionLayout->addWidget(robot_qt_viewer::makePanelTitle("Collision Detector", this));

    auto* collisionControlLayout = new QGridLayout();
    collisionControlLayout->setContentsMargins(0, 0, 0, 0);
    collisionControlLayout->setSpacing(8);
    robot_qt_viewer::configureInspectorGrid(collisionControlLayout);

    m_collisionDetectorCombo = new QComboBox(this);
    robot_qt_viewer::makeHorizontallyCompressible(m_collisionDetectorCombo);
    m_collisionDetectorCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    connect(m_collisionDetectorCombo, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this, [this]() {
            if(m_updatingUi || m_collisionDetectorCombo == nullptr) {
                return;
            }
            emit collisionDetectorSelectionChanged(currentCollisionDetectorId());
        });
    collisionControlLayout->addWidget(m_collisionDetectorCombo, 0, 0, 1, 2);

    m_collisionMonitoringButton = new QPushButton("Enable Detection", this);
    m_collisionMonitoringButton->setCheckable(true);
    m_collisionMonitoringButton->setMinimumHeight(32);
    m_collisionMonitoringButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    m_collisionMonitoringButton->setToolTip("Enable collision detection for Robot Run.");
    robot_qt_viewer::makeHorizontallyCompressible(m_collisionMonitoringButton);
    connect(m_collisionMonitoringButton, &QPushButton::toggled, this, [this](bool checked) {
        if(m_collisionMonitoringButton != nullptr) {
            m_collisionMonitoringButton->setText(checked ? "Disable Detection" : "Enable Detection");
            m_collisionMonitoringButton->setToolTip(checked
                ? "Disable collision queries for Robot Run."
                : "Enable collision queries for Robot Run.");
        }
        if(m_collisionGeometryButton != nullptr) {
            m_collisionGeometryButton->setEnabled(m_collisionMonitoringAvailable && checked);
            if(!checked && m_collisionGeometryButton->isChecked()) {
                m_collisionGeometryButton->setChecked(false);
            }
        }
        if(!m_updatingUi) {
            emit collisionMonitoringChanged(checked);
        }
    });
    collisionControlLayout->addWidget(m_collisionMonitoringButton, 1, 0, 1, 2);

    m_collisionGeometryButton = new QPushButton("Show Collision Model", this);
    m_collisionGeometryButton->setCheckable(true);
    m_collisionGeometryButton->setMinimumHeight(32);
    m_collisionGeometryButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    m_collisionGeometryButton->setToolTip("Show collision geometry for the active robot run detector.");
    robot_qt_viewer::makeHorizontallyCompressible(m_collisionGeometryButton);
    connect(m_collisionGeometryButton, &QPushButton::toggled, this, [this](bool checked) {
        if(m_collisionGeometryButton != nullptr) {
            m_collisionGeometryButton->setText(checked ? "Hide Collision Model" : "Show Collision Model");
            m_collisionGeometryButton->setToolTip(checked
                ? "Hide collision geometry and keep detector results visible."
                : "Show collision geometry for the active robot run detector.");
        }
        if(!m_updatingUi) {
            emit collisionGeometryVisibilityChanged(checked);
        }
    });
    collisionControlLayout->addWidget(m_collisionGeometryButton, 2, 0, 1, 2);
    collisionControlLayout->setColumnStretch(0, 1);
    collisionControlLayout->setColumnStretch(1, 1);
    motionLayout->addLayout(collisionControlLayout);

    setMotionActionsEnabled(false);
    setCollisionMonitoringAvailable(false);
}

void MotionControlWidget::setRobotId(const QString& robotId)
{
    if(m_jointRobotLabel == nullptr) {
        return;
    }

    m_jointRobotLabel->setText(robotId.isEmpty()
        ? "No robot selected"
        : QString("Robot: %1").arg(robotId));
}

void MotionControlWidget::setJoints(const QStringList& jointNames, const QStringList& jointTypes)
{
    m_updatingUi = true;
    clearJointRows();
    for(int i = 0; i < jointNames.size(); ++i) {
        const QString jointType = i < jointTypes.size() ? jointTypes.at(i) : QString();
        addJointRow(jointNames.at(i), jointType);
    }
    setAutoMotionChecked(false);
    m_updatingUi = false;
    setMotionActionsEnabled(!m_jointRows.empty());
}

void MotionControlWidget::setJointDisplayValue(const QString& jointName, double displayValue, bool valueValid)
{
    for(JointControlRow& row : m_jointRows) {
        if(row.jointName != jointName) {
            continue;
        }

        QSignalBlocker sliderBlocker(row.slider);
        QSignalBlocker spinBlocker(row.valueSpin);
        if(row.slider != nullptr) {
            row.slider->setEnabled(valueValid);
            row.slider->setValue(valueValid ? static_cast<int>(displayValue * 10.0) : 0);
        }
        if(row.valueSpin != nullptr) {
            row.valueSpin->setEnabled(valueValid);
            row.valueSpin->setValue(valueValid ? displayValue : 0.0);
        }
        return;
    }
}

void MotionControlWidget::setMotionActionsEnabled(bool enabled)
{
    if(m_autoMotionCheck != nullptr) {
        m_autoMotionCheck->setEnabled(enabled);
    }
    if(m_autoAmplitudeSpin != nullptr) {
        m_autoAmplitudeSpin->setEnabled(enabled);
    }
    if(m_autoSpeedSpin != nullptr) {
        m_autoSpeedSpin->setEnabled(enabled);
    }
    if(m_applyInitialPoseButton != nullptr) {
        m_applyInitialPoseButton->setEnabled(enabled);
    }
}

void MotionControlWidget::setAutoMotionChecked(bool checked)
{
    if(m_autoMotionCheck == nullptr) {
        return;
    }

    QSignalBlocker blocker(m_autoMotionCheck);
    m_autoMotionCheck->setChecked(checked);
}

bool MotionControlWidget::autoMotionChecked() const
{
    return m_autoMotionCheck != nullptr && m_autoMotionCheck->isChecked();
}

double MotionControlWidget::autoAmplitude() const
{
    return m_autoAmplitudeSpin != nullptr ? m_autoAmplitudeSpin->value() : 30.0;
}

double MotionControlWidget::autoSpeed() const
{
    return m_autoSpeedSpin != nullptr ? m_autoSpeedSpin->value() : 1.0;
}

bool MotionControlWidget::hasRevoluteJoint() const
{
    for(const JointControlRow& row : m_jointRows) {
        if(isRevoluteJoint(row.jointType)) {
            return true;
        }
    }
    return false;
}

void MotionControlWidget::setTrajectories(const QVector<TrajectoryItem>& trajectories)
{
    const QString previousId = currentTrajectoryId();
    QSignalBlocker blocker(m_trajectoryCombo);
    m_trajectoryCombo->clear();
    int selectedIndex = -1;
    for(const TrajectoryItem& trajectory : trajectories) {
        m_trajectoryCombo->addItem(trajectory.label, trajectory.id);
        if(trajectory.id == previousId) {
            selectedIndex = m_trajectoryCombo->count() - 1;
        }
    }
    if(selectedIndex >= 0) {
        m_trajectoryCombo->setCurrentIndex(selectedIndex);
    }
    const bool available = m_trajectoryCombo->count() > 0;
    m_trajectoryCombo->setEnabled(available);
    m_trajectoryLoadButton->setEnabled(available);
}

QString MotionControlWidget::currentTrajectoryId() const
{
    return m_trajectoryCombo != nullptr
        ? m_trajectoryCombo->currentData().toString()
        : QString();
}

void MotionControlWidget::setTrajectoryStatus(const QString& status)
{
    if(m_trajectoryStatus != nullptr) {
        m_trajectoryStatus->setText(status);
    }
}

void MotionControlWidget::setCollisionDetectors(
    const QVector<CollisionDetectorItem>& detectors,
    const QString& preferredDetectorId)
{
    if(m_collisionDetectorCombo == nullptr || m_collisionMonitoringButton == nullptr) {
        return;
    }

    const QString previousId = currentCollisionDetectorId();
    QString targetId = !preferredDetectorId.isEmpty() ? preferredDetectorId : previousId;

    QSignalBlocker comboBlocker(m_collisionDetectorCombo);
    QSignalBlocker buttonBlocker(m_collisionMonitoringButton);
    m_collisionDetectorCombo->clear();

    int targetIndex = -1;
    int activeIndex = -1;
    for(const CollisionDetectorItem& detector : detectors) {
        const QString label = detector.label.isEmpty() ? detector.id : detector.label;
        m_collisionDetectorCombo->addItem(label, detector.id);
        const int index = m_collisionDetectorCombo->count() - 1;
        if(detector.id == targetId) {
            targetIndex = index;
        }
        if(detector.active) {
            activeIndex = index;
        }
    }

    if(targetIndex < 0) {
        targetIndex = activeIndex >= 0 ? activeIndex : (m_collisionDetectorCombo->count() > 0 ? 0 : -1);
    }
    if(targetIndex >= 0) {
        m_collisionDetectorCombo->setCurrentIndex(targetIndex);
        const QString targetDetectorId = m_collisionDetectorCombo->itemData(targetIndex).toString();
        for(const CollisionDetectorItem& detector : detectors) {
            if(detector.id == targetDetectorId) {
                setCollisionMonitoringChecked(detector.enabled);
                break;
            }
        }
    } else {
        setCollisionMonitoringChecked(false);
    }

    setCollisionMonitoringAvailable(m_collisionDetectorCombo->count() > 0);
}

QString MotionControlWidget::currentCollisionDetectorId() const
{
    return m_collisionDetectorCombo != nullptr
        ? m_collisionDetectorCombo->currentData().toString()
        : QString();
}

void MotionControlWidget::setCollisionMonitoringChecked(bool checked)
{
    if(m_collisionMonitoringButton == nullptr) {
        return;
    }

    QSignalBlocker blocker(m_collisionMonitoringButton);
    m_collisionMonitoringButton->setChecked(checked);
    m_collisionMonitoringButton->setText(checked ? "Disable Detection" : "Enable Detection");
    m_collisionMonitoringButton->setToolTip(checked
        ? "Disable collision detection and show motion only."
        : "Enable collision detection for Robot Run.");
    if(m_collisionGeometryButton != nullptr) {
        m_collisionGeometryButton->setEnabled(m_collisionMonitoringAvailable && checked);
        if(!checked) {
            QSignalBlocker geometryBlocker(m_collisionGeometryButton);
            m_collisionGeometryButton->setChecked(false);
            m_collisionGeometryButton->setText("Show Collision Model");
            m_collisionGeometryButton->setToolTip("Show collision geometry for the active robot run detector.");
        }
    }
}

bool MotionControlWidget::collisionMonitoringChecked() const
{
    return m_collisionMonitoringButton != nullptr && m_collisionMonitoringButton->isChecked();
}

void MotionControlWidget::setCollisionMonitoringAvailable(bool available)
{
    m_collisionMonitoringAvailable = available;
    if(m_collisionDetectorCombo != nullptr) {
        m_collisionDetectorCombo->setEnabled(available);
    }
    if(m_collisionMonitoringButton != nullptr) {
        m_collisionMonitoringButton->setEnabled(available);
    }
    if(m_collisionGeometryButton != nullptr) {
        m_collisionGeometryButton->setEnabled(available && collisionMonitoringChecked());
        if(!available) {
            QSignalBlocker blocker(m_collisionGeometryButton);
            m_collisionGeometryButton->setChecked(false);
            m_collisionGeometryButton->setText("Show Collision Model");
        }
    }
    if(m_collisionResultsWidget != nullptr) {
        m_collisionResultsWidget->setEnabled(available);
    }
}

void MotionControlWidget::setCollisionGeometryChecked(bool checked)
{
    if(m_collisionGeometryButton == nullptr) {
        return;
    }

    QSignalBlocker blocker(m_collisionGeometryButton);
    m_collisionGeometryButton->setChecked(checked);
    m_collisionGeometryButton->setText(checked ? "Hide Collision Model" : "Show Collision Model");
    m_collisionGeometryButton->setToolTip(checked
        ? "Hide collision geometry and keep detector results visible."
        : "Show collision geometry for the active robot run detector.");
}

CollisionResultsWidget* MotionControlWidget::collisionResultsWidget() const
{
    return m_collisionResultsWidget;
}

void MotionControlWidget::clearJointRows()
{
    for(const JointControlRow& row : m_jointRows) {
        if(row.rowWidget != nullptr) {
            row.rowWidget->deleteLater();
        }
    }
    m_jointRows.clear();
}

void MotionControlWidget::addJointRow(const QString& jointName, const QString& jointType)
{
    if(m_jointRowsLayout == nullptr || m_jointRowsWidget == nullptr) {
        return;
    }

    const bool revolute = isRevoluteJoint(jointType);
    const int rowIndex = m_jointRows.size();

    auto* rowWidget = new QFrame(m_jointRowsWidget);
    rowWidget->setProperty("jointRow", true);
    auto* rowLayout = new QGridLayout(rowWidget);
    rowLayout->setContentsMargins(8, 6, 8, 6);
    rowLayout->setHorizontalSpacing(8);
    rowLayout->setVerticalSpacing(4);

    auto* nameLabel = new QLabel(jointName, rowWidget);
    nameLabel->setProperty("jointName", true);
    nameLabel->setWordWrap(true);
    nameLabel->setToolTip(jointName);
    robot_qt_viewer::makeHorizontallyCompressible(nameLabel);
    rowLayout->addWidget(nameLabel, 0, 0, 1, 2);

    auto* slider = new QSlider(Qt::Horizontal, rowWidget);
    slider->setRange(revolute ? -1800 : -1000, revolute ? 1800 : 1000);
    slider->setMinimumWidth(40);
    slider->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);
    rowLayout->addWidget(slider, 1, 0);

    auto* spin = new QDoubleSpinBox(rowWidget);
    spin->setDecimals(3);
    spin->setRange(revolute ? -180.0 : -100.0, revolute ? 180.0 : 100.0);
    spin->setSingleStep(revolute ? 1.0 : 0.01);
    spin->setSuffix(revolute ? " deg" : " raw");
    spin->setMinimumWidth(82);
    spin->setMaximumWidth(118);
    spin->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    rowLayout->addWidget(spin, 1, 1, Qt::AlignLeft);
    rowLayout->setColumnStretch(0, 1);

    if(m_jointRowsLayout->count() > 0) {
        m_jointRowsLayout->insertWidget(m_jointRowsLayout->count() - 1, rowWidget);
    } else {
        m_jointRowsLayout->addWidget(rowWidget);
    }

    connect(slider, &QSlider::valueChanged, this, [this, rowIndex](int value) {
        applyRowDisplayValue(rowIndex, static_cast<double>(value) / 10.0);
    });
    connect(spin, static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, [this, rowIndex](double value) {
        applyRowDisplayValue(rowIndex, value);
    });

    JointControlRow row;
    row.jointName = jointName;
    row.jointType = jointType;
    row.rowWidget = rowWidget;
    row.slider = slider;
    row.valueSpin = spin;
    m_jointRows.push_back(row);
}

void MotionControlWidget::applyRowDisplayValue(int rowIndex, double displayValue)
{
    if(m_updatingUi || rowIndex < 0 || rowIndex >= m_jointRows.size()) {
        return;
    }

    JointControlRow& row = m_jointRows[rowIndex];
    m_updatingUi = true;
    if(row.slider != nullptr) {
        row.slider->setValue(static_cast<int>(displayValue * 10.0));
    }
    if(row.valueSpin != nullptr) {
        row.valueSpin->setValue(displayValue);
    }
    m_updatingUi = false;

    emit jointDisplayValueChanged(row.jointName, row.jointType, displayValue);
}

bool MotionControlWidget::isRevoluteJoint(const QString& jointType) const
{
    return jointType.compare("revolute", Qt::CaseInsensitive) == 0 ||
        jointType.compare("continuous", Qt::CaseInsensitive) == 0;
}

