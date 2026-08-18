#include "MotionPlanningEditorWidget.h"

#include "RobotQtWidgetUtils.h"

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QStyle>
#include <QVBoxLayout>

MotionPlanningEditorWidget::MotionPlanningEditorWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);

    auto* title = new QLabel(QStringLiteral("Motion Planning"), this);
    title->setProperty("panelTitle", true);
    layout->addWidget(title);

    auto* form = new QFormLayout();
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    m_robotValue = new QLabel(QStringLiteral("No robot selected"), this);
    robot_qt_viewer::makeHorizontallyCompressible(m_robotValue);
    form->addRow(QStringLiteral("Robot"), m_robotValue);

    m_startJoints = new QLineEdit(this);
    m_startJoints->setPlaceholderText(QStringLiteral("0, 0, 0"));
    form->addRow(QStringLiteral("Start joints"), m_startJoints);

    m_jointNames = new QLineEdit(this);
    m_jointNames->setPlaceholderText(QStringLiteral("joint_1, joint_2, joint_3"));
    form->addRow(QStringLiteral("Joint names"), m_jointNames);

    m_goalJoints = new QLineEdit(this);
    m_goalJoints->setPlaceholderText(QStringLiteral("0.5, -0.2, 0.8"));
    form->addRow(QStringLiteral("Goal joints"), m_goalJoints);

    m_duration = new QDoubleSpinBox(this);
    m_duration->setRange(0.01, 3600.0);
    m_duration->setValue(5.0);
    m_duration->setSuffix(QStringLiteral(" s"));
    form->addRow(QStringLiteral("Duration"), m_duration);

    m_sampleCount = new QSpinBox(this);
    m_sampleCount->setRange(2, 10000);
    m_sampleCount->setValue(50);
    form->addRow(QStringLiteral("Samples"), m_sampleCount);

    layout->addLayout(form);

    m_planButton = new QPushButton(QStringLiteral("Plan trajectory"), this);
    layout->addWidget(m_planButton);

    m_result = new QLabel(QStringLiteral("Select a robot and enter joint vectors."), this);
    m_result->setWordWrap(true);
    layout->addWidget(m_result);
    layout->addStretch(1);

    connect(m_planButton, &QPushButton::clicked, this, [this]() {
        emit planRequested(
            m_startJoints->text(),
            m_goalJoints->text(),
            m_jointNames->text(),
            m_duration->value(),
            m_sampleCount->value());
    });
}

void MotionPlanningEditorWidget::setJointDefaults(
    const QString& jointNames,
    const QString& startJoints)
{
    if(m_jointNames->text().trimmed().isEmpty()) {
        m_jointNames->setText(jointNames);
    }
    if(m_startJoints->text().trimmed().isEmpty()) {
        m_startJoints->setText(startJoints);
    }
}

void MotionPlanningEditorWidget::setRobotId(const QString& robotId)
{
    m_robotValue->setText(robotId.isEmpty() ? QStringLiteral("No robot selected") : robotId);
    m_planButton->setEnabled(!robotId.isEmpty());
}

void MotionPlanningEditorWidget::setResult(const QString& summary, bool success)
{
    m_result->setText(summary);
    m_result->setProperty("error", !success);
    m_result->style()->unpolish(m_result);
    m_result->style()->polish(m_result);
}
