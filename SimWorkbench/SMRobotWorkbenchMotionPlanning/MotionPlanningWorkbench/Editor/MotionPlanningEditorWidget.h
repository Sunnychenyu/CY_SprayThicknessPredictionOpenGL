#pragma once

#include <QWidget>

class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QSpinBox;

class MotionPlanningEditorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MotionPlanningEditorWidget(QWidget* parent = nullptr);

    void setRobotId(const QString& robotId);
    void setJointDefaults(const QString& jointNames, const QString& startJoints);
    void setResult(const QString& summary, bool success);

signals:
    void planRequested(
        const QString& startJoints,
        const QString& goalJoints,
        const QString& jointNames,
        double duration,
        int sampleCount);

private:
    QLabel* m_robotValue = nullptr;
    QLineEdit* m_startJoints = nullptr;
    QLineEdit* m_goalJoints = nullptr;
    QLineEdit* m_jointNames = nullptr;
    QDoubleSpinBox* m_duration = nullptr;
    QSpinBox* m_sampleCount = nullptr;
    QPushButton* m_planButton = nullptr;
    QLabel* m_result = nullptr;
};
