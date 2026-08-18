#pragma once

#include <QVector>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QFrame;
class QLabel;
class QPushButton;
class QScrollArea;
class QSlider;
class QString;
class QStringList;
class QVBoxLayout;

class CollisionResultsWidget;

class MotionControlWidget : public QWidget
{
    Q_OBJECT

public:
    struct CollisionDetectorItem
    {
        QString id;
        QString label;
        bool enabled = true;
        bool active = false;
    };

    struct TrajectoryItem
    {
        QString id;
        QString label;
    };

    explicit MotionControlWidget(QWidget* parent = nullptr);

    void setRobotId(const QString& robotId);
    void setJoints(const QStringList& jointNames, const QStringList& jointTypes);
    void setJointDisplayValue(const QString& jointName, double displayValue, bool valueValid);
    void setMotionActionsEnabled(bool enabled);
    void setAutoMotionChecked(bool checked);
    bool autoMotionChecked() const;
    double autoAmplitude() const;
    double autoSpeed() const;
    bool hasRevoluteJoint() const;
    void setTrajectories(const QVector<TrajectoryItem>& trajectories);
    QString currentTrajectoryId() const;
    void setTrajectoryStatus(const QString& status);
    void setCollisionDetectors(const QVector<CollisionDetectorItem>& detectors, const QString& preferredDetectorId);
    QString currentCollisionDetectorId() const;
    void setCollisionMonitoringChecked(bool checked);
    bool collisionMonitoringChecked() const;
    void setCollisionMonitoringAvailable(bool available);
    void setCollisionGeometryChecked(bool checked);
    CollisionResultsWidget* collisionResultsWidget() const;

signals:
    void jointDisplayValueChanged(const QString& jointName, const QString& jointType, double displayValue);
    void autoMotionChanged();
    void applyInitialPoseRequested();
    void trajectoryLoadRequested(const QString& trajectoryId);
    void trajectoryStartRequested();
    void trajectoryPauseRequested();
    void trajectoryStopRequested();
    void trajectoryStepRequested();
    void collisionDetectorSelectionChanged(const QString& detectorId);
    void collisionMonitoringChanged(bool enabled);
    void collisionGeometryVisibilityChanged(bool visible);

private:
    struct JointControlRow
    {
        QString jointName;
        QString jointType;
        QWidget* rowWidget = nullptr;
        QSlider* slider = nullptr;
        QDoubleSpinBox* valueSpin = nullptr;
    };

    void clearJointRows();
    void addJointRow(const QString& jointName, const QString& jointType);
    void applyRowDisplayValue(int rowIndex, double displayValue);
    bool isRevoluteJoint(const QString& jointType) const;

    QLabel* m_jointRobotLabel = nullptr;
    QScrollArea* m_jointScrollArea = nullptr;
    QWidget* m_jointRowsWidget = nullptr;
    QVBoxLayout* m_jointRowsLayout = nullptr;
    QCheckBox* m_autoMotionCheck = nullptr;
    QDoubleSpinBox* m_autoAmplitudeSpin = nullptr;
    QDoubleSpinBox* m_autoSpeedSpin = nullptr;
    QPushButton* m_applyInitialPoseButton = nullptr;
    QComboBox* m_trajectoryCombo = nullptr;
    QPushButton* m_trajectoryLoadButton = nullptr;
    QPushButton* m_trajectoryStartButton = nullptr;
    QPushButton* m_trajectoryPauseButton = nullptr;
    QPushButton* m_trajectoryStopButton = nullptr;
    QPushButton* m_trajectoryStepButton = nullptr;
    QLabel* m_trajectoryStatus = nullptr;
    QFrame* m_collisionSeparator = nullptr;
    QComboBox* m_collisionDetectorCombo = nullptr;
    QPushButton* m_collisionMonitoringButton = nullptr;
    QPushButton* m_collisionGeometryButton = nullptr;
    CollisionResultsWidget* m_collisionResultsWidget = nullptr;
    QVector<JointControlRow> m_jointRows;
    bool m_collisionMonitoringAvailable = false;
    bool m_updatingUi = false;
};

