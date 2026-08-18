#pragma once

#include <SimulationProject/ProjectDocument.h>

#include <QMainWindow>
#include <QString>

#include <filesystem>

class QLabel;
class QPushButton;
class QCheckBox;
class QComboBox;
class QTimer;
class RobotViewport;

class VisualQuickStartWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit VisualQuickStartWindow(QWidget* parent = nullptr);
    ~VisualQuickStartWindow() override;

private slots:
    void toggleRunning();
    void resetDemo();
    void rebuildScene();
    void tickDemo();
    void refreshStatus();

private:
    simulation_project::ProjectDocument makeProjectDocument() const;
    simulation_project::TransformDesc obstacleTransform(double x) const;
    std::filesystem::path projectRoot() const;
    QString obstacleType() const;
    void setStatusText(const QString& text);

    RobotViewport* m_viewport = nullptr;
    QLabel* m_robotLabel = nullptr;
    QLabel* m_detectorLabel = nullptr;
    QLabel* m_stateLabel = nullptr;
    QLabel* m_contactsLabel = nullptr;
    QLabel* m_distanceLabel = nullptr;
    QLabel* m_pairLabel = nullptr;
    QLabel* m_motionLabel = nullptr;
    QPushButton* m_runButton = nullptr;
    QPushButton* m_resetButton = nullptr;
    QComboBox* m_obstacleType = nullptr;
    QCheckBox* m_collisionGeometry = nullptr;
    QCheckBox* m_pairEnabled = nullptr;
    QTimer* m_timer = nullptr;

    double m_phase = 0.0;
    bool m_running = true;
};

