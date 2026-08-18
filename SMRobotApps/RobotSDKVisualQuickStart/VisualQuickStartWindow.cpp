#include "VisualQuickStartWindow.h"

#include "RobotViewport.h"

#include <data_path.h>

#include <QCheckBox>
#include <QComboBox>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

namespace
{
    constexpr const char* kRobotId = "sdk_visual_robot";
    constexpr const char* kObjectId = "moving_obstacle";
    constexpr const char* kDetectorId = "robot_object_collision";

    QString boolText(bool value)
    {
        return value ? "yes" : "no";
    }

    simulation_project::CollisionDetectorTargetDesc robotTarget()
    {
        simulation_project::CollisionDetectorTargetDesc target;
        target.robotId = kRobotId;
        return target;
    }

    simulation_project::CollisionDetectorTargetDesc objectTarget()
    {
        simulation_project::CollisionDetectorTargetDesc target;
        target.objectId = kObjectId;
        return target;
    }

    simulation_project::CollisionPairGeneratorDesc robotObjectGenerator()
    {
        simulation_project::CollisionPairGeneratorDesc generator;
        generator.type = "RobotObject";
        generator.robotId = kRobotId;
        generator.objectId = kObjectId;
        return generator;
    }
}

VisualQuickStartWindow::VisualQuickStartWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("RobotSDK Visual QuickStart");

    m_viewport = new RobotViewport(this);

    QWidget* panel = new QWidget(this);
    panel->setMinimumWidth(300);
    panel->setMaximumWidth(360);

    m_robotLabel = new QLabel(panel);
    m_detectorLabel = new QLabel(panel);
    m_stateLabel = new QLabel(panel);
    m_contactsLabel = new QLabel(panel);
    m_distanceLabel = new QLabel(panel);
    m_pairLabel = new QLabel(panel);
    m_motionLabel = new QLabel(panel);

    m_runButton = new QPushButton("Pause", panel);
    m_resetButton = new QPushButton("Reset", panel);
    m_obstacleType = new QComboBox(panel);
    m_obstacleType->addItem("sphere");
    m_obstacleType->addItem("box");
    m_obstacleType->addItem("cylinder");

    m_collisionGeometry = new QCheckBox("Collision geometry", panel);
    m_collisionGeometry->setChecked(true);
    m_pairEnabled = new QCheckBox("Robot-obstacle pairs", panel);
    m_pairEnabled->setChecked(true);

    QGridLayout* statusLayout = new QGridLayout();
    statusLayout->addWidget(new QLabel("Robot", panel), 0, 0);
    statusLayout->addWidget(m_robotLabel, 0, 1);
    statusLayout->addWidget(new QLabel("Detector", panel), 1, 0);
    statusLayout->addWidget(m_detectorLabel, 1, 1);
    statusLayout->addWidget(new QLabel("Collision", panel), 2, 0);
    statusLayout->addWidget(m_stateLabel, 2, 1);
    statusLayout->addWidget(new QLabel("Contacts", panel), 3, 0);
    statusLayout->addWidget(m_contactsLabel, 3, 1);
    statusLayout->addWidget(new QLabel("Distance", panel), 4, 0);
    statusLayout->addWidget(m_distanceLabel, 4, 1);
    statusLayout->addWidget(new QLabel("First pair", panel), 5, 0);
    statusLayout->addWidget(m_pairLabel, 5, 1);
    statusLayout->addWidget(new QLabel("Obstacle X", panel), 6, 0);
    statusLayout->addWidget(m_motionLabel, 6, 1);

    QGroupBox* statusGroup = new QGroupBox("Status", panel);
    statusGroup->setLayout(statusLayout);

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(m_runButton);
    buttonLayout->addWidget(m_resetButton);

    QVBoxLayout* panelLayout = new QVBoxLayout(panel);
    panelLayout->addWidget(statusGroup);
    panelLayout->addWidget(new QLabel("Obstacle", panel));
    panelLayout->addWidget(m_obstacleType);
    panelLayout->addWidget(m_collisionGeometry);
    panelLayout->addWidget(m_pairEnabled);
    panelLayout->addLayout(buttonLayout);
    panelLayout->addStretch(1);

    QWidget* central = new QWidget(this);
    QHBoxLayout* layout = new QHBoxLayout(central);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->addWidget(m_viewport, 1);
    layout->addWidget(panel);
    setCentralWidget(central);

    connect(m_runButton, &QPushButton::clicked, this, &VisualQuickStartWindow::toggleRunning);
    connect(m_resetButton, &QPushButton::clicked, this, &VisualQuickStartWindow::resetDemo);
    connect(m_obstacleType, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &VisualQuickStartWindow::rebuildScene);
    connect(m_collisionGeometry, &QCheckBox::toggled, m_viewport, &RobotViewport::setCollisionGeometryVisible);
    connect(m_pairEnabled, &QCheckBox::toggled, this,
        [this](bool checked) {
            m_viewport->setCollisionDetectorEnabled(kDetectorId, checked);
            refreshStatus();
        });
    connect(m_viewport, &RobotViewport::robotStateUpdated, this, &VisualQuickStartWindow::refreshStatus);
    connect(m_viewport, &RobotViewport::robotLinksAvailable, this,
        [this](const QString&, const QString& robotName, const QStringList& links, const QStringList& joints, const QStringList&, const QStringList&) {
            m_robotLabel->setText(QString("%1 (%2 links, %3 joints)").arg(robotName).arg(links.size()).arg(joints.size()));
        });

    m_timer = new QTimer(this);
    m_timer->setInterval(33);
    connect(m_timer, &QTimer::timeout, this, &VisualQuickStartWindow::tickDemo);
    m_timer->start();

    rebuildScene();
}

VisualQuickStartWindow::~VisualQuickStartWindow() = default;

std::filesystem::path VisualQuickStartWindow::projectRoot() const
{
    return std::filesystem::path(PROJECT_SOURCE_PATH);
}

QString VisualQuickStartWindow::obstacleType() const
{
    return m_obstacleType->currentText();
}

simulation_project::TransformDesc VisualQuickStartWindow::obstacleTransform(double x) const
{
    simulation_project::TransformDesc transform;
    transform.x = x;
    transform.y = 0.0;
    transform.z = 0.55;

    if(obstacleType() == "cylinder") {
        transform.roll = 1.5707963267948966;
    }

    return transform;
}

simulation_project::ProjectDocument VisualQuickStartWindow::makeProjectDocument() const
{
    simulation_project::ProjectDocument document;
    document.view.camera.target = simulation_project::Vec3Desc{ 0.0, 0.0, 0.55 };
    document.view.camera.distance = 3.0;
    document.view.camera.yaw = 35.0;
    document.view.camera.pitch = -20.0;
    document.view.showGrid = true;
    document.view.showAxis = true;

    simulation_project::RobotDesc robot;
    robot.id = kRobotId;
    robot.name = "IIWA SDK Visual";
    robot.sourceType = "urdf";
    robot.sourcePath = "data/drake_models/iiwa_description/urdf/iiwa14_spheres_collision.urdf";
    robot.visible = true;
    robot.collisionEnabled = true;
    document.robots.push_back(robot);

    simulation_project::SceneObjectDesc object;
    object.id = kObjectId;
    object.name = "Moving obstacle";
    object.objectType = "fixture";
    object.sourcePath = "data/model/cylinder.stl";
    object.visible = true;
    object.collisionEnabled = true;
    object.transform = obstacleTransform(-0.55);

    if(obstacleType() == "box") {
        object.visualScale = 0.35;
        object.collisionScale = 0.35;
    } else if(obstacleType() == "cylinder") {
        object.visualScale = 0.28;
        object.collisionScale = 0.28;
    } else {
        object.visualScale = 0.42;
        object.collisionScale = 0.42;
    }
    document.objects.push_back(object);

    simulation_project::CollisionDetectorDesc detector;
    detector.id = kDetectorId;
    detector.name = "Robot vs obstacle";
    detector.type = "RobotObject";
    detector.enabled = true;
    detector.contacts = true;
    detector.nearestPoints = true;
    detector.distance = true;
    detector.maxContacts = 32;
    detector.enabled = m_pairEnabled == nullptr || m_pairEnabled->isChecked();
    detector.targets.push_back(robotTarget());
    detector.targets.push_back(objectTarget());
    detector.pairGenerators.push_back(robotObjectGenerator());
    detector.visualization.showCollisionGeometry = true;
    detector.visualization.showContacts = true;
    detector.visualization.showNormals = true;
    detector.visualization.showNearestPoints = true;
    detector.visualization.showObjectHighlight = true;
    detector.visualization.alpha = 0.25;
    document.collision.detectors.push_back(detector);

    return document;
}

void VisualQuickStartWindow::toggleRunning()
{
    m_running = !m_running;
    m_runButton->setText(m_running ? "Pause" : "Start");
}

void VisualQuickStartWindow::resetDemo()
{
    m_phase = 0.0;
    m_viewport->previewSceneObjectTransform(kObjectId, obstacleTransform(-0.55));
    refreshStatus();
}

void VisualQuickStartWindow::rebuildScene()
{
    m_phase = 0.0;
    const simulation_project::ProjectDocument document = makeProjectDocument();
    const bool loaded = m_viewport->loadProjectDocument(document, projectRoot());
    m_viewport->setRobotAutoMotion(kRobotId, true, 0.35, 1.1);
    m_viewport->setCollisionGeometryVisible(m_collisionGeometry->isChecked());
    m_viewport->setActiveCollisionDetector(kDetectorId);
    setStatusText(loaded ? "scene loaded" : "scene load failed");
}

void VisualQuickStartWindow::tickDemo()
{
    if(!m_running) {
        return;
    }

    m_phase += 0.018;
    if(m_phase > 1.0) {
        m_phase = 0.0;
    }

    const double x = -0.55 + 1.10 * m_phase;
    m_viewport->previewSceneObjectTransform(kObjectId, obstacleTransform(x));
    m_motionLabel->setText(QString::number(x, 'f', 3));
}

void VisualQuickStartWindow::refreshStatus()
{
    const std::vector<ProjectScene::CollisionDetectorInfo> detectors = m_viewport->collisionDetectors();
    if(detectors.empty()) {
        m_detectorLabel->setText("none");
        m_stateLabel->setText("n/a");
        m_contactsLabel->setText("0");
        return;
    }

    const ProjectScene::CollisionDetectorInfo* active = &detectors.front();
    for(const ProjectScene::CollisionDetectorInfo& detector : detectors) {
        if(detector.active) {
            active = &detector;
            break;
        }
    }

    const bool colliding = active->contactCount > 0;
    m_detectorLabel->setText(QString("%1 (%2 pairs)")
        .arg(QString::fromStdString(active->name))
        .arg(static_cast<qulonglong>(active->includePairCount)));
    m_stateLabel->setText(active->enabled ? boolText(colliding) : "disabled");
    m_contactsLabel->setText(QString::number(static_cast<qulonglong>(active->contactCount)));

    if(active->hasResult && active->minDistance < std::numeric_limits<double>::max()) {
        m_distanceLabel->setText(QString::number(active->minDistance, 'f', 4));
    } else {
        m_distanceLabel->setText("n/a");
    }

    if(!active->firstPairA.empty() || !active->firstPairB.empty()) {
        m_pairLabel->setText(QString("%1 vs %2")
            .arg(QString::fromStdString(active->firstPairA))
            .arg(QString::fromStdString(active->firstPairB)));
    } else {
        m_pairLabel->setText("n/a");
    }
}

void VisualQuickStartWindow::setStatusText(const QString& text)
{
    m_detectorLabel->setText(text);
    m_stateLabel->setText("n/a");
    m_contactsLabel->setText("0");
    m_distanceLabel->setText("n/a");
    m_pairLabel->setText("n/a");
    m_motionLabel->setText("-0.550");
}
