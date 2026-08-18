#include "ToolTransformEditorWidget.h"

#include "RobotQtWidgetUtils.h"

#include <QDoubleSpinBox>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QList>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <array>
#include <cmath>
#include <memory>
#include <vector>

namespace
{
    constexpr double kPi = 3.14159265358979323846;

    double toDegrees(double radians)
    {
        return radians * 180.0 / kPi;
    }

    double toRadians(double degrees)
    {
        return degrees * kPi / 180.0;
    }

    std::array<double, 16> transformMatrixValues(const simulation_project::TransformDesc& transform)
    {
        const double cr = std::cos(transform.roll);
        const double sr = std::sin(transform.roll);
        const double cp = std::cos(transform.pitch);
        const double sp = std::sin(transform.pitch);
        const double cy = std::cos(transform.yaw);
        const double sy = std::sin(transform.yaw);

        return {
            cy * cp,
            cy * sp * sr - sy * cr,
            cy * sp * cr + sy * sr,
            transform.x,
            sy * cp,
            sy * sp * sr + cy * cr,
            sy * sp * cr - cy * sr,
            transform.y,
            -sp,
            cp * sr,
            cp * cr,
            transform.z,
            0.0,
            0.0,
            0.0,
            1.0
        };
    }

    QDoubleSpinBox* makeDistanceSpin(QWidget* parent)
    {
        auto* spin = new QDoubleSpinBox(parent);
        spin->setRange(-100000.0, 100000.0);
        spin->setDecimals(4);
        spin->setSingleStep(0.01);
        spin->setSuffix(" m");
        spin->setKeyboardTracking(false);
        robot_qt_viewer::makeHorizontallyCompressible(spin);
        return spin;
    }

    QDoubleSpinBox* makeAngleSpin(QWidget* parent)
    {
        auto* spin = new QDoubleSpinBox(parent);
        spin->setRange(-360.0, 360.0);
        spin->setDecimals(3);
        spin->setSingleStep(1.0);
        spin->setSuffix(" deg");
        spin->setKeyboardTracking(false);
        robot_qt_viewer::makeHorizontallyCompressible(spin);
        return spin;
    }

    void addTransformRows(
        QGridLayout* layout,
        QWidget* parent,
        QDoubleSpinBox* xSpin,
        QDoubleSpinBox* ySpin,
        QDoubleSpinBox* zSpin,
        QDoubleSpinBox* rollSpin,
        QDoubleSpinBox* pitchSpin,
        QDoubleSpinBox* yawSpin)
    {
        layout->addWidget(new QLabel("X", parent), 0, 0);
        layout->addWidget(xSpin, 0, 1);
        layout->addWidget(new QLabel("Y", parent), 1, 0);
        layout->addWidget(ySpin, 1, 1);
        layout->addWidget(new QLabel("Z", parent), 2, 0);
        layout->addWidget(zSpin, 2, 1);
        layout->addWidget(new QLabel("Theta", parent), 3, 0);
        layout->addWidget(rollSpin, 3, 1);
        layout->addWidget(new QLabel("Phi", parent), 4, 0);
        layout->addWidget(pitchSpin, 4, 1);
        layout->addWidget(new QLabel("Psi", parent), 5, 0);
        layout->addWidget(yawSpin, 5, 1);
        layout->setColumnStretch(1, 1);
    }
}

ToolTransformEditorWidget::ToolTransformEditorWidget(const QString& title, QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);

    m_titleLabel = robot_qt_viewer::makePanelTitle(title, this);
    layout->addWidget(m_titleLabel);

    m_matrixGroup = new QGroupBox(QStringLiteral("Matrix 4x4"), this);
    auto* matrixGrid = new QGridLayout(m_matrixGroup);
    matrixGrid->setContentsMargins(8, 24, 8, 8);
    matrixGrid->setHorizontalSpacing(10);
    matrixGrid->setVerticalSpacing(4);
    for(int row = 0; row < 4; ++row) {
        for(int column = 0; column < 4; ++column) {
            QLabel* label = new QLabel(QStringLiteral("0.0000"), m_matrixGroup);
            label->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
            QFont font(QStringLiteral("Consolas"));
            font.setStyleHint(QFont::Monospace);
            label->setFont(font);
            label->setMinimumWidth(68);
            label->setProperty("matrixCell", true);
            m_matrixLabels[static_cast<std::size_t>(row * 4 + column)] = label;
            matrixGrid->addWidget(label, row, column);
        }
    }
    layout->addWidget(m_matrixGroup);

    auto* parameterGroup = new QGroupBox(QStringLiteral("Local Transform"), this);
    auto* grid = new QGridLayout(parameterGroup);
    grid->setContentsMargins(8, 24, 2, 8);
    grid->setHorizontalSpacing(10);
    grid->setVerticalSpacing(7);
    m_xSpin = makeDistanceSpin(this);
    m_ySpin = makeDistanceSpin(this);
    m_zSpin = makeDistanceSpin(this);
    m_rollSpin = makeAngleSpin(this);
    m_pitchSpin = makeAngleSpin(this);
    m_yawSpin = makeAngleSpin(this);
    addTransformRows(grid, parameterGroup, m_xSpin, m_ySpin, m_zSpin, m_rollSpin, m_pitchSpin, m_yawSpin);
    layout->addWidget(parameterGroup);

    const QList<QDoubleSpinBox*> spins = {
        m_xSpin,
        m_ySpin,
        m_zSpin,
        m_rollSpin,
        m_pitchSpin,
        m_yawSpin
    };
    for(QDoubleSpinBox* spin : spins) {
        connect(
            spin,
            static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
            this,
            [this](double) {
                emitTransformChanged();
            });
    }
}

void ToolTransformEditorWidget::setTitle(const QString& title)
{
    if(m_titleLabel != nullptr) {
        m_titleLabel->setText(title);
    }
}

void ToolTransformEditorWidget::setMatrixVisible(bool visible)
{
    if(m_matrixGroup != nullptr) {
        m_matrixGroup->setVisible(visible);
    }
}

void ToolTransformEditorWidget::setTransform(const simulation_project::TransformDesc& transform)
{
    const QList<QDoubleSpinBox*> spins = {
        m_xSpin,
        m_ySpin,
        m_zSpin,
        m_rollSpin,
        m_pitchSpin,
        m_yawSpin
    };
    std::vector<std::unique_ptr<QSignalBlocker>> blockers;
    blockers.reserve(static_cast<std::size_t>(spins.size()));
    for(QDoubleSpinBox* spin : spins) {
        if(spin != nullptr) {
            blockers.push_back(std::make_unique<QSignalBlocker>(spin));
        }
    }

    m_xSpin->setValue(transform.x);
    m_ySpin->setValue(transform.y);
    m_zSpin->setValue(transform.z);
    m_rollSpin->setValue(toDegrees(transform.roll));
    m_pitchSpin->setValue(toDegrees(transform.pitch));
    m_yawSpin->setValue(toDegrees(transform.yaw));
    updateMatrix(transform);
}

simulation_project::TransformDesc ToolTransformEditorWidget::transform() const
{
    simulation_project::TransformDesc value;
    value.x = m_xSpin->value();
    value.y = m_ySpin->value();
    value.z = m_zSpin->value();
    value.roll = toRadians(m_rollSpin->value());
    value.pitch = toRadians(m_pitchSpin->value());
    value.yaw = toRadians(m_yawSpin->value());
    return value;
}

void ToolTransformEditorWidget::emitTransformChanged()
{
    const simulation_project::TransformDesc value = transform();
    updateMatrix(value);
    emit transformChanged(value);
}

void ToolTransformEditorWidget::updateMatrix(const simulation_project::TransformDesc& transform)
{
    const std::array<double, 16> values = transformMatrixValues(transform);
    for(std::size_t i = 0; i < values.size(); ++i) {
        QLabel* label = m_matrixLabels[i];
        if(label != nullptr) {
            label->setText(QString::number(values[i], 'f', 4));
        }
    }
}
