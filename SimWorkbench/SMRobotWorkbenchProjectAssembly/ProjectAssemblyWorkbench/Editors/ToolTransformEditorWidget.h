#pragma once

#include <QWidget>

#include <SimulationProject/ProjectDocument.h>

#include <array>

class QLabel;
class QDoubleSpinBox;

class ToolTransformEditorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ToolTransformEditorWidget(const QString& title, QWidget* parent = nullptr);

    void setTitle(const QString& title);
    void setMatrixVisible(bool visible);
    void setTransform(const simulation_project::TransformDesc& transform);
    simulation_project::TransformDesc transform() const;

signals:
    void transformChanged(const simulation_project::TransformDesc& transform);

private:
    void emitTransformChanged();
    void updateMatrix(const simulation_project::TransformDesc& transform);

    QLabel* m_titleLabel = nullptr;
    QWidget* m_matrixGroup = nullptr;
    std::array<QLabel*, 16> m_matrixLabels{};
    QDoubleSpinBox* m_xSpin = nullptr;
    QDoubleSpinBox* m_ySpin = nullptr;
    QDoubleSpinBox* m_zSpin = nullptr;
    QDoubleSpinBox* m_rollSpin = nullptr;
    QDoubleSpinBox* m_pitchSpin = nullptr;
    QDoubleSpinBox* m_yawSpin = nullptr;
};
