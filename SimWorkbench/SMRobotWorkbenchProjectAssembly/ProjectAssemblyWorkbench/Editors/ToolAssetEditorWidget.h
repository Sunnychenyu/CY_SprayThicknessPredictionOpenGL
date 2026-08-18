#pragma once

#include <QWidget>

#include <SimulationProject/ProjectDocument.h>

class QDoubleSpinBox;
class QLineEdit;
class QPushButton;
class ToolFrameDiagramWidget;
class ToolTransformEditorWidget;

class ToolAssetEditorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ToolAssetEditorWidget(QWidget* parent = nullptr);

    void setAsset(const simulation_project::AttachmentAssetDesc& asset);
    simulation_project::AttachmentAssetDesc asset() const;
    void setApplyButtonVisible(bool visible);

signals:
    void assetChanged(const simulation_project::AttachmentAssetDesc& asset);
    void visualTransformChanged(const simulation_project::AttachmentAssetDesc& asset);
    void tcpTransformChanged(const simulation_project::AttachmentAssetDesc& asset);
    void applyRequested(const simulation_project::AttachmentAssetDesc& asset);

private:
    void loadAssetToUi();
    simulation_project::AttachmentAssetDesc collectUiAsset() const;
    void storeUiToAsset();
    void emitAssetChanged();
    void emitVisualTransformChanged();
    void emitTcpTransformChanged();

    simulation_project::AttachmentAssetDesc m_asset;
    bool m_updating = false;

    QLineEdit* m_idEdit = nullptr;
    QLineEdit* m_nameEdit = nullptr;
    QLineEdit* m_typeEdit = nullptr;
    QLineEdit* m_visualPathEdit = nullptr;
    QDoubleSpinBox* m_visualScaleSpin = nullptr;
    ToolFrameDiagramWidget* m_frameDiagram = nullptr;
    ToolTransformEditorWidget* m_visualTransformEditor = nullptr;
    ToolTransformEditorWidget* m_tcpTransformEditor = nullptr;
    QPushButton* m_applyButton = nullptr;
};
