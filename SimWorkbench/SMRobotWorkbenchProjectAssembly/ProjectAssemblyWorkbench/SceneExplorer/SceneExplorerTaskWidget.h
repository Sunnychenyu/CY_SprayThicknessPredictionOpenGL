#pragma once

#include "SceneExplorerViewModel.h"

#include <QWidget>

class QCheckBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTimer;
class SceneExplorerMountBindingDiagramWidget;
class SceneExplorerObjectFrameDiagramWidget;
class ToolTransformEditorWidget;

class SceneExplorerTaskWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SceneExplorerTaskWidget(QWidget* parent = nullptr);

    void setDocumentView(const robot_qt_viewer::SceneExplorerViewModel& viewModel);

signals:
    void transformPreviewChanged(
        const robot_qt_viewer::SceneExplorerNodeRef& target,
        const simulation_project::TransformDesc& transform);
    void transformApplyRequested(
        const robot_qt_viewer::SceneExplorerNodeRef& target,
        const simulation_project::TransformDesc& transform);
    void transformCancelRequested(const robot_qt_viewer::SceneExplorerNodeRef& target);
    void objectFrameVisibilityChanged(
        const robot_qt_viewer::SceneExplorerNodeRef& target,
        bool visible);

private:
    robot_qt_viewer::SceneExplorerNodeRef currentTransformTarget() const;
    robot_qt_viewer::SceneExplorerNodeRef currentObjectFrameVisibilityTarget() const;
    void scheduleTransformPreview(
        const robot_qt_viewer::SceneExplorerNodeRef& target,
        const simulation_project::TransformDesc& transform);
    void flushPendingTransformPreview();
    void discardPendingTransformPreview();

    QLabel* m_titleLabel = nullptr;
    QLabel* m_detailsLabel = nullptr;
    QLabel* m_readOnlyTransformTitleLabel = nullptr;
    QLabel* m_readOnlyTransformMatrixLabel = nullptr;
    SceneExplorerMountBindingDiagramWidget* m_mountBindingDiagram = nullptr;
    SceneExplorerObjectFrameDiagramWidget* m_objectFrameDiagram = nullptr;
    QLabel* m_scaleLabel = nullptr;
    QLabel* m_objectNameLabel = nullptr;
    QLabel* m_objectNameValueLabel = nullptr;
    QLabel* m_objectFrameNameLabel = nullptr;
    QLineEdit* m_objectFrameNameEdit = nullptr;
    QCheckBox* m_objectFrameVisibleCheck = nullptr;
    ToolTransformEditorWidget* m_transformEditor = nullptr;
    QPushButton* m_applyTransformButton = nullptr;
    QPushButton* m_cancelTransformButton = nullptr;
    QTimer* m_previewTimer = nullptr;
    robot_qt_viewer::SceneExplorerNodeRef m_transformTarget;
    robot_qt_viewer::SceneExplorerNodeRef m_objectFrameVisibilityTarget;
    robot_qt_viewer::SceneExplorerNodeRef m_pendingPreviewTarget;
    simulation_project::TransformDesc m_pendingPreviewTransform;
    bool m_objectFrameEditorVisible = false;
    bool m_objectFrameVisibilityControlVisible = false;
    bool m_hasPendingPreview = false;
};
