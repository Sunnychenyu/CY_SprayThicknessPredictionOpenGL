#pragma once

#include "ToolSetupViewModel.h"

#include <QVector>
#include <QWidget>

class QCheckBox;
class QComboBox;
class QFrame;
class QLabel;
class QLineEdit;
class ObjectBindingDiagramWidget;
class QPushButton;
class ToolAssetEditorWidget;
class ToolTransformEditorWidget;

class ToolSetupWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ToolSetupWidget(QWidget* parent = nullptr);

    void setDocumentView(const ToolSetupPanelView& view);
    void setViewModel(const ToolSetupPanelView& view);

    void setMountItems(const QVector<ToolSetupComboItem>& items, const QString& selectedId, bool enabled);
    QString currentMountId() const;
    QString mountIdAt(int index) const;
    void setMountDetails(const QString& text);
    void setMountFrameNameEditor(bool visible, bool enabled, const QString& name);
    QString currentMountFrameName() const;
    void setMountLinkItems(const QVector<ToolSetupComboItem>& items, const QString& selectedLinkName, bool enabled);
    QString currentMountLinkName() const;
    void setMountActionEnabled(bool addEnabled, bool deleteEnabled);
    void setMountTransformEditor(
        bool visible,
        bool enabled,
        const QString& title,
        const simulation_project::TransformDesc& transform);
    void setRobotMountFramePinned(bool enabled, bool checked);
    bool hasMountTransformEditor() const;
    simulation_project::TransformDesc currentMountTransform() const;

    void setAttachmentItems(const QVector<ToolSetupComboItem>& items, const QString& selectedId, bool enabled);
    QString currentAttachmentId() const;
    QString attachmentIdAt(int index) const;
    int attachmentCount() const;
    void setAttachmentDetails(const QString& text, const QString& toolTip = QString());
    void setConfigureAttachmentEnabled(bool enabled);
    void setAttachmentInstanceEditor(
        bool visible,
        bool enabled,
        const simulation_project::MountedAttachmentDesc& attachment,
        const QVector<ToolSetupComboItem>& assetItems);
    simulation_project::MountedAttachmentDesc attachmentInstance() const;
    bool hasAttachmentInstanceEditor() const;
    void focusAttachmentInstanceEditor();
    void setAttachmentOffsetEditor(
        bool visible,
        bool enabled,
        const QString& title,
        const simulation_project::TransformDesc& transform);
    bool hasAttachmentOffsetEditor() const;
    simulation_project::TransformDesc currentAttachmentOffset() const;

    void setAssetDetails(const QString& text, const QString& toolTip = QString());
    void setAssetItems(const QVector<ToolSetupComboItem>& items, const QString& selectedId, bool enabled);
    QString currentAssetId() const;
    QString assetIdAt(int index) const;
    void setAttachAssetEnabled(bool enabled);
    void setEditAssetEnabled(bool enabled);
    void setAssetEditor(
        bool visible,
        bool enabled,
        const simulation_project::AttachmentAssetDesc& asset);
    bool hasAssetEditor() const;
    simulation_project::AttachmentAssetDesc currentAssetEditorAsset() const;
    void focusAssetEditor();
    void setObjectBindingEditor(
        bool visible,
        const ToolSetupBindingView& bindingView,
        const QVector<ToolSetupComboItem>& mountItems,
        const QString& selectedMountId,
        const QVector<ToolSetupComboItem>& objectItems,
        const QString& selectedObjectId,
        const QVector<ToolSetupComboItem>& frameItems,
        const QString& selectedFrameId,
        const QString& details,
        bool applyEnabled);
    QString currentBindingMountId() const;
    QString currentBindingObjectId() const;
    QString currentBindingFrameId() const;
    void setMountFrameMode(ToolSetupMountFrameMode mode);
    void setFrameEditorMode(bool newMountFrame);
    void setObjectBindingMode(bool enabled);
    void setTaskDirty(bool dirty, const QString& message = QString());

    bool showLinkFrame() const;
    bool showRobotMountFrame() const;
    bool showToolMountFrame() const;
    bool showVisualFrame() const;
    bool showTcpFrame() const;
    bool showSensorPreview() const;

signals:
    void mountSelectionChanged(int index);
    void addRobotMountRequested();
    void deleteRobotMountRequested();
    void attachmentSelectionChanged(int index);
    void configureAttachmentRequested();
    void importToolAssetRequested();
    void attachToolAssetRequested();
    void assetSelectionChanged(int index);
    void editToolAssetRequested();
    void attachmentOffsetApplyRequested(const simulation_project::TransformDesc& transform);
    void toolAssetApplyRequested(const simulation_project::AttachmentAssetDesc& asset);
    void objectBindingSelectionChanged(
        const QString& mountId,
        const QString& objectId,
        const QString& frameId);
    void objectBindingApplyRequested(
        const QString& mountId,
        const QString& objectId,
        const QString& frameId);
    void taskDirtyChanged(bool dirty);
    void taskApplyRequested(
        bool hasMountTransform,
        const simulation_project::TransformDesc& mountTransform,
        bool hasAttachmentInstance,
        const simulation_project::MountedAttachmentDesc& attachmentInstance,
        bool hasAttachmentOffset,
        const simulation_project::TransformDesc& attachmentOffset,
        bool hasToolAsset,
        const simulation_project::AttachmentAssetDesc& toolAsset);
    void taskCancelRequested();
    void taskExitRequested();
    void mountTransformPreviewChanged(const simulation_project::TransformDesc& transform);
    void frameVisibilityChanged();

private:
    void markTaskDirty();
    void applyFrameEditorLayout();

    QLabel* m_frameEditorTitleLabel = nullptr;
    QLabel* m_taskStatusLabel = nullptr;
    QPushButton* m_applyTaskButton = nullptr;
    QPushButton* m_cancelTaskButton = nullptr;
    QPushButton* m_exitTaskButton = nullptr;
    QComboBox* m_robotMountCombo = nullptr;
    QLabel* m_robotMountDetailsLabel = nullptr;
    QLabel* m_robotMountNameLabel = nullptr;
    QLineEdit* m_robotMountNameEdit = nullptr;
    QComboBox* m_robotMountLinkCombo = nullptr;
    QPushButton* m_addRobotMountButton = nullptr;
    QPushButton* m_deleteRobotMountButton = nullptr;
    ToolTransformEditorWidget* m_robotMountTransformEditor = nullptr;
    QLabel* m_attachmentSectionTitle = nullptr;
    QComboBox* m_toolAttachmentCombo = nullptr;
    QLabel* m_toolDetailsLabel = nullptr;
    QPushButton* m_configureToolAttachmentButton = nullptr;
    QLineEdit* m_attachmentNameEdit = nullptr;
    QComboBox* m_attachmentAssetCombo = nullptr;
    QCheckBox* m_attachmentEnabledCheck = nullptr;
    ToolTransformEditorWidget* m_attachmentOffsetEditor = nullptr;
    QPushButton* m_applyAttachmentOffsetButton = nullptr;
    QLabel* m_assetSectionTitle = nullptr;
    QLabel* m_toolAssetDetailsLabel = nullptr;
    QComboBox* m_toolAssetCombo = nullptr;
    ToolAssetEditorWidget* m_toolAssetEditor = nullptr;
    QPushButton* m_importToolAssetButton = nullptr;
    QPushButton* m_attachToolAssetButton = nullptr;
    QPushButton* m_editToolAssetButton = nullptr;
    QLabel* m_frameVisibilitySectionTitle = nullptr;
    QCheckBox* m_showLinkFrameCheck = nullptr;
    QCheckBox* m_showRobotMountFrameCheck = nullptr;
    QCheckBox* m_showToolMountFrameCheck = nullptr;
    QCheckBox* m_showVisualFrameCheck = nullptr;
    QCheckBox* m_showTcpFrameCheck = nullptr;
    QCheckBox* m_showSensorPreviewCheck = nullptr;
    QLabel* m_objectBindingSectionTitle = nullptr;
    QFrame* m_bindingSummaryFrame = nullptr;
    QLabel* m_bindingNameTitleLabel = nullptr;
    QLabel* m_bindingNameValueLabel = nullptr;
    ObjectBindingDiagramWidget* m_objectBindingDiagram = nullptr;
    QLabel* m_bindingObjectNameLabel = nullptr;
    QLabel* m_bindingObjectFrameLabel = nullptr;
    QLabel* m_objectBindingDetailsLabel = nullptr;
    QComboBox* m_bindingMountCombo = nullptr;
    QComboBox* m_bindingObjectCombo = nullptr;
    QComboBox* m_bindingFrameCombo = nullptr;
    QString m_bindingMountId;
    bool m_attachmentOffsetDirty = false;
    bool m_taskDirty = false;
    bool m_mountTransformEditorVisible = false;
    bool m_attachmentInstanceEditorVisible = false;
    bool m_attachmentOffsetEditorVisible = false;
    bool m_toolAssetEditorVisible = false;
    bool m_objectBindingEditorVisible = false;
    simulation_project::MountedAttachmentDesc m_attachmentEditorAttachment;
};
