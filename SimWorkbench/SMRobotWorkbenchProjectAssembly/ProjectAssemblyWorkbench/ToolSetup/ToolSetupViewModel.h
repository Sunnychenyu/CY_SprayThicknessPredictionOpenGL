#pragma once

#include <QVector>
#include <QString>

#include <SimulationProject/ProjectDocument.h>

struct ToolSetupComboItem
{
    QString text;
    QString id;
};

struct ToolSetupBindingView
{
    bool visible = false;
    bool editable = false;
    bool hasObjectBinding = false;
    QString bindingName;
    QString mountLinkName;
    QString mountFrameName;
    QString objectFrameName;
    QString objectName;
    QString mountTransformText;
    QString objectFrameInverseTransformText;
};

enum class ToolSetupMountFrameMode
{
    Selection,
    Create,
    Edit
};

struct ToolSetupPanelView
{
    ToolSetupMountFrameMode mountFrameMode = ToolSetupMountFrameMode::Selection;
    QVector<ToolSetupComboItem> mountItems;
    QString selectedMountId;
    bool mountItemsEnabled = false;
    QString mountDetails;
    QString mountFrameName;
    bool mountFrameNameEditorVisible = false;
    bool mountFrameNameEditorEnabled = false;
    QVector<ToolSetupComboItem> mountLinkItems;
    QString selectedMountLinkName;
    bool mountLinkItemsEnabled = false;
    bool addMountEnabled = false;
    bool deleteMountEnabled = false;
    bool mountTransformEditorVisible = false;
    bool mountTransformEditorEnabled = false;
    QString mountTransformEditorTitle;
    simulation_project::TransformDesc mountTransform;
    bool robotMountFramePinnedEnabled = false;
    bool robotMountFramePinned = false;

    QVector<ToolSetupComboItem> attachmentItems;
    QString selectedAttachmentId;
    bool attachmentItemsEnabled = false;
    QString attachmentDetails;
    QString attachmentToolTip;
    bool configureAttachmentEnabled = false;
    bool attachmentInstanceEditorVisible = false;
    bool attachmentInstanceEditorEnabled = false;
    simulation_project::MountedAttachmentDesc attachmentInstanceEditorAttachment;
    QVector<ToolSetupComboItem> attachmentAssetItems;
    bool attachmentOffsetEditorVisible = false;
    bool attachmentOffsetEditorEnabled = false;
    QString attachmentOffsetEditorTitle;
    simulation_project::TransformDesc attachmentOffset;

    QString assetDetails;
    QString assetToolTip;
    QVector<ToolSetupComboItem> assetItems;
    QString selectedAssetId;
    bool assetItemsEnabled = false;
    bool attachAssetEnabled = false;
    bool editAssetEnabled = false;
    bool assetEditorVisible = false;
    bool assetEditorEnabled = false;
    simulation_project::AttachmentAssetDesc assetEditorAsset;
    ToolSetupBindingView bindingView;
};

