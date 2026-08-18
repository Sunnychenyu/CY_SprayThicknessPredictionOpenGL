#include "ToolSetupViewModelBuilder.h"

#include <SimulationProject/ProjectDocument.h>

#include <Eigen/Geometry>

#include <QStringList>

#include <algorithm>
#include <string>
#include <vector>

namespace
{
    const simulation_project::RobotDesc* findRobotDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId)
    {
        auto it = std::find_if(
            document.robots.begin(),
            document.robots.end(),
            [&](const simulation_project::RobotDesc& robot) {
                return robot.id == robotId;
            });
        return it == document.robots.end() ? nullptr : &(*it);
    }

    const simulation_project::RobotMountDesc* findRobotMountDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& mountId)
    {
        auto it = std::find_if(
            document.robotMounts.begin(),
            document.robotMounts.end(),
            [&](const simulation_project::RobotMountDesc& mount) {
                return mount.id == mountId;
            });
        return it == document.robotMounts.end() ? nullptr : &(*it);
    }

    const simulation_project::AttachmentAssetDesc* findAttachmentAssetDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& assetId)
    {
        auto it = std::find_if(
            document.attachmentAssets.begin(),
            document.attachmentAssets.end(),
            [&](const simulation_project::AttachmentAssetDesc& asset) {
                return asset.id == assetId;
            });
        return it == document.attachmentAssets.end() ? nullptr : &(*it);
    }

    QString conciseUiText(const std::string& text, int maxChars)
    {
        QString value = QString::fromStdString(text);
        if(maxChars > 3 && value.size() > maxChars) {
            value = value.left(maxChars - 3) + "...";
        }
        return value;
    }

    bool hasAttachmentAsset(const simulation_project::ProjectDocument& document)
    {
        return !document.attachmentAssets.empty();
    }

    struct ToolAttachmentView
    {
        std::string id;
        std::string name;
        std::string mountFrameId;
        std::string assetId;
        std::string linkName;
        std::string visualPath;
        std::string sourceObjectId;
        std::string sourceObjectFrameId;
        bool enabled = false;
        bool visible = false;
        std::string assetKind;
        std::string assetType;
        simulation_project::TransformDesc mountToAssetMount;
    };

    void appendToolAttachmentView(
        std::vector<ToolAttachmentView>& tools,
        const simulation_project::MountedAttachmentDesc& attachment,
        const simulation_project::RobotMountDesc& mount,
        const simulation_project::AttachmentAssetDesc* asset)
    {
        ToolAttachmentView view;
        view.id = attachment.id;
        view.name = attachment.name;
        view.mountFrameId = attachment.mountFrameId;
        view.assetId = attachment.assetId;
        view.linkName = mount.linkName;
        view.visualPath = asset != nullptr ? asset->visualPath : std::string();
        view.sourceObjectId = attachment.sourceObjectId;
        view.sourceObjectFrameId = attachment.sourceObjectFrameId;
        view.enabled = attachment.enabled;
        view.visible = attachment.visible;
        view.mountToAssetMount = attachment.mountToAssetMount;
        if(asset != nullptr) {
            view.assetKind = asset->assetKind;
            view.assetType = asset->assetType;
        }
        tools.push_back(std::move(view));
    }

    std::vector<ToolAttachmentView> buildToolAttachmentViews(
        const simulation_project::ProjectDocument& document,
        const simulation_project::RobotMountDesc* selectedMount)
    {
        std::vector<ToolAttachmentView> tools;
        if(selectedMount == nullptr) {
            return tools;
        }

        for(const simulation_project::MountedAttachmentDesc& attachment : document.mountedAttachments) {
            if(attachment.mountFrameId != selectedMount->id) {
                continue;
            }
            const simulation_project::AttachmentAssetDesc* asset =
                findAttachmentAssetDesc(document, attachment.assetId);
            if(asset == nullptr) {
                continue;
            }
            appendToolAttachmentView(tools, attachment, *selectedMount, asset);
        }
        return tools;
    }

    bool hasMountedAttachmentForMount(
        const simulation_project::ProjectDocument& document,
        const std::string& mountId)
    {
        return std::any_of(
            document.mountedAttachments.begin(),
            document.mountedAttachments.end(),
            [&](const simulation_project::MountedAttachmentDesc& attachment) {
                return attachment.mountFrameId == mountId;
            });
    }

    bool hasRobotMountForLink(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId,
        const std::string& linkName)
    {
        return std::any_of(
            document.robotMounts.begin(),
            document.robotMounts.end(),
            [&](const simulation_project::RobotMountDesc& mount) {
                return mount.robotId == robotId && mount.linkName == linkName;
            });
    }

    const simulation_project::SceneObjectDesc* findSceneObjectDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& objectId)
    {
        auto it = std::find_if(
            document.objects.begin(),
            document.objects.end(),
            [&](const simulation_project::SceneObjectDesc& object) {
                return object.id == objectId;
            });
        return it == document.objects.end() ? nullptr : &(*it);
    }

    const simulation_project::ObjectFrameDesc* findObjectFrameDesc(
        const simulation_project::SceneObjectDesc& object,
        const std::string& frameId)
    {
        auto it = std::find_if(
            object.objectFrames.begin(),
            object.objectFrames.end(),
            [&](const simulation_project::ObjectFrameDesc& frame) {
                return frame.id == frameId;
            });
        return it == object.objectFrames.end() ? nullptr : &(*it);
    }

    Eigen::Isometry3d makeTransform(const simulation_project::TransformDesc& desc)
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.translate(Eigen::Vector3d(desc.x, desc.y, desc.z));
        const Eigen::AngleAxisd yaw(desc.yaw, Eigen::Vector3d::UnitZ());
        const Eigen::AngleAxisd pitch(desc.pitch, Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd roll(desc.roll, Eigen::Vector3d::UnitX());
        transform.rotate(yaw * pitch * roll);
        return transform;
    }

    simulation_project::TransformDesc makeTransformDesc(const Eigen::Isometry3d& transform)
    {
        simulation_project::TransformDesc desc;
        const Eigen::Vector3d translation = transform.translation();
        desc.x = translation.x();
        desc.y = translation.y();
        desc.z = translation.z();
        const Eigen::Vector3d euler = transform.rotation().eulerAngles(2, 1, 0);
        desc.yaw = euler[0];
        desc.pitch = euler[1];
        desc.roll = euler[2];
        return desc;
    }

    simulation_project::TransformDesc inverseTransform(
        const simulation_project::TransformDesc& transform)
    {
        return makeTransformDesc(makeTransform(transform).inverse());
    }

    QString formatTransformMatrix(const simulation_project::TransformDesc& transform)
    {
        const Eigen::Matrix4d matrix = makeTransform(transform).matrix();
        QStringList rows;
        for(int row = 0; row < 4; ++row) {
            QStringList values;
            for(int column = 0; column < 4; ++column) {
                values.push_back(QString::number(matrix(row, column), 'f', 3));
            }
            rows.push_back(QStringLiteral("[ %1 ]").arg(values.join(QStringLiteral("  "))));
        }
        return rows.join(QLatin1Char('\n'));
    }
}

ToolSetupPanelView buildToolSetupPanelView(
    const simulation_project::ProjectDocument& document,
    const QString& selectedRobotId,
    const QString& selectedLinkName,
    const QString& preferredMountId,
    const QString& previousMountId,
    const QString& preferredAttachmentId,
    const QString& previousAttachmentId,
    const QString& preferredAssetId,
    const QString& previousAssetId)
{
    ToolSetupPanelView view;

    const std::string robotIdValue = selectedRobotId.toStdString();
    const bool hasRobotContext =
        !selectedRobotId.isEmpty() &&
        findRobotDesc(document, robotIdValue) != nullptr;

    std::vector<const simulation_project::RobotMountDesc*> mounts;
    mounts.reserve(document.robotMounts.size());
    if(hasRobotContext) {
        for(const simulation_project::RobotMountDesc& mount : document.robotMounts) {
            if(mount.robotId == robotIdValue) {
                mounts.push_back(&mount);
            }
        }
    }

    QString selectedMountId = preferredMountId;
    const auto mountMatches = [&](const QString& mountId) {
        return std::any_of(
            mounts.begin(),
            mounts.end(),
            [&](const simulation_project::RobotMountDesc* mount) {
                return mount != nullptr && QString::fromStdString(mount->id) == mountId;
            });
    };

    if(!mountMatches(selectedMountId)) {
        selectedMountId = previousMountId;
    }
    if(!mountMatches(selectedMountId)) {
        selectedMountId.clear();
        if(!selectedLinkName.isEmpty()) {
            for(const simulation_project::RobotMountDesc* mount : mounts) {
                if(mount != nullptr && QString::fromStdString(mount->linkName) == selectedLinkName) {
                    selectedMountId = QString::fromStdString(mount->id);
                    break;
                }
            }
        }
        for(const simulation_project::MountedAttachmentDesc& attachment : document.mountedAttachments) {
            if(!selectedMountId.isEmpty()) {
                break;
            }
            const simulation_project::RobotMountDesc* mount =
                findRobotMountDesc(document, attachment.mountFrameId);
            if(attachment.enabled &&
                attachment.visible &&
                mount != nullptr &&
                mount->robotId == robotIdValue) {
                selectedMountId = QString::fromStdString(attachment.mountFrameId);
                break;
            }
        }
        if(selectedMountId.isEmpty()) {
            for(const simulation_project::MountedAttachmentDesc& attachment : document.mountedAttachments) {
                const simulation_project::AttachmentAssetDesc* asset =
                    findAttachmentAssetDesc(document, attachment.assetId);
                const simulation_project::RobotMountDesc* mount =
                    findRobotMountDesc(document, attachment.mountFrameId);
                if(asset != nullptr &&
                    attachment.enabled &&
                    attachment.visible &&
                    mount != nullptr &&
                    mount->robotId == robotIdValue) {
                    selectedMountId = QString::fromStdString(attachment.mountFrameId);
                    break;
                }
            }
        }
        if(selectedMountId.isEmpty() && !mounts.empty() && mounts.front() != nullptr) {
            selectedMountId = QString::fromStdString(mounts.front()->id);
        }
    }

    view.mountItems.reserve(static_cast<int>(mounts.size()));
    for(const simulation_project::RobotMountDesc* mount : mounts) {
        if(mount == nullptr) {
            continue;
        }

        ToolSetupComboItem item;
        item.text = QString::fromStdString(mount->name.empty() ? mount->id : mount->name);
        item.text += QString(" [%1]").arg(QString::fromStdString(mount->linkName));
        item.id = QString::fromStdString(mount->id);
        view.mountItems.push_back(item);
    }
    view.selectedMountId = selectedMountId;
    view.mountItemsEnabled = hasRobotContext;

    const simulation_project::RobotMountDesc* selectedMount =
        findRobotMountDesc(document, selectedMountId.toStdString());
    if(selectedMount != nullptr && selectedMount->robotId != robotIdValue) {
        selectedMount = nullptr;
    }

    for(const simulation_project::AttachmentAssetDesc& asset : document.attachmentAssets) {
        ToolSetupComboItem item;
        item.text = QString::fromStdString(asset.name.empty() ? asset.id : asset.name);
        if(!asset.assetKind.empty()) {
            item.text += QString(" (%1)").arg(QString::fromStdString(asset.assetKind));
        }
        if(!asset.assetType.empty()) {
            item.text += QString(" [%1]").arg(QString::fromStdString(asset.assetType));
        }
        item.id = QString::fromStdString(asset.id);
        view.assetItems.push_back(item);
    }
    view.assetItemsEnabled = !view.assetItems.empty();
    const auto assetMatches = [&](const QString& assetId) {
        return std::any_of(
            view.assetItems.begin(),
            view.assetItems.end(),
            [&](const ToolSetupComboItem& item) {
                return item.id == assetId;
            });
    };

    if(!hasRobotContext) {
        view.mountDetails = "Select a robot in Scene Explorer.";
    } else if(selectedMount != nullptr) {
        view.mountDetails = QString("mount frame: %1\nrobot: %2\nlink: %3")
            .arg(QString::fromStdString(selectedMount->id))
            .arg(QString::fromStdString(selectedMount->robotId))
            .arg(QString::fromStdString(selectedMount->linkName));
        view.mountFrameName = QString::fromStdString(selectedMount->name.empty()
            ? selectedMount->id
            : selectedMount->name);
        view.mountFrameNameEditorVisible = true;
        view.mountFrameNameEditorEnabled = true;
        if(!selectedRobotId.isEmpty() && !selectedLinkName.isEmpty()) {
            view.mountDetails += QString("\nselected link: %1.%2").arg(selectedRobotId, selectedLinkName);
        }
        view.mountTransformEditorVisible = false;
        view.mountTransformEditorEnabled = true;
        view.mountTransformEditorTitle = "Frame Transform";
        view.mountTransform = selectedMount->linkToMount;
        view.bindingView.visible = true;
        view.bindingView.editable = false;
        view.bindingView.hasObjectBinding = false;
        view.bindingView.bindingName = QStringLiteral("No bound object");
        view.bindingView.mountLinkName = QString::fromStdString(selectedMount->linkName);
        view.bindingView.mountFrameName = view.mountFrameName;
        view.bindingView.mountTransformText = formatTransformMatrix(selectedMount->linkToMount);

        ToolSetupComboItem currentLinkItem;
        currentLinkItem.text = QString("Current mount link: %1").arg(QString::fromStdString(selectedMount->linkName));
        currentLinkItem.id = QString::fromStdString(selectedMount->linkName);
        view.mountLinkItems.push_back(currentLinkItem);
        if(!selectedLinkName.isEmpty() &&
            selectedRobotId == QString::fromStdString(selectedMount->robotId) &&
            selectedLinkName != currentLinkItem.id) {
            ToolSetupComboItem selectedLinkItem;
            selectedLinkItem.text = QString("Selected link: %1").arg(selectedLinkName);
            selectedLinkItem.id = selectedLinkName;
            view.mountLinkItems.push_back(selectedLinkItem);
        }
        view.selectedMountLinkName = currentLinkItem.id;
        view.mountLinkItemsEnabled = view.mountLinkItems.size() > 1;
        view.deleteMountEnabled = !hasMountedAttachmentForMount(document, selectedMount->id);
    } else {
        view.mountDetails = QString("No mount frame for %1").arg(selectedRobotId);
    }
    view.addMountEnabled =
        hasRobotContext &&
        !selectedLinkName.isEmpty() &&
        !hasRobotMountForLink(document, robotIdValue, selectedLinkName.toStdString());

    const std::vector<ToolAttachmentView> tools = buildToolAttachmentViews(document, selectedMount);
    int preferredActiveIndex = -1;
    int rememberedActiveIndex = -1;
    int enabledIndex = -1;
    int visibleIndex = -1;
    view.attachmentItems.reserve(static_cast<int>(tools.size()));
    for(int i = 0; i < static_cast<int>(tools.size()); ++i) {
        const ToolAttachmentView& tool = tools[static_cast<std::size_t>(i)];
        QString text = QString::fromStdString(tool.name.empty() ? tool.id : tool.name);
        if(!tool.name.empty() && tool.name != tool.id) {
            text += QString(" [%1]").arg(conciseUiText(tool.id, 28));
        }
        if(!tool.enabled) {
            text += " [disabled]";
        }
        if(!tool.visible) {
            text += " [hidden]";
        }

        ToolSetupComboItem item;
        item.text = text;
        item.id = QString::fromStdString(tool.id);
        view.attachmentItems.push_back(item);
        if(QString::fromStdString(tool.id) == preferredAttachmentId) {
            preferredActiveIndex = i;
        }
        if(QString::fromStdString(tool.id) == previousAttachmentId && tool.enabled && tool.visible) {
            rememberedActiveIndex = i;
        }
        if(enabledIndex < 0 && tool.enabled && tool.visible) {
            enabledIndex = i;
        }
        if(visibleIndex < 0 && tool.visible) {
            visibleIndex = i;
        }
    }

    const int activeIndex = preferredActiveIndex >= 0
        ? preferredActiveIndex
        : (rememberedActiveIndex >= 0 ? rememberedActiveIndex : (enabledIndex >= 0 ? enabledIndex : visibleIndex));

    view.selectedAttachmentId =
        activeIndex >= 0 && activeIndex < static_cast<int>(tools.size())
            ? QString::fromStdString(tools[static_cast<std::size_t>(activeIndex)].id)
            : QString();
    const QString activeAssetId =
        activeIndex >= 0 && activeIndex < static_cast<int>(tools.size())
            ? QString::fromStdString(tools[static_cast<std::size_t>(activeIndex)].assetId)
            : QString();
    view.selectedAssetId = preferredAssetId;
    if(!assetMatches(view.selectedAssetId)) {
        view.selectedAssetId = activeAssetId;
    }
    if(!assetMatches(view.selectedAssetId)) {
        view.selectedAssetId = previousAssetId;
    }
    if(!assetMatches(view.selectedAssetId) && !view.assetItems.empty()) {
        view.selectedAssetId = view.assetItems.front().id;
    }
    view.attachmentItemsEnabled = !view.attachmentItems.empty();
    view.configureAttachmentEnabled = !view.attachmentItems.empty();
    view.attachAssetEnabled =
        hasRobotContext &&
        selectedMount != nullptr &&
        hasAttachmentAsset(document);

    if(!view.attachmentItems.empty() && activeIndex >= 0 && activeIndex < static_cast<int>(tools.size())) {
        const ToolAttachmentView& tool = tools[static_cast<std::size_t>(activeIndex)];
        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(document, tool.mountFrameId);
        const simulation_project::AttachmentAssetDesc* asset =
            findAttachmentAssetDesc(document, tool.assetId);
        const std::string assetName = asset != nullptr
            ? (asset->name.empty() ? asset->id : asset->name)
            : tool.assetId;
        const std::string mountName = mount != nullptr
            ? (mount->name.empty() ? mount->id : mount->name)
            : tool.mountFrameId;

        view.attachmentDetails = QString("name: %1\nkind: %2\nmount: %3 [%4]\nasset: %5\nlink: %6\nvisual: %7\nid: %8")
            .arg(QString::fromStdString(tool.name.empty() ? tool.id : tool.name))
            .arg(QString::fromStdString(tool.assetKind.empty() ? std::string("attachment") : tool.assetKind))
            .arg(QString::fromStdString(mountName))
            .arg(QString::fromStdString(tool.linkName))
            .arg(QString::fromStdString(assetName))
            .arg(QString::fromStdString(tool.linkName))
            .arg(conciseUiText(tool.visualPath, 72))
            .arg(conciseUiText(tool.id, 48));
        view.attachmentToolTip = QString("id: %1\nasset id: %2\nmount id: %3\nvisual: %4")
            .arg(QString::fromStdString(tool.id))
            .arg(QString::fromStdString(tool.assetId))
            .arg(QString::fromStdString(tool.mountFrameId))
            .arg(QString::fromStdString(tool.visualPath));
        if(!tool.sourceObjectId.empty()) {
            const simulation_project::SceneObjectDesc* sourceObject =
                findSceneObjectDesc(document, tool.sourceObjectId);
            const std::string objectName =
                sourceObject != nullptr
                    ? (sourceObject->name.empty() ? sourceObject->id : sourceObject->name)
                    : tool.sourceObjectId;
            std::string frameName = "Object origin";
            if(sourceObject != nullptr && !tool.sourceObjectFrameId.empty()) {
                const simulation_project::ObjectFrameDesc* frame =
                    findObjectFrameDesc(*sourceObject, tool.sourceObjectFrameId);
                frameName = frame != nullptr
                    ? (frame->name.empty() ? frame->id : frame->name)
                    : tool.sourceObjectFrameId;
            }
            view.attachmentDetails += QString("\nsource object: %1")
                .arg(QString::fromStdString(tool.sourceObjectId));
            view.attachmentToolTip += QString("\nsource object: %1")
                .arg(QString::fromStdString(tool.sourceObjectId));
            if(!tool.sourceObjectFrameId.empty()) {
                view.attachmentDetails += QString("\nsource frame: %1")
                    .arg(QString::fromStdString(tool.sourceObjectFrameId));
                view.attachmentToolTip += QString("\nsource frame: %1")
                    .arg(QString::fromStdString(tool.sourceObjectFrameId));
            }
            view.mountFrameNameEditorVisible = false;
            view.mountFrameNameEditorEnabled = false;
            view.mountTransformEditorVisible = false;
            view.mountTransformEditorEnabled = false;
            view.attachmentInstanceEditorVisible = false;
            view.attachmentInstanceEditorEnabled = false;
            view.configureAttachmentEnabled = false;
            view.assetEditorVisible = false;
            view.assetEditorEnabled = false;
            view.editAssetEnabled = false;
            view.bindingView.visible = true;
            view.bindingView.editable = false;
            view.bindingView.hasObjectBinding = true;
            view.bindingView.bindingName = QString::fromStdString(tool.name.empty() ? tool.id : tool.name);
            view.bindingView.mountLinkName = QString::fromStdString(tool.linkName);
            view.bindingView.mountFrameName = QString::fromStdString(mountName);
            view.bindingView.objectFrameName = QString::fromStdString(frameName);
            view.bindingView.objectName = QString::fromStdString(objectName);
            if(mount != nullptr) {
                view.bindingView.mountTransformText = formatTransformMatrix(mount->linkToMount);
            }
            if(sourceObject != nullptr && !tool.sourceObjectFrameId.empty()) {
                const simulation_project::ObjectFrameDesc* frame =
                    findObjectFrameDesc(*sourceObject, tool.sourceObjectFrameId);
                if(frame != nullptr) {
                    view.bindingView.objectFrameInverseTransformText =
                        formatTransformMatrix(inverseTransform(frame->objectToFrame));
                }
            }
            if(view.bindingView.objectFrameInverseTransformText.isEmpty()) {
                view.bindingView.objectFrameInverseTransformText =
                    formatTransformMatrix(simulation_project::TransformDesc());
            }
        }
        for(const simulation_project::AttachmentAssetDesc& candidateAsset : document.attachmentAssets) {
            ToolSetupComboItem item;
            item.text = QString::fromStdString(candidateAsset.name.empty() ? candidateAsset.id : candidateAsset.name);
            if(!candidateAsset.assetKind.empty()) {
                item.text += QString(" (%1)").arg(QString::fromStdString(candidateAsset.assetKind));
            }
            if(!candidateAsset.assetType.empty()) {
                item.text += QString(" [%1]").arg(QString::fromStdString(candidateAsset.assetType));
            }
            item.id = QString::fromStdString(candidateAsset.id);
            view.attachmentAssetItems.push_back(item);
        }
        view.attachmentInstanceEditorVisible = true;
        view.attachmentInstanceEditorEnabled = true;
        view.attachmentInstanceEditorAttachment.id = tool.id;
        view.attachmentInstanceEditorAttachment.name = tool.name;
        view.attachmentInstanceEditorAttachment.mountFrameId = tool.mountFrameId;
        view.attachmentInstanceEditorAttachment.assetId = tool.assetId;
        view.attachmentInstanceEditorAttachment.sourceObjectId = tool.sourceObjectId;
        view.attachmentInstanceEditorAttachment.sourceObjectFrameId = tool.sourceObjectFrameId;
        view.attachmentInstanceEditorAttachment.mountToAssetMount = tool.mountToAssetMount;
        view.attachmentInstanceEditorAttachment.visible = tool.visible;
        view.attachmentInstanceEditorAttachment.enabled = tool.enabled;
        if(tool.sourceObjectId.empty()) {
            view.attachmentOffsetEditorVisible = true;
            view.attachmentOffsetEditorEnabled = true;
            view.attachmentOffsetEditorTitle = "Attachment offset: Mount -> Asset mount";
            view.attachmentOffset = tool.mountToAssetMount;
        } else {
            view.attachmentInstanceEditorVisible = false;
            view.attachmentInstanceEditorEnabled = false;
        }

        if(asset != nullptr) {
            view.assetDetails = QString("name: %1\nkind: %2\ntype: %3\nvisual: %4\nid: %5")
                .arg(QString::fromStdString(asset->name.empty() ? asset->id : asset->name))
                .arg(QString::fromStdString(asset->assetKind))
                .arg(QString::fromStdString(asset->assetType))
                .arg(conciseUiText(asset->visualPath, 72))
                .arg(conciseUiText(asset->id, 48));
            view.assetToolTip = QString("id: %1\nvisual: %2")
                .arg(QString::fromStdString(asset->id))
                .arg(QString::fromStdString(asset->visualPath));
            if(tool.sourceObjectId.empty()) {
                view.editAssetEnabled = true;
                view.assetEditorVisible = true;
                view.assetEditorEnabled = true;
                view.assetEditorAsset = *asset;
            }
        } else {
            view.assetDetails = "No attachment asset";
            view.editAssetEnabled = false;
        }
    } else {
        view.attachmentDetails = hasRobotContext
            ? "No attachment"
            : "Select a robot in Scene Explorer.";
        view.assetDetails = hasRobotContext
            ? "No attachment asset"
            : "Select a robot in Scene Explorer.";
        view.editAssetEnabled = false;
    }

    const simulation_project::AttachmentAssetDesc* selectedAsset =
        findAttachmentAssetDesc(document, view.selectedAssetId.toStdString());
    if(selectedAsset != nullptr) {
        view.assetDetails = QString("name: %1\nkind: %2\ntype: %3\nvisual: %4\nid: %5")
            .arg(QString::fromStdString(selectedAsset->name.empty() ? selectedAsset->id : selectedAsset->name))
            .arg(QString::fromStdString(selectedAsset->assetKind))
            .arg(QString::fromStdString(selectedAsset->assetType))
            .arg(conciseUiText(selectedAsset->visualPath, 72))
            .arg(conciseUiText(selectedAsset->id, 48));
        view.assetToolTip = QString("id: %1\nvisual: %2")
            .arg(QString::fromStdString(selectedAsset->id))
            .arg(QString::fromStdString(selectedAsset->visualPath));
        view.editAssetEnabled = true;
        view.assetEditorVisible = true;
        view.assetEditorEnabled = true;
        view.assetEditorAsset = *selectedAsset;
    } else if(!hasRobotContext) {
        view.assetDetails = "Select a robot in Scene Explorer.";
        view.editAssetEnabled = false;
        view.assetEditorVisible = false;
        view.assetEditorEnabled = false;
    } else {
        view.assetDetails = "No attachment asset";
        view.editAssetEnabled = false;
        view.assetEditorVisible = false;
        view.assetEditorEnabled = false;
    }

    return view;
}

