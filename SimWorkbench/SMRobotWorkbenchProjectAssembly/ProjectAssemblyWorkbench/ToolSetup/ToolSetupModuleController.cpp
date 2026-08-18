#include "ToolSetupModuleController.h"

#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerSelectionModel.h"
#include "RobotQtViewerViewportPreviewState.h"
#include "ToolAttachmentCommandController.h"
#include "ToolSetupAppServices.h"
#include "ToolSetupViewModelBuilder.h"
#include "ToolSetupWidget.h"

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectSession.h>

#include <CustomLog/CustomLog.h>

#include <Eigen/Geometry>

#include <QApplication>
#include <QByteArray>
#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <vector>

namespace
{
    const simulation_project::RobotMountDesc* findRobotMountDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& mountId)
    {
        const auto it = std::find_if(
            document.robotMounts.begin(),
            document.robotMounts.end(),
            [&](const simulation_project::RobotMountDesc& mount) {
                return mount.id == mountId;
            });
        return it == document.robotMounts.end() ? nullptr : &(*it);
    }

    const simulation_project::MountedAttachmentDesc* findMountedAttachmentDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& attachmentId)
    {
        const auto it = std::find_if(
            document.mountedAttachments.begin(),
            document.mountedAttachments.end(),
            [&](const simulation_project::MountedAttachmentDesc& attachment) {
                return attachment.id == attachmentId;
            });
        return it == document.mountedAttachments.end() ? nullptr : &(*it);
    }

    const simulation_project::RobotDesc* findRobotDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId)
    {
        const auto it = std::find_if(
            document.robots.begin(),
            document.robots.end(),
            [&](const simulation_project::RobotDesc& robot) {
                return robot.id == robotId;
            });
        return it == document.robots.end() ? nullptr : &(*it);
    }

    const simulation_project::SceneObjectDesc* findSceneObjectDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& objectId)
    {
        const auto it = std::find_if(
            document.objects.begin(),
            document.objects.end(),
            [&](const simulation_project::SceneObjectDesc& object) {
                return object.id == objectId;
            });
        return it == document.objects.end() ? nullptr : &(*it);
    }

    const simulation_project::AttachmentAssetDesc* findAttachmentAssetDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& assetId)
    {
        const auto it = std::find_if(
            document.attachmentAssets.begin(),
            document.attachmentAssets.end(),
            [&](const simulation_project::AttachmentAssetDesc& asset) {
                return asset.id == assetId;
            });
        return it == document.attachmentAssets.end() ? nullptr : &(*it);
    }

    const simulation_project::MountedAttachmentDesc* findMountedAttachmentByAssetId(
        const simulation_project::ProjectDocument& document,
        const std::string& assetId)
    {
        const auto it = std::find_if(
            document.mountedAttachments.begin(),
            document.mountedAttachments.end(),
            [&](const simulation_project::MountedAttachmentDesc& attachment) {
                return attachment.assetId == assetId;
            });
        return it == document.mountedAttachments.end() ? nullptr : &(*it);
    }

    const simulation_project::MountedAttachmentDesc* findMountedAttachmentByMountId(
        const simulation_project::ProjectDocument& document,
        const std::string& mountId)
    {
        const auto it = std::find_if(
            document.mountedAttachments.begin(),
            document.mountedAttachments.end(),
            [&](const simulation_project::MountedAttachmentDesc& attachment) {
                return attachment.mountFrameId == mountId;
            });
        return it == document.mountedAttachments.end() ? nullptr : &(*it);
    }

    const simulation_project::ObjectFrameDesc* findObjectFrameDesc(
        const simulation_project::SceneObjectDesc& object,
        const std::string& frameId)
    {
        const auto it = std::find_if(
            object.objectFrames.begin(),
            object.objectFrames.end(),
            [&](const simulation_project::ObjectFrameDesc& frame) {
                return frame.id == frameId;
            });
        return it == object.objectFrames.end() ? nullptr : &(*it);
    }

    bool nearlyEqual(double lhs, double rhs)
    {
        return std::abs(lhs - rhs) <= 1.0e-9;
    }

    bool sameTransform(
        const simulation_project::TransformDesc& lhs,
        const simulation_project::TransformDesc& rhs)
    {
        return nearlyEqual(lhs.x, rhs.x) &&
            nearlyEqual(lhs.y, rhs.y) &&
            nearlyEqual(lhs.z, rhs.z) &&
            nearlyEqual(lhs.roll, rhs.roll) &&
            nearlyEqual(lhs.pitch, rhs.pitch) &&
            nearlyEqual(lhs.yaw, rhs.yaw);
    }

    bool sameRobotMount(
        const simulation_project::RobotMountDesc& lhs,
        const simulation_project::RobotMountDesc& rhs)
    {
        return lhs.id == rhs.id &&
            lhs.name == rhs.name &&
            lhs.robotId == rhs.robotId &&
            lhs.linkName == rhs.linkName &&
            sameTransform(lhs.linkToMount, rhs.linkToMount);
    }

    std::string findHiddenSceneObjectIdBySourcePath(
        const simulation_project::ProjectDocument& document,
        const std::string& sourcePath)
    {
        if(sourcePath.empty()) {
            return std::string();
        }
        const auto it = std::find_if(
            document.objects.begin(),
            document.objects.end(),
            [&](const simulation_project::SceneObjectDesc& object) {
                return !object.visible && object.sourcePath == sourcePath;
            });
        return it == document.objects.end() ? std::string() : it->id;
    }

    Eigen::Isometry3d makeTransform(const simulation_project::TransformDesc& desc)
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.translation() = Eigen::Vector3d(desc.x, desc.y, desc.z);
        transform.linear() =
            Eigen::AngleAxisd(desc.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(desc.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(desc.roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        return transform;
    }

    simulation_project::TransformDesc makeTransformDesc(const Eigen::Isometry3d& transform)
    {
        simulation_project::TransformDesc desc;
        desc.x = transform.translation().x();
        desc.y = transform.translation().y();
        desc.z = transform.translation().z();
        const Eigen::Vector3d euler = transform.linear().eulerAngles(2, 1, 0);
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

    QString formatMatrixValue(double value)
    {
        return QString::number(value, 'f', 3);
    }

    QString formatTransformMatrix(const simulation_project::TransformDesc& transform)
    {
        const Eigen::Matrix4d matrix = makeTransform(transform).matrix();
        QStringList rows;
        for(int row = 0; row < 4; ++row) {
            QStringList values;
            for(int column = 0; column < 4; ++column) {
                values.push_back(formatMatrixValue(matrix(row, column)));
            }
            rows.push_back(QStringLiteral("[ %1 ]").arg(values.join(QStringLiteral("  "))));
        }
        return rows.join(QLatin1Char('\n'));
    }

    ToolSetupBindingView makeBindingView(
        const simulation_project::ProjectDocument& document,
        const QString& mountId,
        const QString& objectId,
        const QString& frameId)
    {
        ToolSetupBindingView view;
        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(document, mountId.toStdString());
        const simulation_project::SceneObjectDesc* object =
            findSceneObjectDesc(document, objectId.toStdString());
        view.visible = mount != nullptr;
        if(mount != nullptr) {
            view.mountLinkName = QString::fromStdString(mount->linkName);
            view.mountFrameName = QString::fromStdString(mount->name.empty() ? mount->id : mount->name);
            view.mountTransformText = formatTransformMatrix(mount->linkToMount);
        }
        if(object != nullptr) {
            view.hasObjectBinding = true;
            const std::string objectName = object->name.empty() ? object->id : object->name;
            const std::string mountName =
                mount != nullptr ? (mount->name.empty() ? mount->id : mount->name) : std::string();
            view.objectName = QString::fromStdString(objectName);
            view.bindingName = QString::fromStdString(
                objectName.empty()
                    ? (mountName.empty() ? std::string("Attachment") : mountName + " Attachment")
                    : objectName);
            if(frameId.isEmpty()) {
                view.objectFrameName = QStringLiteral("Object origin");
                view.objectFrameInverseTransformText =
                    formatTransformMatrix(simulation_project::TransformDesc());
            } else {
                const simulation_project::ObjectFrameDesc* frame =
                    findObjectFrameDesc(*object, frameId.toStdString());
                if(frame != nullptr) {
                    view.objectFrameName = QString::fromStdString(frame->name.empty() ? frame->id : frame->name);
                    view.objectFrameInverseTransformText = formatTransformMatrix(inverseTransform(frame->objectToFrame));
                }
            }
        } else {
            view.bindingName = QStringLiteral("Pending binding");
            view.hasObjectBinding = false;
            view.objectFrameName = QStringLiteral("Object Frame");
            view.objectName = QStringLiteral("Object");
            view.objectFrameInverseTransformText =
                formatTransformMatrix(simulation_project::TransformDesc());
        }
        return view;
    }

    std::string qStringToUtf8(const QString& text)
    {
        const QByteArray bytes = text.toUtf8();
        return std::string(bytes.constData(), static_cast<size_t>(bytes.size()));
    }

    std::string trimUnderscores(std::string value)
    {
        while(!value.empty() && value.front() == '_') {
            value.erase(value.begin());
        }
        while(!value.empty() && value.back() == '_') {
            value.pop_back();
        }
        return value;
    }

    std::string makeAsciiSlug(
        const std::string& displayName,
        const std::string& fallbackPrefix,
        std::size_t maxLength)
    {
        std::string slug;
        slug.reserve(displayName.size());
        bool lastWasSeparator = false;
        for(const unsigned char ch : displayName) {
            if(ch < 128 && std::isalnum(ch)) {
                slug.push_back(static_cast<char>(std::tolower(ch)));
                lastWasSeparator = false;
            } else if(ch < 128 && (ch == '_' || ch == '-' || ch == '.' || std::isspace(ch))) {
                if(!slug.empty() && !lastWasSeparator) {
                    slug.push_back('_');
                    lastWasSeparator = true;
                }
            }
        }

        slug = trimUnderscores(slug);
        if(slug.empty()) {
            slug = fallbackPrefix;
        }
        if(slug.size() > maxLength) {
            slug.resize(maxLength);
            slug = trimUnderscores(slug);
        }
        return slug.empty() ? fallbackPrefix : slug;
    }

    std::string makeToolAssetDisplayName(
        const std::string& sourceName,
        const std::string& fallbackName)
    {
        return sourceName.empty() ? fallbackName : sourceName;
    }

    std::string makeAttachmentDisplayName(
        const std::string& assetName,
        const std::string& mountName)
    {
        if(!assetName.empty()) {
            return assetName;
        }
        return mountName.empty() ? std::string("Attachment") : mountName + " Attachment";
    }

    QString conciseUiText(const std::string& value, int maxCharacters)
    {
        QString text = QString::fromStdString(value);
        if(maxCharacters <= 8 || text.size() <= maxCharacters) {
            return text;
        }

        const int headCount = (maxCharacters - 3) / 2;
        const int tailCount = maxCharacters - 3 - headCount;
        return text.left(headCount) + "..." + text.right(tailCount);
    }

    simulation_project::AttachmentAssetDesc makeAttachmentAssetMirror(
        const simulation_project::AttachmentAssetDesc& asset,
        const std::string& assetId)
    {
        simulation_project::AttachmentAssetDesc value = asset;
        value.id = assetId;
        if(value.name.empty()) {
            value.name = assetId;
        }
        return value;
    }

    simulation_project::MountedAttachmentDesc makeMountedAttachmentMirror(
        const simulation_project::MountedAttachmentDesc& attachment)
    {
        simulation_project::MountedAttachmentDesc value;
        value.id = attachment.id;
        value.name = attachment.name;
        value.mountFrameId = attachment.mountFrameId;
        value.assetId = attachment.assetId;
        value.sourceObjectId = attachment.sourceObjectId;
        value.sourceObjectFrameId = attachment.sourceObjectFrameId;
        value.mountToAssetMount = attachment.mountToAssetMount;
        value.visible = attachment.visible;
        value.enabled = attachment.enabled;
        return value;
    }

    std::vector<simulation_project::AttachmentAssetDesc> genericAttachmentAssetMirrors(
        const simulation_project::ProjectDocument& document)
    {
        std::vector<simulation_project::AttachmentAssetDesc> assets;
        for(const simulation_project::AttachmentAssetDesc& asset : document.attachmentAssets) {
            assets.push_back(makeAttachmentAssetMirror(asset, asset.id));
        }
        return assets;
    }

    ToolSetupComboItem makeComboItem(const QString& text, const QString& id)
    {
        ToolSetupComboItem item;
        item.text = text;
        item.id = id;
        return item;
    }

}

namespace robot_qt_viewer
{
    ToolSetupModuleController::ToolSetupModuleController(
        ToolSetupWidget& widget,
        RobotQtViewerDocumentContext& context,
        ToolSetupAppServices& appServices,
        QObject* parent)
        : QObject(parent)
        , m_widget(widget)
        , m_context(context)
        , m_appServices(appServices)
    {
        connect(&m_widget, &ToolSetupWidget::mountSelectionChanged,
            this, &ToolSetupModuleController::handleMountSelectionChanged);
        connect(&m_widget, &ToolSetupWidget::attachmentSelectionChanged,
            this, &ToolSetupModuleController::handleAttachmentSelectionChanged);
        connect(&m_widget, &ToolSetupWidget::addRobotMountRequested,
            this, &ToolSetupModuleController::createRobotMountForSelectedLink);
        connect(&m_widget, &ToolSetupWidget::deleteRobotMountRequested,
            this, &ToolSetupModuleController::deleteCurrentRobotMount);
        connect(&m_widget, &ToolSetupWidget::configureAttachmentRequested,
            this, &ToolSetupModuleController::configureCurrentToolAttachment);
        connect(&m_widget, &ToolSetupWidget::importToolAssetRequested,
            this, &ToolSetupModuleController::importToolAsset);
        connect(&m_widget, &ToolSetupWidget::attachToolAssetRequested,
            this, &ToolSetupModuleController::attachExistingToolAsset);
        connect(&m_widget, &ToolSetupWidget::assetSelectionChanged,
            this, &ToolSetupModuleController::handleAssetSelectionChanged);
        connect(&m_widget, &ToolSetupWidget::editToolAssetRequested,
            this, &ToolSetupModuleController::editCurrentToolAsset);
        connect(&m_widget, &ToolSetupWidget::attachmentOffsetApplyRequested,
            this, &ToolSetupModuleController::applyCurrentAttachmentOffset);
        connect(&m_widget, &ToolSetupWidget::toolAssetApplyRequested,
            this, &ToolSetupModuleController::applyCurrentToolAsset);
        connect(&m_widget, &ToolSetupWidget::objectBindingSelectionChanged,
            this, &ToolSetupModuleController::handleObjectBindingSelectionChanged);
        connect(&m_widget, &ToolSetupWidget::objectBindingApplyRequested,
            this, &ToolSetupModuleController::applyObjectBinding);
        connect(&m_widget, &ToolSetupWidget::taskDirtyChanged,
            this, [this](bool dirty) {
                setTaskDirty(dirty);
            });
        connect(&m_widget, &ToolSetupWidget::taskApplyRequested,
            this, &ToolSetupModuleController::applyTaskChanges);
        connect(&m_widget, &ToolSetupWidget::taskCancelRequested,
            this, &ToolSetupModuleController::cancelTaskChanges);
        connect(&m_widget, &ToolSetupWidget::taskExitRequested,
            this, &ToolSetupModuleController::requestTaskExit);
        connect(&m_widget, &ToolSetupWidget::mountTransformPreviewChanged,
            this, [this](const simulation_project::TransformDesc& transform) {
                const QString mountId = m_widget.currentMountId();
                if(mountId.isEmpty()) {
                    return;
                }
                RobotQtViewerViewportPreviewPayload preview;
                preview.previewRobotMountTransform = true;
                preview.previewRobotMountId = mountId;
                preview.robotMountTransform = transform;
                preview.setActivePreviewRobotMount = true;
                preview.activePreviewRobotMountId = mountId;
                mutateViewportPreview(preview, QStringLiteral("toolSetupMountTransformPreview"));
            });
        connect(&m_widget, &ToolSetupWidget::frameVisibilityChanged,
            this, &ToolSetupModuleController::updateToolFrameVisibility);
    }

    void ToolSetupModuleController::refresh(
        const QString& selectedRobotId,
        const QString& selectedLinkName,
        const QString& preferredMountId,
        const QString& preferredAttachmentId,
        const QString& preferredAssetId)
    {
        m_updating = true;
        const QString previousMountId = m_widget.currentMountId();
        const QString previousAttachmentId = m_widget.currentAttachmentId();
        const QString previousAssetId = m_widget.currentAssetId();
        ToolSetupPanelView view = buildToolSetupPanelView(
            m_context.document(),
            selectedRobotId,
            selectedLinkName,
            preferredMountId,
            previousMountId,
            preferredAttachmentId,
            previousAttachmentId,
            preferredAssetId,
            previousAssetId);
        view.mountFrameMode = m_objectBindingTaskActive
            ? ToolSetupMountFrameMode::Selection
            : m_mountFrameMode;
        if(view.mountFrameMode == ToolSetupMountFrameMode::Create ||
            view.mountFrameMode == ToolSetupMountFrameMode::Edit) {
            view.mountFrameNameEditorVisible = true;
            view.mountFrameNameEditorEnabled = true;
            view.mountTransformEditorVisible = true;
            view.mountTransformEditorEnabled = true;
            if(view.mountTransformEditorTitle.isEmpty()) {
                view.mountTransformEditorTitle = QStringLiteral("Frame Transform");
            }
        }
        view.robotMountFramePinnedEnabled = !view.selectedMountId.isEmpty();
        view.robotMountFramePinned =
            view.robotMountFramePinnedEnabled &&
            m_pinnedRobotMountFrameIds.contains(view.selectedMountId);
        m_widget.setDocumentView(view);
        m_updating = false;
        if(!m_taskDirty && !m_objectBindingTaskActive) {
            captureMountEditSnapshot(m_widget.currentMountId());
        }
        syncPinnedRobotMountFrames();
        emit viewModelRefreshed();
    }

    void ToolSetupModuleController::handleEvent(
        const RobotQtViewerEvent& event,
        const QString& selectedRobotId,
        const QString& selectedLinkName)
    {
        if(m_objectBindingTaskActive) {
            if(event.kind == RobotQtViewerEventKind::ViewportReloaded ||
                event.kind == RobotQtViewerEventKind::SelectionChanged ||
                event.kind == RobotQtViewerEventKind::ProjectDocumentChanged) {
                refreshObjectBindingEditor(m_bindingMountId, m_bindingObjectId, m_bindingFrameId);
            }
            return;
        }

        switch(event.kind) {
        case RobotQtViewerEventKind::SelectionChanged:
            refresh(
                selectedRobotId,
                selectedLinkName,
                event.selection.mountId,
                event.selection.attachmentId,
                event.selection.assetId);
            break;
        case RobotQtViewerEventKind::AttachmentChanged:
            refresh(
                selectedRobotId,
                selectedLinkName,
                event.attachment.mountId,
                event.attachment.attachmentId,
                event.attachment.assetId);
            break;
        case RobotQtViewerEventKind::ProjectDocumentChanged:
        case RobotQtViewerEventKind::ViewportReloaded:
            refresh(selectedRobotId, selectedLinkName);
            break;
        default:
            break;
        }
    }

    void ToolSetupModuleController::handleMountSelectionChanged(int index)
    {
        if(m_updating || index < 0) {
            return;
        }

        const QString mountId = m_widget.mountIdAt(index);
        if(m_taskDirty && mountId != m_activeMountEditId && !resolvePendingTaskChanges(&m_widget)) {
            refresh(m_appServices.selectedRobotId(), m_appServices.selectedLinkName(), m_activeMountEditId);
            return;
        }
        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(m_context.document(), mountId.toStdString());
        if(mount != nullptr) {
            m_mountFrameMode = ToolSetupMountFrameMode::Selection;
            const QString robotId = QString::fromStdString(mount->robotId);
            const QString linkName = QString::fromStdString(mount->linkName);
            m_appServices.setRobotMountContext(robotId, linkName, mountId);
            m_context.selectionModel().selectRobotMount(robotId, linkName, mountId, QStringLiteral("toolSetup"));
            RobotQtViewerViewportPreviewPayload preview;
            preview.setActivePreviewRobotMount = true;
            preview.activePreviewRobotMountId = QString();
            preview.setRobotMountFrameVisibility = true;
            preview.selectedLinkFrameVisible = false;
            preview.mountFrameVisible = false;
            mutateViewportPreview(preview, QStringLiteral("toolSetupMountSelection"));
            emit robotContextSelected(robotId);
            emit selectionDependentViewsRefreshRequested();
        }
        if(!mountId.isEmpty()) {
            emit statusMessageRequested(QString("Selected mount frame: %1").arg(mountId), 3000);
        }
        refresh(m_appServices.selectedRobotId(), m_appServices.selectedLinkName(), mountId);
    }

    void ToolSetupModuleController::handleAttachmentSelectionChanged(int index)
    {
        if(m_updating || index < 0) {
            return;
        }

        const QString attachmentId = m_widget.attachmentIdAt(index);
        if(attachmentId.isEmpty()) {
            return;
        }

        const simulation_project::MountedAttachmentDesc* attachment =
            findMountedAttachmentDesc(m_context.document(), attachmentId.toStdString());
        if(attachment == nullptr) {
            emit statusMessageRequested(QString("Attachment not found: %1").arg(attachmentId), 4000);
            return;
        }

        const std::string mountFrameId = attachment->mountFrameId;
        const std::string selectedAttachmentId = attachment->id;
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("toolSetupActiveAttachment"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(!service.setActiveMountedAttachment(mountFrameId, selectedAttachmentId, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            emit statusMessageRequested(QString("Attachment selection failed: %1").arg(mutationResult.message), 5000);
            return;
        }

        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(m_context.document(), attachment->mountFrameId);
        if(mount != nullptr) {
            const QString robotId = QString::fromStdString(mount->robotId);
            const QString linkName = QString::fromStdString(mount->linkName);
            const QString mountId = QString::fromStdString(mount->id);
            m_appServices.setToolAttachmentContext(robotId, linkName, mountId, attachmentId);
            m_context.selectionModel().selectMountedAttachment(
                attachmentId,
                robotId,
                linkName,
                mountId,
                QStringLiteral("toolSetup"));
            emit robotContextSelected(robotId);
            emit selectionDependentViewsRefreshRequested();
        }

        RobotQtViewerAttachmentPayload payload;
        payload.mountId = QString::fromStdString(attachment->mountFrameId);
        payload.attachmentId = attachmentId;
        payload.assetId = QString::fromStdString(attachment->assetId);
        m_context.documentController().publishAttachmentChanged(payload, QStringLiteral("toolSetup"));
        m_context.documentController().publishDocumentChanged(QStringLiteral("toolSetup"), false);
        RobotQtViewerViewportPreviewPayload preview;
        preview.setActiveMountedAttachment = true;
        preview.activeMountedAttachmentId = attachmentId;
        mutateViewportPreview(preview, QStringLiteral("toolSetupAttachmentSelection"));
        emit statusMessageRequested(QString("Active attachment: %1").arg(attachmentId), 3000);
    }

    void ToolSetupModuleController::handleAssetSelectionChanged(int index)
    {
        if(m_updating || index < 0) {
            return;
        }

        const QString assetId = m_widget.assetIdAt(index);
        if(assetId.isEmpty()) {
            return;
        }

        const simulation_project::AttachmentAssetDesc* asset =
            findAttachmentAssetDesc(m_context.document(), assetId.toStdString());
        if(asset == nullptr) {
            emit statusMessageRequested(QString("Attachment asset not found: %1").arg(assetId), 4000);
            return;
        }

        m_context.selectionModel().selectToolAsset(assetId, QStringLiteral("toolSetup"));
        refresh(m_appServices.selectedRobotId(), m_appServices.selectedLinkName(), QString(), QString(), assetId);
        m_widget.focusAssetEditor();
        emit statusMessageRequested(
            QString("Selected attachment asset: %1").arg(conciseUiText(asset->id, 48)),
            3000);
    }

    void ToolSetupModuleController::focusRobotMountTask(
        const QString& robotId,
        const QString& linkName,
        const QString& preferredMountId)
    {
        if(robotId.isEmpty()) {
            emit statusMessageRequested("Select a robot before opening mount setup.", 3000);
            return;
        }

        QString mountId = preferredMountId;
        const std::string robotIdValue = robotId.toStdString();
        const std::string linkNameValue = linkName.toStdString();
        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(m_context.document(), mountId.toStdString());
        if((mount == nullptr || mount->robotId != robotIdValue) && !linkName.isEmpty()) {
            for(const simulation_project::RobotMountDesc& candidate : m_context.document().robotMounts) {
                if(candidate.robotId == robotIdValue && candidate.linkName == linkNameValue) {
                    mount = &candidate;
                    mountId = QString::fromStdString(candidate.id);
                    break;
                }
            }
        }

        if(mount != nullptr && mount->robotId == robotIdValue) {
            m_mountFrameMode = ToolSetupMountFrameMode::Edit;
            const QString mountLinkName = QString::fromStdString(mount->linkName);
            m_appServices.setRobotMountContext(robotId, mountLinkName, mountId);
            m_context.selectionModel().selectRobotMount(
                robotId,
                mountLinkName,
                mountId,
                QStringLiteral("focusRobotMountTask"));
            RobotQtViewerViewportPreviewPayload preview;
            preview.setActivePreviewRobotMount = true;
            preview.activePreviewRobotMountId = mountId;
            preview.setRobotMountFrameVisibility = true;
            preview.selectedLinkFrameVisible = true;
            preview.mountFrameVisible = true;
            mutateViewportPreview(preview, QStringLiteral("focusRobotMountTask"));
            enterMountFrameViewportFocus(robotId, mountLinkName);
            refresh(robotId, mountLinkName, mountId);
            captureMountEditSnapshot(mountId);
            setTaskDirty(false);
            emit mountFrameFocusRequested(robotId, mountLinkName, mountId);
        } else {
            m_mountFrameMode = ToolSetupMountFrameMode::Selection;
            m_appServices.setRobotMountContext(robotId, linkName, QString());
            if(!linkName.isEmpty()) {
                m_context.selectionModel().selectRobotLink(robotId, linkName, QStringLiteral("focusRobotMountTask"));
                RobotQtViewerViewportPreviewPayload preview;
                preview.setActivePreviewRobotMount = true;
                preview.activePreviewRobotMountId = QString();
                preview.setRobotMountFrameVisibility = true;
                preview.selectedLinkFrameVisible = true;
                preview.mountFrameVisible = false;
                mutateViewportPreview(preview, QStringLiteral("focusRobotMountTask"));
                enterMountFrameViewportFocus(robotId, linkName);
            }
            refresh(robotId, linkName);
        }

        emit robotContextSelected(robotId);
        emit selectionDependentViewsRefreshRequested();
        emit statusMessageRequested("Mount setup is ready for mount frame editing.", 3000);
    }

    void ToolSetupModuleController::importToolAsset()
    {
        const QString fileName = QFileDialog::getOpenFileName(
            &m_widget,
            "Import tool model",
            QString(),
            "Tool Meshes (*.stl *.STL *.obj *.OBJ *.dae *.DAE *.ply *.PLY *.gltf *.GLTF *.glb *.GLB *.step *.STEP *.stp *.STP);;All Files (*.*)");

        if(fileName.isEmpty()) {
            return;
        }

        const std::filesystem::path path = fileName.toStdWString();
        const std::string stem = qStringToUtf8(QString::fromStdWString(path.stem().wstring()));
        const simulation_project::ProjectDocument previousDocument = m_context.document();
        const bool previousDirty = m_context.projectSession().isDirty();

        simulation_project::AttachmentAssetDesc asset;
        asset.assetKind = "tool";
        asset.assetType = "tool";
        asset.visualPath = m_context.projectSession().makePortableAssetPath(path);
        asset.visualScale = 1.0;
        asset.visible = true;

        std::string mountId;
        std::string attachmentId;
        const QString currentMountId = m_widget.currentMountId();
        const QString selectedRobotId = m_appServices.selectedRobotId();
        const QString selectedLinkName = m_appServices.selectedLinkName();
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("importToolAsset"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                asset.id = service.makeUniqueId(makeAsciiSlug(stem, "tool_asset", 32));
                asset.name = makeToolAssetDisplayName(stem, asset.id);
                if(!service.addAttachmentAsset(asset, &error)) {
                    return false;
                }

                const simulation_project::RobotMountDesc* existing =
                    findRobotMountDesc(service.document(), currentMountId.toStdString());
                if(existing != nullptr &&
                    (selectedRobotId.isEmpty() || existing->robotId == selectedRobotId.toStdString())) {
                    mountId = existing->id;
                } else if(!selectedRobotId.isEmpty() && !selectedLinkName.isEmpty()) {
                    const std::string robotId = selectedRobotId.toStdString();
                    const std::string linkName = selectedLinkName.toStdString();
                    for(const simulation_project::RobotMountDesc& candidate : service.document().robotMounts) {
                        if(candidate.robotId == robotId && candidate.linkName == linkName) {
                            mountId = candidate.id;
                            break;
                        }
                    }
                    if(mountId.empty()) {
                        simulation_project::RobotMountDesc mount;
                        mount.id = service.makeUniqueId(robotId + "_" + linkName + "_mount");
                        mount.name = mount.id;
                        mount.robotId = robotId;
                        mount.linkName = linkName;
                        if(!service.addRobotMount(mount, &error)) {
                            return false;
                        }
                        mountId = mount.id;
                    }
                }

                if(!mountId.empty()) {
                    const simulation_project::RobotMountDesc* mount =
                        findRobotMountDesc(service.document(), mountId);
                    const std::string mountName = mount != nullptr && !mount->name.empty()
                        ? mount->name
                        : mountId;
                    const std::string attachmentName =
                        makeAttachmentDisplayName(asset.name, mountName);
                    attachmentId = service.makeUniqueId(makeAsciiSlug(attachmentName, "tool_attachment", 48));

                    simulation_project::MountedAttachmentDesc attachment;
                    attachment.id = attachmentId;
                    attachment.name = attachmentName;
                    attachment.mountFrameId = mountId;
                    attachment.assetId = asset.id;
                    if(!service.addMountedAttachment(attachment, &error) ||
                        !service.setActiveMountedAttachment(mountId, attachmentId, &error)) {
                        return false;
                    }
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            emit statusMessageRequested(QString("Import tool failed: %1").arg(mutationResult.message), 5000);
            return;
        }

        QApplication::setOverrideCursor(Qt::WaitCursor);
        emit statusMessageRequested(QString("Importing tool %1...").arg(fileName), 0);
        QApplication::processEvents();

        const ToolSetupViewportReloadResult reloadResult =
            m_appServices.reloadViewport(QStringLiteral("importToolAsset"));
        QApplication::restoreOverrideCursor();

        if(!reloadResult.success) {
            m_context.documentController().restoreProjectSnapshot(
                QStringLiteral("importToolAssetRollback"),
                previousDocument,
                previousDirty,
                false);
            m_appServices.reloadViewport(QStringLiteral("importToolAssetRollback"));
            const QString message = reloadResult.errorMessage.isEmpty()
                ? QString("Failed to rebuild viewport scene.")
                : reloadResult.errorMessage;
            LOG_ERROR("rs2022") << "Import tool failed: " << message.toStdString();
            emit statusMessageRequested(QString("Import tool failed: %1").arg(message), 8000);
            return;
        }

        if(!attachmentId.empty()) {
            RobotQtViewerViewportPreviewPayload preview;
            preview.setActiveMountedAttachment = true;
            preview.activeMountedAttachmentId = QString::fromStdString(attachmentId);
            mutateViewportPreview(preview, QStringLiteral("importToolAsset"));
        }
        RobotQtViewerAttachmentPayload payload;
        payload.mountId = QString::fromStdString(mountId);
        payload.attachmentId = QString::fromStdString(attachmentId);
        payload.assetId = QString::fromStdString(asset.id);
        m_context.documentController().publishAttachmentChanged(payload, QStringLiteral("importToolAsset"));
        m_context.documentController().publishDocumentChanged(QStringLiteral("importToolAsset"), false);
        if(!attachmentId.empty()) {
            selectToolAttachmentById(attachmentId);
            m_widget.focusAssetEditor();
        }
        emit statusMessageRequested(QString(!attachmentId.empty()
            ? "Imported and attached tool %1"
            : "Imported tool asset %1; select a robot link or mount frame to attach it.")
            .arg(QString::fromUtf8(asset.id.c_str())), 5000);
    }

    void ToolSetupModuleController::attachExistingToolAsset()
    {
        const QString mountId = m_widget.currentMountId();
        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(m_context.document(), mountId.toStdString());
        if(mount == nullptr) {
            emit statusMessageRequested("Select a mount frame before attaching an asset.", 4000);
            return;
        }
        const std::vector<simulation_project::AttachmentAssetDesc> genericAssets =
            genericAttachmentAssetMirrors(m_context.document());
        if(m_context.document().attachmentAssets.empty() && genericAssets.empty()) {
            emit statusMessageRequested("No attachment asset is available to attach.", 4000);
            return;
        }

        QStringList labels;
        std::vector<std::string> assetIds;
        std::vector<std::string> assetNames;
        std::vector<bool> genericAssetFlags;
        labels.reserve(static_cast<int>(genericAssets.size() + m_context.document().attachmentAssets.size()));
        for(const simulation_project::AttachmentAssetDesc& asset : genericAssets) {
            const QString assetId = QString::fromStdString(asset.id);
            const QString assetName = QString::fromStdString(asset.name.empty() ? asset.id : asset.name);
            labels.push_back(assetName == assetId
                ? assetId
                : QString("%1 [%2]").arg(assetName, conciseUiText(asset.id, 32)));
            assetIds.push_back(asset.id);
            assetNames.push_back(asset.name.empty() ? asset.id : asset.name);
            genericAssetFlags.push_back(true);
        }
        for(const simulation_project::AttachmentAssetDesc& asset : m_context.document().attachmentAssets) {
            const QString assetId = QString::fromStdString(asset.id);
            const QString assetName = QString::fromStdString(asset.name.empty() ? asset.id : asset.name);
            labels.push_back(assetName == assetId
                ? assetId
                : QString("%1 [%2]").arg(assetName, conciseUiText(asset.id, 32)));
            assetIds.push_back(asset.id);
            assetNames.push_back(asset.name.empty() ? asset.id : asset.name);
            genericAssetFlags.push_back(false);
        }

        bool ok = false;
        const QString label = QInputDialog::getItem(
            &m_widget,
            "Attach Existing Asset",
            "Attachment asset:",
            labels,
            0,
            false,
            &ok);
        if(!ok || label.isEmpty()) {
            return;
        }

        const int assetIndex = labels.indexOf(label);
        if(assetIndex < 0 || assetIndex >= static_cast<int>(assetIds.size())) {
            return;
        }

        const std::string assetId = assetIds[static_cast<std::size_t>(assetIndex)];
        const std::string assetName = assetNames[static_cast<std::size_t>(assetIndex)];
        const bool genericAsset = genericAssetFlags[static_cast<std::size_t>(assetIndex)];
        const simulation_project::ProjectDocument previousDocument = m_context.document();
        const bool previousDirty = m_context.projectSession().isDirty();

        const std::string mountName = mount->name.empty() ? mount->id : mount->name;
        const std::string attachmentName = makeAttachmentDisplayName(assetName, mountName);
        std::string attachmentId;
        std::string mountedAssetId = assetId;

        const std::string mountFrameId = mount->id;
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("attachExistingToolAsset"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(!genericAsset) {
                    const simulation_project::AttachmentAssetDesc* sourceAsset =
                        findAttachmentAssetDesc(service.document(), assetId);
                    if(sourceAsset == nullptr) {
                        error = "asset not found: " + assetId;
                        return false;
                    }

                    mountedAssetId = service.makeUniqueId(makeAsciiSlug(assetName, "tool_asset", 32));
                    const simulation_project::AttachmentAssetDesc copiedAsset =
                        makeAttachmentAssetMirror(*sourceAsset, mountedAssetId);
                    if(!service.addAttachmentAsset(copiedAsset, &error)) {
                        return false;
                    }
                }

                attachmentId = service.makeUniqueId(makeAsciiSlug(attachmentName, "tool_attachment", 48));
                simulation_project::MountedAttachmentDesc attachment;
                attachment.id = attachmentId;
                attachment.name = attachmentName;
                attachment.mountFrameId = mountFrameId;
                attachment.assetId = mountedAssetId;
                if(!service.addMountedAttachment(attachment, &error) ||
                    !service.setActiveMountedAttachment(mountFrameId, attachmentId, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            emit statusMessageRequested(QString("Attach asset failed: %1").arg(mutationResult.message), 5000);
            return;
        }

        const ToolSetupViewportReloadResult reloadResult =
            m_appServices.reloadViewport(QStringLiteral("attachExistingToolAsset"));
        if(!reloadResult.success) {
            m_context.documentController().restoreProjectSnapshot(
                QStringLiteral("attachExistingToolAssetRollback"),
                previousDocument,
                previousDirty,
                false);
            m_appServices.reloadViewport(QStringLiteral("attachExistingToolAssetRollback"));
            emit statusMessageRequested("Attach asset failed: viewport reload failed.", 8000);
            return;
        }

        selectToolAttachmentById(attachmentId);
        RobotQtViewerAttachmentPayload payload;
        payload.mountId = QString::fromStdString(mount->id);
        payload.attachmentId = QString::fromStdString(attachmentId);
        payload.assetId = QString::fromStdString(mountedAssetId);
        m_context.documentController().publishAttachmentChanged(payload, QStringLiteral("attachExistingToolAsset"));
        m_context.documentController().publishDocumentChanged(QStringLiteral("attachExistingToolAsset"), false);
        emit statusMessageRequested(QString("Attached asset %1 to %2")
            .arg(QString::fromStdString(mountedAssetId), mountId), 5000);
    }

    void ToolSetupModuleController::createRobotMountForSelectedLink()
    {
        if(m_taskDirty) {
            emit statusMessageRequested("Apply or cancel mount setup edits before adding a mount frame.", 4000);
            return;
        }

        const QString selectedRobotId = m_appServices.selectedRobotId();
        const QString selectedLinkName = m_appServices.selectedLinkName();
        if(selectedRobotId.isEmpty() || selectedLinkName.isEmpty()) {
            emit statusMessageRequested("Select a robot link before adding a mount frame.", 4000);
            return;
        }

        const std::string robotId = selectedRobotId.toStdString();
        const std::string linkName = selectedLinkName.toStdString();
        if(findRobotDesc(m_context.document(), robotId) == nullptr) {
            emit statusMessageRequested(QString("Robot not found: %1").arg(selectedRobotId), 4000);
            return;
        }

        m_mountDraftSourceRobotId = selectedRobotId;
        m_mountDraftSourceLinkName = selectedLinkName;
        m_mountFrameMode = ToolSetupMountFrameMode::Create;
        m_taskRollbackDocument = m_context.document();
        m_taskRollbackDirty = m_context.projectSession().isDirty();
        m_hasTaskRollbackDocument = true;

        simulation_project::RobotMountDesc mount;
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("createRobotMountForSelectedLink"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                mount.id = service.makeUniqueId(robotId + "_" + linkName + "_mount");
                mount.name = mount.id;
                mount.robotId = robotId;
                mount.linkName = linkName;
                if(!service.addRobotMount(mount, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            emit statusMessageRequested(
                QString("Create mount frame failed: %1").arg(mutationResult.message),
                5000);
            return;
        }

        const QString mountId = QString::fromStdString(mount.id);
        m_appServices.setRobotMountContext(selectedRobotId, selectedLinkName, mountId);
        m_context.selectionModel().selectRobotMount(
            selectedRobotId,
            selectedLinkName,
            mountId,
            QStringLiteral("createRobotMountForSelectedLink"));
        RobotQtViewerViewportPreviewPayload preview;
        preview.upsertPreviewRobotMount = true;
        preview.robotMount = mount;
        preview.setActivePreviewRobotMount = true;
        preview.activePreviewRobotMountId = mountId;
        preview.setRobotMountFrameVisibility = true;
        preview.selectedLinkFrameVisible = true;
        preview.mountFrameVisible = true;
        mutateViewportPreview(preview, QStringLiteral("createRobotMountForSelectedLink"));
        enterMountFrameViewportFocus(selectedRobotId, selectedLinkName);
        refresh(selectedRobotId, selectedLinkName, mountId);
        captureMountEditSnapshot(mountId, true);
        RobotQtViewerAttachmentPayload payload;
        payload.mountId = mountId;
        m_context.documentController().publishAttachmentChanged(
            payload,
            QStringLiteral("createRobotMountForSelectedLink"));
        m_context.documentController().publishDocumentChanged(
            QStringLiteral("createRobotMountForSelectedLink"),
            false);
        emit robotContextSelected(selectedRobotId);
        emit mountFrameFocusRequested(selectedRobotId, selectedLinkName, mountId);
        emit selectionDependentViewsRefreshRequested();
        setTaskDirty(true, QString("New mount frame is pending: %1").arg(mountId));
        RobotQtViewerViewportPreviewPayload editorPreview;
        editorPreview.setActivePreviewRobotMount = true;
        editorPreview.activePreviewRobotMountId = mountId;
        editorPreview.setRobotMountFrameVisibility = true;
        editorPreview.selectedLinkFrameVisible = true;
        editorPreview.mountFrameVisible = true;
        mutateViewportPreview(editorPreview, QStringLiteral("createRobotMountForSelectedLinkEditor"));
        enterMountFrameViewportFocus(selectedRobotId, selectedLinkName);
        emit statusMessageRequested(QString("Created mount frame draft: %1").arg(mountId), 4000);
    }

    void ToolSetupModuleController::deleteCurrentRobotMount()
    {
        if(m_taskDirty) {
            emit statusMessageRequested("Apply or cancel mount setup edits before deleting a mount frame.", 4000);
            return;
        }

        QString selectedMountId = m_widget.currentMountId();
        if(selectedMountId.isEmpty()) {
            emit statusMessageRequested("Select a mount frame before deleting.", 3000);
            return;
        }

        const simulation_project::RobotMountDesc* selectedMount =
            findRobotMountDesc(m_context.document(), selectedMountId.toStdString());
        if(selectedMount == nullptr) {
            emit statusMessageRequested(QString("Mount frame not found: %1").arg(selectedMountId), 4000);
            return;
        }

        const simulation_project::MountedAttachmentDesc* attached =
            findMountedAttachmentByMountId(m_context.document(), selectedMount->id);
        if(attached != nullptr) {
            emit statusMessageRequested(
                QString("Detach or delete attachment %1 before deleting this mount frame.")
                    .arg(QString::fromStdString(attached->id)),
                5000);
            return;
        }

        const std::string robotId = selectedMount->robotId;
        const std::string linkName = selectedMount->linkName;
        std::vector<simulation_project::RobotMountDesc> remainingMounts;
        remainingMounts.reserve(m_context.document().robotMounts.size());
        for(const simulation_project::RobotMountDesc& mount : m_context.document().robotMounts) {
            if(mount.robotId == robotId && mount.id != selectedMount->id) {
                remainingMounts.push_back(mount);
            }
        }

        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("deleteCurrentRobotMount"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(!service.replaceRobotMountsForRobot(robotId, remainingMounts, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            emit statusMessageRequested(
                QString("Delete mount frame failed: %1").arg(mutationResult.message),
                5000);
            return;
        }
        m_pinnedRobotMountFrameIds.remove(selectedMountId);

        const QString robotIdText = QString::fromStdString(robotId);
        const QString linkNameText = QString::fromStdString(linkName);
        m_appServices.setRobotMountContext(robotIdText, linkNameText, QString());
        m_context.selectionModel().selectRobotLink(
            robotIdText,
            linkNameText,
            QStringLiteral("deleteCurrentRobotMount"));
        m_appServices.reloadViewport(QStringLiteral("deleteCurrentRobotMount"));
        syncPinnedRobotMountFrames();
        refresh(robotIdText, linkNameText);
        RobotQtViewerAttachmentPayload payload;
        payload.mountId = selectedMountId;
        m_context.documentController().publishAttachmentChanged(
            payload,
            QStringLiteral("deleteCurrentRobotMount"));
        m_context.documentController().publishDocumentChanged(
            QStringLiteral("deleteCurrentRobotMount"),
            false);
        emit robotContextSelected(robotIdText);
        emit selectionDependentViewsRefreshRequested();
        emit statusMessageRequested(QString("Deleted mount frame: %1").arg(selectedMountId), 4000);
    }

    bool ToolSetupModuleController::hasPendingTaskChanges() const
    {
        return m_taskDirty;
    }

    bool ToolSetupModuleController::resolvePendingTaskChanges(QWidget* parentWidget, bool restoreEditorTarget)
    {
        if(!m_taskDirty) {
            return true;
        }

        const QMessageBox::StandardButton result = QMessageBox::question(
            parentWidget != nullptr ? parentWidget : &m_widget,
            m_objectBindingTaskActive
                ? QStringLiteral("Unsaved Object Binding")
                : QStringLiteral("Unsaved Frame Editor Changes"),
            m_objectBindingTaskActive
                ? QStringLiteral("Apply the current object binding before leaving Frame Editor?")
                : QStringLiteral("Save changes to the current mount frame before leaving Frame Editor?"),
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
            QMessageBox::Save);
        if(result == QMessageBox::Cancel) {
            return false;
        }
        if(result == QMessageBox::Discard) {
            discardPendingTaskChanges(QStringLiteral("Discarded unsaved frame edits."), restoreEditorTarget);
            return true;
        }

        return applyPendingTaskChanges();
    }

    bool ToolSetupModuleController::applyPendingTaskChanges()
    {
        if(!m_taskDirty) {
            return true;
        }

        if(m_objectBindingTaskActive) {
            applyObjectBinding(m_bindingMountId, m_bindingObjectId, m_bindingFrameId);
            return !m_taskDirty;
        }

        applyTaskChanges(
            m_widget.hasMountTransformEditor(),
            m_widget.currentMountTransform(),
            m_widget.hasAttachmentInstanceEditor(),
            m_widget.attachmentInstance(),
            m_widget.hasAttachmentOffsetEditor(),
            m_widget.currentAttachmentOffset(),
            m_widget.hasAssetEditor(),
            m_widget.currentAssetEditorAsset());
        return !m_taskDirty;
    }

    void ToolSetupModuleController::discardPendingTaskChanges(const QString& message, bool restoreEditorTarget)
    {
        const bool discardingNewMount = m_mountEditSnapshotIsNew;
        const QString draftSourceRobotId = m_mountDraftSourceRobotId;
        const QString draftSourceLinkName = m_mountDraftSourceLinkName;
        RobotQtViewerViewportPreviewPayload preview;
        if(m_mountEditSnapshotIsNew) {
            m_pinnedRobotMountFrameIds.remove(m_activeMountEditId);
            preview.removePreviewRobotMount = true;
            preview.removePreviewRobotMountId = m_activeMountEditId;
            mutateViewportPreview(preview, QStringLiteral("discardMountFrameDraft"));
        } else if(m_hasMountEditSnapshot) {
            preview.upsertPreviewRobotMount = true;
            preview.robotMount = m_mountEditSnapshot;
            preview.previewRobotMountTransform = true;
            preview.previewRobotMountId = m_activeMountEditId;
            preview.robotMountTransform = m_mountEditSnapshot.linkToMount;
            preview.setActivePreviewRobotMount = true;
            preview.activePreviewRobotMountId = m_activeMountEditId;
            mutateViewportPreview(preview, QStringLiteral("discardMountFrameDraft"));
        }

        if(m_hasTaskRollbackDocument) {
            m_context.documentController().restoreProjectSnapshot(
                QStringLiteral("discardMountFrameDraft"),
                m_taskRollbackDocument,
                m_taskRollbackDirty);
            m_hasTaskRollbackDocument = false;
            m_taskRollbackDocument = simulation_project::ProjectDocument();
            m_appServices.reloadViewport(QStringLiteral("discardTaskRollbackDocument"));
        }

        const QString robotId = m_hasMountEditSnapshot
            ? QString::fromStdString(m_mountEditSnapshot.robotId)
            : m_appServices.selectedRobotId();
        const QString linkName = m_hasMountEditSnapshot
            ? QString::fromStdString(m_mountEditSnapshot.linkName)
            : m_appServices.selectedLinkName();
        const QString mountId = m_mountEditSnapshotIsNew
            ? QString()
            : m_activeMountEditId;

        setTaskDirty(false, message);
        m_mountFrameMode = ToolSetupMountFrameMode::Selection;
        m_objectBindingTaskActive = false;
        m_bindingMountId.clear();
        m_bindingObjectId.clear();
        m_bindingFrameId.clear();
        m_widget.setObjectBindingEditor(
            false,
            ToolSetupBindingView(),
            QVector<ToolSetupComboItem>(),
            QString(),
            QVector<ToolSetupComboItem>(),
            QString(),
            QVector<ToolSetupComboItem>(),
            QString(),
            QString(),
            false);
        m_widget.setObjectBindingMode(false);
        clearMountEditSnapshot();
        if(!restoreEditorTarget) {
            clearMountFrameViewportFocus();
            if(discardingNewMount && !draftSourceRobotId.isEmpty() && !draftSourceLinkName.isEmpty()) {
                m_appServices.setRobotMountContext(draftSourceRobotId, draftSourceLinkName, QString());
                m_context.selectionModel().selectRobotLink(
                    draftSourceRobotId,
                    draftSourceLinkName,
                    QStringLiteral("discardMountFrameDraft"));
                emit robotContextSelected(draftSourceRobotId);
                emit linkFocusRequested(draftSourceRobotId, draftSourceLinkName);
            }
            emit selectionDependentViewsRefreshRequested();
            return;
        }
        if(!robotId.isEmpty()) {
            m_appServices.setRobotMountContext(robotId, linkName, mountId);
        }
        refresh(robotId, linkName, mountId);
        if(!mountId.isEmpty()) {
            enterMountFrameViewportFocus(robotId, linkName);
            emit mountFrameFocusRequested(robotId, linkName, mountId);
        } else {
            clearMountFrameViewportFocus();
        }
        emit selectionDependentViewsRefreshRequested();
    }

    void ToolSetupModuleController::configureCurrentToolAttachment()
    {
        const QString attachmentId = m_widget.currentAttachmentId();
        const simulation_project::MountedAttachmentDesc* attachment =
            findMountedAttachmentDesc(m_context.document(), attachmentId.toStdString());
        if(attachment == nullptr) {
            emit statusMessageRequested("Select an attachment first.", 3000);
            return;
        }

        m_widget.focusAttachmentInstanceEditor();
        emit statusMessageRequested(
            QString("Editing attachment in task panel: %1").arg(conciseUiText(attachment->id, 48)),
            3000);
    }

    void ToolSetupModuleController::applyCurrentAttachmentOffset(
        const simulation_project::TransformDesc& transform)
    {
        const QString attachmentId = m_widget.currentAttachmentId();
        const simulation_project::MountedAttachmentDesc* attachment =
            findMountedAttachmentDesc(m_context.document(), attachmentId.toStdString());
        if(attachment == nullptr) {
            emit statusMessageRequested("Select an attachment first.", 3000);
            return;
        }

        simulation_project::MountedAttachmentDesc updatedAttachment =
            makeMountedAttachmentMirror(*attachment);
        updatedAttachment.mountToAssetMount = transform;

        ToolAttachmentCommandResult commandResult;
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("applyToolAttachmentOffset"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                commandResult =
                    ToolAttachmentCommandController::applyMountedAttachmentUpdate(service.document(), updatedAttachment);
                if(!commandResult.success) {
                    error = commandResult.message.toStdString();
                    return false;
                }
                changed = commandResult.projectChanged;
                return true;
            });
        if(!mutationResult.success) {
            emit statusMessageRequested(mutationResult.message, 5000);
            return;
        }

        m_appServices.reloadViewport(QStringLiteral("applyToolAttachmentOffset"));
        selectToolAttachmentById(updatedAttachment.id);
        RobotQtViewerAttachmentPayload payload;
        payload.mountId = QString::fromStdString(updatedAttachment.mountFrameId);
        payload.attachmentId = QString::fromStdString(updatedAttachment.id);
        payload.assetId = QString::fromStdString(updatedAttachment.assetId);
        m_context.documentController().publishAttachmentChanged(payload, QStringLiteral("applyToolAttachmentOffset"));
        m_context.documentController().publishDocumentChanged(QStringLiteral("applyToolAttachmentOffset"), false);
        setTaskDirty(false);
        emit statusMessageRequested(
            QString("Attachment offset updated: %1").arg(conciseUiText(updatedAttachment.id, 48)),
            3000);
    }

    void ToolSetupModuleController::applyCurrentToolAsset(
        const simulation_project::AttachmentAssetDesc& asset)
    {
        const QString attachmentId = m_widget.currentAttachmentId();
        const simulation_project::MountedAttachmentDesc* attachment =
            findMountedAttachmentDesc(m_context.document(), attachmentId.toStdString());
        const QString selectedAssetId = m_widget.currentAssetId();
        const std::string assetId = attachment != nullptr
            ? attachment->assetId
            : selectedAssetId.toStdString();
        if(assetId.empty()) {
            emit statusMessageRequested("Select an attachment asset first.", 3000);
            return;
        }

        const simulation_project::AttachmentAssetDesc* existingAsset =
            findAttachmentAssetDesc(m_context.document(), assetId);
        if(existingAsset == nullptr) {
            emit statusMessageRequested(
                QString("Attachment asset not found: %1").arg(QString::fromStdString(assetId)),
                5000);
            return;
        }

        const std::string attachmentIdValue = attachment != nullptr ? attachment->id : std::string();
        const std::string mountId = attachment != nullptr ? attachment->mountFrameId : std::string();
        ToolAttachmentCommandResult commandResult;
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("applyToolAssetEditor"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                commandResult =
                    ToolAttachmentCommandController::applyAttachmentAssetUpdate(service.document(), assetId, asset);
                if(!commandResult.success) {
                    error = commandResult.message.toStdString();
                    return false;
                }
                changed = commandResult.projectChanged;
                return true;
            });
        if(!mutationResult.success) {
            emit statusMessageRequested(mutationResult.message, 5000);
            return;
        }

        m_appServices.reloadViewport(QStringLiteral("applyToolAssetEditor"));
        if(!attachmentIdValue.empty()) {
            selectToolAttachmentById(attachmentIdValue);
        } else {
            const QString appliedAssetId = QString::fromStdString(assetId);
            m_context.selectionModel().selectToolAsset(appliedAssetId, QStringLiteral("applyToolAssetEditor"));
            refresh(m_appServices.selectedRobotId(), m_appServices.selectedLinkName(), QString(), QString(), appliedAssetId);
        }
        RobotQtViewerAttachmentPayload payload;
        payload.mountId = QString::fromStdString(mountId);
        payload.attachmentId = QString::fromStdString(attachmentIdValue);
        payload.assetId = QString::fromStdString(assetId);
        m_context.documentController().publishAttachmentChanged(payload, QStringLiteral("applyToolAssetEditor"));
        m_context.documentController().publishDocumentChanged(QStringLiteral("applyToolAssetEditor"), false);
        setTaskDirty(false);
        emit statusMessageRequested(
            QString("Attachment asset updated: %1").arg(conciseUiText(assetId, 48)),
            3000);
    }

    void ToolSetupModuleController::applyTaskChanges(
        bool hasMountTransform,
        const simulation_project::TransformDesc& mountTransform,
        bool hasAttachmentInstance,
        const simulation_project::MountedAttachmentDesc& attachmentInstance,
        bool hasAttachmentOffset,
        const simulation_project::TransformDesc& attachmentOffset,
        bool hasToolAsset,
        const simulation_project::AttachmentAssetDesc& toolAsset)
    {
        const QString attachmentId = m_widget.currentAttachmentId();
        QString selectedMountId = m_widget.currentMountId();
        const QString selectedMountLinkName = m_widget.currentMountLinkName();
        const simulation_project::RobotMountDesc* selectedMount =
            findRobotMountDesc(m_context.document(), selectedMountId.toStdString());
        const simulation_project::MountedAttachmentDesc* attachment =
            findMountedAttachmentDesc(m_context.document(), attachmentId.toStdString());
        if(attachment == nullptr) {
            if(hasAttachmentInstance || hasAttachmentOffset) {
                emit statusMessageRequested("Select an attachment first.", 3000);
                return;
            }
            if(!hasMountTransform && !hasToolAsset) {
                emit statusMessageRequested("Select an attachment or attachment asset first.", 3000);
                return;
            }

            bool projectChanged = false;
            bool mountFrameChanged = false;
            QString appliedMountRobotId;
            QString appliedMountLinkName;
            if(hasMountTransform) {
                if(selectedMount == nullptr) {
                    emit statusMessageRequested("Select a mount frame first.", 3000);
                    return;
                }
                simulation_project::RobotMountDesc updatedMount = *selectedMount;
                const QString oldMountId = QString::fromStdString(selectedMount->id);
                const QString mountName = m_widget.currentMountFrameName();
                if(mountName.isEmpty()) {
                    emit statusMessageRequested("Frame key/name cannot be empty.", 4000);
                    return;
                }
                updatedMount.id = qStringToUtf8(mountName);
                updatedMount.name = updatedMount.id;
                if(!selectedMountLinkName.isEmpty()) {
                    updatedMount.linkName = selectedMountLinkName.toStdString();
                }
                updatedMount.linkToMount = mountTransform;
                selectedMountId = QString::fromStdString(updatedMount.id);
                if(!sameRobotMount(*selectedMount, updatedMount)) {
                    const std::string previousMountId = selectedMount->id;
                    const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
                        QStringLiteral("applyToolSetupMountTask"),
                        ProjectDirtyPolicy::UserEdit,
                        [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                            if(!service.updateRobotMountFrame(previousMountId, updatedMount, &error)) {
                                return false;
                            }
                            changed = true;
                            return true;
                        });
                    if(!mutationResult.success) {
                        emit statusMessageRequested(
                            QString("Mount frame update failed: %1").arg(mutationResult.message),
                            5000);
                        return;
                    }
                    const bool mountFrameWasPinned = m_pinnedRobotMountFrameIds.contains(oldMountId);
                    if(oldMountId != selectedMountId) {
                        m_pinnedRobotMountFrameIds.remove(oldMountId);
                        if(mountFrameWasPinned) {
                            m_pinnedRobotMountFrameIds.insert(selectedMountId);
                        }
                        syncPinnedRobotMountFrames();
                    }
                    RobotQtViewerViewportPreviewPayload preview;
                    if(oldMountId != selectedMountId) {
                        preview.removePreviewRobotMount = true;
                        preview.removePreviewRobotMountId = oldMountId;
                    }
                    preview.upsertPreviewRobotMount = true;
                    preview.robotMount = updatedMount;
                    preview.previewRobotMountTransform = true;
                    preview.previewRobotMountId = selectedMountId;
                    preview.robotMountTransform = updatedMount.linkToMount;
                    preview.setActivePreviewRobotMount = true;
                    preview.activePreviewRobotMountId = selectedMountId;
                    mutateViewportPreview(preview, QStringLiteral("applyTaskChanges"));
                    projectChanged = true;
                    mountFrameChanged = true;
                }
                appliedMountRobotId = QString::fromStdString(updatedMount.robotId);
                appliedMountLinkName = QString::fromStdString(updatedMount.linkName);
            }

            const QString selectedAssetId = m_widget.currentAssetId();
            const std::string assetId = selectedAssetId.toStdString();
            if(hasToolAsset) {
                if(assetId.empty()) {
                    emit statusMessageRequested("Select an attachment asset first.", 3000);
                    return;
                }

                ToolAttachmentCommandResult commandResult;
                const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
                    QStringLiteral("applyToolSetupAssetTask"),
                    ProjectDirtyPolicy::UserEdit,
                    [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                        commandResult =
                            ToolAttachmentCommandController::applyAttachmentAssetUpdate(service.document(), assetId, toolAsset);
                        if(!commandResult.success) {
                            error = commandResult.message.toStdString();
                            return false;
                        }
                        changed = commandResult.projectChanged;
                        return true;
                    });
                if(!mutationResult.success) {
                    emit statusMessageRequested(mutationResult.message, 5000);
                    return;
                }
                projectChanged = projectChanged || mutationResult.changed;
            }
            const QString appliedAssetId = QString::fromStdString(assetId);
            if(hasMountTransform && !appliedMountRobotId.isEmpty()) {
                m_appServices.setRobotMountContext(appliedMountRobotId, appliedMountLinkName, selectedMountId);
                m_context.selectionModel().selectRobotMount(
                    appliedMountRobotId,
                    appliedMountLinkName,
                    selectedMountId,
                    QStringLiteral("applyToolSetupMountTask"));
            } else if(!appliedAssetId.isEmpty()) {
                m_context.selectionModel().selectToolAsset(appliedAssetId, QStringLiteral("applyToolSetupAssetTask"));
            }
            m_mountFrameMode = ToolSetupMountFrameMode::Selection;
            refresh(
                m_appServices.selectedRobotId(),
                m_appServices.selectedLinkName(),
                selectedMountId,
                QString(),
                appliedAssetId);
            RobotQtViewerAttachmentPayload payload;
            payload.mountId = selectedMountId;
            payload.assetId = appliedAssetId;
            m_context.documentController().publishAttachmentChanged(payload, QStringLiteral("applyToolSetupAssetTask"));
            m_context.documentController().publishDocumentChanged(QStringLiteral("applyToolSetupAssetTask"), false);
            clearMountEditSnapshot();
            captureMountEditSnapshot(selectedMountId);
            setTaskDirty(false);
            if(hasMountTransform && !appliedMountRobotId.isEmpty()) {
                clearMountFrameViewportFocus();
                emit robotContextSelected(appliedMountRobotId);
                emit mountFrameFocusRequested(appliedMountRobotId, appliedMountLinkName, selectedMountId);
                emit selectionDependentViewsRefreshRequested();
            }
            if(hasMountTransform) {
                clearMountFrameViewportFocus();
                emit taskExitRequested();
            }
            emit statusMessageRequested("Mount setup changes applied.", 3000);
            return;
        }

        const std::string attachmentIdValue = attachment->id;
        const std::string mountId = attachment->mountFrameId;
        const std::string assetId = attachment->assetId;
        std::string appliedMountId = mountId;
        std::string appliedAssetId = assetId;
        bool projectChanged = false;

        if(hasMountTransform) {
            if(selectedMount == nullptr) {
                emit statusMessageRequested("Select a mount frame first.", 3000);
                return;
            }
            simulation_project::RobotMountDesc updatedMount = *selectedMount;
            const QString oldMountId = QString::fromStdString(selectedMount->id);
            const QString mountName = m_widget.currentMountFrameName();
            if(mountName.isEmpty()) {
                emit statusMessageRequested("Frame key/name cannot be empty.", 4000);
                return;
            }
            updatedMount.id = qStringToUtf8(mountName);
            updatedMount.name = updatedMount.id;
            if(!selectedMountLinkName.isEmpty()) {
                updatedMount.linkName = selectedMountLinkName.toStdString();
            }
            updatedMount.linkToMount = mountTransform;
            appliedMountId = updatedMount.id;
            selectedMountId = QString::fromStdString(updatedMount.id);
            if(!sameRobotMount(*selectedMount, updatedMount)) {
                const std::string previousMountId = selectedMount->id;
                const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
                    QStringLiteral("applyToolSetupTask"),
                    ProjectDirtyPolicy::UserEdit,
                    [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                        if(!service.updateRobotMountFrame(previousMountId, updatedMount, &error)) {
                            return false;
                        }
                        changed = true;
                        return true;
                    });
                if(!mutationResult.success) {
                    emit statusMessageRequested(
                        QString("Mount frame update failed: %1").arg(mutationResult.message),
                        5000);
                    return;
                }
                RobotQtViewerViewportPreviewPayload preview;
                if(oldMountId != selectedMountId) {
                    preview.removePreviewRobotMount = true;
                    preview.removePreviewRobotMountId = oldMountId;
                }
                preview.upsertPreviewRobotMount = true;
                preview.robotMount = updatedMount;
                preview.previewRobotMountTransform = true;
                preview.previewRobotMountId = selectedMountId;
                preview.robotMountTransform = updatedMount.linkToMount;
                preview.setActivePreviewRobotMount = true;
                preview.activePreviewRobotMountId = selectedMountId;
                mutateViewportPreview(preview, QStringLiteral("applyTaskChanges"));
                projectChanged = true;
            }
        }

        if(hasAttachmentInstance || hasAttachmentOffset) {
            simulation_project::MountedAttachmentDesc updatedAttachment =
                hasAttachmentInstance
                    ? makeMountedAttachmentMirror(attachmentInstance)
                    : makeMountedAttachmentMirror(*attachment);
            updatedAttachment.id = attachment->id;
            if(updatedAttachment.mountFrameId.empty()) {
                updatedAttachment.mountFrameId = attachment->mountFrameId;
            }
            if(updatedAttachment.assetId.empty()) {
                updatedAttachment.assetId = attachment->assetId;
            }
            if(hasMountTransform) {
                updatedAttachment.mountFrameId = appliedMountId;
            }
            if(hasAttachmentOffset) {
                updatedAttachment.mountToAssetMount = attachmentOffset;
            }
            appliedMountId = updatedAttachment.mountFrameId;
            appliedAssetId = updatedAttachment.assetId;
            ToolAttachmentCommandResult commandResult;
            const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
                QStringLiteral("applyToolSetupTask"),
                ProjectDirtyPolicy::UserEdit,
                [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                    commandResult =
                        ToolAttachmentCommandController::applyMountedAttachmentUpdate(service.document(), updatedAttachment);
                    if(!commandResult.success) {
                        error = commandResult.message.toStdString();
                        return false;
                    }
                    changed = commandResult.projectChanged;
                    return true;
                });
            if(!mutationResult.success) {
                emit statusMessageRequested(mutationResult.message, 5000);
                return;
            }
            projectChanged = projectChanged || mutationResult.changed;
        }

        if(hasToolAsset) {
            ToolAttachmentCommandResult commandResult;
            const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
                QStringLiteral("applyToolSetupTask"),
                ProjectDirtyPolicy::UserEdit,
                [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                    commandResult =
                        ToolAttachmentCommandController::applyAttachmentAssetUpdate(service.document(), appliedAssetId, toolAsset);
                    if(!commandResult.success) {
                        error = commandResult.message.toStdString();
                        return false;
                    }
                    changed = commandResult.projectChanged;
                    return true;
                });
            if(!mutationResult.success) {
                emit statusMessageRequested(mutationResult.message, 5000);
                return;
            }
            projectChanged = projectChanged || mutationResult.changed;
            if(!hasAttachmentInstance) {
                appliedAssetId = toolAsset.id;
            }
        }
        selectToolAttachmentById(attachmentIdValue);
        RobotQtViewerAttachmentPayload payload;
        payload.mountId = QString::fromStdString(appliedMountId);
        payload.attachmentId = QString::fromStdString(attachmentIdValue);
        payload.assetId = QString::fromStdString(appliedAssetId);
        m_context.documentController().publishAttachmentChanged(payload, QStringLiteral("applyToolSetupTask"));
        m_context.documentController().publishDocumentChanged(QStringLiteral("applyToolSetupTask"), false);
        m_mountFrameMode = ToolSetupMountFrameMode::Selection;
        clearMountEditSnapshot();
        captureMountEditSnapshot(QString::fromStdString(appliedMountId));
        setTaskDirty(false);
        if(const simulation_project::RobotMountDesc* appliedMount =
            findRobotMountDesc(m_context.document(), appliedMountId)) {
            if(hasMountTransform) {
                const QString appliedRobotId = QString::fromStdString(appliedMount->robotId);
                const QString appliedLinkName = QString::fromStdString(appliedMount->linkName);
                const QString appliedMountFrameId = QString::fromStdString(appliedMount->id);
                clearMountFrameViewportFocus();
                emit robotContextSelected(appliedRobotId);
                emit mountFrameFocusRequested(appliedRobotId, appliedLinkName, appliedMountFrameId);
                emit selectionDependentViewsRefreshRequested();
                emit taskExitRequested();
            } else {
                enterMountFrameViewportFocus(
                    QString::fromStdString(appliedMount->robotId),
                    QString::fromStdString(appliedMount->linkName));
            }
        }
        emit statusMessageRequested("Mount setup changes applied.", 3000);
    }

    void ToolSetupModuleController::cancelTaskChanges()
    {
        discardPendingTaskChanges(QStringLiteral("Canceled unsaved frame edits."), false);
        emit statusMessageRequested("Mount setup edits canceled.", 3000);
        emit taskExitRequested();
    }

    void ToolSetupModuleController::requestTaskExit()
    {
        if(!resolvePendingTaskChanges(&m_widget)) {
            return;
        }
        clearMountFrameViewportFocus();
        emit taskExitRequested();
    }

    void ToolSetupModuleController::setTaskDirty(bool dirty, const QString& message)
    {
        if(m_taskDirty == dirty && message.isEmpty()) {
            return;
        }
        m_taskDirty = dirty;
        m_widget.setTaskDirty(dirty, message);
        emit taskDirtyChanged(dirty);
    }

    void ToolSetupModuleController::captureMountEditSnapshot(const QString& mountId, bool newMount)
    {
        if(mountId.isEmpty()) {
            clearMountEditSnapshot();
            return;
        }

        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(m_context.document(), mountId.toStdString());
        if(mount == nullptr) {
            clearMountEditSnapshot();
            return;
        }

        m_hasMountEditSnapshot = true;
        m_mountEditSnapshotIsNew = newMount;
        m_activeMountEditId = mountId;
        m_mountEditSnapshot = *mount;
    }

    void ToolSetupModuleController::clearMountEditSnapshot()
    {
        m_hasMountEditSnapshot = false;
        m_mountEditSnapshotIsNew = false;
        m_activeMountEditId.clear();
        m_mountDraftSourceRobotId.clear();
        m_mountDraftSourceLinkName.clear();
        m_mountEditSnapshot = simulation_project::RobotMountDesc();
        m_hasTaskRollbackDocument = false;
        m_taskRollbackDirty = false;
        m_taskRollbackDocument = simulation_project::ProjectDocument();
    }

    void ToolSetupModuleController::enterMountFrameViewportFocus(
        const QString& robotId,
        const QString& linkName)
    {
        if(robotId.isEmpty() || linkName.isEmpty()) {
            return;
        }
        RobotQtViewerViewportPreviewPayload preview;
        preview.focusMountFrameLink = true;
        preview.focusMountFrameRobotId = robotId;
        preview.focusMountFrameLinkName = linkName;
        mutateViewportPreview(preview, QStringLiteral("toolSetupMountFrameFocus"));
    }

    void ToolSetupModuleController::clearMountFrameViewportFocus()
    {
        RobotQtViewerViewportPreviewPayload preview;
        preview.clearMountFrameLinkFocus = true;
        mutateViewportPreview(preview, QStringLiteral("toolSetupClearMountFrameFocus"));
    }

    void ToolSetupModuleController::clearObjectBindingPreviewViewportState()
    {
        m_context.viewportPreviewState().clearTaskPreview(
            QStringLiteral("clearObjectBindingPreviewViewportState"));
    }

    void ToolSetupModuleController::mutateViewportPreview(
        const RobotQtViewerViewportPreviewPayload& preview,
        const QString& sourceId)
    {
        RobotQtViewerViewportPreviewPayload mergedPreview = preview;
        if(!mergedPreview.setPinnedRobotMountFrames) {
            mergedPreview.setPinnedRobotMountFrames = true;
            mergedPreview.pinnedRobotMountFrameIds = pinnedRobotMountFrameIds();
        }
        m_context.viewportPreviewState().mutate(mergedPreview, sourceId);
    }

    void ToolSetupModuleController::selectToolAttachmentById(const std::string& attachmentId)
    {
        const simulation_project::MountedAttachmentDesc* attachment =
            findMountedAttachmentDesc(m_context.document(), attachmentId);

        const simulation_project::RobotMountDesc* mount =
            attachment != nullptr
                ? findRobotMountDesc(m_context.document(), attachment->mountFrameId)
                : nullptr;
        if(mount == nullptr) {
            return;
        }

        const QString robotId = QString::fromStdString(mount->robotId);
        const QString linkName = QString::fromStdString(mount->linkName);
        const QString mountId = QString::fromStdString(mount->id);
        const QString attachmentIdText = QString::fromStdString(attachmentId);
        m_appServices.setToolAttachmentContext(
            robotId,
            linkName,
            mountId,
            attachmentIdText);
        m_context.selectionModel().selectMountedAttachment(
            attachmentIdText,
            robotId,
            linkName,
            mountId,
            QStringLiteral("selectToolAttachmentById"));
        emit robotContextSelected(robotId);
        emit selectionDependentViewsRefreshRequested();
        refresh(robotId, linkName, mountId, attachmentIdText);
        RobotQtViewerViewportPreviewPayload preview;
        preview.setActiveMountedAttachment = true;
        preview.activeMountedAttachmentId = attachmentIdText;
        mutateViewportPreview(preview, QStringLiteral("selectToolAttachmentById"));
    }

    void ToolSetupModuleController::editCurrentToolAsset()
    {
        const QString attachmentId = m_widget.currentAttachmentId();
        const simulation_project::MountedAttachmentDesc* attachment =
            findMountedAttachmentDesc(m_context.document(), attachmentId.toStdString());
        const QString selectedAssetId = m_widget.currentAssetId();
        const std::string assetId = attachment != nullptr
            ? attachment->assetId
            : selectedAssetId.toStdString();
        const simulation_project::AttachmentAssetDesc* asset =
            findAttachmentAssetDesc(m_context.document(), assetId);
        if(asset == nullptr) {
            emit statusMessageRequested(
                assetId.empty()
                    ? QStringLiteral("Select an attachment asset first.")
                    : QString("Attachment asset not found: %1").arg(QString::fromStdString(assetId)),
                5000);
            return;
        }

        m_widget.focusAssetEditor();
        emit statusMessageRequested(
            QString("Editing attachment asset in task panel: %1").arg(conciseUiText(asset->id, 48)),
            3000);
    }

    bool ToolSetupModuleController::editToolAssetById(const std::string& assetId)
    {
        const simulation_project::AttachmentAssetDesc* attachmentAsset =
            findAttachmentAssetDesc(m_context.document(), assetId);
        if(attachmentAsset == nullptr) {
            emit statusMessageRequested(QString("Attachment asset not found: %1").arg(QString::fromStdString(assetId)), 5000);
            return false;
        }

        const simulation_project::MountedAttachmentDesc* attachment =
            findMountedAttachmentByAssetId(m_context.document(), assetId);
        if(attachment == nullptr) {
            const QString assetIdText = QString::fromStdString(assetId);
            m_context.selectionModel().selectToolAsset(assetIdText, QStringLiteral("editToolAssetById"));
            refresh(m_appServices.selectedRobotId(), m_appServices.selectedLinkName(), QString(), QString(), assetIdText);
            m_widget.focusAssetEditor();
            emit statusMessageRequested(
                QString("Editing unmounted attachment asset in task panel: %1")
                    .arg(conciseUiText(assetId, 48)),
                3000);
            return true;
        }

        selectToolAttachmentById(attachment->id);
        m_widget.focusAssetEditor();
        emit statusMessageRequested(
            QString("Editing attachment asset in task panel: %1").arg(conciseUiText(assetId, 48)),
            3000);
        return true;
    }

    void ToolSetupModuleController::createToolAssetFromSceneObject(const QString& objectId)
    {
        if(objectId.isEmpty()) {
            return;
        }

        const simulation_project::SceneObjectDesc* object =
            findSceneObjectDesc(m_context.document(), objectId.toStdString());
        if(object == nullptr) {
            emit statusMessageRequested(QStringLiteral("Selected object is not in project."), 3000);
            return;
        }
        if(object->sourcePath.empty()) {
            emit statusMessageRequested(QStringLiteral("Selected object has no source model path."), 4000);
            return;
        }

        const std::string objectName = object->name.empty() ? object->id : object->name;
        const std::string objectSourcePath = object->sourcePath;
        const std::string objectType = object->objectType;
        const double objectVisualScale = object->visualScale;
        const bool objectVisible = object->visible;
        simulation_project::AttachmentAssetDesc asset;
        asset.assetKind = "tool";
        asset.assetType = objectType.empty() ? std::string("tool") : objectType;
        asset.visualPath = objectSourcePath;
        asset.visualScale = objectVisualScale > 0.0 ? objectVisualScale : 1.0;
        asset.visible = objectVisible;

        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("createToolAssetFromSceneObject"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                asset.id = service.makeUniqueId(makeAsciiSlug(objectName, "tool_asset", 32));
                asset.name = makeToolAssetDisplayName(objectName, asset.id);
                if(!service.addAttachmentAsset(asset, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            emit statusMessageRequested(QString("Create tool asset failed: %1").arg(mutationResult.message), 5000);
            return;
        }

        RobotQtViewerAttachmentPayload payload;
        payload.assetId = QString::fromStdString(asset.id);
        m_context.documentController().publishAttachmentChanged(payload, QStringLiteral("createToolAssetFromSceneObject"));
        emit statusMessageRequested(QString("Created tool asset %1 from object %2; attach it before editing in mount setup")
            .arg(QString::fromStdString(asset.id), objectId), 5000);
    }

    void ToolSetupModuleController::focusObjectBindingTask(
        const QString& preferredMountId,
        const QString& preferredObjectId,
        const QString& preferredFrameId)
    {
        if(m_taskDirty && !resolvePendingTaskChanges(&m_widget)) {
            return;
        }

        m_taskRollbackDocument = m_context.document();
        m_taskRollbackDirty = m_context.projectSession().isDirty();
        m_hasTaskRollbackDocument = true;
        m_objectBindingTaskActive = true;

        QString mountId = preferredMountId;
        if(mountId.isEmpty()) {
            mountId = m_widget.currentMountId();
        }

        QString objectId = preferredObjectId;
        QString frameId = preferredFrameId;

        m_bindingMountId = mountId;
        m_bindingObjectId = objectId;
        m_bindingFrameId = frameId;
        m_widget.setObjectBindingMode(true);
        refreshObjectBindingEditor(
            mountId,
            objectId,
            frameId,
            QStringLiteral("Select a mount frame, object, and object frame, then review the preview."));
        setTaskDirty(false, QStringLiteral("Select binding targets."));
        emit statusMessageRequested(QStringLiteral("Object binding task is ready."), 3000);
    }

    void ToolSetupModuleController::handleObjectBindingSelectionChanged(
        const QString& mountId,
        const QString& objectId,
        const QString& frameId)
    {
        if(!m_objectBindingTaskActive) {
            return;
        }
        m_bindingMountId = mountId;
        m_bindingObjectId = objectId;
        m_bindingFrameId = frameId;
        refreshObjectBindingEditor(mountId, objectId, frameId);
        const bool previewReady = previewObjectBinding(mountId, objectId, frameId);
        setTaskDirty(previewReady, previewReady
            ? QStringLiteral("Object binding preview is pending.")
            : QStringLiteral("Select valid binding targets."));
    }

    bool ToolSetupModuleController::previewObjectBinding(
        const QString& mountId,
        const QString& objectId,
        const QString& frameId,
        QString* attachmentId,
        QString* assetId)
    {
        if(attachmentId != nullptr) {
            attachmentId->clear();
        }
        if(assetId != nullptr) {
            assetId->clear();
        }
        if(!m_hasTaskRollbackDocument) {
            return false;
        }

        m_context.documentController().restoreProjectSnapshot(
            QStringLiteral("previewObjectBindingReset"),
            m_taskRollbackDocument,
            m_taskRollbackDirty,
            false);

        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(m_context.document(), mountId.toStdString());
        const simulation_project::SceneObjectDesc* object =
            findSceneObjectDesc(m_context.document(), objectId.toStdString());
        if(mount == nullptr || object == nullptr || object->sourcePath.empty()) {
            m_appServices.reloadViewport(QStringLiteral("previewObjectBindingInvalid"));
            return false;
        }

        simulation_project::TransformDesc assetMountToVisual;
        QString frameLabel = QStringLiteral("object origin");
        if(!frameId.isEmpty()) {
            const simulation_project::ObjectFrameDesc* frame =
                findObjectFrameDesc(*object, frameId.toStdString());
            if(frame == nullptr) {
                m_appServices.reloadViewport(QStringLiteral("previewObjectBindingMissingFrame"));
                return false;
            }
            assetMountToVisual = inverseTransform(frame->objectToFrame);
            frameLabel = QString::fromStdString(frame->name.empty() ? frame->id : frame->name);
        }

        const std::string objectName = object->name.empty() ? object->id : object->name;
        const std::string objectSlug = makeAsciiSlug(objectName, "bound_object", 32);
        std::string previewAssetId;
        std::string previewAttachmentId;

        simulation_project::AttachmentAssetDesc asset;
        asset.id = previewAssetId;
        asset.name = objectName;
        asset.assetKind = "tool";
        asset.assetType = object->objectType.empty() ? std::string("object") : object->objectType;
        asset.visualPath = object->sourcePath;
        asset.visualScale = object->visualScale > 0.0 ? object->visualScale : 1.0;
        asset.assetMountToVisual = assetMountToVisual;
        asset.visible = true;
        if(!frameId.isEmpty()) {
            simulation_project::AttachmentFunctionalFrameDesc frame;
            frame.id = previewAssetId + ".object_frame";
            frame.name = frameLabel.toStdString();
            frame.frameType = "objectFrame";
            frame.primary = true;
            asset.functionalFrames.push_back(frame);
        }

        simulation_project::MountedAttachmentDesc attachment;
        attachment.name = makeAttachmentDisplayName(asset.name, mount->name.empty() ? mount->id : mount->name);
        attachment.mountFrameId = mount->id;
        attachment.sourceObjectId = object->id;
        attachment.sourceObjectFrameId = frameId.toStdString();
        attachment.visible = true;
        attachment.enabled = true;

        const std::string mountFrameId = mount->id;
        const std::string sourceObjectId = object->id;
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("previewObjectBinding"),
            ProjectDirtyPolicy::PreviewOnly,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                previewAssetId = service.makeUniqueId(objectSlug + "_asset");
                previewAttachmentId = service.makeUniqueId(objectSlug + "_attachment");
                asset.id = previewAssetId;
                if(!asset.functionalFrames.empty()) {
                    asset.functionalFrames.front().id = previewAssetId + ".object_frame";
                }
                attachment.id = previewAttachmentId;
                attachment.assetId = asset.id;
                if(!service.addAttachmentAsset(asset, &error) ||
                    !service.addMountedAttachment(attachment, &error) ||
                    !service.setActiveMountedAttachment(mountFrameId, attachment.id, &error) ||
                    !service.setSceneObjectVisibility(sourceObjectId, false, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            m_context.documentController().restoreProjectSnapshot(
                QStringLiteral("previewObjectBindingRollback"),
                m_taskRollbackDocument,
                m_taskRollbackDirty,
                false);
            m_appServices.reloadViewport(QStringLiteral("previewObjectBindingRollback"));
            emit statusMessageRequested(QString("Object binding preview failed: %1").arg(mutationResult.message), 5000);
            return false;
        }

        const ToolSetupViewportReloadResult reloadResult =
            m_appServices.reloadViewport(QStringLiteral("previewObjectBinding"));
        if(!reloadResult.success) {
            m_context.documentController().restoreProjectSnapshot(
                QStringLiteral("previewObjectBindingReloadRollback"),
                m_taskRollbackDocument,
                m_taskRollbackDirty,
                false);
            m_appServices.reloadViewport(QStringLiteral("previewObjectBindingReloadRollback"));
            emit statusMessageRequested(
                reloadResult.errorMessage.isEmpty()
                    ? QStringLiteral("Object binding preview failed: viewport reload failed.")
                    : reloadResult.errorMessage,
                8000);
            return false;
        }

        const QString attachmentIdText = QString::fromStdString(attachment.id);
        const QString robotId = QString::fromStdString(mount->robotId);
        const QString linkName = QString::fromStdString(mount->linkName);
        m_context.selectionModel().selectMountedAttachment(
            attachmentIdText,
            robotId,
            linkName,
            mountId,
            QStringLiteral("previewObjectBinding"));
        RobotQtViewerViewportPreviewPayload preview;
        preview.setActiveMountedAttachment = true;
        preview.activeMountedAttachmentId = attachmentIdText;
        preview.setRobotMountFrameVisibility = true;
        preview.selectedLinkFrameVisible = true;
        preview.mountFrameVisible = true;
        mutateViewportPreview(preview, QStringLiteral("previewObjectBinding"));
        if(attachmentId != nullptr) {
            *attachmentId = attachmentIdText;
        }
        if(assetId != nullptr) {
            *assetId = QString::fromStdString(asset.id);
        }
        return true;
    }

    void ToolSetupModuleController::refreshObjectBindingEditor(
        const QString& mountId,
        const QString& objectId,
        const QString& frameId,
        const QString& statusMessage)
    {
        const simulation_project::ProjectDocument& document =
            m_hasTaskRollbackDocument ? m_taskRollbackDocument : m_context.document();

        QVector<ToolSetupComboItem> mountItems;
        mountItems.reserve(static_cast<int>(document.robotMounts.size()));
        mountItems.push_back(makeComboItem(QStringLiteral("Select mount frame..."), QString()));
        for(const simulation_project::RobotMountDesc& mount : document.robotMounts) {
            const QString id = QString::fromStdString(mount.id);
            const QString name = QString::fromStdString(mount.name.empty() ? mount.id : mount.name);
            mountItems.push_back(makeComboItem(
                QString("%1 [%2.%3]")
                    .arg(name)
                    .arg(QString::fromStdString(mount.robotId))
                    .arg(QString::fromStdString(mount.linkName)),
                id));
        }

        QVector<ToolSetupComboItem> objectItems;
        objectItems.reserve(static_cast<int>(document.objects.size()));
        objectItems.push_back(makeComboItem(QStringLiteral("Select object..."), QString()));
        for(const simulation_project::SceneObjectDesc& object : document.objects) {
            if(object.sourcePath.empty() || !object.visible) {
                continue;
            }
            const QString id = QString::fromStdString(object.id);
            const QString name = QString::fromStdString(object.name.empty() ? object.id : object.name);
            objectItems.push_back(makeComboItem(
                name == id ? id : QString("%1 [%2]").arg(name, conciseUiText(object.id, 32)),
                id));
        }

        QVector<ToolSetupComboItem> frameItems;
        const simulation_project::SceneObjectDesc* object =
            findSceneObjectDesc(document, objectId.toStdString());
        if(object != nullptr) {
            frameItems.push_back(makeComboItem(QStringLiteral("Object origin"), QString()));
            for(const simulation_project::ObjectFrameDesc& frame : object->objectFrames) {
                const QString id = QString::fromStdString(frame.id);
                const QString name = QString::fromStdString(frame.name.empty() ? frame.id : frame.name);
                frameItems.push_back(makeComboItem(
                    name == id ? id : QString("%1 [%2]").arg(name, conciseUiText(frame.id, 32)),
                    id));
            }
        }

        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(document, mountId.toStdString());
        const bool applyEnabled = mount != nullptr && object != nullptr && !object->sourcePath.empty();
        QString details = statusMessage;
        if(details.isEmpty()) {
            details = applyEnabled
                ? QStringLiteral("Preview uses Mount -> inverse(Object Frame) -> Object.")
                : QStringLiteral("Select a valid mount frame and object.");
        }
        if(mount != nullptr) {
            details += QString("\nmount: %1 [%2.%3]")
                .arg(QString::fromStdString(mount->id))
                .arg(QString::fromStdString(mount->robotId))
                .arg(QString::fromStdString(mount->linkName));
        }
        if(object != nullptr) {
            details += QString("\nobject: %1\nsource: %2")
                .arg(QString::fromStdString(object->id))
                .arg(conciseUiText(object->sourcePath, 72));
        }
        if(object != nullptr && !frameId.isEmpty()) {
            const simulation_project::ObjectFrameDesc* frame =
                findObjectFrameDesc(*object, frameId.toStdString());
            if(frame != nullptr) {
                details += QString("\nobject frame: %1")
                    .arg(QString::fromStdString(frame->name.empty() ? frame->id : frame->name));
            }
        }

        ToolSetupBindingView bindingView = makeBindingView(document, mountId, objectId, frameId);
        bindingView.editable = true;
        m_widget.setObjectBindingMode(true);
        m_widget.setObjectBindingEditor(
            true,
            bindingView,
            mountItems,
            mountId,
            objectItems,
            objectId,
            frameItems,
            frameId,
            details,
            applyEnabled);
    }

    void ToolSetupModuleController::applyObjectBinding(
        const QString& mountId,
        const QString& objectId,
        const QString& frameId)
    {
        if(!m_objectBindingTaskActive) {
            return;
        }

        QString attachmentId;
        QString assetId;
        if(!previewObjectBinding(mountId, objectId, frameId, &attachmentId, &assetId)) {
            refreshObjectBindingEditor(mountId, objectId, frameId, QStringLiteral("Binding cannot be applied."));
            return;
        }

        m_hasTaskRollbackDocument = false;
        m_taskRollbackDocument = simulation_project::ProjectDocument();
        m_context.documentController().mutateProject(
            QStringLiteral("applyObjectBinding"),
            ProjectDirtyPolicy::UserEdit,
            [](simulation_project::ProjectDocumentService&, bool& changed, std::string&) {
                changed = true;
                return true;
            });
        m_objectBindingTaskActive = false;
        m_bindingMountId.clear();
        m_bindingObjectId.clear();
        m_bindingFrameId.clear();
        m_mountFrameMode = ToolSetupMountFrameMode::Selection;
        clearMountEditSnapshot();
        m_widget.setObjectBindingMode(false);
        setTaskDirty(false, QStringLiteral("Object binding applied."));
        RobotQtViewerAttachmentPayload payload;
        payload.mountId = mountId;
        payload.attachmentId = attachmentId;
        payload.assetId = assetId;
        m_context.documentController().publishAttachmentChanged(payload, QStringLiteral("applyObjectBinding"));
        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(m_context.document(), mountId.toStdString());
        if(mount != nullptr) {
            const QString robotId = QString::fromStdString(mount->robotId);
            const QString linkName = QString::fromStdString(mount->linkName);
            m_appServices.setRobotMountContext(robotId, linkName, mountId);
            m_context.selectionModel().selectRobotMount(
                robotId,
                linkName,
                mountId,
                QStringLiteral("applyObjectBinding"));
            refresh(robotId, linkName, mountId, attachmentId, assetId);
            emit robotContextSelected(robotId);
            emit mountFrameFocusRequested(robotId, linkName, mountId);
            emit selectionDependentViewsRefreshRequested();
        }
        clearObjectBindingPreviewViewportState();
        emit statusMessageRequested(
            QString("Bound object %1 to mount %2.").arg(objectId, mountId),
            5000);
    }

    void ToolSetupModuleController::unbindMountedAttachment(const QString& attachmentId)
    {
        if(attachmentId.isEmpty()) {
            emit statusMessageRequested(QStringLiteral("No mounted attachment selected."), 3000);
            return;
        }

        if(!resolvePendingTaskChanges(&m_widget)) {
            emit statusMessageRequested(QStringLiteral("Finish or cancel the current task first."), 3000);
            return;
        }

        const simulation_project::MountedAttachmentDesc* attachment =
            findMountedAttachmentDesc(m_context.document(), attachmentId.toStdString());
        if(attachment == nullptr) {
            emit statusMessageRequested(QString("Mounted attachment not found: %1").arg(attachmentId), 5000);
            return;
        }

        const std::string mountId = attachment->mountFrameId;
        const std::string assetId = attachment->assetId;
        const std::string attachmentName = attachment->name.empty() ? attachment->id : attachment->name;
        const simulation_project::AttachmentAssetDesc* asset =
            findAttachmentAssetDesc(m_context.document(), assetId);
        std::string sourceObjectId = attachment->sourceObjectId;
        if(sourceObjectId.empty() && asset != nullptr) {
            sourceObjectId = findHiddenSceneObjectIdBySourcePath(m_context.document(), asset->visualPath);
        }

        const QMessageBox::StandardButton choice = QMessageBox::question(
            &m_widget,
            QStringLiteral("Unbind Attachment"),
            QString("Unbind attachment %1 from its mount frame?").arg(QString::fromStdString(attachmentName)),
            QMessageBox::Yes | QMessageBox::No,
            QMessageBox::No);
        if(choice != QMessageBox::Yes) {
            return;
        }

        const simulation_project::ProjectDocument previousDocument = m_context.document();
        const bool previousDirty = m_context.projectSession().isDirty();

        bool assetRemoved = false;
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("unbindMountedAttachment"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(!sourceObjectId.empty() &&
                    !service.setSceneObjectVisibility(sourceObjectId, true, &error)) {
                    return false;
                }
                if(!service.removeMountedAttachment(attachmentId.toStdString(), &error)) {
                    return false;
                }
                if(!assetId.empty() && service.findAttachmentAsset(assetId) != nullptr &&
                    !service.removeAttachmentAssetIfUnused(assetId, &assetRemoved, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            emit statusMessageRequested(
                QString("Unbind attachment failed: %1").arg(mutationResult.message),
                5000);
            return;
        }

        const ToolSetupViewportReloadResult reloadResult =
            m_appServices.reloadViewport(QStringLiteral("unbindMountedAttachment"));
        if(!reloadResult.success) {
            m_context.documentController().restoreProjectSnapshot(
                QStringLiteral("unbindMountedAttachmentRollback"),
                previousDocument,
                previousDirty,
                false);
            m_appServices.reloadViewport(QStringLiteral("unbindMountedAttachmentRollback"));
            emit statusMessageRequested(
                reloadResult.errorMessage.isEmpty()
                    ? QStringLiteral("Unbind attachment failed: viewport reload failed.")
                    : reloadResult.errorMessage,
                8000);
            return;
        }

        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(m_context.document(), mountId);
        if(mount != nullptr) {
            m_context.selectionModel().selectRobotMount(
                QString::fromStdString(mount->robotId),
                QString::fromStdString(mount->linkName),
                QString::fromStdString(mount->id),
                QStringLiteral("unbindMountedAttachment"));
        }

        RobotQtViewerAttachmentPayload payload;
        payload.mountId = QString::fromStdString(mountId);
        payload.attachmentId = attachmentId;
        payload.assetId = QString::fromStdString(assetId);
        m_context.documentController().publishAttachmentChanged(
            payload,
            QStringLiteral("unbindMountedAttachment"));
        m_context.documentController().publishDocumentChanged(
            QStringLiteral("unbindMountedAttachment"),
            false);
        emit selectionDependentViewsRefreshRequested();
        emit statusMessageRequested(
            assetRemoved
                ? QString("Unbound attachment %1 and removed unused asset.").arg(attachmentId)
                : QString("Unbound attachment %1.").arg(attachmentId),
            5000);
    }

    void ToolSetupModuleController::updateToolFrameVisibility()
    {
        const QString currentMountId = m_widget.currentMountId();
        if(!m_updating && !currentMountId.isEmpty()) {
            if(m_widget.showRobotMountFrame()) {
                m_pinnedRobotMountFrameIds.insert(currentMountId);
            } else {
                m_pinnedRobotMountFrameIds.remove(currentMountId);
            }
        }

        RobotQtViewerToolFrameVisibility visibility;
        visibility.link = m_widget.showLinkFrame();
        visibility.robotMount = m_widget.showRobotMountFrame();
        visibility.toolMount = m_widget.showToolMountFrame();
        visibility.visual = m_widget.showVisualFrame();
        visibility.tcp = m_widget.showTcpFrame();
        visibility.sensorPreview = m_widget.showSensorPreview();
        RobotQtViewerViewportPreviewPayload preview;
        preview.setToolFrameVisibility = true;
        preview.toolFrameVisibility = visibility;
        const bool mountFrameTaskActive =
            m_mountFrameMode == ToolSetupMountFrameMode::Create ||
            m_mountFrameMode == ToolSetupMountFrameMode::Edit;
        preview.setRobotMountFrameVisibility = true;
        preview.selectedLinkFrameVisible = mountFrameTaskActive;
        preview.mountFrameVisible = mountFrameTaskActive || m_widget.showRobotMountFrame();
        mutateViewportPreview(preview, QStringLiteral("toolSetupFrameVisibility"));
        syncPinnedRobotMountFrames();
        emit frameVisibilityChanged();
    }

    void ToolSetupModuleController::syncPinnedRobotMountFrames()
    {
        RobotQtViewerViewportPreviewPayload preview;
        preview.setPinnedRobotMountFrames = true;
        preview.pinnedRobotMountFrameIds = pinnedRobotMountFrameIds();
        mutateViewportPreview(preview, QStringLiteral("toolSetupPinnedMountFrames"));
    }

    QStringList ToolSetupModuleController::pinnedRobotMountFrameIds() const
    {
        QStringList ids;
        for(const QString& id : m_pinnedRobotMountFrameIds) {
            if(!id.isEmpty() && findRobotMountDesc(m_context.document(), id.toStdString()) != nullptr) {
                ids.push_back(id);
            }
        }
        ids.sort();
        return ids;
    }

    std::string ToolSetupModuleController::makeUniqueToolAssetId(const std::string& baseName) const
    {
        const std::string cleanBase = makeAsciiSlug(baseName, "tool_asset", 32);

        int suffix = 0;
        while(true) {
            const std::string candidate = suffix == 0
                ? cleanBase
                : cleanBase + "_" + QString("%1").arg(suffix, 3, 10, QLatin1Char('0')).toStdString();
            if(findAttachmentAssetDesc(m_context.document(), candidate) == nullptr &&
                findRobotDesc(m_context.document(), candidate) == nullptr &&
                findRobotMountDesc(m_context.document(), candidate) == nullptr &&
                findMountedAttachmentDesc(m_context.document(), candidate) == nullptr) {
                return candidate;
            }
            ++suffix;
        }
    }

    std::string ToolSetupModuleController::makeUniqueToolAttachmentId(const std::string& baseName) const
    {
        const std::string cleanBase = makeAsciiSlug(baseName, "tool_attachment", 32);

        int suffix = 0;
        while(true) {
            const std::string candidate = suffix == 0
                ? cleanBase
                : cleanBase + "_" + QString("%1").arg(suffix, 3, 10, QLatin1Char('0')).toStdString();
            if(findMountedAttachmentDesc(m_context.document(), candidate) == nullptr &&
                findAttachmentAssetDesc(m_context.document(), candidate) == nullptr &&
                findRobotDesc(m_context.document(), candidate) == nullptr &&
                findRobotMountDesc(m_context.document(), candidate) == nullptr) {
                return candidate;
            }
            ++suffix;
        }
    }
}
