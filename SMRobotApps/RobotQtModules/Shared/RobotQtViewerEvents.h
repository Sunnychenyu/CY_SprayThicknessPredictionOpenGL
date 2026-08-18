#pragma once

#include <QVector>
#include <QString>
#include <QStringList>

#include <SimulationProject/ProjectDocument.h>

namespace robot_qt_viewer
{
    enum class RobotQtViewerEventKind
    {
        ProjectOpened,
        ProjectSaved,
        ProjectDocumentChanged,
        ProjectDirtyChanged,
        SelectionChanged,
        ViewportReloadRequested,
        ViewportReloaded,
        RobotRuntimeChanged,
        ToolSetupChanged,
        AttachmentChanged,
        CollisionChanged,
        CollisionSelectionChanged,
        CoatingAnalysisChanged,
        TaskStateChanged,
        ViewportPreviewChanged,
        StatusMessageRequested
    };

    struct RobotQtViewerSelectionPayload
    {
        QString robotId;
        QString linkName;
        QString objectId;
        QString objectFrameId;
        QString mountId;
        QString attachmentId;
        QString assetId;
        QString jointName;
        QString collisionDetectorId;
        QString collisionPairRobotA;
        QString collisionPairLinkA;
    };

    struct RobotQtViewerAttachmentPayload
    {
        QString mountId;
        QString attachmentId;
        QString assetId;
    };

    struct RobotQtViewerCollisionPayload
    {
        QString detectorId;
        QString selectionSetId;
        QString variantSource;
        QString variantRole;
        QString elementId;
    };

    struct RobotQtViewerViewportPayload
    {
        bool reloadRequested = false;
        bool reloadSucceeded = false;
    };

    struct RobotQtViewerCoatingAnalysisPayload
    {
        QString objectId;
        bool hasResult = false;
        bool showThickness = false;
        double minimumThicknessMeters = 0.0;
        double maximumThicknessMeters = 0.0;
    };

    struct RobotQtViewerToolFrameVisibility
    {
        bool link = false;
        bool robotMount = false;
        bool toolMount = true;
        bool visual = true;
        bool tcp = true;
        bool sensorPreview = true;
    };

    struct RobotQtViewerViewportPreviewPayload
    {
        bool setActivePreviewRobotMount = false;
        QString activePreviewRobotMountId;
        bool setRobotMountFrameVisibility = false;
        bool selectedLinkFrameVisible = false;
        bool mountFrameVisible = false;
        bool previewRobotMountTransform = false;
        QString previewRobotMountId;
        simulation_project::TransformDesc robotMountTransform;
        bool upsertPreviewRobotMount = false;
        simulation_project::RobotMountDesc robotMount;
        bool removePreviewRobotMount = false;
        QString removePreviewRobotMountId;
        bool upsertPreviewObjectFrame = false;
        QString objectFrameObjectId;
        simulation_project::ObjectFrameDesc objectFrame;
        bool previewRobotBaseTransform = false;
        QString robotBaseRobotId;
        simulation_project::TransformDesc robotBaseTransform;
        bool previewObjectFrameTransform = false;
        QString previewObjectFrameObjectId;
        QString previewObjectFrameId;
        simulation_project::TransformDesc objectFrameTransform;
        bool previewSceneObjectTransform = false;
        QString sceneObjectId;
        simulation_project::TransformDesc sceneObjectTransform;
        bool focusMountFrameLink = false;
        QString focusMountFrameRobotId;
        QString focusMountFrameLinkName;
        bool clearMountFrameLinkFocus = false;
        bool focusObjectFrameObject = false;
        QString focusObjectFrameObjectId;
        bool clearObjectFrameObjectFocus = false;
        bool setActiveMountedAttachment = false;
        QString activeMountedAttachmentId;
        bool focusMountedAttachment = false;
        QString focusMountedAttachmentId;
        bool clearMountedAttachmentFocus = false;
        bool previewObjectCollisionModelVariant = false;
        QString previewObjectCollisionModelObjectId;
        QString previewObjectCollisionModelVariantId;
        bool clearObjectCollisionModelVariantPreview = false;
        bool setToolFrameVisibility = false;
        RobotQtViewerToolFrameVisibility toolFrameVisibility;
        bool setPinnedRobotMountFrames = false;
        QStringList pinnedRobotMountFrameIds;
    };

    struct RobotQtViewerEvent
    {
        RobotQtViewerEventKind kind = RobotQtViewerEventKind::ProjectDocumentChanged;
        QString sourceId;
        QVector<QString> affectedIds;
        QString message;
        int timeoutMs = 0;
        bool projectChanged = false;
        bool projectDirty = false;
        bool viewportReloadRequested = false;
        bool refreshRequested = false;
        RobotQtViewerSelectionPayload selection;
        RobotQtViewerAttachmentPayload attachment;
        RobotQtViewerCollisionPayload collision;
        RobotQtViewerCoatingAnalysisPayload coatingAnalysis;
        RobotQtViewerViewportPayload viewport;
    };

    inline QString robotQtViewerEventKindName(RobotQtViewerEventKind kind)
    {
        switch(kind) {
        case RobotQtViewerEventKind::ProjectOpened:
            return QStringLiteral("ProjectOpened");
        case RobotQtViewerEventKind::ProjectSaved:
            return QStringLiteral("ProjectSaved");
        case RobotQtViewerEventKind::ProjectDocumentChanged:
            return QStringLiteral("ProjectDocumentChanged");
        case RobotQtViewerEventKind::ProjectDirtyChanged:
            return QStringLiteral("ProjectDirtyChanged");
        case RobotQtViewerEventKind::SelectionChanged:
            return QStringLiteral("SelectionChanged");
        case RobotQtViewerEventKind::ViewportReloadRequested:
            return QStringLiteral("ViewportReloadRequested");
        case RobotQtViewerEventKind::ViewportReloaded:
            return QStringLiteral("ViewportReloaded");
        case RobotQtViewerEventKind::RobotRuntimeChanged:
            return QStringLiteral("RobotRuntimeChanged");
        case RobotQtViewerEventKind::ToolSetupChanged:
            return QStringLiteral("ToolSetupChanged");
        case RobotQtViewerEventKind::AttachmentChanged:
            return QStringLiteral("AttachmentChanged");
        case RobotQtViewerEventKind::CollisionChanged:
            return QStringLiteral("CollisionChanged");
        case RobotQtViewerEventKind::CollisionSelectionChanged:
            return QStringLiteral("CollisionSelectionChanged");
        case RobotQtViewerEventKind::CoatingAnalysisChanged:
            return QStringLiteral("CoatingAnalysisChanged");
        case RobotQtViewerEventKind::TaskStateChanged:
            return QStringLiteral("TaskStateChanged");
        case RobotQtViewerEventKind::ViewportPreviewChanged:
            return QStringLiteral("ViewportPreviewChanged");
        case RobotQtViewerEventKind::StatusMessageRequested:
            return QStringLiteral("StatusMessageRequested");
        }
        return QStringLiteral("ProjectDocumentChanged");
    }

    inline bool robotQtViewerEventKindFromName(const QString& name, RobotQtViewerEventKind& kind)
    {
        const RobotQtViewerEventKind allKinds[] = {
            RobotQtViewerEventKind::ProjectOpened,
            RobotQtViewerEventKind::ProjectSaved,
            RobotQtViewerEventKind::ProjectDocumentChanged,
            RobotQtViewerEventKind::ProjectDirtyChanged,
            RobotQtViewerEventKind::SelectionChanged,
            RobotQtViewerEventKind::ViewportReloadRequested,
            RobotQtViewerEventKind::ViewportReloaded,
            RobotQtViewerEventKind::RobotRuntimeChanged,
            RobotQtViewerEventKind::ToolSetupChanged,
            RobotQtViewerEventKind::AttachmentChanged,
            RobotQtViewerEventKind::CollisionChanged,
            RobotQtViewerEventKind::CollisionSelectionChanged,
            RobotQtViewerEventKind::CoatingAnalysisChanged,
            RobotQtViewerEventKind::TaskStateChanged,
            RobotQtViewerEventKind::ViewportPreviewChanged,
            RobotQtViewerEventKind::StatusMessageRequested
        };

        for(RobotQtViewerEventKind candidate : allKinds) {
            if(robotQtViewerEventKindName(candidate) == name) {
                kind = candidate;
                return true;
            }
        }
        return false;
    }
}
