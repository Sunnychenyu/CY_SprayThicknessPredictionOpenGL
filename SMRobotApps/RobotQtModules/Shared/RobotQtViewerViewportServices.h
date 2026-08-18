#pragma once

#include <filesystem>
#include <vector>

#include <QString>
#include <QStringList>

#include <SimulationProject/ProjectDocument.h>
#include <VisualizationSDK/SurfaceScalarOverlay.h>

#include "CollisionRuntimeViewModel.h"
#include "RobotQtViewerEvents.h"

namespace robot_qt_viewer
{
    struct RobotQtViewerViewportLoadResult
    {
        bool success = false;
        QString errorMessage;
    };

    class RobotQtViewerViewportServices
    {
    public:
        virtual ~RobotQtViewerViewportServices() = default;

        virtual void selectRobotMount(
            const QString& robotId,
            const QString& linkName,
            const QString& robotMountId) = 0;
        virtual bool setActivePreviewRobotMount(const QString& robotMountId) = 0;
        virtual bool previewRobotMountTransform(
            const QString& robotMountId,
            const simulation_project::TransformDesc& transform) = 0;
        virtual bool previewRobotMountLink(
            const QString& robotMountId,
            const QString& linkName) = 0;
        virtual bool upsertPreviewRobotMount(const simulation_project::RobotMountDesc& mount) = 0;
        virtual bool removePreviewRobotMount(const QString& robotMountId) = 0;
        virtual void previewRobotBaseTransform(
            const QString& robotId,
            const simulation_project::TransformDesc& transform) = 0;
        virtual void previewSceneObjectTransform(
            const QString& objectId,
            const simulation_project::TransformDesc& transform) = 0;
        virtual bool commitSceneObjectTransform(
            const QString& objectId,
            const simulation_project::TransformDesc& transform) = 0;
        virtual bool removeSceneObject(const QString& objectId) = 0;
        virtual void selectObjectFrame(
            const QString& objectId,
            const QString& frameId) = 0;
        virtual bool previewObjectFrameTransform(
            const QString& objectId,
            const QString& frameId,
            const simulation_project::TransformDesc& transform) = 0;
        virtual bool upsertPreviewObjectFrame(
            const QString& objectId,
            const simulation_project::ObjectFrameDesc& frame) = 0;
        virtual void selectRobotLink(
            const QString& robotId,
            const QString& linkName) = 0;
        virtual void selectRobotJointFrame(
            const QString& robotId,
            const QString& jointName) = 0;
        virtual void setActiveToolFrameRobot(const QString& robotId) = 0;
        virtual void selectSceneObject(const QString& objectId) = 0;
        virtual void selectMountedAttachment(const QString& attachmentId) = 0;
        virtual bool setActiveMountedAttachment(const QString& attachmentId) = 0;
        virtual void setToolFrameVisibility(const RobotQtViewerToolFrameVisibility& visibility) = 0;
        virtual void setRobotMountFrameVisibility(bool selectedLinkFrameVisible, bool mountFrameVisible) = 0;
        virtual void setPinnedRobotMountFrames(const QStringList& robotMountIds) = 0;
        virtual void focusMountFrameLink(const QString& robotId, const QString& linkName) = 0;
        virtual void clearMountFrameLinkFocus() = 0;
        virtual void focusObjectFrameObject(const QString& objectId) = 0;
        virtual void clearObjectFrameObjectFocus() = 0;
        virtual void focusMountedAttachment(const QString& attachmentId) = 0;
        virtual void clearMountedAttachmentFocus() = 0;
        virtual void previewObjectCollisionModelVariant(
            const QString& objectId,
            const QString& variantId) = 0;
        virtual void clearObjectCollisionModelVariantPreview() = 0;
        virtual void previewCollisionPairTargets(
            const QString& robotAId,
            const QString& linkAName,
            const QString& objectAId,
            const QString& attachmentAId,
            const QString& robotBId,
            const QString& linkBName,
            const QString& objectBId,
            const QString& attachmentBId) = 0;
        virtual RobotQtViewerViewportLoadResult loadProjectDocument(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& basePath) = 0;
        virtual bool refreshCollisionConfiguration(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& basePath) = 0;
        virtual void setCollisionGeometryVisible(bool visible) = 0;
        virtual bool collisionQueriesEnabled() const = 0;
        virtual bool setCollisionQueriesEnabled(bool enabled) = 0;
        virtual bool setActiveCollisionDetector(const QString& detectorId) = 0;
        virtual bool setCollisionDetectorEnabled(const QString& detectorId, bool enabled) = 0;
        virtual bool setCollisionDetectorVisible(const QString& detectorId, bool visible) = 0;
        virtual bool updateCollisionDetectorRuntimeOptions(
            const simulation_project::CollisionDetectorDesc& detector) = 0;
        virtual bool rebuildCollisionDetectorsFromDocument(
            const simulation_project::ProjectDocument& document) = 0;
        virtual bool removeCollisionDetector(const QString& detectorId) = 0;
        virtual bool setVisibleRobotCollisionVariant(
            const QString& robotId,
            const QString& linkName,
            const QString& variantId) = 0;
        virtual QString visibleRobotCollisionVariant(
            const QString& robotId,
            const QString& linkName) const = 0;
        virtual CollisionRuntimeRobotSummary robotCollisionSummary(
            const QString& robotId,
            const QString& activeDetectorRole = QString(),
            const QString& activeDetectorSource = QString()) const = 0;
        virtual std::vector<CollisionRuntimeDetectorInfo> collisionRuntimeDetectors() const = 0;
        virtual bool generateRobotCollisionProxies(
            const QString& robotId,
            const QString& linkName,
            const CollisionRuntimeProxyRequest& request,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const = 0;
        virtual bool generateRobotCollisionProxiesFromExistingCollision(
            const QString& robotId,
            const QString& linkName,
            const CollisionRuntimeProxyRequest& request,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const = 0;
        virtual bool generateRobotCollisionProxiesFromExistingCollision(
            const QString& robotId,
            const CollisionRuntimeProxyRequest& request,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const = 0;
        virtual bool generateRobotCollisionCoacdFromVisual(
            const QString& robotId,
            const QString& linkName,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const = 0;
        virtual bool generateRobotCollisionCoacdFromExistingCollision(
            const QString& robotId,
            const QString& linkName,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const = 0;
        virtual bool generateObjectCollisionCoacdFromVisual(
            const QString& objectId,
            std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements) const = 0;
        virtual bool generateMissingRobotCollisionProxies(
            const QString& robotId,
            const CollisionRuntimeProxyRequest& request,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const = 0;
        virtual bool evaluateRobotCollisionProxyQuality(
            const QString& robotId,
            const QString& linkName,
            const CollisionRuntimeProxyRequest& request,
            const std::vector<simulation_project::CollisionElementOverrideDesc>& elements,
            bool useExistingCollisionInput,
            CollisionRuntimeProxyQualitySummary& summary) const = 0;
        virtual double robotJointValue(
            const QString& robotId,
            const QString& jointName,
            bool* ok = nullptr) const = 0;
        virtual void setRobotJointValue(
            const QString& robotId,
            const QString& jointName,
            double value) = 0;
        virtual void setRobotAutoMotion(
            const QString& robotId,
            bool enabled,
            double amplitude,
            double speed) = 0;
        virtual bool applySurfaceScalarOverlay(
            const smrobot::visualization::SurfaceScalarOverlay& overlay,
            QString* errorMessage)
        {
            (void)overlay;
            if(errorMessage != nullptr) {
                *errorMessage = QStringLiteral("Surface scalar overlays are not supported by this viewport.");
            }
            return false;
        }
        virtual bool setSurfaceScalarOverlayVisible(const QString& objectId, bool visible)
        {
            (void)objectId;
            (void)visible;
            return false;
        }
        virtual bool clearSurfaceScalarOverlay(const QString& objectId)
        {
            (void)objectId;
            return false;
        }
        virtual void setSurfaceScalarProbeEnabled(bool enabled, const QString& objectId)
        {
            (void)enabled;
            (void)objectId;
        }
    };
}
