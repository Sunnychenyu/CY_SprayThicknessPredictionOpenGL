#pragma once

#include "RobotQtViewerViewportServices.h"

#include <RobotRuntime/RobotRunService.h>
#include <RobotRuntime/RobotTrajectoryExecutionSession.h>

class RobotViewport;

namespace robot_qt_viewer
{
    class RobotQtViewerViewportServicesAdapter :
        public RobotQtViewerViewportServices,
        public robotruntime::IRobotRunService
    {
    public:
        explicit RobotQtViewerViewportServicesAdapter(RobotViewport& viewport);

        void selectRobotMount(
            const QString& robotId,
            const QString& linkName,
            const QString& robotMountId) override;
        bool setActivePreviewRobotMount(const QString& robotMountId) override;
        bool previewRobotMountTransform(
            const QString& robotMountId,
            const simulation_project::TransformDesc& transform) override;
        bool previewRobotMountLink(
            const QString& robotMountId,
            const QString& linkName) override;
        bool upsertPreviewRobotMount(const simulation_project::RobotMountDesc& mount) override;
        bool removePreviewRobotMount(const QString& robotMountId) override;
        void previewRobotBaseTransform(
            const QString& robotId,
            const simulation_project::TransformDesc& transform) override;
        void previewSceneObjectTransform(
            const QString& objectId,
            const simulation_project::TransformDesc& transform) override;
        bool commitSceneObjectTransform(
            const QString& objectId,
            const simulation_project::TransformDesc& transform) override;
        bool removeSceneObject(const QString& objectId) override;
        void selectObjectFrame(
            const QString& objectId,
            const QString& frameId) override;
        bool previewObjectFrameTransform(
            const QString& objectId,
            const QString& frameId,
            const simulation_project::TransformDesc& transform) override;
        bool upsertPreviewObjectFrame(
            const QString& objectId,
            const simulation_project::ObjectFrameDesc& frame) override;
        void selectRobotLink(
            const QString& robotId,
            const QString& linkName) override;
        void selectRobotJointFrame(
            const QString& robotId,
            const QString& jointName) override;
        void setActiveToolFrameRobot(const QString& robotId) override;
        void selectSceneObject(const QString& objectId) override;
        void selectMountedAttachment(const QString& attachmentId) override;
        bool setActiveMountedAttachment(const QString& attachmentId) override;
        void setToolFrameVisibility(const RobotQtViewerToolFrameVisibility& visibility) override;
        void setRobotMountFrameVisibility(bool selectedLinkFrameVisible, bool mountFrameVisible) override;
        void setPinnedRobotMountFrames(const QStringList& robotMountIds) override;
        void focusMountFrameLink(const QString& robotId, const QString& linkName) override;
        void clearMountFrameLinkFocus() override;
        void focusObjectFrameObject(const QString& objectId) override;
        void clearObjectFrameObjectFocus() override;
        void focusMountedAttachment(const QString& attachmentId) override;
        void clearMountedAttachmentFocus() override;
        void previewObjectCollisionModelVariant(
            const QString& objectId,
            const QString& variantId) override;
        void clearObjectCollisionModelVariantPreview() override;
        void previewCollisionPairTargets(
            const QString& robotAId,
            const QString& linkAName,
            const QString& objectAId,
            const QString& attachmentAId,
            const QString& robotBId,
            const QString& linkBName,
            const QString& objectBId,
            const QString& attachmentBId) override;
        RobotQtViewerViewportLoadResult loadProjectDocument(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& basePath) override;
        bool refreshCollisionConfiguration(
            const simulation_project::ProjectDocument& document,
            const std::filesystem::path& basePath) override;
        void setCollisionGeometryVisible(bool visible) override;
        bool collisionQueriesEnabled() const override;
        bool setCollisionQueriesEnabled(bool enabled) override;
        bool setActiveCollisionDetector(const QString& detectorId) override;
        bool setCollisionDetectorEnabled(const QString& detectorId, bool enabled) override;
        bool setCollisionDetectorVisible(const QString& detectorId, bool visible) override;
        bool updateCollisionDetectorRuntimeOptions(
            const simulation_project::CollisionDetectorDesc& detector) override;
        bool rebuildCollisionDetectorsFromDocument(
            const simulation_project::ProjectDocument& document) override;
        bool removeCollisionDetector(const QString& detectorId) override;
        bool setVisibleRobotCollisionVariant(
            const QString& robotId,
            const QString& linkName,
            const QString& variantId) override;
        QString visibleRobotCollisionVariant(
            const QString& robotId,
            const QString& linkName) const override;
        CollisionRuntimeRobotSummary robotCollisionSummary(
            const QString& robotId,
            const QString& activeDetectorRole = QString(),
            const QString& activeDetectorSource = QString()) const override;
        std::vector<CollisionRuntimeDetectorInfo> collisionRuntimeDetectors() const override;
        bool generateRobotCollisionProxies(
            const QString& robotId,
            const QString& linkName,
            const CollisionRuntimeProxyRequest& request,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const override;
        bool generateRobotCollisionProxiesFromExistingCollision(
            const QString& robotId,
            const QString& linkName,
            const CollisionRuntimeProxyRequest& request,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const override;
        bool generateRobotCollisionProxiesFromExistingCollision(
            const QString& robotId,
            const CollisionRuntimeProxyRequest& request,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const override;
        bool generateRobotCollisionCoacdFromVisual(
            const QString& robotId,
            const QString& linkName,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const override;
        bool generateRobotCollisionCoacdFromExistingCollision(
            const QString& robotId,
            const QString& linkName,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const override;
        bool generateObjectCollisionCoacdFromVisual(
            const QString& objectId,
            std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements) const override;
        bool generateMissingRobotCollisionProxies(
            const QString& robotId,
            const CollisionRuntimeProxyRequest& request,
            std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const override;
        bool evaluateRobotCollisionProxyQuality(
            const QString& robotId,
            const QString& linkName,
            const CollisionRuntimeProxyRequest& request,
            const std::vector<simulation_project::CollisionElementOverrideDesc>& elements,
            bool useExistingCollisionInput,
            CollisionRuntimeProxyQualitySummary& summary) const override;
        double robotJointValue(
            const QString& robotId,
            const QString& jointName,
            bool* ok = nullptr) const override;
        void setRobotJointValue(
            const QString& robotId,
            const QString& jointName,
            double value) override;
        void setRobotAutoMotion(
            const QString& robotId,
            bool enabled,
            double amplitude,
            double speed) override;
        bool applySurfaceScalarOverlay(
            const smrobot::visualization::SurfaceScalarOverlay& overlay,
            QString* errorMessage) override;
        bool setSurfaceScalarOverlayVisible(const QString& objectId, bool visible) override;
        bool clearSurfaceScalarOverlay(const QString& objectId) override;
        void setSurfaceScalarProbeEnabled(bool enabled, const QString& objectId) override;

        bool jointValue(
            const std::string& robotId,
            const std::string& jointName,
            double& value) const override;
        robotruntime::RobotRunCommandResult setJointValue(
            const std::string& robotId,
            const std::string& jointName,
            double value) override;
        robotruntime::RobotRunCommandResult setAutoMotion(
            const std::string& robotId,
            bool enabled,
            double amplitude,
            double speed) override;
        robotruntime::RobotRunCommandResult loadTrajectory(
            const std::string& robotId,
            const std::string& trajectoryId,
            const std::vector<std::string>& jointNames,
            const robottrajectory::JointTrajectory& trajectory) override;
        robotruntime::RobotRunCommandResult startTrajectory() override;
        robotruntime::RobotRunCommandResult pauseTrajectory() override;
        robotruntime::RobotRunCommandResult stopTrajectory() override;
        robotruntime::RobotRunCommandResult stepTrajectory(double timeStep) override;
        robotruntime::RobotRunExecutionSnapshot trajectorySnapshot() const override;

    private:
        RobotViewport& m_viewport;
        robotruntime::RobotTrajectoryExecutionSession m_trajectorySession;
        std::vector<std::string> m_trajectoryJointNames;
    };
}
