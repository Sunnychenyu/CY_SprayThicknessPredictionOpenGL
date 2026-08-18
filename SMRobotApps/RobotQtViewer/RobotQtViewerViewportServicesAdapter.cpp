#include "RobotQtViewerViewportServicesAdapter.h"

#include "ProjectScene.h"
#include "RobotViewport.h"

namespace
{
    robot_qt_viewer::CollisionRuntimeDetectorInfo::Vec3Info toRuntimeVec3(
        const ProjectScene::CollisionDetectorInfo::Vec3Info& value)
    {
        robot_qt_viewer::CollisionRuntimeDetectorInfo::Vec3Info result;
        result.x = value.x;
        result.y = value.y;
        result.z = value.z;
        return result;
    }

    robot_qt_viewer::CollisionRuntimeDetectorInfo::ContactInfo toRuntimeContact(
        const ProjectScene::CollisionDetectorInfo::ContactInfo& value)
    {
        robot_qt_viewer::CollisionRuntimeDetectorInfo::ContactInfo result;
        result.bodyA = value.bodyA;
        result.bodyB = value.bodyB;
        result.position = toRuntimeVec3(value.position);
        result.normal = toRuntimeVec3(value.normal);
        result.penetrationDepth = value.penetrationDepth;
        return result;
    }

    robot_qt_viewer::CollisionRuntimeDetectorInfo::NearestInfo toRuntimeNearest(
        const ProjectScene::CollisionDetectorInfo::NearestInfo& value)
    {
        robot_qt_viewer::CollisionRuntimeDetectorInfo::NearestInfo result;
        result.valid = value.valid;
        result.state = value.state;
        result.reason = value.reason;
        result.bodyA = value.bodyA;
        result.bodyB = value.bodyB;
        result.pointA = toRuntimeVec3(value.pointA);
        result.pointB = toRuntimeVec3(value.pointB);
        result.distance = value.distance;
        return result;
    }

    robot_qt_viewer::CollisionRuntimeDetectorInfo toRuntimeDetector(
        const ProjectScene::CollisionDetectorInfo& value)
    {
        robot_qt_viewer::CollisionRuntimeDetectorInfo result;
        result.id = value.id;
        result.name = value.name;
        result.type = value.type;
        result.enabled = value.enabled;
        result.visible = value.visible;
        result.active = value.active;
        result.includePairCount = value.includePairCount;
        result.effectiveIncludePairCount = value.effectiveIncludePairCount;
        result.contactCount = value.contactCount;
        result.inCollision = value.inCollision;
        result.minDistance = value.minDistance;
        result.lastCheckMs = value.lastCheckMs;
        result.lastDistanceMs = value.lastDistanceMs;
        result.lastQueryMs = value.lastQueryMs;
        result.frameRobotPoseMs = value.frameRobotPoseMs;
        result.frameCollisionWorldUpdateMs = value.frameCollisionWorldUpdateMs;
        result.frameOverlayMs = value.frameOverlayMs;
        result.frameOverlayHighlightMs = value.frameOverlayHighlightMs;
        result.frameOverlayDebugBuildMs = value.frameOverlayDebugBuildMs;
        result.frameOverlayVariantFilterMs = value.frameOverlayVariantFilterMs;
        result.frameOverlayDebugSubmitMs = value.frameOverlayDebugSubmitMs;
        result.frameOverlayAuxFramesMs = value.frameOverlayAuxFramesMs;
        result.frameOverlayDetectorCount = value.frameOverlayDetectorCount;
        result.frameOverlayGeometryCount = value.frameOverlayGeometryCount;
        result.frameOverlayContactCount = value.frameOverlayContactCount;
        result.frameOverlayNearestCount = value.frameOverlayNearestCount;
        result.frameOverlayPrimitiveEstimate = value.frameOverlayPrimitiveEstimate;
        result.frameOverlayLineEstimate = value.frameOverlayLineEstimate;
        result.firstPairA = value.firstPairA;
        result.firstPairB = value.firstPairB;
        result.hasResult = value.hasResult;
        result.contacts.reserve(value.contacts.size());
        for(const ProjectScene::CollisionDetectorInfo::ContactInfo& contact : value.contacts) {
            result.contacts.push_back(toRuntimeContact(contact));
        }
        result.nearest = toRuntimeNearest(value.nearest);
        return result;
    }

    robot_qt_viewer::CollisionRuntimeStat toRuntimeStat(const RobotCollisionStat& value)
    {
        robot_qt_viewer::CollisionRuntimeStat result;
        result.name = value.name;
        result.count = value.count;
        return result;
    }

    std::vector<robot_qt_viewer::CollisionRuntimeStat> toRuntimeStats(
        const std::vector<RobotCollisionStat>& values)
    {
        std::vector<robot_qt_viewer::CollisionRuntimeStat> result;
        result.reserve(values.size());
        for(const RobotCollisionStat& value : values) {
            result.push_back(toRuntimeStat(value));
        }
        return result;
    }

    robot_qt_viewer::CollisionRuntimeModelVariantSummary toRuntimeVariantSummary(
        const RobotCollisionModelVariantSummary& value)
    {
        robot_qt_viewer::CollisionRuntimeModelVariantSummary result;
        result.variantId = value.variantId;
        result.label = value.label;
        result.linkName = value.linkName;
        result.role = value.role;
        result.source = value.source;
        result.enabled = value.enabled;
        result.replaceOriginal = value.replaceOriginal;
        result.visibleInViewport = value.visibleInViewport;
        result.selectedInViewport = value.selectedInViewport;
        result.usedByActiveDetector = value.usedByActiveDetector;
        result.elementCount = value.elementCount;
        result.meshElementCount = value.meshElementCount;
        result.meshVertexCount = value.meshVertexCount;
        result.meshTriangleCount = value.meshTriangleCount;
        result.unresolvedMeshCount = value.unresolvedMeshCount;
        result.primitiveCount = value.primitiveCount;
        result.elementNames = value.elementNames;
        result.geometryTypes = toRuntimeStats(value.geometryTypes);
        return result;
    }

    robot_qt_viewer::CollisionRuntimeLinkSummary toRuntimeLinkSummary(
        const RobotCollisionLinkSummary& value)
    {
        robot_qt_viewer::CollisionRuntimeLinkSummary result;
        result.linkName = value.linkName;
        result.hasVisual = value.hasVisual;
        result.hasOriginalCollision = value.hasOriginalCollision;
        result.hasOverrideCollision = value.hasOverrideCollision;
        result.hasMeshCollision = value.hasMeshCollision;
        result.hasSphereCover = value.hasSphereCover;
        result.visualCount = value.visualCount;
        result.originalCollisionCount = value.originalCollisionCount;
        result.overrideCollisionCount = value.overrideCollisionCount;
        result.effectiveCollisionCount = value.effectiveCollisionCount;
        result.geometryTypes = toRuntimeStats(value.geometryTypes);
        result.roles = toRuntimeStats(value.roles);
        result.sources = toRuntimeStats(value.sources);
        result.variants.reserve(value.variants.size());
        for(const RobotCollisionModelVariantSummary& variant : value.variants) {
            result.variants.push_back(toRuntimeVariantSummary(variant));
        }
        return result;
    }

    robot_qt_viewer::CollisionRuntimeRobotSummary toRuntimeRobotSummary(
        const RobotCollisionRobotSummary& value)
    {
        robot_qt_viewer::CollisionRuntimeRobotSummary result;
        result.robotId = value.robotId;
        result.robotName = value.robotName;
        result.hasVisualOnlyLinks = value.hasVisualOnlyLinks;
        result.hasCollisionReadyLinks = value.hasCollisionReadyLinks;
        result.hasOverrideAppliedLinks = value.hasOverrideAppliedLinks;
        result.hasMeshCollision = value.hasMeshCollision;
        result.hasSphereCover = value.hasSphereCover;
        result.linkCount = value.linkCount;
        result.visualLinkCount = value.visualLinkCount;
        result.visualOnlyLinkCount = value.visualOnlyLinkCount;
        result.collisionReadyLinkCount = value.collisionReadyLinkCount;
        result.overrideAppliedLinkCount = value.overrideAppliedLinkCount;
        result.visualCount = value.visualCount;
        result.originalCollisionCount = value.originalCollisionCount;
        result.overrideCollisionCount = value.overrideCollisionCount;
        result.effectiveCollisionCount = value.effectiveCollisionCount;
        result.geometryTypes = toRuntimeStats(value.geometryTypes);
        result.roles = toRuntimeStats(value.roles);
        result.sources = toRuntimeStats(value.sources);
        result.links.reserve(value.links.size());
        for(const RobotCollisionLinkSummary& link : value.links) {
            result.links.push_back(toRuntimeLinkSummary(link));
        }
        return result;
    }

    RobotCollisionProxyRequest toViewportProxyRequest(
        const robot_qt_viewer::CollisionRuntimeProxyRequest& value)
    {
        RobotCollisionProxyRequest result;
        result.proxyType = value.proxyType;
        result.role = value.role;
        result.inflationMargin = value.inflationMargin;
        result.maxSphereCount = value.maxSphereCount;
        result.useVisualWhenCollisionMissing = value.useVisualWhenCollisionMissing;
        result.useExistingCollisionAsInput = value.useExistingCollisionAsInput;
        return result;
    }

    robot_qt_viewer::CollisionRuntimeProxyQualitySummary toRuntimeProxyQualitySummary(
        const RobotCollisionProxyQualitySummary& value)
    {
        robot_qt_viewer::CollisionRuntimeProxyQualitySummary result;
        result.requestedMaxSpheres = value.requestedMaxSpheres;
        result.generatedSphereCount = value.generatedSphereCount;
        result.inputPointCount = value.inputPointCount;
        result.uncoveredPointCount = value.uncoveredPointCount;
        result.maxOutsideDistance = value.maxOutsideDistance;
        result.mainAxisLength = value.mainAxisLength;
        result.estimatedCrossSectionRadius = value.estimatedCrossSectionRadius;
        result.maxSphereRadius = value.maxSphereRadius;
        result.maxRadiusToLinkLength = value.maxRadiusToLinkLength;
        result.maxRadiusToCrossSectionRadius = value.maxRadiusToCrossSectionRadius;
        result.oversizedSphereCount = value.oversizedSphereCount;
        result.recommendedShape = value.recommendedShape;
        result.hasWarning = value.hasWarning;
        result.warning = value.warning;
        return result;
    }
}

namespace robot_qt_viewer
{
    RobotQtViewerViewportServicesAdapter::RobotQtViewerViewportServicesAdapter(RobotViewport& viewport)
        : m_viewport(viewport)
    {
    }

    void RobotQtViewerViewportServicesAdapter::selectRobotMount(
        const QString& robotId,
        const QString& linkName,
        const QString& robotMountId)
    {
        m_viewport.selectRobotMount(robotId, linkName, robotMountId);
    }

    bool RobotQtViewerViewportServicesAdapter::setActivePreviewRobotMount(const QString& robotMountId)
    {
        return m_viewport.setActivePreviewRobotMount(robotMountId);
    }

    bool RobotQtViewerViewportServicesAdapter::previewRobotMountTransform(
        const QString& robotMountId,
        const simulation_project::TransformDesc& transform)
    {
        return m_viewport.setPreviewRobotMountTransform(robotMountId, transform);
    }

    bool RobotQtViewerViewportServicesAdapter::previewRobotMountLink(
        const QString& robotMountId,
        const QString& linkName)
    {
        return m_viewport.setPreviewRobotMountLink(robotMountId, linkName);
    }

    bool RobotQtViewerViewportServicesAdapter::upsertPreviewRobotMount(
        const simulation_project::RobotMountDesc& mount)
    {
        return m_viewport.upsertPreviewRobotMount(mount);
    }

    bool RobotQtViewerViewportServicesAdapter::removePreviewRobotMount(const QString& robotMountId)
    {
        return m_viewport.removePreviewRobotMount(robotMountId);
    }

    void RobotQtViewerViewportServicesAdapter::previewRobotBaseTransform(
        const QString& robotId,
        const simulation_project::TransformDesc& transform)
    {
        m_viewport.previewRobotBaseTransform(robotId, transform);
    }

    void RobotQtViewerViewportServicesAdapter::previewSceneObjectTransform(
        const QString& objectId,
        const simulation_project::TransformDesc& transform)
    {
        m_viewport.previewSceneObjectTransform(objectId, transform);
    }

    bool RobotQtViewerViewportServicesAdapter::commitSceneObjectTransform(
        const QString& objectId,
        const simulation_project::TransformDesc& transform)
    {
        return m_viewport.setSceneObjectTransform(objectId, transform);
    }

    bool RobotQtViewerViewportServicesAdapter::removeSceneObject(const QString& objectId)
    {
        return m_viewport.removeSceneObject(objectId);
    }

    void RobotQtViewerViewportServicesAdapter::selectObjectFrame(
        const QString& objectId,
        const QString& frameId)
    {
        m_viewport.selectObjectFrame(objectId, frameId);
    }

    bool RobotQtViewerViewportServicesAdapter::previewObjectFrameTransform(
        const QString& objectId,
        const QString& frameId,
        const simulation_project::TransformDesc& transform)
    {
        return m_viewport.previewObjectFrameTransform(objectId, frameId, transform);
    }

    bool RobotQtViewerViewportServicesAdapter::upsertPreviewObjectFrame(
        const QString& objectId,
        const simulation_project::ObjectFrameDesc& frame)
    {
        return m_viewport.upsertPreviewObjectFrame(objectId, frame);
    }

    void RobotQtViewerViewportServicesAdapter::selectRobotLink(
        const QString& robotId,
        const QString& linkName)
    {
        m_viewport.selectRobotLink(robotId, linkName);
    }

    void RobotQtViewerViewportServicesAdapter::selectRobotJointFrame(
        const QString& robotId,
        const QString& jointName)
    {
        m_viewport.selectRobotJointFrame(robotId, jointName);
    }

    void RobotQtViewerViewportServicesAdapter::setActiveToolFrameRobot(const QString& robotId)
    {
        m_viewport.setActiveToolFrameRobot(robotId);
    }

    void RobotQtViewerViewportServicesAdapter::selectSceneObject(const QString& objectId)
    {
        m_viewport.selectSceneObject(objectId);
    }

    void RobotQtViewerViewportServicesAdapter::selectMountedAttachment(const QString& attachmentId)
    {
        m_viewport.selectMountedAttachment(attachmentId);
    }

    bool RobotQtViewerViewportServicesAdapter::setActiveMountedAttachment(const QString& attachmentId)
    {
        return m_viewport.setActiveMountedAttachment(attachmentId);
    }

    void RobotQtViewerViewportServicesAdapter::setToolFrameVisibility(
        const RobotQtViewerToolFrameVisibility& visibility)
    {
        ProjectScene::ToolFrameVisibility viewportVisibility;
        viewportVisibility.link = visibility.link;
        viewportVisibility.robotMount = visibility.robotMount;
        viewportVisibility.toolMount = visibility.toolMount;
        viewportVisibility.visual = visibility.visual;
        viewportVisibility.tcp = visibility.tcp;
        viewportVisibility.sensorPreview = visibility.sensorPreview;
        m_viewport.setToolFrameVisibility(viewportVisibility);
    }

    void RobotQtViewerViewportServicesAdapter::setRobotMountFrameVisibility(
        bool selectedLinkFrameVisible,
        bool mountFrameVisible)
    {
        m_viewport.setRobotMountFrameVisibility(selectedLinkFrameVisible, mountFrameVisible);
    }

    void RobotQtViewerViewportServicesAdapter::setPinnedRobotMountFrames(const QStringList& robotMountIds)
    {
        m_viewport.setPinnedRobotMountFrames(robotMountIds);
    }

    void RobotQtViewerViewportServicesAdapter::focusMountFrameLink(
        const QString& robotId,
        const QString& linkName)
    {
        m_viewport.focusMountFrameLink(robotId, linkName);
    }

    void RobotQtViewerViewportServicesAdapter::clearMountFrameLinkFocus()
    {
        m_viewport.clearMountFrameLinkFocus();
    }

    void RobotQtViewerViewportServicesAdapter::focusObjectFrameObject(const QString& objectId)
    {
        m_viewport.focusObjectFrameObject(objectId);
    }

    void RobotQtViewerViewportServicesAdapter::clearObjectFrameObjectFocus()
    {
        m_viewport.clearObjectFrameObjectFocus();
    }

    void RobotQtViewerViewportServicesAdapter::focusMountedAttachment(const QString& attachmentId)
    {
        m_viewport.focusMountedAttachment(attachmentId);
    }

    void RobotQtViewerViewportServicesAdapter::clearMountedAttachmentFocus()
    {
        m_viewport.clearMountedAttachmentFocus();
    }

    void RobotQtViewerViewportServicesAdapter::previewObjectCollisionModelVariant(
        const QString& objectId,
        const QString& variantId)
    {
        m_viewport.previewObjectCollisionModelVariant(objectId, variantId);
    }

    void RobotQtViewerViewportServicesAdapter::clearObjectCollisionModelVariantPreview()
    {
        m_viewport.clearObjectCollisionModelVariantPreview();
    }

    void RobotQtViewerViewportServicesAdapter::previewCollisionPairTargets(
        const QString& robotAId,
        const QString& linkAName,
        const QString& objectAId,
        const QString& attachmentAId,
        const QString& robotBId,
        const QString& linkBName,
        const QString& objectBId,
        const QString& attachmentBId)
    {
        m_viewport.previewCollisionPairTargets(
            robotAId,
            linkAName,
            objectAId,
            attachmentAId,
            robotBId,
            linkBName,
            objectBId,
            attachmentBId);
    }

    RobotQtViewerViewportLoadResult RobotQtViewerViewportServicesAdapter::loadProjectDocument(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& basePath)
    {
        RobotQtViewerViewportLoadResult result;
        result.success = m_viewport.loadProjectDocument(document, basePath);
        if(!result.success) {
            result.errorMessage = m_viewport.lastError();
        }
        return result;
    }

    bool RobotQtViewerViewportServicesAdapter::refreshCollisionConfiguration(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& basePath)
    {
        return m_viewport.refreshCollisionConfiguration(document, basePath);
    }

    void RobotQtViewerViewportServicesAdapter::setCollisionGeometryVisible(bool visible)
    {
        m_viewport.setCollisionGeometryVisible(visible);
    }

    bool RobotQtViewerViewportServicesAdapter::collisionQueriesEnabled() const
    {
        return m_viewport.collisionQueriesEnabled();
    }

    bool RobotQtViewerViewportServicesAdapter::setCollisionQueriesEnabled(bool enabled)
    {
        return m_viewport.setCollisionQueriesEnabled(enabled);
    }

    bool RobotQtViewerViewportServicesAdapter::setActiveCollisionDetector(const QString& detectorId)
    {
        return m_viewport.setActiveCollisionDetector(detectorId);
    }

    bool RobotQtViewerViewportServicesAdapter::setCollisionDetectorEnabled(const QString& detectorId, bool enabled)
    {
        return m_viewport.setCollisionDetectorEnabled(detectorId, enabled);
    }

    bool RobotQtViewerViewportServicesAdapter::setCollisionDetectorVisible(const QString& detectorId, bool visible)
    {
        return m_viewport.setCollisionDetectorVisible(detectorId, visible);
    }

    bool RobotQtViewerViewportServicesAdapter::updateCollisionDetectorRuntimeOptions(
        const simulation_project::CollisionDetectorDesc& detector)
    {
        return m_viewport.updateCollisionDetectorRuntimeOptions(detector);
    }

    bool RobotQtViewerViewportServicesAdapter::rebuildCollisionDetectorsFromDocument(
        const simulation_project::ProjectDocument& document)
    {
        return m_viewport.rebuildCollisionDetectorsFromDocument(document);
    }

    bool RobotQtViewerViewportServicesAdapter::removeCollisionDetector(const QString& detectorId)
    {
        return m_viewport.removeCollisionDetector(detectorId);
    }

    bool RobotQtViewerViewportServicesAdapter::setVisibleRobotCollisionVariant(
        const QString& robotId,
        const QString& linkName,
        const QString& variantId)
    {
        return m_viewport.setVisibleRobotCollisionVariant(robotId, linkName, variantId);
    }

    QString RobotQtViewerViewportServicesAdapter::visibleRobotCollisionVariant(
        const QString& robotId,
        const QString& linkName) const
    {
        return m_viewport.visibleRobotCollisionVariant(robotId, linkName);
    }

    CollisionRuntimeRobotSummary RobotQtViewerViewportServicesAdapter::robotCollisionSummary(
        const QString& robotId,
        const QString& activeDetectorRole,
        const QString& activeDetectorSource) const
    {
        return toRuntimeRobotSummary(
            m_viewport.robotCollisionSummary(robotId, activeDetectorRole, activeDetectorSource));
    }

    std::vector<CollisionRuntimeDetectorInfo> RobotQtViewerViewportServicesAdapter::collisionRuntimeDetectors() const
    {
        const std::vector<ProjectScene::CollisionDetectorInfo> viewportDetectors = m_viewport.collisionDetectors();
        std::vector<CollisionRuntimeDetectorInfo> detectors;
        detectors.reserve(viewportDetectors.size());
        for(const ProjectScene::CollisionDetectorInfo& detector : viewportDetectors) {
            detectors.push_back(toRuntimeDetector(detector));
        }
        return detectors;
    }

    bool RobotQtViewerViewportServicesAdapter::generateRobotCollisionProxies(
        const QString& robotId,
        const QString& linkName,
        const CollisionRuntimeProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
    {
        return m_viewport.generateRobotCollisionProxies(
            robotId,
            linkName,
            toViewportProxyRequest(request),
            elements);
    }

    bool RobotQtViewerViewportServicesAdapter::generateRobotCollisionProxiesFromExistingCollision(
        const QString& robotId,
        const QString& linkName,
        const CollisionRuntimeProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
    {
        return m_viewport.generateRobotCollisionProxiesFromExistingCollision(
            robotId,
            linkName,
            toViewportProxyRequest(request),
            elements);
    }

    bool RobotQtViewerViewportServicesAdapter::generateRobotCollisionProxiesFromExistingCollision(
        const QString& robotId,
        const CollisionRuntimeProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
    {
        return m_viewport.generateRobotCollisionProxiesFromExistingCollision(
            robotId,
            toViewportProxyRequest(request),
            elements);
    }

    bool RobotQtViewerViewportServicesAdapter::generateRobotCollisionCoacdFromVisual(
        const QString& robotId,
        const QString& linkName,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
    {
        return m_viewport.generateRobotCollisionCoacdFromVisual(robotId, linkName, elements);
    }

    bool RobotQtViewerViewportServicesAdapter::generateRobotCollisionCoacdFromExistingCollision(
        const QString& robotId,
        const QString& linkName,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
    {
        return m_viewport.generateRobotCollisionCoacdFromExistingCollision(robotId, linkName, elements);
    }

    bool RobotQtViewerViewportServicesAdapter::generateObjectCollisionCoacdFromVisual(
        const QString& objectId,
        std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements) const
    {
        return m_viewport.generateObjectCollisionCoacdFromVisual(objectId, elements);
    }

    bool RobotQtViewerViewportServicesAdapter::generateMissingRobotCollisionProxies(
        const QString& robotId,
        const CollisionRuntimeProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
    {
        return m_viewport.generateMissingRobotCollisionProxies(
            robotId,
            toViewportProxyRequest(request),
            elements);
    }

    bool RobotQtViewerViewportServicesAdapter::evaluateRobotCollisionProxyQuality(
        const QString& robotId,
        const QString& linkName,
        const CollisionRuntimeProxyRequest& request,
        const std::vector<simulation_project::CollisionElementOverrideDesc>& elements,
        bool useExistingCollisionInput,
        CollisionRuntimeProxyQualitySummary& summary) const
    {
        RobotCollisionProxyQualitySummary viewportSummary;
        if(!m_viewport.evaluateRobotCollisionProxyQuality(
               robotId,
               linkName,
               toViewportProxyRequest(request),
               elements,
               useExistingCollisionInput,
               viewportSummary)) {
            return false;
        }
        summary = toRuntimeProxyQualitySummary(viewportSummary);
        return true;
    }

    double RobotQtViewerViewportServicesAdapter::robotJointValue(
        const QString& robotId,
        const QString& jointName,
        bool* ok) const
    {
        return m_viewport.robotJointValue(robotId, jointName, ok);
    }

    void RobotQtViewerViewportServicesAdapter::setRobotJointValue(
        const QString& robotId,
        const QString& jointName,
        double value)
    {
        m_viewport.setRobotJointValue(robotId, jointName, value);
    }

    void RobotQtViewerViewportServicesAdapter::setRobotAutoMotion(
        const QString& robotId,
        bool enabled,
        double amplitude,
        double speed)
    {
        m_viewport.setRobotAutoMotion(robotId, enabled, amplitude, speed);
    }

    bool RobotQtViewerViewportServicesAdapter::applySurfaceScalarOverlay(
        const smrobot::visualization::SurfaceScalarOverlay& overlay,
        QString* errorMessage)
    {
        return m_viewport.applySurfaceScalarOverlay(overlay, errorMessage);
    }

    bool RobotQtViewerViewportServicesAdapter::setSurfaceScalarOverlayVisible(
        const QString& objectId,
        bool visible)
    {
        return m_viewport.setSurfaceScalarOverlayVisible(objectId, visible);
    }

    bool RobotQtViewerViewportServicesAdapter::clearSurfaceScalarOverlay(const QString& objectId)
    {
        return m_viewport.clearSurfaceScalarOverlay(objectId);
    }

    void RobotQtViewerViewportServicesAdapter::setSurfaceScalarProbeEnabled(
        bool enabled,
        const QString& objectId)
    {
        m_viewport.setSurfaceScalarProbeEnabled(enabled, objectId);
    }

    bool RobotQtViewerViewportServicesAdapter::jointValue(
        const std::string& robotId,
        const std::string& jointName,
        double& value) const
    {
        bool ok = false;
        value = m_viewport.robotJointValue(
            QString::fromStdString(robotId),
            QString::fromStdString(jointName),
            &ok);
        return ok;
    }

    robotruntime::RobotRunCommandResult RobotQtViewerViewportServicesAdapter::setJointValue(
        const std::string& robotId,
        const std::string& jointName,
        double value)
    {
        m_viewport.setRobotJointValue(
            QString::fromStdString(robotId),
            QString::fromStdString(jointName),
            value);
        return { true, "Robot joint value updated." };
    }

    robotruntime::RobotRunCommandResult RobotQtViewerViewportServicesAdapter::setAutoMotion(
        const std::string& robotId,
        bool enabled,
        double amplitude,
        double speed)
    {
        m_viewport.setRobotAutoMotion(
            QString::fromStdString(robotId),
            enabled,
            amplitude,
            speed);
        return { true, enabled ? "Robot auto motion enabled." : "Robot auto motion disabled." };
    }

    robotruntime::RobotRunCommandResult RobotQtViewerViewportServicesAdapter::loadTrajectory(
        const std::string& robotId,
        const std::string& trajectoryId,
        const std::vector<std::string>& jointNames,
        const robottrajectory::JointTrajectory& trajectory)
    {
        if(trajectory.empty() || jointNames.size() != trajectory.points.front().q.size()) {
            return { false, "Trajectory joint names must match its joint values." };
        }
        const robotruntime::RobotRunCommandResult result =
            m_trajectorySession.load(robotId, trajectoryId, trajectory);
        if(result.success) {
            m_trajectoryJointNames = jointNames;
        }
        return result;
    }

    robotruntime::RobotRunCommandResult RobotQtViewerViewportServicesAdapter::startTrajectory()
    {
        const robotruntime::RobotRunCommandResult result = m_trajectorySession.start();
        if(result.success) {
            const robotruntime::RobotRunExecutionSnapshot snapshot = m_trajectorySession.snapshot();
            for(std::size_t index = 0;
                index < m_trajectoryJointNames.size() && index < snapshot.jointValues.size();
                ++index) {
                m_viewport.setRobotJointValue(
                    QString::fromStdString(snapshot.robotId),
                    QString::fromStdString(m_trajectoryJointNames[index]),
                    snapshot.jointValues[index]);
            }
        }
        return result;
    }

    robotruntime::RobotRunCommandResult RobotQtViewerViewportServicesAdapter::pauseTrajectory()
    {
        return m_trajectorySession.pause();
    }

    robotruntime::RobotRunCommandResult RobotQtViewerViewportServicesAdapter::stopTrajectory()
    {
        return m_trajectorySession.stop();
    }

    robotruntime::RobotRunCommandResult RobotQtViewerViewportServicesAdapter::stepTrajectory(
        double timeStep)
    {
        const robotruntime::RobotRunCommandResult result = m_trajectorySession.step(timeStep);
        if(!result.success) {
            return result;
        }
        const robotruntime::RobotRunExecutionSnapshot snapshot = m_trajectorySession.snapshot();
        for(std::size_t index = 0;
            index < m_trajectoryJointNames.size() && index < snapshot.jointValues.size();
            ++index) {
            m_viewport.setRobotJointValue(
                QString::fromStdString(snapshot.robotId),
                QString::fromStdString(m_trajectoryJointNames[index]),
                snapshot.jointValues[index]);
        }
        return result;
    }

    robotruntime::RobotRunExecutionSnapshot
    RobotQtViewerViewportServicesAdapter::trajectorySnapshot() const
    {
        return m_trajectorySession.snapshot();
    }
}
