#pragma once

#include <memory>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

#include <SimulationProject/ProjectDocument.h>
#include <VisualizationSDK/SurfaceScalarOverlay.h>

namespace simulation_project
{
    struct ProjectDocument;
}

struct RobotCollisionProxyRequest;
struct RobotCollisionProxyQualitySummary;
struct RobotCollisionRobotSummary;
struct ProjectScenePickResult;
struct ProjectSceneTrianglePickResult;

enum class ProjectSceneInteractionMode
{
    Browse,
    SelectRobot,
    SelectLink,
    SelectMount,
    SelectAttachment,
    EditTransformPreview,
    EditCollisionProxy,
    SelectCollisionTarget,
    SelectRotationSurface
};

enum class ProjectSceneCameraView
{
    Home,
    Isometric,
    Front,
    Back,
    Left,
    Right,
    Top,
    Bottom
};

enum class ProjectSceneProjectionMode
{
    Perspective,
    Orthographic
};

class ProjectScene
{
public:
    struct CoatingTrajectoryPreviewPoint
    {
        double positionX = 0.0;
        double positionY = 0.0;
        double positionZ = 0.0;
        double directionX = 1.0;
        double directionY = 0.0;
        double directionZ = 0.0;
        bool sprayEnabled = false;
        bool startsNewSegment = false;
    };

    struct CoatingPredictionDebugTriangle
    {
        double ax = 0.0;
        double ay = 0.0;
        double az = 0.0;
        double bx = 0.0;
        double by = 0.0;
        double bz = 0.0;
        double cx = 0.0;
        double cy = 0.0;
        double cz = 0.0;
    };

    struct CoatingPredictionDebugPoint
    {
        double positionX = 0.0;
        double positionY = 0.0;
        double positionZ = 0.0;
        double directionX = 1.0;
        double directionY = 0.0;
        double directionZ = 0.0;
    };

    struct CoatingPredictionDebugState
    {
        bool visible = false;
        std::string objectId;
        std::vector<CoatingPredictionDebugTriangle> seedTriangles;
        std::vector<CoatingPredictionDebugTriangle> cylindricalTriangles;
        std::vector<CoatingPredictionDebugTriangle> localSectorTriangles;
        std::vector<std::uint32_t> localSectorVertexIndices;
        std::vector<CoatingPredictionDebugPoint> profileLinePoints;
        std::vector<CoatingPredictionDebugPoint> selectedProfileLinePoints;
        std::vector<CoatingPredictionDebugPoint> sprayPoints;
        double axisOriginX = 0.0;
        double axisOriginY = 0.0;
        double axisOriginZ = 0.0;
        double axisDirectionX = 0.0;
        double axisDirectionY = 0.0;
        double axisDirectionZ = 1.0;
        double axisLength = 1.0;
        double markerRadius = 0.002;
    };

    struct CoatingPredictionDebugVisibility
    {
        bool cylindricalSurface = true;
        bool rotationAxis = true;
        bool localSector = true;
        bool sprayPoints = true;
    };

    struct RobotJointInfo
    {
        std::string jointName;
        std::string jointType;
    };

    struct RobotLinkGroup
    {
        std::string robotId;
        std::string robotName;
        std::vector<std::string> links;
        std::vector<std::string> joints;
        std::vector<RobotJointInfo> movableJoints;
    };

    struct SceneObjectGroup
    {
        std::string objectId;
        std::string objectName;
    };

    struct PointCloudGroup
    {
        std::string pointCloudId;
        std::string pointCloudName;
    };

    struct RobotLinkMaterialInfo
    {
        std::string robotId;
        std::string linkName;
        std::string partUid;
        std::string meshPath;
        std::string source;
        std::string overrideState;
        float r = 0.0f;
        float g = 0.0f;
        float b = 0.0f;
        float a = 1.0f;
        std::size_t subMeshCount = 0;
    };

    struct CollisionDetectorInfo
    {
        struct Vec3Info
        {
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
        };

        struct ContactInfo
        {
            std::string bodyA;
            std::string bodyB;
            Vec3Info position;
            Vec3Info normal;
            double penetrationDepth = 0.0;
        };

        struct NearestInfo
        {
            bool valid = false;
            std::string state = "NotComputed";
            std::string reason;
            std::string bodyA;
            std::string bodyB;
            Vec3Info pointA;
            Vec3Info pointB;
            double distance = 0.0;
        };

        std::string id;
        std::string name;
        std::string type;
        bool enabled = true;
        bool visible = true;
        bool active = false;
        std::size_t includePairCount = 0;
        std::size_t effectiveIncludePairCount = 0;
        std::size_t contactCount = 0;
        bool inCollision = false;
        double minDistance = 0.0;
        double lastCheckMs = 0.0;
        double lastDistanceMs = 0.0;
        double lastQueryMs = 0.0;
        double frameRobotPoseMs = 0.0;
        double frameCollisionWorldUpdateMs = 0.0;
        double frameOverlayMs = 0.0;
        double frameOverlayHighlightMs = 0.0;
        double frameOverlayDebugBuildMs = 0.0;
        double frameOverlayVariantFilterMs = 0.0;
        double frameOverlayDebugSubmitMs = 0.0;
        double frameOverlayAuxFramesMs = 0.0;
        std::size_t frameOverlayDetectorCount = 0;
        std::size_t frameOverlayGeometryCount = 0;
        std::size_t frameOverlayContactCount = 0;
        std::size_t frameOverlayNearestCount = 0;
        std::size_t frameOverlayPrimitiveEstimate = 0;
        std::size_t frameOverlayLineEstimate = 0;
        std::string firstPairA;
        std::string firstPairB;
        bool hasResult = false;
        std::vector<ContactInfo> contacts;
        NearestInfo nearest;
    };

    struct ToolAttachmentInfo
    {
        std::string id;
        std::string name;
        std::string attachmentKind;
        std::string assetKind;
        std::string assetType;
        std::string functionalFrameType;
        std::string robotMountId;
        std::string toolAssetId;
        std::string mountFrameId;
        std::string assetId;
        std::string linkName;
        std::string visualPath;
        bool enabled = true;
        bool visible = true;
        bool active = false;
    };
    using MountedAttachmentInfo = ToolAttachmentInfo;

    struct ToolFrameVisibility
    {
        bool link = false;
        bool robotMount = false;
        bool toolMount = true;
        bool visual = true;
        bool tcp = true;
        bool sensorPreview = true;
    };

    ProjectScene();
    ~ProjectScene();

    ProjectScene(const ProjectScene&) = delete;
    ProjectScene& operator=(const ProjectScene&) = delete;

    bool initialize();
    void setProjectDocument(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& basePath);
    void setDefaultBackgroundColor(const simulation_project::ColorDesc& color);
    bool refreshCollisionConfiguration(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& basePath);
    bool setToolAssetPreview(
        const simulation_project::AttachmentAssetDesc& asset,
        const std::filesystem::path& basePath);
    bool isInitialized() const;

    void resize(int width, int height);
    void update(double timeSeconds);
    void render();

    void onMouseMove(float dx, float dy, int button);
    void onScroll(float delta, int x, int y);
    void setCameraView(ProjectSceneCameraView view);
    void focusFullScene(double duration = 0.3);
    void setProjectionMode(ProjectSceneProjectionMode mode);
    ProjectSceneProjectionMode projectionMode() const;
    bool setRotationCenterFromScreenPoint(int x, int y);
    std::size_t setRotationCenterFromScreenRect(int left, int top, int right, int bottom);
    void setRotationCenterMarkerVisible(bool visible);
    void focusMountFrameLink(const std::string& robotId, const std::string& linkName);
    void clearMountFrameLinkFocus();
    void focusObjectFrameObject(const std::string& objectId);
    void clearObjectFrameObjectFocus();
    void focusCoatingObject(const std::string& objectId, double duration = 0.3);
    void focusMountedAttachment(const std::string& attachmentId);
    void clearMountedAttachmentFocus();
    void previewObjectCollisionModelVariant(
        const std::string& objectId,
        const std::string& variantId);
    void clearObjectCollisionModelVariantPreview();
    void previewCollisionPairTargets(
        const std::string& robotAId,
        const std::string& linkAName,
        const std::string& objectAId,
        const std::string& attachmentAId,
        const std::string& robotBId,
        const std::string& linkBName,
        const std::string& objectBId,
        const std::string& attachmentBId);
    void setInteractionMode(ProjectSceneInteractionMode mode);
    ProjectSceneInteractionMode interactionMode() const;
    ProjectScenePickResult pickScreenPoint(int x, int y, bool preciseMesh = true) const;
    ProjectSceneTrianglePickResult pickTriangleScreenPoint(int x, int y) const;
    void requestSceneObjectHover(int x, int y);
    void clearSceneObjectHover();
    std::string hoveredSceneObjectId() const;
    bool applySurfaceScalarOverlay(
        const smrobot::visualization::SurfaceScalarOverlay& overlay,
        std::string* errorMessage = nullptr);
    bool setSurfaceScalarOverlayVisible(const std::string& objectId, bool visible);
    bool clearSurfaceScalarOverlay(const std::string& objectId);
    void setCoatingTrajectoryPreview(
        const std::vector<CoatingTrajectoryPreviewPoint>& points,
        bool visible);
    bool setCoatingTrajectoryPreviewVisible(bool visible);
    void setCoatingPredictionDebugState(const CoatingPredictionDebugState& state);
    void setCoatingPredictionDebugVisibility(
        const CoatingPredictionDebugVisibility& visibility);
    void clearCoatingPredictionDebugState();
    bool setCoatingModelVisible(const std::string& objectId, bool visible);
    void setCoatingModelVisibilities(const std::unordered_map<std::string, bool>& visibility);
    void clearCoatingModelVisibility(const std::string& objectId);
    void setCoatingAnalysisView(bool active);
    smrobot::visualization::SurfaceScalarProbeResult probeSurfaceScalarAtScreenPoint(
        const std::string& objectId,
        int x,
        int y) const;
    void setShowCollisionGeometry(bool visible);
    void setSelectedLink(const std::string& robotId, const std::string& linkName);
    void setSelectedJointFrame(const std::string& robotId, const std::string& jointName);
    bool setVisibleRobotCollisionVariant(
        const std::string& robotId,
        const std::string& linkName,
        const std::string& variantId);
    std::string visibleRobotCollisionVariant(
        const std::string& robotId,
        const std::string& linkName) const;
    void setSelectedRobotMount(
        const std::string& robotId,
        const std::string& linkName,
        const std::string& robotMountId);
    void setSelectedSceneObject(const std::string& objectId);
    void setSelectedObjectFrame(
        const std::string& objectId,
        const std::string& frameId);
    bool setSelectedMountedAttachment(const std::string& attachmentId);
    bool setSelectedToolAttachment(const std::string& attachmentId);
    bool setActivePreviewRobotMount(const std::string& robotMountId);
    bool setPreviewRobotMountTransform(
        const std::string& robotMountId,
        const simulation_project::TransformDesc& transform);
    bool setPreviewRobotMountLink(
        const std::string& robotMountId,
        const std::string& linkName);
    bool upsertPreviewRobotMount(const simulation_project::RobotMountDesc& mount);
    bool removePreviewRobotMount(const std::string& robotMountId);
    void setRobotMountFrameVisibility(bool selectedLinkFrameVisible, bool mountFrameVisible);
    void setPinnedRobotMountFrames(const std::vector<std::string>& robotMountIds);
    bool setRobotBaseTransform(
        const std::string& robotId,
        const simulation_project::TransformDesc& transform);
    bool setRobotJointValue(
        const std::string& robotId,
        const std::string& jointName,
        double value);
    bool robotJointValue(
        const std::string& robotId,
        const std::string& jointName,
        double& value) const;
    bool setRobotAutoMotion(
        const std::string& robotId,
        bool enabled,
        double amplitude,
        double speed);
    bool setSceneObjectTransform(
        const std::string& objectId,
        const simulation_project::TransformDesc& transform);
    bool removeSceneObject(const std::string& objectId);
    bool previewSceneObjectTransform(
        const std::string& objectId,
        const simulation_project::TransformDesc& transform);
    bool setPreviewObjectFrameTransform(
        const std::string& objectId,
        const std::string& frameId,
        const simulation_project::TransformDesc& transform);
    bool upsertPreviewObjectFrame(
        const std::string& objectId,
        const simulation_project::ObjectFrameDesc& frame);
    bool setRobotMountTransform(
        const std::string& robotMountId,
        const simulation_project::TransformDesc& transform);
    bool setMountedAttachmentTransform(
        const std::string& attachmentId,
        const simulation_project::TransformDesc& transform);
    bool setToolAttachmentTransform(
        const std::string& attachmentId,
        const simulation_project::TransformDesc& transform);
    bool setToolAssetMountToVisual(
        const std::string& assetId,
        const simulation_project::TransformDesc& transform);
    bool setToolAssetTcp(
        const std::string& assetId,
        const simulation_project::TransformDesc& transform);
    bool setToolAssetPreviewMountToVisual(
        const simulation_project::TransformDesc& transform);
    bool setToolAssetPreviewTcp(
        const simulation_project::TransformDesc& transform);
    bool cycleToolAttachment(bool reverse);
    bool cycleMountedAttachment(bool reverse);
    bool setActiveMountedAttachment(const std::string& id);
    bool setActiveToolAttachment(const std::string& id);
    void setActiveToolFrameRobot(const std::string& robotId);
    void setToolFrameVisibility(const ToolFrameVisibility& visibility);
    std::string activeMountedAttachmentId() const;
    std::string activeToolAttachmentId() const;
    std::vector<MountedAttachmentInfo> mountedAttachments() const;
    std::vector<ToolAttachmentInfo> toolAttachments() const;
    std::vector<RobotLinkMaterialInfo> robotLinkMaterials(
        const std::string& robotId,
        const std::string& linkName) const;
    const std::vector<RobotLinkGroup>& robotLinks() const;
    const std::vector<SceneObjectGroup>& sceneObjects() const;
    const std::vector<PointCloudGroup>& pointClouds() const;
    std::vector<CollisionDetectorInfo> collisionDetectors() const;
    bool collisionQueriesEnabled() const;
    bool setCollisionQueriesEnabled(bool enabled);
    bool setActiveCollisionDetector(const std::string& id);
    bool setCollisionDetectorEnabled(const std::string& id, bool enabled);
    bool setCollisionDetectorVisible(const std::string& id, bool visible);
    bool updateCollisionDetectorRuntimeOptions(const simulation_project::CollisionDetectorDesc& desc);
    bool rebuildCollisionDetectorsFromDocument(const simulation_project::ProjectDocument& document);
    bool removeCollisionDetector(const std::string& id);
    bool generateRobotCollisionProxy(
        const std::string& robotId,
        const std::string& linkName,
        const std::string& proxyType,
        simulation_project::CollisionElementOverrideDesc& element) const;
    bool generateRobotCollisionProxies(
        const std::string& robotId,
        const std::string& linkName,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateRobotCollisionProxiesFromExistingCollision(
        const std::string& robotId,
        const std::string& linkName,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateRobotCollisionProxiesFromExistingCollision(
        const std::string& robotId,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateRobotCollisionCoacdFromVisual(
        const std::string& robotId,
        const std::string& linkName,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateRobotCollisionCoacdFromExistingCollision(
        const std::string& robotId,
        const std::string& linkName,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateObjectCollisionCoacdFromVisual(
        const std::string& objectId,
        std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements) const;
    bool evaluateRobotCollisionProxyQuality(
        const std::string& robotId,
        const std::string& linkName,
        const RobotCollisionProxyRequest& request,
        const std::vector<simulation_project::CollisionElementOverrideDesc>& elements,
        bool useExistingCollisionInput,
        RobotCollisionProxyQualitySummary& summary) const;
    bool generateMissingRobotCollisionProxies(
        const std::string& robotId,
        const std::string& proxyType,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateMissingRobotCollisionProxies(
        const std::string& robotId,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    RobotCollisionRobotSummary robotCollisionSummary(
        const std::string& robotId,
        const std::string& activeDetectorRole = std::string(),
        const std::string& activeDetectorSource = std::string()) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};
