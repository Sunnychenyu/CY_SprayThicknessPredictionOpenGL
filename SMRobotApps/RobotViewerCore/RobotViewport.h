#pragma once

#include "ProjectScene.h"
#include "RobotCollisionModelInspector.h"
#include "RobotCollisionProxyGenerator.h"

#include <SimulationProject/ProjectDocument.h>

#include <QOpenGLWidget>
#include <QPoint>
#include <QRect>
#include <QString>
#include <QStringList>

#include <cstddef>
#include <cstdint>
#include <chrono>
#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

class QMouseEvent;
class QEvent;
class QKeyEvent;
class QTimer;
class QWheelEvent;
class QRubberBand;
namespace simulation_project
{
    struct ProjectDocument;
}

class RobotViewport : public QOpenGLWidget
{
    Q_OBJECT

public:
    explicit RobotViewport(QWidget* parent = nullptr);
    ~RobotViewport() override;

    void setJointPreview(int degrees);
    void setRobotSummary(const char* name, std::size_t linkCount, std::size_t jointCount);
    bool loadProjectDocument(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& basePath);
    void setDefaultBackgroundColor(const simulation_project::ColorDesc& color);
    bool refreshCollisionConfiguration(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& basePath);
    QString lastError() const;
    bool loadToolAssetPreview(
        const simulation_project::AttachmentAssetDesc& asset,
        const std::filesystem::path& basePath);
    bool setActivePreviewRobotMount(const QString& robotMountId);
    bool setPreviewRobotMountTransform(
        const QString& robotMountId,
        const simulation_project::TransformDesc& transform);
    bool setPreviewRobotMountLink(
        const QString& robotMountId,
        const QString& linkName);
    bool upsertPreviewRobotMount(const simulation_project::RobotMountDesc& mount);
    bool removePreviewRobotMount(const QString& robotMountId);
    void setRobotMountFrameVisibility(bool selectedLinkFrameVisible, bool mountFrameVisible);
    void setPinnedRobotMountFrames(const QStringList& robotMountIds);
    void focusMountFrameLink(const QString& robotId, const QString& linkName);
    void clearMountFrameLinkFocus();
    void focusObjectFrameObject(const QString& objectId);
    void clearObjectFrameObjectFocus();
    void focusCoatingObject(const QString& objectId, double duration = 0.3);
    void focusMountedAttachment(const QString& attachmentId);
    void clearMountedAttachmentFocus();
    void previewObjectCollisionModelVariant(const QString& objectId, const QString& variantId);
    void clearObjectCollisionModelVariantPreview();
    void previewCollisionPairTargets(
        const QString& robotAId,
        const QString& linkAName,
        const QString& objectAId,
        const QString& attachmentAId,
        const QString& robotBId,
        const QString& linkBName,
        const QString& objectBId,
        const QString& attachmentBId);
    void selectRobotLink(const QString& robotId, const QString& linkName);
    void selectRobotJointFrame(const QString& robotId, const QString& jointName);
    void selectRobotMount(const QString& robotId, const QString& linkName, const QString& robotMountId);
    void selectSceneObject(const QString& objectId);
    void selectMountedAttachment(const QString& attachmentId);
    void selectToolAttachment(const QString& attachmentId);
    void previewRobotBaseTransform(
        const QString& robotId,
        const simulation_project::TransformDesc& transform);
    void setRobotJointValue(
        const QString& robotId,
        const QString& jointName,
        double value);
    double robotJointValue(
        const QString& robotId,
        const QString& jointName,
        bool* ok = nullptr) const;
    void setRobotAutoMotion(
        const QString& robotId,
        bool enabled,
        double amplitude,
        double speed);
    void previewSceneObjectTransform(
        const QString& objectId,
        const simulation_project::TransformDesc& transform);
    bool setSceneObjectTransform(
        const QString& objectId,
        const simulation_project::TransformDesc& transform);
    bool removeSceneObject(const QString& objectId);
    void selectObjectFrame(const QString& objectId, const QString& frameId);
    bool previewObjectFrameTransform(
        const QString& objectId,
        const QString& frameId,
        const simulation_project::TransformDesc& transform);
    bool upsertPreviewObjectFrame(
        const QString& objectId,
        const simulation_project::ObjectFrameDesc& frame);
    bool setRobotMountTransform(
        const QString& robotMountId,
        const simulation_project::TransformDesc& transform);
    bool setMountedAttachmentTransform(
        const QString& attachmentId,
        const simulation_project::TransformDesc& transform);
    bool setToolAttachmentTransform(
        const QString& attachmentId,
        const simulation_project::TransformDesc& transform);
    bool setToolAssetMountToVisual(
        const QString& assetId,
        const simulation_project::TransformDesc& transform);
    bool setToolAssetTcp(
        const QString& assetId,
        const simulation_project::TransformDesc& transform);
    bool setToolAssetPreviewMountToVisual(
        const simulation_project::TransformDesc& transform);
    bool setToolAssetPreviewTcp(
        const simulation_project::TransformDesc& transform);
    void setCollisionGeometryVisible(bool visible);
    bool setVisibleRobotCollisionVariant(
        const QString& robotId,
        const QString& linkName,
        const QString& variantId);
    QString visibleRobotCollisionVariant(
        const QString& robotId,
        const QString& linkName) const;
    std::vector<ProjectScene::CollisionDetectorInfo> collisionDetectors() const;
    bool collisionQueriesEnabled() const;
    bool setCollisionQueriesEnabled(bool enabled);
    bool setActiveCollisionDetector(const QString& id);
    bool setCollisionDetectorEnabled(const QString& id, bool enabled);
    bool setCollisionDetectorVisible(const QString& id, bool visible);
    bool updateCollisionDetectorRuntimeOptions(const simulation_project::CollisionDetectorDesc& desc);
    bool rebuildCollisionDetectorsFromDocument(const simulation_project::ProjectDocument& document);
    bool removeCollisionDetector(const QString& id);
    bool generateRobotCollisionProxy(
        const QString& robotId,
        const QString& linkName,
        const QString& proxyType,
        simulation_project::CollisionElementOverrideDesc& element) const;
    bool generateRobotCollisionProxies(
        const QString& robotId,
        const QString& linkName,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateRobotCollisionProxiesFromExistingCollision(
        const QString& robotId,
        const QString& linkName,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateRobotCollisionProxiesFromExistingCollision(
        const QString& robotId,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateRobotCollisionCoacdFromVisual(
        const QString& robotId,
        const QString& linkName,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateRobotCollisionCoacdFromExistingCollision(
        const QString& robotId,
        const QString& linkName,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateObjectCollisionCoacdFromVisual(
        const QString& objectId,
        std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements) const;
    bool evaluateRobotCollisionProxyQuality(
        const QString& robotId,
        const QString& linkName,
        const RobotCollisionProxyRequest& request,
        const std::vector<simulation_project::CollisionElementOverrideDesc>& elements,
        bool useExistingCollisionInput,
        RobotCollisionProxyQualitySummary& summary) const;
    bool generateMissingRobotCollisionProxies(
        const QString& robotId,
        const QString& proxyType,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    bool generateMissingRobotCollisionProxies(
        const QString& robotId,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const;
    RobotCollisionRobotSummary robotCollisionSummary(
        const QString& robotId,
        const QString& activeDetectorRole = QString(),
        const QString& activeDetectorSource = QString()) const;
    std::vector<ProjectScene::MountedAttachmentInfo> mountedAttachments() const;
    std::vector<ProjectScene::ToolAttachmentInfo> toolAttachments() const;
    bool setActiveMountedAttachment(const QString& id);
    bool setActiveToolAttachment(const QString& id);
    void setActiveToolFrameRobot(const QString& robotId);
    void setToolFrameVisibility(const ProjectScene::ToolFrameVisibility& visibility);
    std::vector<ProjectScene::RobotLinkMaterialInfo> robotLinkMaterials(
        const QString& robotId,
        const QString& linkName) const;
    void setCameraView(ProjectSceneCameraView view);
    void resetCamera();
    void focusFullScene(double duration = 0.3);
    void setProjectionMode(ProjectSceneProjectionMode mode);
    ProjectSceneProjectionMode projectionMode() const;
    void setInteractionMode(ProjectSceneInteractionMode mode);
    void beginRotationSurfacePick();
    ProjectSceneInteractionMode interactionMode() const;
    bool applySurfaceScalarOverlay(
        const smrobot::visualization::SurfaceScalarOverlay& overlay,
        QString* errorMessage = nullptr);
    bool setSurfaceScalarOverlayVisible(const QString& objectId, bool visible);
    bool clearSurfaceScalarOverlay(const QString& objectId);
    void setCoatingTrajectoryPreview(
        const std::vector<ProjectScene::CoatingTrajectoryPreviewPoint>& points,
        bool visible);
    bool setCoatingTrajectoryPreviewVisible(bool visible);
    void setCoatingPredictionDebugState(
        const ProjectScene::CoatingPredictionDebugState& state);
    void setCoatingPredictionDebugVisibility(
        const ProjectScene::CoatingPredictionDebugVisibility& visibility);
    void clearCoatingPredictionDebugState();
    bool setCoatingModelVisible(const QString& objectId, bool visible);
    void setCoatingModelVisibilities(const std::unordered_map<std::string, bool>& visibility);
    void clearCoatingModelVisibility(const QString& objectId);
    void setCoatingAnalysisView(bool active);
    void setGpuPredictionBusy(bool busy);
    void setSurfaceScalarProbeEnabled(bool enabled, const QString& objectId = QString());

signals:
    void robotLinksAvailable(
        const QString& robotId,
        const QString& robotName,
        const QStringList& links,
        const QStringList& joints,
        const QStringList& movableJoints,
        const QStringList& movableJointTypes);
    void sceneObjectAvailable(
        const QString& objectId,
        const QString& objectName);
    void robotStateUpdated();
    void scenePicked(
        const QString& kind,
        const QString& robotId,
        const QString& linkName,
        const QString& robotMountId,
        const QString& mountedAttachmentId,
        const QString& sceneObjectId);
    void sceneSelectionCleared();
    void rotationSurfacePicked(
        const QString& objectId,
        std::uint32_t triangleIndex,
        double hitX,
        double hitY,
        double hitZ,
        double normalX,
        double normalY,
        double normalZ);
    void surfaceScalarHovered(
        const QString& objectId,
        double value,
        double worldX,
        double worldY,
        double worldZ,
        const QPoint& viewportPosition,
        bool hit);

protected:
    void initializeGL() override;
    void resizeGL(int width, int height) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void leaveEvent(QEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private:
    using Clock = std::chrono::steady_clock;

    bool initializeSceneWithCurrentContext(bool releaseContext);
    void publishRobotLinks();
    void showViewportContextMenu(const QPoint& position);
    void beginRotationCenterSelection();
    void cancelRotationCenterSelection();
    bool completeRotationCenterSelection(const QPoint& position);
    bool completeRotationCenterSelection(const QRect& rectangle);
    void updateSurfaceScalarProbe(const QPoint& position);

    int m_jointPreviewDegrees = 0;
    QString m_robotName;
    std::size_t m_linkCount = 0;
    std::size_t m_jointCount = 0;
    std::unique_ptr<ProjectScene> m_scene;
    simulation_project::ColorDesc m_defaultBackgroundColor;
    simulation_project::ProjectDocument m_pendingProjectDocument;
    std::filesystem::path m_pendingProjectBasePath;
    QTimer* m_updateTimer = nullptr;
    QRubberBand* m_rotationCenterRubberBand = nullptr;
    QPoint m_lastMousePos;
    QPoint m_mousePressPos;
    QPoint m_rotationCenterPressPos;
    QString m_lastError;
    Clock::time_point m_startTime;
    ProjectSceneInteractionMode m_interactionMode = ProjectSceneInteractionMode::Browse;
    bool m_treePublished = false;
    bool m_hasPendingProjectDocument = false;
    bool m_sceneUpdatePending = true;
    bool m_surfaceScalarProbeEnabled = false;
    QString m_surfaceScalarProbeObjectId;
    bool m_coatingAnalysisViewActive = false;
    bool m_gpuPredictionBusy = false;
    bool m_rotationCenterSelectionActive = false;
    bool m_rotationCenterDragActive = false;
};
