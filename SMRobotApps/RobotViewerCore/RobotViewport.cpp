#include "RobotViewport.h"

#include "ProjectScene.h"
#include "ProjectScenePickingService.h"

#include <CustomLog/CustomLog.h>
#include <SimulationProject/ProjectDocument.h>

#include <QApplication>
#include <QKeyEvent>
#include <QEvent>
#include <QMenu>
#include <QMouseEvent>
#include <QRubberBand>
#include <QStringList>
#include <QTimer>
#include <QWheelEvent>

#include <algorithm>
#include <chrono>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>

namespace
{
    double elapsedMilliseconds(const std::chrono::steady_clock::time_point& start)
    {
        const auto elapsed = std::chrono::steady_clock::now() - start;
        return std::chrono::duration<double, std::milli>(elapsed).count();
    }

    void logProfileRow(const std::string& stage, double ms, const std::string& detail)
    {
        std::ostringstream out;
        out << "| " << std::left << std::setw(32) << stage
            << " | " << std::right << std::setw(10) << std::fixed << std::setprecision(2) << ms
            << " ms | " << detail;
        const std::string line = out.str();
        std::cout << line << "\n";
        LOG_DEBUG("rs2026") << line;
    }

    bool modeAllowsMountedAttachmentSelection(ProjectSceneInteractionMode mode)
    {
        return mode == ProjectSceneInteractionMode::Browse ||
            mode == ProjectSceneInteractionMode::SelectMount ||
            mode == ProjectSceneInteractionMode::SelectAttachment ||
            mode == ProjectSceneInteractionMode::SelectCollisionTarget;
    }

    QString pickKindName(ProjectScenePickTargetKind kind)
    {
        switch(kind) {
        case ProjectScenePickTargetKind::Robot:
            return QStringLiteral("robot");
        case ProjectScenePickTargetKind::RobotLink:
            return QStringLiteral("robotLink");
        case ProjectScenePickTargetKind::RobotMount:
            return QStringLiteral("robotMount");
        case ProjectScenePickTargetKind::MountedAttachment:
            return QStringLiteral("mountedAttachment");
        case ProjectScenePickTargetKind::SceneObject:
            return QStringLiteral("sceneObject");
        case ProjectScenePickTargetKind::PointCloud:
            return QStringLiteral("pointCloud");
        case ProjectScenePickTargetKind::None:
            break;
        }
        return QString();
    }

    QString toQString(const std::string& text)
    {
        return QString::fromStdString(text);
    }

    int mouseDragThreshold()
    {
        return std::max(QApplication::startDragDistance(), 1);
    }
}

RobotViewport::RobotViewport(QWidget* parent)
    : QOpenGLWidget(parent)
    , m_scene(std::make_unique<ProjectScene>())
{
    setMinimumSize(640, 480);
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    m_rotationCenterRubberBand = new QRubberBand(QRubberBand::Rectangle, this);

    m_updateTimer = new QTimer(this);
    m_updateTimer->setInterval(16);
    connect(m_updateTimer, &QTimer::timeout, this, [this]() {
        if(m_gpuPredictionBusy) {
            return;
        }
        if(!m_surfaceScalarProbeEnabled) {
            m_sceneUpdatePending = true;
        }
        update();
    });
    m_updateTimer->start();
}

RobotViewport::~RobotViewport() = default;

void RobotViewport::showViewportContextMenu(const QPoint& position)
{
    if(m_scene == nullptr ||
        !m_scene->isInitialized() ||
        m_gpuPredictionBusy) {
        return;
    }
    if(m_scene->pickScreenPoint(position.x(), position.y()).valid() ||
        m_scene->pickTriangleScreenPoint(position.x(), position.y()).valid()) {
        return;
    }

    QMenu menu(this);
    QAction* resetView = menu.addAction(
        QString::fromUtf8(u8"\u91cd\u65b0\u5b9a\u4f4d"));
    menu.addSeparator();
    QAction* changeRotationCenter = menu.addAction(
        QString::fromUtf8(u8"\u66f4\u6539\u65cb\u8f6c\u4e2d\u5fc3\u70b9"));
    QAction* selectedAction = menu.exec(mapToGlobal(position));
    if(selectedAction == resetView) {
        m_lastMousePos = QPoint();
        m_scene->setCameraView(ProjectSceneCameraView::Home);
        update();
    } else if(selectedAction == changeRotationCenter) {
        beginRotationCenterSelection();
    }
}

void RobotViewport::beginRotationCenterSelection()
{
    if(m_scene == nullptr || !m_scene->isInitialized()) {
        return;
    }

    m_rotationCenterSelectionActive = true;
    m_rotationCenterDragActive = false;
    m_rotationCenterPressPos = QPoint();
    if(m_rotationCenterRubberBand != nullptr) {
        m_rotationCenterRubberBand->hide();
    }
    setCursor(Qt::CrossCursor);
    setFocus(Qt::MouseFocusReason);
}

void RobotViewport::cancelRotationCenterSelection()
{
    m_rotationCenterSelectionActive = false;
    m_rotationCenterDragActive = false;
    m_rotationCenterPressPos = QPoint();
    if(m_rotationCenterRubberBand != nullptr) {
        m_rotationCenterRubberBand->hide();
    }
    unsetCursor();
}

bool RobotViewport::completeRotationCenterSelection(const QPoint& position)
{
    if(m_scene == nullptr) {
        return false;
    }
    const bool changed = m_scene->setRotationCenterFromScreenPoint(position.x(), position.y());
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::completeRotationCenterSelection(const QRect& rectangle)
{
    if(m_scene == nullptr || rectangle.isEmpty()) {
        return false;
    }
    const std::size_t selectedVertexCount = m_scene->setRotationCenterFromScreenRect(
        rectangle.left(),
        rectangle.top(),
        rectangle.right(),
        rectangle.bottom());
    if(selectedVertexCount > 0) {
        update();
    }
    return selectedVertexCount > 0;
}

void RobotViewport::setJointPreview(int degrees)
{
    m_jointPreviewDegrees = degrees;
}

void RobotViewport::setRobotSummary(const char* name, std::size_t linkCount, std::size_t jointCount)
{
    m_robotName = name != nullptr ? name : "";
    m_linkCount = linkCount;
    m_jointCount = jointCount;
}

bool RobotViewport::loadProjectDocument(
    const simulation_project::ProjectDocument& document,
    const std::filesystem::path& basePath)
{
    cancelRotationCenterSelection();
    const auto loadStart = std::chrono::steady_clock::now();
    m_lastError.clear();
    bool ok = true;
    try {
        const auto createStart = std::chrono::steady_clock::now();
        m_scene = std::make_unique<ProjectScene>();
        m_scene->setDefaultBackgroundColor(m_defaultBackgroundColor);
        m_scene->setProjectDocument(document, basePath);
        m_scene->resize(width(), height());
        logProfileRow(
            "RobotViewport prepare scene",
            elapsedMilliseconds(createStart),
            "size=" + std::to_string(width()) + "x" + std::to_string(height()));
        m_treePublished = false;
        m_pendingProjectDocument = document;
        m_pendingProjectBasePath = basePath;
        m_hasPendingProjectDocument = context() == nullptr || !isValid();

        if(!m_hasPendingProjectDocument) {
            const auto initStart = std::chrono::steady_clock::now();
            ok = initializeSceneWithCurrentContext(true);
            logProfileRow(
                "RobotViewport initialize",
                elapsedMilliseconds(initStart),
                ok ? "ok=true" : "ok=false");
        }
    } catch(const std::exception& e) {
        ok = false;
        m_lastError = QString::fromLocal8Bit(e.what());
        LOG_ERROR("rs2026") << "RobotViewport loadProjectDocument failed: " << e.what();
    } catch(...) {
        ok = false;
        m_lastError = "Unknown viewport project load error.";
        LOG_ERROR("rs2026") << "RobotViewport loadProjectDocument failed: unknown error";
    }

    if(m_scene != nullptr) {
        m_scene->setInteractionMode(m_interactionMode);
    }
    m_startTime = Clock::now();
    const auto repaintStart = std::chrono::steady_clock::now();
    update();
    repaint();
    logProfileRow("RobotViewport repaint request", elapsedMilliseconds(repaintStart), "");
    LOG_DEBUG("rs2026") << "RobotViewport loadProjectDocument: elapsedMs=" << elapsedMilliseconds(loadStart)
        << ", initializedNow=" << (!m_hasPendingProjectDocument)
        << ", ok=" << ok;
    logProfileRow(
        "RobotViewport total",
        elapsedMilliseconds(loadStart),
        std::string("initializedNow=") + (!m_hasPendingProjectDocument ? "true" : "false") +
            " ok=" + (ok ? "true" : "false"));
    return ok;
}

void RobotViewport::setDefaultBackgroundColor(const simulation_project::ColorDesc& color)
{
    m_defaultBackgroundColor = color;
    if(m_scene != nullptr) {
        m_scene->setDefaultBackgroundColor(color);
    }
    update();
}

bool RobotViewport::refreshCollisionConfiguration(
    const simulation_project::ProjectDocument& document,
    const std::filesystem::path& basePath)
{
    if(m_scene == nullptr) {
        return false;
    }

    if(context() == nullptr || !isValid()) {
        m_lastError = "Viewport OpenGL context is not valid for collision refresh.";
        return false;
    }

    bool ok = false;
    makeCurrent();
    try {
        ok = m_scene->refreshCollisionConfiguration(document, basePath);
    } catch(const std::exception& e) {
        ok = false;
        m_lastError = QString::fromLocal8Bit(e.what());
        LOG_ERROR("rs2026") << "RobotViewport refreshCollisionConfiguration failed: " << e.what();
    } catch(...) {
        ok = false;
        m_lastError = "Unknown viewport collision refresh error.";
        LOG_ERROR("rs2026") << "RobotViewport refreshCollisionConfiguration failed: unknown error";
    }
    doneCurrent();

    if(ok) {
        update();
    }
    return ok;
}

QString RobotViewport::lastError() const
{
    return m_lastError;
}

bool RobotViewport::loadToolAssetPreview(
    const simulation_project::AttachmentAssetDesc& asset,
    const std::filesystem::path& basePath)
{
    m_scene = std::make_unique<ProjectScene>();
    m_scene->setDefaultBackgroundColor(m_defaultBackgroundColor);
    m_scene->setToolAssetPreview(asset, basePath);
    m_scene->setInteractionMode(m_interactionMode);
    m_scene->resize(width(), height());
    m_treePublished = true;
    m_hasPendingProjectDocument = false;

    bool ok = true;
    if(context() == nullptr || !isValid()) {
        ok = true;
    } else {
        ok = initializeSceneWithCurrentContext(true);
    }

    m_startTime = Clock::now();
    update();
    repaint();
    return ok;
}

bool RobotViewport::setActivePreviewRobotMount(const QString& robotMountId)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setActivePreviewRobotMount(robotMountId.toStdString());
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::setPreviewRobotMountTransform(
    const QString& robotMountId,
    const simulation_project::TransformDesc& transform)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setPreviewRobotMountTransform(robotMountId.toStdString(), transform);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::setPreviewRobotMountLink(
    const QString& robotMountId,
    const QString& linkName)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setPreviewRobotMountLink(robotMountId.toStdString(), linkName.toStdString());
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::upsertPreviewRobotMount(const simulation_project::RobotMountDesc& mount)
{
    const bool changed = m_scene != nullptr && m_scene->upsertPreviewRobotMount(mount);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::removePreviewRobotMount(const QString& robotMountId)
{
    const bool changed = m_scene != nullptr && m_scene->removePreviewRobotMount(robotMountId.toStdString());
    if(changed) {
        update();
    }
    return changed;
}

void RobotViewport::setRobotMountFrameVisibility(bool selectedLinkFrameVisible, bool mountFrameVisible)
{
    if(m_scene != nullptr) {
        m_scene->setRobotMountFrameVisibility(selectedLinkFrameVisible, mountFrameVisible);
        update();
    }
}

void RobotViewport::setPinnedRobotMountFrames(const QStringList& robotMountIds)
{
    std::vector<std::string> ids;
    ids.reserve(static_cast<std::size_t>(robotMountIds.size()));
    for(const QString& id : robotMountIds) {
        if(!id.isEmpty()) {
            ids.push_back(id.toStdString());
        }
    }

    if(m_scene != nullptr) {
        m_scene->setPinnedRobotMountFrames(ids);
        update();
    }
}

void RobotViewport::focusMountFrameLink(const QString& robotId, const QString& linkName)
{
    if(m_scene != nullptr) {
        m_scene->focusMountFrameLink(robotId.toStdString(), linkName.toStdString());
        update();
    }
}

void RobotViewport::clearMountFrameLinkFocus()
{
    if(m_scene != nullptr) {
        m_scene->clearMountFrameLinkFocus();
        update();
    }
}

void RobotViewport::focusObjectFrameObject(const QString& objectId)
{
    if(m_scene != nullptr) {
        m_scene->focusObjectFrameObject(objectId.toStdString());
        update();
    }
}

void RobotViewport::clearObjectFrameObjectFocus()
{
    if(m_scene != nullptr) {
        m_scene->clearObjectFrameObjectFocus();
        update();
    }
}

void RobotViewport::focusCoatingObject(const QString& objectId, double duration)
{
    if(m_scene != nullptr) {
        m_scene->focusCoatingObject(objectId.toStdString(), duration);
        update();
    }
}

void RobotViewport::focusMountedAttachment(const QString& attachmentId)
{
    if(m_scene != nullptr) {
        m_scene->focusMountedAttachment(attachmentId.toStdString());
        update();
    }
}

void RobotViewport::clearMountedAttachmentFocus()
{
    if(m_scene != nullptr) {
        m_scene->clearMountedAttachmentFocus();
        update();
    }
}

void RobotViewport::previewObjectCollisionModelVariant(const QString& objectId, const QString& variantId)
{
    if(m_scene != nullptr) {
        if(context() == nullptr || !isValid()) {
            m_lastError = "Viewport OpenGL context is not valid for collision model preview.";
            return;
        }

        makeCurrent();
        try {
            m_scene->previewObjectCollisionModelVariant(objectId.toStdString(), variantId.toStdString());
        } catch(const std::exception& e) {
            m_lastError = QString::fromLocal8Bit(e.what());
            LOG_ERROR("rs2026") << "RobotViewport previewObjectCollisionModelVariant failed: " << e.what();
        } catch(...) {
            m_lastError = "Unknown viewport collision model preview error.";
            LOG_ERROR("rs2026") << "RobotViewport previewObjectCollisionModelVariant failed: unknown error";
        }
        doneCurrent();
        update();
    }
}

void RobotViewport::clearObjectCollisionModelVariantPreview()
{
    if(m_scene != nullptr) {
        m_scene->clearObjectCollisionModelVariantPreview();
        update();
    }
}

void RobotViewport::previewCollisionPairTargets(
    const QString& robotAId,
    const QString& linkAName,
    const QString& objectAId,
    const QString& attachmentAId,
    const QString& robotBId,
    const QString& linkBName,
    const QString& objectBId,
    const QString& attachmentBId)
{
    if(m_scene != nullptr) {
        m_scene->previewCollisionPairTargets(
            robotAId.toStdString(),
            linkAName.toStdString(),
            objectAId.toStdString(),
            attachmentAId.toStdString(),
            robotBId.toStdString(),
            linkBName.toStdString(),
            objectBId.toStdString(),
            attachmentBId.toStdString());
        update();
    }
}

void RobotViewport::selectRobotLink(const QString& robotId, const QString& linkName)
{
    if(m_scene != nullptr) {
        m_scene->setSelectedLink(robotId.toStdString(), linkName.toStdString());
        m_sceneUpdatePending = true;
        update();
    }
}

void RobotViewport::selectRobotJointFrame(const QString& robotId, const QString& jointName)
{
    if(m_scene != nullptr) {
        m_scene->setSelectedJointFrame(robotId.toStdString(), jointName.toStdString());
        update();
    }
}

void RobotViewport::selectRobotMount(const QString& robotId, const QString& linkName, const QString& robotMountId)
{
    if(m_scene != nullptr) {
        m_scene->setSelectedRobotMount(
            robotId.toStdString(),
            linkName.toStdString(),
            robotMountId.toStdString());
        m_sceneUpdatePending = true;
        update();
    }
}

void RobotViewport::selectSceneObject(const QString& objectId)
{
    if(m_scene != nullptr) {
        m_scene->setSelectedSceneObject(objectId.toStdString());
        m_sceneUpdatePending = true;
        update();
    }
}

void RobotViewport::selectToolAttachment(const QString& attachmentId)
{
    selectMountedAttachment(attachmentId);
}

void RobotViewport::selectMountedAttachment(const QString& attachmentId)
{
    if(m_scene != nullptr) {
        m_scene->setSelectedMountedAttachment(attachmentId.toStdString());
        m_sceneUpdatePending = true;
        update();
    }
}

void RobotViewport::previewRobotBaseTransform(
    const QString& robotId,
    const simulation_project::TransformDesc& transform)
{
    if(m_scene != nullptr) {
        m_scene->setRobotBaseTransform(robotId.toStdString(), transform);
        update();
    }
}

void RobotViewport::setRobotJointValue(
    const QString& robotId,
    const QString& jointName,
    double value)
{
    if(m_scene != nullptr) {
        m_scene->setRobotJointValue(robotId.toStdString(), jointName.toStdString(), value);
        update();
    }
}

double RobotViewport::robotJointValue(
    const QString& robotId,
    const QString& jointName,
    bool* ok) const
{
    double value = 0.0;
    const bool found = m_scene != nullptr
        && m_scene->robotJointValue(robotId.toStdString(), jointName.toStdString(), value);
    if(ok != nullptr) {
        *ok = found;
    }
    return value;
}

void RobotViewport::setRobotAutoMotion(
    const QString& robotId,
    bool enabled,
    double amplitude,
    double speed)
{
    if(m_scene != nullptr) {
        m_scene->setRobotAutoMotion(robotId.toStdString(), enabled, amplitude, speed);
        update();
    }
}

void RobotViewport::previewSceneObjectTransform(
    const QString& objectId,
    const simulation_project::TransformDesc& transform)
{
    if(m_scene != nullptr) {
        m_scene->previewSceneObjectTransform(objectId.toStdString(), transform);
        update();
    }
}

bool RobotViewport::setSceneObjectTransform(
    const QString& objectId,
    const simulation_project::TransformDesc& transform)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setSceneObjectTransform(objectId.toStdString(), transform);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::removeSceneObject(const QString& objectId)
{
    const bool changed = m_scene != nullptr && m_scene->removeSceneObject(objectId.toStdString());
    if(changed) {
        update();
    }
    return changed;
}

void RobotViewport::selectObjectFrame(const QString& objectId, const QString& frameId)
{
    if(m_scene != nullptr) {
        m_scene->setSelectedObjectFrame(objectId.toStdString(), frameId.toStdString());
        update();
    }
}

bool RobotViewport::previewObjectFrameTransform(
    const QString& objectId,
    const QString& frameId,
    const simulation_project::TransformDesc& transform)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setPreviewObjectFrameTransform(
            objectId.toStdString(),
            frameId.toStdString(),
            transform);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::upsertPreviewObjectFrame(
    const QString& objectId,
    const simulation_project::ObjectFrameDesc& frame)
{
    const bool changed = m_scene != nullptr &&
        m_scene->upsertPreviewObjectFrame(objectId.toStdString(), frame);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::setRobotMountTransform(
    const QString& robotMountId,
    const simulation_project::TransformDesc& transform)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setRobotMountTransform(robotMountId.toStdString(), transform);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::setToolAttachmentTransform(
    const QString& attachmentId,
    const simulation_project::TransformDesc& transform)
{
    return setMountedAttachmentTransform(attachmentId, transform);
}

bool RobotViewport::setMountedAttachmentTransform(
    const QString& attachmentId,
    const simulation_project::TransformDesc& transform)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setMountedAttachmentTransform(attachmentId.toStdString(), transform);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::setToolAssetMountToVisual(
    const QString& assetId,
    const simulation_project::TransformDesc& transform)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setToolAssetMountToVisual(assetId.toStdString(), transform);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::setToolAssetTcp(
    const QString& assetId,
    const simulation_project::TransformDesc& transform)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setToolAssetTcp(assetId.toStdString(), transform);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::setToolAssetPreviewMountToVisual(
    const simulation_project::TransformDesc& transform)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setToolAssetPreviewMountToVisual(transform);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::setToolAssetPreviewTcp(
    const simulation_project::TransformDesc& transform)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setToolAssetPreviewTcp(transform);
    if(changed) {
        update();
    }
    return changed;
}

void RobotViewport::setCollisionGeometryVisible(bool visible)
{
    if(m_scene != nullptr) {
        m_scene->setShowCollisionGeometry(visible);
        update();
    }
}

bool RobotViewport::setVisibleRobotCollisionVariant(
    const QString& robotId,
    const QString& linkName,
    const QString& variantId)
{
    const bool changed = m_scene != nullptr &&
        m_scene->setVisibleRobotCollisionVariant(
            robotId.toStdString(),
            linkName.toStdString(),
            variantId.toStdString());
    if(changed) {
        update();
    }
    return changed;
}

QString RobotViewport::visibleRobotCollisionVariant(
    const QString& robotId,
    const QString& linkName) const
{
    return m_scene != nullptr
        ? QString::fromStdString(m_scene->visibleRobotCollisionVariant(
              robotId.toStdString(),
              linkName.toStdString()))
        : QString();
}

std::vector<ProjectScene::CollisionDetectorInfo> RobotViewport::collisionDetectors() const
{
    return m_scene != nullptr ? m_scene->collisionDetectors() : std::vector<ProjectScene::CollisionDetectorInfo>();
}

bool RobotViewport::collisionQueriesEnabled() const
{
    return m_scene != nullptr && m_scene->collisionQueriesEnabled();
}

bool RobotViewport::setCollisionQueriesEnabled(bool enabled)
{
    const bool changed = m_scene != nullptr && m_scene->setCollisionQueriesEnabled(enabled);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::setActiveCollisionDetector(const QString& id)
{
    const bool changed = m_scene != nullptr && m_scene->setActiveCollisionDetector(id.toStdString());
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::setCollisionDetectorEnabled(const QString& id, bool enabled)
{
    const bool changed = m_scene != nullptr && m_scene->setCollisionDetectorEnabled(id.toStdString(), enabled);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::setCollisionDetectorVisible(const QString& id, bool visible)
{
    const bool changed = m_scene != nullptr && m_scene->setCollisionDetectorVisible(id.toStdString(), visible);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::updateCollisionDetectorRuntimeOptions(const simulation_project::CollisionDetectorDesc& desc)
{
    const bool changed = m_scene != nullptr && m_scene->updateCollisionDetectorRuntimeOptions(desc);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::rebuildCollisionDetectorsFromDocument(const simulation_project::ProjectDocument& document)
{
    const bool changed = m_scene != nullptr && m_scene->rebuildCollisionDetectorsFromDocument(document);
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::removeCollisionDetector(const QString& id)
{
    const bool changed = m_scene != nullptr && m_scene->removeCollisionDetector(id.toStdString());
    if(changed) {
        update();
    }
    return changed;
}

bool RobotViewport::generateRobotCollisionProxy(
    const QString& robotId,
    const QString& linkName,
    const QString& proxyType,
    simulation_project::CollisionElementOverrideDesc& element) const
{
    return m_scene != nullptr &&
        m_scene->generateRobotCollisionProxy(
            robotId.toStdString(),
            linkName.toStdString(),
            proxyType.toStdString(),
            element);
}

bool RobotViewport::generateRobotCollisionProxies(
    const QString& robotId,
    const QString& linkName,
    const RobotCollisionProxyRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    return m_scene != nullptr &&
        m_scene->generateRobotCollisionProxies(
            robotId.toStdString(),
            linkName.toStdString(),
            request,
            elements);
}

bool RobotViewport::generateRobotCollisionProxiesFromExistingCollision(
    const QString& robotId,
    const QString& linkName,
    const RobotCollisionProxyRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    return m_scene != nullptr &&
        m_scene->generateRobotCollisionProxiesFromExistingCollision(
            robotId.toStdString(),
            linkName.toStdString(),
            request,
            elements);
}

bool RobotViewport::generateRobotCollisionProxiesFromExistingCollision(
    const QString& robotId,
    const RobotCollisionProxyRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    return m_scene != nullptr &&
        m_scene->generateRobotCollisionProxiesFromExistingCollision(
            robotId.toStdString(),
            request,
            elements);
}

bool RobotViewport::generateRobotCollisionCoacdFromVisual(
    const QString& robotId,
    const QString& linkName,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    return m_scene != nullptr &&
        m_scene->generateRobotCollisionCoacdFromVisual(
            robotId.toStdString(),
            linkName.toStdString(),
            elements);
}

bool RobotViewport::generateRobotCollisionCoacdFromExistingCollision(
    const QString& robotId,
    const QString& linkName,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    return m_scene != nullptr &&
        m_scene->generateRobotCollisionCoacdFromExistingCollision(
            robotId.toStdString(),
            linkName.toStdString(),
            elements);
}

bool RobotViewport::generateObjectCollisionCoacdFromVisual(
    const QString& objectId,
    std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements) const
{
    return m_scene != nullptr &&
        m_scene->generateObjectCollisionCoacdFromVisual(
            objectId.toStdString(),
            elements);
}

bool RobotViewport::evaluateRobotCollisionProxyQuality(
    const QString& robotId,
    const QString& linkName,
    const RobotCollisionProxyRequest& request,
    const std::vector<simulation_project::CollisionElementOverrideDesc>& elements,
    bool useExistingCollisionInput,
    RobotCollisionProxyQualitySummary& summary) const
{
    return m_scene != nullptr &&
        m_scene->evaluateRobotCollisionProxyQuality(
            robotId.toStdString(),
            linkName.toStdString(),
            request,
            elements,
            useExistingCollisionInput,
            summary);
}

bool RobotViewport::generateMissingRobotCollisionProxies(
    const QString& robotId,
    const QString& proxyType,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    return m_scene != nullptr &&
        m_scene->generateMissingRobotCollisionProxies(
            robotId.toStdString(),
            proxyType.toStdString(),
            elements);
}

bool RobotViewport::generateMissingRobotCollisionProxies(
    const QString& robotId,
    const RobotCollisionProxyRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    return m_scene != nullptr &&
        m_scene->generateMissingRobotCollisionProxies(
            robotId.toStdString(),
            request,
            elements);
}

RobotCollisionRobotSummary RobotViewport::robotCollisionSummary(
    const QString& robotId,
    const QString& activeDetectorRole,
    const QString& activeDetectorSource) const
{
    return m_scene != nullptr
        ? m_scene->robotCollisionSummary(
            robotId.toStdString(),
            activeDetectorRole.toStdString(),
            activeDetectorSource.toStdString())
        : RobotCollisionRobotSummary();
}

std::vector<ProjectScene::ToolAttachmentInfo> RobotViewport::toolAttachments() const
{
    return mountedAttachments();
}

std::vector<ProjectScene::MountedAttachmentInfo> RobotViewport::mountedAttachments() const
{
    return m_scene != nullptr ? m_scene->mountedAttachments() : std::vector<ProjectScene::MountedAttachmentInfo>();
}

bool RobotViewport::setActiveToolAttachment(const QString& id)
{
    return setActiveMountedAttachment(id);
}

bool RobotViewport::setActiveMountedAttachment(const QString& id)
{
    const bool changed = m_scene != nullptr && m_scene->setActiveMountedAttachment(id.toStdString());
    if(changed) {
        update();
    }
    return changed;
}

void RobotViewport::setActiveToolFrameRobot(const QString& robotId)
{
    if(m_scene != nullptr) {
        m_scene->setActiveToolFrameRobot(robotId.toStdString());
        update();
    }
}

void RobotViewport::setToolFrameVisibility(const ProjectScene::ToolFrameVisibility& visibility)
{
    if(m_scene != nullptr) {
        m_scene->setToolFrameVisibility(visibility);
        update();
    }
}

std::vector<ProjectScene::RobotLinkMaterialInfo> RobotViewport::robotLinkMaterials(
    const QString& robotId,
    const QString& linkName) const
{
    return m_scene != nullptr
        ? m_scene->robotLinkMaterials(robotId.toStdString(), linkName.toStdString())
        : std::vector<ProjectScene::RobotLinkMaterialInfo>();
}

void RobotViewport::resetCamera()
{
    cancelRotationCenterSelection();
    m_lastMousePos = QPoint();
    setCameraView(ProjectSceneCameraView::Home);
}

void RobotViewport::setCameraView(ProjectSceneCameraView view)
{
    m_lastMousePos = QPoint();
    if(m_scene != nullptr) {
        m_scene->setCameraView(view);
        update();
    }
}

void RobotViewport::focusFullScene(double duration)
{
    if(m_scene != nullptr) {
        m_scene->focusFullScene(duration);
        update();
    }
}

void RobotViewport::setProjectionMode(ProjectSceneProjectionMode mode)
{
    if(m_scene != nullptr) {
        m_scene->setProjectionMode(mode);
        update();
    }
}

ProjectSceneProjectionMode RobotViewport::projectionMode() const
{
    return m_scene != nullptr
        ? m_scene->projectionMode()
        : ProjectSceneProjectionMode::Perspective;
}

void RobotViewport::setInteractionMode(ProjectSceneInteractionMode mode)
{
    if(m_rotationCenterSelectionActive) {
        cancelRotationCenterSelection();
    }
    if(m_interactionMode == mode) {
        return;
    }
    m_interactionMode = mode;
    if(mode != ProjectSceneInteractionMode::SelectRotationSurface) {
        unsetCursor();
    }
    if(m_scene != nullptr) {
        m_scene->setInteractionMode(mode);
        update();
    }
}

void RobotViewport::beginRotationSurfacePick()
{
    cancelRotationCenterSelection();
    m_interactionMode = ProjectSceneInteractionMode::SelectRotationSurface;
    if(m_scene != nullptr) {
        m_scene->setInteractionMode(m_interactionMode);
    }
    setCursor(Qt::CrossCursor);
    update();
}

ProjectSceneInteractionMode RobotViewport::interactionMode() const
{
    return m_interactionMode;
}

bool RobotViewport::applySurfaceScalarOverlay(
    const smrobot::visualization::SurfaceScalarOverlay& overlay,
    QString* errorMessage)
{
    if(m_scene == nullptr || context() == nullptr || !isValid()) {
        if(errorMessage != nullptr) {
            *errorMessage = QStringLiteral("Viewport OpenGL context is not available.");
        }
        return false;
    }

    std::string error;
    bool ok = false;
    makeCurrent();
    try {
        ok = m_scene->applySurfaceScalarOverlay(overlay, &error);
    } catch(const std::exception& exception) {
        error = exception.what();
    } catch(...) {
        error = "Unknown surface scalar overlay error.";
    }
    doneCurrent();
    if(errorMessage != nullptr) {
        *errorMessage = QString::fromStdString(error);
    }
    if(ok) {
        update();
    }
    return ok;
}

bool RobotViewport::setSurfaceScalarOverlayVisible(const QString& objectId, bool visible)
{
    if(m_scene == nullptr) {
        return false;
    }
    const bool ok = m_scene->setSurfaceScalarOverlayVisible(objectId.toStdString(), visible);
    if(ok) {
        update();
    }
    return ok;
}

bool RobotViewport::clearSurfaceScalarOverlay(const QString& objectId)
{
    if(m_scene == nullptr) {
        return false;
    }
    const bool ok = m_scene->clearSurfaceScalarOverlay(objectId.toStdString());
    if(ok) {
        update();
    }
    return ok;
}

void RobotViewport::setCoatingTrajectoryPreview(
    const std::vector<ProjectScene::CoatingTrajectoryPreviewPoint>& points,
    bool visible)
{
    if(m_scene == nullptr) {
        return;
    }
    m_scene->setCoatingTrajectoryPreview(points, visible);
    m_sceneUpdatePending = true;
    update();
}

bool RobotViewport::setCoatingTrajectoryPreviewVisible(bool visible)
{
    if(m_scene == nullptr) {
        return false;
    }
    const bool cached = m_scene->setCoatingTrajectoryPreviewVisible(visible);
    update();
    return cached;
}

void RobotViewport::setCoatingPredictionDebugState(
    const ProjectScene::CoatingPredictionDebugState& state)
{
    if(m_scene == nullptr) {
        return;
    }
    m_scene->setCoatingPredictionDebugState(state);
    m_sceneUpdatePending = true;
    update();
}

void RobotViewport::setCoatingPredictionDebugVisibility(
    const ProjectScene::CoatingPredictionDebugVisibility& visibility)
{
    if(m_scene == nullptr) {
        return;
    }
    m_scene->setCoatingPredictionDebugVisibility(visibility);
    m_sceneUpdatePending = true;
    update();
}

void RobotViewport::clearCoatingPredictionDebugState()
{
    if(m_scene == nullptr) {
        return;
    }
    const bool hasOpenGlContext = context() != nullptr && isValid();
    if(hasOpenGlContext) {
        makeCurrent();
    }
    m_scene->clearCoatingPredictionDebugState();
    if(hasOpenGlContext) {
        doneCurrent();
    }
    m_sceneUpdatePending = true;
    update();
}

bool RobotViewport::setCoatingModelVisible(const QString& objectId, bool visible)
{
    if(m_scene == nullptr) {
        return false;
    }
    const bool ok = m_scene->setCoatingModelVisible(objectId.toStdString(), visible);
    if(ok) {
        update();
    }
    return ok;
}

void RobotViewport::setCoatingModelVisibilities(
    const std::unordered_map<std::string, bool>& visibility)
{
    if(m_scene == nullptr) {
        return;
    }
    m_scene->setCoatingModelVisibilities(visibility);
    update();
}

void RobotViewport::clearCoatingModelVisibility(const QString& objectId)
{
    if(m_scene == nullptr) {
        return;
    }
    m_scene->clearCoatingModelVisibility(objectId.toStdString());
    update();
}

void RobotViewport::setCoatingAnalysisView(bool active)
{
    m_coatingAnalysisViewActive = active;
    if(m_scene == nullptr) {
        return;
    }
    m_scene->setCoatingAnalysisView(active);
    update();
}

void RobotViewport::setGpuPredictionBusy(bool busy)
{
    if(m_gpuPredictionBusy == busy) {
        return;
    }
    m_gpuPredictionBusy = busy;
    if(!busy) {
        m_sceneUpdatePending = true;
        update();
    }
}

void RobotViewport::setSurfaceScalarProbeEnabled(bool enabled, const QString& objectId)
{
    m_surfaceScalarProbeEnabled = enabled;
    m_surfaceScalarProbeObjectId = enabled ? objectId : QString();
    if(enabled && m_scene != nullptr) {
        // Thickness picking uses the surface probe, not the scene-object hover outline.
        m_scene->clearSceneObjectHover();
        m_sceneUpdatePending = true;
        update();
    }
    if(!enabled) {
        emit surfaceScalarHovered(QString(), 0.0, 0.0, 0.0, 0.0, QPoint(), false);
    }
}

void RobotViewport::updateSurfaceScalarProbe(const QPoint& position)
{
    if(!m_surfaceScalarProbeEnabled || m_surfaceScalarProbeObjectId.isEmpty() ||
        m_scene == nullptr || m_gpuPredictionBusy) {
        return;
    }
    const smrobot::visualization::SurfaceScalarProbeResult probe =
        m_scene->probeSurfaceScalarAtScreenPoint(
            m_surfaceScalarProbeObjectId.toStdString(),
            position.x(),
            position.y());
    emit surfaceScalarHovered(
        m_surfaceScalarProbeObjectId,
        probe.value,
        probe.worldPosition.x(),
        probe.worldPosition.y(),
        probe.worldPosition.z(),
        position,
        probe.hit);
}

bool RobotViewport::initializeSceneWithCurrentContext(bool releaseContext)
{
    const auto initializeStart = std::chrono::steady_clock::now();
    if(m_scene == nullptr || context() == nullptr) {
        return false;
    }

    if(releaseContext) {
        makeCurrent();
    }
    bool ok = true;
    try {
        ok = m_scene->initialize();
    } catch(const std::exception& e) {
        ok = false;
        m_lastError = QString::fromLocal8Bit(e.what());
        LOG_ERROR("rs2026") << "RobotViewport initializeSceneWithCurrentContext failed: " << e.what();
    } catch(...) {
        ok = false;
        m_lastError = "Unknown viewport scene initialize error.";
        LOG_ERROR("rs2026") << "RobotViewport initializeSceneWithCurrentContext failed: unknown error";
    }
    if(ok) {
        // Apply the coating view mode at scene-initialize time so robots are
        // hidden before the first frame after a viewport reload (no flash).
        m_scene->setCoatingAnalysisView(m_coatingAnalysisViewActive);
        publishRobotLinks();
    }
    if(releaseContext) {
        doneCurrent();
    }
    LOG_DEBUG("rs2026") << "RobotViewport initializeSceneWithCurrentContext: elapsedMs="
        << elapsedMilliseconds(initializeStart)
        << ", releaseContext=" << releaseContext
        << ", ok=" << ok;
    return ok;
}

void RobotViewport::initializeGL()
{
    if(m_hasPendingProjectDocument) {
        if(m_scene == nullptr) {
            m_scene = std::make_unique<ProjectScene>();
            m_scene->setDefaultBackgroundColor(m_defaultBackgroundColor);
            m_scene->setProjectDocument(m_pendingProjectDocument, m_pendingProjectBasePath);
            m_scene->resize(width(), height());
            m_treePublished = false;
        }
        m_hasPendingProjectDocument = false;
    }
    initializeSceneWithCurrentContext(false);
    m_startTime = Clock::now();
}

void RobotViewport::publishRobotLinks()
{
    if(m_scene == nullptr || !m_scene->isInitialized() || m_treePublished) {
        return;
    }

    for(const auto& group : m_scene->robotLinks()) {
        QStringList links;
        for(const auto& link : group.links) {
            links.push_back(QString::fromStdString(link));
        }
        QStringList joints;
        for(const auto& joint : group.joints) {
            joints.push_back(QString::fromStdString(joint));
        }
        QStringList movableJoints;
        QStringList movableJointTypes;
        for(const auto& joint : group.movableJoints) {
            movableJoints.push_back(QString::fromStdString(joint.jointName));
            movableJointTypes.push_back(QString::fromStdString(joint.jointType));
        }
        emit robotLinksAvailable(
            QString::fromStdString(group.robotId),
            QString::fromStdString(group.robotName),
            links,
            joints,
            movableJoints,
            movableJointTypes);
    }

    for(const auto& object : m_scene->sceneObjects()) {
        emit sceneObjectAvailable(
            QString::fromStdString(object.objectId),
            QString::fromStdString(object.objectName));
    }
    m_treePublished = true;
}

void RobotViewport::resizeGL(int width, int height)
{
    if(m_scene != nullptr) {
        m_scene->resize(width, height);
    }
}

void RobotViewport::paintGL()
{
    if(m_scene == nullptr || !m_scene->isInitialized()) {
        return;
    }

    const auto now = Clock::now();
    const std::chrono::duration<double> elapsed = now - m_startTime;
    if(!m_gpuPredictionBusy && (!m_surfaceScalarProbeEnabled || m_sceneUpdatePending)) {
        m_scene->update(elapsed.count());
        m_sceneUpdatePending = false;
        emit robotStateUpdated();
    }
    m_scene->render();
}

void RobotViewport::mousePressEvent(QMouseEvent* event)
{
    m_lastMousePos = event->pos();
    m_mousePressPos = event->pos();
    if(m_scene != nullptr && event->button() != Qt::NoButton) {
        m_scene->clearSceneObjectHover();
    }
    if(m_rotationCenterSelectionActive &&
        (event->button() == Qt::LeftButton || event->button() == Qt::RightButton)) {
        if(event->button() == Qt::LeftButton) {
            m_rotationCenterPressPos = event->pos();
            m_rotationCenterDragActive = false;
            if(m_rotationCenterRubberBand != nullptr) {
                m_rotationCenterRubberBand->setGeometry(QRect(event->pos(), QSize()));
                m_rotationCenterRubberBand->hide();
            }
        }
        return;
    }
    if(m_surfaceScalarProbeEnabled) {
        emit surfaceScalarHovered(
            m_surfaceScalarProbeObjectId,
            0.0,
            0.0,
            0.0,
            0.0,
            event->pos(),
            false);
    }
}

void RobotViewport::mouseReleaseEvent(QMouseEvent* event)
{
    if(m_scene == nullptr) {
        return;
    }

    if(event->button() == Qt::LeftButton) {
        m_scene->setRotationCenterMarkerVisible(false);
        update();
    }

    if(event->button() == Qt::RightButton) {
        if(m_rotationCenterSelectionActive) {
            cancelRotationCenterSelection();
            return;
        }
        const QPoint delta = event->pos() - m_mousePressPos;
        if(delta.manhattanLength() <= mouseDragThreshold()) {
            showViewportContextMenu(event->pos());
        }
        return;
    }

    if(event->button() != Qt::LeftButton) {
        return;
    }

    if(m_rotationCenterSelectionActive) {
        const QRect selectionRect = QRect(m_rotationCenterPressPos, event->pos()).normalized();
        if(m_rotationCenterRubberBand != nullptr) {
            m_rotationCenterRubberBand->hide();
        }
        const bool changed = m_rotationCenterDragActive
            ? completeRotationCenterSelection(selectionRect)
            : completeRotationCenterSelection(event->pos());
        m_rotationCenterDragActive = false;
        if(changed) {
            cancelRotationCenterSelection();
        }
        return;
    }

    const QPoint delta = event->pos() - m_mousePressPos;
    if(delta.manhattanLength() > mouseDragThreshold()) {
        return;
    }

    if(m_interactionMode == ProjectSceneInteractionMode::SelectRotationSurface) {
        const ProjectSceneTrianglePickResult result =
            m_scene->pickTriangleScreenPoint(event->pos().x(), event->pos().y());
        if(result.valid()) {
            emit rotationSurfacePicked(
                toQString(result.sceneObjectId),
                result.triangleIndex,
                result.hitPosition.x(),
                result.hitPosition.y(),
                result.hitPosition.z(),
                result.normal.x(),
                result.normal.y(),
                result.normal.z());
            setInteractionMode(ProjectSceneInteractionMode::Browse);
        }
        return;
    }

    if(m_surfaceScalarProbeEnabled && !m_surfaceScalarProbeObjectId.isEmpty()) {
        updateSurfaceScalarProbe(event->pos());
    }

    const ProjectScenePickResult result = m_scene->pickScreenPoint(event->pos().x(), event->pos().y());
    if(!result.valid()) {
        if(m_coatingAnalysisViewActive) {
            m_scene->setSelectedSceneObject(std::string());
            m_sceneUpdatePending = true;
        }
        emit sceneSelectionCleared();
        update();
        return;
    }

    if(m_coatingAnalysisViewActive && !result.sceneObjectId.empty()) {
        // Keep coating viewport selection responsive instead of waiting for
        // the workbench selection event to make a second render pass.
        m_scene->setSelectedSceneObject(result.sceneObjectId);
        m_sceneUpdatePending = true;
    }

    emit scenePicked(
        pickKindName(result.kind),
        toQString(result.robotId),
        toQString(result.linkName),
        toQString(result.robotMountId),
        toQString(result.mountedAttachmentId),
        toQString(result.sceneObjectId));
    update();
}

void RobotViewport::mouseMoveEvent(QMouseEvent* event)
{
    if(m_rotationCenterSelectionActive) {
        if(event->buttons() & Qt::LeftButton) {
            const QPoint delta = event->pos() - m_rotationCenterPressPos;
            if(delta.manhattanLength() > mouseDragThreshold()) {
                m_rotationCenterDragActive = true;
                if(m_rotationCenterRubberBand != nullptr) {
                    m_rotationCenterRubberBand->setGeometry(
                        QRect(m_rotationCenterPressPos, event->pos()).normalized());
                    m_rotationCenterRubberBand->show();
                }
            }
        }
        return;
    }

    if(m_scene == nullptr || m_lastMousePos.isNull()) {
        m_lastMousePos = event->pos();
        return;
    }

    const QPoint delta = event->pos() - m_lastMousePos;
    int button = -1;
    if(event->buttons() & Qt::LeftButton) {
        button = 0;
    } else if(event->buttons() & Qt::RightButton) {
        button = 1;
    }

    if(button == 0) {
        m_scene->clearSceneObjectHover();
        m_scene->onMouseMove(
            static_cast<float>(delta.x()) * 0.8f,
            static_cast<float>(-delta.y()) * 0.8f,
            button);
        update();
    } else if(button == 1) {
        m_scene->clearSceneObjectHover();
        m_scene->onMouseMove(
            static_cast<float>(delta.x()),
            static_cast<float>(delta.y()),
            button);
        update();
    } else if((m_interactionMode == ProjectSceneInteractionMode::Browse ||
        (m_coatingAnalysisViewActive &&
            m_interactionMode != ProjectSceneInteractionMode::SelectRotationSurface)) &&
        !m_gpuPredictionBusy && !m_surfaceScalarProbeEnabled) {
        m_scene->requestSceneObjectHover(event->pos().x(), event->pos().y());
        update();
    }

    if(button < 0) {
        updateSurfaceScalarProbe(event->pos());
    }

    m_lastMousePos = event->pos();
}

void RobotViewport::leaveEvent(QEvent* event)
{
    if(m_scene != nullptr) {
        m_scene->clearSceneObjectHover();
        m_scene->setRotationCenterMarkerVisible(false);
        m_sceneUpdatePending = true;
        update();
    }
    if(m_surfaceScalarProbeEnabled) {
        emit surfaceScalarHovered(
            m_surfaceScalarProbeObjectId,
            0.0,
            0.0,
            0.0,
            0.0,
            QPoint(),
            false);
    }
    QOpenGLWidget::leaveEvent(event);
}

void RobotViewport::wheelEvent(QWheelEvent* event)
{
    if(m_scene == nullptr) {
        return;
    }

    m_scene->clearSceneObjectHover();
    const float delta = static_cast<float>(event->angleDelta().y()) / 120.0f;
    m_scene->onScroll(delta * 0.8f, event->pos().x(), event->pos().y());
    update();
}

void RobotViewport::keyPressEvent(QKeyEvent* event)
{
    if(m_rotationCenterSelectionActive &&
        event->key() == Qt::Key_Escape &&
        !event->isAutoRepeat()) {
        cancelRotationCenterSelection();
        return;
    }

    if(m_scene == nullptr) {
        QOpenGLWidget::keyPressEvent(event);
        return;
    }

    if(event->key() == Qt::Key_T && !event->isAutoRepeat()) {
        if(!modeAllowsMountedAttachmentSelection(m_interactionMode)) {
            QOpenGLWidget::keyPressEvent(event);
            return;
        }
        const bool reverse = (event->modifiers() & Qt::ShiftModifier) != 0;
        if(m_scene->cycleToolAttachment(reverse)) {
            update();
            return;
        }
    }

    QOpenGLWidget::keyPressEvent(event);
}
