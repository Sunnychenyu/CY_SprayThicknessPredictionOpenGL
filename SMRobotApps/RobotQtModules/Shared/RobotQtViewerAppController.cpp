#include "RobotQtViewerAppController.h"

#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerEventHub.h"
#include "RobotQtViewerSelectionModel.h"

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectDocumentService.h>

#include <algorithm>

namespace robot_qt_viewer
{
    RobotQtViewerAppController::RobotQtViewerAppController(RobotQtViewerDocumentContext& context)
        : m_context(context)
        , m_projectWorkflow(context)
        , m_viewportReloadWorkflow(context)
        , m_sceneEntityWorkflow(context)
    {
    }

    RobotQtViewerAppController::~RobotQtViewerAppController() = default;

    RobotQtViewerDocumentContext& RobotQtViewerAppController::documentContext()
    {
        return m_context;
    }

    const RobotQtViewerDocumentContext& RobotQtViewerAppController::documentContext() const
    {
        return m_context;
    }

    simulation_project::ProjectSession& RobotQtViewerAppController::session()
    {
        return m_context.projectSession();
    }

    const simulation_project::ProjectSession& RobotQtViewerAppController::session() const
    {
        return m_context.projectSession();
    }

    const simulation_project::ProjectDocument& RobotQtViewerAppController::document() const
    {
        return m_context.document();
    }

    RobotQtViewerDocumentController& RobotQtViewerAppController::documentController()
    {
        return m_context.documentController();
    }

    RobotQtViewerSelectionModel& RobotQtViewerAppController::selectionModel()
    {
        return m_context.selectionModel();
    }

    RobotQtViewerEventHub& RobotQtViewerAppController::eventHub()
    {
        return m_context.eventHub();
    }

    ProjectSessionWorkflowResult RobotQtViewerAppController::resetProject(const QString& sourceId)
    {
        return m_projectWorkflow.resetNew(sourceId);
    }

    ProjectSessionWorkflowResult RobotQtViewerAppController::loadProject(
        const std::filesystem::path& path,
        const QString& sourceId)
    {
        return m_projectWorkflow.loadFromPath(path, sourceId);
    }

    ProjectSessionWorkflowResult RobotQtViewerAppController::loadStartupProject(
        const std::filesystem::path& path,
        const QString& sourceId)
    {
        return m_projectWorkflow.loadStartupProject(path, sourceId);
    }

    ProjectSessionWorkflowResult RobotQtViewerAppController::saveProject(
        const std::filesystem::path& path,
        bool saveAsV3,
        const QString& sourceId)
    {
        return m_projectWorkflow.saveToPath(path, saveAsV3, sourceId);
    }

    ViewportReloadWorkflowResult RobotQtViewerAppController::reloadViewport(const QString& sourceId)
    {
        return m_viewportReloadWorkflow.reload(sourceId);
    }

    void RobotQtViewerAppController::setCollisionGeometryVisible(bool visible, const QString& sourceId)
    {
        m_context.documentController().mutateProject(
            sourceId,
            ProjectDirtyPolicy::UserEdit,
            [visible](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                simulation_project::ProjectDocument& document = service.document();
                if(document.collision.visualization.showCollisionGeometry == visible) {
                    changed = false;
                    return true;
                }
                document.collision.visualization.showCollisionGeometry = visible;
                changed = true;
                return true;
            });
    }

    bool RobotQtViewerAppController::hasRobot(const QString& robotId) const
    {
        if(robotId.isEmpty()) {
            return false;
        }

        const std::string robotIdValue = robotId.toStdString();
        const std::vector<simulation_project::RobotDesc>& robots = m_context.document().robots;
        return std::any_of(
            robots.begin(),
            robots.end(),
            [&](const simulation_project::RobotDesc& robot) {
                return robot.id == robotIdValue;
            });
    }

    void RobotQtViewerAppController::setObjectInspectorContext(const QString& objectId)
    {
        m_inspectorContext.setObject(objectId);
    }

    void RobotQtViewerAppController::setToolAssetInspectorContext(const QString& assetId)
    {
        m_inspectorContext.setToolAsset(assetId);
    }

    void RobotQtViewerAppController::setRobotLinkInspectorContext(
        const QString& robotId,
        const QString& linkName)
    {
        m_selectedRobotId = robotId;
        m_selectedLinkName = linkName;
        m_inspectorContext.setRobotLink(robotId, linkName);
    }

    void RobotQtViewerAppController::setRobotMountInspectorContext(
        const QString& robotId,
        const QString& linkName,
        const QString& mountId)
    {
        m_selectedRobotId = robotId;
        m_selectedLinkName = linkName;
        m_inspectorContext.setRobotMount(robotId, linkName, mountId);
    }

    void RobotQtViewerAppController::setToolAttachmentInspectorContext(
        const QString& robotId,
        const QString& linkName,
        const QString& mountId,
        const QString& attachmentId)
    {
        m_selectedRobotId = robotId;
        m_selectedLinkName = linkName;
        m_inspectorContext.setToolAttachment(robotId, linkName, mountId, attachmentId);
    }

    void RobotQtViewerAppController::clearInspectorSelectionContext()
    {
        m_inspectorContext.clearSelection();
        m_selectedRobotId.clear();
        m_selectedLinkName.clear();
    }

    void RobotQtViewerAppController::setActiveCollisionDetectorContext(const QString& detectorId)
    {
        m_inspectorContext.setActiveCollisionDetectorId(detectorId);
    }

    void RobotQtViewerAppController::setMarkedCollisionPairAContext(
        const QString& robotId,
        const QString& linkName)
    {
        m_collisionPairRobotA = robotId;
        m_collisionPairLinkA = linkName;
        m_inspectorContext.setMarkedCollisionPairA(robotId, linkName);
    }

    bool RobotQtViewerAppController::setJointControlRobotContext(const QString& robotId)
    {
        if(robotId == m_selectedRobotId) {
            return false;
        }

        m_selectedRobotId = robotId;
        m_inspectorContext.setRobotLink(m_selectedRobotId, m_selectedLinkName);
        return true;
    }

    ProjectSessionWorkflowController& RobotQtViewerAppController::projectWorkflow()
    {
        return m_projectWorkflow;
    }

    ViewportReloadWorkflowController& RobotQtViewerAppController::viewportReloadWorkflow()
    {
        return m_viewportReloadWorkflow;
    }

    SceneEntityWorkflowController& RobotQtViewerAppController::sceneEntityWorkflow()
    {
        return m_sceneEntityWorkflow;
    }

    InspectorContext& RobotQtViewerAppController::inspectorContext()
    {
        return m_inspectorContext;
    }

    const InspectorContext& RobotQtViewerAppController::inspectorContext() const
    {
        return m_inspectorContext;
    }

    QString& RobotQtViewerAppController::selectedRobotId()
    {
        return m_selectedRobotId;
    }

    const QString& RobotQtViewerAppController::selectedRobotId() const
    {
        return m_selectedRobotId;
    }

    QString& RobotQtViewerAppController::selectedLinkName()
    {
        return m_selectedLinkName;
    }

    const QString& RobotQtViewerAppController::selectedLinkName() const
    {
        return m_selectedLinkName;
    }

    QString& RobotQtViewerAppController::collisionPairRobotA()
    {
        return m_collisionPairRobotA;
    }

    const QString& RobotQtViewerAppController::collisionPairRobotA() const
    {
        return m_collisionPairRobotA;
    }

    QString& RobotQtViewerAppController::collisionPairLinkA()
    {
        return m_collisionPairLinkA;
    }

    const QString& RobotQtViewerAppController::collisionPairLinkA() const
    {
        return m_collisionPairLinkA;
    }
}
