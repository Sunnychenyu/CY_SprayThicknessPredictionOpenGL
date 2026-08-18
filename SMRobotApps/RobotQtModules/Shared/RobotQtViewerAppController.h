#pragma once

#include "InspectorContext.h"
#include "ProjectSessionWorkflowController.h"
#include "SceneEntityWorkflowController.h"
#include "ViewportReloadWorkflowController.h"

namespace simulation_project
{
    struct ProjectDocument;
    class ProjectSession;
}

namespace robot_qt_viewer
{
    class RobotQtViewerDocumentContext;
    class RobotQtViewerDocumentController;
    class RobotQtViewerEventHub;
    class RobotQtViewerSelectionModel;

    class RobotQtViewerAppController
    {
    public:
        explicit RobotQtViewerAppController(RobotQtViewerDocumentContext& context);
        ~RobotQtViewerAppController();

        RobotQtViewerDocumentContext& documentContext();
        const RobotQtViewerDocumentContext& documentContext() const;
        simulation_project::ProjectSession& session();
        const simulation_project::ProjectSession& session() const;
        const simulation_project::ProjectDocument& document() const;
        RobotQtViewerDocumentController& documentController();
        RobotQtViewerSelectionModel& selectionModel();
        RobotQtViewerEventHub& eventHub();

        ProjectSessionWorkflowResult resetProject(const QString& sourceId);
        ProjectSessionWorkflowResult loadProject(
            const std::filesystem::path& path,
            const QString& sourceId);
        ProjectSessionWorkflowResult loadStartupProject(
            const std::filesystem::path& path,
            const QString& sourceId);
        ProjectSessionWorkflowResult saveProject(
            const std::filesystem::path& path,
            bool saveAsV3,
            const QString& sourceId);
        ViewportReloadWorkflowResult reloadViewport(const QString& sourceId);
        void setCollisionGeometryVisible(bool visible, const QString& sourceId);
        bool hasRobot(const QString& robotId) const;
        void setObjectInspectorContext(const QString& objectId);
        void setToolAssetInspectorContext(const QString& assetId);
        void setRobotLinkInspectorContext(
            const QString& robotId,
            const QString& linkName);
        void setRobotMountInspectorContext(
            const QString& robotId,
            const QString& linkName,
            const QString& mountId);
        void setToolAttachmentInspectorContext(
            const QString& robotId,
            const QString& linkName,
            const QString& mountId,
            const QString& attachmentId);
        void clearInspectorSelectionContext();
        void setActiveCollisionDetectorContext(const QString& detectorId);
        void setMarkedCollisionPairAContext(
            const QString& robotId,
            const QString& linkName);
        bool setJointControlRobotContext(const QString& robotId);

        ProjectSessionWorkflowController& projectWorkflow();
        ViewportReloadWorkflowController& viewportReloadWorkflow();
        SceneEntityWorkflowController& sceneEntityWorkflow();
        InspectorContext& inspectorContext();
        const InspectorContext& inspectorContext() const;
        QString& selectedRobotId();
        const QString& selectedRobotId() const;
        QString& selectedLinkName();
        const QString& selectedLinkName() const;
        QString& collisionPairRobotA();
        const QString& collisionPairRobotA() const;
        QString& collisionPairLinkA();
        const QString& collisionPairLinkA() const;

    private:
        RobotQtViewerDocumentContext& m_context;
        ProjectSessionWorkflowController m_projectWorkflow;
        ViewportReloadWorkflowController m_viewportReloadWorkflow;
        SceneEntityWorkflowController m_sceneEntityWorkflow;
        InspectorContext m_inspectorContext;
        QString m_selectedRobotId;
        QString m_selectedLinkName;
        QString m_collisionPairRobotA;
        QString m_collisionPairLinkA;
    };
}
