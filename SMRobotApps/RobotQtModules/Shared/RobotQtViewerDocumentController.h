#pragma once

#include "RobotQtViewerEventHub.h"
#include "RobotQtViewerEvents.h"

#include <SimulationProject/ProjectSession.h>

#include <QObject>
#include <QString>

#include <functional>
#include <string>

namespace simulation_project
{
    class ProjectDocumentService;
}

namespace robot_qt_viewer
{
    enum class ProjectDirtyPolicy
    {
        UserEdit,
        LoadNormalization,
        PreviewOnly,
        RuntimeOnly
    };

    struct ProjectMutationResult
    {
        bool success = false;
        bool changed = false;
        QString message;
    };

    using ProjectMutation =
        std::function<bool(simulation_project::ProjectDocumentService&, bool& changed, std::string& errorMessage)>;

    class RobotQtViewerDocumentController : public QObject
    {
    public:
        RobotQtViewerDocumentController(
            simulation_project::ProjectSession& session,
            RobotQtViewerEventHub& eventHub,
            QObject* parent = nullptr);

        simulation_project::ProjectSession& session();
        const simulation_project::ProjectSession& session() const;

        void publishProjectOpened(const QString& sourceId = QString());
        void publishProjectSaved(const QString& sourceId = QString());
        void publishDocumentChanged(const QString& sourceId = QString(), bool markDirty = true);
        ProjectMutationResult mutateProject(
            const QString& sourceId,
            ProjectDirtyPolicy dirtyPolicy,
            const ProjectMutation& mutation);
        void restoreProjectSnapshot(
            const QString& sourceId,
            simulation_project::ProjectDocument document,
            bool dirty,
            bool publishDocumentChanged = true);
        void publishDirtyChanged(const QString& sourceId = QString());
        void publishViewportReloadRequested(const QString& sourceId = QString());
        void publishViewportReloaded(bool succeeded, const QString& sourceId = QString());
        void publishRobotRuntimeChanged(const QString& sourceId = QString());
        void publishAttachmentChanged(
            const RobotQtViewerAttachmentPayload& attachment,
            const QString& sourceId = QString());
        void publishCollisionChanged(
            const RobotQtViewerCollisionPayload& collision,
            const QString& sourceId = QString());
        void publishCoatingAnalysisChanged(
            const RobotQtViewerCoatingAnalysisPayload& coatingAnalysis,
            const QString& sourceId = QString());
        void publishSelectionChanged(
            const RobotQtViewerSelectionPayload& selection,
            const QString& sourceId = QString());
        void publishStatusMessage(
            const QString& message,
            int timeoutMs = 0,
            const QString& sourceId = QString());

    private:
        RobotQtViewerEvent makeEvent(RobotQtViewerEventKind kind, const QString& sourceId) const;

        simulation_project::ProjectSession& m_session;
        RobotQtViewerEventHub& m_eventHub;
    };
}
