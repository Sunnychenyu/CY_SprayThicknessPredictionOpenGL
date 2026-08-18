#include "RobotQtViewerDocumentController.h"

#include <SimulationProject/ProjectDocumentService.h>

#include <utility>

namespace robot_qt_viewer
{
    RobotQtViewerDocumentController::RobotQtViewerDocumentController(
        simulation_project::ProjectSession& session,
        RobotQtViewerEventHub& eventHub,
        QObject* parent)
        : QObject(parent)
        , m_session(session)
        , m_eventHub(eventHub)
    {
    }

    simulation_project::ProjectSession& RobotQtViewerDocumentController::session()
    {
        return m_session;
    }

    const simulation_project::ProjectSession& RobotQtViewerDocumentController::session() const
    {
        return m_session;
    }

    void RobotQtViewerDocumentController::publishProjectOpened(const QString& sourceId)
    {
        m_eventHub.publish(makeEvent(RobotQtViewerEventKind::ProjectOpened, sourceId));
    }

    void RobotQtViewerDocumentController::publishProjectSaved(const QString& sourceId)
    {
        m_eventHub.publish(makeEvent(RobotQtViewerEventKind::ProjectSaved, sourceId));
        publishDirtyChanged(sourceId);
    }

    void RobotQtViewerDocumentController::publishDocumentChanged(const QString& sourceId, bool markDirty)
    {
        if(markDirty) {
            m_session.markDirty();
        }
        m_eventHub.publish(makeEvent(RobotQtViewerEventKind::ProjectDocumentChanged, sourceId));
        publishDirtyChanged(sourceId);
    }

    ProjectMutationResult RobotQtViewerDocumentController::mutateProject(
        const QString& sourceId,
        ProjectDirtyPolicy dirtyPolicy,
        const ProjectMutation& mutation)
    {
        ProjectMutationResult result;
        if(!mutation) {
            result.message = QStringLiteral("Project mutation is empty.");
            return result;
        }

        const bool previousDirty = m_session.isDirty();
        const simulation_project::ProjectDocument previousDocument = m_session.document();
        simulation_project::ProjectDocumentService service(m_session.document());
        bool changed = false;
        std::string error;
        if(!mutation(service, changed, error)) {
            const std::filesystem::path path = m_session.path();
            const bool requiresSaveAs = m_session.requiresSaveAs();
            m_session.setDocument(previousDocument, path, previousDirty, requiresSaveAs);
            publishDirtyChanged(sourceId);
            result.message = error.empty()
                ? QStringLiteral("Project mutation failed.")
                : QString::fromStdString(error);
            return result;
        }

        result.success = true;
        result.changed = changed;
        if(!changed) {
            if(m_session.isDirty() != previousDirty &&
                dirtyPolicy != ProjectDirtyPolicy::UserEdit) {
                m_session.setDirty(previousDirty);
                publishDirtyChanged(sourceId);
            }
            return result;
        }

        if(dirtyPolicy == ProjectDirtyPolicy::UserEdit) {
            m_session.markDirty();
        } else if(m_session.isDirty() != previousDirty) {
            m_session.setDirty(previousDirty);
        }

        if(dirtyPolicy != ProjectDirtyPolicy::PreviewOnly &&
            dirtyPolicy != ProjectDirtyPolicy::RuntimeOnly) {
            m_eventHub.publish(makeEvent(RobotQtViewerEventKind::ProjectDocumentChanged, sourceId));
        }
        if(m_session.isDirty() != previousDirty ||
            dirtyPolicy == ProjectDirtyPolicy::UserEdit ||
            dirtyPolicy == ProjectDirtyPolicy::LoadNormalization) {
            publishDirtyChanged(sourceId);
        }
        return result;
    }

    void RobotQtViewerDocumentController::restoreProjectSnapshot(
        const QString& sourceId,
        simulation_project::ProjectDocument document,
        bool dirty,
        bool publishDocumentChanged)
    {
        const std::filesystem::path path = m_session.path();
        const bool requiresSaveAs = m_session.requiresSaveAs();
        const bool previousDirty = m_session.isDirty();
        m_session.setDocument(std::move(document), path, dirty, requiresSaveAs);
        if(publishDocumentChanged) {
            m_eventHub.publish(makeEvent(RobotQtViewerEventKind::ProjectDocumentChanged, sourceId));
        }
        if(previousDirty != dirty || publishDocumentChanged) {
            publishDirtyChanged(sourceId);
        }
    }

    void RobotQtViewerDocumentController::publishDirtyChanged(const QString& sourceId)
    {
        m_eventHub.publish(makeEvent(RobotQtViewerEventKind::ProjectDirtyChanged, sourceId));
    }

    void RobotQtViewerDocumentController::publishViewportReloadRequested(const QString& sourceId)
    {
        RobotQtViewerEvent event = makeEvent(RobotQtViewerEventKind::ViewportReloadRequested, sourceId);
        event.viewportReloadRequested = true;
        event.viewport.reloadRequested = true;
        m_eventHub.publish(event);
    }

    void RobotQtViewerDocumentController::publishViewportReloaded(bool succeeded, const QString& sourceId)
    {
        RobotQtViewerEvent event = makeEvent(RobotQtViewerEventKind::ViewportReloaded, sourceId);
        event.viewport.reloadSucceeded = succeeded;
        m_eventHub.publish(event);
    }

    void RobotQtViewerDocumentController::publishRobotRuntimeChanged(const QString& sourceId)
    {
        m_eventHub.publish(makeEvent(RobotQtViewerEventKind::RobotRuntimeChanged, sourceId));
    }

    void RobotQtViewerDocumentController::publishAttachmentChanged(
        const RobotQtViewerAttachmentPayload& attachment,
        const QString& sourceId)
    {
        RobotQtViewerEvent event = makeEvent(RobotQtViewerEventKind::AttachmentChanged, sourceId);
        event.attachment = attachment;
        m_eventHub.publish(event);
    }

    void RobotQtViewerDocumentController::publishCollisionChanged(
        const RobotQtViewerCollisionPayload& collision,
        const QString& sourceId)
    {
        RobotQtViewerEvent event = makeEvent(RobotQtViewerEventKind::CollisionChanged, sourceId);
        event.collision = collision;
        m_eventHub.publish(event);
    }

    void RobotQtViewerDocumentController::publishCoatingAnalysisChanged(
        const RobotQtViewerCoatingAnalysisPayload& coatingAnalysis,
        const QString& sourceId)
    {
        RobotQtViewerEvent event = makeEvent(RobotQtViewerEventKind::CoatingAnalysisChanged, sourceId);
        event.coatingAnalysis = coatingAnalysis;
        m_eventHub.publish(event);
    }

    void RobotQtViewerDocumentController::publishSelectionChanged(
        const RobotQtViewerSelectionPayload& selection,
        const QString& sourceId)
    {
        RobotQtViewerEvent event = makeEvent(RobotQtViewerEventKind::SelectionChanged, sourceId);
        event.selection = selection;
        m_eventHub.publish(event);
    }

    void RobotQtViewerDocumentController::publishStatusMessage(
        const QString& message,
        int timeoutMs,
        const QString& sourceId)
    {
        RobotQtViewerEvent event = makeEvent(RobotQtViewerEventKind::StatusMessageRequested, sourceId);
        event.message = message;
        event.timeoutMs = timeoutMs;
        m_eventHub.publish(event);
    }

    RobotQtViewerEvent RobotQtViewerDocumentController::makeEvent(
        RobotQtViewerEventKind kind,
        const QString& sourceId) const
    {
        RobotQtViewerEvent event;
        event.kind = kind;
        event.sourceId = sourceId;
        event.projectChanged = m_session.isDirty();
        event.projectDirty = m_session.isDirty();
        return event;
    }
}
