#include "RobotQtViewerDocumentContext.h"

#include "RobotQtViewerDocumentController.h"
#include "RobotQtViewerEventHub.h"
#include "RobotQtViewerSelectionModel.h"
#include "RobotQtViewerViewportPreviewState.h"

namespace robot_qt_viewer
{
    RobotQtViewerDocumentContext::RobotQtViewerDocumentContext(
        simulation_project::ProjectSession& session,
        RobotQtViewerDocumentController& documentController,
        RobotQtViewerSelectionModel& selectionModel,
        RobotQtViewerViewportPreviewState& viewportPreviewState,
        RobotQtViewerEventHub& eventHub)
        : m_session(session)
        , m_documentController(documentController)
        , m_selectionModel(selectionModel)
        , m_viewportPreviewState(viewportPreviewState)
        , m_eventHub(eventHub)
    {
    }

    simulation_project::ProjectSession& RobotQtViewerDocumentContext::projectSession()
    {
        return m_session;
    }

    const simulation_project::ProjectSession& RobotQtViewerDocumentContext::projectSession() const
    {
        return m_session;
    }

    const simulation_project::ProjectDocument& RobotQtViewerDocumentContext::document() const
    {
        return m_session.document();
    }

    RobotQtViewerDocumentController& RobotQtViewerDocumentContext::documentController()
    {
        return m_documentController;
    }

    const RobotQtViewerDocumentController& RobotQtViewerDocumentContext::documentController() const
    {
        return m_documentController;
    }

    RobotQtViewerSelectionModel& RobotQtViewerDocumentContext::selectionModel()
    {
        return m_selectionModel;
    }

    const RobotQtViewerSelectionModel& RobotQtViewerDocumentContext::selectionModel() const
    {
        return m_selectionModel;
    }

    RobotQtViewerViewportPreviewState& RobotQtViewerDocumentContext::viewportPreviewState()
    {
        return m_viewportPreviewState;
    }

    const RobotQtViewerViewportPreviewState& RobotQtViewerDocumentContext::viewportPreviewState() const
    {
        return m_viewportPreviewState;
    }

    RobotQtViewerEventHub& RobotQtViewerDocumentContext::eventHub()
    {
        return m_eventHub;
    }

    const RobotQtViewerEventHub& RobotQtViewerDocumentContext::eventHub() const
    {
        return m_eventHub;
    }

    void RobotQtViewerDocumentContext::setViewportServices(RobotQtViewerViewportServices* viewportServices)
    {
        m_viewportServices = viewportServices;
    }

    RobotQtViewerViewportServices* RobotQtViewerDocumentContext::viewportServices() const
    {
        return m_viewportServices;
    }
}
