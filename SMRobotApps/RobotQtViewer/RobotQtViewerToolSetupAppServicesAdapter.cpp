#include "RobotQtViewerToolSetupAppServicesAdapter.h"

#include "RobotQtViewerAppController.h"

namespace robot_qt_viewer
{
    RobotQtViewerToolSetupAppServicesAdapter::RobotQtViewerToolSetupAppServicesAdapter(
        RobotQtViewerAppController& appController)
        : m_appController(appController)
    {
    }

    QString RobotQtViewerToolSetupAppServicesAdapter::selectedRobotId() const
    {
        return m_appController.selectedRobotId();
    }

    QString RobotQtViewerToolSetupAppServicesAdapter::selectedLinkName() const
    {
        return m_appController.selectedLinkName();
    }

    void RobotQtViewerToolSetupAppServicesAdapter::setRobotMountContext(
        const QString& robotId,
        const QString& linkName,
        const QString& mountId)
    {
        m_appController.setRobotMountInspectorContext(robotId, linkName, mountId);
    }

    void RobotQtViewerToolSetupAppServicesAdapter::setToolAttachmentContext(
        const QString& robotId,
        const QString& linkName,
        const QString& mountId,
        const QString& attachmentId)
    {
        m_appController.setToolAttachmentInspectorContext(robotId, linkName, mountId, attachmentId);
    }

    ToolSetupViewportReloadResult RobotQtViewerToolSetupAppServicesAdapter::reloadViewport(const QString& sourceId)
    {
        const ViewportReloadWorkflowResult reloadResult = m_appController.reloadViewport(sourceId);
        ToolSetupViewportReloadResult result;
        result.success = reloadResult.success;
        result.errorMessage = reloadResult.errorMessage;
        return result;
    }
}
