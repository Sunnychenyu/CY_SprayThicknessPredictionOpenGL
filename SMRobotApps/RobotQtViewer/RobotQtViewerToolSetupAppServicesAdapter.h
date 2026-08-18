#pragma once

#include "ToolSetupAppServices.h"

namespace robot_qt_viewer
{
    class RobotQtViewerAppController;

    class RobotQtViewerToolSetupAppServicesAdapter final : public ToolSetupAppServices
    {
    public:
        explicit RobotQtViewerToolSetupAppServicesAdapter(RobotQtViewerAppController& appController);

        QString selectedRobotId() const override;
        QString selectedLinkName() const override;
        void setRobotMountContext(
            const QString& robotId,
            const QString& linkName,
            const QString& mountId) override;
        void setToolAttachmentContext(
            const QString& robotId,
            const QString& linkName,
            const QString& mountId,
            const QString& attachmentId) override;
        ToolSetupViewportReloadResult reloadViewport(const QString& sourceId) override;

    private:
        RobotQtViewerAppController& m_appController;
    };
}
