#pragma once

#include <QString>

namespace robot_qt_viewer
{
    struct ToolSetupViewportReloadResult
    {
        bool success = false;
        QString errorMessage;
    };

    class ToolSetupAppServices
    {
    public:
        virtual ~ToolSetupAppServices() = default;

        virtual QString selectedRobotId() const = 0;
        virtual QString selectedLinkName() const = 0;
        virtual void setRobotMountContext(
            const QString& robotId,
            const QString& linkName,
            const QString& mountId) = 0;
        virtual void setToolAttachmentContext(
            const QString& robotId,
            const QString& linkName,
            const QString& mountId,
            const QString& attachmentId) = 0;
        virtual ToolSetupViewportReloadResult reloadViewport(const QString& sourceId) = 0;
    };
}
