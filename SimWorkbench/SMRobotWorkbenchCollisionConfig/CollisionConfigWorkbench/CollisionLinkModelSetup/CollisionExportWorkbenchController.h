#pragma once

#include <QString>

#include <filesystem>
#include <functional>
#include <string>

namespace robot_qt_viewer
{
    class CollisionExportDocumentFacade;
    class CollisionWorkbenchServices;

    class CollisionExportWorkbenchController
    {
    public:
        struct Callbacks
        {
            std::function<void(const QString&, int)> statusMessage;
        };

        CollisionExportWorkbenchController(
            CollisionWorkbenchServices& appServices,
            CollisionExportDocumentFacade& documentFacade,
            Callbacks callbacks);

        void saveOverridesToProject();
        bool selectedRobotHasCollisionOverrides() const;
        std::filesystem::path selectedRobotSourcePath() const;
        void saveOverridesAsSidecar(
            const std::filesystem::path& sidecarPath,
            const std::string& portableSidecarPath);
        void exportRobotUrdfWithCollision(
            const std::filesystem::path& sourceUrdfPath,
            const std::filesystem::path& outputUrdfPath);

    private:
        void showStatus(const QString& message, int timeoutMs) const;

        CollisionWorkbenchServices& m_appServices;
        CollisionExportDocumentFacade& m_documentFacade;
        Callbacks m_callbacks;
    };
}
