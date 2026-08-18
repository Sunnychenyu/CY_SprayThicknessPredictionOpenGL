#pragma once

#include "CollisionWorkbenchServices.h"

namespace robot_qt_viewer
{
    class RobotQtViewerAppController;

    class RobotQtViewerCollisionWorkbenchServicesAdapter : public CollisionWorkbenchServices
    {
    public:
        explicit RobotQtViewerCollisionWorkbenchServicesAdapter(RobotQtViewerAppController& appController);

        const simulation_project::ProjectDocument& document() const override;
        const simulation_project::ProjectSession& session() const override;
        ProjectMutationResult mutateProject(
            const QString& sourceId,
            ProjectDirtyPolicy dirtyPolicy,
            const ProjectMutation& mutation) override;

        const QString& selectedRobotId() const override;
        const QString& selectedLinkName() const override;
        const QString& selectedObjectId() const override;
        const QString& selectedToolAttachmentId() const override;
        const QString& collisionPairRobotA() const override;
        const QString& collisionPairLinkA() const override;
        void setActiveCollisionDetectorContext(const QString& detectorId) override;
        void setMarkedCollisionPairAContext(
            const QString& robotId,
            const QString& linkName) override;

        ProjectSessionWorkflowResult saveProject(
            const std::filesystem::path& path,
            bool saveAsV3,
            const QString& sourceId) override;
        void reloadViewport(const QString& sourceId) override;
        bool refreshViewportCollisionConfiguration(const QString& sourceId) override;
        bool saveRobotCollisionOverride(
            const std::filesystem::path& sidecarPath,
            const simulation_project::RobotCollisionOverrideDesc& collisionOverride,
            std::string* error) override;
        bool exportRobotCollisionOverrideToUrdf(
            const std::filesystem::path& sourceUrdfPath,
            const std::filesystem::path& outputUrdfPath,
            const simulation_project::RobotCollisionOverrideDesc& collisionOverride,
            std::string* error) override;

    private:
        RobotQtViewerAppController& m_appController;
    };
}
