#pragma once

#include "ProjectSessionWorkflowController.h"
#include "RobotQtViewerDocumentController.h"

#include <QString>

#include <filesystem>
#include <string>

namespace simulation_project
{
    struct ProjectDocument;
    struct RobotCollisionOverrideDesc;
    class ProjectSession;
}

namespace robot_qt_viewer
{
    class CollisionWorkbenchServices
    {
    public:
        virtual ~CollisionWorkbenchServices() = default;

        virtual const simulation_project::ProjectDocument& document() const = 0;
        virtual const simulation_project::ProjectSession& session() const = 0;
        virtual ProjectMutationResult mutateProject(
            const QString& sourceId,
            ProjectDirtyPolicy dirtyPolicy,
            const ProjectMutation& mutation) = 0;

        virtual const QString& selectedRobotId() const = 0;
        virtual const QString& selectedLinkName() const = 0;
        virtual const QString& selectedObjectId() const = 0;
        virtual const QString& selectedToolAttachmentId() const = 0;
        virtual const QString& collisionPairRobotA() const = 0;
        virtual const QString& collisionPairLinkA() const = 0;
        virtual void setActiveCollisionDetectorContext(const QString& detectorId) = 0;
        virtual void setMarkedCollisionPairAContext(
            const QString& robotId,
            const QString& linkName) = 0;

        virtual ProjectSessionWorkflowResult saveProject(
            const std::filesystem::path& path,
            bool saveAsV3,
            const QString& sourceId) = 0;
        virtual void reloadViewport(const QString& sourceId) = 0;
        virtual bool refreshViewportCollisionConfiguration(const QString& sourceId) = 0;
        virtual bool saveRobotCollisionOverride(
            const std::filesystem::path& sidecarPath,
            const simulation_project::RobotCollisionOverrideDesc& collisionOverride,
            std::string* error) = 0;
        virtual bool exportRobotCollisionOverrideToUrdf(
            const std::filesystem::path& sourceUrdfPath,
            const std::filesystem::path& outputUrdfPath,
            const simulation_project::RobotCollisionOverrideDesc& collisionOverride,
            std::string* error) = 0;
    };
}
