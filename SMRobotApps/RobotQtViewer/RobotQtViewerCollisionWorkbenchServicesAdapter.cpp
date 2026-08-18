#include "RobotQtViewerCollisionWorkbenchServicesAdapter.h"

#include "RobotQtViewerAppController.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerViewportServices.h"

#include <RobotIO/RobotCollisionOverrideIo.h>
#include <RobotIO/RobotUrdfCollisionExporter.h>
#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/RuntimePaths.h>

#include <data_path.h>

namespace robot_qt_viewer
{
    namespace
    {
        robotio::Vec3Desc toRobotIoVec3(const simulation_project::Vec3Desc& desc)
        {
            return robotio::Vec3Desc{ desc.x, desc.y, desc.z };
        }

        robotio::TransformDesc toRobotIoTransform(const simulation_project::TransformDesc& desc)
        {
            robotio::TransformDesc result;
            result.x = desc.x;
            result.y = desc.y;
            result.z = desc.z;
            result.roll = desc.roll;
            result.pitch = desc.pitch;
            result.yaw = desc.yaw;
            return result;
        }

        robotio::CollisionElementOverrideDesc toRobotIoCollisionElement(
            const simulation_project::CollisionElementOverrideDesc& desc)
        {
            robotio::CollisionElementOverrideDesc result;
            result.id = desc.id;
            result.linkName = desc.linkName;
            result.label = desc.label;
            result.type = desc.type;
            result.role = desc.role;
            result.enabled = desc.enabled;
            result.localTransform = toRobotIoTransform(desc.localTransform);
            result.boxSize = toRobotIoVec3(desc.boxSize);
            result.radius = desc.radius;
            result.length = desc.length;
            result.meshPath = desc.meshPath;
            result.meshScale = toRobotIoVec3(desc.meshScale);
            result.inflationMargin = desc.inflationMargin;
            result.source = desc.source;
            return result;
        }

        robotio::RobotCollisionOverrideDesc toRobotIoCollisionOverride(
            const simulation_project::RobotCollisionOverrideDesc& desc)
        {
            robotio::RobotCollisionOverrideDesc result;
            result.robotId = desc.robotId;
            result.sourceRobotPath = desc.sourceRobotPath;
            result.overridePath = desc.overridePath;
            result.replaceOriginalCollisions = desc.replaceOriginalCollisions;
            result.elements.reserve(desc.elements.size());
            for(const simulation_project::CollisionElementOverrideDesc& element : desc.elements) {
                result.elements.push_back(toRobotIoCollisionElement(element));
            }
            return result;
        }
    }

    RobotQtViewerCollisionWorkbenchServicesAdapter::RobotQtViewerCollisionWorkbenchServicesAdapter(
        RobotQtViewerAppController& appController)
        : m_appController(appController)
    {
    }

    const simulation_project::ProjectDocument& RobotQtViewerCollisionWorkbenchServicesAdapter::document() const
    {
        return m_appController.document();
    }

    const simulation_project::ProjectSession& RobotQtViewerCollisionWorkbenchServicesAdapter::session() const
    {
        return m_appController.session();
    }

    ProjectMutationResult RobotQtViewerCollisionWorkbenchServicesAdapter::mutateProject(
        const QString& sourceId,
        ProjectDirtyPolicy dirtyPolicy,
        const ProjectMutation& mutation)
    {
        return m_appController.documentContext().documentController().mutateProject(
            sourceId,
            dirtyPolicy,
            mutation);
    }

    const QString& RobotQtViewerCollisionWorkbenchServicesAdapter::selectedRobotId() const
    {
        return m_appController.selectedRobotId();
    }

    const QString& RobotQtViewerCollisionWorkbenchServicesAdapter::selectedLinkName() const
    {
        return m_appController.selectedLinkName();
    }

    const QString& RobotQtViewerCollisionWorkbenchServicesAdapter::selectedObjectId() const
    {
        return m_appController.inspectorContext().selectedObjectId();
    }

    const QString& RobotQtViewerCollisionWorkbenchServicesAdapter::selectedToolAttachmentId() const
    {
        return m_appController.inspectorContext().selectedToolAttachmentId();
    }

    const QString& RobotQtViewerCollisionWorkbenchServicesAdapter::collisionPairRobotA() const
    {
        return m_appController.collisionPairRobotA();
    }

    const QString& RobotQtViewerCollisionWorkbenchServicesAdapter::collisionPairLinkA() const
    {
        return m_appController.collisionPairLinkA();
    }

    void RobotQtViewerCollisionWorkbenchServicesAdapter::setActiveCollisionDetectorContext(
        const QString& detectorId)
    {
        m_appController.setActiveCollisionDetectorContext(detectorId);
    }

    void RobotQtViewerCollisionWorkbenchServicesAdapter::setMarkedCollisionPairAContext(
        const QString& robotId,
        const QString& linkName)
    {
        m_appController.setMarkedCollisionPairAContext(robotId, linkName);
    }

    ProjectSessionWorkflowResult RobotQtViewerCollisionWorkbenchServicesAdapter::saveProject(
        const std::filesystem::path& path,
        bool saveAsV3,
        const QString& sourceId)
    {
        return m_appController.saveProject(path, saveAsV3, sourceId);
    }

    void RobotQtViewerCollisionWorkbenchServicesAdapter::reloadViewport(const QString& sourceId)
    {
        m_appController.reloadViewport(sourceId);
    }

    bool RobotQtViewerCollisionWorkbenchServicesAdapter::refreshViewportCollisionConfiguration(const QString& sourceId)
    {
        (void)sourceId;
        RobotQtViewerViewportServices* viewportServices =
            m_appController.documentContext().viewportServices();
        if(viewportServices == nullptr) {
            return false;
        }

        std::filesystem::path basePath = simulation_project::RuntimePaths::applicationRoot();
        if(!m_appController.session().path().empty()) {
            basePath = m_appController.session().path().parent_path();
        }
        return viewportServices->refreshCollisionConfiguration(m_appController.document(), basePath);
    }

    bool RobotQtViewerCollisionWorkbenchServicesAdapter::saveRobotCollisionOverride(
        const std::filesystem::path& sidecarPath,
        const simulation_project::RobotCollisionOverrideDesc& collisionOverride,
        std::string* error)
    {
        return robotio::saveRobotCollisionOverride(
            sidecarPath,
            toRobotIoCollisionOverride(collisionOverride),
            error);
    }

    bool RobotQtViewerCollisionWorkbenchServicesAdapter::exportRobotCollisionOverrideToUrdf(
        const std::filesystem::path& sourceUrdfPath,
        const std::filesystem::path& outputUrdfPath,
        const simulation_project::RobotCollisionOverrideDesc& collisionOverride,
        std::string* error)
    {
        return robotio::exportRobotCollisionOverrideToUrdf(
            sourceUrdfPath,
            outputUrdfPath,
            toRobotIoCollisionOverride(collisionOverride),
            error);
    }
}
