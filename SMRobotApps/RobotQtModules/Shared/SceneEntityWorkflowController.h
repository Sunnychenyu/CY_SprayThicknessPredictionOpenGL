#pragma once

#include <QString>

#include <filesystem>
#include <cstddef>
#include <string>
#include <vector>

#include <SimulationProject/ProjectDocument.h>

namespace robot_qt_viewer
{
    class RobotQtViewerDocumentContext;

    struct SceneEntityWorkflowResult
    {
        bool success = false;
        QString message;
        QString entityId;
        QString storedPath;
    };

    struct SceneEntityImportResult : SceneEntityWorkflowResult
    {
        simulation_project::ProjectDocument previousDocument;
        bool previousDirty = false;
    };

    enum class SceneEntityKind
    {
        Robot,
        Object,
        PointCloud
    };

    struct SceneEntityDeleteResult
    {
        bool success = false;
        QString message;
        QString entityId;
        SceneEntityKind kind = SceneEntityKind::Robot;
        std::size_t removedCollisionDetectors = 0;
        std::size_t removedTools = 0;
        std::size_t removedSensors = 0;
    };

    struct SceneEntityMutationResult
    {
        bool success = false;
        QString message;
        QString entityId;
    };

    class SceneEntityWorkflowController
    {
    public:
        explicit SceneEntityWorkflowController(RobotQtViewerDocumentContext& context);

        SceneEntityImportResult importRobotFromPath(
            const std::filesystem::path& path,
            const std::string& sourceType);
        SceneEntityImportResult importSceneObjectFromPath(
            const std::filesystem::path& path,
            const std::string& objectType);
        SceneEntityImportResult importPointCloudFromPath(
            const std::filesystem::path& path,
            const std::string& format);
        void restoreImportState(const SceneEntityImportResult& result);
        SceneEntityDeleteResult deleteEntity(SceneEntityKind kind, const QString& entityId);
        SceneEntityMutationResult setRobotBaseTransform(
            const QString& robotId,
            const simulation_project::TransformDesc& transform);
        SceneEntityMutationResult setSceneObjectTransform(
            const QString& objectId,
            const simulation_project::TransformDesc& transform);
        SceneEntityMutationResult setPointCloudTransform(
            const QString& pointCloudId,
            const simulation_project::TransformDesc& transform);
        SceneEntityMutationResult renamePointCloud(
            const QString& pointCloudId,
            const QString& name);
        SceneEntityMutationResult replaceRobotMountsForRobot(
            const QString& robotId,
            const std::vector<simulation_project::RobotMountDesc>& mounts);

    private:
        RobotQtViewerDocumentContext& m_context;
    };
}
