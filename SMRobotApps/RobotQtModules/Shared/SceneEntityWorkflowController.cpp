#include "SceneEntityWorkflowController.h"

#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectSceneEntityCommands.h>
#include <SimulationProject/ProjectSession.h>

#include <cmath>
#include <cctype>
#include <fstream>
#include <sstream>
#include <vector>

namespace
{
    struct PcdHeaderProbe
    {
        bool success = false;
        std::string error;
        std::string dataType;
        std::vector<std::string> fields;
    };

    std::string pathStemUtf8(const std::filesystem::path& path)
    {
        return path.stem().generic_u8string();
    }

    std::string lowercaseAscii(std::string value)
    {
        for(char& ch : value) {
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
        }
        return value;
    }

    std::vector<std::string> splitWhitespace(const std::string& line)
    {
        std::istringstream stream(line);
        std::vector<std::string> words;
        std::string word;
        while(stream >> word) {
            words.push_back(word);
        }
        return words;
    }

    PcdHeaderProbe probePcdHeader(const std::filesystem::path& path)
    {
        PcdHeaderProbe probe;
        std::ifstream file(path, std::ios::binary);
        if(!file) {
            probe.error = "Failed to open PCD file.";
            return probe;
        }

        std::string line;
        while(std::getline(file, line)) {
            if(line.empty() || line[0] == '#') {
                continue;
            }
            const std::vector<std::string> words = splitWhitespace(line);
            if(words.empty()) {
                continue;
            }
            const std::string key = words.front();
            if(key == "FIELDS") {
                probe.fields.assign(words.begin() + 1, words.end());
            } else if(key == "DATA" && words.size() > 1) {
                probe.dataType = lowercaseAscii(words[1]);
                break;
            }
        }

        if(probe.dataType.empty()) {
            probe.error = "PCD header is missing DATA.";
            return probe;
        }
        const auto hasField = [&](const std::string& name) {
            for(const std::string& field : probe.fields) {
                if(field == name) {
                    return true;
                }
            }
            return false;
        };
        if(!hasField("x") || !hasField("y") || !hasField("z")) {
            probe.error = "PCD file does not contain x/y/z fields.";
            return probe;
        }
        if(probe.dataType != "ascii" &&
            probe.dataType != "binary" &&
            probe.dataType != "binary_compressed") {
            probe.error = "Unsupported PCD DATA type: " + probe.dataType;
            return probe;
        }

        probe.success = true;
        return probe;
    }

    std::string sanitizeIdBase(std::string value, const std::string& fallback)
    {
        if(value.empty()) {
            value = fallback;
        }
        for(char& ch : value) {
            if(!std::isalnum(static_cast<unsigned char>(ch)) && ch != '_') {
                ch = '_';
            }
        }
        return value.empty() ? fallback : value;
    }

    std::string normalizedStoredPath(std::string value)
    {
        for(char& ch : value) {
            if(ch == '\\') {
                ch = '/';
            } else {
                ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            }
        }
        return value;
    }

    const simulation_project::PointCloudDesc* findPointCloudWithSourcePath(
        const simulation_project::ProjectDocument& document,
        const std::string& sourcePath)
    {
        const std::string normalizedSourcePath = normalizedStoredPath(sourcePath);
        for(const simulation_project::PointCloudDesc& pointCloud : document.pointClouds) {
            if(normalizedStoredPath(pointCloud.sourcePath) == normalizedSourcePath) {
                return &pointCloud;
            }
        }
        return nullptr;
    }

    bool nearlyEqual(double lhs, double rhs)
    {
        return std::abs(lhs - rhs) <= 1.0e-9;
    }

    bool sameTransform(
        const simulation_project::TransformDesc& lhs,
        const simulation_project::TransformDesc& rhs)
    {
        return nearlyEqual(lhs.x, rhs.x) &&
            nearlyEqual(lhs.y, rhs.y) &&
            nearlyEqual(lhs.z, rhs.z) &&
            nearlyEqual(lhs.roll, rhs.roll) &&
            nearlyEqual(lhs.pitch, rhs.pitch) &&
            nearlyEqual(lhs.yaw, rhs.yaw);
    }
}

namespace robot_qt_viewer
{
    SceneEntityWorkflowController::SceneEntityWorkflowController(RobotQtViewerDocumentContext& context)
        : m_context(context)
    {
    }

    SceneEntityImportResult SceneEntityWorkflowController::importRobotFromPath(
        const std::filesystem::path& path,
        const std::string& sourceType)
    {
        SceneEntityImportResult result;
        result.previousDocument = m_context.document();
        result.previousDirty = m_context.projectSession().isDirty();

        simulation_project::AddRobotEntityCommand command;
        command.displayName = pathStemUtf8(path);
        command.sourceType = sourceType;
        command.sourcePath = m_context.projectSession().makePortableAssetPath(path);
        simulation_project::ProjectSceneEntityCommandResult commandResult;
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("importRobot"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                simulation_project::ProjectSceneEntityCommands commands(service.document());
                commandResult = commands.addRobot(command);
                if(!commandResult.success) {
                    error = commandResult.message;
                    return false;
                }
                changed = true;
                return true;
            });
        result.success = mutationResult.success;
        result.message = mutationResult.success
            ? QString::fromStdString(commandResult.message)
            : mutationResult.message;
        result.entityId = QString::fromStdString(commandResult.entityId);
        result.storedPath = QString::fromStdString(commandResult.storedPath);
        return result;
    }

    SceneEntityImportResult SceneEntityWorkflowController::importSceneObjectFromPath(
        const std::filesystem::path& path,
        const std::string& objectType,
        double modelScale)
    {
        SceneEntityImportResult result;
        result.previousDocument = m_context.document();
        result.previousDirty = m_context.projectSession().isDirty();

        simulation_project::AddSceneObjectEntityCommand command;
        command.displayName = pathStemUtf8(path);
        command.objectType = objectType;
        command.sourcePath = m_context.projectSession().makePortableAssetPath(path);
        simulation_project::ProjectSceneEntityCommandResult commandResult;
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("importObject"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                simulation_project::ProjectSceneEntityCommands commands(service.document());
                commandResult = commands.addSceneObject(command);
                if(!commandResult.success) {
                    error = commandResult.message;
                    return false;
                }
                simulation_project::SceneObjectDesc* object =
                    service.findSceneObject(commandResult.entityId);
                if(object == nullptr) {
                    error = "Imported scene object is unavailable.";
                    return false;
                }
                object->visualScale = modelScale;
                object->collisionScale = modelScale;
                changed = true;
                return true;
            });
        result.success = mutationResult.success;
        result.message = mutationResult.success
            ? QString::fromStdString(commandResult.message)
            : mutationResult.message;
        result.entityId = QString::fromStdString(commandResult.entityId);
        result.storedPath = QString::fromStdString(commandResult.storedPath);
        return result;
    }

    SceneEntityImportResult SceneEntityWorkflowController::importPointCloudFromPath(
        const std::filesystem::path& path,
        const std::string& format)
    {
        SceneEntityImportResult result;
        result.previousDocument = m_context.document();
        result.previousDirty = m_context.projectSession().isDirty();

        if(!std::filesystem::exists(path)) {
            result.message = QString("Point cloud file not found: %1").arg(QString::fromStdWString(path.wstring()));
            return result;
        }
        const PcdHeaderProbe headerProbe = probePcdHeader(path);
        if(!headerProbe.success) {
            result.message = QString("Import point cloud failed: %1").arg(QString::fromStdString(headerProbe.error));
            return result;
        }

        const std::string displayName = pathStemUtf8(path);
        const std::string storedPath = m_context.projectSession().makePortableAssetPath(path);
        simulation_project::PointCloudDesc pointCloud;
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("importPointCloud"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                pointCloud.id = service.makeUniqueId(sanitizeIdBase(displayName, "point_cloud"));
                pointCloud.name = displayName.empty() ? pointCloud.id : displayName;
                pointCloud.sourcePath = storedPath;
                pointCloud.format = format.empty() ? "pcd" : format;
                pointCloud.scale = 1.0;
                if(const simulation_project::PointCloudDesc* existing =
                       findPointCloudWithSourcePath(service.document(), pointCloud.sourcePath)) {
                    pointCloud.visualization = existing->visualization;
                } else {
                    pointCloud.visualization.visible = true;
                    pointCloud.visualization.pointSize = 2.5;
                    pointCloud.visualization.maxRenderPoints = 300000;
                }
                pointCloud.collision.enabled = false;
                if(!service.addPointCloud(pointCloud, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        result.success = mutationResult.success;
        result.message = mutationResult.success
            ? QString("Imported point cloud %1").arg(QString::fromStdString(pointCloud.id))
            : mutationResult.message;
        result.entityId = QString::fromStdString(pointCloud.id);
        result.storedPath = QString::fromStdString(pointCloud.sourcePath);
        return result;
    }

    void SceneEntityWorkflowController::restoreImportState(const SceneEntityImportResult& result)
    {
        m_context.documentController().restoreProjectSnapshot(
            QStringLiteral("restoreImportState"),
            result.previousDocument,
            result.previousDirty);
    }

    SceneEntityDeleteResult SceneEntityWorkflowController::deleteEntity(
        SceneEntityKind kind,
        const QString& entityId)
    {
        SceneEntityDeleteResult result;
        result.kind = kind;
        result.entityId = entityId;
        if(entityId.isEmpty()) {
            result.message = QStringLiteral("No scene item selected.");
            return result;
        }

        simulation_project::ProjectReferenceCleanupReport report;
        bool removed = false;
        QString itemKind;
        if(kind == SceneEntityKind::Robot) {
            itemKind = QStringLiteral("robot");
        } else if(kind == SceneEntityKind::PointCloud) {
            itemKind = QStringLiteral("point cloud");
        } else {
            itemKind = QStringLiteral("object");
        }
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("deleteSceneEntity"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(kind == SceneEntityKind::Robot) {
                    removed = service.removeRobot(entityId.toStdString(), report, &error);
                } else if(kind == SceneEntityKind::PointCloud) {
                    removed = service.removePointCloud(entityId.toStdString(), report, &error);
                } else {
                    removed = service.removeObject(entityId.toStdString(), report, &error);
                }
                changed = removed;
                return removed;
            });
        if(!mutationResult.success) {
            result.message = mutationResult.message.isEmpty()
                ? QString("Selected %1 is not in project.").arg(itemKind)
                : mutationResult.message;
            return result;
        }

        result.success = true;
        result.removedCollisionDetectors = report.removedCollisionDetectors.size();
        result.removedTools = report.removedMountedAttachments.size();
        result.removedSensors = 0;
        result.message = QString("Deleted %1 %2; removed %3 detectors, %4 tools, %5 sensors")
            .arg(itemKind, entityId)
            .arg(static_cast<qulonglong>(result.removedCollisionDetectors))
            .arg(static_cast<qulonglong>(result.removedTools))
            .arg(static_cast<qulonglong>(result.removedSensors));
        return result;
    }

    SceneEntityMutationResult SceneEntityWorkflowController::setRobotBaseTransform(
        const QString& robotId,
        const simulation_project::TransformDesc& transform)
    {
        SceneEntityMutationResult result;
        result.entityId = robotId;
        if(robotId.isEmpty()) {
            result.message = QStringLiteral("No robot selected.");
            return result;
        }

        const simulation_project::ProjectDocument& document = m_context.document();
        simulation_project::ProjectDocumentService readService(document);
        const simulation_project::RobotDesc* robot = readService.findRobot(robotId.toStdString());
        if(robot != nullptr && sameTransform(robot->baseTransform, transform)) {
            result.success = true;
            result.message = QString("Robot base unchanged: %1").arg(robotId);
            return result;
        }
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("robotBaseTransform"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                const simulation_project::RobotDesc* current = service.findRobot(robotId.toStdString());
                if(current != nullptr && sameTransform(current->baseTransform, transform)) {
                    changed = false;
                    return true;
                }
                if(!service.setRobotBaseTransform(robotId.toStdString(), transform, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            result.message = QString("Move base failed: %1").arg(mutationResult.message);
            return result;
        }
        result.success = true;
        result.message = mutationResult.changed
            ? QString("Moved base of %1").arg(robotId)
            : QString("Robot base unchanged: %1").arg(robotId);
        return result;
    }

    SceneEntityMutationResult SceneEntityWorkflowController::setSceneObjectTransform(
        const QString& objectId,
        const simulation_project::TransformDesc& transform)
    {
        SceneEntityMutationResult result;
        result.entityId = objectId;
        if(objectId.isEmpty()) {
            result.message = QStringLiteral("No object selected.");
            return result;
        }

        const simulation_project::ProjectDocument& document = m_context.document();
        simulation_project::ProjectDocumentService readService(document);
        const simulation_project::SceneObjectDesc* object = readService.findSceneObject(objectId.toStdString());
        if(object != nullptr && sameTransform(object->transform, transform)) {
            result.success = true;
            result.message = QString("Object transform unchanged: %1").arg(objectId);
            return result;
        }
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("sceneObjectTransform"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                const simulation_project::SceneObjectDesc* current = service.findSceneObject(objectId.toStdString());
                if(current != nullptr && sameTransform(current->transform, transform)) {
                    changed = false;
                    return true;
                }
                if(!service.setSceneObjectTransform(objectId.toStdString(), transform, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            result.message = QString("Move object failed: %1").arg(mutationResult.message);
            return result;
        }

        result.success = true;
        result.message = mutationResult.changed
            ? QString("Moved object %1").arg(objectId)
            : QString("Object transform unchanged: %1").arg(objectId);
        return result;
    }

    SceneEntityMutationResult SceneEntityWorkflowController::setPointCloudTransform(
        const QString& pointCloudId,
        const simulation_project::TransformDesc& transform)
    {
        SceneEntityMutationResult result;
        result.entityId = pointCloudId;
        if(pointCloudId.isEmpty()) {
            result.message = QStringLiteral("No point cloud selected.");
            return result;
        }

        const simulation_project::ProjectDocument& document = m_context.document();
        simulation_project::ProjectDocumentService readService(document);
        const simulation_project::PointCloudDesc* pointCloud = readService.findPointCloud(pointCloudId.toStdString());
        if(pointCloud != nullptr && sameTransform(pointCloud->transform, transform)) {
            result.success = true;
            result.message = QString("Point cloud transform unchanged: %1").arg(pointCloudId);
            return result;
        }
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("pointCloudTransform"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                const simulation_project::PointCloudDesc* current = service.findPointCloud(pointCloudId.toStdString());
                if(current != nullptr && sameTransform(current->transform, transform)) {
                    changed = false;
                    return true;
                }
                if(!service.setPointCloudTransform(pointCloudId.toStdString(), transform, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            result.message = QString("Move point cloud failed: %1").arg(mutationResult.message);
            return result;
        }

        result.success = true;
        result.message = mutationResult.changed
            ? QString("Moved point cloud %1").arg(pointCloudId)
            : QString("Point cloud transform unchanged: %1").arg(pointCloudId);
        return result;
    }

    SceneEntityMutationResult SceneEntityWorkflowController::renamePointCloud(
        const QString& pointCloudId,
        const QString& name)
    {
        SceneEntityMutationResult result;
        result.entityId = pointCloudId;
        if(pointCloudId.isEmpty()) {
            result.message = QStringLiteral("No point cloud selected.");
            return result;
        }

        const std::string newName = name.toStdString();
        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("pointCloudRename"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                const simulation_project::PointCloudDesc* pointCloud =
                    service.findPointCloud(pointCloudId.toStdString());
                if(pointCloud != nullptr &&
                    pointCloud->name == (newName.empty() ? pointCloud->id : newName)) {
                    changed = false;
                    return true;
                }
                if(!service.renamePointCloud(pointCloudId.toStdString(), newName, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            result.message = QString("Rename point cloud failed: %1").arg(mutationResult.message);
            return result;
        }

        result.success = true;
        result.message = mutationResult.changed
            ? QString("Renamed point cloud %1").arg(pointCloudId)
            : QString("Point cloud unchanged: %1").arg(pointCloudId);
        return result;
    }

    SceneEntityMutationResult SceneEntityWorkflowController::replaceRobotMountsForRobot(
        const QString& robotId,
        const std::vector<simulation_project::RobotMountDesc>& mounts)
    {
        SceneEntityMutationResult result;
        result.entityId = robotId;
        if(robotId.isEmpty()) {
            result.message = QStringLiteral("No robot selected.");
            return result;
        }

        const ProjectMutationResult mutationResult = m_context.documentController().mutateProject(
            QStringLiteral("robotFlangeConfig"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string& error) {
                if(!service.replaceRobotMountsForRobot(robotId.toStdString(), mounts, &error)) {
                    return false;
                }
                changed = true;
                return true;
            });
        if(!mutationResult.success) {
            result.message =
                QString("Configure robot flange failed: %1").arg(mutationResult.message);
            return result;
        }

        result.success = true;
        result.message = QString("Configured robot flange for %1").arg(robotId);
        return result;
    }

}
