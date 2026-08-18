#include "CollisionPairWorkflowController.h"

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectDocumentService.h>

#include <algorithm>
#include <string>
#include <utility>

namespace robot_qt_viewer
{
    bool CollisionPairWorkflowController::syncRobotObjectPairs(
        simulation_project::ProjectDocument& document) const
    {
        bool changed = false;
        auto& pairs = document.collision.query.robotObjectPairs;

        pairs.erase(
            std::remove_if(
                pairs.begin(),
                pairs.end(),
                [&](const simulation_project::RobotObjectCollisionPairDesc& pair) {
                    const bool robotExists = std::any_of(
                        document.robots.begin(),
                        document.robots.end(),
                        [&](const simulation_project::RobotDesc& robot) {
                            return robot.id == pair.robotId;
                        });
                    const bool objectExists = std::any_of(
                        document.objects.begin(),
                        document.objects.end(),
                        [&](const simulation_project::SceneObjectDesc& object) {
                            return object.id == pair.objectId;
                        });
                    const bool removePair = !robotExists || !objectExists;
                    changed = changed || removePair;
                    return removePair;
                }),
            pairs.end());

        for(const simulation_project::RobotDesc& robot : document.robots) {
            for(const simulation_project::SceneObjectDesc& object : document.objects) {
                const bool exists = std::any_of(
                    pairs.begin(),
                    pairs.end(),
                    [&](const simulation_project::RobotObjectCollisionPairDesc& pair) {
                        return pair.robotId == robot.id && pair.objectId == object.id;
                    });
                if(exists) {
                    continue;
                }

                simulation_project::RobotObjectCollisionPairDesc pair;
                pair.robotId = robot.id;
                pair.objectId = object.id;
                pair.enabled = true;
                pairs.push_back(std::move(pair));
                changed = true;
            }
        }
        return changed;
    }

    CollisionPairWorkflowResult CollisionPairWorkflowController::setAllRobotObjectPairsEnabled(
        simulation_project::ProjectDocument& document,
        bool enabled) const
    {
        CollisionPairWorkflowResult result;
        simulation_project::ProjectDocumentService service(document);
        service.setAllRobotObjectCollisionPairsEnabled(enabled);
        result.success = true;
        result.message = enabled
            ? QStringLiteral("Robot-object collision pairs enabled.")
            : QStringLiteral("Robot-object collision pairs disabled.");
        return result;
    }

    CollisionPairWorkflowResult CollisionPairWorkflowController::setRobotObjectPairEnabled(
        simulation_project::ProjectDocument& document,
        const QString& robotId,
        const QString& objectId,
        bool enabled) const
    {
        CollisionPairWorkflowResult result;
        simulation_project::ProjectDocumentService service(document);
        std::string error;
        if(!service.setRobotObjectCollisionPairEnabled(
               robotId.toStdString(),
               objectId.toStdString(),
               enabled,
               &error)) {
            result.message = QString("Collision pair update failed: %1").arg(QString::fromStdString(error));
            return result;
        }

        result.success = true;
        result.message = QString("Updated collision pair %1 <-> %2").arg(robotId, objectId);
        return result;
    }
}
