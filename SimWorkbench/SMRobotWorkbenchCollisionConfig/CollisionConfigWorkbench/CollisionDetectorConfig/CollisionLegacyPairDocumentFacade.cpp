#include "CollisionLegacyPairDocumentFacade.h"

#include "CollisionWorkbenchServices.h"

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectSession.h>

#include <algorithm>
#include <utility>

namespace robot_qt_viewer
{
    CollisionLegacyPairDocumentFacade::CollisionLegacyPairDocumentFacade(
        const simulation_project::ProjectDocument& document,
        CollisionWorkbenchServices& appServices)
        : m_document(document)
        , m_appServices(appServices)
    {
    }

    QVector<CollisionLegacyPairItemView> CollisionLegacyPairDocumentFacade::buildPairItems() const
    {
        QVector<CollisionLegacyPairItemView> items;

        for(const simulation_project::RobotObjectCollisionPairDesc& pair : m_document.collision.query.robotObjectPairs) {
            const auto robotIt = std::find_if(
                m_document.robots.begin(),
                m_document.robots.end(),
                [&](const simulation_project::RobotDesc& robot) {
                    return robot.id == pair.robotId;
                });
            const auto objectIt = std::find_if(
                m_document.objects.begin(),
                m_document.objects.end(),
                [&](const simulation_project::SceneObjectDesc& object) {
                    return object.id == pair.objectId;
                });
            if(robotIt == m_document.robots.end() || objectIt == m_document.objects.end()) {
                continue;
            }

            const QString robotText = robotIt->name == robotIt->id
                ? QString::fromStdString(robotIt->id)
                : QString("%1 (%2)").arg(QString::fromStdString(robotIt->name), QString::fromStdString(robotIt->id));
            const QString objectText = objectIt->name == objectIt->id
                ? QString::fromStdString(objectIt->id)
                : QString("%1 (%2)").arg(QString::fromStdString(objectIt->name), QString::fromStdString(objectIt->id));

            CollisionLegacyPairItemView item;
            item.robotId = QString::fromStdString(pair.robotId);
            item.objectId = QString::fromStdString(pair.objectId);
            item.label = QString("%1 <-> %2").arg(robotText, objectText);
            item.enabled = pair.enabled;
            item.selectable = true;
            items.push_back(std::move(item));
        }

        if(items.empty()) {
            CollisionLegacyPairItemView emptyItem;
            emptyItem.label = "No robot-object collision pairs";
            emptyItem.selectable = false;
            items.push_back(std::move(emptyItem));
        }

        return items;
    }

    bool CollisionLegacyPairDocumentFacade::canAutoPairAll() const
    {
        return !m_document.robots.empty() && !m_document.objects.empty();
    }

    bool CollisionLegacyPairDocumentFacade::syncRobotObjectPairs()
    {
        bool changed = false;
        m_appServices.mutateProject(
            QStringLiteral("collisionLegacyPairs"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& mutationChanged, std::string&) {
                const CollisionPairWorkflowController workflow;
                changed = workflow.syncRobotObjectPairs(service.document());
                mutationChanged = changed;
                return true;
            });
        return changed;
    }

    CollisionPairWorkflowResult CollisionLegacyPairDocumentFacade::setAllRobotObjectPairsEnabled(bool enabled)
    {
        CollisionPairWorkflowResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionLegacyPairs"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                const CollisionPairWorkflowController workflow;
                result = workflow.setAllRobotObjectPairsEnabled(service.document(), enabled);
                changed = result.success;
                return result.success;
            });
        return result;
    }

    CollisionPairWorkflowResult CollisionLegacyPairDocumentFacade::setRobotObjectPairEnabled(
        const QString& robotId,
        const QString& objectId,
        bool enabled)
    {
        CollisionPairWorkflowResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionLegacyPairs"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                const CollisionPairWorkflowController workflow;
                result = workflow.setRobotObjectPairEnabled(
                    service.document(),
                    robotId,
                    objectId,
                    enabled);
                changed = result.success;
                return result.success;
            });
        return result;
    }
}
