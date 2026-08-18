#include "CollisionLinkModelDocumentFacade.h"

#include <SimulationProject/ProjectDocumentService.h>

#include <algorithm>

CollisionLinkModelDocumentFacade::CollisionLinkModelDocumentFacade(
    simulation_project::ProjectDocument& document)
    : m_document(document)
{
}

simulation_project::CollisionDetectorDesc* CollisionLinkModelDocumentFacade::findDetector(
    const std::string& detectorId)
{
    auto it = std::find_if(
        m_document.collision.detectors.begin(),
        m_document.collision.detectors.end(),
        [&](const simulation_project::CollisionDetectorDesc& detector) {
            return detector.id == detectorId;
        });
    return it == m_document.collision.detectors.end() ? nullptr : &(*it);
}

simulation_project::RobotDesc* CollisionLinkModelDocumentFacade::findRobot(
    const std::string& robotId)
{
    auto it = std::find_if(
        m_document.robots.begin(),
        m_document.robots.end(),
        [&](const simulation_project::RobotDesc& robot) {
            return robot.id == robotId;
        });
    return it == m_document.robots.end() ? nullptr : &(*it);
}

const simulation_project::RobotDesc* CollisionLinkModelDocumentFacade::findRobot(
    const std::string& robotId) const
{
    auto it = std::find_if(
        m_document.robots.begin(),
        m_document.robots.end(),
        [&](const simulation_project::RobotDesc& robot) {
            return robot.id == robotId;
        });
    return it == m_document.robots.end() ? nullptr : &(*it);
}

simulation_project::RobotCollisionOverrideDesc* CollisionLinkModelDocumentFacade::findOverride(
    const std::string& robotId)
{
    auto it = std::find_if(
        m_document.collision.robotOverrides.begin(),
        m_document.collision.robotOverrides.end(),
        [&](const simulation_project::RobotCollisionOverrideDesc& collisionOverride) {
            return collisionOverride.robotId == robotId;
        });
    return it == m_document.collision.robotOverrides.end() ? nullptr : &(*it);
}

const simulation_project::RobotCollisionOverrideDesc* CollisionLinkModelDocumentFacade::findOverride(
    const std::string& robotId) const
{
    auto it = std::find_if(
        m_document.collision.robotOverrides.begin(),
        m_document.collision.robotOverrides.end(),
        [&](const simulation_project::RobotCollisionOverrideDesc& collisionOverride) {
            return collisionOverride.robotId == robotId;
        });
    return it == m_document.collision.robotOverrides.end() ? nullptr : &(*it);
}

bool CollisionLinkModelDocumentFacade::hasOverrideElements(const std::string& robotId) const
{
    const simulation_project::RobotCollisionOverrideDesc* collisionOverride = findOverride(robotId);
    return collisionOverride != nullptr && !collisionOverride->elements.empty();
}

simulation_project::RobotCollisionOverrideDesc& CollisionLinkModelDocumentFacade::ensureOverride(
    const std::string& robotId)
{
    simulation_project::ProjectDocumentService service(m_document);
    return service.ensureRobotCollisionOverride(robotId);
}

bool CollisionLinkModelDocumentFacade::setReplaceOriginal(
    const std::string& robotId,
    bool replaceOriginal)
{
    simulation_project::ProjectDocumentService service(m_document);
    return service.setRobotCollisionReplaceOriginal(robotId, replaceOriginal);
}

simulation_project::CollisionElementOverrideDesc CollisionLinkModelDocumentFacade::addBoxElement(
    const std::string& robotId,
    const std::string& linkName)
{
    simulation_project::ProjectDocumentService service(m_document);
    return service.addBoxCollisionElement(robotId, linkName);
}

bool CollisionLinkModelDocumentFacade::removeElement(
    const std::string& robotId,
    const std::string& linkName,
    const std::string& elementId)
{
    simulation_project::ProjectDocumentService service(m_document);
    return service.removeCollisionElement(robotId, linkName, elementId);
}

void CollisionLinkModelDocumentFacade::appendUniqueElements(
    const std::string& robotId,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
{
    simulation_project::ProjectDocumentService service(m_document);
    service.appendUniqueCollisionElements(robotId, elements);
}
