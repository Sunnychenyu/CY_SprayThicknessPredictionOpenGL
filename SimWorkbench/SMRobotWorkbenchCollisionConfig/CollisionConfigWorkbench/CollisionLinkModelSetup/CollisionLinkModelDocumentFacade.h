#pragma once

#include <SimulationProject/ProjectDocument.h>

#include <string>
#include <vector>

class CollisionLinkModelDocumentFacade
{
public:
    explicit CollisionLinkModelDocumentFacade(simulation_project::ProjectDocument& document);

    simulation_project::CollisionDetectorDesc* findDetector(const std::string& detectorId);
    simulation_project::RobotDesc* findRobot(const std::string& robotId);
    const simulation_project::RobotDesc* findRobot(const std::string& robotId) const;
    simulation_project::RobotCollisionOverrideDesc* findOverride(const std::string& robotId);
    const simulation_project::RobotCollisionOverrideDesc* findOverride(const std::string& robotId) const;

    bool hasOverrideElements(const std::string& robotId) const;
    simulation_project::RobotCollisionOverrideDesc& ensureOverride(const std::string& robotId);
    bool setReplaceOriginal(const std::string& robotId, bool replaceOriginal);
    simulation_project::CollisionElementOverrideDesc addBoxElement(
        const std::string& robotId,
        const std::string& linkName);
    bool removeElement(
        const std::string& robotId,
        const std::string& linkName,
        const std::string& elementId);
    void appendUniqueElements(
        const std::string& robotId,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements);

private:
    simulation_project::ProjectDocument& m_document;
};
