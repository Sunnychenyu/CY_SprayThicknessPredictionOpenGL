#pragma once

#include <SimulationProject/ProjectDocument.h>

#include <cstddef>
#include <string>
#include <vector>

class CollisionDetectorDocumentFacade
{
public:
    explicit CollisionDetectorDocumentFacade(simulation_project::ProjectDocument& document);

    simulation_project::CollisionDetectorDesc* findDetector(const std::string& detectorId);
    std::size_t detectorCount() const;
    std::vector<std::string> detectorIds() const;

    bool setDetectorEnabled(const std::string& detectorId, bool enabled);
    bool setDetectorVisible(const std::string& detectorId, bool visible);
    bool removeDetector(const std::string& detectorId);

private:
    simulation_project::ProjectDocument& m_document;
};
