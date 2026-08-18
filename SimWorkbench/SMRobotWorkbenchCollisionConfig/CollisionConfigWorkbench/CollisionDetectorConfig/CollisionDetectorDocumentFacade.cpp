#include "CollisionDetectorDocumentFacade.h"

#include <SimulationProject/ProjectDocumentService.h>

#include <algorithm>

CollisionDetectorDocumentFacade::CollisionDetectorDocumentFacade(
    simulation_project::ProjectDocument& document)
    : m_document(document)
{
}

simulation_project::CollisionDetectorDesc* CollisionDetectorDocumentFacade::findDetector(
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

std::size_t CollisionDetectorDocumentFacade::detectorCount() const
{
    return m_document.collision.detectors.size();
}

std::vector<std::string> CollisionDetectorDocumentFacade::detectorIds() const
{
    std::vector<std::string> ids;
    ids.reserve(m_document.collision.detectors.size());
    for(const simulation_project::CollisionDetectorDesc& detector : m_document.collision.detectors) {
        ids.push_back(detector.id);
    }
    return ids;
}

bool CollisionDetectorDocumentFacade::setDetectorEnabled(
    const std::string& detectorId,
    bool enabled)
{
    simulation_project::ProjectDocumentService service(m_document);
    return service.setCollisionDetectorEnabled(detectorId, enabled);
}

bool CollisionDetectorDocumentFacade::setDetectorVisible(
    const std::string& detectorId,
    bool visible)
{
    simulation_project::ProjectDocumentService service(m_document);
    return service.setCollisionDetectorVisible(detectorId, visible);
}

bool CollisionDetectorDocumentFacade::removeDetector(const std::string& detectorId)
{
    simulation_project::ProjectDocumentService service(m_document);
    return service.removeCollisionDetector(detectorId);
}
