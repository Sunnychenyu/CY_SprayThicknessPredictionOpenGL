#include "CollisionModelDocumentFacade.h"

#include "CollisionLinkModelsController.h"

#include <SimulationProject/ProjectDocument.h>

#include <algorithm>
#include <string>

namespace
{
    const simulation_project::CollisionDetectorDesc* findCollisionDetector(
        const simulation_project::ProjectDocument& document,
        const QString& detectorId)
    {
        const std::string detectorIdText = detectorId.toStdString();
        const auto it = std::find_if(
            document.collision.detectors.begin(),
            document.collision.detectors.end(),
            [&](const simulation_project::CollisionDetectorDesc& detector) {
                return detector.id == detectorIdText;
            });
        return it == document.collision.detectors.end() ? nullptr : &(*it);
    }

    std::string normalizedCollisionSource(const std::string& source)
    {
        return source.empty() ? std::string("original") : source;
    }
}

namespace robot_qt_viewer
{
    CollisionModelDocumentFacade::CollisionModelDocumentFacade(
        const simulation_project::ProjectDocument& document)
        : m_document(document)
    {
    }

    CollisionModelRuntimeContext CollisionModelDocumentFacade::runtimeContext(
        const QString& activeDetectorId) const
    {
        CollisionModelRuntimeContext context;
        const simulation_project::CollisionDetectorDesc* activeDetector =
            findCollisionDetector(m_document, activeDetectorId);
        if(activeDetector == nullptr) {
            return context;
        }

        context.role = QString::fromStdString(activeDetector->geometryRole);
        if(!activeDetector->geometrySource.empty()) {
            context.source = QString::fromStdString(normalizedCollisionSource(activeDetector->geometrySource));
        }
        return context;
    }

    CollisionLinkModelsViewModel CollisionModelDocumentFacade::buildViewModel(
        const QString& selectedRobotId,
        const QString& selectedLinkName,
        const QString& selectedObjectId,
        const QString& selectedAttachmentId,
        const QString& activeDetectorId,
        const QString& previousVariantId,
        const QString& visibleVariantId,
        const QString& qualityMessage,
        const std::filesystem::path& projectBasePath,
        const CollisionRuntimeRobotSummary* robotSummary,
        const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors) const
    {
        return CollisionLinkModelsController::buildViewModel(
            m_document,
            selectedRobotId,
            selectedLinkName,
            selectedObjectId,
            selectedAttachmentId,
            activeDetectorId,
            previousVariantId,
            visibleVariantId,
            qualityMessage,
            projectBasePath,
            robotSummary,
            runtimeDetectors);
    }

    CollisionLinkModelsSummaryView CollisionModelDocumentFacade::buildSummary(
        const QString& selectedRobotId,
        const QString& selectedLinkName,
        const QString& selectedObjectId,
        const QString& selectedAttachmentId,
        const QString& activeDetectorId,
        const QString& qualityMessage,
        const CollisionRuntimeRobotSummary* robotSummary,
        const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors) const
    {
        return CollisionLinkModelsController::buildSummary(
            m_document,
            selectedRobotId,
            selectedLinkName,
            selectedObjectId,
            selectedAttachmentId,
            activeDetectorId,
            qualityMessage,
            robotSummary,
            runtimeDetectors);
    }
}
