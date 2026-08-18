#include "CollisionLinkModelsCommandController.h"

#include "CollisionWorkbenchServices.h"
#include "CollisionLinkModelDocumentFacade.h"
#include "RobotQtViewerViewportServices.h"

#include <SimulationProject/CollisionModelSelectionIds.h>
#include <SimulationProject/ProjectDocumentService.h>

#include <algorithm>

namespace
{
    CollisionLinkModelsCommandResult makeFailure(const QString& message)
    {
        CollisionLinkModelsCommandResult result;
        result.success = false;
        result.message = message;
        return result;
    }

    std::string normalizedCollisionRole(const std::string& role)
    {
        return role.empty() ? std::string("Exact") : role;
    }

    std::string normalizedCollisionSource(const std::string& source)
    {
        return source.empty() ? std::string("original") : source;
    }

    QString collisionProxyQualityText(const robot_qt_viewer::CollisionRuntimeProxyQualitySummary& summary)
    {
        QString text = QString(
            "sphere cover quality: generated %1/%2, input points %3, uncovered %4, max outside %5, max radius %6, axis %7, radius/axis %8")
            .arg(summary.generatedSphereCount)
            .arg(summary.requestedMaxSpheres)
            .arg(static_cast<qulonglong>(summary.inputPointCount))
            .arg(static_cast<qulonglong>(summary.uncoveredPointCount))
            .arg(summary.maxOutsideDistance, 0, 'f', 6)
            .arg(summary.maxSphereRadius, 0, 'f', 6)
            .arg(summary.mainAxisLength, 0, 'f', 6)
            .arg(summary.maxRadiusToLinkLength, 0, 'f', 6);
        if(summary.estimatedCrossSectionRadius > 0.0) {
            text += QString(", cross radius %1, radius/cross %2")
                .arg(summary.estimatedCrossSectionRadius, 0, 'f', 6)
                .arg(summary.maxRadiusToCrossSectionRadius, 0, 'f', 6);
        }
        if(summary.oversizedSphereCount > 0) {
            text += QString(", oversized %1").arg(summary.oversizedSphereCount);
        }
        if(!summary.recommendedShape.empty()) {
            text += QString(", recommended: %1").arg(QString::fromStdString(summary.recommendedShape));
        }
        if(summary.hasWarning && !summary.warning.empty()) {
            text += QString("  %1").arg(QString::fromStdString(summary.warning));
        }
        return text;
    }

    void populateGeneratedVariantResult(
        CollisionLinkModelsCommandResult& result,
        const std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
    {
        result.generatedCount = static_cast<int>(elements.size());
        if(elements.empty()) {
            return;
        }

        result.selectedSource =
            QString::fromStdString(normalizedCollisionSource(elements.front().source));
        result.selectedRole =
            QString::fromStdString(normalizedCollisionRole(elements.front().role));
    }

    void populateGeneratedObjectVariantResult(
        CollisionLinkModelsCommandResult& result,
        const std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements)
    {
        result.generatedCount = static_cast<int>(elements.size());
        if(elements.empty()) {
            return;
        }

        result.selectedSource =
            QString::fromStdString(normalizedCollisionSource(elements.front().source));
        result.selectedRole =
            QString::fromStdString(normalizedCollisionRole(elements.front().role));
    }
}

CollisionLinkModelsCommandResult CollisionLinkModelsCommandController::generateFromVisual(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& robotId,
    const QString& linkName,
    const robot_qt_viewer::CollisionRuntimeProxyRequest& request,
    bool replaceOriginal)
{
    if(robotId.isEmpty() || linkName.isEmpty()) {
        return makeFailure("Select a robot link first.");
    }
    if(viewportServices == nullptr) {
        return makeFailure("Viewport is not available.");
    }

    std::vector<simulation_project::CollisionElementOverrideDesc> elements;
    if(!viewportServices->generateRobotCollisionProxies(robotId, linkName, request, elements)) {
        return makeFailure(
            QString("Failed to generate %1 collision from visual mesh.")
                .arg(QString::fromStdString(request.proxyType)));
    }

    CollisionLinkModelDocumentFacade documentFacade(document);
    documentFacade.setReplaceOriginal(robotId.toStdString(), replaceOriginal);
    CollisionLinkModelsCommandResult result;
    result.success = true;
    result.projectChanged = true;
    populateGeneratedVariantResult(result, elements);

    robot_qt_viewer::CollisionRuntimeProxyQualitySummary qualitySummary;
    if(request.proxyType == "sphereCover" &&
        viewportServices->evaluateRobotCollisionProxyQuality(
            robotId,
            linkName,
            request,
            elements,
            false,
            qualitySummary)) {
        result.qualityMessage = collisionProxyQualityText(qualitySummary);
    }

    documentFacade.appendUniqueElements(robotId.toStdString(), elements);

    const QString qualitySuffix = result.qualityMessage.isEmpty()
        ? QString()
        : QString("  %1").arg(result.qualityMessage);
    result.message = QString("Generated %1 %2 collision element(s) from %3.%4%5")
        .arg(result.generatedCount)
        .arg(QString::fromStdString(request.proxyType), robotId, linkName, qualitySuffix);
    return result;
}

CollisionLinkModelsCommandResult CollisionLinkModelsCommandController::generateFromExistingCollision(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& robotId,
    const QString& linkName,
    const robot_qt_viewer::CollisionRuntimeProxyRequest& request,
    bool replaceOriginal)
{
    if(robotId.isEmpty() || linkName.isEmpty()) {
        return makeFailure("Select a robot link first.");
    }
    if(viewportServices == nullptr) {
        return makeFailure("Viewport is not available.");
    }

    std::vector<simulation_project::CollisionElementOverrideDesc> elements;
    if(!viewportServices->generateRobotCollisionProxiesFromExistingCollision(
           robotId,
           linkName,
           request,
           elements)) {
        return makeFailure(
            QString("Failed to generate %1 from existing collision.")
                .arg(QString::fromStdString(request.proxyType)));
    }

    CollisionLinkModelDocumentFacade documentFacade(document);
    documentFacade.setReplaceOriginal(robotId.toStdString(), replaceOriginal);

    CollisionLinkModelsCommandResult result;
    result.success = true;
    result.projectChanged = true;
    populateGeneratedVariantResult(result, elements);

    robot_qt_viewer::CollisionRuntimeProxyQualitySummary qualitySummary;
    if(request.proxyType == "sphereCover" &&
        viewportServices->evaluateRobotCollisionProxyQuality(
            robotId,
            linkName,
            request,
            elements,
            true,
            qualitySummary)) {
        result.qualityMessage = collisionProxyQualityText(qualitySummary);
    }

    documentFacade.appendUniqueElements(robotId.toStdString(), elements);

    const QString qualitySuffix = result.qualityMessage.isEmpty()
        ? QString()
        : QString("  %1").arg(result.qualityMessage);
    result.message = QString("Generated %1 %2 element(s) from existing collision%3")
        .arg(result.generatedCount)
        .arg(QString::fromStdString(request.proxyType), qualitySuffix);
    return result;
}

CollisionLinkModelsCommandResult CollisionLinkModelsCommandController::generateRobotFromExistingCollision(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& robotId,
    const robot_qt_viewer::CollisionRuntimeProxyRequest& request,
    bool replaceOriginal)
{
    if(robotId.isEmpty()) {
        return makeFailure("Select a robot first.");
    }
    if(viewportServices == nullptr) {
        return makeFailure("Viewport is not available.");
    }

    std::vector<simulation_project::CollisionElementOverrideDesc> elements;
    if(!viewportServices->generateRobotCollisionProxiesFromExistingCollision(robotId, request, elements)) {
        return makeFailure(
            QString("Failed to generate robot %1 from existing collision.")
                .arg(QString::fromStdString(request.proxyType)));
    }

    CollisionLinkModelDocumentFacade documentFacade(document);
    documentFacade.setReplaceOriginal(robotId.toStdString(), replaceOriginal);

    CollisionLinkModelsCommandResult result;
    result.success = true;
    result.projectChanged = true;
    populateGeneratedVariantResult(result, elements);
    documentFacade.appendUniqueElements(robotId.toStdString(), elements);

    result.message = QString("Generated %1 robot %2 element(s) from existing collision")
        .arg(result.generatedCount)
        .arg(QString::fromStdString(request.proxyType));
    return result;
}

CollisionLinkModelsCommandResult CollisionLinkModelsCommandController::generateMissingFromVisual(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& robotId,
    const robot_qt_viewer::CollisionRuntimeProxyRequest& request,
    bool replaceOriginal)
{
    if(robotId.isEmpty()) {
        return makeFailure("Select a robot first.");
    }
    if(viewportServices == nullptr) {
        return makeFailure("Viewport is not available.");
    }

    std::vector<simulation_project::CollisionElementOverrideDesc> elements;
    if(!viewportServices->generateMissingRobotCollisionProxies(robotId, request, elements)) {
        return makeFailure("No missing visual collision proxies could be generated.");
    }

    CollisionLinkModelDocumentFacade documentFacade(document);
    documentFacade.setReplaceOriginal(robotId.toStdString(), replaceOriginal);

    CollisionLinkModelsCommandResult result;
    result.success = true;
    result.projectChanged = true;
    populateGeneratedVariantResult(result, elements);
    documentFacade.appendUniqueElements(robotId.toStdString(), elements);

    result.message =
        QString("Generated %1 missing collision proxies for %2").arg(result.generatedCount).arg(robotId);
    return result;
}

CollisionLinkModelsCommandResult CollisionLinkModelsCommandController::generateCoacdForLink(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& robotId,
    const QString& linkName)
{
    if(robotId.isEmpty() || linkName.isEmpty()) {
        return makeFailure("Select a robot link first.");
    }
    if(viewportServices == nullptr) {
        return makeFailure("Viewport is not available.");
    }

    std::vector<simulation_project::CollisionElementOverrideDesc> elements;
    QString inputLabel = "existing collision";
    if(!viewportServices->generateRobotCollisionCoacdFromExistingCollision(
           robotId,
           linkName,
           elements)) {
        inputLabel = "visual mesh";
        if(!viewportServices->generateRobotCollisionCoacdFromVisual(
               robotId,
               linkName,
               elements)) {
            return makeFailure("Failed to generate simplified collision model with COACD.");
        }
    }

    CollisionLinkModelsCommandResult result;
    result.success = true;
    result.projectChanged = true;
    populateGeneratedVariantResult(result, elements);

    simulation_project::ProjectDocumentService service(document);
    service.appendUniqueCollisionElements(robotId.toStdString(), elements);

    result.message = QString("Generated %1 COACD collision part(s) from %2.")
        .arg(result.generatedCount)
        .arg(inputLabel);
    return result;
}

CollisionLinkModelsCommandResult CollisionLinkModelsCommandController::generateCoacdForObject(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& objectId)
{
    if(objectId.isEmpty()) {
        return makeFailure("Select an object or attachment first.");
    }
    if(viewportServices == nullptr) {
        return makeFailure("Viewport is not available.");
    }

    std::vector<simulation_project::ObjectCollisionElementOverrideDesc> elements;
    if(!viewportServices->generateObjectCollisionCoacdFromVisual(objectId, elements)) {
        return makeFailure("Failed to generate simplified object collision model with COACD.");
    }

    CollisionLinkModelsCommandResult result;
    result.success = true;
    result.projectChanged = true;
    populateGeneratedObjectVariantResult(result, elements);

    simulation_project::ProjectDocumentService service(document);
    service.ensureObjectCollisionOverride(objectId.toStdString());
    service.appendUniqueObjectCollisionElements(objectId.toStdString(), elements);

    result.message = QString("Generated %1 COACD object collision part(s).")
        .arg(result.generatedCount);
    return result;
}

CollisionLinkModelsCommandResult CollisionLinkModelsCommandController::saveSidecar(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::CollisionWorkbenchServices& appServices,
    const QString& robotId,
    const std::filesystem::path& sidecarPath,
    const std::string& portableSidecarPath)
{
    if(robotId.isEmpty()) {
        return makeFailure("Select a robot first.");
    }

    CollisionLinkModelDocumentFacade documentFacade(document);
    simulation_project::RobotCollisionOverrideDesc* collisionOverride =
        documentFacade.findOverride(robotId.toStdString());
    if(collisionOverride == nullptr || collisionOverride->elements.empty()) {
        return makeFailure("Selected robot has no collision overrides.");
    }

    const simulation_project::RobotDesc* robotDesc = documentFacade.findRobot(robotId.toStdString());
    if(robotDesc != nullptr && collisionOverride->sourceRobotPath.empty()) {
        collisionOverride->sourceRobotPath = robotDesc->sourcePath;
    }

    std::string error;
    if(!appServices.saveRobotCollisionOverride(sidecarPath, *collisionOverride, &error)) {
        return makeFailure(QString("Sidecar save failed: %1").arg(QString::fromStdString(error)));
    }

    collisionOverride->overridePath = portableSidecarPath;

    CollisionLinkModelsCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.message = QString("Saved sidecar %1").arg(QString::fromStdWString(sidecarPath.wstring()));
    return result;
}

CollisionLinkModelsCommandResult CollisionLinkModelsCommandController::exportUrdf(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::CollisionWorkbenchServices& appServices,
    const QString& robotId,
    const std::filesystem::path& sourceUrdfPath,
    const std::filesystem::path& outputUrdfPath)
{
    if(robotId.isEmpty()) {
        return makeFailure("Select a robot first.");
    }

    CollisionLinkModelDocumentFacade documentFacade(document);
    simulation_project::RobotDesc* robotDesc = documentFacade.findRobot(robotId.toStdString());
    if(robotDesc == nullptr) {
        return makeFailure("Selected robot is not in project.");
    }
    if(robotDesc->sourceType != "urdf") {
        return makeFailure("URDF collision export only supports URDF robots.");
    }

    simulation_project::RobotCollisionOverrideDesc* collisionOverride =
        documentFacade.findOverride(robotId.toStdString());
    if(collisionOverride == nullptr || collisionOverride->elements.empty()) {
        return makeFailure("Selected robot has no collision overrides.");
    }

    collisionOverride->sourceRobotPath = robotDesc->sourcePath;

    std::string error;
    if(!appServices.exportRobotCollisionOverrideToUrdf(
           sourceUrdfPath,
           outputUrdfPath,
           *collisionOverride,
           &error)) {
        return makeFailure(QString("URDF export failed: %1").arg(QString::fromStdString(error)));
    }

    CollisionLinkModelsCommandResult result;
    result.success = true;
    result.message = QString("Exported URDF %1").arg(QString::fromStdWString(outputUrdfPath.wstring()));
    return result;
}
