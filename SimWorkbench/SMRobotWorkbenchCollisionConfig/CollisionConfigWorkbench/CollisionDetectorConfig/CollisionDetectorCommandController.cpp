#include "CollisionDetectorCommandController.h"

#include "CollisionDetectorDocumentFacade.h"
#include "RobotQtViewerViewportServices.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

namespace
{
    bool shouldRequestDetectorContacts(const simulation_project::CollisionVisualizationDesc& visualization)
    {
        return visualization.showContacts ||
            visualization.showNormals ||
            visualization.showObjectHighlight;
    }

    bool detectorPairCompileChanged(
        const simulation_project::CollisionDetectorDesc& previous,
        const simulation_project::CollisionDetectorDesc& current)
    {
        return previous.type != current.type ||
            previous.queryPolicy != current.queryPolicy ||
            previous.selectionSetAId != current.selectionSetAId ||
            previous.selectionSetBId != current.selectionSetBId ||
            previous.targets.size() != current.targets.size() ||
            previous.pairGenerators.size() != current.pairGenerators.size() ||
            previous.pairFilters.size() != current.pairFilters.size() ||
            previous.excludePairs.size() != current.excludePairs.size();
    }

    bool detectorRuntimeOptionsChanged(
        const simulation_project::CollisionDetectorDesc& previous,
        const simulation_project::CollisionDetectorDesc& current)
    {
        return previous.name != current.name ||
            previous.geometryRole != current.geometryRole ||
            previous.geometrySource != current.geometrySource ||
            previous.contacts != current.contacts ||
            previous.nearestPoints != current.nearestPoints ||
            previous.distance != current.distance ||
            previous.maxContacts != current.maxContacts ||
            previous.distanceThreshold != current.distanceThreshold ||
            previous.visualization.showCollisionGeometry != current.visualization.showCollisionGeometry ||
            previous.visualization.showContacts != current.visualization.showContacts ||
            previous.visualization.showNormals != current.visualization.showNormals ||
            previous.visualization.showNearestPoints != current.visualization.showNearestPoints ||
            previous.visualization.showObjectHighlight != current.visualization.showObjectHighlight ||
            previous.visualization.alpha != current.visualization.alpha ||
            previous.visualization.normalLength != current.visualization.normalLength ||
            previous.visualization.contactPointRadius != current.visualization.contactPointRadius;
    }

    CollisionDetectorCommandResult makeFailure(const QString& message)
    {
        CollisionDetectorCommandResult result;
        result.success = false;
        result.message = message;
        return result;
    }

    bool hasDuplicateDetectorName(
        const simulation_project::ProjectDocument& document,
        const std::string& currentDetectorId,
        const std::string& candidateName)
    {
        return std::any_of(
            document.collision.detectors.begin(),
            document.collision.detectors.end(),
            [&](const simulation_project::CollisionDetectorDesc& detector) {
                if(detector.id == currentDetectorId) {
                    return false;
                }
                const std::string effectiveName = detector.name.empty() ? detector.id : detector.name;
                return effectiveName == candidateName;
            });
    }

    bool sameDraftMember(
        const CollisionDetectorDraftMemberView& a,
        const CollisionDetectorDraftMemberView& b)
    {
        return a.robotId == b.robotId &&
            a.linkName == b.linkName &&
            a.objectId == b.objectId &&
            a.attachmentId == b.attachmentId;
    }

    bool isRobotMember(const CollisionDetectorDraftMemberView& member)
    {
        return !member.robotId.isEmpty();
    }

    QString collisionObjectId(const CollisionDetectorDraftMemberView& member)
    {
        return !member.objectId.isEmpty() ? member.objectId : member.attachmentId;
    }

    bool isObjectMember(const CollisionDetectorDraftMemberView& member)
    {
        return !collisionObjectId(member).isEmpty();
    }

    bool isSupportedPairGeneratorMember(const CollisionDetectorDraftMemberView& member)
    {
        return isRobotMember(member) || isObjectMember(member);
    }

    bool sameStringVector(
        const std::vector<std::string>& a,
        const std::vector<std::string>& b)
    {
        return a == b;
    }

    bool samePairGenerator(
        const simulation_project::CollisionPairGeneratorDesc& a,
        const simulation_project::CollisionPairGeneratorDesc& b)
    {
        return a.type == b.type &&
            a.robotId == b.robotId &&
            a.linkName == b.linkName &&
            a.objectId == b.objectId &&
            a.objectGroupId == b.objectGroupId &&
            a.robotA == b.robotA &&
            a.linkA == b.linkA &&
            a.objectA == b.objectA &&
            a.objectGroupA == b.objectGroupA &&
            sameStringVector(a.includeLinksA, b.includeLinksA) &&
            sameStringVector(a.excludeLinksA, b.excludeLinksA) &&
            a.robotB == b.robotB &&
            a.linkB == b.linkB &&
            a.objectB == b.objectB &&
            a.objectGroupB == b.objectGroupB &&
            sameStringVector(a.includeLinksB, b.includeLinksB) &&
            sameStringVector(a.excludeLinksB, b.excludeLinksB);
    }

    bool containsPairGenerator(
        const std::vector<simulation_project::CollisionPairGeneratorDesc>& generators,
        const simulation_project::CollisionPairGeneratorDesc& candidate)
    {
        return std::any_of(
            generators.begin(),
            generators.end(),
            [&](const simulation_project::CollisionPairGeneratorDesc& generator) {
                return samePairGenerator(generator, candidate);
            });
    }

    simulation_project::CollisionPairGeneratorDesc makeGenerator(
        const CollisionDetectorDraftMemberView& a,
        const CollisionDetectorDraftMemberView& b)
    {
        simulation_project::CollisionPairGeneratorDesc generator;
        if(isRobotMember(a) && isRobotMember(b)) {
            const bool linkA = !a.linkName.isEmpty();
            const bool linkB = !b.linkName.isEmpty();
            generator.type = linkA && linkB ? "LinkLink" : "RobotRobot";
            generator.robotA = a.robotId.toStdString();
            generator.linkA = a.linkName.toStdString();
            generator.robotB = b.robotId.toStdString();
            generator.linkB = b.linkName.toStdString();
            if(linkA) {
                generator.includeLinksA.push_back(a.linkName.toStdString());
            }
            if(linkB) {
                generator.includeLinksB.push_back(b.linkName.toStdString());
            }
            return generator;
        }

        const CollisionDetectorDraftMemberView* robotMember = nullptr;
        const CollisionDetectorDraftMemberView* objectMember = nullptr;
        if(isRobotMember(a) && isObjectMember(b)) {
            robotMember = &a;
            objectMember = &b;
        } else if(isObjectMember(a) && isRobotMember(b)) {
            robotMember = &b;
            objectMember = &a;
        }
        if(robotMember != nullptr && objectMember != nullptr) {
            generator.type = robotMember->linkName.isEmpty() ? "RobotObject" : "LinkObject";
            generator.robotId = robotMember->robotId.toStdString();
            generator.linkName = robotMember->linkName.toStdString();
            generator.objectId = collisionObjectId(*objectMember).toStdString();
            if(!robotMember->linkName.isEmpty()) {
                generator.includeLinksA.push_back(robotMember->linkName.toStdString());
            }
            return generator;
        }

        if(isObjectMember(a) && isObjectMember(b)) {
            generator.type = "ObjectObject";
            generator.objectA = collisionObjectId(a).toStdString();
            generator.objectB = collisionObjectId(b).toStdString();
            return generator;
        }

        return generator;
    }

    std::vector<simulation_project::CollisionPairGeneratorDesc> makeGenerators(
        const QVector<CollisionDetectorDraftMemberView>& setA,
        const QVector<CollisionDetectorDraftMemberView>& setB,
        QString* error)
    {
        std::vector<simulation_project::CollisionPairGeneratorDesc> generators;
        if(setA.isEmpty()) {
            if(error != nullptr) {
                *error = "Temporary Set A is empty.";
            }
            return generators;
        }

        const bool withinSet = setB.isEmpty();
        if(withinSet && setA.size() < 2) {
            if(error != nullptr) {
                *error = "Within-set binding needs at least two members in Set A.";
            }
            return generators;
        }

        auto appendPair = [&](const CollisionDetectorDraftMemberView& a, const CollisionDetectorDraftMemberView& b) {
            if(sameDraftMember(a, b)) {
                return;
            }
            if(!isSupportedPairGeneratorMember(a) || !isSupportedPairGeneratorMember(b)) {
                return;
            }
            simulation_project::CollisionPairGeneratorDesc generator = makeGenerator(a, b);
            if(!generator.type.empty()) {
                generators.push_back(std::move(generator));
            }
        };

        if(withinSet) {
            for(int i = 0; i < setA.size(); ++i) {
                for(int j = i + 1; j < setA.size(); ++j) {
                    appendPair(setA[i], setA[j]);
                }
            }
        } else {
            for(const CollisionDetectorDraftMemberView& a : setA) {
                for(const CollisionDetectorDraftMemberView& b : setB) {
                    appendPair(a, b);
                }
            }
        }

        if(generators.empty() && error != nullptr) {
            *error = "Temporary sets did not produce any supported collision pair generators.";
        }
        return generators;
    }
}

CollisionDetectorCommandResult CollisionDetectorCommandController::setDetectorEnabled(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& detectorId,
    bool enabled)
{
    if(detectorId.isEmpty()) {
        return makeFailure("No collision detector selected.");
    }

    CollisionDetectorDocumentFacade documentFacade(document);
    if(!documentFacade.setDetectorEnabled(detectorId.toStdString(), enabled)) {
        return makeFailure("Selected collision detector is not in the project.");
    }

    if(viewportServices != nullptr) {
        viewportServices->setCollisionDetectorEnabled(detectorId, enabled);
    }

    CollisionDetectorCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.message = QString("Updated collision detector %1").arg(detectorId);
    return result;
}

CollisionDetectorCommandResult CollisionDetectorCommandController::applyDetectorProperties(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& detectorId,
    const CollisionDetectorPropertiesView& properties)
{
    CollisionDetectorDocumentFacade documentFacade(document);
    simulation_project::CollisionDetectorDesc* detector =
        documentFacade.findDetector(detectorId.toStdString());
    if(detector == nullptr) {
        return makeFailure("Selected collision detector is not in the project.");
    }

    const QString policy = properties.policy;
    const QString setA = properties.setA;
    const QString setB = properties.setB;

    if(policy == "WithinSet" && setA.isEmpty()) {
        return makeFailure("Within One Set needs Set A.");
    }
    if(policy == "BetweenSets" && (setA.isEmpty() || setB.isEmpty())) {
        return makeFailure("Between Sets needs both Set A and Set B.");
    }
    if(policy == "BetweenSets" && setA == setB) {
        return makeFailure("Set A and Set B must be different.");
    }
    if(policy == "Legacy" && detector->targets.empty() && detector->pairGenerators.empty()) {
        return makeFailure("Legacy mode needs existing targets or pair generators.");
    }

    const simulation_project::CollisionDetectorDesc previousDetector = *detector;

    const QString name = properties.name;
    detector->name = name.isEmpty() ? detector->id : name.toStdString();
    detector->enabled = properties.enabled;
    detector->visualization.visible = properties.visible;
    detector->visualization.showCollisionGeometry = properties.geometry;
    detector->visualization.showContacts = properties.contacts;
    detector->visualization.showNormals = properties.normals;
    detector->contacts = shouldRequestDetectorContacts(detector->visualization);
    detector->nearestPoints = properties.nearest;
    detector->distance = detector->nearestPoints;
    detector->visualization.showNearestPoints = detector->nearestPoints;
    detector->maxContacts = static_cast<int>(std::lround(properties.maxContacts));
    detector->distanceThreshold = properties.distanceThreshold;

    const std::string selectedRole = properties.role.toStdString();
    if(detector->geometryRole != selectedRole) {
        detector->geometryRole = selectedRole;
        detector->geometrySource.clear();
    }

    if(policy == "AllEnabled") {
        detector->type = "SceneAll";
        detector->queryPolicy = "AllEnabled";
        detector->selectionSetAId.clear();
        detector->selectionSetBId.clear();
        detector->targets.clear();
        detector->pairGenerators.clear();
    } else if(policy == "WithinSet") {
        detector->type = "SelectedObjects";
        detector->queryPolicy = "WithinSet";
        detector->selectionSetAId = setA.toStdString();
        detector->selectionSetBId.clear();
        detector->targets.clear();
        detector->pairGenerators.clear();
    } else if(policy == "BetweenSets") {
        detector->type = "SelectedObjects";
        detector->queryPolicy = "BetweenSets";
        detector->selectionSetAId = setA.toStdString();
        detector->selectionSetBId = setB.toStdString();
        detector->targets.clear();
        detector->pairGenerators.clear();
    } else {
        detector->queryPolicy.clear();
        detector->selectionSetAId.clear();
        detector->selectionSetBId.clear();
        if(detector->type.empty()) {
            detector->type = "SelectedObjects";
        }
    }

    const bool pairCompileChanged = detectorPairCompileChanged(previousDetector, *detector);
    const bool runtimeOptionsChanged = detectorRuntimeOptionsChanged(previousDetector, *detector);
    const bool enabledChanged = previousDetector.enabled != detector->enabled;
    const bool visibleChanged = previousDetector.visualization.visible != detector->visualization.visible;

    bool updatedRuntime = viewportServices == nullptr;
    if(viewportServices != nullptr) {
        if(pairCompileChanged) {
            updatedRuntime = viewportServices->rebuildCollisionDetectorsFromDocument(document);
        } else if(runtimeOptionsChanged) {
            updatedRuntime = viewportServices->updateCollisionDetectorRuntimeOptions(*detector);
        } else {
            updatedRuntime = true;
            if(enabledChanged) {
                updatedRuntime =
                    viewportServices->setCollisionDetectorEnabled(detectorId, detector->enabled) && updatedRuntime;
            }
            if(visibleChanged) {
                updatedRuntime =
                    viewportServices->setCollisionDetectorVisible(detectorId, detector->visualization.visible) &&
                    updatedRuntime;
            }
        }

        if(updatedRuntime) {
            viewportServices->setActiveCollisionDetector(detectorId);
        }
    }

    CollisionDetectorCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.runtimeUpdateFailed = viewportServices != nullptr && !updatedRuntime;
    result.message = QString("Updated collision detector %1").arg(detectorId);
    return result;
}

CollisionDetectorCommandResult CollisionDetectorCommandController::applyDetectorQueryContract(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& detectorId,
    const CollisionDetectorQueryContractView& contract)
{
    if(detectorId.isEmpty()) {
        return makeFailure("No collision detector selected.");
    }

    CollisionDetectorDocumentFacade documentFacade(document);
    simulation_project::CollisionDetectorDesc* detector =
        documentFacade.findDetector(detectorId.toStdString());
    if(detector == nullptr) {
        return makeFailure("Selected collision detector is not in the project.");
    }

    const QString trimmedName = contract.name.trimmed();
    const std::string effectiveName = trimmedName.isEmpty()
        ? detector->id
        : trimmedName.toStdString();
    if(hasDuplicateDetectorName(document, detector->id, effectiveName)) {
        return makeFailure(QString("Collision detector name is already used: %1")
            .arg(QString::fromStdString(effectiveName)));
    }

    const simulation_project::CollisionDetectorDesc previousDetector = *detector;
    const bool requestContacts = contract.contacts || contract.normals;

    detector->name = effectiveName;
    detector->contacts = requestContacts;
    detector->visualization.showContacts = requestContacts;
    detector->visualization.showNormals = contract.normals;
    detector->nearestPoints = contract.nearest;
    detector->distance = contract.nearest;
    detector->visualization.showNearestPoints = contract.nearest;
    detector->maxContacts = static_cast<int>(std::lround(contract.maxContacts));
    detector->distanceThreshold = contract.distanceThreshold;

    const std::string selectedRole = contract.role.toStdString();
    if(detector->geometryRole != selectedRole) {
        detector->geometryRole = selectedRole;
        detector->geometrySource.clear();
    }

    const bool runtimeOptionsChanged = detectorRuntimeOptionsChanged(previousDetector, *detector);
    bool updatedRuntime = viewportServices == nullptr;
    if(viewportServices != nullptr) {
        if(runtimeOptionsChanged) {
            updatedRuntime = viewportServices->updateCollisionDetectorRuntimeOptions(*detector);
        } else {
            updatedRuntime = true;
        }

        if(updatedRuntime) {
            viewportServices->setActiveCollisionDetector(detectorId);
        }
    }

    CollisionDetectorCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.runtimeUpdateFailed = viewportServices != nullptr && !updatedRuntime;
    result.message = QString("Updated collision detector query contract %1").arg(detectorId);
    return result;
}

CollisionDetectorCommandResult CollisionDetectorCommandController::showOnlyDetectors(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QVector<QString>& detectorIds)
{
    if(detectorIds.empty()) {
        return makeFailure("Select a detector to show.");
    }

    CollisionDetectorDocumentFacade documentFacade(document);
    const std::vector<std::string> documentDetectorIds = documentFacade.detectorIds();
    for(const std::string& documentDetectorId : documentDetectorIds) {
        const QString detectorId = QString::fromStdString(documentDetectorId);
        const bool visible = std::find(detectorIds.begin(), detectorIds.end(), detectorId) != detectorIds.end();
        documentFacade.setDetectorVisible(documentDetectorId, visible);
        if(viewportServices != nullptr) {
            viewportServices->setCollisionDetectorVisible(detectorId, visible);
        }
    }

    CollisionDetectorCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.message = QString("Showing %1 collision detector result(s)").arg(detectorIds.size());
    return result;
}

CollisionDetectorCommandResult CollisionDetectorCommandController::removeDetector(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& detectorId)
{
    if(detectorId.isEmpty()) {
        return makeFailure("No collision detector selected.");
    }
    CollisionDetectorDocumentFacade documentFacade(document);
    if(documentFacade.detectorCount() <= 1) {
        return makeFailure("At least one collision detector is required.");
    }

    if(!documentFacade.removeDetector(detectorId.toStdString())) {
        return makeFailure("Selected collision detector is not in the project.");
    }

    if(viewportServices != nullptr) {
        viewportServices->removeCollisionDetector(detectorId);
    }

    CollisionDetectorCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.message = QString("Removed collision detector %1").arg(detectorId);
    return result;
}

CollisionDetectorCommandResult CollisionDetectorCommandController::bindDetectorPairGenerators(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& detectorId,
    const QVector<CollisionDetectorDraftMemberView>& setA,
    const QVector<CollisionDetectorDraftMemberView>& setB)
{
    if(detectorId.isEmpty()) {
        return makeFailure("No collision detector selected.");
    }

    QString error;
    std::vector<simulation_project::CollisionPairGeneratorDesc> generators =
        makeGenerators(setA, setB, &error);
    if(generators.empty()) {
        return makeFailure(error);
    }

    CollisionDetectorDocumentFacade documentFacade(document);
    simulation_project::CollisionDetectorDesc* detector =
        documentFacade.findDetector(detectorId.toStdString());
    if(detector == nullptr) {
        return makeFailure("Selected collision detector is not in the project.");
    }

    std::vector<simulation_project::CollisionPairGeneratorDesc> newGenerators;
    for(const simulation_project::CollisionPairGeneratorDesc& generator : generators) {
        if(!containsPairGenerator(detector->pairGenerators, generator)
            && !containsPairGenerator(newGenerators, generator)) {
            newGenerators.push_back(generator);
        }
    }
    if(newGenerators.empty()) {
        return makeFailure("Generated model-pair rules already exist on this detector.");
    }

    const std::size_t addedGeneratorCount = newGenerators.size();
    detector->type = "SelectedObjects";
    detector->queryPolicy.clear();
    detector->selectionSetAId.clear();
    detector->selectionSetBId.clear();
    detector->targets.clear();
    for(const simulation_project::CollisionPairGeneratorDesc& generator : newGenerators) {
        detector->pairGenerators.push_back(generator);
    }

    bool updatedRuntime = viewportServices == nullptr;
    if(viewportServices != nullptr) {
        updatedRuntime = viewportServices->rebuildCollisionDetectorsFromDocument(document);
        if(updatedRuntime) {
            viewportServices->setActiveCollisionDetector(detectorId);
        }
    }

    CollisionDetectorCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.runtimeUpdateFailed = viewportServices != nullptr && !updatedRuntime;
    result.message = QString("Added %1 generated model-pair rule(s) to detector %2")
        .arg(static_cast<qulonglong>(addedGeneratorCount))
        .arg(detectorId);
    return result;
}

CollisionDetectorCommandResult CollisionDetectorCommandController::removeDetectorPairGenerators(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& detectorId,
    const QVector<int>& generatorIndexes)
{
    if(detectorId.isEmpty()) {
        return makeFailure("No collision detector selected.");
    }
    if(generatorIndexes.isEmpty()) {
        return makeFailure("Select removable model-pair scope rows first.");
    }

    CollisionDetectorDocumentFacade documentFacade(document);
    simulation_project::CollisionDetectorDesc* detector =
        documentFacade.findDetector(detectorId.toStdString());
    if(detector == nullptr) {
        return makeFailure("Selected collision detector is not in the project.");
    }

    QVector<int> sortedIndexes = generatorIndexes;
    std::sort(sortedIndexes.begin(), sortedIndexes.end(), std::greater<int>());
    int removedCount = 0;
    for(int index : sortedIndexes) {
        if(index < 0 || static_cast<std::size_t>(index) >= detector->pairGenerators.size()) {
            continue;
        }
        detector->pairGenerators.erase(detector->pairGenerators.begin() + index);
        ++removedCount;
    }
    if(removedCount == 0) {
        return makeFailure("Selected rows do not map to detector pair generators.");
    }

    bool updatedRuntime = viewportServices == nullptr;
    if(viewportServices != nullptr) {
        updatedRuntime = viewportServices->rebuildCollisionDetectorsFromDocument(document);
        if(updatedRuntime) {
            viewportServices->setActiveCollisionDetector(detectorId);
        }
    }

    CollisionDetectorCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.runtimeUpdateFailed = viewportServices != nullptr && !updatedRuntime;
    result.message = QString("Removed %1 model-pair rule(s) from detector %2")
        .arg(removedCount)
        .arg(detectorId);
    return result;
}

CollisionDetectorCommandResult CollisionDetectorCommandController::clearDetectorPairScope(
    simulation_project::ProjectDocument& document,
    robot_qt_viewer::RobotQtViewerViewportServices* viewportServices,
    const QString& detectorId)
{
    if(detectorId.isEmpty()) {
        return makeFailure("No collision detector selected.");
    }

    CollisionDetectorDocumentFacade documentFacade(document);
    simulation_project::CollisionDetectorDesc* detector =
        documentFacade.findDetector(detectorId.toStdString());
    if(detector == nullptr) {
        return makeFailure("Selected collision detector is not in the project.");
    }

    const bool hadScope =
        !detector->queryPolicy.empty() ||
        !detector->selectionSetAId.empty() ||
        !detector->selectionSetBId.empty() ||
        !detector->targets.empty() ||
        !detector->pairGenerators.empty();
    if(!hadScope) {
        return makeFailure("Detector pair scope is already empty.");
    }

    detector->type = "SelectedObjects";
    detector->queryPolicy.clear();
    detector->selectionSetAId.clear();
    detector->selectionSetBId.clear();
    detector->targets.clear();
    detector->pairGenerators.clear();

    bool updatedRuntime = viewportServices == nullptr;
    if(viewportServices != nullptr) {
        updatedRuntime = viewportServices->rebuildCollisionDetectorsFromDocument(document);
        if(updatedRuntime) {
            viewportServices->setActiveCollisionDetector(detectorId);
        }
    }

    CollisionDetectorCommandResult result;
    result.success = true;
    result.projectChanged = true;
    result.runtimeUpdateFailed = viewportServices != nullptr && !updatedRuntime;
    result.message = QString("Cleared model-pair scope for detector %1").arg(detectorId);
    return result;
}
