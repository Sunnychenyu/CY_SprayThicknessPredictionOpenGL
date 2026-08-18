#include "ProjectCollisionDetectorRuntime.h"

#include <CustomLog/CustomLog.h>
#include <SimulationProject/CollisionModelSelectionIds.h>
#include <SimulationProject/CollisionTargetRef.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <unordered_set>

using namespace collision;

namespace
{
    constexpr const char* kCollisionDetectorTypeSceneAll = "SceneAll";
    constexpr const char* kCollisionDetectorTypeRobotSelf = "RobotSelf";
    constexpr const char* kCollisionDetectorTypeRobotRobot = "RobotRobot";
    constexpr const char* kCollisionDetectorTypeRobotObject = "RobotObject";
    constexpr const char* kCollisionDetectorTypeRobotObjectGroup = "RobotObjectGroup";
    constexpr const char* kCollisionDetectorTypeObjectObject = "ObjectObject";
    constexpr const char* kCollisionGeneratorTypeLinkLink = "LinkLink";
    constexpr const char* kCollisionGeneratorTypeLinkRobot = "LinkRobot";
    constexpr const char* kCollisionGeneratorTypeLinkObject = "LinkObject";
    constexpr const char* kCollisionGeneratorTypeLinkObjectGroup = "LinkObjectGroup";
    constexpr const char* kCollisionGeneratorTypeObjectGroupObjectGroup = "ObjectGroupObjectGroup";
    constexpr const char* kCollisionQueryPolicyWithinSet = "WithinSet";
    constexpr const char* kCollisionQueryPolicyBetweenSets = "BetweenSets";
    constexpr const char* kCollisionQueryPolicyAllEnabled = "AllEnabled";

    double elapsedMilliseconds(const std::chrono::steady_clock::time_point& start)
    {
        const auto elapsed = std::chrono::steady_clock::now() - start;
        return std::chrono::duration<double, std::milli>(elapsed).count();
    }

    bool isRobotObjectDetectorType(const std::string& type)
    {
        return type == kCollisionDetectorTypeRobotObject ||
            type == kCollisionDetectorTypeRobotObjectGroup;
    }

    bool isLinkObjectGeneratorType(const std::string& type)
    {
        return type == kCollisionGeneratorTypeLinkObject ||
            type == kCollisionGeneratorTypeLinkObjectGroup;
    }

    const RuntimeRobot* findRobot(
        const std::vector<RuntimeRobot>& robots,
        const std::string& id)
    {
        for(const RuntimeRobot& robot : robots) {
            if(robot.documentId == id) {
                return &robot;
            }
        }
        return nullptr;
    }

    const RuntimeSceneObject* findObject(
        const std::vector<RuntimeSceneObject>& objects,
        const std::string& id)
    {
        for(const RuntimeSceneObject& object : objects) {
            if(object.documentId == id) {
                return &object;
            }
        }
        return nullptr;
    }

    const simulation_project::CollisionSelectionSetDesc* findSelectionSet(
        const std::vector<simulation_project::CollisionSelectionSetDesc>& selectionSets,
        const std::string& id)
    {
        for(const simulation_project::CollisionSelectionSetDesc& selectionSet : selectionSets) {
            if(selectionSet.id == id) {
                return &selectionSet;
            }
        }
        return nullptr;
    }

    bool containsName(const std::vector<std::string>& names, const std::string& name)
    {
        return std::find(names.begin(), names.end(), name) != names.end();
    }

    bool linkIncluded(
        const std::string& linkName,
        const std::vector<std::string>& includeLinks,
        const std::vector<std::string>& excludeLinks)
    {
        if(!includeLinks.empty() && !containsName(includeLinks, linkName)) {
            return false;
        }
        return !containsName(excludeLinks, linkName);
    }

    std::pair<std::string, std::string> normalizeLinkPair(
        std::string a,
        std::string b)
    {
        if(b < a) {
            std::swap(a, b);
        }
        return std::make_pair(std::move(a), std::move(b));
    }

    std::unordered_set<std::string> makeExcludedLinkPairs(
        const std::vector<std::string>& excludePairs)
    {
        std::unordered_set<std::string> pairs;
        for(const std::string& pair : excludePairs) {
            const std::size_t separator = pair.find('|') == std::string::npos
                ? pair.find(':')
                : pair.find('|');
            if(separator == std::string::npos) {
                continue;
            }

            auto normalized = normalizeLinkPair(pair.substr(0, separator), pair.substr(separator + 1));
            pairs.insert(normalized.first + "|" + normalized.second);
        }
        return pairs;
    }

    bool isExcludedLinkPair(
        const std::unordered_set<std::string>& excludedPairs,
        const std::string& a,
        const std::string& b)
    {
        const auto normalized = normalizeLinkPair(a, b);
        return excludedPairs.count(normalized.first + "|" + normalized.second) != 0;
    }

    std::unordered_set<std::string> makeAdjacentLinkPairs(const robot::RobotModel& model)
    {
        std::unordered_set<std::string> pairs;
        for(const robot::RobotJoint& joint : model.joints) {
            if(joint.parent.empty() || joint.child.empty()) {
                continue;
            }

            const auto normalized = normalizeLinkPair(joint.parent, joint.child);
            pairs.insert(normalized.first + "|" + normalized.second);
        }
        return pairs;
    }

    CollisionGeometryRole geometryRoleFromString(const std::string& value)
    {
        if(value == "PlanningProxy") {
            return CollisionGeometryRole::PlanningProxy;
        }
        if(value == "VisualizationProxy" || value == "Simplified") {
            return CollisionGeometryRole::Simplified;
        }
        if(value == simulation_project::kCoacdCollisionModelRole) {
            return CollisionGeometryRole::Simplified;
        }
        if(value == "SafetyMargin") {
            return CollisionGeometryRole::SafetyMargin;
        }
        if(value == "SphereCover") {
            return CollisionGeometryRole::SphereCover;
        }
        return CollisionGeometryRole::Exact;
    }

    CollisionVisualizationOptions makeVisualizationOptions(
        const simulation_project::CollisionVisualizationDesc& desc)
    {
        CollisionVisualizationOptions options;
        options.showAllCollisionGeometry = desc.showCollisionGeometry;
        options.showOnlyCollidingObjects = false;
        options.showContacts = desc.showContacts;
        options.showNormals = desc.showNormals;
        options.showNearestPoints = desc.showNearestPoints;
        options.showObjectHighlight = desc.showObjectHighlight;
        options.alpha = desc.alpha;
        options.normalLength = desc.normalLength;
        options.contactPointRadius = desc.contactPointRadius;
        return options;
    }

    void appendRobotObjects(
        const RuntimeRobot& robot,
        const std::vector<std::string>& includeLinks,
        const std::vector<std::string>& excludeLinks,
        std::vector<ObjectID>& objects)
    {
        if(!robot.collisionEnabled || !robot.collisionInstance) {
            return;
        }

        for(const auto& item : robot.collisionInstance->objects()) {
            const auto& object = item.second;
            if(!object) {
                continue;
            }

            const LinkInfo* info = robot.collisionInstance->getLinkInfo(object->id());
            if(info == nullptr || !linkIncluded(info->linkName, includeLinks, excludeLinks)) {
                continue;
            }

            objects.push_back(object->id());
        }
    }

    void appendSceneObject(
        const RuntimeSceneObject& object,
        std::vector<ObjectID>& objects)
    {
        if(!object.collisionEnabled) {
            return;
        }

        if(!object.collisionObjects.empty()) {
            for(const RuntimeSceneCollisionObject& collisionObject : object.collisionObjects) {
                if(collisionObject.collisionObject) {
                    objects.push_back(collisionObject.collisionObject->id());
                }
            }
            return;
        }

        if(object.collisionObject) {
            objects.push_back(object.collisionObject->id());
        }
    }

    void appendRobotLink(
        const RuntimeRobot& robot,
        const std::string& linkName,
        std::vector<ObjectID>& objects)
    {
        appendRobotObjects(robot, { linkName }, {}, objects);
    }

    void appendRobotLinkPairs(
        const RuntimeRobot& robot,
        const std::unordered_set<std::string>& linkPairs,
        std::vector<std::pair<ObjectID, ObjectID>>& pairs)
    {
        for(const std::string& linkPair : linkPairs) {
            const std::size_t separator = linkPair.find('|');
            if(separator == std::string::npos) {
                continue;
            }

            std::vector<ObjectID> objectsA;
            std::vector<ObjectID> objectsB;
            appendRobotLink(robot, linkPair.substr(0, separator), objectsA);
            appendRobotLink(robot, linkPair.substr(separator + 1), objectsB);
            for(ObjectID objectA : objectsA) {
                for(ObjectID objectB : objectsB) {
                    if(objectA != objectB) {
                        pairs.emplace_back(objectA, objectB);
                    }
                }
            }
        }
    }

    void removeRobotLinkPairs(
        const RuntimeRobot& robot,
        const std::unordered_set<std::string>& linkPairs,
        CollisionQueryOptions& options)
    {
        if(linkPairs.empty() || !robot.collisionInstance) {
            return;
        }

        options.includePairs.erase(
            std::remove_if(
                options.includePairs.begin(),
                options.includePairs.end(),
                [&](const std::pair<ObjectID, ObjectID>& pair) {
                    const LinkInfo* infoA = robot.collisionInstance->getLinkInfo(pair.first);
                    const LinkInfo* infoB = robot.collisionInstance->getLinkInfo(pair.second);
                    return infoA != nullptr && infoB != nullptr &&
                        isExcludedLinkPair(linkPairs, infoA->linkName, infoB->linkName);
                }),
            options.includePairs.end());
    }

    void applyDefaultAdjacentLinkExcludes(
        const std::vector<RuntimeRobot>& robots,
        CollisionQueryOptions& options)
    {
        for(const RuntimeRobot& robot : robots) {
            const std::unordered_set<std::string> adjacentPairs = makeAdjacentLinkPairs(robot.model);
            appendRobotLinkPairs(robot, adjacentPairs, options.excludePairs);
            removeRobotLinkPairs(robot, adjacentPairs, options);
        }
    }

    std::vector<ObjectID> collectObjectGroup(
        const std::vector<RuntimeSceneObject>& objects,
        const std::string& groupId)
    {
        std::vector<ObjectID> result;
        for(const RuntimeSceneObject& object : objects) {
            if(object.name == groupId ||
                object.documentId == groupId ||
                object.objectType == groupId ||
                object.objectType + "s" == groupId) {
                appendSceneObject(object, result);
            }
        }
        return result;
    }

    void appendTargetObjects(
        const simulation_project::CollisionTargetRef& target,
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& sourceObjects,
        std::vector<ObjectID>& objects)
    {
        switch(target.kind) {
        case simulation_project::CollisionTargetKind::Robot:
        {
            const RuntimeRobot* robot = findRobot(robots, target.robotId);
            if(robot != nullptr) {
                appendRobotObjects(*robot, target.includeLinks, target.excludeLinks, objects);
            }
            break;
        }
        case simulation_project::CollisionTargetKind::RobotLink:
        {
            const RuntimeRobot* robot = findRobot(robots, target.robotId);
            if(robot != nullptr) {
                appendRobotLink(*robot, target.linkName, objects);
            }
            break;
        }
        case simulation_project::CollisionTargetKind::SceneObject:
        {
            const RuntimeSceneObject* object = findObject(sourceObjects, target.objectId);
            if(object != nullptr) {
                appendSceneObject(*object, objects);
            }
            break;
        }
        case simulation_project::CollisionTargetKind::ObjectGroup:
        {
            std::vector<ObjectID> groupObjects = collectObjectGroup(sourceObjects, target.objectGroupId);
            objects.insert(objects.end(), groupObjects.begin(), groupObjects.end());
            break;
        }
        case simulation_project::CollisionTargetKind::Attachment:
        {
            const RuntimeSceneObject* object = findObject(sourceObjects, target.attachmentId);
            if(object != nullptr) {
                appendSceneObject(*object, objects);
            }
            break;
        }
        case simulation_project::CollisionTargetKind::PointCloud:
        {
            const RuntimeSceneObject* object = findObject(sourceObjects, target.pointCloudId);
            if(object != nullptr) {
                appendSceneObject(*object, objects);
            }
            break;
        }
        case simulation_project::CollisionTargetKind::Unknown:
            break;
        }
    }

    void appendCartesianPairs(
        const std::vector<ObjectID>& a,
        const std::vector<ObjectID>& b,
        std::vector<std::pair<ObjectID, ObjectID>>& pairs)
    {
        for(ObjectID objectA : a) {
            for(ObjectID objectB : b) {
                if(objectA != objectB) {
                    pairs.emplace_back(objectA, objectB);
                }
            }
        }
    }

    void appendCombinations(
        const std::vector<ObjectID>& objects,
        std::vector<std::pair<ObjectID, ObjectID>>& pairs)
    {
        for(std::size_t i = 0; i < objects.size(); ++i) {
            for(std::size_t j = i + 1; j < objects.size(); ++j) {
                pairs.emplace_back(objects[i], objects[j]);
            }
        }
    }

    CollisionQueryOptions makeBaseOptions(
        const simulation_project::CollisionDetectorDesc& desc)
    {
        CollisionQueryOptions options;
        options.geometryRole = geometryRoleFromString(desc.geometryRole);
        options.geometrySource = desc.geometrySource;
        options.filterEnvironmentByGeometryRole = false;
        options.enableContacts =
            desc.contacts ||
            desc.visualization.showContacts ||
            desc.visualization.showNormals ||
            desc.visualization.showObjectHighlight;
        options.enableNearestPoints = desc.nearestPoints;
        options.enableDistance = desc.distance;
        options.maxContacts = static_cast<std::size_t>(std::max(0, desc.maxContacts));
        options.distanceThreshold = desc.distanceThreshold;
        options.scope = CollisionQueryScope::SelectedObjects;
        return options;
    }

    CollisionQueryOptions makeLegacyOptions(
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects,
        const simulation_project::CollisionQueryDesc& desc)
    {
        CollisionQueryOptions options;
        options.geometryRole = CollisionGeometryRole::Exact;
        options.enableContacts = desc.contacts;
        options.enableNearestPoints = desc.nearestPoints;
        options.enableDistance = desc.nearestPoints;
        options.maxContacts = static_cast<std::size_t>(std::max(0, desc.maxContacts));

        for(const auto& pairDesc : desc.pairs) {
            const RuntimeRobot* a = findRobot(robots, pairDesc.robotA);
            const RuntimeRobot* b = findRobot(robots, pairDesc.robotB);
            if(a == nullptr || b == nullptr) {
                continue;
            }

            std::vector<ObjectID> objectsA;
            std::vector<ObjectID> objectsB;
            appendRobotObjects(*a, pairDesc.includeLinksA, {}, objectsA);
            appendRobotObjects(*b, pairDesc.includeLinksB, {}, objectsB);
            appendCartesianPairs(objectsA, objectsB, options.includePairs);
        }

        if(!desc.robotObjectPairs.empty()) {
            for(const auto& pairDesc : desc.robotObjectPairs) {
                if(!pairDesc.enabled) {
                    continue;
                }

                const RuntimeRobot* robot = findRobot(robots, pairDesc.robotId);
                const RuntimeSceneObject* object = findObject(objects, pairDesc.objectId);
                if(robot == nullptr || object == nullptr) {
                    continue;
                }

                std::vector<ObjectID> robotObjects;
                std::vector<ObjectID> sceneObjects;
                appendRobotObjects(*robot, pairDesc.includeRobotLinks, {}, robotObjects);
                appendSceneObject(*object, sceneObjects);
                appendCartesianPairs(robotObjects, sceneObjects, options.includePairs);
            }
        } else {
            for(const RuntimeRobot& robot : robots) {
                std::vector<ObjectID> robotObjects;
                appendRobotObjects(robot, {}, {}, robotObjects);

                for(const RuntimeSceneObject& object : objects) {
                    std::vector<ObjectID> sceneObjects;
                    appendSceneObject(object, sceneObjects);
                    appendCartesianPairs(robotObjects, sceneObjects, options.includePairs);
                }
            }
        }

        options.scope = (!options.includePairs.empty() || !desc.pairs.empty() || !desc.robotObjectPairs.empty())
            ? CollisionQueryScope::SelectedObjects
            : CollisionQueryScope::All;
        return options;
    }

    const simulation_project::CollisionDetectorTargetDesc* firstRobotTarget(
        const simulation_project::CollisionDetectorDesc& desc)
    {
        for(const auto& target : desc.targets) {
            if(!target.robotId.empty()) {
                return &target;
            }
        }
        return nullptr;
    }

    const simulation_project::CollisionDetectorTargetDesc* secondRobotTarget(
        const simulation_project::CollisionDetectorDesc& desc)
    {
        bool foundFirst = false;
        for(const auto& target : desc.targets) {
            if(target.robotId.empty()) {
                continue;
            }
            if(foundFirst) {
                return &target;
            }
            foundFirst = true;
        }
        return nullptr;
    }

    std::vector<ObjectID> collectSceneTargets(
        const simulation_project::CollisionDetectorDesc& desc,
        const std::vector<RuntimeSceneObject>& objects)
    {
        std::vector<ObjectID> result;
        for(const auto& target : desc.targets) {
            const simulation_project::CollisionTargetRef targetRef =
                simulation_project::collisionTargetFromDetectorTarget(target);
            if(targetRef.kind == simulation_project::CollisionTargetKind::SceneObject ||
                targetRef.kind == simulation_project::CollisionTargetKind::ObjectGroup) {
                appendTargetObjects(targetRef, {}, objects, result);
            }
        }
        return result;
    }

    void buildRobotSelfPairs(
        const simulation_project::CollisionDetectorDesc& desc,
        const std::vector<RuntimeRobot>& robots,
        CollisionQueryOptions& options)
    {
        const auto* target = firstRobotTarget(desc);
        const RuntimeRobot* robot = target != nullptr ? findRobot(robots, target->robotId) : nullptr;
        if(robot == nullptr) {
            return;
        }

        std::vector<ObjectID> robotObjects;
        appendRobotObjects(*robot, target->includeLinks, target->excludeLinks, robotObjects);
        appendCombinations(robotObjects, options.includePairs);

        const auto excludedPairs = makeExcludedLinkPairs(desc.excludePairs);
        removeRobotLinkPairs(*robot, excludedPairs, options);
    }

    void buildRobotRobotPairs(
        const simulation_project::CollisionDetectorDesc& desc,
        const std::vector<RuntimeRobot>& robots,
        CollisionQueryOptions& options)
    {
        const auto* targetA = firstRobotTarget(desc);
        const auto* targetB = secondRobotTarget(desc);
        const RuntimeRobot* robotA = targetA != nullptr ? findRobot(robots, targetA->robotId) : nullptr;
        const RuntimeRobot* robotB = targetB != nullptr ? findRobot(robots, targetB->robotId) : nullptr;
        if(robotA == nullptr || robotB == nullptr) {
            return;
        }

        std::vector<ObjectID> objectsA;
        std::vector<ObjectID> objectsB;
        appendRobotObjects(*robotA, targetA->includeLinks, targetA->excludeLinks, objectsA);
        appendRobotObjects(*robotB, targetB->includeLinks, targetB->excludeLinks, objectsB);
        appendCartesianPairs(objectsA, objectsB, options.includePairs);
    }

    void buildRobotObjectPairs(
        const simulation_project::CollisionDetectorDesc& desc,
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects,
        CollisionQueryOptions& options)
    {
        const auto* robotTarget = firstRobotTarget(desc);
        const RuntimeRobot* robot = robotTarget != nullptr ? findRobot(robots, robotTarget->robotId) : nullptr;
        if(robot == nullptr) {
            return;
        }

        std::vector<ObjectID> robotObjects;
        appendRobotObjects(*robot, robotTarget->includeLinks, robotTarget->excludeLinks, robotObjects);
        std::vector<ObjectID> sceneTargets = collectSceneTargets(desc, objects);
        const std::size_t oldPairCount = options.includePairs.size();
        appendCartesianPairs(robotObjects, sceneTargets, options.includePairs);
        LOG_DEBUG("rs2026") << "Robot-object detector pair expansion: detector=" << desc.id
            << ", robot=" << robotTarget->robotId
            << ", includeLinks=" << robotTarget->includeLinks.size()
            << ", excludeLinks=" << robotTarget->excludeLinks.size()
            << ", robotObjects=" << robotObjects.size()
            << ", sceneTargets=" << sceneTargets.size()
            << ", pairsAdded=" << (options.includePairs.size() - oldPairCount)
            << ", totalPairs=" << options.includePairs.size();
    }

    void buildSelectedObjectPairs(
        const simulation_project::CollisionDetectorDesc& desc,
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects,
        CollisionQueryOptions& options)
    {
        std::vector<ObjectID> selectedObjects;
        for(const auto& target : desc.targets) {
            appendTargetObjects(
                simulation_project::collisionTargetFromDetectorTarget(target),
                robots,
                objects,
                selectedObjects);
        }

        appendCombinations(selectedObjects, options.includePairs);
    }

    void appendSelectionSetObjects(
        const simulation_project::CollisionSelectionSetDesc& selectionSet,
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects,
        std::vector<ObjectID>& result)
    {
        if(!selectionSet.enabled) {
            return;
        }

        for(const simulation_project::CollisionSelectionSetMemberDesc& member : selectionSet.members) {
            appendTargetObjects(
                simulation_project::collisionTargetFromSelectionSetMember(member),
                robots,
                objects,
                result);
        }
    }

    bool buildSelectionSetPairs(
        const simulation_project::CollisionDetectorDesc& desc,
        const std::vector<simulation_project::CollisionSelectionSetDesc>& selectionSets,
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects,
        CollisionQueryOptions& options)
    {
        if(desc.queryPolicy.empty()) {
            return false;
        }

        if(desc.queryPolicy == kCollisionQueryPolicyAllEnabled) {
            options.scope = CollisionQueryScope::All;
            return true;
        }

        const simulation_project::CollisionSelectionSetDesc* setA =
            findSelectionSet(selectionSets, desc.selectionSetAId);
        if(setA == nullptr) {
            return true;
        }

        std::vector<ObjectID> objectsA;
        appendSelectionSetObjects(*setA, robots, objects, objectsA);

        if(desc.queryPolicy == kCollisionQueryPolicyWithinSet) {
            appendCombinations(objectsA, options.includePairs);
            return true;
        }

        if(desc.queryPolicy == kCollisionQueryPolicyBetweenSets) {
            const simulation_project::CollisionSelectionSetDesc* setB =
                findSelectionSet(selectionSets, desc.selectionSetBId);
            if(setB == nullptr) {
                return true;
            }

            std::vector<ObjectID> objectsB;
            appendSelectionSetObjects(*setB, robots, objects, objectsB);
            appendCartesianPairs(objectsA, objectsB, options.includePairs);
            return true;
        }

        return false;
    }

    void appendObjectsFromGeneratorSide(
        const std::string& robotId,
        const std::string& linkName,
        const std::string& objectId,
        const std::string& objectGroupId,
        const std::vector<std::string>& includeLinks,
        const std::vector<std::string>& excludeLinks,
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects,
        std::vector<ObjectID>& result)
    {
        appendTargetObjects(
            simulation_project::collisionTargetFromPairGeneratorSide(
                robotId,
                linkName,
                objectId,
                objectGroupId,
                includeLinks,
                excludeLinks),
            robots,
            objects,
            result);
    }

    void buildGeneratorPairs(
        const simulation_project::CollisionPairGeneratorDesc& generator,
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects,
        CollisionQueryOptions& options)
    {
        if(generator.type == kCollisionDetectorTypeSceneAll) {
            options.scope = CollisionQueryScope::All;
            return;
        }

        if(generator.type == kCollisionDetectorTypeRobotSelf) {
            const RuntimeRobot* robot = findRobot(robots, generator.robotId);
            if(robot == nullptr) {
                return;
            }
            std::vector<ObjectID> robotObjects;
            appendRobotObjects(*robot, generator.includeLinksA, generator.excludeLinksA, robotObjects);
            appendCombinations(robotObjects, options.includePairs);
            return;
        }

        std::vector<ObjectID> objectsA;
        std::vector<ObjectID> objectsB;

        if(generator.type == kCollisionGeneratorTypeLinkLink) {
            appendObjectsFromGeneratorSide(
                generator.robotA,
                generator.linkA,
                {},
                {},
                {},
                {},
                robots,
                objects,
                objectsA);
            appendObjectsFromGeneratorSide(
                generator.robotB,
                generator.linkB,
                {},
                {},
                {},
                {},
                robots,
                objects,
                objectsB);
        } else if(generator.type == kCollisionGeneratorTypeLinkRobot) {
            appendObjectsFromGeneratorSide(
                generator.robotId,
                generator.linkName,
                {},
                {},
                {},
                {},
                robots,
                objects,
                objectsA);
            appendObjectsFromGeneratorSide(
                generator.robotB,
                {},
                {},
                {},
                generator.includeLinksB,
                generator.excludeLinksB,
                robots,
                objects,
                objectsB);
        } else if(isLinkObjectGeneratorType(generator.type)) {
            appendObjectsFromGeneratorSide(
                generator.robotId,
                generator.linkName,
                {},
                {},
                {},
                {},
                robots,
                objects,
                objectsA);
            appendObjectsFromGeneratorSide(
                {},
                {},
                generator.objectId,
                generator.objectGroupId,
                {},
                {},
                robots,
                objects,
                objectsB);
        } else if(generator.type == kCollisionDetectorTypeRobotRobot) {
            appendObjectsFromGeneratorSide(
                generator.robotA,
                {},
                {},
                {},
                generator.includeLinksA,
                generator.excludeLinksA,
                robots,
                objects,
                objectsA);
            appendObjectsFromGeneratorSide(
                generator.robotB,
                {},
                {},
                {},
                generator.includeLinksB,
                generator.excludeLinksB,
                robots,
                objects,
                objectsB);
        } else if(isRobotObjectDetectorType(generator.type)) {
            appendObjectsFromGeneratorSide(
                generator.robotId,
                {},
                {},
                {},
                generator.includeLinksA,
                generator.excludeLinksA,
                robots,
                objects,
                objectsA);
            appendObjectsFromGeneratorSide(
                {},
                {},
                generator.objectId,
                generator.objectGroupId,
                {},
                {},
                robots,
                objects,
                objectsB);
        } else if(generator.type == kCollisionDetectorTypeObjectObject) {
            appendObjectsFromGeneratorSide({}, {}, generator.objectA, {}, {}, {}, robots, objects, objectsA);
            appendObjectsFromGeneratorSide({}, {}, generator.objectB, {}, {}, {}, robots, objects, objectsB);
        } else if(generator.type == kCollisionGeneratorTypeObjectGroupObjectGroup) {
            appendObjectsFromGeneratorSide({}, {}, {}, generator.objectGroupA, {}, {}, robots, objects, objectsA);
            appendObjectsFromGeneratorSide({}, {}, {}, generator.objectGroupB, {}, {}, robots, objects, objectsB);
        }

        appendCartesianPairs(objectsA, objectsB, options.includePairs);
    }

    const LinkInfo* findLinkInfo(
        const std::vector<RuntimeRobot>& robots,
        ObjectID objectId)
    {
        for(const RuntimeRobot& robot : robots) {
            if(robot.collisionInstance) {
                const LinkInfo* info = robot.collisionInstance->getLinkInfo(objectId);
                if(info != nullptr) {
                    return info;
                }
            }
        }
        return nullptr;
    }

    const RuntimeRobot* findRobotForObject(
        const std::vector<RuntimeRobot>& robots,
        ObjectID objectId)
    {
        for(const RuntimeRobot& robot : robots) {
            if(robot.collisionInstance && robot.collisionInstance->getLinkInfo(objectId) != nullptr) {
                return &robot;
            }
        }
        return nullptr;
    }

    void appendAllowedBodyObjects(
        const std::string& body,
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects,
        std::vector<ObjectID>& result)
    {
        if(body.empty()) {
            return;
        }

        const RuntimeSceneObject* sceneObject = findObject(objects, body);
        if(sceneObject != nullptr) {
            appendSceneObject(*sceneObject, result);
            return;
        }

        const RuntimeRobot* robot = findRobot(robots, body);
        if(robot != nullptr) {
            appendRobotObjects(*robot, {}, {}, result);
            return;
        }

        std::size_t separator = body.find('/');
        if(separator == std::string::npos) {
            separator = body.find(':');
        }
        if(separator == std::string::npos) {
            separator = body.find('|');
        }
        if(separator != std::string::npos) {
            const RuntimeRobot* scopedRobot = findRobot(robots, body.substr(0, separator));
            if(scopedRobot != nullptr) {
                appendRobotLink(*scopedRobot, body.substr(separator + 1), result);
            }
        }
    }

    void applyAllowedCollisionPairs(
        const std::vector<simulation_project::AllowedCollisionPairDesc>& allowedPairs,
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects,
        CollisionQueryOptions& options)
    {
        for(const simulation_project::AllowedCollisionPairDesc& pairDesc : allowedPairs) {
            if(!pairDesc.enabled) {
                continue;
            }

            std::vector<ObjectID> objectsA;
            std::vector<ObjectID> objectsB;
            appendAllowedBodyObjects(pairDesc.bodyA, robots, objects, objectsA);
            appendAllowedBodyObjects(pairDesc.bodyB, robots, objects, objectsB);
            appendCartesianPairs(objectsA, objectsB, options.excludePairs);
        }
    }

    bool pairMatchesLinkFilter(
        const simulation_project::CollisionPairFilterDesc& filter,
        const std::vector<RuntimeRobot>& robots,
        const std::pair<ObjectID, ObjectID>& pair)
    {
        const LinkInfo* infoA = findLinkInfo(robots, pair.first);
        const LinkInfo* infoB = findLinkInfo(robots, pair.second);
        const RuntimeRobot* robotA = findRobotForObject(robots, pair.first);
        const RuntimeRobot* robotB = findRobotForObject(robots, pair.second);
        if(infoA == nullptr || infoB == nullptr || robotA == nullptr || robotB == nullptr) {
            return false;
        }

        const bool direct =
            (filter.robotA.empty() || filter.robotA == robotA->documentId) &&
            (filter.linkA.empty() || filter.linkA == infoA->linkName) &&
            (filter.robotB.empty() || filter.robotB == robotB->documentId) &&
            (filter.linkB.empty() || filter.linkB == infoB->linkName);
        const bool reversed =
            (filter.robotA.empty() || filter.robotA == robotB->documentId) &&
            (filter.linkA.empty() || filter.linkA == infoB->linkName) &&
            (filter.robotB.empty() || filter.robotB == robotA->documentId) &&
            (filter.linkB.empty() || filter.linkB == infoA->linkName);
        return direct || reversed;
    }

    void applyPairFilters(
        const simulation_project::CollisionDetectorDesc& desc,
        const std::vector<RuntimeRobot>& robots,
        CollisionQueryOptions& options)
    {
        for(const simulation_project::CollisionPairFilterDesc& filter : desc.pairFilters) {
            if(filter.type == "ExcludeLinkPairs") {
                options.includePairs.erase(
                    std::remove_if(
                        options.includePairs.begin(),
                        options.includePairs.end(),
                        [&](const std::pair<ObjectID, ObjectID>& pair) {
                            return pairMatchesLinkFilter(filter, robots, pair);
                        }),
                    options.includePairs.end());
            } else if(filter.type == "IncludeOnlyLinkPairs") {
                options.includePairs.erase(
                    std::remove_if(
                        options.includePairs.begin(),
                        options.includePairs.end(),
                        [&](const std::pair<ObjectID, ObjectID>& pair) {
                            return !pairMatchesLinkFilter(filter, robots, pair);
                        }),
                    options.includePairs.end());
            }
        }
    }

    ProjectCollisionDetectorRuntime makeRuntimeDetector(
        const simulation_project::CollisionDetectorDesc& desc,
        const std::vector<simulation_project::CollisionSelectionSetDesc>& selectionSets,
        const std::vector<simulation_project::AllowedCollisionPairDesc>& allowedPairs,
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects)
    {
        ProjectCollisionDetectorRuntime runtime;
        runtime.id = desc.id;
        runtime.name = desc.name.empty() ? desc.id : desc.name;
        runtime.type = desc.type;
        runtime.enabled = desc.enabled;
        runtime.visible = desc.visualization.visible;
        runtime.options = makeBaseOptions(desc);
        runtime.visualization = makeVisualizationOptions(desc.visualization);

        const bool usesSelectionSetPolicy = buildSelectionSetPairs(desc, selectionSets, robots, objects, runtime.options);
        if(usesSelectionSetPolicy) {
            applyDefaultAdjacentLinkExcludes(robots, runtime.options);
            applyPairFilters(desc, robots, runtime.options);
        } else if(!desc.pairGenerators.empty()) {
            for(const simulation_project::CollisionPairGeneratorDesc& generator : desc.pairGenerators) {
                buildGeneratorPairs(generator, robots, objects, runtime.options);
            }
            applyPairFilters(desc, robots, runtime.options);
        } else if(desc.type == kCollisionDetectorTypeSceneAll) {
            runtime.options.scope = CollisionQueryScope::All;
            applyDefaultAdjacentLinkExcludes(robots, runtime.options);
        } else if(desc.type == kCollisionDetectorTypeRobotSelf) {
            buildRobotSelfPairs(desc, robots, runtime.options);
            applyDefaultAdjacentLinkExcludes(robots, runtime.options);
        } else if(desc.type == kCollisionDetectorTypeRobotRobot) {
            buildRobotRobotPairs(desc, robots, runtime.options);
        } else if(isRobotObjectDetectorType(desc.type)) {
            buildRobotObjectPairs(desc, robots, objects, runtime.options);
        } else if(desc.type == kCollisionDetectorTypeObjectObject) {
            appendCombinations(collectSceneTargets(desc, objects), runtime.options.includePairs);
        } else {
            buildSelectedObjectPairs(desc, robots, objects, runtime.options);
        }

        if(runtime.options.scope != CollisionQueryScope::All) {
            runtime.options.scope = CollisionQueryScope::SelectedObjects;
        }
        applyAllowedCollisionPairs(allowedPairs, robots, objects, runtime.options);
        LOG_DEBUG("rs2026") << "Collision detector runtime built: detector=" << desc.id
            << ", type=" << desc.type
            << ", queryPolicy=" << desc.queryPolicy
            << ", targets=" << desc.targets.size()
            << ", pairGenerators=" << desc.pairGenerators.size()
            << ", includePairs=" << runtime.options.includePairs.size()
            << ", excludePairs=" << runtime.options.excludePairs.size()
            << ", enabled=" << runtime.enabled
            << ", visible=" << runtime.visible
            << ", nearestPoints=" << runtime.options.enableNearestPoints
            << ", distance=" << runtime.options.enableDistance;
        return runtime;
    }
}

std::vector<ProjectCollisionDetectorRuntime> ProjectCollisionDetectorBuilder::build(
    const simulation_project::ProjectDocument& document,
    const std::vector<RuntimeRobot>& robots,
    const std::vector<RuntimeSceneObject>& objects)
{
    const auto buildStart = std::chrono::steady_clock::now();
    std::vector<ProjectCollisionDetectorRuntime> result;

    if(document.collision.detectors.empty()) {
        ProjectCollisionDetectorRuntime runtime;
        runtime.id = "default_collision";
        runtime.name = "Default Collision";
        runtime.type = "LegacyCollisionQuery";
        runtime.enabled = document.collision.query.enabled;
        runtime.options = makeLegacyOptions(robots, objects, document.collision.query);
        applyAllowedCollisionPairs(document.collision.allowedPairs, robots, objects, runtime.options);
        runtime.visualization = makeVisualizationOptions(document.collision.visualization);
        result.push_back(std::move(runtime));
        LOG_DEBUG("rs2026") << "ProjectCollisionDetectorBuilder build: source=legacy"
            << ", detectors=" << result.size()
            << ", includePairs=" << result.front().options.includePairs.size()
            << ", elapsedMs=" << elapsedMilliseconds(buildStart);
        return result;
    }

    result.reserve(document.collision.detectors.size());
    for(const simulation_project::CollisionDetectorDesc& detector : document.collision.detectors) {
        result.push_back(makeRuntimeDetector(
            detector,
            document.collision.selectionSets,
            document.collision.allowedPairs,
            robots,
            objects));
    }

    std::size_t includePairCount = 0;
    for(const ProjectCollisionDetectorRuntime& detector : result) {
        includePairCount += detector.options.includePairs.size();
    }
    LOG_DEBUG("rs2026") << "ProjectCollisionDetectorBuilder build: source=detectors"
        << ", detectors=" << result.size()
        << ", includePairs=" << includePairCount
        << ", elapsedMs=" << elapsedMilliseconds(buildStart);
    return result;
}
