#include "RobotCollisionOverrideApplier.h"

#include "ProjectRuntimeBuilder.h"

#include <RobotIO/RobotCollisionOverrideIo.h>

#include <SimulationProject/AssetResolver.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <unordered_set>
#include <utility>

namespace
{
    std::string pathToUtf8(const std::filesystem::path& path)
    {
        return path.generic_u8string();
    }

    std::string lowerAscii(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        return value;
    }

    robot::RobotGeometryType geometryTypeFromString(const std::string& type)
    {
        const std::string lowered = lowerAscii(type);
        if(lowered == "box") {
            return robot::RobotGeometryType::Box;
        }
        if(lowered == "sphere") {
            return robot::RobotGeometryType::Sphere;
        }
        if(lowered == "cylinder" || lowered == "capsule" || lowered == "spherecover") {
            return robot::RobotGeometryType::Cylinder;
        }
        if(lowered == "mesh") {
            return robot::RobotGeometryType::Mesh;
        }
        return robot::RobotGeometryType::Unknown;
    }

    Eigen::Isometry3d makeTransform(const simulation_project::TransformDesc& desc)
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.translation() = Eigen::Vector3d(desc.x, desc.y, desc.z);
        transform.linear() =
            Eigen::AngleAxisd(desc.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(desc.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(desc.roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        return transform;
    }

    Eigen::Vector3d makeVec3(const simulation_project::Vec3Desc& desc)
    {
        return Eigen::Vector3d(desc.x, desc.y, desc.z);
    }

    simulation_project::Vec3Desc fromRobotIoVec3(const robotio::Vec3Desc& desc)
    {
        return simulation_project::Vec3Desc{ desc.x, desc.y, desc.z };
    }

    simulation_project::TransformDesc fromRobotIoTransform(const robotio::TransformDesc& desc)
    {
        simulation_project::TransformDesc result;
        result.x = desc.x;
        result.y = desc.y;
        result.z = desc.z;
        result.roll = desc.roll;
        result.pitch = desc.pitch;
        result.yaw = desc.yaw;
        return result;
    }

    simulation_project::CollisionElementOverrideDesc fromRobotIoCollisionElement(
        const robotio::CollisionElementOverrideDesc& desc)
    {
        simulation_project::CollisionElementOverrideDesc result;
        result.id = desc.id;
        result.linkName = desc.linkName;
        result.label = desc.label;
        result.type = desc.type;
        result.role = desc.role;
        result.enabled = desc.enabled;
        result.localTransform = fromRobotIoTransform(desc.localTransform);
        result.boxSize = fromRobotIoVec3(desc.boxSize);
        result.radius = desc.radius;
        result.length = desc.length;
        result.meshPath = desc.meshPath;
        result.meshScale = fromRobotIoVec3(desc.meshScale);
        result.inflationMargin = desc.inflationMargin;
        result.source = desc.source;
        return result;
    }

    simulation_project::RobotCollisionOverrideDesc fromRobotIoCollisionOverride(
        const robotio::RobotCollisionOverrideDesc& desc)
    {
        simulation_project::RobotCollisionOverrideDesc result;
        result.robotId = desc.robotId;
        result.sourceRobotPath = desc.sourceRobotPath;
        result.overridePath = desc.overridePath;
        result.replaceOriginalCollisions = desc.replaceOriginalCollisions;
        result.elements.reserve(desc.elements.size());
        for(const robotio::CollisionElementOverrideDesc& element : desc.elements) {
            result.elements.push_back(fromRobotIoCollisionElement(element));
        }
        return result;
    }

    const simulation_project::RobotCollisionOverrideDesc* findOverride(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId)
    {
        for(const simulation_project::RobotCollisionOverrideDesc& collisionOverride : document.collision.robotOverrides) {
            if(collisionOverride.robotId == robotId) {
                return &collisionOverride;
            }
        }
        return nullptr;
    }

    std::string resolveMeshPath(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& projectBasePath,
        const std::string& meshPath)
    {
        if(meshPath.empty()) {
            return {};
        }

        return pathToUtf8(simulation_project::AssetResolver::resolveProjectPath(
            ProjectRuntimeBuilder::makeAssetResolveContext(projectBasePath, document.assetSearchPaths),
            meshPath));
    }

    bool loadSidecarOverride(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& projectBasePath,
        const simulation_project::RobotCollisionOverrideDesc& collisionOverride,
        simulation_project::RobotCollisionOverrideDesc& loadedOverride)
    {
        if(collisionOverride.overridePath.empty()) {
            return false;
        }

        const std::filesystem::path path = simulation_project::AssetResolver::resolveProjectPath(
            ProjectRuntimeBuilder::makeAssetResolveContext(projectBasePath, document.assetSearchPaths),
            collisionOverride.overridePath);
        std::string error;
        robotio::RobotCollisionOverrideDesc robotIoOverride;
        if(!robotio::loadRobotCollisionOverride(path, robotIoOverride, &error)) {
            return false;
        }
        loadedOverride = fromRobotIoCollisionOverride(robotIoOverride);

        if(loadedOverride.robotId.empty()) {
            loadedOverride.robotId = collisionOverride.robotId;
        }
        if(loadedOverride.robotId != collisionOverride.robotId) {
            return false;
        }
        return true;
    }
}

void RobotCollisionOverrideApplier::apply(
    const simulation_project::ProjectDocument& document,
    const simulation_project::RobotDesc& robotDesc,
    const std::filesystem::path& projectBasePath,
    robot::RobotModel& model)
{
    const simulation_project::RobotCollisionOverrideDesc* collisionOverride = findOverride(document, robotDesc.id);
    if(collisionOverride == nullptr) {
        return;
    }

    simulation_project::RobotCollisionOverrideDesc sidecarOverride;
    if(loadSidecarOverride(document, projectBasePath, *collisionOverride, sidecarOverride)) {
        collisionOverride = &sidecarOverride;
    }

    if(collisionOverride->replaceOriginalCollisions) {
        std::unordered_set<std::string> linksToClear;
        for(const simulation_project::CollisionElementOverrideDesc& element : collisionOverride->elements) {
            linksToClear.insert(element.linkName);
        }

        for(const std::string& linkName : linksToClear) {
            auto linkIt = model.links.find(linkName);
            if(linkIt != model.links.end()) {
                linkIt->second.collisions.clear();
            }
        }
    }

    for(const simulation_project::CollisionElementOverrideDesc& element : collisionOverride->elements) {
        auto linkIt = model.links.find(element.linkName);
        if(linkIt == model.links.end()) {
            continue;
        }

        robot::RobotCollisionGeometry geometry;
        geometry.partUid = element.id;
        geometry.T_part = makeTransform(element.localTransform);
        geometry.type = geometryTypeFromString(element.type);
        geometry.meshPath = resolveMeshPath(document, projectBasePath, element.meshPath);
        geometry.meshScale = makeVec3(element.meshScale);
        geometry.boxSize = makeVec3(element.boxSize);
        geometry.radius = element.radius;
        geometry.length = element.length;
        geometry.role = element.role;
        geometry.enabled = element.enabled;
        geometry.inflationMargin = element.inflationMargin;
        geometry.source = element.source.empty() ? "projectOverride" : element.source;

        if(geometry.type != robot::RobotGeometryType::Unknown) {
            linkIt->second.collisions.push_back(std::move(geometry));
        }
    }
}
