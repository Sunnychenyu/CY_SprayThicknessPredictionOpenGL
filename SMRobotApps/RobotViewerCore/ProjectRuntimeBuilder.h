#pragma once

#include "ProjectRuntimeTypes.h"

#include <SimulationProject/AssetResolver.h>
#include <SimulationProject/ProjectDocument.h>

#include <filesystem>
#include <string>
#include <vector>

namespace scenecore
{
    class SceneGraph;
}

class ProjectRuntimeBuilder
{
public:
    static simulation_project::AssetResolveContext makeAssetResolveContext(
        const std::filesystem::path& basePath,
        const std::vector<std::string>& assetSearchPaths);

    static collision::Transform3 makeTransform(const simulation_project::TransformDesc& desc);

    static robot::RobotModel loadSingleRobot(
        const std::filesystem::path& path,
        const std::string& sourceType);

    static void applyInitialJoints(
        robotinstance::RobotInstance& instance,
        const std::vector<simulation_project::JointValueDesc>& joints);

    static void updateRobotPose(RuntimeRobot& runtime);

    static void updateSceneObjectPose(RuntimeSceneObject& runtime);

    static void updateSceneObjectVisualPose(RuntimeSceneObject& runtime);

    static void updatePointCloudPose(RuntimeSceneObject& runtime);

    static void updatePointCloudVisualPose(RuntimeSceneObject& runtime);

    static const simulation_project::ObjectCollisionOverrideDesc* findObjectCollisionOverride(
        const simulation_project::CollisionSceneDesc& collisionDesc,
        const std::string& objectId);

    static bool appendObjectCollisionOverrideObjects(
        RuntimeSceneObject& runtime,
        const simulation_project::ObjectCollisionOverrideDesc& collisionOverride,
        uint64_t runtimeIdBase,
        const std::filesystem::path& projectBasePath,
        const std::vector<std::string>& assetSearchPaths,
        const std::string& collisionModelId);

    static RuntimeSceneObject buildSceneObject(
        const simulation_project::SceneObjectDesc& objectDesc,
        const simulation_project::CollisionSceneDesc& collisionDesc,
        uint64_t runtimeId,
        const std::filesystem::path& projectBasePath,
        const std::vector<std::string>& assetSearchPaths,
        scenecore::SceneGraph& graph,
        bool buildCollision = true);

    static bool ensureSceneObjectCollisionObjects(
        RuntimeSceneObject& runtime,
        const simulation_project::SceneObjectDesc& objectDesc,
        const simulation_project::CollisionSceneDesc& collisionDesc,
        const std::filesystem::path& projectBasePath,
        const std::vector<std::string>& assetSearchPaths);

    static RuntimeSceneObject buildPointCloud(
        const simulation_project::PointCloudDesc& pointCloudDesc,
        uint64_t runtimeId,
        const std::filesystem::path& projectBasePath,
        const std::vector<std::string>& assetSearchPaths,
        scenecore::SceneGraph& graph);

    static bool setJointValue(RuntimeRobot& runtime, const std::string& jointName, double value);

    static bool getJointValue(const RuntimeRobot& runtime, const std::string& jointName, double& value);

    static void applyAutoMotion(RuntimeRobot& runtime, double timeSeconds);
};
