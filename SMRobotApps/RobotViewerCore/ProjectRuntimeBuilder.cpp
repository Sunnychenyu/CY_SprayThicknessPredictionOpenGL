#include "ProjectRuntimeBuilder.h"

#include <AssetCore/AssetManager.h>
#include <Collision/CollisionGeometryBuilder.h>
#include <Collision/PointCloudCollisionBuilder.h>
#include <CustomLog/CustomLog.h>
#include <RenderCore/ModelManager.h>
#include <RenderCore/PointCloudVertex.h>
#include <RobotIO/IRobotLoader.h>
#include <SensorCore/PointCloudProcessing.h>
#include <SensorSimulation/PcdPointCloudLoader.h>
#include <SceneCore/SceneGraph.h>
#include <SimulationProject/CollisionModelSelectionIds.h>
#include <SimulationProject/RuntimePaths.h>
#include <Utility/MathConvert.hpp>
#include <data_path.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace
{
    using namespace collision;

    std::string pathToUtf8(const std::filesystem::path& path)
    {
        return path.generic_u8string();
    }

    struct RuntimeObjMesh
    {
        std::vector<Vec3> vertices;
        std::vector<uint32_t> indices;
    };

    struct CachedSceneObjectCollision
    {
        CollisionGeometryPtr geometry;
        RuntimeObjMesh mesh;
    };

    struct SceneObjectBuildProfile
    {
        double resolveMs = 0.0;
        double assetLoadMs = 0.0;
        double renderBuildMs = 0.0;
        double visualSetupMs = 0.0;
        double meshConvertMs = 0.0;
        double fclBuildMs = 0.0;
        double collisionSetupMs = 0.0;
        double graphRegisterMs = 0.0;
        double totalMs = 0.0;
        bool collisionCacheHit = false;
        std::size_t modelVertices = 0;
        std::size_t modelIndices = 0;
        std::size_t collisionVertices = 0;
        std::size_t collisionIndices = 0;
    };

    std::mutex g_sceneObjectCollisionCacheMutex;
    std::unordered_map<std::string, std::shared_ptr<CachedSceneObjectCollision>> g_sceneObjectCollisionCache;

    double elapsedMilliseconds(const std::chrono::steady_clock::time_point& start)
    {
        return std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    }

    std::string formatDouble(double value)
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(6) << value;
        return stream.str();
    }

    int parallelPoseVariableIndex(const std::string& name)
    {
        static const char* kNames[] = {
            "parallel.pose.x",
            "parallel.pose.y",
            "parallel.pose.z",
            "parallel.pose.roll",
            "parallel.pose.pitch",
            "parallel.pose.yaw"
        };
        for(int i = 0; i < 6; ++i) {
            if(name == kNames[i]) {
                return i;
            }
        }
        return -1;
    }

    int parallelActuatorIndex(const std::string& name)
    {
        const std::string prefix = "parallel.actuator.";
        if(name.rfind(prefix, 0) != 0) {
            return -1;
        }

        const std::string suffix = name.substr(prefix.size());
        if(suffix.size() != 1 || suffix[0] < '1' || suffix[0] > '6') {
            return -1;
        }
        return suffix[0] - '1';
    }

    double parallelPoseValue(const kine::StewartPlatformPose& pose, int index)
    {
        switch(index) {
        case 0:
            return pose.x;
        case 1:
            return pose.y;
        case 2:
            return pose.z;
        case 3:
            return pose.roll;
        case 4:
            return pose.pitch;
        case 5:
            return pose.yaw;
        default:
            return 0.0;
        }
    }

    void setParallelPoseValue(kine::StewartPlatformPose& pose, int index, double value)
    {
        switch(index) {
        case 0:
            pose.x = value;
            break;
        case 1:
            pose.y = value;
            break;
        case 2:
            pose.z = value;
            break;
        case 3:
            pose.roll = value;
            break;
        case 4:
            pose.pitch = value;
            break;
        case 5:
            pose.yaw = value;
            break;
        default:
            break;
        }
    }

    void syncParallelPlatformTransform(RuntimeRobot& runtime)
    {
        runtime.baseTransform = runtime.parallelHomeBaseTransform;
    }

    void applyParallelActuatorJointValues(RuntimeRobot& runtime)
    {
        if(!runtime.instance) {
            return;
        }

        for(std::size_t index = 0; index < runtime.parallelActuatorLengths.size(); ++index) {
            const int dofIndex = runtime.parallelActuatorDofIndices[index];
            if(dofIndex < 0) {
                continue;
            }

            const double travel =
                runtime.parallelActuatorSigns[index] *
                (runtime.parallelActuatorLengths[index] -
                    runtime.parallelActuatorHomeLengths[index]);
            runtime.instance->setJoint(static_cast<std::size_t>(dofIndex), travel);
        }
    }

    bool refreshParallelActuatorLengths(RuntimeRobot& runtime)
    {
        std::string error;
        const bool ok = kine::StewartPlatformKinematics::computeActuatorLengths(
            runtime.parallelGeometry,
            runtime.parallelPose,
            runtime.parallelActuatorLengths,
            &error);
        if(!ok) {
            LOG_WARNING("rs2026") << "Failed to update Stewart actuator lengths: " << error;
        }
        return ok;
    }

    bool setParallelControlValue(RuntimeRobot& runtime, const std::string& name, double value)
    {
        if(!runtime.parallelControlEnabled) {
            return false;
        }

        const int poseIndex = parallelPoseVariableIndex(name);
        if(poseIndex >= 0) {
            setParallelPoseValue(runtime.parallelPose, poseIndex, value);
            if(!refreshParallelActuatorLengths(runtime)) {
                return false;
            }
            applyParallelActuatorJointValues(runtime);
            syncParallelPlatformTransform(runtime);
            return true;
        }

        const int actuatorIndex = parallelActuatorIndex(name);
        if(actuatorIndex >= 0) {
            if(!std::isfinite(value) || value <= 0.0) {
                const std::size_t index = static_cast<std::size_t>(actuatorIndex);
                LOG_WARNING("rs2026") << "Rejected Stewart actuator target length: "
                    << name << "=" << value << " m"
                    << ", home=" << runtime.parallelActuatorHomeLengths[index] << " m"
                    << ", current=" << runtime.parallelActuatorLengths[index] << " m";
                return false;
            }

            std::array<double, 6> requestedLengths = runtime.parallelActuatorLengths;
            requestedLengths[static_cast<std::size_t>(actuatorIndex)] = value;

            kine::StewartPlatformPose solvedPose;
            std::string error;
            if(!kine::StewartPlatformKinematics::solvePoseFromLengths(
                   runtime.parallelGeometry,
                   requestedLengths,
                   runtime.parallelPose,
                   solvedPose,
                   &error)) {
                LOG_WARNING("rs2026") << "Failed to solve Stewart pose from actuator lengths: "
                    << error;
                return false;
            }

            runtime.parallelPose = solvedPose;
            runtime.parallelActuatorLengths = requestedLengths;
            refreshParallelActuatorLengths(runtime);
            applyParallelActuatorJointValues(runtime);
            syncParallelPlatformTransform(runtime);
            return true;
        }

        return false;
    }

    bool getParallelControlValue(const RuntimeRobot& runtime, const std::string& name, double& value)
    {
        if(!runtime.parallelControlEnabled) {
            return false;
        }

        const int poseIndex = parallelPoseVariableIndex(name);
        if(poseIndex >= 0) {
            value = parallelPoseValue(runtime.parallelPose, poseIndex);
            return true;
        }

        const int actuatorIndex = parallelActuatorIndex(name);
        if(actuatorIndex >= 0) {
            value = runtime.parallelActuatorLengths[static_cast<std::size_t>(actuatorIndex)];
            return true;
        }

        return false;
    }

    std::string fileTimestampKey(const std::filesystem::path& path)
    {
        std::error_code ec;
        const auto time = std::filesystem::last_write_time(path, ec);
        if(ec) {
            return "mtime=unknown";
        }
        return "mtime=" + std::to_string(time.time_since_epoch().count());
    }

    std::string fileSizeKey(const std::filesystem::path& path)
    {
        std::error_code ec;
        const auto size = std::filesystem::file_size(path, ec);
        if(ec) {
            return "size=unknown";
        }
        return "size=" + std::to_string(size);
    }

    std::string normalizedPathKey(const std::filesystem::path& path)
    {
        std::error_code ec;
        std::filesystem::path normalized = std::filesystem::weakly_canonical(path, ec);
        if(ec) {
            normalized = std::filesystem::absolute(path, ec);
        }
        if(ec) {
            normalized = path;
        }
        return pathToUtf8(normalized.lexically_normal());
    }

    std::string makeSceneObjectCollisionCacheKey(
        const std::filesystem::path& objectPath,
        double visualScale,
        double collisionScale)
    {
        return normalizedPathKey(objectPath) +
            "|" + fileSizeKey(objectPath) +
            "|" + fileTimestampKey(objectPath) +
            "|visualScale=" + formatDouble(visualScale) +
            "|collisionScale=" + formatDouble(collisionScale);
    }

    std::shared_ptr<CachedSceneObjectCollision> findSceneObjectCollisionCache(const std::string& key)
    {
        std::lock_guard<std::mutex> lock(g_sceneObjectCollisionCacheMutex);
        const auto it = g_sceneObjectCollisionCache.find(key);
        return it == g_sceneObjectCollisionCache.end() ? nullptr : it->second;
    }

    void storeSceneObjectCollisionCache(
        const std::string& key,
        std::shared_ptr<CachedSceneObjectCollision> value)
    {
        if(!value || !value->geometry) {
            return;
        }

        std::lock_guard<std::mutex> lock(g_sceneObjectCollisionCacheMutex);
        g_sceneObjectCollisionCache[key] = std::move(value);
    }

    std::size_t countModelVertices(const assetcore::ModelDesc& modelDesc)
    {
        std::size_t count = 0;
        for(const auto& subMesh : modelDesc.subMeshes()) {
            count += subMesh.geometry.positions.size();
        }
        return count;
    }

    std::size_t countModelIndices(const assetcore::ModelDesc& modelDesc)
    {
        std::size_t count = 0;
        for(const auto& subMesh : modelDesc.subMeshes()) {
            count += subMesh.geometry.indices.size();
        }
        return count;
    }

    std::string formatObjectProfileRow(
        const std::string& objectId,
        const std::string& stage,
        double ms,
        const std::string& detail)
    {
        std::ostringstream stream;
        stream << "| Object " << std::left << std::setw(18) << objectId.substr(0, 18)
            << " " << std::setw(22) << stage.substr(0, 22)
            << " | " << std::right << std::setw(10) << std::fixed << std::setprecision(2) << ms
            << " ms | " << detail;
        return stream.str();
    }

    void logObjectProfileRow(
        const std::string& objectId,
        const std::string& stage,
        double ms,
        const std::string& detail)
    {
        const std::string line = formatObjectProfileRow(objectId, stage, ms, detail);
        std::cout << line << '\n';
        LOG_DEBUG("rs2026") << line;
    }

    void logSceneObjectProfile(
        const simulation_project::SceneObjectDesc& objectDesc,
        const std::filesystem::path& objectPath,
        const SceneObjectBuildProfile& profile)
    {
        const std::string objectId = objectDesc.id.empty() ? objectDesc.name : objectDesc.id;
        logObjectProfileRow(objectId, "asset path resolve", profile.resolveMs, pathToUtf8(objectPath));
        logObjectProfileRow(
            objectId,
            "AssetManager load",
            profile.assetLoadMs,
            "vertices=" + std::to_string(profile.modelVertices) +
                " indices=" + std::to_string(profile.modelIndices));
        logObjectProfileRow(objectId, "RenderCore build", profile.renderBuildMs, "gl buffers included");
        logObjectProfileRow(objectId, "visual setup", profile.visualSetupMs, "");
        logObjectProfileRow(
            objectId,
            "mesh convert",
            profile.meshConvertMs,
            "vertices=" + std::to_string(profile.collisionVertices) +
                " indices=" + std::to_string(profile.collisionIndices));
        logObjectProfileRow(
            objectId,
            "FCL BVH build",
            profile.fclBuildMs,
            profile.collisionCacheHit ? "cache=hit" : "cache=miss");
        logObjectProfileRow(objectId, "collision setup", profile.collisionSetupMs, "");
        logObjectProfileRow(objectId, "graph register", profile.graphRegisterMs, "");
        logObjectProfileRow(
            objectId,
            "total",
            profile.totalMs,
            "collisionCache=" + std::string(profile.collisionCacheHit ? "hit" : "miss"));
    }

    RobotType robotTypeFromDesc(const std::string& sourceType)
    {
        if(sourceType == "simscape" || sourceType == "Simscape") {
            return RobotType::SimscapeRobot;
        }
        return RobotType::URDFRobot;
    }

    void syncCollisionInstance(const RuntimeRobot& runtime)
    {
        if(!runtime.collisionInstance || !runtime.instance) {
            return;
        }

        std::unordered_map<LinkName, Transform3> transforms;

        for(const auto& item : runtime.collisionInstance->model()->linkElements()) {
            transforms[item.first] = runtime.instance->getLinkTransform(item.first);
        }

        runtime.collisionInstance->setLinkTransforms(transforms);
    }

    void setJointIfPresent(robotinstance::RobotInstance& instance, const std::string& jointName, double value)
    {
        const auto& model = instance.getModel();
        auto it = model.jointNameToIndex.find(jointName);
        if(it == model.jointNameToIndex.end()) {
            return;
        }

        const auto& joint = model.joints[static_cast<size_t>(it->second)];
        if(joint.dofIndex >= 0) {
            instance.setJoint(static_cast<size_t>(joint.dofIndex), value);
        }
    }

    RuntimeObjMesh makeObjMesh(assetcore::ModelDesc& modelDesc, double scale)
    {
        RuntimeObjMesh mesh;
        uint32_t vertexOffset = 0;

        for(const auto& subMesh : modelDesc.subMeshes()) {
            for(const auto& position : subMesh.geometry.positions) {
                const glm::vec4 corrected = modelDesc.get_local() * glm::vec4(
                    position.x(),
                    position.y(),
                    position.z(),
                    1.0f);
                mesh.vertices.emplace_back(
                    static_cast<double>(corrected.x) * scale,
                    static_cast<double>(corrected.y) * scale,
                    static_cast<double>(corrected.z) * scale);
            }

            const std::vector<uint32_t>& indices = subMesh.geometry.indices;
            if(!indices.empty()) {
                for(const uint32_t index : indices) {
                    if(index < subMesh.geometry.positions.size()) {
                        mesh.indices.push_back(vertexOffset + index);
                    }
                }
            } else {
                for(size_t i = 0; i + 2 < subMesh.geometry.positions.size(); i += 3) {
                    mesh.indices.push_back(vertexOffset + static_cast<uint32_t>(i));
                    mesh.indices.push_back(vertexOffset + static_cast<uint32_t>(i + 1));
                    mesh.indices.push_back(vertexOffset + static_cast<uint32_t>(i + 2));
                }
            }

            vertexOffset += static_cast<uint32_t>(subMesh.geometry.positions.size());
        }

        return mesh;
    }

    TriangleMeshGeometryData makeTriangleMeshData(const RuntimeObjMesh& mesh)
    {
        TriangleMeshGeometryData data;
        data.vertices = mesh.vertices;
        data.indices = mesh.indices;
        return data;
    }

    CollisionShapeDesc makeSceneObjectShapeDesc(
        const simulation_project::SceneObjectDesc& desc,
        const RuntimeObjMesh& mesh)
    {
        CollisionShapeDesc shape;
        shape.type = CollisionShapeType::TriangleMesh;
        shape.label = desc.id;
        shape.role = CollisionGeometryRole::Exact;
        shape.vertices = mesh.vertices;
        shape.indices = mesh.indices;
        return shape;
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

    CollisionShapeType shapeTypeFromString(const std::string& type)
    {
        std::string lowered = type;
        std::transform(lowered.begin(), lowered.end(), lowered.begin(),
            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

        if(lowered == "box") {
            return CollisionShapeType::Box;
        }
        if(lowered == "sphere") {
            return CollisionShapeType::Sphere;
        }
        if(lowered == "cylinder" || lowered == "spherecover") {
            return CollisionShapeType::Cylinder;
        }
        if(lowered == "capsule") {
            return CollisionShapeType::Capsule;
        }
        if(lowered == "mesh") {
            return CollisionShapeType::TriangleMesh;
        }
        return CollisionShapeType::Unknown;
    }

    Transform3 makeLocalTransform(const simulation_project::TransformDesc& desc)
    {
        Transform3 tf = Transform3::Identity();
        tf.translation() = Vec3(desc.x, desc.y, desc.z);
        tf.linear() =
            Eigen::AngleAxisd(desc.yaw, Vec3::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(desc.pitch, Vec3::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(desc.roll, Vec3::UnitX()).toRotationMatrix();
        return tf;
    }

    RuntimeObjMesh makeOverrideMesh(
        const simulation_project::ObjectCollisionElementOverrideDesc& element,
        const simulation_project::AssetResolveContext& context)
    {
        RuntimeObjMesh mesh;
        if(element.meshPath.empty()) {
            return mesh;
        }

        const std::filesystem::path meshPath = simulation_project::AssetResolver::resolveProjectPath(
            context,
            element.meshPath);
        auto modelDesc = assetcore::AssetManager::instance().loadModel(pathToUtf8(meshPath), 1.0f);
        if(!modelDesc) {
            return mesh;
        }

        uint32_t vertexOffset = 0;
        for(const auto& subMesh : modelDesc->subMeshes()) {
            for(const auto& position : subMesh.geometry.positions) {
                const glm::vec4 corrected = modelDesc->get_local() * glm::vec4(
                    position.x(),
                    position.y(),
                    position.z(),
                    1.0f);
                mesh.vertices.emplace_back(
                    static_cast<double>(corrected.x) * element.meshScale.x,
                    static_cast<double>(corrected.y) * element.meshScale.y,
                    static_cast<double>(corrected.z) * element.meshScale.z);
            }

            for(const uint32_t index : subMesh.geometry.indices) {
                mesh.indices.push_back(vertexOffset + index);
            }

            vertexOffset += static_cast<uint32_t>(subMesh.geometry.positions.size());
        }
        return mesh;
    }

    CollisionGeometryPtr buildOverrideGeometry(
        const simulation_project::ObjectCollisionElementOverrideDesc& element,
        const simulation_project::AssetResolveContext& context,
        CollisionShapeDesc& shape)
    {
        shape.type = shapeTypeFromString(element.type);
        shape.label = element.label.empty() ? element.id : element.label;
        shape.source = element.source;
        shape.role = geometryRoleFromString(element.role);
        shape.localTransform = makeLocalTransform(element.localTransform);
        shape.inflationMargin = element.inflationMargin;
        shape.boxSize = Vec3(element.boxSize.x, element.boxSize.y, element.boxSize.z);
        shape.radius = element.radius;
        shape.length = element.length;

        if(shape.type == CollisionShapeType::Box) {
            return CollisionGeometryBuilder::buildBox(shape.boxSize);
        }
        if(shape.type == CollisionShapeType::Sphere) {
            return CollisionGeometryBuilder::buildSphere(shape.radius);
        }
        if(shape.type == CollisionShapeType::Cylinder || shape.type == CollisionShapeType::Capsule) {
            return CollisionGeometryBuilder::buildCylinder(shape.radius, shape.length);
        }
        if(shape.type == CollisionShapeType::TriangleMesh) {
            RuntimeObjMesh mesh = makeOverrideMesh(element, context);
            shape.vertices = mesh.vertices;
            shape.indices = mesh.indices;
            return CollisionGeometryBuilder::buildTriangleMesh(makeTriangleMeshData(mesh));
        }

        return nullptr;
    }

    std::shared_ptr<rendercore::Material> makeSceneObjectMaterial()
    {
        auto material = std::make_shared<rendercore::Material>();
        material->baseColor = Eigen::Vector4f(0.72f, 0.74f, 0.76f, 1.0f);
        material->specular = Eigen::Vector3f(0.25f, 0.25f, 0.25f);
        material->shininess = 20.0f;
        return material;
    }

    std::shared_ptr<rendercore::Material> makeSceneObjectHighlightMaterial()
    {
        auto material = std::make_shared<rendercore::Material>();
        material->baseColor = Eigen::Vector4f(1.0f, 0.25f, 0.05f, 1.0f);
        material->emissiveColor = Eigen::Vector3f(0.2f, 0.04f, 0.01f);
        material->specular = Eigen::Vector3f(0.35f, 0.25f, 0.2f);
        material->shininess = 32.0f;
        return material;
    }

    void ensureModelMaterial(
        const std::shared_ptr<rendercore::Model>& model,
        const std::shared_ptr<rendercore::Material>& material)
    {
        if(!model) {
            return;
        }

        for(size_t i = 0; i < model->subMeshCount(); ++i) {
            auto& subMesh = model->subMesh(static_cast<unsigned int>(i));
            if(!subMesh.material) {
                subMesh.material = material;
            }
        }
    }

    std::vector<std::shared_ptr<rendercore::Material>> captureModelMaterials(
        const std::shared_ptr<rendercore::Model>& model)
    {
        std::vector<std::shared_ptr<rendercore::Material>> materials;
        if(!model) {
            return materials;
        }

        materials.reserve(model->subMeshCount());
        for(size_t i = 0; i < model->subMeshCount(); ++i) {
            materials.push_back(model->subMesh(static_cast<unsigned int>(i)).material);
        }
        return materials;
    }

    std::vector<collision::Vec3> makePointCloudCollisionPoints(const sensorcore::PointCloud& cloud)
    {
        std::vector<collision::Vec3> points;
        points.reserve(cloud.points.size());
        for(const Eigen::Vector3d& point : cloud.points) {
            points.emplace_back(point.x(), point.y(), point.z());
        }
        return points;
    }

    void updatePointCloudBounds(RuntimeSceneObject& runtime, const sensorcore::PointCloud& cloud)
    {
        runtime.pointCloudBoundsValid = false;
        if(cloud.points.empty()) {
            return;
        }

        runtime.pointCloudLocalBoundsMin = cloud.points.front();
        runtime.pointCloudLocalBoundsMax = cloud.points.front();
        runtime.pointCloudBoundsValid = true;

        for(const Eigen::Vector3d& point : cloud.points) {
            runtime.pointCloudLocalBoundsMin = runtime.pointCloudLocalBoundsMin.cwiseMin(point);
            runtime.pointCloudLocalBoundsMax = runtime.pointCloudLocalBoundsMax.cwiseMax(point);
        }
    }

    std::vector<rendercore::PointCloudVertex> makePointCloudVertices(
        const sensorcore::PointCloud& cloud,
        const simulation_project::ColorDesc& defaultColor)
    {
        std::vector<rendercore::PointCloudVertex> vertices;
        vertices.reserve(cloud.points.size());

        const bool hasColors = cloud.colors.size() == cloud.points.size();
        for(std::size_t index = 0; index < cloud.points.size(); ++index) {
            const Eigen::Vector3d& point = cloud.points[index];
            rendercore::PointCloudVertex vertex;
            vertex.position = glm::vec3(
                static_cast<float>(point.x()),
                static_cast<float>(point.y()),
                static_cast<float>(point.z()));

            if(hasColors) {
                const sensorcore::PointColor& color = cloud.colors[index];
                vertex.color = glm::vec4(color.r, color.g, color.b, color.a);
            } else {
                vertex.color = glm::vec4(
                    static_cast<float>(defaultColor.r),
                    static_cast<float>(defaultColor.g),
                    static_cast<float>(defaultColor.b),
                    static_cast<float>(defaultColor.a));
            }
            vertices.push_back(vertex);
        }
        return vertices;
    }
}

simulation_project::AssetResolveContext ProjectRuntimeBuilder::makeAssetResolveContext(
    const std::filesystem::path& basePath,
    const std::vector<std::string>& assetSearchPaths)
{
    simulation_project::AssetResolveContext context;
    context.projectBasePath = basePath;
    context.sourceRootPath = simulation_project::RuntimePaths::sourceRoot();
    context.dataRootPath = simulation_project::RuntimePaths::dataRoot();
    context.appRootPath = simulation_project::RuntimePaths::applicationRoot();
    context.assetSearchPaths = assetSearchPaths;
    return context;
}

collision::Transform3 ProjectRuntimeBuilder::makeTransform(const simulation_project::TransformDesc& desc)
{
    collision::Transform3 tf = collision::Transform3::Identity();
    tf.translation() = collision::Vec3(desc.x, desc.y, desc.z);
    tf.linear() =
        Eigen::AngleAxisd(desc.yaw, collision::Vec3::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(desc.pitch, collision::Vec3::UnitY()).toRotationMatrix() *
        Eigen::AngleAxisd(desc.roll, collision::Vec3::UnitX()).toRotationMatrix();
    return tf;
}

const simulation_project::ObjectCollisionOverrideDesc* ProjectRuntimeBuilder::findObjectCollisionOverride(
    const simulation_project::CollisionSceneDesc& collisionDesc,
    const std::string& objectId)
{
    for(const simulation_project::ObjectCollisionOverrideDesc& collisionOverride : collisionDesc.objectOverrides) {
        if(collisionOverride.objectId == objectId) {
            return &collisionOverride;
        }
    }
    return nullptr;
}

namespace
{
    std::string activeObjectCollisionModelId(
        const simulation_project::CollisionSceneDesc& collisionDesc,
        const std::string& objectId)
    {
        for(const simulation_project::ObjectCollisionModelSelectionDesc& selection :
            collisionDesc.objectModelSelections) {
            if(selection.objectId == objectId) {
                return simulation_project::normalizeRobotLinkCollisionModelId(selection.activeModelId);
            }
        }

        const simulation_project::ObjectCollisionOverrideDesc* collisionOverride =
            ProjectRuntimeBuilder::findObjectCollisionOverride(collisionDesc, objectId);
        if(collisionOverride != nullptr &&
            collisionOverride->replaceOriginalCollisions &&
            std::any_of(
                collisionOverride->elements.begin(),
                collisionOverride->elements.end(),
                [](const simulation_project::ObjectCollisionElementOverrideDesc& element) {
                    return element.enabled &&
                        !simulation_project::isCoacdCollisionSource(element.source);
                })) {
            return simulation_project::kDefinedInProjectCollisionModelId;
        }
        return simulation_project::kConvertFromVisualCollisionModelId;
    }
}

bool ProjectRuntimeBuilder::appendObjectCollisionOverrideObjects(
    RuntimeSceneObject& runtime,
    const simulation_project::ObjectCollisionOverrideDesc& collisionOverride,
    uint64_t runtimeIdBase,
    const std::filesystem::path& projectBasePath,
    const std::vector<std::string>& assetSearchPaths,
    const std::string& collisionModelId)
{
    const simulation_project::AssetResolveContext context = makeAssetResolveContext(
        projectBasePath,
        assetSearchPaths);

    bool appended = false;
    uint64_t overrideIndex = 1;
    for(const simulation_project::ObjectCollisionElementOverrideDesc& element : collisionOverride.elements) {
        if(!element.enabled) {
            ++overrideIndex;
            continue;
        }
        if(!simulation_project::objectCollisionElementMatchesModelId(
               collisionModelId,
               element.source,
               element.role)) {
            ++overrideIndex;
            continue;
        }

        RuntimeSceneCollisionObject collisionRuntime;
        auto geometry = buildOverrideGeometry(element, context, collisionRuntime.collisionShape);
        if(geometry) {
            collisionRuntime.localTransform = collisionRuntime.collisionShape.localTransform;
            const uint64_t objectId = runtimeIdBase * 100000ull + overrideIndex;
            collisionRuntime.collisionObject = std::make_shared<collision::CollisionObject>(objectId, geometry);
            runtime.collisionObjects.push_back(std::move(collisionRuntime));
            appended = true;
        }
        ++overrideIndex;
    }

    return appended;
}

robot::RobotModel ProjectRuntimeBuilder::loadSingleRobot(
    const std::filesystem::path& path,
    const std::string& sourceType,
    int sourceModelIndex)
{
    const std::string robotPath = pathToUtf8(path);
    auto robots = IRobotLoader::get_robots(robotTypeFromDesc(sourceType), robotPath);
    if(robots.empty()) {
        throw std::runtime_error("Failed to load robot: " + robotPath);
    }
    if(sourceModelIndex < 0 || static_cast<std::size_t>(sourceModelIndex) >= robots.size()) {
        throw std::runtime_error(
            "Robot source model index " + std::to_string(sourceModelIndex) +
            " is out of range for " + robotPath +
            " (models=" + std::to_string(robots.size()) + ")");
    }

    return robots[static_cast<std::size_t>(sourceModelIndex)];
}

void ProjectRuntimeBuilder::applyInitialJoints(
    robotinstance::RobotInstance& instance,
    const std::vector<simulation_project::JointValueDesc>& joints)
{
    for(const auto& joint : joints) {
        setJointIfPresent(instance, joint.jointName, joint.value);
    }
}

void ProjectRuntimeBuilder::updateRobotPose(RuntimeRobot& runtime)
{
    runtime.instance->setBaseTransform(runtime.baseTransform);
    runtime.instance->update();
    runtime.visualBridge->sync();
    syncCollisionInstance(runtime);
}

void ProjectRuntimeBuilder::updateSceneObjectPose(RuntimeSceneObject& runtime)
{
    updateSceneObjectVisualPose(runtime);

    if(runtime.collisionObject) {
        runtime.collisionObject->setTransform(runtime.transform);
    }

    for(RuntimeSceneCollisionObject& collisionObject : runtime.collisionObjects) {
        if(collisionObject.collisionObject) {
            collisionObject.collisionObject->setTransform(runtime.transform * collisionObject.localTransform);
        }
    }
}

void ProjectRuntimeBuilder::updateSceneObjectVisualPose(RuntimeSceneObject& runtime)
{
    if(runtime.visualNode) {
        runtime.visualNode->setLocal(math::eigenToGlm(runtime.transform) * runtime.visualLocal);
    }
}

void ProjectRuntimeBuilder::updatePointCloudPose(RuntimeSceneObject& runtime)
{
    updatePointCloudVisualPose(runtime);

    for(RuntimeSceneCollisionObject& collisionObject : runtime.collisionObjects) {
        if(collisionObject.collisionObject) {
            collisionObject.collisionObject->setTransform(runtime.transform * collisionObject.localTransform);
        }
    }

    if(!runtime.collisionObjects.empty()) {
        runtime.collisionObject = runtime.collisionObjects.front().collisionObject;
        runtime.collisionShape = runtime.collisionObjects.front().collisionShape;
    }
}

void ProjectRuntimeBuilder::updatePointCloudVisualPose(RuntimeSceneObject& runtime)
{
    if(runtime.pointCloudNode) {
        runtime.pointCloudNode->setLocal(math::eigenToGlm(runtime.transform));
    }
}

RuntimeSceneObject ProjectRuntimeBuilder::buildSceneObject(
    const simulation_project::SceneObjectDesc& objectDesc,
    const simulation_project::CollisionSceneDesc& collisionDesc,
    uint64_t runtimeId,
    const std::filesystem::path& projectBasePath,
    const std::vector<std::string>& assetSearchPaths,
    scenecore::SceneGraph& graph,
    bool buildCollision)
{
    const auto totalStart = std::chrono::steady_clock::now();
    SceneObjectBuildProfile profile;
    RuntimeSceneObject runtime;
    runtime.runtimeId = runtimeId;
    runtime.documentId = objectDesc.id;
    runtime.name = objectDesc.name;
    runtime.objectType = objectDesc.objectType;
    runtime.transform = makeTransform(objectDesc.transform);
    runtime.collisionEnabled = objectDesc.collisionEnabled;

    const auto resolveStart = std::chrono::steady_clock::now();
    const std::filesystem::path objectPath = simulation_project::AssetResolver::resolveProjectPath(
        makeAssetResolveContext(projectBasePath, assetSearchPaths),
        objectDesc.sourcePath);
    profile.resolveMs = elapsedMilliseconds(resolveStart);

    const auto assetLoadStart = std::chrono::steady_clock::now();
    auto modelDesc = assetcore::AssetManager::instance().loadModel(
        pathToUtf8(objectPath),
        static_cast<float>(objectDesc.visualScale));
    profile.assetLoadMs = elapsedMilliseconds(assetLoadStart);
    if(!modelDesc) {
        throw std::runtime_error("Failed to load scene object model: " + pathToUtf8(objectPath));
    }
    profile.modelVertices = countModelVertices(*modelDesc);
    profile.modelIndices = countModelIndices(*modelDesc);

    const auto renderBuildStart = std::chrono::steady_clock::now();
    auto renderModel = rendercore::ModelManager::instance().buildModelFromDesc(*modelDesc);
    profile.renderBuildMs = elapsedMilliseconds(renderBuildStart);
    if(!renderModel) {
        throw std::runtime_error("Failed to build scene object model: " + pathToUtf8(objectPath));
    }

    const auto visualSetupStart = std::chrono::steady_clock::now();
    ensureModelMaterial(renderModel, makeSceneObjectMaterial());
    runtime.visualModel = renderModel;
    runtime.originalMaterials = captureModelMaterials(renderModel);
    runtime.highlightMaterial = makeSceneObjectHighlightMaterial();
    runtime.visualNode = std::make_shared<VisibleModelNode>(renderModel);
    runtime.visualNode->setName(objectDesc.id);
    runtime.visualNode->setVisible(objectDesc.visible);
    runtime.visualLocal = modelDesc->get_local();
    updateSceneObjectPose(runtime);
    profile.visualSetupMs = elapsedMilliseconds(visualSetupStart);

    const auto graphRegisterStart = std::chrono::steady_clock::now();
    graph.root()->addChild(runtime.visualNode);
    graph.registerNode(runtime.visualNode);
    profile.graphRegisterMs = elapsedMilliseconds(graphRegisterStart);

    if(buildCollision && objectDesc.collisionEnabled) {
        const auto collisionSetupStart = std::chrono::steady_clock::now();
        const simulation_project::ObjectCollisionOverrideDesc* collisionOverride =
            findObjectCollisionOverride(collisionDesc, objectDesc.id);
        const std::string activeModelId = activeObjectCollisionModelId(collisionDesc, objectDesc.id);
        const bool useVisualCollision =
            simulation_project::isConvertFromVisualCollisionModelId(activeModelId);

        if(useVisualCollision) {
            const double collisionScale = objectDesc.visualScale != 0.0
                ? objectDesc.collisionScale / objectDesc.visualScale
                : 1.0;
            const std::string cacheKey = makeSceneObjectCollisionCacheKey(
                objectPath,
                objectDesc.visualScale,
                objectDesc.collisionScale);
            std::shared_ptr<CachedSceneObjectCollision> cachedCollision =
                findSceneObjectCollisionCache(cacheKey);
            if(cachedCollision && cachedCollision->geometry) {
                profile.collisionCacheHit = true;
            } else {
                cachedCollision = std::make_shared<CachedSceneObjectCollision>();
                const auto meshConvertStart = std::chrono::steady_clock::now();
                cachedCollision->mesh = makeObjMesh(*modelDesc, collisionScale);
                profile.meshConvertMs = elapsedMilliseconds(meshConvertStart);

                const auto fclBuildStart = std::chrono::steady_clock::now();
                cachedCollision->geometry = CollisionGeometryBuilder::buildTriangleMesh(
                    makeTriangleMeshData(cachedCollision->mesh));
                profile.fclBuildMs = elapsedMilliseconds(fclBuildStart);
                storeSceneObjectCollisionCache(cacheKey, cachedCollision);
            }

            profile.collisionVertices = cachedCollision ? cachedCollision->mesh.vertices.size() : 0;
            profile.collisionIndices = cachedCollision ? cachedCollision->mesh.indices.size() : 0;

            if(cachedCollision && cachedCollision->geometry) {
                RuntimeSceneCollisionObject collisionRuntime;
                collisionRuntime.collisionShape = makeSceneObjectShapeDesc(objectDesc, cachedCollision->mesh);
                collisionRuntime.collisionObject = std::make_shared<CollisionObject>(
                    runtimeId,
                    cachedCollision->geometry);
                runtime.collisionObjects.push_back(std::move(collisionRuntime));
            }
        }

        if(collisionOverride != nullptr && !useVisualCollision) {
            appendObjectCollisionOverrideObjects(
                runtime,
                *collisionOverride,
                runtimeId,
                projectBasePath,
                assetSearchPaths,
                activeModelId);
        }

        if(!runtime.collisionObjects.empty()) {
            runtime.collisionObject = runtime.collisionObjects.front().collisionObject;
            runtime.collisionShape = runtime.collisionObjects.front().collisionShape;
            updateSceneObjectPose(runtime);
        }
        profile.collisionSetupMs = elapsedMilliseconds(collisionSetupStart);
    }

    profile.totalMs = elapsedMilliseconds(totalStart);
    logSceneObjectProfile(objectDesc, objectPath, profile);
    return runtime;
}

bool ProjectRuntimeBuilder::ensureSceneObjectCollisionObjects(
    RuntimeSceneObject& runtime,
    const simulation_project::SceneObjectDesc& objectDesc,
    const simulation_project::CollisionSceneDesc& collisionDesc,
    const std::filesystem::path& projectBasePath,
    const std::vector<std::string>& assetSearchPaths)
{
    if(!objectDesc.collisionEnabled) {
        runtime.collisionEnabled = false;
        return false;
    }
    if(!runtime.collisionObjects.empty()) {
        return true;
    }

    const auto totalStart = std::chrono::steady_clock::now();
    SceneObjectBuildProfile profile;
    runtime.collisionEnabled = true;

    const auto resolveStart = std::chrono::steady_clock::now();
    const std::filesystem::path objectPath = simulation_project::AssetResolver::resolveProjectPath(
        makeAssetResolveContext(projectBasePath, assetSearchPaths),
        objectDesc.sourcePath);
    profile.resolveMs = elapsedMilliseconds(resolveStart);

    const auto assetLoadStart = std::chrono::steady_clock::now();
    auto modelDesc = assetcore::AssetManager::instance().loadModel(
        pathToUtf8(objectPath),
        static_cast<float>(objectDesc.visualScale));
    profile.assetLoadMs = elapsedMilliseconds(assetLoadStart);
    if(!modelDesc) {
        throw std::runtime_error("Failed to load scene object collision model: " + pathToUtf8(objectPath));
    }
    profile.modelVertices = countModelVertices(*modelDesc);
    profile.modelIndices = countModelIndices(*modelDesc);

    const auto collisionSetupStart = std::chrono::steady_clock::now();
    const simulation_project::ObjectCollisionOverrideDesc* collisionOverride =
        findObjectCollisionOverride(collisionDesc, objectDesc.id);
    const std::string activeModelId = activeObjectCollisionModelId(collisionDesc, objectDesc.id);
    const bool useVisualCollision =
        simulation_project::isConvertFromVisualCollisionModelId(activeModelId);

    if(useVisualCollision) {
        const double collisionScale = objectDesc.visualScale != 0.0
            ? objectDesc.collisionScale / objectDesc.visualScale
            : 1.0;
        const std::string cacheKey = makeSceneObjectCollisionCacheKey(
            objectPath,
            objectDesc.visualScale,
            objectDesc.collisionScale);
        std::shared_ptr<CachedSceneObjectCollision> cachedCollision =
            findSceneObjectCollisionCache(cacheKey);
        if(cachedCollision && cachedCollision->geometry) {
            profile.collisionCacheHit = true;
        } else {
            cachedCollision = std::make_shared<CachedSceneObjectCollision>();
            const auto meshConvertStart = std::chrono::steady_clock::now();
            cachedCollision->mesh = makeObjMesh(*modelDesc, collisionScale);
            profile.meshConvertMs = elapsedMilliseconds(meshConvertStart);

            const auto fclBuildStart = std::chrono::steady_clock::now();
            cachedCollision->geometry = CollisionGeometryBuilder::buildTriangleMesh(
                makeTriangleMeshData(cachedCollision->mesh));
            profile.fclBuildMs = elapsedMilliseconds(fclBuildStart);
            storeSceneObjectCollisionCache(cacheKey, cachedCollision);
        }

        profile.collisionVertices = cachedCollision ? cachedCollision->mesh.vertices.size() : 0;
        profile.collisionIndices = cachedCollision ? cachedCollision->mesh.indices.size() : 0;

        if(cachedCollision && cachedCollision->geometry) {
            RuntimeSceneCollisionObject collisionRuntime;
            collisionRuntime.collisionShape = makeSceneObjectShapeDesc(objectDesc, cachedCollision->mesh);
            collisionRuntime.collisionObject = std::make_shared<CollisionObject>(
                runtime.runtimeId,
                cachedCollision->geometry);
            runtime.collisionObjects.push_back(std::move(collisionRuntime));
        }
    }

    if(collisionOverride != nullptr && !useVisualCollision) {
        appendObjectCollisionOverrideObjects(
            runtime,
            *collisionOverride,
            runtime.runtimeId,
            projectBasePath,
            assetSearchPaths,
            activeModelId);
    }

    if(!runtime.collisionObjects.empty()) {
        runtime.collisionObject = runtime.collisionObjects.front().collisionObject;
        runtime.collisionShape = runtime.collisionObjects.front().collisionShape;
        updateSceneObjectPose(runtime);
    }

    profile.collisionSetupMs = elapsedMilliseconds(collisionSetupStart);
    profile.totalMs = elapsedMilliseconds(totalStart);
    logSceneObjectProfile(objectDesc, objectPath, profile);
    return !runtime.collisionObjects.empty();
}

RuntimeSceneObject ProjectRuntimeBuilder::buildPointCloud(
    const simulation_project::PointCloudDesc& pointCloudDesc,
    uint64_t runtimeId,
    const std::filesystem::path& projectBasePath,
    const std::vector<std::string>& assetSearchPaths,
    scenecore::SceneGraph& graph)
{
    if(pointCloudDesc.format != "pcd" && pointCloudDesc.format != "PCD") {
        throw std::runtime_error("Unsupported point cloud format: " + pointCloudDesc.format);
    }

    RuntimeSceneObject runtime;
    runtime.runtimeId = runtimeId;
    runtime.documentId = pointCloudDesc.id;
    runtime.name = pointCloudDesc.name;
    runtime.objectType = "pointCloud";
    runtime.transform = makeTransform(pointCloudDesc.transform);
    runtime.collisionEnabled = pointCloudDesc.collision.enabled;

    const std::filesystem::path cloudPath = simulation_project::AssetResolver::resolveProjectPath(
        makeAssetResolveContext(projectBasePath, assetSearchPaths),
        pointCloudDesc.sourcePath);

    sensorsimulation::PcdPointCloudLoader loader;
    sensorsimulation::PcdLoadOptions loadOptions;
    loadOptions.scale = pointCloudDesc.scale;
    loadOptions.maxPoints = static_cast<std::size_t>(std::max(0, pointCloudDesc.visualization.maxRenderPoints));

    sensorsimulation::PcdLoadResult loadResult = loader.load(pathToUtf8(cloudPath), loadOptions);
    if(!loadResult.success) {
        throw std::runtime_error("Failed to load point cloud '" + pointCloudDesc.id + "': " + loadResult.error);
    }

    sensorcore::PointCloud cloud = std::move(loadResult.cloud);
    cloud.sourcePath = pathToUtf8(cloudPath);
    cloud.format = pointCloudDesc.format;
    cloud.sourceFrame = pointCloudDesc.sourceFrame;
    cloud.unitScale = pointCloudDesc.scale;

    if(pointCloudDesc.visualization.voxelDownsampleSize > 0.0) {
        cloud = sensorcore::voxelDownsample(
            cloud,
            pointCloudDesc.visualization.voxelDownsampleSize,
            0);
    }

    updatePointCloudBounds(runtime, cloud);

    runtime.pointCloudNode = std::make_shared<scenecore::PointCloudNode>(pointCloudDesc.id);
    runtime.pointCloudNode->upload(makePointCloudVertices(cloud, pointCloudDesc.visualization.defaultColor));
    runtime.pointCloudBasePointSize = static_cast<float>(pointCloudDesc.visualization.pointSize);
    runtime.pointCloudNode->setPointSize(runtime.pointCloudBasePointSize);
    runtime.pointCloudNode->setVisible(pointCloudDesc.visualization.visible);
    updatePointCloudPose(runtime);

    graph.root()->addChild(runtime.pointCloudNode);
    graph.registerNode(runtime.pointCloudNode);

    if(runtime.collisionEnabled) {
        collision::PointCloudCollisionOptions collisionOptions;
        collisionOptions.voxelSize = pointCloudDesc.collision.voxelSize;
        collisionOptions.inflationMargin = pointCloudDesc.collision.inflationMargin;
        collisionOptions.maxVoxelBoxes = static_cast<std::size_t>(std::max(1, pointCloudDesc.collision.maxProxyBoxes));

        collision::PointCloudCollisionBuilder builder;
        collision::PointCloudCollisionBuildResult buildResult =
            builder.buildVoxelBoxes(makePointCloudCollisionPoints(cloud), collisionOptions);
        if(!buildResult.success) {
            throw std::runtime_error("Failed to build point cloud collision proxy '" +
                pointCloudDesc.id + "': " + buildResult.error);
        }

        LOG_DEBUG("rs2026") << "Point cloud collision proxy build: pointCloud=" << pointCloudDesc.id
            << ", sourcePoints=" << buildResult.sourcePointCount
            << ", skippedPoints=" << buildResult.skippedPointCount
            << ", occupiedVoxels=" << buildResult.occupiedVoxelCount
            << ", geometries=" << buildResult.geometries.size()
            << ", voxelSize=" << collisionOptions.voxelSize
            << ", inflationMargin=" << collisionOptions.inflationMargin
            << ", maxProxyBoxes=" << collisionOptions.maxVoxelBoxes;

        for(std::size_t index = 0; index < buildResult.geometries.size(); ++index) {
            if(!buildResult.geometries[index]) {
                continue;
            }

            RuntimeSceneCollisionObject collisionRuntime;
            collisionRuntime.collisionShape = buildResult.shapes[index];
            collisionRuntime.localTransform = buildResult.shapes[index].localTransform;
            if(collisionRuntime.collisionShape.label.empty()) {
                collisionRuntime.collisionShape.label = pointCloudDesc.id;
            }

            const uint64_t objectId = runtimeId * 100000ull + static_cast<uint64_t>(index + 1);
            collisionRuntime.collisionObject =
                std::make_shared<collision::CollisionObject>(objectId, buildResult.geometries[index]);
            runtime.collisionObjects.push_back(std::move(collisionRuntime));
        }
        LOG_DEBUG("rs2026") << "Point cloud collision objects registered: pointCloud=" << pointCloudDesc.id
            << ", collisionObjects=" << runtime.collisionObjects.size();
        updatePointCloudPose(runtime);
    }

    return runtime;
}

bool ProjectRuntimeBuilder::setJointValue(RuntimeRobot& runtime, const std::string& jointName, double value)
{
    if(setParallelControlValue(runtime, jointName, value)) {
        return true;
    }

    auto it = runtime.model.jointNameToIndex.find(jointName);
    if(it == runtime.model.jointNameToIndex.end()) {
        return false;
    }

    const auto& joint = runtime.model.joints[static_cast<size_t>(it->second)];
    if(joint.dofIndex < 0) {
        return false;
    }

    runtime.instance->setJoint(static_cast<size_t>(joint.dofIndex), value);
    return true;
}

bool ProjectRuntimeBuilder::getJointValue(const RuntimeRobot& runtime, const std::string& jointName, double& value)
{
    if(getParallelControlValue(runtime, jointName, value)) {
        return true;
    }

    auto it = runtime.model.jointNameToIndex.find(jointName);
    if(it == runtime.model.jointNameToIndex.end()) {
        return false;
    }

    const auto& joint = runtime.model.joints[static_cast<size_t>(it->second)];
    if(joint.dofIndex < 0) {
        return false;
    }

    const auto& q = runtime.instance->getState().q;
    if(static_cast<size_t>(joint.dofIndex) >= q.size()) {
        return false;
    }

    value = q[static_cast<size_t>(joint.dofIndex)];
    return true;
}

void ProjectRuntimeBuilder::applyAutoMotion(RuntimeRobot& runtime, double timeSeconds)
{
    if(!runtime.autoMotionEnabled) {
        return;
    }

    if(runtime.parallelControlEnabled) {
        runtime.parallelPose.z =
            runtime.autoMotionAmplitude * std::sin(timeSeconds * runtime.autoMotionSpeed);
        refreshParallelActuatorLengths(runtime);
        applyParallelActuatorJointValues(runtime);
        syncParallelPlatformTransform(runtime);
        return;
    }

    for(const auto& joint : runtime.model.joints) {
        if(joint.dofIndex < 0) {
            continue;
        }

        const double phase = static_cast<double>(joint.dofIndex) * 0.7;
        const double value = runtime.autoMotionAmplitude
            * std::sin(timeSeconds * runtime.autoMotionSpeed + phase);
        runtime.instance->setJoint(static_cast<size_t>(joint.dofIndex), value);
    }
}
