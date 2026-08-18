#include "ProjectScene.h"

#include "ProjectCollisionDetectorRuntime.h"
#include "ProjectRuntimeBuilder.h"
#include "ProjectRuntimeTypes.h"
#include "ProjectScenePickingService.h"
#include "RobotCollisionModelInspector.h"
#include "RobotCollisionOverrideApplier.h"
#include "RobotCollisionProxyGenerator.h"

#include <glad/glad.h>

#include <AssetCore/AssetManager.h>
#include <Collision/CollisionDebugDrawBuilder.h>
#include <CameraCore/CameraFactory.h>
#include <Collision/CollisionGeometryBuilder.h>
#include <Collision/CollisionScene.h>
#include <Collision/RobotCollisionInstance.h>
#include <Collision/RobotCollisionModel.h>
#include <CustomLog/CustomLog.h>
#include <GLRuntime/GLRuntime.h>
#include <RenderCore/Material.h>
#include <RenderCore/Model.h>
#include <RenderCore/ModelManager.h>
#include <RenderCore/ShaderLibrary.h>
#include <RobotCore/RobotModel.h>
#include <RobotInstance/RobotInstance.h>
#include <RobotRenderBridge/CollisionRenderBridge.h>
#include <RobotRenderBridge/MountedAttachmentVisualBridge.h>
#include <RobotRenderBridge/RobotVisualBridge.h>
#include <SceneCore/CameraNode.h>
#include <SceneCore/CollisionOverlayRenderConfig.h>
#include <SceneCore/DefaultLighting.h>
#include <SceneCore/ModelNode.h>
#include <SceneCore/Renderer.h>
#include <SceneCore/RenderPass/AxisPass.h>
#include <SceneCore/RenderPass/GridPass.h>
#include <SceneCore/RenderPass/MeshPass.h>
#include <SceneCore/RenderPass/PointCloudPass.h>
#include <SceneCore/RenderPass/PrimitivePass.h>
#include <SceneCore/RenderPass/TrajectoryPass.h>
#include <SimulationProject/AssetResolver.h>
#include <SimulationProject/CollisionModelSelectionIds.h>
#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectIo.h>
#include <SimulationProject/RuntimePaths.h>
#include <SimulationProject/ProjectV3View.h>
#include <SimulationRuntime/AttachmentCollisionPolicy.h>
#include <SimulationRuntime/ProjectSelectionState.h>
#include <SimulationRuntime/RuntimeMountedAttachment.h>
#include <Utility/MathConvert.hpp>
#include <data_path.h>

#include <Eigen/Geometry>

#include <glm/gtc/matrix_transform.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace collision;

namespace
{
    constexpr double kPi = 3.14159265358979323846;
    constexpr float kCollisionPreviewRed = 0.20f;
    constexpr float kCollisionPreviewGreen = 0.70f;
    constexpr float kCollisionPreviewBlue = 1.00f;
    constexpr float kCollisionPreviewAlpha = 0.18f;
    constexpr float kCollisionPreviewVisibleAlpha = 0.34f;
    constexpr float kCollisionPreviewBlend = 0.34f;
    constexpr float kCollisionPreviewVisualDarken = 0.74f;

    Eigen::Vector3f gammaToLinear(const Eigen::Vector3f& color)
    {
        return color.cwiseMax(0.0f).cwiseMin(1.0f).array().pow(2.2f).matrix();
    }

    Eigen::Vector4f regularCollisionGeometryMaterialColor(float alpha)
    {
        return Eigen::Vector4f(0.2f, 0.7f, 1.0f, alpha);
    }

    double elapsedMilliseconds(const std::chrono::steady_clock::time_point& start)
    {
        const auto elapsed = std::chrono::steady_clock::now() - start;
        return std::chrono::duration<double, std::milli>(elapsed).count();
    }

    std::string formatProfileRow(const std::string& stage, double ms, const std::string& detail)
    {
        std::ostringstream out;
        out << "| " << std::left << std::setw(32) << stage
            << " | " << std::right << std::setw(10) << std::fixed << std::setprecision(2) << ms
            << " ms | " << detail;
        return out.str();
    }

    void logProfileRow(const std::string& stage, double ms, const std::string& detail)
    {
        const std::string line = formatProfileRow(stage, ms, detail);
        std::cout << line << "\n";
        LOG_DEBUG("rs2026") << line;
    }

    std::string pathToUtf8(const std::filesystem::path& path)
    {
        return path.generic_u8string();
    }

    std::string collisionVariantSelectionKey(
        const std::string& robotId,
        const std::string& linkName)
    {
        return robotId + "|" + linkName;
    }

    std::string runtimeCollisionVariantSelectionKey(
        int robotInstance,
        const std::string& linkName)
    {
        return std::to_string(robotInstance) + "|" + linkName;
    }

    struct VisibleCollisionVariantFilter
    {
        std::string robotId;
        std::string linkName;
        std::string variantId;
        std::unordered_set<std::string> elementNames;
    };

    struct CollisionGeometryOverlayCache
    {
        bool valid = false;
        std::uint64_t version = 0;
        std::string key;
        CollisionDebugDrawData data;

        void clear()
        {
            valid = false;
            key.clear();
            data.clear();
        }
    };

    std::filesystem::path pathFromUtf8(const std::string& path)
    {
        return std::filesystem::u8path(path);
    }

    struct ObjMesh
    {
        std::vector<Vec3> vertices;
        std::vector<uint32_t> indices;
    };

    struct MeshBounds
    {
        Vec3 min = Vec3::Zero();
        Vec3 max = Vec3::Zero();
        bool valid = false;
    };

    struct CollisionMeshCacheEntry
    {
        ObjMesh mesh;
        MeshBounds bounds;
    };

    struct RobotCollisionModelCacheEntry
    {
        RobotCollisionModelPtr model;
        std::size_t requestedGeometries = 0;
        std::size_t meshGeometries = 0;
        std::size_t addedGeometries = 0;
        std::size_t failedGeometries = 0;
    };

    struct RobotCollisionLinkSelection
    {
        bool allLinks = false;
        std::unordered_set<std::string> links;
    };

    using RobotCollisionLinkSelectionMap =
        std::unordered_map<std::string, RobotCollisionLinkSelection>;

    std::unordered_map<std::string, CollisionMeshCacheEntry>& collisionMeshCache()
    {
        static std::unordered_map<std::string, CollisionMeshCacheEntry> cache;
        return cache;
    }

    std::unordered_map<std::string, RobotCollisionModelCacheEntry>& robotCollisionModelCache()
    {
        static std::unordered_map<std::string, RobotCollisionModelCacheEntry> cache;
        return cache;
    }

    void addRobotLinkSelection(
        RobotCollisionLinkSelectionMap& selection,
        const std::string& robotId,
        const std::vector<std::string>& includeLinks)
    {
        if(robotId.empty()) {
            return;
        }

        RobotCollisionLinkSelection& robotSelection = selection[robotId];
        if(includeLinks.empty()) {
            robotSelection.allLinks = true;
            robotSelection.links.clear();
            return;
        }

        if(robotSelection.allLinks) {
            return;
        }

        robotSelection.links.insert(includeLinks.begin(), includeLinks.end());
    }

    void addRobotLinkSelection(
        RobotCollisionLinkSelectionMap& selection,
        const std::string& robotId,
        const std::string& linkName,
        const std::vector<std::string>& fallbackLinks = {})
    {
        if(!linkName.empty()) {
            addRobotLinkSelection(selection, robotId, std::vector<std::string>{ linkName });
            return;
        }
        addRobotLinkSelection(selection, robotId, fallbackLinks);
    }

    bool generatorNeedsAllSceneRobots(const simulation_project::CollisionPairGeneratorDesc& generator)
    {
        return generator.type == "SceneAll";
    }

    void addGeneratorRobotLinkSelection(
        RobotCollisionLinkSelectionMap& selection,
        const simulation_project::CollisionPairGeneratorDesc& generator)
    {
        if(generatorNeedsAllSceneRobots(generator)) {
            return;
        }

        if(generator.type == "LinkLink") {
            addRobotLinkSelection(selection, generator.robotA, generator.linkA, generator.includeLinksA);
            addRobotLinkSelection(selection, generator.robotB, generator.linkB, generator.includeLinksB);
        } else if(generator.type == "LinkRobot") {
            addRobotLinkSelection(selection, generator.robotId, generator.linkName, generator.includeLinksA);
            addRobotLinkSelection(selection, generator.robotB, generator.includeLinksB);
        } else if(generator.type == "LinkObject" || generator.type == "LinkObjectGroup") {
            addRobotLinkSelection(selection, generator.robotId, generator.linkName, generator.includeLinksA);
        } else if(generator.type == "RobotRobot") {
            addRobotLinkSelection(selection, generator.robotA, generator.includeLinksA);
            addRobotLinkSelection(selection, generator.robotB, generator.includeLinksB);
        } else if(generator.type == "RobotObject" ||
            generator.type == "RobotObjectGroup" ||
            generator.type == "RobotSelf") {
            addRobotLinkSelection(selection, generator.robotId, generator.includeLinksA);
        }
    }

    RobotCollisionLinkSelectionMap buildRobotCollisionLinkSelection(
        const simulation_project::ProjectDocument& document)
    {
        RobotCollisionLinkSelectionMap selection;
        bool requiresAllRobots = false;
        const bool useDetectors = !document.collision.detectors.empty();

        if(useDetectors) {
            for(const simulation_project::CollisionDetectorDesc& detector : document.collision.detectors) {
                if(detector.type == "SceneAll") {
                    requiresAllRobots = true;
                }
                for(const simulation_project::CollisionDetectorTargetDesc& target : detector.targets) {
                    if(!target.robotId.empty()) {
                        addRobotLinkSelection(selection, target.robotId, target.includeLinks);
                    }
                }
                for(const simulation_project::CollisionPairGeneratorDesc& generator : detector.pairGenerators) {
                    if(generatorNeedsAllSceneRobots(generator)) {
                        requiresAllRobots = true;
                    }
                    addGeneratorRobotLinkSelection(selection, generator);
                }
            }
        } else {
            for(const simulation_project::CollisionPairDesc& pair : document.collision.query.pairs) {
                addRobotLinkSelection(selection, pair.robotA, pair.includeLinksA);
                addRobotLinkSelection(selection, pair.robotB, pair.includeLinksB);
            }
            for(const simulation_project::RobotObjectCollisionPairDesc& pair : document.collision.query.robotObjectPairs) {
                if(pair.enabled) {
                    addRobotLinkSelection(selection, pair.robotId, pair.includeRobotLinks);
                }
            }
        }

        if(requiresAllRobots) {
            selection.clear();
            for(const simulation_project::RobotDesc& robot : document.robots) {
                selection[robot.id].allLinks = true;
            }
        }

        return selection;
    }

    void logInit()
    {
        std::map<std::string, std::pair<bool, bool>> logInfo = {
            {"rs2026", {true, true}},
            {"SMRobot", {true, true}},
            {"RobotIO", {true, true}},
            {"AssetCore", {true, true}},
        };

        CustomLog::init(logInfo);
        CustomLog::set_level(CustomLog::Level::debug);
        CustomLog::rotate_all();
    }

    std::string formatCollisionObjectInfo(const CollisionObjectInfo& info)
    {
        if(!info.linkName.empty()) {
            return info.linkName;
        }

        if(!info.elementName.empty()) {
            return info.elementName;
        }

        if(info.robotInstance >= 0) {
            return "robot:" + std::to_string(info.robotInstance);
        }

        if(info.object != 0) {
            return "object:" + std::to_string(info.object);
        }

        return "-";
    }

    const char* collisionShapeTypeName(CollisionShapeType type)
    {
        switch(type) {
        case CollisionShapeType::Box:
            return "Box";
        case CollisionShapeType::Sphere:
            return "Sphere";
        case CollisionShapeType::Cylinder:
            return "Cylinder";
        case CollisionShapeType::Capsule:
            return "Capsule";
        case CollisionShapeType::ConvexMesh:
            return "ConvexMesh";
        case CollisionShapeType::TriangleMesh:
            return "TriangleMesh";
        case CollisionShapeType::Compound:
            return "Compound";
        case CollisionShapeType::Unknown:
        default:
            return "Unknown";
        }
    }

    void fillCollisionPairSummary(
        const collision::CollisionResult& collisionResult,
        std::string& firstPairA,
        std::string& firstPairB)
    {
        if(!collisionResult.contacts.empty()) {
            const collision::Contact& contact = collisionResult.contacts.front();
            firstPairA = formatCollisionObjectInfo(contact.infoA);
            firstPairB = formatCollisionObjectInfo(contact.infoB);
            return;
        }

        if(collisionResult.hasNearestPoints) {
            firstPairA = formatCollisionObjectInfo(collisionResult.nearestInfoA);
            firstPairB = formatCollisionObjectInfo(collisionResult.nearestInfoB);
        }
    }

    ProjectScene::CollisionDetectorInfo::Vec3Info toCollisionVec3Info(const collision::Vec3& value)
    {
        ProjectScene::CollisionDetectorInfo::Vec3Info result;
        result.x = value.x();
        result.y = value.y();
        result.z = value.z();
        return result;
    }

    std::vector<ProjectScene::CollisionDetectorInfo::ContactInfo> makeCollisionContactInfo(
        const collision::CollisionResult& collisionResult)
    {
        std::vector<ProjectScene::CollisionDetectorInfo::ContactInfo> result;
        result.reserve(collisionResult.contacts.size());
        for(const collision::Contact& contact : collisionResult.contacts) {
            if(!contact.visualizable) {
                continue;
            }

            ProjectScene::CollisionDetectorInfo::ContactInfo info;
            info.bodyA = formatCollisionObjectInfo(contact.infoA);
            info.bodyB = formatCollisionObjectInfo(contact.infoB);
            info.position = toCollisionVec3Info(contact.position);
            info.normal = toCollisionVec3Info(contact.normal);
            info.penetrationDepth = contact.penetrationDepth;
            result.push_back(std::move(info));
        }
        return result;
    }

    ProjectScene::CollisionDetectorInfo::NearestInfo makeCollisionNearestInfo(
        const collision::CollisionResult& collisionResult,
        const std::string& nearestState,
        const std::string& nearestReason)
    {
        ProjectScene::CollisionDetectorInfo::NearestInfo result;
        result.state = nearestState;
        result.reason = nearestReason;
        result.valid = collisionResult.hasNearestPoints;
        if(!result.valid) {
            return result;
        }

        result.bodyA = formatCollisionObjectInfo(collisionResult.nearestInfoA);
        result.bodyB = formatCollisionObjectInfo(collisionResult.nearestInfoB);
        result.pointA = toCollisionVec3Info(collisionResult.nearestPointA);
        result.pointB = toCollisionVec3Info(collisionResult.nearestPointB);
        result.distance = collisionResult.minDistance;
        return result;
    }

    std::shared_ptr<scenecore::CameraNode> createMainCamera(scenecore::SceneGraph& graph)
    {
        auto camera = cameracore::CameraFactory::createOrbitCamera();
        camera->setUpAxis(cameracore::UpAxis::Z_UP);

        auto node = std::make_shared<scenecore::CameraNode>(camera, "mainCameraNode");
        graph.addNode(node);
        graph.registerNode(node);
        return node;
    }

    struct CameraSceneBounds
    {
        bool valid = false;
        Vec3 min = Vec3::Zero();
        Vec3 max = Vec3::Zero();

        void includePoint(const Vec3& point)
        {
            if(!valid) {
                min = point;
                max = point;
                valid = true;
                return;
            }
            min = min.cwiseMin(point);
            max = max.cwiseMax(point);
        }

        void includeAabb(const fcl::AABBd& aabb)
        {
            includePoint(aabb.min_);
            includePoint(aabb.max_);
        }

        void includeTransformedBounds(
            const Transform3& transform,
            const Vec3& localMin,
            const Vec3& localMax)
        {
            for(int x = 0; x < 2; ++x) {
                for(int y = 0; y < 2; ++y) {
                    for(int z = 0; z < 2; ++z) {
                        const Vec3 local(
                            x == 0 ? localMin.x() : localMax.x(),
                            y == 0 ? localMin.y() : localMax.y(),
                            z == 0 ? localMin.z() : localMax.z());
                        includePoint(transform * local);
                    }
                }
            }
        }

        Vec3 center() const
        {
            if(!valid) {
                return Vec3::Zero();
            }
            return (min + max) * 0.5;
        }

        double radius() const
        {
            if(!valid) {
                return 2.0;
            }
            return std::max(0.5, (max - min).norm() * 0.5);
        }
    };

    Eigen::Vector3f cameraDirection(ProjectSceneCameraView view)
    {
        switch(view) {
        case ProjectSceneCameraView::Front:
            return Eigen::Vector3f(0.0f, -1.0f, 0.0f);
        case ProjectSceneCameraView::Back:
            return Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        case ProjectSceneCameraView::Left:
            return Eigen::Vector3f(-1.0f, 0.0f, 0.0f);
        case ProjectSceneCameraView::Right:
            return Eigen::Vector3f(1.0f, 0.0f, 0.0f);
        case ProjectSceneCameraView::Top:
            return Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        case ProjectSceneCameraView::Bottom:
            return Eigen::Vector3f(0.0f, 0.0f, -1.0f);
        case ProjectSceneCameraView::Home:
        case ProjectSceneCameraView::Isometric:
        default:
            return Eigen::Vector3f(1.0f, -1.0f, 0.75f).normalized();
        }
    }

    Eigen::Vector3f cameraUp(ProjectSceneCameraView view)
    {
        switch(view) {
        case ProjectSceneCameraView::Top:
            return Eigen::Vector3f(0.0f, 1.0f, 0.0f);
        case ProjectSceneCameraView::Bottom:
            return Eigen::Vector3f(0.0f, -1.0f, 0.0f);
        default:
            return Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        }
    }

    struct CameraLookAtState
    {
        Eigen::Vector3f position = Eigen::Vector3f(0.0f, -5.0f, 3.0f);
        Eigen::Vector3f target = Eigen::Vector3f::Zero();
        Eigen::Vector3f up = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        bool valid = false;
    };

    CameraLookAtState makeCameraLookAtState(
        const CameraSceneBounds& bounds,
        ProjectSceneCameraView view,
        double distanceScale)
    {
        const Vec3 center = bounds.center();
        const double radius = bounds.radius();
        constexpr double kVerticalFovDegrees = 45.0;
        const double fovRadians = kVerticalFovDegrees * kPi / 180.0;
        const double distance = std::max(2.5, radius / std::sin(fovRadians * 0.5) * distanceScale);

        CameraLookAtState state;
        state.target = Eigen::Vector3f(
            static_cast<float>(center.x()),
            static_cast<float>(center.y()),
            static_cast<float>(center.z()));
        state.position = state.target + cameraDirection(view) * static_cast<float>(distance);
        state.up = cameraUp(view);
        state.valid = true;
        return state;
    }

    CameraLookAtState interpolateCameraLookAt(
        const CameraLookAtState& start,
        const CameraLookAtState& end,
        double t)
    {
        const float weight = static_cast<float>(std::clamp(t, 0.0, 1.0));
        CameraLookAtState state;
        state.position = start.position + (end.position - start.position) * weight;
        state.target = start.target + (end.target - start.target) * weight;
        state.up = (start.up + (end.up - start.up) * weight).normalized();
        state.valid = true;
        return state;
    }

    int parseObjIndex(const std::string& token)
    {
        const auto slash = token.find('/');
        const std::string value = slash == std::string::npos ? token : token.substr(0, slash);
        return std::stoi(value);
    }

    ObjMesh loadObjMesh(const std::string& path, const Eigen::Vector3d& scale)
    {
        std::ifstream input(pathFromUtf8(path));
        if(!input) {
            throw std::runtime_error("Failed to open OBJ collision mesh: " + path);
        }

        ObjMesh mesh;
        std::string line;

        while(std::getline(input, line)) {
            std::istringstream stream(line);
            std::string tag;
            stream >> tag;

            if(tag == "v") {
                double x = 0.0;
                double y = 0.0;
                double z = 0.0;
                stream >> x >> y >> z;
                mesh.vertices.emplace_back(x * scale.x(), y * scale.y(), z * scale.z());
            } else if(tag == "f") {
                std::vector<uint32_t> face;
                std::string token;
                while(stream >> token) {
                    const int rawIndex = parseObjIndex(token);
                    const int resolved = rawIndex > 0
                        ? rawIndex - 1
                        : static_cast<int>(mesh.vertices.size()) + rawIndex;

                    if(resolved >= 0 && resolved < static_cast<int>(mesh.vertices.size())) {
                        face.push_back(static_cast<uint32_t>(resolved));
                    }
                }

                for(size_t i = 1; i + 1 < face.size(); ++i) {
                    mesh.indices.push_back(face[0]);
                    mesh.indices.push_back(face[i]);
                    mesh.indices.push_back(face[i + 1]);
                }
            }
        }

        return mesh;
    }

    bool isFiniteVec(const Vec3& value)
    {
        return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
    }

    MeshBounds computeMeshBounds(const ObjMesh& mesh)
    {
        MeshBounds bounds;
        bounds.min = Vec3(
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        bounds.max = Vec3(
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest());

        for(const Vec3& vertex : mesh.vertices) {
            if(!isFiniteVec(vertex)) {
                continue;
            }
            bounds.min = bounds.min.cwiseMin(vertex);
            bounds.max = bounds.max.cwiseMax(vertex);
            bounds.valid = true;
        }

        return bounds;
    }

    TriangleMeshGeometryData makeTriangleMeshData(const ObjMesh& mesh)
    {
        TriangleMeshGeometryData data;
        data.vertices = mesh.vertices;
        data.indices = mesh.indices;
        return data;
    }

    CollisionGeometryPtr buildTriangleMeshGeometry(const ObjMesh& mesh)
    {
        return CollisionGeometryBuilder::buildTriangleMesh(makeTriangleMeshData(mesh));
    }

    CollisionGeometryPtr makeCollisionGeometry(const robot::RobotCollisionGeometry& geometry)
    {
        switch(geometry.type) {
        case robot::RobotGeometryType::Box:
            return CollisionGeometryBuilder::buildBox(geometry.boxSize);
        case robot::RobotGeometryType::Sphere:
            return CollisionGeometryBuilder::buildSphere(geometry.radius);
        case robot::RobotGeometryType::Cylinder:
            return CollisionGeometryBuilder::buildCylinder(geometry.radius, geometry.length);
        case robot::RobotGeometryType::Mesh:
            return buildTriangleMeshGeometry(loadObjMesh(geometry.meshPath, geometry.meshScale));
        default:
            return nullptr;
        }
    }

    CollisionGeometryRole collisionRoleFromString(const std::string& role)
    {
        if(role == "PlanningProxy") {
            return CollisionGeometryRole::PlanningProxy;
        }
        if(role == "VisualizationProxy" || role == "Simplified") {
            return CollisionGeometryRole::Simplified;
        }
        if(role == simulation_project::kCoacdCollisionModelRole) {
            return CollisionGeometryRole::Simplified;
        }
        if(role == "SafetyMargin") {
            return CollisionGeometryRole::SafetyMargin;
        }
        if(role == "SphereCover") {
            return CollisionGeometryRole::SphereCover;
        }
        return CollisionGeometryRole::Exact;
    }

    CollisionShapeDesc makeShapeDesc(const robot::RobotCollisionGeometry& geometry, const ObjMesh* mesh)
    {
        CollisionShapeDesc desc;
        desc.label = geometry.partUid;
        desc.source = geometry.source;
        desc.localTransform = geometry.T_part;
        desc.role = collisionRoleFromString(geometry.role);
        desc.inflationMargin = geometry.inflationMargin;

        switch(geometry.type) {
        case robot::RobotGeometryType::Box:
            desc.type = CollisionShapeType::Box;
            desc.boxSize = geometry.boxSize;
            break;
        case robot::RobotGeometryType::Sphere:
            desc.type = CollisionShapeType::Sphere;
            desc.radius = geometry.radius;
            break;
        case robot::RobotGeometryType::Cylinder:
            desc.type = CollisionShapeType::Cylinder;
            desc.radius = geometry.radius;
            desc.length = geometry.length;
            break;
        case robot::RobotGeometryType::Mesh:
            desc.type = CollisionShapeType::TriangleMesh;
            if(mesh != nullptr) {
                desc.vertices = mesh->vertices;
                desc.indices = mesh->indices;
            }
            break;
        default:
            desc.type = CollisionShapeType::Unknown;
            break;
        }

        return desc;
    }

    CollisionShapeDesc makeMeshProxyBoxShapeDesc(
        const robot::RobotCollisionGeometry& geometry,
        const MeshBounds& bounds)
    {
        Vec3 size = bounds.max - bounds.min;
        Vec3 center = (bounds.min + bounds.max) * 0.5;
        for(int i = 0; i < 3; ++i) {
            if(!std::isfinite(size[i]) || size[i] <= 1.0e-9) {
                size[i] = 1.0e-9;
            }
            if(!std::isfinite(center[i])) {
                center[i] = 0.0;
            }
        }

        Transform3 localTransform = geometry.T_part;
        localTransform.translation() = localTransform.translation() + center;

        CollisionShapeDesc desc;
        desc.type = CollisionShapeType::Box;
        desc.label = geometry.partUid;
        desc.source = geometry.source;
        desc.role = collisionRoleFromString(geometry.role);
        desc.inflationMargin = geometry.inflationMargin;
        desc.boxSize = size;
        desc.localTransform = localTransform;
        return desc;
    }

    ObjMesh makeObjMesh(assetcore::ModelDesc& modelDesc, double scale)
    {
        ObjMesh mesh;
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

    bool hasTriangleMeshCollisionShape(const RuntimeSceneObject& object)
    {
        return !object.collisionShape.vertices.empty() &&
            !object.collisionShape.indices.empty();
    }

    bool buildTemporaryCoacdInputObjectFromModel(
        const std::filesystem::path& modelPath,
        double visualScale,
        const std::string& objectId,
        const std::string& objectName,
        const collision::Transform3& visualLocalTransform,
        RuntimeSceneObject& output)
    {
        if(modelPath.empty()) {
            return false;
        }

        std::string loadError;
        std::shared_ptr<assetcore::ModelDesc> modelDesc =
            assetcore::AssetManager::instance().tryLoadModel(
                pathToUtf8(modelPath),
                static_cast<float>(visualScale),
                &loadError);
        if(!modelDesc) {
            LOG_WARNING("rs2026") << "COACD visual input load failed: object=" << objectId
                << ", path=" << pathToUtf8(modelPath)
                << ", error=" << loadError;
            return false;
        }

        ObjMesh mesh = makeObjMesh(*modelDesc, 1.0);
        if(mesh.vertices.empty() || mesh.indices.empty()) {
            LOG_WARNING("rs2026") << "COACD visual input mesh is empty: object=" << objectId
                << ", path=" << pathToUtf8(modelPath)
                << ", vertices=" << mesh.vertices.size()
                << ", indices=" << mesh.indices.size();
            return false;
        }

        output = RuntimeSceneObject();
        output.runtimeId = 900001ull;
        output.documentId = objectId;
        output.name = objectName.empty() ? objectId : objectName;
        output.collisionShape.type = CollisionShapeType::TriangleMesh;
        output.collisionShape.label = "tool:" + objectId;
        output.collisionShape.role = CollisionGeometryRole::Exact;
        output.collisionShape.vertices = std::move(mesh.vertices);
        output.collisionShape.indices = std::move(mesh.indices);
        output.collisionShape.localTransform = visualLocalTransform;
        return true;
    }

    ObjMesh loadCollisionMesh(const robot::RobotCollisionGeometry& geometry)
    {
        auto modelDesc = assetcore::AssetManager::instance().loadModel(
            geometry.meshPath,
            static_cast<float>(geometry.meshScale.x()));
        if(modelDesc) {
            return makeObjMesh(*modelDesc, 1.0);
        }

        const std::filesystem::path meshPath = pathFromUtf8(geometry.meshPath);
        if(meshPath.extension() == ".obj" || meshPath.extension() == ".OBJ") {
            return loadObjMesh(geometry.meshPath, geometry.meshScale);
        }

        return {};
    }

    std::string meshTimestampKey(const std::string& path)
    {
        std::error_code error;
        const auto timestamp = std::filesystem::last_write_time(pathFromUtf8(path), error);
        if(error) {
            return "missing";
        }
        return std::to_string(timestamp.time_since_epoch().count());
    }

    std::string makeCollisionMeshCacheKey(const robot::RobotCollisionGeometry& geometry)
    {
        std::ostringstream stream;
        stream << geometry.meshPath
            << "|scale=" << geometry.meshScale.x()
            << "," << geometry.meshScale.y()
            << "," << geometry.meshScale.z()
            << "|stamp=" << meshTimestampKey(geometry.meshPath);
        return stream.str();
    }

    const CollisionMeshCacheEntry& loadCollisionMeshCacheEntry(const robot::RobotCollisionGeometry& geometry)
    {
        const auto cacheStart = std::chrono::steady_clock::now();
        const std::string key = makeCollisionMeshCacheKey(geometry);
        auto& cache = collisionMeshCache();
        auto it = cache.find(key);
        if(it != cache.end()) {
            LOG_DEBUG("rs2026") << "Collision mesh cache hit: mesh=" << geometry.meshPath
                << ", vertices=" << it->second.mesh.vertices.size()
                << ", validBounds=" << it->second.bounds.valid
                << ", elapsedMs=" << elapsedMilliseconds(cacheStart);
            return it->second;
        }

        const auto loadStart = std::chrono::steady_clock::now();
        CollisionMeshCacheEntry entry;
        entry.mesh = loadCollisionMesh(geometry);
        const double loadMs = elapsedMilliseconds(loadStart);
        const auto boundsStart = std::chrono::steady_clock::now();
        entry.bounds = computeMeshBounds(entry.mesh);
        const double boundsMs = elapsedMilliseconds(boundsStart);
        auto inserted = cache.emplace(key, std::move(entry));
        LOG_DEBUG("rs2026") << "Collision mesh cache miss: mesh=" << geometry.meshPath
            << ", vertices=" << inserted.first->second.mesh.vertices.size()
            << ", validBounds=" << inserted.first->second.bounds.valid
            << ", loadMs=" << loadMs
            << ", boundsMs=" << boundsMs
            << ", elapsedMs=" << elapsedMilliseconds(loadStart);
        return inserted.first->second;
    }

    std::string activeRobotLinkCollisionModelId(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId,
        const std::string& linkName)
    {
        for(const simulation_project::RobotLinkCollisionModelSelectionDesc& selection :
            document.collision.robotLinkModelSelections) {
            if(selection.robotId == robotId && selection.linkName == linkName) {
                return simulation_project::normalizeRobotLinkCollisionModelId(selection.activeModelId);
            }
        }
        return {};
    }

    robot::RobotCollisionGeometry makeVisualCollisionGeometry(
        const robot::RobotVisual& visual,
        const std::string& linkName,
        std::size_t visualIndex)
    {
        robot::RobotCollisionGeometry geometry;
        geometry.partUid = visual.partUid.empty()
            ? linkName + "_visual_collision_" + std::to_string(visualIndex + 1)
            : visual.partUid;
        geometry.T_part = visual.T_part;
        geometry.type = robot::RobotGeometryType::Mesh;
        geometry.meshPath = visual.meshPath;
        geometry.meshScale = Eigen::Vector3d::Ones() * static_cast<double>(visual.meshScale);
        geometry.role = "Exact";
        geometry.enabled = true;
        geometry.source = simulation_project::kConvertFromVisualCollisionSource;
        return geometry;
    }

    bool shouldUseVisualCollisionFallback(
        const std::string& activeModelId,
        const robot::RobotLink& link,
        bool explicitlySelectedLink)
    {
        return explicitlySelectedLink &&
            activeModelId.empty() &&
            link.collisions.empty() &&
            !link.visuals.empty();
    }

    std::string makeRobotCollisionModelCacheKey(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId,
        const robot::RobotModel& robotModel,
        const RobotCollisionLinkSelection* linkSelection)
    {
        std::ostringstream stream;
        stream << robotId;
        if(linkSelection == nullptr || linkSelection->allLinks) {
            stream << "|selectedLinks=*";
        } else {
            std::vector<std::string> selectedLinks(
                linkSelection->links.begin(),
                linkSelection->links.end());
            std::sort(selectedLinks.begin(), selectedLinks.end());
            stream << "|selectedLinks=";
            for(const std::string& linkName : selectedLinks) {
                stream << linkName << ",";
            }
        }
        for(const auto& linkName : robotModel.linkNames) {
            if(linkSelection != nullptr &&
                !linkSelection->allLinks &&
                linkSelection->links.count(linkName) == 0) {
                continue;
            }

            const auto linkIt = robotModel.links.find(linkName);
            if(linkIt == robotModel.links.end()) {
                continue;
            }

            stream << "|link=" << linkName;
            const std::string activeModelId =
                activeRobotLinkCollisionModelId(document, robotId, linkName);
            stream << "|active=" << activeModelId;
            const bool useVisualCollision =
                simulation_project::isConvertFromVisualCollisionModelId(activeModelId) ||
                shouldUseVisualCollisionFallback(
                    activeModelId,
                    linkIt->second,
                    linkSelection != nullptr && !linkSelection->allLinks);
            if(useVisualCollision) {
                for(const robot::RobotVisual& visual : linkIt->second.visuals) {
                    stream << "|v=" << visual.partUid
                        << "," << visual.meshPath
                        << "," << visual.meshScale
                        << "," << meshTimestampKey(visual.meshPath);
                    const Eigen::Matrix4d matrix = visual.T_part.matrix();
                    for(int row = 0; row < 4; ++row) {
                        for(int column = 0; column < 4; ++column) {
                            stream << "," << matrix(row, column);
                        }
                    }
                }
                continue;
            }

            for(const auto& geometry : linkIt->second.collisions) {
                if(!simulation_project::robotLinkCollisionElementMatchesModelId(
                       activeModelId,
                       robotId,
                       linkName,
                       geometry.source,
                       geometry.role)) {
                    continue;
                }
                stream << "|g=" << geometry.partUid
                    << "," << static_cast<int>(geometry.type)
                    << "," << geometry.enabled
                    << "," << geometry.role
                    << "," << geometry.boxSize.x() << "," << geometry.boxSize.y() << "," << geometry.boxSize.z()
                    << "," << geometry.radius
                    << "," << geometry.length
                    << "," << geometry.meshPath
                    << "," << geometry.meshScale.x() << "," << geometry.meshScale.y() << "," << geometry.meshScale.z()
                    << "," << geometry.inflationMargin
                    << "," << meshTimestampKey(geometry.meshPath);
                const Eigen::Matrix4d matrix = geometry.T_part.matrix();
                for(int row = 0; row < 4; ++row) {
                    for(int column = 0; column < 4; ++column) {
                        stream << "," << matrix(row, column);
                    }
                }
            }
        }
        return stream.str();
    }

    std::shared_ptr<rendercore::Material> makeOverlayMaterial(bool highlighted)
    {
        auto material = std::make_shared<rendercore::Material>();
        material->baseColor = highlighted
            ? Eigen::Vector4f(1.0f, 0.25f, 0.05f, 0.72f)
            : regularCollisionGeometryMaterialColor(0.35f);
        material->specular = Eigen::Vector3f(0.25f, 0.25f, 0.25f);
        material->shininess = 24.0f;
        return material;
    }

    void applyOverlayMaterial(
        const std::shared_ptr<rendercore::Model>& model,
        const std::shared_ptr<rendercore::Material>& material)
    {
        if(!model) {
            return;
        }

        for(size_t i = 0; i < model->subMeshCount(); ++i) {
            model->subMesh(static_cast<unsigned int>(i)).material = material;
        }
    }

    void addMeshOverlay(
        const robot::RobotCollisionGeometry& geometry,
        const std::shared_ptr<scenecore::SceneNode>& linkNode,
        scenecore::SceneGraph& graph,
        ObjectID object,
        const std::string& linkName,
        std::unordered_map<ObjectID, MeshOverlay>& overlays)
    {
        if(geometry.type != robot::RobotGeometryType::Mesh) {
            LOG_DEBUG("rs2026") << "Collision overlay mesh skipped: link=" << linkName
                << ", part=" << geometry.partUid
                << ", reason=notMesh";
            return;
        }
        if(geometry.meshPath.empty()) {
            LOG_DEBUG("rs2026") << "Collision overlay mesh skipped: link=" << linkName
                << ", part=" << geometry.partUid
                << ", reason=emptyMeshPath";
            return;
        }
        if(!linkNode) {
            LOG_DEBUG("rs2026") << "Collision overlay mesh skipped: link=" << linkName
                << ", part=" << geometry.partUid
                << ", mesh=" << geometry.meshPath
                << ", object=" << object
                << ", reason=missingLinkNode";
            return;
        }

        const auto overlayStart = std::chrono::steady_clock::now();
        LOG_DEBUG("rs2026") << "Collision overlay mesh begin: mesh=" << geometry.meshPath
            << ", link=" << linkName
            << ", part=" << geometry.partUid
            << ", object=" << object;
        const auto loadStart = std::chrono::steady_clock::now();
        auto modelDesc = assetcore::AssetManager::instance().loadModel(
            geometry.meshPath,
            static_cast<float>(geometry.meshScale.x()));
        const double loadMs = elapsedMilliseconds(loadStart);
        if(!modelDesc) {
            LOG_DEBUG("rs2026") << "Collision overlay mesh load skipped: mesh=" << geometry.meshPath
                << ", link=" << linkName
                << ", part=" << geometry.partUid
                << ", object=" << object
                << ", loadMs=" << loadMs
                << ", elapsedMs=" << elapsedMilliseconds(overlayStart);
            return;
        }

        const auto buildStart = std::chrono::steady_clock::now();
        auto renderModel = rendercore::ModelManager::instance().buildModelFromDesc(*modelDesc);
        const double buildMs = elapsedMilliseconds(buildStart);
        if(!renderModel) {
            LOG_DEBUG("rs2026") << "Collision overlay mesh build failed: mesh=" << geometry.meshPath
                << ", link=" << linkName
                << ", part=" << geometry.partUid
                << ", object=" << object
                << ", loadMs=" << loadMs
                << ", buildMs=" << buildMs
                << ", elapsedMs=" << elapsedMilliseconds(overlayStart);
            return;
        }

        auto material = makeOverlayMaterial(false);
        applyOverlayMaterial(renderModel, material);

        auto node = std::make_shared<VisibleModelNode>(renderModel);
        node->setName(geometry.partUid + "_collision_overlay");
        node->setLocal(math::eigenToGlm(geometry.T_part));

        linkNode->addChild(node);
        graph.registerNode(node);
        overlays[object] = MeshOverlay{ node, material, linkName };
        LOG_DEBUG("rs2026") << "Collision overlay mesh build: mesh=" << geometry.meshPath
            << ", link=" << linkName
            << ", part=" << geometry.partUid
            << ", object=" << object
            << ", loadMs=" << loadMs
            << ", buildMs=" << buildMs
            << ", elapsedMs=" << elapsedMilliseconds(overlayStart);
    }

    void ensureRobotMeshOverlays(RuntimeRobot& runtime, scenecore::SceneGraph& graph)
    {
        if(!runtime.collisionInstance || !runtime.visualBridge) {
            LOG_DEBUG("rs2026") << "Collision overlay ensure skipped: robot=" << runtime.documentId
                << ", hasCollisionInstance=" << (runtime.collisionInstance != nullptr)
                << ", hasVisualBridge=" << (runtime.visualBridge != nullptr);
            return;
        }

        const auto overlayStart = std::chrono::steady_clock::now();
        std::size_t created = 0;
        std::size_t considered = 0;
        std::size_t existing = 0;
        std::size_t invalidObject = 0;
        std::size_t missingLinkNode = 0;
        for(const auto& linkName : runtime.model.linkNames) {
            const auto linkIt = runtime.model.links.find(linkName);
            if(linkIt == runtime.model.links.end()) {
                continue;
            }

            for(const auto& geometry : linkIt->second.collisions) {
                if(!geometry.enabled || geometry.type != robot::RobotGeometryType::Mesh) {
                    continue;
                }

                ++considered;
                const ObjectID object = runtime.collisionInstance->getObjectID(linkName, geometry.partUid);
                if(object == 0) {
                    ++invalidObject;
                    LOG_DEBUG("rs2026") << "Collision overlay mesh skipped: robot=" << runtime.documentId
                        << ", link=" << linkName
                        << ", part=" << geometry.partUid
                        << ", mesh=" << geometry.meshPath
                        << ", reason=invalidObject";
                    continue;
                }
                if(runtime.meshOverlays.find(object) != runtime.meshOverlays.end()) {
                    ++existing;
                    continue;
                }

                auto linkNode = runtime.visualBridge->linkNode(linkName);
                if(!linkNode) {
                    ++missingLinkNode;
                    LOG_DEBUG("rs2026") << "Collision overlay mesh skipped: robot=" << runtime.documentId
                        << ", link=" << linkName
                        << ", part=" << geometry.partUid
                        << ", mesh=" << geometry.meshPath
                        << ", object=" << object
                        << ", reason=missingLinkNode";
                    continue;
                }

                const std::size_t oldCount = runtime.meshOverlays.size();
                addMeshOverlay(geometry, linkNode, graph, object, linkName, runtime.meshOverlays);
                if(runtime.meshOverlays.size() != oldCount) {
                    ++created;
                }
            }
        }

        LOG_DEBUG("rs2026") << "Collision overlay ensure: robot=" << runtime.documentId
            << ", considered=" << considered
            << ", created=" << created
            << ", existing=" << existing
            << ", invalidObject=" << invalidObject
            << ", missingLinkNode=" << missingLinkNode
            << ", total=" << runtime.meshOverlays.size()
            << ", elapsedMs=" << elapsedMilliseconds(overlayStart);
    }

    void buildRobotCollision(
        RuntimeRobot& runtime,
        const simulation_project::ProjectDocument& document,
        const robot::RobotModel& robotModel,
        scenecore::SceneGraph& graph,
        bool buildOverlays,
        const RobotCollisionLinkSelection* linkSelection)
    {
        const auto buildStart = std::chrono::steady_clock::now();
        const std::string cacheKey = makeRobotCollisionModelCacheKey(
            document,
            runtime.documentId,
            robotModel,
            linkSelection);
        auto& modelCache = robotCollisionModelCache();
        const auto cacheIt = modelCache.find(cacheKey);
        if(cacheIt != modelCache.end() && cacheIt->second.model) {
            runtime.collisionModel = cacheIt->second.model;
            runtime.collisionInstance = std::make_shared<RobotCollisionInstance>(runtime.runtimeId, runtime.collisionModel);
            if(buildOverlays) {
                ensureRobotMeshOverlays(runtime, graph);
            }
            LOG_DEBUG("rs2026") << "Robot collision model cache hit: robot=" << runtime.documentId
                << ", requested=" << cacheIt->second.requestedGeometries
                << ", mesh=" << cacheIt->second.meshGeometries
                << ", added=" << cacheIt->second.addedGeometries
                << ", failed=" << cacheIt->second.failedGeometries
                << ", selectedLinks=" << (linkSelection == nullptr || linkSelection->allLinks
                    ? std::string("all")
                    : std::to_string(linkSelection->links.size()))
                << ", collisionObjects=" << runtime.collisionInstance->objects().size()
                << ", elapsedMs=" << elapsedMilliseconds(buildStart);
            return;
        }

        runtime.collisionModel = std::make_shared<RobotCollisionModel>();
        std::size_t requestedGeometries = 0;
        std::size_t addedGeometries = 0;
        std::size_t failedGeometries = 0;
        std::size_t meshGeometries = 0;
        std::size_t skippedLinks = 0;

        for(const auto& linkName : robotModel.linkNames) {
            if(linkSelection != nullptr &&
                !linkSelection->allLinks &&
                linkSelection->links.count(linkName) == 0) {
                ++skippedLinks;
                continue;
            }

            const auto linkIt = robotModel.links.find(linkName);
            if(linkIt == robotModel.links.end()) {
                continue;
            }

            std::vector<robot::RobotCollisionGeometry> visualCollisionGeometries;
            const std::string activeModelId =
                activeRobotLinkCollisionModelId(document, runtime.documentId, linkName);
            const bool useVisualCollision =
                simulation_project::isConvertFromVisualCollisionModelId(activeModelId) ||
                shouldUseVisualCollisionFallback(
                    activeModelId,
                    linkIt->second,
                    linkSelection != nullptr && !linkSelection->allLinks);
            if(useVisualCollision) {
                visualCollisionGeometries.reserve(linkIt->second.visuals.size());
                for(std::size_t visualIndex = 0; visualIndex < linkIt->second.visuals.size(); ++visualIndex) {
                    const robot::RobotVisual& visual = linkIt->second.visuals[visualIndex];
                    if(!visual.meshPath.empty()) {
                        visualCollisionGeometries.push_back(
                            makeVisualCollisionGeometry(visual, linkName, visualIndex));
                    }
                }
            }

            const std::vector<robot::RobotCollisionGeometry>& sourceGeometries =
                useVisualCollision ? visualCollisionGeometries : linkIt->second.collisions;

            for(const auto& geometry : sourceGeometries) {
                if(!useVisualCollision &&
                    !simulation_project::robotLinkCollisionElementMatchesModelId(
                        activeModelId,
                        runtime.documentId,
                        linkName,
                        geometry.source,
                        geometry.role)) {
                    continue;
                }
                if(!geometry.enabled) {
                    continue;
                }

                ++requestedGeometries;
                ObjMesh meshData;
                ObjMesh* meshPtr = nullptr;
                CollisionGeometryPtr fclGeometry;
                CollisionShapeDesc shape;

                if(geometry.type == robot::RobotGeometryType::Mesh) {
                    ++meshGeometries;
                    const auto meshStart = std::chrono::steady_clock::now();
                    const CollisionMeshCacheEntry& meshEntry = loadCollisionMeshCacheEntry(geometry);
                    meshData = meshEntry.mesh;
                    meshPtr = &meshData;
                    const MeshBounds bounds = meshEntry.bounds;
                    LOG_DEBUG("rs2026") << "Collision mesh bounds: robot=" << runtime.documentId
                        << ", link=" << linkName
                        << ", part=" << geometry.partUid
                        << ", mesh=" << geometry.meshPath
                        << ", vertices=" << meshData.vertices.size()
                        << ", valid=" << bounds.valid
                        << ", elapsedMs=" << elapsedMilliseconds(meshStart);
                    if(bounds.valid) {
                        const auto triangleMeshStart = std::chrono::steady_clock::now();
                        shape = makeShapeDesc(geometry, meshPtr);
                        fclGeometry = buildTriangleMeshGeometry(meshData);
                        const double triangleMeshMs = elapsedMilliseconds(triangleMeshStart);
                        if(fclGeometry) {
                            LOG_DEBUG("rs2026") << "Robot mesh collision uses triangle mesh: robot=" << runtime.documentId
                                << ", link=" << linkName
                                << ", part=" << geometry.partUid
                                << ", mesh=" << geometry.meshPath
                                << ", sourceGeometry=mesh"
                                << ", queryGeometry=triangleMesh"
                                << ", shapeType=" << collisionShapeTypeName(shape.type)
                                << ", buildMs=" << triangleMeshMs
                                << ", elapsedMs=" << elapsedMilliseconds(meshStart);
                        }
                    } else {
                        LOG_WARNING("rs2026") << "Robot mesh collision has invalid bounds: robot=" << runtime.documentId
                            << ", link=" << linkName
                            << ", part=" << geometry.partUid
                            << ", mesh=" << geometry.meshPath;
                    }
                } else {
                    fclGeometry = makeCollisionGeometry(geometry);
                    shape = makeShapeDesc(geometry, nullptr);
                }

                if(!fclGeometry) {
                    ++failedGeometries;
                    LOG_WARNING("rs2026") << "Robot collision geometry failed: robot=" << runtime.documentId
                        << ", link=" << linkName
                        << ", part=" << geometry.partUid
                        << ", mesh=" << geometry.meshPath;
                    continue;
                }

                runtime.collisionModel->addLinkShape(linkName, geometry.partUid, shape, fclGeometry);
                ++addedGeometries;
                LOG_DEBUG("rs2026") << "Robot collision geometry added: robot=" << runtime.documentId
                    << ", link=" << linkName
                    << ", part=" << geometry.partUid
                    << ", sourceGeometry=" << (geometry.type == robot::RobotGeometryType::Mesh ? "mesh" : "primitive")
                    << ", queryGeometry=" << collisionShapeTypeName(shape.type)
                    << ", shapeType=" << collisionShapeTypeName(shape.type)
                    << ", elapsedMs=" << elapsedMilliseconds(buildStart);
            }
        }

        runtime.collisionInstance = std::make_shared<RobotCollisionInstance>(runtime.runtimeId, runtime.collisionModel);
        if(buildOverlays) {
            ensureRobotMeshOverlays(runtime, graph);
        }
        modelCache[cacheKey] = RobotCollisionModelCacheEntry{
            runtime.collisionModel,
            requestedGeometries,
            meshGeometries,
            addedGeometries,
            failedGeometries };
        LOG_DEBUG("rs2026") << "Robot collision build: robot=" << runtime.documentId
            << ", links=" << robotModel.linkNames.size()
            << ", requested=" << requestedGeometries
            << ", mesh=" << meshGeometries
            << ", added=" << addedGeometries
            << ", failed=" << failedGeometries
            << ", skippedLinks=" << skippedLinks
            << ", selectedLinks=" << (linkSelection == nullptr || linkSelection->allLinks
                ? std::string("all")
                : std::to_string(linkSelection->links.size()))
            << ", collisionObjects=" << runtime.collisionInstance->objects().size()
            << ", elapsedMs=" << elapsedMilliseconds(buildStart);
    }

    std::vector<std::pair<ObjectID, ObjectID>> makeCrossRobotPairs(
        const RobotCollisionInstancePtr& a,
        const RobotCollisionInstancePtr& b)
    {
        std::vector<std::pair<ObjectID, ObjectID>> pairs;

        for(const auto& [keyA, objectA] : a->objects()) {
            for(const auto& [keyB, objectB] : b->objects()) {
                if(objectA && objectB) {
                    pairs.emplace_back(objectA->id(), objectB->id());
                }
            }
        }

        return pairs;
    }

    bool containsLink(const std::vector<std::string>& links, const std::string& linkName)
    {
        return links.empty() || std::find(links.begin(), links.end(), linkName) != links.end();
    }

    std::unordered_set<ObjectID> collectCollidingObjects(const CollisionResult& result)
    {
        std::unordered_set<ObjectID> objects;
        for(const auto& contact : result.contacts) {
            objects.insert(contact.objectA);
            objects.insert(contact.objectB);
        }
        return objects;
    }

    Color4 collisionHighlightColor()
    {
        return Color4(1.0, 0.2, 0.05, 0.75);
    }

    Color4 regularCollisionGeometryColor(double alpha)
    {
        return Color4(0.2, 0.7, 1.0, alpha);
    }

    std::string collisionGeometryOverlayCacheKey(
        const CollisionVisualizationOptions& options,
        std::uint64_t version)
    {
        std::ostringstream key;
        key << version
            << "|exact=" << options.showExactGeometry
            << "|safety=" << options.showSafetyGeometry
            << "|simplified=" << options.showSimplifiedGeometry
            << "|sphere=" << options.showSphereCover
            << "|planning=" << options.showPlanningProxy
            << "|alpha=" << options.alpha
            << "|mode=" << static_cast<int>(options.drawMode);
        return key.str();
    }

    void appendCachedCollisionGeometry(
        const CollisionDebugDrawData& cachedData,
        const CollisionScene& scene,
        const CollisionVisualizationOptions& options,
        const std::unordered_set<ObjectID>& highlightedObjects,
        CollisionDebugDrawData& out)
    {
        out.geometry.reserve(out.geometry.size() + cachedData.geometry.size());

        for(const CollisionDebugDrawDesc& cachedDesc : cachedData.geometry) {
            CollisionDebugDrawDesc desc = cachedDesc;
            if(!scene.objectTransform(desc.object, desc.worldTransform)) {
                continue;
            }

            desc.highlighted =
                options.showObjectHighlight &&
                highlightedObjects.count(desc.object) > 0;
            if(options.showOnlyCollidingObjects && !desc.highlighted) {
                continue;
            }

            if(desc.highlighted) {
                desc.color = collisionHighlightColor();
            } else {
                desc.color = regularCollisionGeometryColor(options.alpha);
            }

            out.geometry.push_back(std::move(desc));
        }
    }

    void mergeDistanceResult(CollisionResult& target, const CollisionResult& distanceResult)
    {
        if(distanceResult.minDistance != std::numeric_limits<double>::max()) {
            target.minDistance = distanceResult.minDistance;
        }

        if(distanceResult.hasNearestPoints) {
            target.nearestObjectA = distanceResult.nearestObjectA;
            target.nearestObjectB = distanceResult.nearestObjectB;
            target.nearestInfoA = distanceResult.nearestInfoA;
            target.nearestInfoB = distanceResult.nearestInfoB;
            target.nearestPointA = distanceResult.nearestPointA;
            target.nearestPointB = distanceResult.nearestPointB;
            target.hasNearestPoints = true;
        }
    }

    std::size_t visualizableContactCount(const CollisionResult& result)
    {
        return static_cast<std::size_t>(std::count_if(
            result.contacts.begin(),
            result.contacts.end(),
            [](const collision::Contact& contact) {
                return contact.visualizable;
            }));
    }

    void updateMeshOverlays(
        const std::unordered_map<ObjectID, MeshOverlay>& overlays,
        const std::unordered_set<ObjectID>& collidingObjects,
        bool visible,
        const std::string& selectedLink)
    {
        for(const auto& [object, overlay] : overlays) {
            const bool colliding = collidingObjects.count(object) > 0;
            if(overlay.node) {
                overlay.node->setVisible(visible);
            }

            if(!overlay.material) {
                continue;
            }

            if(colliding) {
                overlay.material->baseColor = Eigen::Vector4f(1.0f, 0.25f, 0.05f, 0.72f);
            } else if(!selectedLink.empty() && overlay.linkName == selectedLink) {
                overlay.material->baseColor = regularCollisionGeometryMaterialColor(0.48f);
            } else {
                overlay.material->baseColor = regularCollisionGeometryMaterialColor(0.35f);
            }
        }
    }

    std::shared_ptr<rendercore::Material> makeSelectionMaterial(
        const std::shared_ptr<rendercore::Material>& source,
        const Eigen::Vector4f& color,
        const Eigen::Vector3f& emissive)
    {
        auto material = source ? std::make_shared<rendercore::Material>(*source) : std::make_shared<rendercore::Material>();
        material->baseColor = color;
        material->emissiveColor = emissive;
        return material;
    }

    void applySelectionOverrides(const std::vector<MaterialOverride>& overrides)
    {
        for(const MaterialOverride& item : overrides) {
            if(item.model && item.subMeshIndex < item.model->subMeshCount()) {
                item.model->subMesh(item.subMeshIndex).material =
                    makeSelectionMaterial(item.originalMaterial, item.color, item.emissive);
            }
        }
    }

    void restoreMaterialOverrides(std::vector<MaterialOverride>& overrides)
    {
        for(const MaterialOverride& item : overrides) {
            if(item.model && item.subMeshIndex < item.model->subMeshCount()) {
                item.model->subMesh(item.subMeshIndex).material = item.originalMaterial;
            }
        }
        overrides.clear();
    }

    std::shared_ptr<rendercore::Material> makeRobotCollisionHighlightMaterial(
        const std::shared_ptr<rendercore::Material>& source)
    {
        auto material = source ? std::make_shared<rendercore::Material>(*source) : std::make_shared<rendercore::Material>();
        material->baseColor = Eigen::Vector4f(1.0f, 0.25f, 0.05f, 1.0f);
        material->emissiveColor = Eigen::Vector3f(0.2f, 0.04f, 0.01f);
        material->specular = Eigen::Vector3f(0.35f, 0.25f, 0.2f);
        material->shininess = 32.0f;
        return material;
    }

    std::vector<std::shared_ptr<rendercore::Material>> captureRobotLinkMaterials(
        const RuntimeRobot& robot,
        const std::string& linkName)
    {
        std::vector<std::shared_ptr<rendercore::Material>> materials;
        if(!robot.visualBridge) {
            return materials;
        }

        for(const auto& node : robot.visualBridge->visualNodes(linkName)) {
            const auto model = node ? node->model() : nullptr;
            if(!model) {
                continue;
            }

            for(unsigned int i = 0; i < model->subMeshCount(); ++i) {
                materials.push_back(model->subMesh(i).material);
            }
        }

        return materials;
    }

    bool isDefaultRobotMaterial(const rendercore::Material& material)
    {
        const Eigen::Vector4f defaultColor(0.8f, 0.8f, 0.8f, 1.0f);
        return (material.baseColor - defaultColor).cwiseAbs().maxCoeff() < 1.0e-5f &&
            material.textures.empty();
    }

    Eigen::Vector4f visualDiffuseColor(const robot::RobotVisual& visual)
    {
        if(!visual.hasMaterial) {
            return Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f);
        }
        return visual.diffuse.cast<float>();
    }

    void setRobotLinkHighlighted(RuntimeRobot& robot, const std::string& linkName, bool highlighted)
    {
        if(!robot.visualBridge) {
            return;
        }

        const bool currentlyHighlighted = robot.highlightedLinks.count(linkName) > 0;
        if(currentlyHighlighted == highlighted) {
            return;
        }

        if(highlighted && robot.linkOriginalMaterials.find(linkName) == robot.linkOriginalMaterials.end()) {
            robot.linkOriginalMaterials[linkName] = captureRobotLinkMaterials(robot, linkName);
        }

        const auto originalIt = robot.linkOriginalMaterials.find(linkName);
        std::size_t materialIndex = 0;
        for(const auto& node : robot.visualBridge->visualNodes(linkName)) {
            const auto model = node ? node->model() : nullptr;
            if(!model) {
                continue;
            }

            for(unsigned int i = 0; i < model->subMeshCount(); ++i) {
                auto& subMesh = model->subMesh(i);
                if(highlighted) {
                    subMesh.material = makeRobotCollisionHighlightMaterial(subMesh.material);
                } else if(originalIt != robot.linkOriginalMaterials.end() &&
                    materialIndex < originalIt->second.size()) {
                    subMesh.material = originalIt->second[materialIndex];
                }
                ++materialIndex;
            }
        }

        if(highlighted) {
            robot.highlightedLinks.insert(linkName);
        } else {
            robot.highlightedLinks.erase(linkName);
            robot.linkOriginalMaterials.erase(linkName);
        }
    }

    void updateRobotVisualHighlights(
        std::vector<RuntimeRobot>& robots,
        const std::unordered_set<ObjectID>& collidingObjects)
    {
        for(RuntimeRobot& robot : robots) {
            std::unordered_set<std::string> collidingLinks;
            if(robot.collisionInstance) {
                for(ObjectID object : collidingObjects) {
                    const LinkInfo* info = robot.collisionInstance->getLinkInfo(object);
                    if(info != nullptr && !info->linkName.empty()) {
                        collidingLinks.insert(info->linkName);
                    }
                }
            }

            std::vector<std::string> linksToRestore;
            linksToRestore.reserve(robot.highlightedLinks.size());
            for(const std::string& linkName : robot.highlightedLinks) {
                if(collidingLinks.count(linkName) == 0) {
                    linksToRestore.push_back(linkName);
                }
            }

            for(const std::string& linkName : linksToRestore) {
                setRobotLinkHighlighted(robot, linkName, false);
            }
            for(const std::string& linkName : collidingLinks) {
                setRobotLinkHighlighted(robot, linkName, true);
            }
        }
    }

    void setSceneObjectHighlighted(RuntimeSceneObject& object, bool highlighted)
    {
        if(object.highlighted == highlighted) {
            return;
        }

        if(object.visualModel) {
            for(unsigned int i = 0; i < object.visualModel->subMeshCount(); ++i) {
                auto& subMesh = object.visualModel->subMesh(i);
                if(highlighted) {
                    subMesh.material = object.highlightMaterial;
                } else if(i < object.originalMaterials.size()) {
                    subMesh.material = object.originalMaterials[i];
                }
            }
        }
        if(object.pointCloudNode) {
            const float pointSize = highlighted
                ? std::max(object.pointCloudBasePointSize * 1.8f, object.pointCloudBasePointSize + 2.0f)
                : object.pointCloudBasePointSize;
            object.pointCloudNode->setPointSize(pointSize);
        }
        object.highlighted = highlighted;
    }

    void updateSceneObjectHighlights(
        std::vector<RuntimeSceneObject>& objects,
        const std::unordered_set<ObjectID>& collidingObjects,
        const std::string& selectedObject)
    {
        for(RuntimeSceneObject& object : objects) {
            bool highlighted = object.documentId == selectedObject;
            if(object.collisionObject) {
                highlighted = highlighted || collidingObjects.count(object.collisionObject->id()) > 0;
            }
            for(const RuntimeSceneCollisionObject& collisionObject : object.collisionObjects) {
                highlighted = highlighted ||
                    (collisionObject.collisionObject &&
                        collidingObjects.count(collisionObject.collisionObject->id()) > 0);
            }
            setSceneObjectHighlighted(object, highlighted);
        }
    }

    std::vector<std::string> jointNames(const robot::RobotModel& model)
    {
        std::vector<std::string> names;
        names.reserve(model.joints.size());
        for(const auto& joint : model.joints) {
            names.push_back(joint.name);
        }
        return names;
    }

    std::string jointTypeName(robot::JointType type)
    {
        switch(type) {
        case robot::JointType::Revolute:
            return "revolute";
        case robot::JointType::Prismatic:
            return "prismatic";
        default:
            return "fixed";
        }
    }

    std::vector<ProjectScene::RobotJointInfo> movableJointInfos(const robot::RobotModel& model)
    {
        std::vector<ProjectScene::RobotJointInfo> joints;
        for(const auto& joint : model.joints) {
            if(joint.dofIndex < 0) {
                continue;
            }

            joints.push_back(ProjectScene::RobotJointInfo{
                joint.name,
                jointTypeName(joint.type) });
        }
        return joints;
    }

    std::vector<ProjectScene::RobotJointInfo> parallelControlJointInfos()
    {
        return {
            { "parallel.pose.x", "prismatic" },
            { "parallel.pose.y", "prismatic" },
            { "parallel.pose.z", "prismatic" },
            { "parallel.pose.roll", "revolute" },
            { "parallel.pose.pitch", "revolute" },
            { "parallel.pose.yaw", "revolute" },
            { "parallel.actuator.1", "prismatic" },
            { "parallel.actuator.2", "prismatic" },
            { "parallel.actuator.3", "prismatic" },
            { "parallel.actuator.4", "prismatic" },
            { "parallel.actuator.5", "prismatic" },
            { "parallel.actuator.6", "prismatic" }
        };
    }

    std::vector<std::string> parallelControlJointNames()
    {
        std::vector<std::string> names;
        const std::vector<ProjectScene::RobotJointInfo> controls =
            parallelControlJointInfos();
        names.reserve(controls.size());
        for(const ProjectScene::RobotJointInfo& control : controls) {
            names.push_back(control.jointName);
        }
        return names;
    }

    std::string lowerCopy(std::string value)
    {
        std::transform(
            value.begin(),
            value.end(),
            value.begin(),
            [](unsigned char ch) {
                return static_cast<char>(std::tolower(ch));
            });
        return value;
    }

    bool isStewartSimscapeRobotDesc(const simulation_project::RobotDesc& robotDesc)
    {
        const std::string sourceType = lowerCopy(robotDesc.sourceType);
        const std::string sourcePath = lowerCopy(robotDesc.sourcePath);
        return sourceType.find("simscape") != std::string::npos &&
            sourcePath.find("stewart") != std::string::npos;
    }

    const std::string& stewartUpperActuatorMarker()
    {
        static const std::string marker =
            "\xE6\x89\xA7\xE8\xA1\x8C\xE5\x99\xA8\xE4\xB8\x8A\xE6\xAE\xB5";
        return marker;
    }

    const std::string& stewartLowerActuatorMarker()
    {
        static const std::string marker =
            "\xE6\x89\xA7\xE8\xA1\x8C\xE5\x99\xA8\xE4\xB8\x8B\xE6\xAE\xB5";
        return marker;
    }

    const std::string& stewartHookeMarker()
    {
        static const std::string marker =
            "\xE8\x99\x8E\xE5\x85\x8B\xE9\x93\xB0";
        return marker;
    }

    const std::string& stewartTopPlatformMarker()
    {
        static const std::string marker =
            "\xE4\xB8\x8A\xE5\x8A\xA8\xE5\xB9\xB3\xE5\x8F\xB0";
        return marker;
    }

    const std::string& stewartPayloadMarker()
    {
        static const std::string marker =
            "\xE5\xB7\xA5\xE4\xBB\xB6";
        return marker;
    }

    int legIndexAfterMarker(const std::string& linkName, const std::string& marker)
    {
        const std::size_t markerPos = linkName.find(marker);
        if(markerPos == std::string::npos) {
            return -1;
        }

        for(std::size_t index = markerPos + marker.size(); index < linkName.size(); ++index) {
            const char ch = linkName[index];
            if(ch >= '1' && ch <= '6') {
                return ch - '1';
            }
        }
        return -1;
    }

    int actuatorLegIndexFromLinkName(const std::string& linkName)
    {
        return legIndexAfterMarker(linkName, stewartUpperActuatorMarker());
    }

    bool robotModelHasLinkContaining(const robot::RobotModel& model, const std::string& marker)
    {
        return std::any_of(
            model.linkNames.begin(),
            model.linkNames.end(),
            [&](const std::string& linkName) {
                return linkName.find(marker) != std::string::npos;
            });
    }

    bool isStewartSimscapePlatformModel(
        const simulation_project::RobotDesc& robotDesc,
        const robot::RobotModel& model)
    {
        if(!isStewartSimscapeRobotDesc(robotDesc)) {
            return false;
        }

        std::array<bool, 6> upperLinksFound{};
        for(const std::string& linkName : model.linkNames) {
            const int legIndex = actuatorLegIndexFromLinkName(linkName);
            if(legIndex >= 0 && legIndex < 6) {
                upperLinksFound[static_cast<std::size_t>(legIndex)] = true;
            }
        }

        return robotModelHasLinkContaining(
                   model,
                   "\xE4\xB8\x8B\xE5\xAE\x9A\xE5\xB9\xB3\xE5\x8F\xB0") &&
            std::all_of(
                upperLinksFound.begin(),
                upperLinksFound.end(),
                [](bool found) { return found; });
    }

    bool isStewartSimscapeFollowerFragment(
        const simulation_project::RobotDesc& robotDesc,
        const robot::RobotModel& model)
    {
        return isStewartSimscapeRobotDesc(robotDesc) &&
            !isStewartSimscapePlatformModel(robotDesc, model) &&
            robotModelHasLinkContaining(model, stewartUpperActuatorMarker());
    }

    bool sameStewartSource(const RuntimeRobot& a, const RuntimeRobot& b)
    {
        return lowerCopy(a.sourceType) == lowerCopy(b.sourceType) &&
            lowerCopy(a.sourcePath) == lowerCopy(b.sourcePath) &&
            lowerCopy(a.sourceType).find("simscape") != std::string::npos &&
            lowerCopy(a.sourcePath).find("stewart") != std::string::npos;
    }

    int stewartLegIndexFromLinkName(const std::string& linkName)
    {
        const int upperIndex = legIndexAfterMarker(linkName, stewartUpperActuatorMarker());
        if(upperIndex >= 0) {
            return upperIndex;
        }

        const int lowerIndex = legIndexAfterMarker(linkName, stewartLowerActuatorMarker());
        if(lowerIndex >= 0) {
            return lowerIndex;
        }

        return legIndexAfterMarker(linkName, stewartHookeMarker());
    }

    std::string findFirstLinkContaining(const robot::RobotModel& model, const std::string& token)
    {
        for(const std::string& linkName : model.linkNames) {
            if(linkName.find(token) != std::string::npos) {
                return linkName;
            }
        }
        return std::string();
    }

    std::string findFirstLinkForLeg(
        const robot::RobotModel& model,
        const std::string& marker,
        int legIndex)
    {
        for(const std::string& linkName : model.linkNames) {
            if(legIndexAfterMarker(linkName, marker) == legIndex) {
                return linkName;
            }
        }
        return std::string();
    }

    bool linkExists(const robot::RobotModel& model, const std::string& linkName)
    {
        return !linkName.empty() && model.links.find(linkName) != model.links.end();
    }

    bool isStewartInternalPlatformDrivenLink(const std::string& linkName)
    {
        return linkName.find(stewartTopPlatformMarker()) != std::string::npos ||
            linkName.find(stewartPayloadMarker()) != std::string::npos;
    }

    bool jointTouchesLegMarker(
        const robot::RobotJoint& joint,
        const std::string& marker,
        int legIndex)
    {
        return legIndexAfterMarker(joint.parent, marker) == legIndex ||
            legIndexAfterMarker(joint.child, marker) == legIndex;
    }

    bool jointAnchorWorld(
        const RuntimeRobot& runtime,
        const robot::RobotJoint& joint,
        collision::Vec3& anchor)
    {
        if(!runtime.instance || !linkExists(runtime.model, joint.parent)) {
            return false;
        }

        const collision::Transform3 worldParent =
            runtime.instance->getLinkTransform(joint.parent);
        anchor = (worldParent * joint.T_parent_joint).translation();
        return true;
    }

    void collectJointAnchorsForLeg(
        const RuntimeRobot& runtime,
        const std::string& marker,
        int legIndex,
        std::vector<collision::Vec3>& anchors)
    {
        for(const robot::RobotJoint& joint : runtime.model.joints) {
            if(!jointTouchesLegMarker(joint, marker, legIndex)) {
                continue;
            }

            collision::Vec3 anchor = collision::Vec3::Zero();
            if(jointAnchorWorld(runtime, joint, anchor)) {
                anchors.push_back(anchor);
            }
        }
    }

    void collectLinkOriginAnchorForLeg(
        const RuntimeRobot& runtime,
        const std::string& marker,
        int legIndex,
        std::vector<collision::Vec3>& anchors)
    {
        const std::string link = findFirstLinkForLeg(runtime.model, marker, legIndex);
        if(!link.empty() && runtime.instance) {
            anchors.push_back(runtime.instance->getLinkTransform(link).translation());
        }
    }

    bool chooseLowestLocalZAnchor(
        const std::vector<collision::Vec3>& anchors,
        const collision::Transform3& referenceInverse,
        collision::Vec3& anchor)
    {
        if(anchors.empty()) {
            return false;
        }

        auto bestIt = anchors.begin();
        double bestZ = (referenceInverse * *bestIt).z();
        for(auto it = std::next(anchors.begin()); it != anchors.end(); ++it) {
            const double z = (referenceInverse * *it).z();
            if(z < bestZ) {
                bestZ = z;
                bestIt = it;
            }
        }

        anchor = *bestIt;
        return true;
    }

    bool chooseFarthestAnchor(
        const std::vector<collision::Vec3>& anchors,
        const collision::Vec3& reference,
        collision::Vec3& anchor)
    {
        if(anchors.empty()) {
            return false;
        }

        auto bestIt = anchors.begin();
        double bestDistance = (*bestIt - reference).squaredNorm();
        for(auto it = std::next(anchors.begin()); it != anchors.end(); ++it) {
            const double distance = (*it - reference).squaredNorm();
            if(distance > bestDistance) {
                bestDistance = distance;
                bestIt = it;
            }
        }

        anchor = *bestIt;
        return true;
    }

    int findStewartFollowerActuatorDofIndex(const RuntimeRobot& follower, int legIndex)
    {
        for(const robot::RobotJoint& joint : follower.model.joints) {
            if(joint.type != robot::JointType::Prismatic || joint.dofIndex < 0) {
                continue;
            }

            if(jointTouchesLegMarker(joint, stewartLowerActuatorMarker(), legIndex) &&
                jointTouchesLegMarker(joint, stewartUpperActuatorMarker(), legIndex)) {
                return joint.dofIndex;
            }
        }

        for(const robot::RobotJoint& joint : follower.model.joints) {
            if(joint.type != robot::JointType::Prismatic || joint.dofIndex < 0) {
                continue;
            }

            if(stewartLegIndexFromLinkName(joint.parent) == legIndex ||
                stewartLegIndexFromLinkName(joint.child) == legIndex) {
                return joint.dofIndex;
            }
        }
        return -1;
    }

    std::array<int, 2> findStewartBaseRevoluteDofIndices(
        const RuntimeRobot& platform,
        int legIndex,
        const std::string& lowerLink)
    {
        std::array<int, 2> result{ { -1, -1 } };
        std::unordered_map<std::string, const robot::RobotJoint*> parentJointByChild;
        for(const robot::RobotJoint& joint : platform.model.joints) {
            if(joint.isLoop) {
                continue;
            }
            parentJointByChild.emplace(joint.child, &joint);
        }

        std::unordered_set<std::string> visited;
        std::string link = lowerLink;
        while(!link.empty() && link != platform.model.root && visited.insert(link).second) {
            const auto parentIt = parentJointByChild.find(link);
            if(parentIt == parentJointByChild.end()) {
                break;
            }

            const robot::RobotJoint* joint = parentIt->second;
            if(joint->type == robot::JointType::Revolute && joint->dofIndex >= 0) {
                if(result[0] < 0) {
                    result[0] = joint->dofIndex;
                } else if(result[1] < 0 && result[0] != joint->dofIndex) {
                    result[1] = joint->dofIndex;
                    return result;
                }
            }
            link = joint->parent;
        }

        for(const robot::RobotJoint& joint : platform.model.joints) {
            if(joint.type != robot::JointType::Revolute || joint.dofIndex < 0) {
                continue;
            }
            if(!jointTouchesLegMarker(joint, stewartHookeMarker(), legIndex) ||
                jointTouchesLegMarker(joint, stewartUpperActuatorMarker(), legIndex)) {
                continue;
            }

            if(result[0] < 0) {
                result[0] = joint.dofIndex;
            } else if(result[1] < 0 && result[0] != joint.dofIndex) {
                result[1] = joint.dofIndex;
                break;
            }
        }
        return result;
    }

    void setStewartLegJointValues(
        robotinstance::RobotInstance& instance,
        const StewartLegRuntimeControl& leg,
        double q0,
        double q1,
        double travel)
    {
        if(leg.baseRevoluteDofIndices[0] >= 0) {
            instance.setJoint(static_cast<std::size_t>(leg.baseRevoluteDofIndices[0]), q0);
        }
        if(leg.baseRevoluteDofIndices[1] >= 0) {
            instance.setJoint(static_cast<std::size_t>(leg.baseRevoluteDofIndices[1]), q1);
        }
        if(leg.actuatorDofIndex >= 0) {
            instance.setJoint(static_cast<std::size_t>(leg.actuatorDofIndex), travel);
        }
    }

    double wrapAngleNear(double value, double reference)
    {
        constexpr double kPi = 3.14159265358979323846;
        constexpr double kTwoPi = 2.0 * kPi;
        while(value - reference > kPi) {
            value -= kTwoPi;
        }
        while(reference - value > kPi) {
            value += kTwoPi;
        }
        return value;
    }

    double squaredAngleDistance(double value, double reference)
    {
        const double diff = wrapAngleNear(value, reference) - reference;
        return diff * diff;
    }

    collision::Vec3 stewartLegPlatformAnchorWorld(
        const RuntimeRobot& platform,
        const StewartLegRuntimeControl& leg)
    {
        if(!platform.instance || leg.upperLink.empty()) {
            return collision::Vec3::Zero();
        }

        return platform.instance->getLinkTransform(leg.upperLink) *
            leg.platformAnchorLocalInUpperLink;
    }

    collision::Vec3 stewartTargetPlatformAnchorWorld(
        const RuntimeRobot& platform,
        int legIndex)
    {
        const collision::Transform3 poseTransform =
            kine::StewartPlatformKinematics::poseTransform(platform.parallelPose);
        const collision::Vec3 localAnchor =
            poseTransform *
            platform.parallelGeometry.platformAnchors[static_cast<std::size_t>(legIndex)];
        return platform.parallelHomeBaseTransform * localAnchor;
    }

    bool solveStewartLegControl(
        RuntimeRobot& platform,
        StewartLegRuntimeControl& leg)
    {
        if(!platform.instance || !leg.enabled || leg.legIndex < 0 || leg.legIndex >= 6) {
            return false;
        }

        const std::size_t legArrayIndex = static_cast<std::size_t>(leg.legIndex);
        const double targetLength = platform.parallelActuatorLengths[legArrayIndex];
        const double travel =
            leg.actuatorSign *
            (targetLength - leg.homeLength);
        double q0 = 0.0;
        double q1 = 0.0;
        const auto& state = platform.instance->getState();
        if(leg.baseRevoluteDofIndices[0] >= 0 &&
            static_cast<std::size_t>(leg.baseRevoluteDofIndices[0]) < state.q.size()) {
            q0 = state.q[static_cast<std::size_t>(leg.baseRevoluteDofIndices[0])];
        }
        if(leg.baseRevoluteDofIndices[1] >= 0 &&
            static_cast<std::size_t>(leg.baseRevoluteDofIndices[1]) < state.q.size()) {
            q1 = state.q[static_cast<std::size_t>(leg.baseRevoluteDofIndices[1])];
        }
        const double seedQ0 = q0;
        const double seedQ1 = q1;
        double bestQ0 = q0;
        double bestQ1 = q1;
        double bestScore = std::numeric_limits<double>::max();
        const collision::Vec3 target = stewartTargetPlatformAnchorWorld(platform, leg.legIndex);

        constexpr double kStep = 1.0e-5;
        constexpr double kTolerance = 1.0e-5;
        constexpr double kContinuityWeight = 1.0e-4;
        constexpr int kMaxIterations = 12;

        for(int iteration = 0; iteration < kMaxIterations; ++iteration) {
            setStewartLegJointValues(*platform.instance, leg, q0, q1, travel);
            platform.instance->update();
            const collision::Vec3 residual =
                stewartLegPlatformAnchorWorld(platform, leg) - target;
            const double residualNorm = residual.norm();
            const double continuityCost =
                kContinuityWeight *
                (squaredAngleDistance(q0, seedQ0) +
                    squaredAngleDistance(q1, seedQ1));
            const double score = residualNorm + continuityCost;
            if(score < bestScore) {
                bestScore = score;
                bestQ0 = q0;
                bestQ1 = q1;
            }
            if(residualNorm <= kTolerance) {
                return true;
            }

            Eigen::Matrix<double, 3, 2> jacobian;
            for(int column = 0; column < 2; ++column) {
                double probeQ0 = q0;
                double probeQ1 = q1;
                if(column == 0) {
                    probeQ0 += kStep;
                } else {
                    probeQ1 += kStep;
                }

                setStewartLegJointValues(*platform.instance, leg, probeQ0, probeQ1, travel);
                platform.instance->update();
                jacobian.col(column) =
                    (stewartLegPlatformAnchorWorld(platform, leg) -
                        (target + residual)) /
                    kStep;
            }

            const Eigen::Matrix2d normal =
                jacobian.transpose() * jacobian +
                Eigen::Matrix2d::Identity() * (1.0e-8 + kContinuityWeight);
            const Eigen::Vector2d prior(
                wrapAngleNear(q0, seedQ0) - seedQ0,
                wrapAngleNear(q1, seedQ1) - seedQ1);
            const Eigen::Vector2d delta =
                normal.ldlt().solve(
                    jacobian.transpose() * residual +
                    kContinuityWeight * prior);
            if(!std::isfinite(delta.x()) || !std::isfinite(delta.y())) {
                break;
            }

            q0 = wrapAngleNear(q0 - std::clamp(delta.x(), -0.25, 0.25), seedQ0);
            q1 = wrapAngleNear(q1 - std::clamp(delta.y(), -0.25, 0.25), seedQ1);
        }

        setStewartLegJointValues(*platform.instance, leg, bestQ0, bestQ1, travel);
        platform.instance->update();
        const double finalError =
            (stewartLegPlatformAnchorWorld(platform, leg) - target).norm();
        if(finalError > 5.0e-3) {
            LOG_WARNING("rs2026") << "Stewart leg IK residual is high: robot="
                << platform.documentId
                << ", leg=" << (leg.legIndex + 1)
                << ", residual=" << finalError << " m";
        }
        return true;
    }

    void solveStewartInternalLegControls(RuntimeRobot& platform)
    {
        if(!platform.parallelControlEnabled || !platform.instance) {
            return;
        }

        for(StewartLegRuntimeControl& leg : platform.parallelLegControls) {
            solveStewartLegControl(platform, leg);
        }
    }

    double distanceBetweenLinks(
        const RuntimeRobot& follower,
        const std::string& a,
        const std::string& b)
    {
        if(!follower.instance || a.empty() || b.empty()) {
            return 0.0;
        }

        return (follower.instance->getLinkTransform(a).translation() -
            follower.instance->getLinkTransform(b).translation()).norm();
    }

    double calibrateStewartFollowerActuatorSign(
        RuntimeRobot& follower,
        int dofIndex,
        const std::string& baseLink,
        const std::string& platformLink)
    {
        if(!follower.instance || dofIndex < 0) {
            return 1.0;
        }

        constexpr double kProbe = 1.0e-4;
        follower.instance->setJoint(static_cast<std::size_t>(dofIndex), 0.0);
        follower.instance->update();
        const double homeDistance = distanceBetweenLinks(follower, baseLink, platformLink);

        follower.instance->setJoint(static_cast<std::size_t>(dofIndex), kProbe);
        follower.instance->update();
        const double probeDistance = distanceBetweenLinks(follower, baseLink, platformLink);

        follower.instance->setJoint(static_cast<std::size_t>(dofIndex), 0.0);
        follower.instance->update();

        return probeDistance >= homeDistance ? 1.0 : -1.0;
    }

    double calibrateStewartInternalActuatorSign(
        RuntimeRobot& platform,
        const StewartLegRuntimeControl& leg)
    {
        if(!platform.instance ||
            leg.actuatorDofIndex < 0 ||
            leg.legIndex < 0 ||
            leg.legIndex >= 6) {
            return 1.0;
        }

        const std::size_t dofIndex = static_cast<std::size_t>(leg.actuatorDofIndex);
        const auto& state = platform.instance->getState();
        const double oldQ = dofIndex < state.q.size() ? state.q[dofIndex] : 0.0;
        const std::size_t legArrayIndex = static_cast<std::size_t>(leg.legIndex);
        const collision::Vec3 baseAnchorWorld =
            platform.parallelHomeBaseTransform *
            platform.parallelGeometry.baseAnchors[legArrayIndex];

        constexpr double kProbe = 1.0e-4;
        platform.instance->setJoint(dofIndex, 0.0);
        platform.instance->update();
        const double homeDistance =
            (stewartLegPlatformAnchorWorld(platform, leg) - baseAnchorWorld).norm();

        platform.instance->setJoint(dofIndex, kProbe);
        platform.instance->update();
        const double probeDistance =
            (stewartLegPlatformAnchorWorld(platform, leg) - baseAnchorWorld).norm();

        platform.instance->setJoint(dofIndex, oldQ);
        platform.instance->update();

        return probeDistance >= homeDistance ? 1.0 : -1.0;
    }

    Eigen::Matrix4d makeLegFollowerAffine(
        const collision::Transform3& platformHomeTransform,
        const kine::StewartPlatformPose& pose,
        const collision::Vec3& homeStart,
        const collision::Vec3& homeEnd)
    {
        const Eigen::Vector3d currentStart = homeStart;
        const Eigen::Vector3d currentEnd =
            kine::StewartPlatformKinematics::poseTransform(pose) * homeEnd;

        const Eigen::Vector3d homeVector = homeEnd - homeStart;
        const Eigen::Vector3d currentVector = currentEnd - currentStart;
        const double homeLength = homeVector.norm();
        const double currentLength = currentVector.norm();
        if(homeLength < 1.0e-9 || currentLength < 1.0e-9) {
            return Eigen::Matrix4d::Identity();
        }

        const Eigen::Vector3d homeAxis = homeVector / homeLength;
        const Eigen::Quaterniond rotation =
            Eigen::Quaterniond::FromTwoVectors(homeVector, currentVector).normalized();
        const Eigen::Matrix3d stretch =
            Eigen::Matrix3d::Identity() +
            ((currentLength / homeLength) - 1.0) *
                (homeAxis * homeAxis.transpose());
        const Eigen::Matrix3d linear = rotation.toRotationMatrix() * stretch;

        Eigen::Matrix4d local = Eigen::Matrix4d::Identity();
        local.block<3, 3>(0, 0) = linear;
        local.block<3, 1>(0, 3) = currentStart - linear * homeStart;

        return platformHomeTransform.matrix() *
            local *
            platformHomeTransform.inverse().matrix();
    }

    void captureStewartFollowerDrivenLinks(RuntimeRobot& follower, int legIndex)
    {
        follower.parallelFollowerDrivenLinks.clear();
        follower.parallelFollowerHomeLinkTransforms.clear();

        if(!follower.instance) {
            return;
        }

        for(const std::string& linkName : follower.model.linkNames) {
            if(linkName == follower.model.root || linkName == follower.model.base_link) {
                continue;
            }
            if(stewartLegIndexFromLinkName(linkName) != legIndex) {
                continue;
            }

            follower.parallelFollowerDrivenLinks.push_back(linkName);
            follower.parallelFollowerHomeLinkTransforms[linkName] =
                follower.instance->getLinkTransform(linkName);
        }
    }

    void configureStewartInternalActuatorDofs(RuntimeRobot& platform)
    {
        platform.parallelActuatorDofIndices.fill(-1);
        platform.parallelActuatorSigns.fill(1.0);
        if(!platform.instance) {
            return;
        }

        for(int legIndex = 0; legIndex < 6; ++legIndex) {
            const std::string upperLink = findFirstLinkForLeg(
                platform.model,
                stewartUpperActuatorMarker(),
                legIndex);
            if(upperLink.empty()) {
                continue;
            }

            std::string baseLink = findFirstLinkForLeg(
                platform.model,
                stewartLowerActuatorMarker(),
                legIndex);
            if(baseLink.empty()) {
                baseLink = findFirstLinkForLeg(
                    platform.model,
                    stewartHookeMarker(),
                    legIndex);
            }

            const int dofIndex = findStewartFollowerActuatorDofIndex(platform, legIndex);
            const std::size_t index = static_cast<std::size_t>(legIndex);
            platform.parallelActuatorDofIndices[index] = dofIndex;

            StewartLegRuntimeControl& leg = platform.parallelLegControls[index];
            leg.enabled = dofIndex >= 0;
            leg.legIndex = legIndex;
            leg.lowerLink = baseLink;
            leg.upperLink = upperLink;
            leg.baseRevoluteDofIndices =
                findStewartBaseRevoluteDofIndices(platform, legIndex, baseLink);
            leg.actuatorDofIndex = dofIndex;
            leg.actuatorSign = 1.0;
            leg.homeLength = platform.parallelActuatorHomeLengths[index];
            leg.platformAnchorLocalInUpperLink = collision::Vec3::Zero();
            if(!upperLink.empty()) {
                const collision::Vec3 homePlatformAnchorWorld =
                    platform.parallelHomeBaseTransform *
                    platform.parallelGeometry.platformAnchors[index];
                leg.platformAnchorLocalInUpperLink =
                    platform.instance->getLinkTransform(upperLink).inverse() *
                    homePlatformAnchorWorld;
            }
            platform.parallelActuatorSigns[index] =
                calibrateStewartInternalActuatorSign(platform, leg);
            leg.actuatorSign = platform.parallelActuatorSigns[index];

            if(leg.enabled &&
                (leg.baseRevoluteDofIndices[0] < 0 || leg.baseRevoluteDofIndices[1] < 0)) {
                leg.enabled = false;
                LOG_WARNING("rs2026") << "Stewart leg IK disabled because base revolute DOFs are incomplete: robot="
                    << platform.documentId
                    << ", leg=" << (legIndex + 1)
                    << ", dof0=" << leg.baseRevoluteDofIndices[0]
                    << ", dof1=" << leg.baseRevoluteDofIndices[1];
            }
        }
    }

    void captureStewartInternalPlatformVisuals(RuntimeRobot& platform)
    {
        platform.parallelInternalPlatformVisualsEnabled = false;
        platform.parallelInternalPlatformDrivenLinks.clear();
        platform.parallelInternalPlatformHomeLocalTransforms.clear();

        if(!platform.instance) {
            return;
        }

        const collision::Transform3 baseInverse =
            platform.parallelHomeBaseTransform.inverse();
        for(const std::string& linkName : platform.model.linkNames) {
            if(!isStewartInternalPlatformDrivenLink(linkName)) {
                continue;
            }

            platform.parallelInternalPlatformDrivenLinks.push_back(linkName);
            platform.parallelInternalPlatformHomeLocalTransforms[linkName] =
                baseInverse * platform.instance->getLinkTransform(linkName);
        }

        platform.parallelInternalPlatformVisualsEnabled =
            !platform.parallelInternalPlatformDrivenLinks.empty();
    }

    void applyStewartInternalPlatformVisualOverride(RuntimeRobot& platform)
    {
        if(!platform.parallelControlEnabled ||
            !platform.parallelInternalPlatformVisualsEnabled ||
            !platform.visualBridge) {
            return;
        }

        const collision::Transform3 poseTransform =
            kine::StewartPlatformKinematics::poseTransform(platform.parallelPose);
        for(const std::string& linkName : platform.parallelInternalPlatformDrivenLinks) {
            const auto homeIt =
                platform.parallelInternalPlatformHomeLocalTransforms.find(linkName);
            if(homeIt == platform.parallelInternalPlatformHomeLocalTransforms.end()) {
                continue;
            }

            const auto node = platform.visualBridge->linkNode(linkName);
            if(!node) {
                continue;
            }

            const collision::Transform3 linkTransform =
                platform.parallelHomeBaseTransform * poseTransform * homeIt->second;
            node->setLocal(math::eigenToGlm(linkTransform.matrix()));
            if(platform.collisionInstance) {
                platform.collisionInstance->setLinkTransform(linkName, linkTransform);
            }
        }
    }

    void applyStewartFollowerVisualOverride(
        const RuntimeRobot& platform,
        RuntimeRobot& follower)
    {
        if(!platform.parallelControlEnabled ||
            !follower.parallelFollowerEnabled ||
            !follower.parallelFollowerAnchorsValid ||
            !follower.visualBridge ||
            follower.parallelFollowerDrivenLinks.empty()) {
            return;
        }

        const Eigen::Matrix4d followerAffine = makeLegFollowerAffine(
            platform.parallelHomeBaseTransform,
            platform.parallelPose,
            follower.parallelFollowerHomeBaseAnchor,
            follower.parallelFollowerHomePlatformAnchor);

        for(const std::string& linkName : follower.parallelFollowerDrivenLinks) {
            const auto homeIt = follower.parallelFollowerHomeLinkTransforms.find(linkName);
            if(homeIt == follower.parallelFollowerHomeLinkTransforms.end()) {
                continue;
            }

            const auto node = follower.visualBridge->linkNode(linkName);
            if(!node) {
                continue;
            }

            const Eigen::Matrix4d linkTransform =
                followerAffine * homeIt->second.matrix();
            node->setLocal(math::eigenToGlm(linkTransform));
        }
    }

    void configureParallelControlIfNeeded(
        RuntimeRobot& runtime,
        const simulation_project::RobotDesc& robotDesc,
        const robot::RobotModel& model)
    {
        if(!isStewartSimscapePlatformModel(robotDesc, model)) {
            return;
        }

        runtime.parallelControlEnabled = true;
        runtime.parallelHomeBaseTransform = runtime.baseTransform;
        runtime.parallelGeometry = kine::StewartPlatformKinematics::makeDefaultGeometry();
        runtime.parallelPose = kine::StewartPlatformPose();

        std::string error;
        if(!kine::StewartPlatformKinematics::computeActuatorLengths(
               runtime.parallelGeometry,
               runtime.parallelPose,
               runtime.parallelActuatorLengths,
               &error)) {
            runtime.parallelControlEnabled = false;
            LOG_WARNING("rs2026") << "Failed to initialize Stewart platform control: robot="
                << robotDesc.id << ", error=" << error;
            return;
        }
        runtime.parallelActuatorHomeLengths = runtime.parallelActuatorLengths;

        LOG_INFO("rs2026") << "Stewart platform task-space control enabled: robot="
            << robotDesc.id
            << ", sourceModelIndex=" << robotDesc.sourceModelIndex;
    }

    void configureParallelFollowerIfNeeded(
        RuntimeRobot& runtime,
        const simulation_project::RobotDesc& robotDesc,
        const robot::RobotModel& model)
    {
        if(!isStewartSimscapeFollowerFragment(robotDesc, model)) {
            return;
        }

        runtime.parallelFollowerEnabled = true;
        runtime.parallelFollowerLegIndex = -1;
        runtime.parallelFollowerHomeTransform = runtime.baseTransform;

        LOG_INFO("rs2026") << "Stewart platform follower candidate enabled: robot="
            << robotDesc.id
            << ", sourceModelIndex=" << robotDesc.sourceModelIndex;
    }

    RuntimeRobot* findRuntimeRobot(
        std::vector<RuntimeRobot>& robots,
        const std::string& documentId)
    {
        for(RuntimeRobot& robot : robots) {
            if(robot.documentId == documentId) {
                return &robot;
            }
        }
        return nullptr;
    }

    RuntimeSceneObject* findRuntimeObject(
        std::vector<RuntimeSceneObject>& objects,
        const std::string& documentId)
    {
        for(RuntimeSceneObject& object : objects) {
            if(object.documentId == documentId) {
                return &object;
            }
        }
        return nullptr;
    }

    const RuntimeRobot* findRuntimeRobot(
        const std::vector<RuntimeRobot>& robots,
        const std::string& documentId)
    {
        for(const RuntimeRobot& robot : robots) {
            if(robot.documentId == documentId) {
                return &robot;
            }
        }
        return nullptr;
    }

    const RuntimeSceneObject* findRuntimeObject(
        const std::vector<RuntimeSceneObject>& objects,
        const std::string& documentId)
    {
        for(const RuntimeSceneObject& object : objects) {
            if(object.documentId == documentId) {
                return &object;
            }
        }
        return nullptr;
    }

    void appendConfiguredRobotPairs(
        const std::vector<RuntimeRobot>& robots,
        const simulation_project::CollisionQueryDesc& desc,
        std::vector<std::pair<ObjectID, ObjectID>>& pairs)
    {
        for(const auto& pairDesc : desc.pairs) {
            const RuntimeRobot* a = findRuntimeRobot(robots, pairDesc.robotA);
            const RuntimeRobot* b = findRuntimeRobot(robots, pairDesc.robotB);
            if(a == nullptr || b == nullptr || !a->collisionEnabled || !b->collisionEnabled ||
                !a->collisionInstance || !b->collisionInstance) {
                continue;
            }

            for(const auto& [keyA, objectA] : a->collisionInstance->objects()) {
                if(!objectA) {
                    continue;
                }
                const LinkInfo* infoA = a->collisionInstance->getLinkInfo(objectA->id());
                if(infoA == nullptr || !containsLink(pairDesc.includeLinksA, infoA->linkName)) {
                    continue;
                }

                for(const auto& [keyB, objectB] : b->collisionInstance->objects()) {
                    if(!objectB) {
                        continue;
                    }
                    const LinkInfo* infoB = b->collisionInstance->getLinkInfo(objectB->id());
                    if(infoB == nullptr || !containsLink(pairDesc.includeLinksB, infoB->linkName)) {
                        continue;
                    }

                    pairs.emplace_back(objectA->id(), objectB->id());
                }
            }
        }
    }

    void appendRobotObjectPairs(
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects,
        const simulation_project::CollisionQueryDesc& desc,
        std::vector<std::pair<ObjectID, ObjectID>>& pairs)
    {
        if(!desc.robotObjectPairs.empty()) {
            for(const auto& pairDesc : desc.robotObjectPairs) {
                if(!pairDesc.enabled) {
                    LOG_DEBUG("rs2026") << "Robot-object pair disabled: robot=" << pairDesc.robotId
                        << ", object=" << pairDesc.objectId;
                    continue;
                }

                const RuntimeRobot* robot = findRuntimeRobot(robots, pairDesc.robotId);
                const RuntimeSceneObject* object = findRuntimeObject(objects, pairDesc.objectId);
                if(robot == nullptr || object == nullptr || !robot->collisionEnabled || !object->collisionEnabled ||
                    !robot->collisionInstance || !object->collisionObject) {
                    LOG_WARNING("rs2026") << "Robot-object pair skipped: robot=" << pairDesc.robotId
                        << ", object=" << pairDesc.objectId
                        << ", hasRobot=" << (robot != nullptr)
                        << ", hasObject=" << (object != nullptr)
                        << ", robotCollisionEnabled=" << (robot != nullptr && robot->collisionEnabled)
                        << ", objectCollisionEnabled=" << (object != nullptr && object->collisionEnabled)
                        << ", hasRobotCollisionInstance=" << (robot != nullptr && robot->collisionInstance != nullptr)
                        << ", hasObjectCollisionObject=" << (object != nullptr && object->collisionObject != nullptr);
                    continue;
                }

                std::size_t appendedForPair = 0;
                for(const auto& [key, robotObject] : robot->collisionInstance->objects()) {
                    if(!robotObject) {
                        continue;
                    }
                    const LinkInfo* info = robot->collisionInstance->getLinkInfo(robotObject->id());
                    if(info == nullptr || !containsLink(pairDesc.includeRobotLinks, info->linkName)) {
                        continue;
                    }
                    pairs.emplace_back(robotObject->id(), object->collisionObject->id());
                    ++appendedForPair;
                }
                LOG_DEBUG("rs2026") << "Robot-object pair resolved: robot=" << pairDesc.robotId
                    << ", object=" << pairDesc.objectId
                    << ", robotCollisionObjects=" << robot->collisionInstance->objects().size()
                    << ", objectCollisionObject=" << object->collisionObject->id()
                    << ", includePairsAdded=" << appendedForPair;
            }
            return;
        }

        for(const RuntimeRobot& robot : robots) {
            if(!robot.collisionEnabled || !robot.collisionInstance) {
                continue;
            }

            for(const auto& object : objects) {
                if(!object.collisionEnabled || !object.collisionObject) {
                    continue;
                }

                for(const auto& [key, robotObject] : robot.collisionInstance->objects()) {
                    if(robotObject) {
                        pairs.emplace_back(robotObject->id(), object.collisionObject->id());
                    }
                }
            }
        }
    }

    std::vector<std::pair<ObjectID, ObjectID>> makeCollisionPairs(
        const std::vector<RuntimeRobot>& robots,
        const std::vector<RuntimeSceneObject>& objects,
        const simulation_project::CollisionQueryDesc& desc)
    {
        std::vector<std::pair<ObjectID, ObjectID>> pairs;
        appendConfiguredRobotPairs(robots, desc, pairs);
        appendRobotObjectPairs(robots, objects, desc, pairs);

        return pairs;
    }

    CollisionQueryOptions makeQueryOptions(
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
        options.includePairs = makeCollisionPairs(robots, objects, desc);
        options.scope = (!options.includePairs.empty() || !desc.pairs.empty() || !desc.robotObjectPairs.empty())
            ? CollisionQueryScope::SelectedObjects
            : CollisionQueryScope::All;
        return options;
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

    bool shouldRequestDetectorContacts(const simulation_project::CollisionDetectorDesc& desc)
    {
        return desc.contacts ||
            desc.visualization.showContacts ||
            desc.visualization.showNormals ||
            desc.visualization.showObjectHighlight;
    }

    bool modeShowsRobotMountHints(ProjectSceneInteractionMode mode)
    {
        return mode == ProjectSceneInteractionMode::SelectMount;
    }

    bool modeShowsAttachmentHints(ProjectSceneInteractionMode mode)
    {
        return mode == ProjectSceneInteractionMode::SelectMount ||
            mode == ProjectSceneInteractionMode::SelectAttachment ||
            mode == ProjectSceneInteractionMode::SelectCollisionTarget;
    }

    bool isFiniteVec3(const Eigen::Vector3d& value)
    {
        return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
    }

    ProjectScenePickCandidate makePointPickCandidate(
        ProjectScenePickTargetKind kind,
        const Eigen::Vector3d& center,
        double radius)
    {
        ProjectScenePickCandidate candidate;
        candidate.kind = kind;
        candidate.center = center;
        candidate.radius = radius;
        return candidate;
    }

    void setCandidateAabb(
        ProjectScenePickCandidate& candidate,
        const fcl::AABBd& aabb)
    {
        candidate.hasAabb = true;
        candidate.aabbMin = aabb.min_;
        candidate.aabbMax = aabb.max_;
        candidate.center = (candidate.aabbMin + candidate.aabbMax) * 0.5;
        candidate.radius = (candidate.aabbMax - candidate.aabbMin).norm() * 0.5;
    }

    void setCandidateTransformedBounds(
        ProjectScenePickCandidate& candidate,
        const collision::Transform3& transform,
        const collision::Vec3& localMin,
        const collision::Vec3& localMax)
    {
        Eigen::Vector3d boundsMin = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
        Eigen::Vector3d boundsMax = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());

        for(int x = 0; x < 2; ++x) {
            for(int y = 0; y < 2; ++y) {
                for(int z = 0; z < 2; ++z) {
                    const collision::Vec3 local(
                        x == 0 ? localMin.x() : localMax.x(),
                        y == 0 ? localMin.y() : localMax.y(),
                        z == 0 ? localMin.z() : localMax.z());
                    const collision::Vec3 world = transform * local;
                    boundsMin = boundsMin.cwiseMin(world);
                    boundsMax = boundsMax.cwiseMax(world);
                }
            }
        }

        candidate.hasAabb = true;
        candidate.aabbMin = boundsMin;
        candidate.aabbMax = boundsMax;
        candidate.center = (boundsMin + boundsMax) * 0.5;
        candidate.radius = (boundsMax - boundsMin).norm() * 0.5;
    }

    bool screenRayFromCamera(
        const cameracore::CameraPtr& camera,
        int width,
        int height,
        int x,
        int y,
        ProjectScenePickRay& ray)
    {
        if(!camera || width <= 0 || height <= 0) {
            return false;
        }

        const double ndcX = (2.0 * (static_cast<double>(x) + 0.5) / static_cast<double>(width)) - 1.0;
        const double ndcY = 1.0 - (2.0 * (static_cast<double>(y) + 0.5) / static_cast<double>(height));

        const Eigen::Matrix4d view = camera->view().cast<double>();
        const Eigen::Matrix4d projection = camera->projection().cast<double>();
        const Eigen::Matrix4d inverseViewProjection = (projection * view).inverse();

        Eigen::Vector4d nearPoint = inverseViewProjection * Eigen::Vector4d(ndcX, ndcY, -1.0, 1.0);
        Eigen::Vector4d farPoint = inverseViewProjection * Eigen::Vector4d(ndcX, ndcY, 1.0, 1.0);
        if(std::abs(nearPoint.w()) < 1.0e-9 || std::abs(farPoint.w()) < 1.0e-9) {
            return false;
        }

        nearPoint /= nearPoint.w();
        farPoint /= farPoint.w();
        Eigen::Vector3d direction = (farPoint.head<3>() - nearPoint.head<3>());
        if(direction.norm() <= 1.0e-9) {
            return false;
        }

        ray.origin = camera->position().cast<double>();
        ray.direction = direction.normalized();
        return isFiniteVec3(ray.origin) && isFiniteVec3(ray.direction);
    }

    bool intersectRayTriangle(
        const ProjectScenePickRay& ray,
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b,
        const Eigen::Vector3d& c,
        double& distance,
        double& barycentricB,
        double& barycentricC)
    {
        constexpr double kEpsilon = 1.0e-10;
        const Eigen::Vector3d edgeAB = b - a;
        const Eigen::Vector3d edgeAC = c - a;
        const Eigen::Vector3d p = ray.direction.cross(edgeAC);
        const double determinant = edgeAB.dot(p);
        if(std::abs(determinant) <= kEpsilon) {
            return false;
        }

        const double inverseDeterminant = 1.0 / determinant;
        const Eigen::Vector3d originToA = ray.origin - a;
        barycentricB = originToA.dot(p) * inverseDeterminant;
        if(barycentricB < 0.0 || barycentricB > 1.0) {
            return false;
        }

        const Eigen::Vector3d q = originToA.cross(edgeAB);
        barycentricC = ray.direction.dot(q) * inverseDeterminant;
        if(barycentricC < 0.0 || barycentricB + barycentricC > 1.0) {
            return false;
        }

        distance = edgeAC.dot(q) * inverseDeterminant;
        return distance > kEpsilon;
    }

    Eigen::Vector3d transformSurfacePoint(
        const glm::mat4& transform,
        const Eigen::Vector3d& point)
    {
        const glm::vec4 world = transform * glm::vec4(
            static_cast<float>(point.x()),
            static_cast<float>(point.y()),
            static_cast<float>(point.z()),
            1.0f);
        return Eigen::Vector3d(world.x, world.y, world.z);
    }

    void applyCollisionDetectorRuntimeOptions(
        ProjectCollisionDetectorRuntime& runtime,
        const simulation_project::CollisionDetectorDesc& desc)
    {
        runtime.name = desc.name.empty() ? desc.id : desc.name;
        runtime.type = desc.type;
        runtime.enabled = desc.enabled;
        runtime.visible = desc.visualization.visible;
        runtime.options.geometryRole = collisionRoleFromString(desc.geometryRole);
        runtime.options.geometrySource = desc.geometrySource;
        runtime.options.enableContacts = shouldRequestDetectorContacts(desc);
        runtime.options.enableNearestPoints = desc.nearestPoints;
        runtime.options.enableDistance = desc.distance;
        runtime.options.maxContacts = static_cast<std::size_t>(std::max(0, desc.maxContacts));
        runtime.options.distanceThreshold = desc.distanceThreshold;
        runtime.visualization = makeVisualizationOptions(desc.visualization);
        runtime.hasResult = false;
        runtime.lastResult.clear();
    }

    Eigen::Vector4f makeBackgroundColor(const simulation_project::ColorDesc& desc)
    {
        return Eigen::Vector4f(
            static_cast<float>(desc.r),
            static_cast<float>(desc.g),
            static_cast<float>(desc.b),
            static_cast<float>(desc.a));
    }

    Eigen::Vector4f makeEffectiveBackgroundColor(
        const simulation_project::ViewDesc& view,
        const simulation_project::ColorDesc& defaultColor)
    {
        return makeBackgroundColor(view.useThemeBackground ? defaultColor : view.backgroundColor);
    }

    using ToolAttachmentHighlightState = robot_render::MountedAttachmentHighlightState;

    struct RuntimeToolAttachmentVisual
    {
        std::string documentId;
        std::string name;
        std::string robotId;
        std::string linkName;
        std::string robotMountId;
        std::string toolAssetId;
        std::string assetKind;
        std::string assetType;
        std::string functionalFrameType;
        std::filesystem::path visualPath;
        double visualScale = 1.0;
        collision::Transform3 linkToMount = collision::Transform3::Identity();
        collision::Transform3 mountToAssetMount = collision::Transform3::Identity();
        collision::Transform3 assetMountToVisual = collision::Transform3::Identity();
        collision::Transform3 assetMountToTcp = collision::Transform3::Identity();
        collision::Transform3 worldLink = collision::Transform3::Identity();
        collision::Transform3 worldRobotMount = collision::Transform3::Identity();
        collision::Transform3 worldToolMount = collision::Transform3::Identity();
        collision::Transform3 worldVisual = collision::Transform3::Identity();
        collision::Transform3 worldTcp = collision::Transform3::Identity();
        bool visible = true;
        bool enabled = true;
        bool hasSensorIntrinsics = false;
        simulation_project::SensorIntrinsicsDesc sensorIntrinsics;
        robot_render::MountedAttachmentVisual visual;
        std::size_t collisionObjectIndex = std::numeric_limits<std::size_t>::max();
    };

    struct RuntimeToolAssetPreview
    {
        simulation_project::AttachmentAssetDesc asset;
        std::filesystem::path basePath;
        std::filesystem::path visualPath;
        collision::Transform3 mountToVisual = collision::Transform3::Identity();
        collision::Transform3 mountToTcp = collision::Transform3::Identity();
        std::shared_ptr<scenecore::SceneNode> rootNode;
        std::shared_ptr<scenecore::ModelNode> modelNode;
        glm::mat4 modelBaseLocal = glm::mat4(1.0f);
        glm::mat4 modelLocal = glm::mat4(1.0f);
    };

    simulation_project::RobotMountDesc* findRobotMountDesc(
        simulation_project::ProjectDocument& document,
        const std::string& id)
    {
        for(simulation_project::RobotMountDesc& mount : document.robotMounts) {
            if(mount.id == id) {
                return &mount;
            }
        }
        return nullptr;
    }

    const simulation_project::RobotMountDesc* findRobotMountDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& id)
    {
        for(const simulation_project::RobotMountDesc& mount : document.robotMounts) {
            if(mount.id == id) {
                return &mount;
            }
        }
        return nullptr;
    }

    simulation_project::SceneObjectDesc* findSceneObjectDesc(
        simulation_project::ProjectDocument& document,
        const std::string& objectId)
    {
        for(simulation_project::SceneObjectDesc& object : document.objects) {
            if(object.id == objectId) {
                return &object;
            }
        }
        return nullptr;
    }

    const simulation_project::SceneObjectDesc* findSceneObjectDesc(
        const simulation_project::ProjectDocument& document,

        const std::string& objectId)
    {
        for(const simulation_project::SceneObjectDesc& object : document.objects) {
            if(object.id == objectId) {
                return &object;
            }
        }
        return nullptr;
    }

    const simulation_project::RobotDesc* findRobotDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& robotId)
    {
        for(const simulation_project::RobotDesc& robot : document.robots) {
            if(robot.id == robotId) {
                return &robot;
            }
        }
        return nullptr;
    }

    std::string activeObjectCollisionModelId(
        const simulation_project::CollisionSceneDesc& collisionDesc,
        const std::string& objectId)
    {
        for(const simulation_project::ObjectCollisionModelSelectionDesc& selection :
            collisionDesc.objectModelSelections) {
            if(selection.objectId == objectId && !selection.activeModelId.empty()) {
                return selection.activeModelId;
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

    CollisionGeneratedAssetRequest makeGeneratedAssetRequest(
        const simulation_project::ProjectDocument& document,
        const std::filesystem::path& projectBasePath,
        const std::string& sourceKey,
        const std::string& targetKey)
    {
        CollisionGeneratedAssetRequest request;
        const simulation_project::AssetResolveContext context =
            ProjectRuntimeBuilder::makeAssetResolveContext(projectBasePath, document.assetSearchPaths);
        request.generatedAssetRoot =
            simulation_project::AssetResolver::appGeneratedAssetRoot(context);
        request.sourceKey = sourceKey;
        request.targetKey = targetKey;
        request.role = simulation_project::kCoacdCollisionModelRole;
        return request;
    }

    const simulation_project::PointCloudDesc* findPointCloudDesc(
        const simulation_project::ProjectDocument& document,
        const std::string& pointCloudId)
    {
        for(const simulation_project::PointCloudDesc& pointCloud : document.pointClouds) {
            if(pointCloud.id == pointCloudId) {
                return &pointCloud;
            }
        }
        return nullptr;
    }

    simulation_project::ObjectFrameDesc* findObjectFrameDesc(
        simulation_project::SceneObjectDesc& object,
        const std::string& frameId)
    {
        for(simulation_project::ObjectFrameDesc& frame : object.objectFrames) {
            if(frame.id == frameId) {
                return &frame;
            }
        }
        return nullptr;
    }

    const simulation_project::ObjectFrameDesc* findObjectFrameDesc(
        const simulation_project::SceneObjectDesc& object,
        const std::string& frameId)
    {
        for(const simulation_project::ObjectFrameDesc& frame : object.objectFrames) {
            if(frame.id == frameId) {
                return &frame;
            }
        }
        return nullptr;
    }

    void setToolAttachmentHighlightState(
        RuntimeToolAttachmentVisual& attachment,
        ToolAttachmentHighlightState highlightState)
    {
        robot_render::MountedAttachmentVisualBridge::setHighlightState(
            attachment.visual,
            highlightState);
    }

    void updateToolAttachmentHighlights(
        std::vector<RuntimeToolAttachmentVisual>& attachments,
        const std::vector<RuntimeSceneObject>& objects,
        const std::unordered_set<ObjectID>& collidingObjects,
        const std::string& selectedToolAttachment)
    {
        for(RuntimeToolAttachmentVisual& attachment : attachments) {
            const bool selected = attachment.documentId == selectedToolAttachment;
            bool colliding = false;
            if(attachment.collisionObjectIndex < objects.size()) {
                for(const RuntimeSceneCollisionObject& collisionObject : objects[attachment.collisionObjectIndex].collisionObjects) {
                    colliding = colliding ||
                        (collisionObject.collisionObject &&
                            collidingObjects.count(collisionObject.collisionObject->id()) > 0);
                }
            }
            setToolAttachmentHighlightState(
                attachment,
                selected
                    ? ToolAttachmentHighlightState::Selected
                    : (colliding ? ToolAttachmentHighlightState::Colliding : ToolAttachmentHighlightState::None));
        }
    }

    const simulation_project::AssetView* findAsset(
        const simulation_project::ProjectAttachmentSetView& attachments,
        const std::string& id)
    {
        for(const simulation_project::AssetView& asset : attachments.assets) {
            if(asset.id == id) {
                return &asset;
            }
        }
        return nullptr;
    }

    const simulation_project::FunctionalFrameView* primaryFunctionalFrame(
        const simulation_project::AssetView& asset)
    {
        for(const simulation_project::FunctionalFrameView& frame : asset.functionalFrames) {
            if(frame.primary) {
                return &frame;
            }
        }
        return asset.functionalFrames.empty() ? nullptr : &asset.functionalFrames.front();
    }

    simulation_project::TransformDesc primaryAssetFrameTransform(
        const simulation_project::AttachmentAssetDesc& asset)
    {
        for(const simulation_project::AttachmentFunctionalFrameDesc& frame : asset.functionalFrames) {
            if(frame.primary) {
                return frame.assetMountToFrame;
            }
        }
        return asset.functionalFrames.empty()
            ? simulation_project::TransformDesc()
            : asset.functionalFrames.front().assetMountToFrame;
    }

    std::shared_ptr<rendercore::Material> makeToolMaterial()
    {
        auto material = std::make_shared<rendercore::Material>();
        material->baseColor = Eigen::Vector4f(0.72f, 0.74f, 0.76f, 1.0f);
        material->specular = Eigen::Vector3f(0.25f, 0.25f, 0.25f);
        material->shininess = 20.0f;
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

    void drawFrameMarker(
        scenecore::DebugDraw& debug,
        const collision::Transform3& transform,
        float frameScale,
        const glm::vec4& markerColor,
        float thicknessScale = 1.0f)
    {
        debug.drawFrame(math::eigenToGlm(transform), frameScale, thicknessScale);
        const collision::Vec3 p = transform.translation();
        const glm::mat4 markerTransform = glm::translate(
            glm::mat4(1.0f),
            glm::vec3(
                static_cast<float>(p.x()),
                static_cast<float>(p.y()),
                static_cast<float>(p.z())));
        debug.drawSphere(markerTransform, frameScale * 0.12f, markerColor);
    }

    glm::vec3 toGlmVec3(const collision::Vec3& value)
    {
        return glm::vec3(
            static_cast<float>(value.x()),
            static_cast<float>(value.y()),
            static_cast<float>(value.z()));
    }

    bool isSensorAttachment(const RuntimeToolAttachmentVisual& attachment)
    {
        return attachment.assetKind == "sensor" ||
            attachment.functionalFrameType == "optical" ||
            attachment.functionalFrameType == "measurement" ||
            attachment.hasSensorIntrinsics;
    }

    const simulation_project::ObjectCollisionOverrideDesc* findToolAttachmentCollisionOverride(
        const simulation_project::ProjectDocument& document,
        const std::string& attachmentId)
    {
        const simulation_project::ObjectCollisionOverrideDesc* collisionOverride =
            ProjectRuntimeBuilder::findObjectCollisionOverride(document.collision, attachmentId);
        if(collisionOverride != nullptr) {
            return collisionOverride;
        }

        for(const simulation_project::MountedAttachmentDesc& attachment : document.mountedAttachments) {
            if(attachment.id == attachmentId && !attachment.sourceObjectId.empty()) {
                return ProjectRuntimeBuilder::findObjectCollisionOverride(
                    document.collision,
                    attachment.sourceObjectId);
            }
        }

        return nullptr;
    }

    void drawSensorDirection(
        scenecore::DebugDraw& debug,
        const collision::Transform3& sensorFrame,
        double length,
        const glm::vec4& color)
    {
        const collision::Vec3 origin = sensorFrame.translation();
        const collision::Vec3 forward =
            origin + sensorFrame.linear() * collision::Vec3(0.0, 0.0, length);
        debug.drawLine(toGlmVec3(origin), toGlmVec3(forward), color);
    }

    void drawSensorFovPreview(
        scenecore::DebugDraw& debug,
        const collision::Transform3& sensorFrame,
        const simulation_project::SensorIntrinsicsDesc& intrinsics)
    {
        const int width = std::max(1, intrinsics.width);
        const int height = std::max(1, intrinsics.height);
        const double aspect = static_cast<double>(width) / static_cast<double>(height);
        const double tanHalfFovY = std::tan(intrinsics.fovY * kPi / 360.0);
        const double tanHalfFovX = tanHalfFovY * aspect;
        const double farDistance = std::max(intrinsics.nearPlane, std::min(intrinsics.farPlane, 2.0));
        const double halfY = farDistance * tanHalfFovY;
        const double halfX = farDistance * tanHalfFovX;

        const collision::Vec3 origin = sensorFrame.translation();
        const auto worldPoint = [&](double x, double y, double z) {
            return sensorFrame * collision::Vec3(x, y, z);
        };

        const collision::Vec3 corners[] = {
            worldPoint(-halfX, -halfY, farDistance),
            worldPoint( halfX, -halfY, farDistance),
            worldPoint( halfX,  halfY, farDistance),
            worldPoint(-halfX,  halfY, farDistance)
        };

        const glm::vec4 fovColor(0.1f, 0.85f, 1.0f, 1.0f);
        for(const collision::Vec3& corner : corners) {
            debug.drawLine(toGlmVec3(origin), toGlmVec3(corner), fovColor);
        }
        for(int i = 0; i < 4; ++i) {
            debug.drawLine(toGlmVec3(corners[i]), toGlmVec3(corners[(i + 1) % 4]), fovColor);
        }
    }
}

struct ProjectScene::Impl
{
    struct ObjectCollisionVariantMeshPreview
    {
        std::shared_ptr<VisibleModelNode> node;
        glm::mat4 targetLocal = glm::mat4(1.0f);
        bool configured = false;
    };

    struct ObjectCollisionVariantMeshPreviewCache
    {
        std::vector<ObjectCollisionVariantMeshPreview> previews;
        std::uint64_t revision = 0;
    };

    bool initialized = false;
    bool showCollisionGeometry = false;
    bool reportedCollision = false;
    std::size_t lastContactCount = static_cast<std::size_t>(-1);
    std::size_t lastIncludePairCount = static_cast<std::size_t>(-1);
    std::uint64_t collisionQueryFrame = 0;
    double lastRobotPoseUpdateMs = 0.0;
    double lastCollisionWorldUpdateMs = 0.0;
    double lastCollisionOverlayMs = 0.0;
    double lastCollisionOverlayHighlightMs = 0.0;
    double lastCollisionOverlayDebugBuildMs = 0.0;
    double lastCollisionOverlayVariantFilterMs = 0.0;
    double lastCollisionOverlayDebugSubmitMs = 0.0;
    double lastCollisionOverlayAuxFramesMs = 0.0;
    std::size_t lastCollisionOverlayDetectorCount = 0;
    std::size_t lastCollisionOverlayGeometryCount = 0;
    std::size_t lastCollisionOverlayContactCount = 0;
    std::size_t lastCollisionOverlayNearestCount = 0;
    std::size_t lastCollisionOverlayPrimitiveEstimate = 0;
    std::size_t lastCollisionOverlayLineEstimate = 0;
    simulation_runtime::ProjectSelectionState selectionState;
    ProjectSceneInteractionMode interactionMode = ProjectSceneInteractionMode::Browse;
    std::string activePreviewRobotMountId;
    std::vector<MaterialOverride> selectionOverrides;
    std::vector<MaterialOverride> pairPreviewOverrides;
    int width = 1;
    int height = 1;
    cameracore::CameraControllerPtr controller;
    scenecore::SceneGraph sceneGraph;
    scenecore::Renderer renderer;
    std::shared_ptr<scenecore::CameraNode> mainCameraNode;
    std::shared_ptr<scenecore::MeshPass> collisionMeshPass;
    std::shared_ptr<scenecore::PrimitivePass> collisionPrimitivePass;
    std::shared_ptr<scenecore::TrajectoryPass> collisionLinePass;
    simulation_project::ProjectDocument projectDocument;
    bool projectDocumentSet = false;
    std::filesystem::path projectBasePath;
    std::vector<RuntimeRobot> robots;
    std::vector<RuntimeSceneObject> objects;
    simulation_runtime::RuntimeMountedAttachmentGraph mountedAttachments;
    std::vector<RuntimeToolAttachmentVisual> toolAttachments;
    bool toolAssetPreviewSet = false;
    RuntimeToolAssetPreview toolAssetPreview;
    std::size_t activeToolAttachmentIndex = static_cast<std::size_t>(-1);
    std::string activeToolFrameRobotId;
    std::string selectedJointFrameRobotId;
    std::string selectedJointFrameName;
    std::string selectedObjectFrameObjectId;
    std::string selectedObjectFrameId;
    ProjectScene::ToolFrameVisibility toolFrameVisibility;
    bool previewSelectedLinkFrameVisible = false;
    bool previewRobotMountFrameVisible = false;
    std::vector<std::string> pinnedRobotMountFrameIds;
    std::vector<ProjectScene::RobotLinkGroup> robotLinks;
    std::vector<ProjectScene::SceneObjectGroup> sceneObjects;
    std::vector<ProjectScene::PointCloudGroup> pointClouds;
    CollisionScene collisionScene;
    std::vector<ProjectCollisionDetectorRuntime> collisionDetectors;
    bool collisionQueriesEnabled = false;
    bool collisionRuntimeBuilt = false;
    bool collisionRuntimeBuildInProgress = false;
    std::string activeCollisionDetectorId;
    std::unordered_map<std::string, std::string> visibleCollisionVariantIds;
    std::unordered_map<std::string, VisibleCollisionVariantFilter> visibleCollisionVariantFilters;
    std::unordered_map<std::string, VisibleCollisionVariantFilter> visibleCollisionVariantFiltersByRuntimeLink;
    CollisionGeometryOverlayCache collisionGeometryOverlayCache;
    std::uint64_t collisionGeometryOverlayCacheVersion = 1;
    simulation_project::ColorDesc defaultBackgroundColor;
    Eigen::Vector4f backgroundColor = Eigen::Vector4f(0.05f, 0.06f, 0.08f, 1.0f);
    bool mountFrameLinkFocusActive = false;
    std::string mountFrameFocusRobotId;
    std::string mountFrameFocusLinkName;
    bool objectFrameObjectFocusActive = false;
    std::string objectFrameFocusObjectId;
    bool mountedAttachmentFocusActive = false;
    std::string mountedAttachmentFocusId;
    bool previewObjectCollisionModelActive = false;
    std::string previewObjectCollisionModelObjectId;
    std::string previewObjectCollisionModelVariantId;
    std::unordered_map<std::string, ObjectCollisionVariantMeshPreviewCache>
        objectCollisionVariantMeshPreviews;
    std::uint64_t objectCollisionVariantMeshPreviewRevision = 1;
    CameraLookAtState cameraLookAt;
    bool cameraAnimationActive = false;
    CameraLookAtState cameraAnimationStart;
    CameraLookAtState cameraAnimationEnd;
    double cameraAnimationElapsed = 0.0;
    double cameraAnimationDuration = 0.45;
    double lastCameraAnimationUpdateTime = -1.0;

    bool initializeOpenGlRuntime();
    void applyCollisionOverlayRenderConfig();
    void ensureProjectDocument();
    CameraSceneBounds fullSceneBounds() const;
    CameraSceneBounds robotLinkBounds(const std::string& robotId, const std::string& linkName) const;
    CameraSceneBounds sceneObjectBounds(const std::string& objectId) const;
    CameraSceneBounds mountedAttachmentBounds(const std::string& attachmentId) const;
    void applyCameraLookAt(const CameraLookAtState& state);
    void animateCameraTo(const CameraLookAtState& state, double duration);
    void updateCameraAnimation(double timeSeconds);
    void applyMountFrameLinkFocusVisibility();
    void invalidateCollisionRuntime();
    std::vector<simulation_runtime::RuntimeAttachmentRobotContext> attachmentRobotContexts() const;
    bool rebuildMountedAttachmentGraph();
    void buildProjectRobots();
    void buildProjectObjects();
    void buildProjectPointClouds();
    void buildProjectToolAttachments();
    void syncProjectToolAttachments();
    void configureStewartGroups();
    void syncStewartFollowers(const RuntimeRobot& platform);
    void syncAllStewartFollowers();
    void setStewartPlatformBaseTransform(RuntimeRobot& platform, const collision::Transform3& baseTransform);
    void applyStewartInternalPlatformVisualOverrides(RuntimeRobot& platform);
    void applyStewartFollowerVisualOverrides(const RuntimeRobot& platform);
    void applyAllStewartFollowerVisualOverrides();
    void ensureToolAttachmentCollisionObjects();
    bool ensureCollisionRuntimeBuilt();
    void registerRuntimeCollisionObjects();
    void drawSelectedJointFrame();
    void drawVisibleObjectFrames();
    const RuntimeToolAttachmentVisual* mountedAttachmentForSourceObject(
        const std::string& objectId) const;
    void drawSelectedObjectFrame();
    void drawPreviewRobotMountFrames();
    void drawPinnedRobotMountFrames();
    void drawActiveToolAttachmentFrames();
    void drawRobotCollisionModelVariantPreview();
    void drawObjectCollisionModelVariantPreview();
    void updateObjectCollisionModelVariantPreviewMeshes();
    void hideObjectCollisionModelVariantPreviewMeshes();
    void drawInteractionModeHints();
    void drawToolAssetPreviewFrames();
    void buildToolAssetPreview();
    void updateToolAssetPreviewModel();
    void applyActiveToolAttachmentVisibility();
    const RuntimeToolAttachmentVisual* activeToolFrameAttachment() const;
    bool cycleActiveToolAttachment(bool reverse);
    bool setActiveToolAttachment(const std::string& id);
    void setActiveToolFrameRobot(const std::string& robotId);
    void applyProjectCollisionAndView();
    ProjectCollisionDetectorRuntime* activeCollisionDetector();
    const ProjectCollisionDetectorRuntime* activeCollisionDetector() const;
    std::string visibleCollisionVariantId(
        const std::string& robotId,
        const std::string& linkName) const;
    void invalidateCollisionGeometryOverlayCache();
    void filterCollisionDebugDrawByVisibleVariants(CollisionDebugDrawData& data) const;
};

std::string ProjectScene::Impl::visibleCollisionVariantId(
    const std::string& robotId,
    const std::string& linkName) const
{
    const auto it = visibleCollisionVariantIds.find(collisionVariantSelectionKey(robotId, linkName));
    return it != visibleCollisionVariantIds.end() ? it->second : std::string();
}

void ProjectScene::Impl::invalidateCollisionGeometryOverlayCache()
{
    collisionGeometryOverlayCache.clear();
    ++collisionGeometryOverlayCacheVersion;
}

void ProjectScene::Impl::filterCollisionDebugDrawByVisibleVariants(CollisionDebugDrawData& data) const
{
    if(visibleCollisionVariantFilters.empty() || data.geometry.empty()) {
        return;
    }

    std::vector<CollisionDebugDrawDesc> filtered;
    filtered.reserve(data.geometry.size());

    for(const CollisionDebugDrawDesc& desc : data.geometry) {
        if(desc.robotInstance < 0 || desc.linkName.empty()) {
            filtered.push_back(desc);
            continue;
        }

        const std::string key = runtimeCollisionVariantSelectionKey(desc.robotInstance, desc.linkName);
        const auto filterIt = visibleCollisionVariantFiltersByRuntimeLink.find(key);
        if(filterIt == visibleCollisionVariantFiltersByRuntimeLink.end() ||
            filterIt->second.elementNames.count(desc.elementName) > 0) {
            filtered.push_back(desc);
        }
    }

    data.geometry = std::move(filtered);
}

void ProjectScene::Impl::drawRobotCollisionModelVariantPreview()
{
    if(visibleCollisionVariantFilters.empty()) {
        return;
    }

    scenecore::DebugDraw& debug = renderer.debug();
    const glm::vec4 previewColor(
        kCollisionPreviewRed,
        kCollisionPreviewGreen,
        kCollisionPreviewBlue,
        kCollisionPreviewAlpha);
    const scenecore::RenderTag previewTag{
        scenecore::RenderLayer::Gizmo,
        scenecore::RenderCategory::Debug,
        scenecore::RenderFeature::Gizmo };

    for(const auto& entry : visibleCollisionVariantFilters) {
        const VisibleCollisionVariantFilter& filter = entry.second;
        if(simulation_project::isConvertFromVisualCollisionModelId(filter.variantId)) {
            continue;
        }

        const RuntimeRobot* robot = findRuntimeRobot(robots, filter.robotId);
        if(robot == nullptr || !robot->instance) {
            continue;
        }

        const auto linkIt = robot->model.links.find(filter.linkName);
        if(linkIt == robot->model.links.end()) {
            continue;
        }

        const collision::Transform3& linkTransform =
            robot->instance->getLinkTransform(filter.linkName);

        for(const robot::RobotCollisionGeometry& geometry : linkIt->second.collisions) {
            if(!geometry.enabled || filter.elementNames.count(geometry.partUid) == 0) {
                continue;
            }

            const glm::mat4 transform =
                math::eigenToGlm(linkTransform * geometry.T_part);
            switch(geometry.type) {
            case robot::RobotGeometryType::Box:
                debug.drawCube(
                    transform,
                    math::eigenToGlm(geometry.boxSize),
                    previewColor,
                    scenecore::DrawType::UsingUnlitShader,
                    previewTag);
                break;
            case robot::RobotGeometryType::Sphere:
                if(std::isfinite(geometry.radius) && geometry.radius > 0.0) {
                    debug.drawSphere(
                        transform,
                        static_cast<float>(geometry.radius),
                        previewColor,
                        scenecore::DrawType::UsingUnlitShader,
                        previewTag);
                }
                break;
            case robot::RobotGeometryType::Cylinder:
                if(std::isfinite(geometry.radius) && geometry.radius > 0.0 &&
                    std::isfinite(geometry.length) && geometry.length > 0.0) {
                    debug.drawCylinder(
                        transform,
                        static_cast<float>(geometry.radius),
                        static_cast<float>(geometry.length),
                        previewColor,
                        scenecore::DrawType::UsingUnlitShader,
                        previewTag);
                }
                break;
            default:
                break;
            }
        }
    }
}

bool ProjectScene::Impl::initializeOpenGlRuntime()
{
    logInit();

    if(!GLRuntime::instance().initialize()) {
        return false;
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    rendercore::ShaderLibrary::initialize();
    renderer.initialize();

    mainCameraNode = createMainCamera(sceneGraph);
    scenecore::DefaultLighting::createDefaultLighting(sceneGraph, 0.6f);
    controller = cameracore::CameraFactory::createOrbitController(mainCameraNode->camera());

    collisionMeshPass = std::make_shared<scenecore::MeshPass>(rendercore::ShaderLibrary::getShader("MLRobotUBO"));
    collisionPrimitivePass = std::make_shared<scenecore::PrimitivePass>();
    collisionLinePass = std::make_shared<scenecore::TrajectoryPass>();
    applyCollisionOverlayRenderConfig();

    renderer.addPass(std::make_shared<scenecore::GridPass>(scenecore::GridType::FadedInfiniteGrid));
    renderer.addPass(std::make_shared<scenecore::MeshPass>(rendercore::ShaderLibrary::getShader("MLRobotUBO")));
    renderer.addPass(collisionMeshPass);
    renderer.addPass(std::make_shared<scenecore::PointCloudPass>());
    renderer.addPass(std::make_shared<scenecore::AxisPass>());
    renderer.addPass(collisionPrimitivePass);
    renderer.addPass(collisionLinePass);
    return true;
}

void ProjectScene::Impl::applyCollisionOverlayRenderConfig()
{
    scenecore::CollisionOverlayRenderConfig overlayConfig;
    overlayConfig.depthMode = scenecore::OverlayDepthMode::AlwaysOnTop;

    if(!showCollisionGeometry) {
        overlayConfig.showExact = false;
        overlayConfig.showSimplified = false;
        overlayConfig.showSafety = false;
        overlayConfig.showPlanningProxy = false;
    }

    if(collisionPrimitivePass) {
        collisionPrimitivePass->setCollisionOverlayConfig(overlayConfig);
    }
    if(collisionMeshPass) {
        collisionMeshPass->setCollisionOverlayConfig(overlayConfig);
    }
    if(collisionLinePass) {
        collisionLinePass->setCollisionOverlayConfig(overlayConfig);
    }
}

void ProjectScene::Impl::ensureProjectDocument()
{
    const std::filesystem::path projectPath = simulation_project::RuntimePaths::applicationRoot();
    if(!projectDocumentSet) {
        const std::filesystem::path defaultProjectFile =
            simulation_project::RuntimePaths::configRoot() / "projects" / "420.scene.20260625.json";

        std::string loadError;
        if(!simulation_project::loadProjectDocument(defaultProjectFile, projectDocument, &loadError)) {
            throw std::runtime_error("Failed to load default scene project: " + loadError);
        }
        projectBasePath = projectPath;
    } else if(projectBasePath.empty()) {
        projectBasePath = projectPath;
    }
}

CameraSceneBounds ProjectScene::Impl::fullSceneBounds() const
{
    CameraSceneBounds bounds;
    bounds.includePoint(Vec3::Zero());

    for(const RuntimeRobot& robot : robots) {
        bounds.includePoint(robot.baseTransform.translation());
        if(robot.instance) {
            for(const auto& linkItem : robot.model.links) {
                bounds.includePoint(robot.instance->getLinkTransform(linkItem.first).translation());
            }
        }
        if(robot.collisionInstance) {
            for(const auto& objectItem : robot.collisionInstance->objects()) {
                if(objectItem.second) {
                    bounds.includeAabb(objectItem.second->fcl()->getAABB());
                }
            }
        }
    }

    for(const RuntimeSceneObject& object : objects) {
        bounds.includePoint(object.transform.translation());
        if(object.pointCloudBoundsValid) {
            bounds.includeTransformedBounds(
                object.transform,
                object.pointCloudLocalBoundsMin,
                object.pointCloudLocalBoundsMax);
        }
        if(object.collisionObject) {
            bounds.includeAabb(object.collisionObject->fcl()->getAABB());
        }
        for(const RuntimeSceneCollisionObject& collisionObject : object.collisionObjects) {
            if(collisionObject.collisionObject) {
                bounds.includeAabb(collisionObject.collisionObject->fcl()->getAABB());
            }
        }
    }

    for(const RuntimeToolAttachmentVisual& attachment : toolAttachments) {
        if(!attachment.visible) {
            continue;
        }
        bounds.includePoint(attachment.worldToolMount.translation());
        bounds.includePoint(attachment.worldVisual.translation());
        bounds.includePoint(attachment.worldTcp.translation());
    }

    return bounds;
}

CameraSceneBounds ProjectScene::Impl::robotLinkBounds(
    const std::string& robotId,
    const std::string& linkName) const
{
    CameraSceneBounds bounds;
    const RuntimeRobot* robot = findRuntimeRobot(robots, robotId);
    if(robot == nullptr || linkName.empty()) {
        return bounds;
    }

    if(robot->instance && robot->model.links.find(linkName) != robot->model.links.end()) {
        bounds.includePoint(robot->instance->getLinkTransform(linkName).translation());
    }

    if(robot->collisionInstance) {
        for(const auto& objectItem : robot->collisionInstance->objects()) {
            if(!objectItem.second) {
                continue;
            }

            const LinkInfo* info = robot->collisionInstance->getLinkInfo(objectItem.second->id());
            if(info != nullptr && info->linkName == linkName) {
                bounds.includeAabb(objectItem.second->fcl()->getAABB());
            }
        }
    }

    return bounds;
}

CameraSceneBounds ProjectScene::Impl::sceneObjectBounds(const std::string& objectId) const
{
    CameraSceneBounds bounds;
    const RuntimeSceneObject* object = findRuntimeObject(objects, objectId);
    if(object == nullptr) {
        return bounds;
    }

    bounds.includePoint(object->transform.translation());
    if(object->pointCloudBoundsValid) {
        bounds.includeTransformedBounds(
            object->transform,
            object->pointCloudLocalBoundsMin,
            object->pointCloudLocalBoundsMax);
    }
    if(object->collisionObject) {
        bounds.includeAabb(object->collisionObject->fcl()->getAABB());
    }
    for(const RuntimeSceneCollisionObject& collisionObject : object->collisionObjects) {
        if(collisionObject.collisionObject) {
            bounds.includeAabb(collisionObject.collisionObject->fcl()->getAABB());
        }
    }
    return bounds;
}

CameraSceneBounds ProjectScene::Impl::mountedAttachmentBounds(const std::string& attachmentId) const
{
    CameraSceneBounds bounds;
    const RuntimeToolAttachmentVisual* attachment = nullptr;
    for(const RuntimeToolAttachmentVisual& candidate : toolAttachments) {
        if(candidate.documentId == attachmentId) {
            attachment = &candidate;
            break;
        }
    }
    if(attachment == nullptr) {
        return bounds;
    }

    bounds.includePoint(attachment->worldToolMount.translation());
    bounds.includePoint(attachment->worldVisual.translation());
    bounds.includePoint(attachment->worldTcp.translation());

    if(!attachment->visualPath.empty()) {
        std::shared_ptr<assetcore::ModelDesc> modelDesc =
            assetcore::AssetManager::instance().loadModel(
                pathToUtf8(attachment->visualPath),
                static_cast<float>(attachment->visualScale));
        if(modelDesc) {
            const ObjMesh mesh = makeObjMesh(*modelDesc, 1.0);
            const MeshBounds meshBounds = computeMeshBounds(mesh);
            if(meshBounds.valid) {
                bounds.includeTransformedBounds(
                    attachment->worldVisual,
                    meshBounds.min,
                    meshBounds.max);
            }
        }
    }

    return bounds;
}

void ProjectScene::Impl::applyCameraLookAt(const CameraLookAtState& state)
{
    if(!state.valid || !mainCameraNode || !mainCameraNode->camera()) {
        return;
    }

    mainCameraNode->camera()->lookAt(state.position, state.target, state.up);
    cameraLookAt = state;
}

void ProjectScene::Impl::animateCameraTo(const CameraLookAtState& state, double duration)
{
    if(!state.valid || !mainCameraNode || !mainCameraNode->camera()) {
        return;
    }

    if(!cameraLookAt.valid) {
        cameraLookAt.position = mainCameraNode->camera()->position();
        cameraLookAt.target = Vec3::Zero().cast<float>();
        cameraLookAt.up = Eigen::Vector3f(0.0f, 0.0f, 1.0f);
        cameraLookAt.valid = true;
    } else {
        cameraLookAt.position = mainCameraNode->camera()->position();
    }

    cameraAnimationStart = cameraLookAt;
    cameraAnimationEnd = state;
    cameraAnimationElapsed = 0.0;
    cameraAnimationDuration = std::max(0.05, duration);
    lastCameraAnimationUpdateTime = -1.0;
    cameraAnimationActive = true;
}

void ProjectScene::Impl::updateCameraAnimation(double timeSeconds)
{
    if(!cameraAnimationActive) {
        return;
    }

    if(lastCameraAnimationUpdateTime < 0.0) {
        lastCameraAnimationUpdateTime = timeSeconds;
    }
    const double delta = std::max(0.0, timeSeconds - lastCameraAnimationUpdateTime);
    lastCameraAnimationUpdateTime = timeSeconds;
    cameraAnimationElapsed += delta;

    const double linearT = cameraAnimationDuration > 0.0
        ? std::clamp(cameraAnimationElapsed / cameraAnimationDuration, 0.0, 1.0)
        : 1.0;
    const double smoothT = linearT * linearT * (3.0 - 2.0 * linearT);
    applyCameraLookAt(interpolateCameraLookAt(cameraAnimationStart, cameraAnimationEnd, smoothT));
    if(linearT >= 1.0) {
        cameraAnimationActive = false;
        applyCameraLookAt(cameraAnimationEnd);
    }
}

void ProjectScene::Impl::applyMountFrameLinkFocusVisibility()
{
    const bool showFullScene =
        !mountFrameLinkFocusActive &&
        !objectFrameObjectFocusActive &&
        !mountedAttachmentFocusActive;
    for(RuntimeRobot& robot : robots) {
        const bool focusedRobot =
            mountFrameLinkFocusActive &&
            robot.documentId == mountFrameFocusRobotId;
        if(robot.visualBridge && robot.visualBridge->rootNode()) {
            robot.visualBridge->rootNode()->setVisible(showFullScene || focusedRobot);
        }

        if(!robot.visualBridge) {
            continue;
        }

        for(const std::string& linkName : robot.model.linkNames) {
            const std::shared_ptr<scenecore::SceneNode> linkNode = robot.visualBridge->linkNode(linkName);
            if(linkNode) {
                linkNode->setVisible(
                    showFullScene ||
                    (focusedRobot && linkName == mountFrameFocusLinkName));
            }
        }
    }

    for(RuntimeSceneObject& object : objects) {
        bool visible = objectFrameObjectFocusActive && object.documentId == objectFrameFocusObjectId;
        if(showFullScene) {
            if(object.pointCloudNode) {
                visible = true;
                if(const simulation_project::PointCloudDesc* pointCloud =
                    findPointCloudDesc(projectDocument, object.documentId)) {
                    visible = pointCloud->visualization.visible;
                }
            } else if(object.visualNode) {
                visible = true;
                if(const simulation_project::SceneObjectDesc* sceneObject =
                    findSceneObjectDesc(projectDocument, object.documentId)) {
                    visible = sceneObject->visible;
                }
            }
        }
        if(object.visualNode) {
            object.visualNode->setVisible(visible);
        }
        if(object.pointCloudNode) {
            object.pointCloudNode->setVisible(visible);
        }
    }

    for(RuntimeToolAttachmentVisual& attachment : toolAttachments) {
        bool attachmentVisible = showFullScene
            ? attachment.visible
            : (mountedAttachmentFocusActive && attachment.documentId == mountedAttachmentFocusId);
        robot_render::MountedAttachmentVisualBridge::setVisible(
            attachment.visual,
            attachmentVisible);
    }
}

void ProjectScene::Impl::invalidateCollisionRuntime()
{
    collisionRuntimeBuilt = false;
    collisionRuntimeBuildInProgress = false;
    collisionScene = CollisionScene();

    for(RuntimeRobot& runtime : robots) {
        runtime.collisionModel.reset();
        runtime.collisionInstance.reset();
    }

    for(RuntimeSceneObject& runtime : objects) {
        if(runtime.objectType == "pointCloud") {
            continue;
        }
        runtime.collisionObjects.clear();
        runtime.collisionObject.reset();
        runtime.collisionShape = CollisionShapeDesc();
    }

    objects.erase(
        std::remove_if(
            objects.begin(),
            objects.end(),
            [](const RuntimeSceneObject& runtime) {
                return runtime.objectType == "toolAttachment";
            }),
        objects.end());

    for(RuntimeToolAttachmentVisual& attachment : toolAttachments) {
        attachment.collisionObjectIndex = std::numeric_limits<std::size_t>::max();
    }

    for(ProjectCollisionDetectorRuntime& detector : collisionDetectors) {
        detector.lastResult.clear();
        detector.hasResult = false;
        detector.effectiveIncludePairCount = 0;
    }

    invalidateCollisionGeometryOverlayCache();
}

std::vector<simulation_runtime::RuntimeAttachmentRobotContext> ProjectScene::Impl::attachmentRobotContexts() const
{
    std::vector<simulation_runtime::RuntimeAttachmentRobotContext> contexts;
    contexts.reserve(robots.size());
    for(const RuntimeRobot& runtime : robots) {
        simulation_runtime::RuntimeAttachmentRobotContext context;
        context.robotId = runtime.documentId;
        context.model = &runtime.model;
        context.instance = runtime.instance.get();
        contexts.push_back(context);
    }
    return contexts;
}

bool ProjectScene::Impl::rebuildMountedAttachmentGraph()
{
    const simulation_project::AssetResolveContext resolveContext =
        ProjectRuntimeBuilder::makeAssetResolveContext(projectBasePath, projectDocument.assetSearchPaths);
    const simulation_runtime::Result result =
        mountedAttachments.load(projectDocument, resolveContext, attachmentRobotContexts());
    if(!result.success) {
        std::cerr << "Failed to build mounted attachment runtime graph: "
            << result.message << "\n";
        mountedAttachments.clear();
        return false;
    }
    return true;
}

void ProjectScene::Impl::syncStewartFollowers(const RuntimeRobot& platform)
{
    if(!platform.parallelControlEnabled) {
        return;
    }

    for(RuntimeRobot& follower : robots) {
        if(!follower.parallelFollowerEnabled ||
            !follower.parallelFollowerAnchorsValid ||
            follower.documentId == platform.documentId ||
            !sameStewartSource(platform, follower)) {
            continue;
        }

        const std::size_t legIndex =
            static_cast<std::size_t>(follower.parallelFollowerLegIndex);
        if(follower.parallelFollowerActuatorDofIndex >= 0 && legIndex < 6) {
            const double targetLength = platform.parallelActuatorLengths[legIndex];
            const double travel =
                follower.parallelFollowerActuatorSign *
                (targetLength - follower.parallelFollowerHomeLength);
            follower.instance->setJoint(
                static_cast<std::size_t>(follower.parallelFollowerActuatorDofIndex),
                travel);
        }

        follower.baseTransform = follower.parallelFollowerHomeTransform;
    }
}

void ProjectScene::Impl::configureStewartGroups()
{
    for(RuntimeRobot& platform : robots) {
        if(!platform.parallelControlEnabled) {
            continue;
        }

        kine::StewartPlatformGeometry importedGeometry = platform.parallelGeometry;
        std::array<bool, 6> baseAnchorsFound{};
        std::array<bool, 6> platformAnchorsFound{};
        std::array<std::vector<collision::Vec3>, 6> baseAnchorCandidates;
        std::array<std::vector<collision::Vec3>, 6> baseFallbackAnchorCandidates;
        std::array<std::vector<collision::Vec3>, 6> platformAnchorCandidates;

        const collision::Transform3 platformHomeInverse =
            platform.parallelHomeBaseTransform.inverse();

        for(RuntimeRobot& candidate : robots) {
            if(!sameStewartSource(platform, candidate) ||
                !candidate.instance) {
                continue;
            }

            for(int legIndex = 0; legIndex < 6; ++legIndex) {
                const std::size_t index = static_cast<std::size_t>(legIndex);
                collectJointAnchorsForLeg(
                    candidate,
                    stewartLowerActuatorMarker(),
                    legIndex,
                    baseAnchorCandidates[index]);
                collectJointAnchorsForLeg(
                    candidate,
                    stewartHookeMarker(),
                    legIndex,
                    baseFallbackAnchorCandidates[index]);
                collectJointAnchorsForLeg(
                    candidate,
                    stewartUpperActuatorMarker(),
                    legIndex,
                    platformAnchorCandidates[index]);
            }
        }

        for(int legIndex = 0; legIndex < 6; ++legIndex) {
            const std::size_t index = static_cast<std::size_t>(legIndex);
            if(baseAnchorCandidates[index].empty()) {
                baseAnchorCandidates[index] = baseFallbackAnchorCandidates[index];
            }
            if(baseAnchorCandidates[index].empty()) {
                for(RuntimeRobot& candidate : robots) {
                    if(!sameStewartSource(platform, candidate) ||
                        !candidate.instance) {
                        continue;
                    }
                    collectLinkOriginAnchorForLeg(
                        candidate,
                        stewartLowerActuatorMarker(),
                        legIndex,
                        baseAnchorCandidates[index]);
                }
            }
            if(baseAnchorCandidates[index].empty()) {
                for(RuntimeRobot& candidate : robots) {
                    if(!sameStewartSource(platform, candidate) ||
                        !candidate.instance) {
                        continue;
                    }
                    collectLinkOriginAnchorForLeg(
                        candidate,
                        stewartHookeMarker(),
                        legIndex,
                        baseAnchorCandidates[index]);
                }
            }
            if(platformAnchorCandidates[index].empty()) {
                for(RuntimeRobot& candidate : robots) {
                    if(!sameStewartSource(platform, candidate) ||
                        !candidate.instance) {
                        continue;
                    }
                    collectLinkOriginAnchorForLeg(
                        candidate,
                        stewartUpperActuatorMarker(),
                        legIndex,
                        platformAnchorCandidates[index]);
                }
            }

            collision::Vec3 baseAnchorWorld = collision::Vec3::Zero();
            if(chooseLowestLocalZAnchor(
                   baseAnchorCandidates[index],
                   platformHomeInverse,
                   baseAnchorWorld)) {
                importedGeometry.baseAnchors[index] =
                    platformHomeInverse * baseAnchorWorld;
                baseAnchorsFound[index] = true;
            }

            collision::Vec3 platformAnchorWorld = collision::Vec3::Zero();
            if(baseAnchorsFound[index] &&
                chooseFarthestAnchor(
                    platformAnchorCandidates[index],
                    baseAnchorWorld,
                    platformAnchorWorld)) {
                importedGeometry.platformAnchors[index] =
                    platformHomeInverse * platformAnchorWorld;
                platformAnchorsFound[index] = true;
            }
        }

        for(RuntimeRobot& follower : robots) {
            if(!follower.parallelFollowerEnabled ||
                follower.documentId == platform.documentId ||
                !sameStewartSource(platform, follower) ||
                !follower.instance) {
                continue;
            }

            const std::string upperLink = findFirstLinkContaining(
                follower.model,
                stewartUpperActuatorMarker());
            if(upperLink.empty()) {
                continue;
            }

            const int legIndex = actuatorLegIndexFromLinkName(upperLink);
            if(legIndex < 0 || legIndex >= 6) {
                continue;
            }
            const std::size_t index = static_cast<std::size_t>(legIndex);
            if(!baseAnchorsFound[index] || !platformAnchorsFound[index]) {
                continue;
            }

            std::string baseLink = findFirstLinkForLeg(
                follower.model,
                stewartLowerActuatorMarker(),
                legIndex);
            if(baseLink.empty()) {
                baseLink = findFirstLinkForLeg(
                    follower.model,
                    stewartHookeMarker(),
                    legIndex);
            }

            follower.parallelFollowerLegIndex = legIndex;
            follower.parallelFollowerHomeTransform = follower.baseTransform;
            follower.parallelFollowerHomeBaseAnchor = importedGeometry.baseAnchors[index];
            follower.parallelFollowerHomePlatformAnchor = importedGeometry.platformAnchors[index];
            follower.parallelFollowerAnchorsValid = true;
            follower.parallelFollowerHomeLength =
                (follower.parallelFollowerHomePlatformAnchor -
                    follower.parallelFollowerHomeBaseAnchor).norm();
            follower.parallelFollowerActuatorDofIndex =
                findStewartFollowerActuatorDofIndex(follower, legIndex);
            follower.parallelFollowerActuatorSign =
                calibrateStewartFollowerActuatorSign(
                    follower,
                    follower.parallelFollowerActuatorDofIndex,
                    baseLink,
                    upperLink);
            captureStewartFollowerDrivenLinks(follower, legIndex);
            LOG_INFO("rs2026") << "Stewart follower configured: platform="
                << platform.documentId
                << ", follower=" << follower.documentId
                << ", leg=" << (legIndex + 1)
                << ", homeLength=" << follower.parallelFollowerHomeLength << " m"
                << ", actuatorDof=" << follower.parallelFollowerActuatorDofIndex
                << ", actuatorSign=" << follower.parallelFollowerActuatorSign
                << ", drivenLinks=" << follower.parallelFollowerDrivenLinks.size();
        }

        const bool hasAllAnchors =
            std::all_of(baseAnchorsFound.begin(), baseAnchorsFound.end(), [](bool found) {
                return found;
            }) &&
            std::all_of(platformAnchorsFound.begin(), platformAnchorsFound.end(), [](bool found) {
                return found;
            });
        if(!hasAllAnchors) {
            LOG_WARNING("rs2026") << "Stewart imported geometry incomplete: robot="
                << platform.documentId;
            continue;
        }

        platform.parallelGeometry = importedGeometry;
        std::string error;
        if(!kine::StewartPlatformKinematics::computeActuatorLengths(
               platform.parallelGeometry,
               platform.parallelPose,
               platform.parallelActuatorLengths,
               &error)) {
            LOG_WARNING("rs2026") << "Failed to initialize imported Stewart geometry: robot="
                << platform.documentId << ", error=" << error;
        } else {
            platform.parallelActuatorHomeLengths = platform.parallelActuatorLengths;
            configureStewartInternalActuatorDofs(platform);
            captureStewartInternalPlatformVisuals(platform);
            const auto lengthRange = std::minmax_element(
                platform.parallelActuatorLengths.begin(),
                platform.parallelActuatorLengths.end());
            LOG_INFO("rs2026") << "Stewart imported actuator home length range: robot="
                << platform.documentId
                << ", min=" << *lengthRange.first << " m"
                << ", max=" << *lengthRange.second << " m"
                << ", internalPlatformLinks="
                << platform.parallelInternalPlatformDrivenLinks.size();
        }
    }
}

void ProjectScene::Impl::syncAllStewartFollowers()
{
    for(const RuntimeRobot& platform : robots) {
        syncStewartFollowers(platform);
    }
}

void ProjectScene::Impl::setStewartPlatformBaseTransform(
    RuntimeRobot& platform,
    const collision::Transform3& baseTransform)
{
    if(!platform.parallelControlEnabled) {
        platform.baseTransform = baseTransform;
        ProjectRuntimeBuilder::updateRobotPose(platform);
        return;
    }

    platform.parallelHomeBaseTransform = baseTransform;
    platform.baseTransform = platform.parallelHomeBaseTransform;
    solveStewartInternalLegControls(platform);
    ProjectRuntimeBuilder::updateRobotPose(platform);
    applyStewartInternalPlatformVisualOverrides(platform);
    syncStewartFollowers(platform);
    for(RuntimeRobot& follower : robots) {
        if(follower.parallelFollowerEnabled && sameStewartSource(platform, follower)) {
            follower.parallelFollowerHomeTransform = baseTransform;
            follower.baseTransform = baseTransform;
            ProjectRuntimeBuilder::updateRobotPose(follower);
        }
    }
    applyStewartFollowerVisualOverrides(platform);
}

void ProjectScene::Impl::applyStewartInternalPlatformVisualOverrides(RuntimeRobot& platform)
{
    applyStewartInternalPlatformVisualOverride(platform);
}

void ProjectScene::Impl::applyStewartFollowerVisualOverrides(const RuntimeRobot& platform)
{
    if(!platform.parallelControlEnabled) {
        return;
    }

    for(RuntimeRobot& follower : robots) {
        if(!follower.parallelFollowerEnabled ||
            follower.documentId == platform.documentId ||
            !sameStewartSource(platform, follower)) {
            continue;
        }

        applyStewartFollowerVisualOverride(platform, follower);
    }
}

void ProjectScene::Impl::applyAllStewartFollowerVisualOverrides()
{
    for(RuntimeRobot& platform : robots) {
        applyStewartInternalPlatformVisualOverrides(platform);
        applyStewartFollowerVisualOverrides(platform);
    }
}

void ProjectScene::Impl::buildProjectRobots()
{
    const auto buildStart = std::chrono::steady_clock::now();
    const RobotCollisionLinkSelectionMap collisionLinkSelection =
        buildRobotCollisionLinkSelection(projectDocument);
    uint64_t runtimeId = 1;
    robots.reserve(projectDocument.robots.size());
    for(const auto& robotDesc : projectDocument.robots) {
        const auto robotStart = std::chrono::steady_clock::now();
        robots.emplace_back();
        RuntimeRobot& runtime = robots.back();
        runtime.runtimeId = runtimeId++;
        runtime.documentId = robotDesc.id;
        runtime.name = robotDesc.name;
        runtime.sourceType = robotDesc.sourceType;
        runtime.sourcePath = robotDesc.sourcePath;
        runtime.sourceModelIndex = robotDesc.sourceModelIndex;
        runtime.baseTransform = ProjectRuntimeBuilder::makeTransform(robotDesc.baseTransform);
        runtime.collisionEnabled = robotDesc.collisionEnabled;

        const std::filesystem::path robotPath = simulation_project::AssetResolver::resolveProjectPath(
            ProjectRuntimeBuilder::makeAssetResolveContext(projectBasePath, projectDocument.assetSearchPaths),
            robotDesc.sourcePath);
        runtime.model = ProjectRuntimeBuilder::loadSingleRobot(
            robotPath,
            robotDesc.sourceType,
            robotDesc.sourceModelIndex);
        configureParallelControlIfNeeded(runtime, robotDesc, runtime.model);
        configureParallelFollowerIfNeeded(runtime, robotDesc, runtime.model);
        RobotCollisionOverrideApplier::apply(projectDocument, robotDesc, projectBasePath, runtime.model);
        runtime.instance = std::make_shared<robotinstance::RobotInstance>(runtime.model, robotDesc.id);
        runtime.instance->setBaseTransform(runtime.baseTransform);
        ProjectRuntimeBuilder::applyInitialJoints(*runtime.instance, robotDesc.initialJoints);
        runtime.instance->update();
        runtime.visualBridge = std::make_shared<robot_render::RobotVisualBridge>(runtime.instance, sceneGraph);
        if(collisionRuntimeBuilt) {
            const auto selectionIt = collisionLinkSelection.find(runtime.documentId);
            buildRobotCollision(
                runtime,
                projectDocument,
                runtime.model,
                sceneGraph,
                showCollisionGeometry,
                selectionIt != collisionLinkSelection.end() ? &selectionIt->second : nullptr);
        }
        ProjectRuntimeBuilder::updateRobotPose(runtime);

        if(collisionRuntimeBuilt && runtime.collisionEnabled && runtime.collisionInstance && !runtime.collisionInstance->objects().empty()) {
            collisionScene.addRobot(runtime.collisionInstance);
        } else if(collisionRuntimeBuilt) {
            LOG_WARNING("rs2026") << "Robot not added to collision scene: robot=" << runtime.documentId
                << ", collisionEnabled=" << runtime.collisionEnabled
                << ", hasInstance=" << (runtime.collisionInstance != nullptr)
                << ", collisionObjects=" << (runtime.collisionInstance ? runtime.collisionInstance->objects().size() : 0);
        }

        if(!runtime.parallelFollowerEnabled) {
            robotLinks.push_back(ProjectScene::RobotLinkGroup{
                runtime.documentId,
                runtime.name,
                runtime.model.linkNames,
                runtime.parallelControlEnabled
                    ? parallelControlJointNames()
                    : jointNames(runtime.model),
                runtime.parallelControlEnabled
                    ? parallelControlJointInfos()
                    : movableJointInfos(runtime.model) });
        }
        LOG_DEBUG("rs2026") << "Project robot build: robot=" << runtime.documentId
            << ", links=" << runtime.model.linkNames.size()
            << ", elapsedMs=" << elapsedMilliseconds(robotStart);
    }
    configureStewartGroups();
    syncAllStewartFollowers();
    for(RuntimeRobot& runtime : robots) {
        if(runtime.parallelControlEnabled) {
            solveStewartInternalLegControls(runtime);
            ProjectRuntimeBuilder::updateRobotPose(runtime);
        } else if(runtime.parallelFollowerEnabled) {
            ProjectRuntimeBuilder::updateRobotPose(runtime);
        }
    }
    applyAllStewartFollowerVisualOverrides();
    LOG_DEBUG("rs2026") << "Project robots build: count=" << robots.size()
        << ", elapsedMs=" << elapsedMilliseconds(buildStart);
}

void ProjectScene::Impl::buildProjectObjects()
{
    const auto buildStart = std::chrono::steady_clock::now();
    uint64_t runtimeId = 100000;
    objects.reserve(projectDocument.objects.size());

    for(const auto& objectDesc : projectDocument.objects) {
        const auto objectStart = std::chrono::steady_clock::now();
        RuntimeSceneObject runtime = ProjectRuntimeBuilder::buildSceneObject(
            objectDesc,
            projectDocument.collision,
            runtimeId++,
            projectBasePath,
            projectDocument.assetSearchPaths,
            sceneGraph,
            collisionRuntimeBuilt);

        if(collisionRuntimeBuilt && runtime.collisionEnabled && !runtime.collisionObjects.empty()) {
            for(const RuntimeSceneCollisionObject& collisionObject : runtime.collisionObjects) {
                if(!collisionObject.collisionObject) {
                    continue;
                }

                EnvironmentCollisionObjectInfo info;
                info.info.object = collisionObject.collisionObject->id();
                info.info.robotInstance = -1;
                info.info.linkName = "object";
                info.info.elementName = collisionObject.collisionShape.label.empty()
                    ? runtime.documentId
                    : collisionObject.collisionShape.label;
                info.info.geometryRole = collisionObject.collisionShape.role;
                info.shape = collisionObject.collisionShape;
                collisionScene.addEnvironmentObject(collisionObject.collisionObject, info);
                LOG_DEBUG("rs2026") << "Scene object collision added: object=" << runtime.documentId
                    << ", collisionObject=" << collisionObject.collisionObject->id();
            }
        } else if(collisionRuntimeBuilt && runtime.collisionEnabled) {
            LOG_WARNING("rs2026") << "Scene object not added to collision scene: object=" << runtime.documentId
                << ", collisionEnabled=" << runtime.collisionEnabled
                << ", hasCollisionObject=" << (runtime.collisionObject != nullptr);
        } else if(collisionRuntimeBuilt) {
            LOG_DEBUG("rs2026") << "Scene object collision disabled: object=" << runtime.documentId;
        }

        sceneObjects.push_back(ProjectScene::SceneObjectGroup{
            runtime.documentId,
            runtime.name });

        LOG_DEBUG("rs2026") << "Project scene object build: object=" << runtime.documentId
            << ", collisionObjects=" << runtime.collisionObjects.size()
            << ", elapsedMs=" << elapsedMilliseconds(objectStart);
        objects.push_back(std::move(runtime));
    }
    LOG_DEBUG("rs2026") << "Project scene objects build: count=" << projectDocument.objects.size()
        << ", elapsedMs=" << elapsedMilliseconds(buildStart);
}

void ProjectScene::Impl::buildProjectPointClouds()
{
    const auto buildStart = std::chrono::steady_clock::now();
    uint64_t runtimeId = 400000;
    pointClouds.reserve(projectDocument.pointClouds.size());

    for(const simulation_project::PointCloudDesc& pointCloudDesc : projectDocument.pointClouds) {
        const auto pointCloudStart = std::chrono::steady_clock::now();
        RuntimeSceneObject runtime = ProjectRuntimeBuilder::buildPointCloud(
            pointCloudDesc,
            runtimeId++,
            projectBasePath,
            projectDocument.assetSearchPaths,
            sceneGraph);

        if(collisionRuntimeBuilt && runtime.collisionEnabled && !runtime.collisionObjects.empty()) {
            for(const RuntimeSceneCollisionObject& collisionObject : runtime.collisionObjects) {
                if(!collisionObject.collisionObject) {
                    continue;
                }

                EnvironmentCollisionObjectInfo info;
                info.info.object = collisionObject.collisionObject->id();
                info.info.robotInstance = -1;
                info.info.linkName = "pointCloud";
                info.info.elementName = runtime.documentId;
                info.info.geometryRole = collisionObject.collisionShape.role;
                info.shape = collisionObject.collisionShape;
                collisionScene.addEnvironmentObject(collisionObject.collisionObject, info);
            }
        } else if(collisionRuntimeBuilt && runtime.collisionEnabled) {
            LOG_WARNING("rs2026") << "Point cloud not added to collision scene: pointCloud="
                << runtime.documentId
                << ", collisionObjects=" << runtime.collisionObjects.size();
        }

        pointClouds.push_back(ProjectScene::PointCloudGroup{
            runtime.documentId,
            runtime.name });

        LOG_DEBUG("rs2026") << "Project point cloud build: pointCloud=" << runtime.documentId
            << ", collisionObjects=" << runtime.collisionObjects.size()
            << ", elapsedMs=" << elapsedMilliseconds(pointCloudStart);
        objects.push_back(std::move(runtime));
    }
    LOG_DEBUG("rs2026") << "Project point clouds build: count=" << projectDocument.pointClouds.size()
        << ", elapsedMs=" << elapsedMilliseconds(buildStart);
}

void ProjectScene::Impl::registerRuntimeCollisionObjects()
{
    collisionScene = CollisionScene();

    for(RuntimeRobot& runtime : robots) {
        if(runtime.collisionEnabled && runtime.collisionInstance && !runtime.collisionInstance->objects().empty()) {
            collisionScene.addRobot(runtime.collisionInstance);
        } else {
            LOG_WARNING("rs2026") << "Robot not added to collision scene: robot=" << runtime.documentId
                << ", collisionEnabled=" << runtime.collisionEnabled
                << ", hasInstance=" << (runtime.collisionInstance != nullptr)
                << ", collisionObjects=" << (runtime.collisionInstance ? runtime.collisionInstance->objects().size() : 0);
        }
    }

    for(RuntimeSceneObject& runtime : objects) {
        if(runtime.collisionEnabled && !runtime.collisionObjects.empty()) {
            for(const RuntimeSceneCollisionObject& collisionObject : runtime.collisionObjects) {
                if(!collisionObject.collisionObject) {
                    continue;
                }

                EnvironmentCollisionObjectInfo info;
                info.info.object = collisionObject.collisionObject->id();
                info.info.robotInstance = -1;
                info.info.linkName = "object";
                info.info.elementName = collisionObject.collisionShape.label.empty()
                    ? runtime.documentId
                    : collisionObject.collisionShape.label;
                info.info.geometryRole = collisionObject.collisionShape.role;
                info.shape = collisionObject.collisionShape;
                collisionScene.addEnvironmentObject(collisionObject.collisionObject, info);
                LOG_DEBUG("rs2026") << "Scene object collision added: object=" << runtime.documentId
                    << ", collisionObject=" << collisionObject.collisionObject->id();
            }
        } else if(runtime.collisionEnabled) {
            LOG_WARNING("rs2026") << "Scene object not added to collision scene: object=" << runtime.documentId
                << ", collisionEnabled=" << runtime.collisionEnabled
                << ", hasCollisionObject=" << (runtime.collisionObject != nullptr);
        } else {
            LOG_DEBUG("rs2026") << "Scene object collision disabled: object=" << runtime.documentId;
        }
    }

}

bool ProjectScene::Impl::ensureCollisionRuntimeBuilt()
{
    if(collisionRuntimeBuilt) {
        return true;
    }
    if(collisionRuntimeBuildInProgress) {
        return false;
    }

    const auto buildStart = std::chrono::steady_clock::now();
    collisionRuntimeBuildInProgress = true;

    const RobotCollisionLinkSelectionMap collisionLinkSelection =
        buildRobotCollisionLinkSelection(projectDocument);
    std::size_t robotCollisionCount = 0;
    for(RuntimeRobot& runtime : robots) {
        const auto selectionIt = collisionLinkSelection.find(runtime.documentId);
        buildRobotCollision(
            runtime,
            projectDocument,
            runtime.model,
            sceneGraph,
            showCollisionGeometry,
            selectionIt != collisionLinkSelection.end() ? &selectionIt->second : nullptr);
        if(runtime.collisionInstance) {
            robotCollisionCount += runtime.collisionInstance->objects().size();
        }
        ProjectRuntimeBuilder::updateRobotPose(runtime);
    }

    std::size_t objectCollisionCount = 0;
    for(RuntimeSceneObject& runtime : objects) {
        const simulation_project::SceneObjectDesc* objectDesc =
            findSceneObjectDesc(projectDocument, runtime.documentId);
        if(objectDesc == nullptr) {
            continue;
        }
        if(ProjectRuntimeBuilder::ensureSceneObjectCollisionObjects(
            runtime,
            *objectDesc,
            projectDocument.collision,
            projectBasePath,
            projectDocument.assetSearchPaths)) {
            objectCollisionCount += runtime.collisionObjects.size();
        }
    }

    ensureToolAttachmentCollisionObjects();
    registerRuntimeCollisionObjects();
    collisionDetectors = ProjectCollisionDetectorBuilder::build(projectDocument, robots, objects);
    collisionRuntimeBuilt = true;
    collisionRuntimeBuildInProgress = false;
    invalidateCollisionGeometryOverlayCache();

    LOG_DEBUG("rs2026") << "Project collision runtime lazy build: robots=" << robots.size()
        << ", robotObjects=" << robotCollisionCount
        << ", objects=" << objects.size()
        << ", objectCollisionObjects=" << objectCollisionCount
        << ", detectors=" << collisionDetectors.size()
        << ", elapsedMs=" << elapsedMilliseconds(buildStart);
    return true;
}

void ProjectScene::Impl::buildProjectToolAttachments()
{
    const simulation_project::ProjectV3View view =
        simulation_project::buildProjectV3View(projectDocument);

    std::size_t firstEnabled = static_cast<std::size_t>(-1);
    std::size_t firstVisible = static_cast<std::size_t>(-1);

    for(const simulation_runtime::RuntimeMountedAttachment& mountedAttachment : mountedAttachments.attachments()) {
        const auto attachmentStart = std::chrono::steady_clock::now();
        const simulation_project::AssetView* asset =
            findAsset(view.core.attachments, mountedAttachment.assetId);
        if(asset == nullptr) {
            continue;
        }

        RuntimeToolAttachmentVisual runtime;
        runtime.documentId = mountedAttachment.documentId;
        runtime.name = mountedAttachment.name;
        runtime.robotId = mountedAttachment.owner.robotId;
        runtime.linkName = mountedAttachment.owner.linkName;
        runtime.robotMountId = mountedAttachment.owner.robotMountId;
        runtime.toolAssetId = asset->id;
        runtime.assetKind = mountedAttachment.kind.empty() ? asset->assetKind : mountedAttachment.kind;
        runtime.assetType = mountedAttachment.type.empty() ? asset->assetType : mountedAttachment.type;
        runtime.visualScale = mountedAttachment.visual.visualScale;
        runtime.linkToMount = mountedAttachment.transform.linkToMount;
        runtime.mountToAssetMount = mountedAttachment.transform.mountToAttachmentMount;
        runtime.assetMountToVisual = mountedAttachment.visual.attachmentMountToVisual;
        runtime.assetMountToTcp = mountedAttachment.visual.attachmentMountToTcp;
        runtime.worldLink = mountedAttachment.transform.worldLink;
        runtime.worldRobotMount = mountedAttachment.transform.worldRobotMount;
        runtime.worldToolMount = mountedAttachment.transform.worldAttachmentMount;
        runtime.worldVisual = mountedAttachment.transform.worldVisual;
        runtime.worldTcp = mountedAttachment.transform.worldTcp;
        if(const simulation_project::FunctionalFrameView* frame = primaryFunctionalFrame(*asset)) {
            runtime.functionalFrameType = frame->frameType;
        }
        runtime.hasSensorIntrinsics = asset->hasSensorIntrinsics;
        runtime.sensorIntrinsics = asset->sensorIntrinsics;
        runtime.visible = mountedAttachment.visual.visible;
        runtime.enabled = mountedAttachment.collision.enabled;
        runtime.visualPath = mountedAttachment.visual.visualPath;

        std::shared_ptr<assetcore::ModelDesc> modelDesc;
        if(!runtime.visualPath.empty()) {
            modelDesc = assetcore::AssetManager::instance().loadModel(
                pathToUtf8(runtime.visualPath),
                static_cast<float>(runtime.visualScale));
        }

        robot_render::MountedAttachmentVisualDesc visualDesc;
        visualDesc.id = runtime.documentId;
        visualDesc.worldAttachmentMount = math::eigenToGlm(runtime.worldToolMount);
        visualDesc.attachmentMountToVisual = math::eigenToGlm(runtime.assetMountToVisual);
        runtime.visual = robot_render::MountedAttachmentVisualBridge::build(
            visualDesc,
            modelDesc.get(),
            sceneGraph);

        const std::size_t index = toolAttachments.size();
        if(runtime.visible && firstVisible == static_cast<std::size_t>(-1)) {
            firstVisible = index;
        }
        if(runtime.visible && runtime.enabled && firstEnabled == static_cast<std::size_t>(-1)) {
            firstEnabled = index;
        }

        std::cout << "QtViewer mounted attachment: " << runtime.documentId
            << " kind=" << runtime.assetKind
            << " type=" << runtime.assetType
            << " mount=" << runtime.robotMountId
            << " asset=" << runtime.toolAssetId
            << " link=" << runtime.linkName
            << " visualPath=" << pathToUtf8(runtime.visualPath)
            << " enabled=" << (runtime.enabled ? "true" : "false")
            << " visible=" << (runtime.visible ? "true" : "false")
            << "\n";

        toolAttachments.push_back(std::move(runtime));
        LOG_DEBUG("rs2026") << "Project mounted attachment build: attachment="
            << mountedAttachment.documentId
            << ", elapsedMs=" << elapsedMilliseconds(attachmentStart);
    }

    activeToolAttachmentIndex = firstEnabled != static_cast<std::size_t>(-1)
        ? firstEnabled
        : firstVisible;
    syncProjectToolAttachments();
    applyActiveToolAttachmentVisibility();

    if(!toolAttachments.empty() && activeToolAttachmentIndex < toolAttachments.size()) {
        std::cout << "QtViewer v3 tool attachments: " << toolAttachments.size()
            << ", active=" << toolAttachments[activeToolAttachmentIndex].documentId << "\n";
    } else if(!toolAttachments.empty()) {
        std::cout << "QtViewer v3 tool attachments: " << toolAttachments.size()
            << ", active=<none visible>\n";
    }
}

void ProjectScene::Impl::syncProjectToolAttachments()
{
    const simulation_runtime::Result result =
        mountedAttachments.updateWorldTransforms(attachmentRobotContexts());
    if(!result.success) {
        std::cerr << "Failed to update mounted attachment runtime graph: "
            << result.message << "\n";
        return;
    }

    for(RuntimeToolAttachmentVisual& attachment : toolAttachments) {
        const simulation_runtime::RuntimeMountedAttachment* mountedAttachment =
            mountedAttachments.attachment(attachment.documentId);
        if(mountedAttachment == nullptr) {
            continue;
        }

        attachment.robotId = mountedAttachment->owner.robotId;
        attachment.linkName = mountedAttachment->owner.linkName;
        attachment.robotMountId = mountedAttachment->owner.robotMountId;
        attachment.linkToMount = mountedAttachment->transform.linkToMount;
        attachment.mountToAssetMount = mountedAttachment->transform.mountToAttachmentMount;
        attachment.assetMountToVisual = mountedAttachment->visual.attachmentMountToVisual;
        attachment.assetMountToTcp = mountedAttachment->visual.attachmentMountToTcp;
        attachment.worldLink = mountedAttachment->transform.worldLink;
        attachment.worldRobotMount = mountedAttachment->transform.worldRobotMount;
        attachment.worldToolMount = mountedAttachment->transform.worldAttachmentMount;
        attachment.worldVisual = mountedAttachment->transform.worldVisual;
        attachment.worldTcp = mountedAttachment->transform.worldTcp;
        if(attachment.visual.rootNode) {
            robot_render::MountedAttachmentVisualBridge::syncTransform(
                attachment.visual,
                math::eigenToGlm(attachment.worldToolMount));
        }
        if(attachment.collisionObjectIndex < objects.size()) {
            RuntimeSceneObject& collisionObject = objects[attachment.collisionObjectIndex];
            collisionObject.transform = attachment.worldToolMount;
            collisionObject.collisionEnabled = attachment.visible && attachment.enabled;
            ProjectRuntimeBuilder::updateSceneObjectPose(collisionObject);
        }
    }
}

void ProjectScene::Impl::ensureToolAttachmentCollisionObjects()
{
    if(toolAttachments.empty()) {
        return;
    }

    const auto buildStart = std::chrono::steady_clock::now();
    std::size_t referencedCount = 0;
    std::size_t builtCount = 0;
    std::size_t skippedCount = 0;

    for(std::size_t index = 0; index < toolAttachments.size(); ++index) {
        RuntimeToolAttachmentVisual& runtime = toolAttachments[index];
        if(runtime.collisionObjectIndex < objects.size()) {
            continue;
        }

        if(!runtime.visible ||
            !runtime.enabled ||
            !simulation_runtime::projectReferencesAttachmentCollision(projectDocument, runtime.documentId)) {
            ++skippedCount;
            continue;
        }
        ++referencedCount;

        const auto attachmentStart = std::chrono::steady_clock::now();
        const uint64_t runtimeIdBase = 400000ull + static_cast<uint64_t>(index);
        RuntimeSceneObject toolCollisionObject;
        toolCollisionObject.runtimeId = runtimeIdBase;
        toolCollisionObject.documentId = runtime.documentId;
        toolCollisionObject.name = runtime.name.empty() ? runtime.documentId : runtime.name;
        toolCollisionObject.objectType = "toolAttachment";
        toolCollisionObject.transform = runtime.worldToolMount;
        toolCollisionObject.collisionEnabled = true;

        const simulation_project::ObjectCollisionOverrideDesc* collisionOverride =
            findToolAttachmentCollisionOverride(projectDocument, runtime.documentId);
        std::string activeModelId =
            activeObjectCollisionModelId(projectDocument.collision, runtime.documentId);
        if(simulation_project::isConvertFromVisualCollisionModelId(activeModelId) &&
            collisionOverride != nullptr &&
            collisionOverride->objectId != runtime.documentId) {
            activeModelId = activeObjectCollisionModelId(projectDocument.collision, collisionOverride->objectId);
        }
        const bool useOverrideCollision =
            collisionOverride != nullptr &&
            !simulation_project::isConvertFromVisualCollisionModelId(activeModelId);
        const bool hasOverrideCollision = useOverrideCollision &&
            ProjectRuntimeBuilder::appendObjectCollisionOverrideObjects(
                toolCollisionObject,
                *collisionOverride,
                runtimeIdBase,
                projectBasePath,
                projectDocument.assetSearchPaths,
                activeModelId);
        if(hasOverrideCollision) {
            toolCollisionObject.collisionObject = toolCollisionObject.collisionObjects.front().collisionObject;
            toolCollisionObject.collisionShape = toolCollisionObject.collisionObjects.front().collisionShape;
            ProjectRuntimeBuilder::updateSceneObjectPose(toolCollisionObject);

            runtime.collisionObjectIndex = objects.size();
            objects.push_back(std::move(toolCollisionObject));
            ++builtCount;

            LOG_DEBUG("rs2026") << "Tool attachment collision override lazy build: attachment="
                << runtime.documentId
                << ", overrideObject=" << collisionOverride->objectId
                << ", elements=" << objects[runtime.collisionObjectIndex].collisionObjects.size()
                << ", replaceOriginal=true"
                << ", elapsedMs=" << elapsedMilliseconds(attachmentStart);
            continue;
        }

        if(useOverrideCollision) {
            ++skippedCount;
            LOG_WARNING("rs2026") << "Tool attachment collision lazy build skipped: attachment="
                << runtime.documentId
                << ", overrideObject=" << collisionOverride->objectId
                << ", reason=collision override produced no geometry";
            continue;
        }

        if(runtime.visualPath.empty()) {
            ++skippedCount;
            continue;
        }

        std::shared_ptr<assetcore::ModelDesc> modelDesc =
            assetcore::AssetManager::instance().loadModel(
                pathToUtf8(runtime.visualPath),
                static_cast<float>(runtime.visualScale));
        if(!modelDesc) {
            ++skippedCount;
            LOG_WARNING("rs2026") << "Tool attachment collision lazy build skipped: attachment="
                << runtime.documentId
                << ", reason=model load failed";
            continue;
        }

        ObjMesh mesh = makeObjMesh(*modelDesc, 1.0);
        if(mesh.vertices.empty() || mesh.indices.empty()) {
            ++skippedCount;
            LOG_WARNING("rs2026") << "Tool attachment collision lazy build skipped: attachment="
                << runtime.documentId
                << ", reason=empty mesh";
            continue;
        }

        auto geometry = CollisionGeometryBuilder::buildTriangleMesh(makeTriangleMeshData(mesh));
        if(!geometry) {
            ++skippedCount;
            LOG_WARNING("rs2026") << "Tool attachment collision lazy build skipped: attachment="
                << runtime.documentId
                << ", reason=triangle mesh build failed";
            continue;
        }

        RuntimeSceneCollisionObject collisionRuntime;
        collisionRuntime.collisionShape.type = CollisionShapeType::TriangleMesh;
        collisionRuntime.collisionShape.label = "tool:" + runtime.documentId;
        collisionRuntime.collisionShape.role = CollisionGeometryRole::Exact;
        collisionRuntime.collisionShape.vertices = mesh.vertices;
        collisionRuntime.collisionShape.indices = mesh.indices;
        collisionRuntime.collisionShape.localTransform = runtime.assetMountToVisual;
        collisionRuntime.localTransform = runtime.assetMountToVisual;
        collisionRuntime.collisionObject =
            std::make_shared<CollisionObject>(runtimeIdBase, geometry);

        toolCollisionObject.collisionObject = collisionRuntime.collisionObject;
        toolCollisionObject.collisionShape = collisionRuntime.collisionShape;
        toolCollisionObject.collisionObjects.push_back(std::move(collisionRuntime));
        ProjectRuntimeBuilder::updateSceneObjectPose(toolCollisionObject);

        runtime.collisionObjectIndex = objects.size();
        objects.push_back(std::move(toolCollisionObject));
        ++builtCount;

        LOG_DEBUG("rs2026") << "Tool attachment collision lazy build: attachment="
            << runtime.documentId
            << ", vertices=" << mesh.vertices.size()
            << ", indices=" << mesh.indices.size()
            << ", elapsedMs=" << elapsedMilliseconds(attachmentStart);
    }

    LOG_DEBUG("rs2026") << "Tool attachment collision lazy build summary: attachments="
        << toolAttachments.size()
        << ", referenced=" << referencedCount
        << ", built=" << builtCount
        << ", skipped=" << skippedCount
        << ", elapsedMs=" << elapsedMilliseconds(buildStart);
}

void ProjectScene::Impl::drawPreviewRobotMountFrames()
{
    if(!previewSelectedLinkFrameVisible && !previewRobotMountFrameVisible) {
        return;
    }

    const simulation_project::RobotMountDesc* mount =
        findRobotMountDesc(projectDocument, activePreviewRobotMountId);
    const RuntimeRobot* robot = nullptr;
    std::string linkName;
    if(mount != nullptr) {
        robot = findRuntimeRobot(robots, mount->robotId);
        linkName = mount->linkName;
    } else if(previewSelectedLinkFrameVisible && mountFrameLinkFocusActive) {
        robot = findRuntimeRobot(robots, mountFrameFocusRobotId);
        linkName = mountFrameFocusLinkName;
    } else if(previewSelectedLinkFrameVisible) {
        robot = findRuntimeRobot(robots, selectionState.selectedRobotId());
        linkName = selectionState.selectedLinkName();
    }
    if(robot == nullptr ||
        !robot->instance ||
        linkName.empty() ||
        robot->model.links.find(linkName) == robot->model.links.end()) {
        return;
    }

    const collision::Transform3 worldLink = robot->instance->getLinkTransform(linkName);
    scenecore::DebugDraw& debug = renderer.debug();
    if(previewSelectedLinkFrameVisible) {
        drawFrameMarker(debug, worldLink, 0.10f, glm::vec4(1.0f, 1.0f, 1.0f, 1.0f), 3.5f);
    }
    if(previewRobotMountFrameVisible && mount != nullptr) {
        const collision::Transform3 worldMount =
            worldLink * ProjectRuntimeBuilder::makeTransform(mount->linkToMount);
        const glm::vec4 mountColor(0.1f, 0.35f, 1.0f, 1.0f);
        drawFrameMarker(debug, worldMount, 0.24f, mountColor, 1.35f);
        const collision::Vec3 linkOrigin = worldLink.translation();
        const collision::Vec3 mountOrigin = worldMount.translation();
        debug.drawLine(
            math::eigenToGlm(linkOrigin),
            math::eigenToGlm(mountOrigin),
            mountColor,
            scenecore::RenderTag{
                scenecore::RenderLayer::Gizmo,
                scenecore::RenderCategory::Debug,
                scenecore::RenderFeature::Gizmo });
    }
}

void ProjectScene::Impl::drawSelectedJointFrame()
{
    if(selectedJointFrameRobotId.empty() || selectedJointFrameName.empty()) {
        return;
    }

    const RuntimeRobot* robot = findRuntimeRobot(robots, selectedJointFrameRobotId);
    if(robot == nullptr || !robot->instance) {
        return;
    }

    const auto jointIt = robot->model.jointNameToIndex.find(selectedJointFrameName);
    if(jointIt == robot->model.jointNameToIndex.end() || jointIt->second < 0) {
        return;
    }

    const std::size_t jointIndex = static_cast<std::size_t>(jointIt->second);
    if(jointIndex >= robot->model.joints.size()) {
        return;
    }

    const robot::RobotJoint& joint = robot->model.joints[jointIndex];
    if(joint.parent.empty() || robot->model.links.find(joint.parent) == robot->model.links.end()) {
        return;
    }

    const collision::Transform3 worldParent = robot->instance->getLinkTransform(joint.parent);
    const collision::Transform3 worldJoint = worldParent * joint.T_parent_joint;
    const glm::vec4 jointColor(1.0f, 0.85f, 0.1f, 1.0f);
    scenecore::DebugDraw& debug = renderer.debug();
    drawFrameMarker(debug, worldJoint, 0.18f, jointColor, 1.2f);

    if(!joint.child.empty() && robot->model.links.find(joint.child) != robot->model.links.end()) {
        const collision::Transform3 worldChild = robot->instance->getLinkTransform(joint.child);
        debug.drawLine(
            toGlmVec3(worldJoint.translation()),
            toGlmVec3(worldChild.translation()),
            jointColor,
            scenecore::RenderTag{
                scenecore::RenderLayer::Gizmo,
                scenecore::RenderCategory::Debug,
                scenecore::RenderFeature::Gizmo });
    }
}

void ProjectScene::Impl::drawSelectedObjectFrame()
{
    if(!objectFrameObjectFocusActive) {
        return;
    }
    if(selectedObjectFrameObjectId.empty() || selectedObjectFrameId.empty()) {
        return;
    }
    if(selectedObjectFrameObjectId != objectFrameFocusObjectId) {
        return;
    }

    const RuntimeSceneObject* runtimeObject = findRuntimeObject(objects, selectedObjectFrameObjectId);
    const simulation_project::SceneObjectDesc* object =
        findSceneObjectDesc(projectDocument, selectedObjectFrameObjectId);
    if(runtimeObject == nullptr || object == nullptr) {
        return;
    }

    const simulation_project::ObjectFrameDesc* frame =
        findObjectFrameDesc(*object, selectedObjectFrameId);
    if(frame == nullptr) {
        return;
    }

    const collision::Transform3 worldFrame =
        runtimeObject->transform * ProjectRuntimeBuilder::makeTransform(frame->objectToFrame);
    scenecore::DebugDraw& debug = renderer.debug();
    const glm::vec4 objectColor(1.0f, 1.0f, 1.0f, 1.0f);
    const glm::vec4 frameColor(0.1f, 1.0f, 0.65f, 1.0f);
    drawFrameMarker(debug, runtimeObject->transform, 0.12f, objectColor, 2.4f);
    drawFrameMarker(debug, worldFrame, 0.24f, frameColor, 1.45f);
    debug.drawLine(
        toGlmVec3(runtimeObject->transform.translation()),
        toGlmVec3(worldFrame.translation()),
        frameColor,
        scenecore::RenderTag{
            scenecore::RenderLayer::Gizmo,
            scenecore::RenderCategory::Debug,
            scenecore::RenderFeature::Gizmo });
}

void ProjectScene::Impl::drawVisibleObjectFrames()
{
    if(objectFrameObjectFocusActive) {
        return;
    }

    scenecore::DebugDraw& debug = renderer.debug();
    const glm::vec4 frameColor(0.1f, 1.0f, 0.65f, 1.0f);
    for(const simulation_project::SceneObjectDesc& object : projectDocument.objects) {
        const RuntimeSceneObject* runtimeObject =
            object.visible ? findRuntimeObject(objects, object.id) : nullptr;
        const RuntimeToolAttachmentVisual* sourceAttachment =
            object.visible ? nullptr : mountedAttachmentForSourceObject(object.id);
        if(runtimeObject == nullptr && sourceAttachment == nullptr) {
            continue;
        }
        const collision::Transform3 objectWorld =
            runtimeObject != nullptr ? runtimeObject->transform : sourceAttachment->worldVisual;
        for(const simulation_project::ObjectFrameDesc& frame : object.objectFrames) {
            if(!frame.visible) {
                continue;
            }
            const collision::Transform3 worldFrame =
                objectWorld * ProjectRuntimeBuilder::makeTransform(frame.objectToFrame);
            drawFrameMarker(debug, worldFrame, 0.20f, frameColor, 1.1f);
        }
    }
}

const RuntimeToolAttachmentVisual* ProjectScene::Impl::mountedAttachmentForSourceObject(
    const std::string& objectId) const
{
    if(objectId.empty()) {
        return nullptr;
    }

    for(const simulation_project::MountedAttachmentDesc& attachmentDesc : projectDocument.mountedAttachments) {
        if(attachmentDesc.sourceObjectId != objectId || !attachmentDesc.visible) {
            continue;
        }
        for(const RuntimeToolAttachmentVisual& attachment : toolAttachments) {
            if(attachment.documentId == attachmentDesc.id && attachment.visible) {
                return &attachment;
            }
        }
    }
    return nullptr;
}

void ProjectScene::Impl::drawPinnedRobotMountFrames()
{
    if(pinnedRobotMountFrameIds.empty()) {
        return;
    }

    scenecore::DebugDraw& debug = renderer.debug();
    const glm::vec4 mountColor(0.1f, 0.35f, 1.0f, 1.0f);
    for(const std::string& mountId : pinnedRobotMountFrameIds) {
        if(mountId.empty() || mountId == activePreviewRobotMountId) {
            continue;
        }

        const simulation_project::RobotMountDesc* mount =
            findRobotMountDesc(projectDocument, mountId);
        if(mount == nullptr) {
            continue;
        }

        const RuntimeRobot* robot = findRuntimeRobot(robots, mount->robotId);
        if(robot == nullptr ||
            !robot->instance ||
            robot->model.links.find(mount->linkName) == robot->model.links.end()) {
            continue;
        }

        const collision::Transform3 worldLink = robot->instance->getLinkTransform(mount->linkName);
        const collision::Transform3 worldMount =
            worldLink * ProjectRuntimeBuilder::makeTransform(mount->linkToMount);
        drawFrameMarker(debug, worldMount, 0.18f, mountColor, 1.05f);
    }
}

void ProjectScene::Impl::drawActiveToolAttachmentFrames()
{
    if(mountFrameLinkFocusActive || objectFrameObjectFocusActive) {
        return;
    }

    const RuntimeToolAttachmentVisual* attachment = activeToolFrameAttachment();
    if(attachment == nullptr || !attachment->visible) {
        return;
    }

    scenecore::DebugDraw& debug = renderer.debug();
    if(toolFrameVisibility.link) {
        drawFrameMarker(debug, attachment->worldLink, 0.20f, glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
    }
    if(toolFrameVisibility.robotMount) {
        drawFrameMarker(debug, attachment->worldRobotMount, 0.18f, glm::vec4(0.1f, 0.35f, 1.0f, 1.0f));
    }
    if(toolFrameVisibility.toolMount) {
        drawFrameMarker(debug, attachment->worldToolMount, 0.16f, glm::vec4(0.1f, 1.0f, 0.25f, 1.0f));
    }
    if(toolFrameVisibility.visual) {
        drawFrameMarker(debug, attachment->worldVisual, 0.13f, glm::vec4(1.0f, 0.8f, 0.1f, 1.0f));
    }
    if(toolFrameVisibility.tcp) {
        drawFrameMarker(debug, attachment->worldTcp, 0.14f, glm::vec4(1.0f, 0.1f, 0.1f, 1.0f));
    }
    if(toolFrameVisibility.sensorPreview && isSensorAttachment(*attachment)) {
        drawFrameMarker(debug, attachment->worldTcp, 0.16f, glm::vec4(0.1f, 0.85f, 1.0f, 1.0f));
        if(attachment->hasSensorIntrinsics) {
            drawSensorFovPreview(debug, attachment->worldTcp, attachment->sensorIntrinsics);
        }
        drawSensorDirection(debug, attachment->worldTcp, 0.45, glm::vec4(0.1f, 1.0f, 0.75f, 1.0f));
    }
}

void ProjectScene::Impl::hideObjectCollisionModelVariantPreviewMeshes()
{
    for(auto& entry : objectCollisionVariantMeshPreviews) {
        for(ObjectCollisionVariantMeshPreview& preview : entry.second.previews) {
            if(preview.node) {
                preview.node->setVisible(false);
            }
        }
    }
}

void ProjectScene::Impl::updateObjectCollisionModelVariantPreviewMeshes()
{
    hideObjectCollisionModelVariantPreviewMeshes();
    if(!previewObjectCollisionModelActive || previewObjectCollisionModelObjectId.empty()) {
        return;
    }

    collision::Transform3 baseTransform = collision::Transform3::Identity();
    bool hasBaseTransform = false;
    const RuntimeToolAttachmentVisual* targetAttachment = nullptr;
    for(const RuntimeToolAttachmentVisual& attachment : toolAttachments) {
        if(attachment.documentId == previewObjectCollisionModelObjectId) {
            targetAttachment = &attachment;
            baseTransform = attachment.worldToolMount;
            hasBaseTransform = true;
            break;
        }
    }
    if(!hasBaseTransform) {
        if(const RuntimeSceneObject* object =
               findRuntimeObject(objects, previewObjectCollisionModelObjectId)) {
            baseTransform = object->transform;
            hasBaseTransform = true;
        }
    }
    if(!hasBaseTransform) {
        return;
    }

    const std::string previewKey = previewObjectCollisionModelObjectId + "|" +
        previewObjectCollisionModelVariantId;
    ObjectCollisionVariantMeshPreviewCache& cache =
        objectCollisionVariantMeshPreviews[previewKey];
    if(cache.revision != objectCollisionVariantMeshPreviewRevision) {
        std::size_t previewIndex = 0;
        for(ObjectCollisionVariantMeshPreview& preview : cache.previews) {
            preview.configured = false;
        }
        const auto addPreviewModel = [&](const std::filesystem::path& path,
                                         double uniformScale,
                                         const glm::mat4& targetLocal) {
            if(path.empty()) {
                return;
            }
            std::shared_ptr<assetcore::ModelDesc> modelDesc =
                assetcore::AssetManager::instance().loadModel(
                    pathToUtf8(path),
                    static_cast<float>(uniformScale));
            if(!modelDesc) {
                return;
            }
            std::shared_ptr<rendercore::Model> renderModel =
                rendercore::ModelManager::instance().buildModelFromDesc(*modelDesc);
            if(!renderModel) {
                return;
            }

            std::shared_ptr<rendercore::Material> material = makeOverlayMaterial(false);
            material->baseColor = Eigen::Vector4f(
                kCollisionPreviewRed,
                kCollisionPreviewGreen,
                kCollisionPreviewBlue,
                kCollisionPreviewVisibleAlpha);
            material->emissiveColor = Eigen::Vector3f(0.0f, 0.06f, 0.10f);
            applyOverlayMaterial(renderModel, material);

            if(previewIndex == cache.previews.size()) {
                ObjectCollisionVariantMeshPreview preview;
                preview.node = std::make_shared<VisibleModelNode>(renderModel);
                preview.node->setName(
                    previewKey + "_mesh_" + std::to_string(previewIndex + 1));
                sceneGraph.root()->addChild(preview.node);
                sceneGraph.registerNode(preview.node);
                cache.previews.push_back(std::move(preview));
            }

            ObjectCollisionVariantMeshPreview& preview = cache.previews[previewIndex++];
            preview.node->setModel(renderModel);
            preview.node->setVisible(false);
            preview.targetLocal = targetLocal * modelDesc->get_local();
            preview.configured = true;
        };

        if(simulation_project::isConvertFromVisualCollisionModelId(
               previewObjectCollisionModelVariantId)) {
            if(targetAttachment != nullptr) {
                addPreviewModel(
                    targetAttachment->visualPath,
                    targetAttachment->visualScale,
                    math::eigenToGlm(targetAttachment->assetMountToVisual));
            } else if(const simulation_project::SceneObjectDesc* objectDesc =
                          findSceneObjectDesc(projectDocument, previewObjectCollisionModelObjectId)) {
                const simulation_project::AssetResolveContext context =
                    ProjectRuntimeBuilder::makeAssetResolveContext(
                        projectBasePath,
                        projectDocument.assetSearchPaths);
                addPreviewModel(
                    simulation_project::AssetResolver::resolveProjectPath(
                        context,
                        objectDesc->sourcePath),
                    objectDesc->visualScale,
                    glm::mat4(1.0f));
            }
        } else {
            const simulation_project::ObjectCollisionOverrideDesc* collisionOverride =
                ProjectRuntimeBuilder::findObjectCollisionOverride(
                    projectDocument.collision,
                    previewObjectCollisionModelObjectId);
            if(collisionOverride == nullptr) {
                collisionOverride = findToolAttachmentCollisionOverride(
                    projectDocument,
                    previewObjectCollisionModelObjectId);
            }

            if(collisionOverride != nullptr) {
                const simulation_project::AssetResolveContext context =
                    ProjectRuntimeBuilder::makeAssetResolveContext(
                        projectBasePath,
                        projectDocument.assetSearchPaths);
                for(const simulation_project::ObjectCollisionElementOverrideDesc& element :
                    collisionOverride->elements) {
                    if(!element.enabled) {
                        continue;
                    }
                    if(!simulation_project::objectCollisionElementMatchesModelId(
                           previewObjectCollisionModelVariantId,
                           element.source,
                           element.role) ||
                        (element.type != "mesh" && element.type != "Mesh")) {
                        continue;
                    }

                    const glm::mat4 elementTransform = math::eigenToGlm(
                        ProjectRuntimeBuilder::makeTransform(element.localTransform));
                    const glm::mat4 elementScale = glm::scale(
                        glm::mat4(1.0f),
                        glm::vec3(
                            static_cast<float>(element.meshScale.x),
                            static_cast<float>(element.meshScale.y),
                            static_cast<float>(element.meshScale.z)));
                    addPreviewModel(
                        simulation_project::AssetResolver::resolveProjectPath(
                            context,
                            element.meshPath),
                        1.0,
                        elementTransform * elementScale);
                }
            }
        }

        cache.revision = objectCollisionVariantMeshPreviewRevision;
    }

    const glm::mat4 worldTarget = math::eigenToGlm(baseTransform);
    for(ObjectCollisionVariantMeshPreview& preview : cache.previews) {
        if(preview.node && preview.configured) {
            preview.node->setLocal(worldTarget * preview.targetLocal);
            preview.node->setVisible(true);
        }
    }
}

void ProjectScene::Impl::drawObjectCollisionModelVariantPreview()
{
    if(!previewObjectCollisionModelActive ||
        previewObjectCollisionModelObjectId.empty() ||
        simulation_project::isConvertFromVisualCollisionModelId(previewObjectCollisionModelVariantId)) {
        return;
    }

    const simulation_project::ObjectCollisionOverrideDesc* collisionOverride =
        ProjectRuntimeBuilder::findObjectCollisionOverride(
            projectDocument.collision,
            previewObjectCollisionModelObjectId);
    collision::Transform3 baseTransform = collision::Transform3::Identity();
    bool hasBaseTransform = false;

    if(collisionOverride == nullptr) {
        collisionOverride = findToolAttachmentCollisionOverride(
            projectDocument,
            previewObjectCollisionModelObjectId);
    }

    if(const RuntimeSceneObject* object =
           findRuntimeObject(objects, previewObjectCollisionModelObjectId)) {
        baseTransform = object->transform;
        hasBaseTransform = true;
    } else {
        for(const RuntimeToolAttachmentVisual& attachment : toolAttachments) {
            if(attachment.documentId == previewObjectCollisionModelObjectId) {
                baseTransform = attachment.worldToolMount;
                hasBaseTransform = true;
                break;
            }
        }
    }

    if(collisionOverride == nullptr || !hasBaseTransform) {
        return;
    }

    RuntimeSceneObject previewObject;
    previewObject.runtimeId = 900000ull;
    previewObject.documentId = previewObjectCollisionModelObjectId;
    previewObject.name = previewObjectCollisionModelObjectId;
    previewObject.transform = baseTransform;
    if(!ProjectRuntimeBuilder::appendObjectCollisionOverrideObjects(
           previewObject,
           *collisionOverride,
           previewObject.runtimeId,
           projectBasePath,
           projectDocument.assetSearchPaths,
           previewObjectCollisionModelVariantId)) {
        return;
    }

    scenecore::DebugDraw& debug = renderer.debug();
    const glm::vec4 previewColor(
        kCollisionPreviewRed,
        kCollisionPreviewGreen,
        kCollisionPreviewBlue,
        kCollisionPreviewAlpha);
    const scenecore::RenderTag previewTag{
        scenecore::RenderLayer::Gizmo,
        scenecore::RenderCategory::Debug,
        scenecore::RenderFeature::Gizmo };
    for(const RuntimeSceneCollisionObject& collisionObject : previewObject.collisionObjects) {
        const glm::mat4 transform = math::eigenToGlm(
            baseTransform * collisionObject.localTransform);
        switch(collisionObject.collisionShape.type) {
        case CollisionShapeType::Box:
            debug.drawCube(
                transform,
                math::eigenToGlm(collisionObject.collisionShape.boxSize),
                previewColor,
                scenecore::DrawType::UsingUnlitShader,
                previewTag);
            break;
        case CollisionShapeType::Sphere:
            debug.drawSphere(
                transform,
                static_cast<float>(collisionObject.collisionShape.radius),
                previewColor,
                scenecore::DrawType::UsingUnlitShader,
                previewTag);
            break;
        case CollisionShapeType::Cylinder:
            debug.drawCylinder(
                transform,
                static_cast<float>(collisionObject.collisionShape.radius),
                static_cast<float>(collisionObject.collisionShape.length),
                previewColor,
                scenecore::DrawType::UsingUnlitShader,
                previewTag);
            break;
        case CollisionShapeType::Capsule:
            debug.drawCapsule(
                transform,
                static_cast<float>(collisionObject.collisionShape.radius),
                static_cast<float>(collisionObject.collisionShape.length * 0.5),
                previewColor,
                scenecore::DrawType::UsingUnlitShader,
                previewTag);
            break;
        default:
            break;
        }
    }
}

void ProjectScene::Impl::drawInteractionModeHints()
{
    if(!modeShowsRobotMountHints(interactionMode) && !modeShowsAttachmentHints(interactionMode)) {
        return;
    }

    scenecore::DebugDraw& debug = renderer.debug();
    if(modeShowsRobotMountHints(interactionMode) && !previewRobotMountFrameVisible) {
        const glm::vec4 mountColor(0.1f, 0.55f, 1.0f, 1.0f);
        const glm::vec4 activeMountColor(0.1f, 1.0f, 0.35f, 1.0f);
        for(const simulation_project::RobotMountDesc& mount : projectDocument.robotMounts) {
            const bool active = mount.id == activePreviewRobotMountId;
            if(!active) {
                continue;
            }

            const RuntimeRobot* robot = findRuntimeRobot(robots, mount.robotId);
            if(robot == nullptr ||
                !robot->instance ||
                robot->model.links.find(mount.linkName) == robot->model.links.end()) {
                continue;
            }

            const collision::Transform3 worldLink = robot->instance->getLinkTransform(mount.linkName);
            const collision::Transform3 worldMount =
                worldLink * ProjectRuntimeBuilder::makeTransform(mount.linkToMount);
            drawFrameMarker(
                debug,
                worldMount,
                active ? 0.20f : 0.13f,
                active ? activeMountColor : mountColor,
                active ? 1.0f : 0.55f);
            debug.drawLine(
                toGlmVec3(worldLink.translation()),
                toGlmVec3(worldMount.translation()),
                active ? activeMountColor : mountColor,
                scenecore::RenderTag{
                    scenecore::RenderLayer::Gizmo,
                    scenecore::RenderCategory::Debug,
                    scenecore::RenderFeature::Gizmo });
        }
    }

    if(modeShowsAttachmentHints(interactionMode)) {
        const glm::vec4 attachmentColor(0.1f, 1.0f, 0.35f, 1.0f);
        const std::string activeAttachmentId =
            activeToolAttachmentIndex < toolAttachments.size()
                ? toolAttachments[activeToolAttachmentIndex].documentId
                : std::string();
        for(const RuntimeToolAttachmentVisual& attachment : toolAttachments) {
            if(!attachment.visible || !attachment.enabled) {
                continue;
            }

            const bool active = attachment.documentId == activeAttachmentId;
            drawFrameMarker(
                debug,
                attachment.worldToolMount,
                active ? 0.18f : 0.12f,
                attachmentColor,
                active ? 1.0f : 0.55f);
        }
    }
}

void ProjectScene::Impl::buildToolAssetPreview()
{
    toolAssetPreview.mountToVisual = ProjectRuntimeBuilder::makeTransform(toolAssetPreview.asset.assetMountToVisual);
    toolAssetPreview.mountToTcp = ProjectRuntimeBuilder::makeTransform(primaryAssetFrameTransform(toolAssetPreview.asset));

    auto rootNode = std::make_shared<scenecore::SceneNode>();
    rootNode->setName(toolAssetPreview.asset.id.empty() ? "tool_asset_preview" : toolAssetPreview.asset.id);
    rootNode->setVisible(true);

    if(!toolAssetPreview.asset.visualPath.empty()) {
        const simulation_project::AssetResolveContext resolveContext =
            ProjectRuntimeBuilder::makeAssetResolveContext(toolAssetPreview.basePath, {});
        toolAssetPreview.visualPath = simulation_project::AssetResolver::resolveProjectPath(
            resolveContext,
            toolAssetPreview.asset.visualPath);

        auto modelDesc = assetcore::AssetManager::instance().loadModel(
            pathToUtf8(toolAssetPreview.visualPath),
            static_cast<float>(toolAssetPreview.asset.visualScale));
        if(modelDesc) {
            auto renderModel = rendercore::ModelManager::instance().buildModelFromDesc(*modelDesc);
            if(renderModel) {
                ensureModelMaterial(renderModel, makeToolMaterial());
                auto modelNode = std::make_shared<scenecore::ModelNode>(renderModel);
                modelNode->setName(rootNode->name() + "_model");
                toolAssetPreview.modelBaseLocal = modelDesc->get_local();
                toolAssetPreview.modelNode = modelNode;
                rootNode->addChild(modelNode);
            }
        } else {
            LOG_WARNING("rs2026") << "Tool asset preview model not loaded: "
                << pathToUtf8(toolAssetPreview.visualPath);
        }
    }

    sceneGraph.root()->addChild(rootNode);
    sceneGraph.registerNode(rootNode);
    toolAssetPreview.rootNode = rootNode;
    updateToolAssetPreviewModel();
}

void ProjectScene::Impl::updateToolAssetPreviewModel()
{
    if(!toolAssetPreviewSet) {
        return;
    }

    toolAssetPreview.modelLocal =
        math::eigenToGlm(toolAssetPreview.mountToVisual) *
        toolAssetPreview.modelBaseLocal;
    if(toolAssetPreview.modelNode) {
        toolAssetPreview.modelNode->setLocal(toolAssetPreview.modelLocal);
    }
}

void ProjectScene::Impl::drawToolAssetPreviewFrames()
{
    if(!toolAssetPreviewSet) {
        return;
    }

    scenecore::DebugDraw& debug = renderer.debug();
    const collision::Transform3 worldFrame = collision::Transform3::Identity();
    const collision::Transform3 flangeFrame = collision::Transform3::Identity();
    const collision::Transform3 tcpFrame = toolAssetPreview.mountToTcp;
    drawFrameMarker(debug, worldFrame, 0.30f, glm::vec4(0.92f, 0.95f, 0.98f, 1.0f), 1.3f);
    drawFrameMarker(debug, flangeFrame, 0.20f, glm::vec4(0.1f, 0.35f, 1.0f, 1.0f), 0.75f);
    drawFrameMarker(debug, tcpFrame, 0.16f, glm::vec4(1.0f, 0.1f, 0.1f, 1.0f), 1.0f);

    const collision::Vec3 flangeOrigin = flangeFrame.translation();
    const collision::Vec3 tcpOrigin = tcpFrame.translation();
    debug.drawLine(
        toGlmVec3(flangeOrigin),
        toGlmVec3(tcpOrigin),
        glm::vec4(1.0f, 0.1f, 0.1f, 1.0f),
        scenecore::RenderTag{
            scenecore::RenderLayer::Gizmo,
            scenecore::RenderCategory::Debug,
            scenecore::RenderFeature::Gizmo });
}

void ProjectScene::Impl::applyActiveToolAttachmentVisibility()
{
    for(std::size_t i = 0; i < toolAttachments.size(); ++i) {
        RuntimeToolAttachmentVisual& attachment = toolAttachments[i];
        robot_render::MountedAttachmentVisualBridge::setVisible(
            attachment.visual,
            attachment.visible && !mountFrameLinkFocusActive && !objectFrameObjectFocusActive);
    }
}

const RuntimeToolAttachmentVisual* ProjectScene::Impl::activeToolFrameAttachment() const
{
    if(activeToolFrameRobotId.empty()) {
        return nullptr;
    }

    if(activeToolAttachmentIndex >= toolAttachments.size()) {
        return nullptr;
    }

    const RuntimeToolAttachmentVisual& attachment = toolAttachments[activeToolAttachmentIndex];
    if(attachment.robotId != activeToolFrameRobotId || !attachment.visible) {
        return nullptr;
    }
    return &attachment;
}

bool ProjectScene::Impl::cycleActiveToolAttachment(bool reverse)
{
    if(toolAttachments.empty()) {
        return false;
    }

    const std::size_t count = toolAttachments.size();
    std::size_t index = activeToolAttachmentIndex < count ? activeToolAttachmentIndex : 0;

    for(std::size_t step = 0; step < count; ++step) {
        index = reverse
            ? (index == 0 ? count - 1 : index - 1)
            : (index + 1) % count;

        if(toolAttachments[index].visible) {
            activeToolAttachmentIndex = index;
            applyActiveToolAttachmentVisibility();
            std::cout << "QtViewer active tool attachment: "
                << toolAttachments[index].documentId
                << " visualPath=" << pathToUtf8(toolAttachments[index].visualPath)
                << "\n";
            return true;
        }
    }

    return false;
}

bool ProjectScene::Impl::setActiveToolAttachment(const std::string& id)
{
    if(id.empty()) {
        const bool changed = activeToolAttachmentIndex < toolAttachments.size();
        activeToolAttachmentIndex = static_cast<std::size_t>(-1);
        if(changed) {
            applyActiveToolAttachmentVisibility();
        }
        return changed;
    }

    for(std::size_t i = 0; i < toolAttachments.size(); ++i) {
        if(toolAttachments[i].documentId == id && toolAttachments[i].visible) {
            activeToolAttachmentIndex = i;
            applyActiveToolAttachmentVisibility();
            std::cout << "QtViewer active tool attachment: "
                << toolAttachments[i].documentId
                << " visualPath=" << pathToUtf8(toolAttachments[i].visualPath)
                << "\n";
            return true;
        }
    }
    return false;
}

void ProjectScene::Impl::setActiveToolFrameRobot(const std::string& robotId)
{
    activeToolFrameRobotId = robotId;
}

void ProjectScene::Impl::applyProjectCollisionAndView()
{
    const auto buildStart = std::chrono::steady_clock::now();
    collisionDetectors = ProjectCollisionDetectorBuilder::build(projectDocument, robots, objects);
    if(activeCollisionDetectorId.empty() ||
        std::none_of(
            collisionDetectors.begin(),
            collisionDetectors.end(),
            [&](const ProjectCollisionDetectorRuntime& detector) {
                return detector.id == activeCollisionDetectorId;
            })) {
        activeCollisionDetectorId = collisionDetectors.empty() ? std::string() : collisionDetectors.front().id;
    }

    showCollisionGeometry = false;
    collisionQueriesEnabled = false;
    applyCollisionOverlayRenderConfig();
    invalidateCollisionGeometryOverlayCache();
    backgroundColor = makeEffectiveBackgroundColor(projectDocument.view, defaultBackgroundColor);
    const ProjectCollisionDetectorRuntime* activeDetector = activeCollisionDetector();
    LOG_DEBUG("rs2026") << "Project collision detectors: robots=" << robots.size()
        << ", objects=" << objects.size()
        << ", detectors=" << collisionDetectors.size()
        << ", active=" << (activeDetector != nullptr ? activeDetector->id : std::string())
        << ", includePairs=" << (activeDetector != nullptr ? activeDetector->options.includePairs.size() : 0)
        << ", elapsedMs=" << elapsedMilliseconds(buildStart);
}

ProjectCollisionDetectorRuntime* ProjectScene::Impl::activeCollisionDetector()
{
    for(ProjectCollisionDetectorRuntime& detector : collisionDetectors) {
        if(detector.id == activeCollisionDetectorId) {
            return &detector;
        }
    }
    return collisionDetectors.empty() ? nullptr : &collisionDetectors.front();
}

const ProjectCollisionDetectorRuntime* ProjectScene::Impl::activeCollisionDetector() const
{
    for(const ProjectCollisionDetectorRuntime& detector : collisionDetectors) {
        if(detector.id == activeCollisionDetectorId) {
            return &detector;
        }
    }
    return collisionDetectors.empty() ? nullptr : &collisionDetectors.front();
}

ProjectScene::ProjectScene()
    : m_impl(std::make_unique<Impl>())
{
}

ProjectScene::~ProjectScene() = default;

void ProjectScene::setProjectDocument(
    const simulation_project::ProjectDocument& document,
    const std::filesystem::path& basePath)
{
    m_impl->hideObjectCollisionModelVariantPreviewMeshes();
    ++m_impl->objectCollisionVariantMeshPreviewRevision;
    m_impl->projectDocument = document;
    m_impl->projectDocumentSet = true;
    m_impl->projectBasePath = basePath;
    m_impl->toolAssetPreviewSet = false;
    m_impl->visibleCollisionVariantIds.clear();
    m_impl->visibleCollisionVariantFilters.clear();
    m_impl->visibleCollisionVariantFiltersByRuntimeLink.clear();
    m_impl->collisionQueriesEnabled = false;
    m_impl->showCollisionGeometry = false;
    m_impl->collisionRuntimeBuilt = false;
    m_impl->collisionRuntimeBuildInProgress = false;
    m_impl->collisionScene = CollisionScene();
    m_impl->invalidateCollisionGeometryOverlayCache();
}

void ProjectScene::setDefaultBackgroundColor(const simulation_project::ColorDesc& color)
{
    m_impl->defaultBackgroundColor = color;
    if(!m_impl->projectDocumentSet || m_impl->projectDocument.view.useThemeBackground) {
        m_impl->backgroundColor = makeBackgroundColor(color);
    }
}

bool ProjectScene::refreshCollisionConfiguration(
    const simulation_project::ProjectDocument& document,
    const std::filesystem::path& basePath)
{
    if(!m_impl->initialized) {
        return false;
    }

    m_impl->hideObjectCollisionModelVariantPreviewMeshes();
    ++m_impl->objectCollisionVariantMeshPreviewRevision;
    m_impl->projectDocument = document;
    m_impl->projectDocumentSet = true;
    m_impl->projectBasePath = basePath;
    m_impl->toolAssetPreviewSet = false;
    m_impl->invalidateCollisionRuntime();
    m_impl->collisionDetectors = ProjectCollisionDetectorBuilder::build(
        m_impl->projectDocument,
        m_impl->robots,
        m_impl->objects);
    if(m_impl->activeCollisionDetectorId.empty() ||
        std::none_of(
            m_impl->collisionDetectors.begin(),
            m_impl->collisionDetectors.end(),
            [&](const ProjectCollisionDetectorRuntime& detector) {
                return detector.id == m_impl->activeCollisionDetectorId;
            })) {
        m_impl->activeCollisionDetectorId = m_impl->collisionDetectors.empty()
            ? std::string()
            : m_impl->collisionDetectors.front().id;
    }
    m_impl->backgroundColor = makeEffectiveBackgroundColor(
        m_impl->projectDocument.view,
        m_impl->defaultBackgroundColor);
    m_impl->applyCollisionOverlayRenderConfig();
    m_impl->applyMountFrameLinkFocusVisibility();
    m_impl->updateObjectCollisionModelVariantPreviewMeshes();
    return true;
}

bool ProjectScene::setToolAssetPreview(
    const simulation_project::AttachmentAssetDesc& asset,
    const std::filesystem::path& basePath)
{
    m_impl->toolAssetPreview = RuntimeToolAssetPreview();
    m_impl->toolAssetPreview.asset = asset;
    m_impl->toolAssetPreview.basePath = basePath;
    m_impl->toolAssetPreviewSet = true;
    m_impl->projectDocumentSet = true;
    m_impl->projectDocument = simulation_project::ProjectDocument();
    m_impl->projectBasePath = basePath;
    if(m_impl->initialized) {
        return false;
    }
    return true;
}

bool ProjectScene::initialize()
{
    const auto initializeStart = std::chrono::steady_clock::now();
    if(m_impl->initialized) {
        return true;
    }

    const auto openGlStart = std::chrono::steady_clock::now();
    if(!m_impl->initializeOpenGlRuntime()) {
        return false;
    }
    const double openGlMs = elapsedMilliseconds(openGlStart);
    LOG_DEBUG("rs2026") << "ProjectScene initialize OpenGL runtime: elapsedMs="
        << openGlMs;

    if(m_impl->toolAssetPreviewSet) {
        const auto previewStart = std::chrono::steady_clock::now();
        m_impl->showCollisionGeometry = false;
        m_impl->backgroundColor = makeBackgroundColor(m_impl->defaultBackgroundColor);
        m_impl->buildToolAssetPreview();
        m_impl->initialized = true;
        LOG_DEBUG("rs2026") << "ProjectScene initialize tool preview: elapsedMs="
            << elapsedMilliseconds(previewStart)
            << ", totalMs=" << elapsedMilliseconds(initializeStart);
        return true;
    }

    const auto ensureStart = std::chrono::steady_clock::now();
    m_impl->ensureProjectDocument();
    const double ensureMs = elapsedMilliseconds(ensureStart);
    LOG_DEBUG("rs2026") << "ProjectScene ensureProjectDocument: elapsedMs="
        << ensureMs;

    const auto robotsStart = std::chrono::steady_clock::now();
    m_impl->buildProjectRobots();
    const double robotsMs = elapsedMilliseconds(robotsStart);
    LOG_DEBUG("rs2026") << "ProjectScene buildProjectRobots: elapsedMs="
        << robotsMs;

    const auto mountedAttachmentsStart = std::chrono::steady_clock::now();
    m_impl->rebuildMountedAttachmentGraph();
    const double attachmentGraphMs = elapsedMilliseconds(mountedAttachmentsStart);
    LOG_DEBUG("rs2026") << "ProjectScene rebuildMountedAttachmentGraph: elapsedMs="
        << attachmentGraphMs;

    const auto objectsStart = std::chrono::steady_clock::now();
    m_impl->buildProjectObjects();
    const double objectsMs = elapsedMilliseconds(objectsStart);
    LOG_DEBUG("rs2026") << "ProjectScene buildProjectObjects: elapsedMs="
        << objectsMs;

    const auto pointCloudsStart = std::chrono::steady_clock::now();
    m_impl->buildProjectPointClouds();
    const double pointCloudsMs = elapsedMilliseconds(pointCloudsStart);
    LOG_DEBUG("rs2026") << "ProjectScene buildProjectPointClouds: elapsedMs="
        << pointCloudsMs;

    const auto toolsStart = std::chrono::steady_clock::now();
    m_impl->buildProjectToolAttachments();
    const double toolsMs = elapsedMilliseconds(toolsStart);
    LOG_DEBUG("rs2026") << "ProjectScene buildProjectToolAttachments: elapsedMs="
        << toolsMs;

    const auto collisionStart = std::chrono::steady_clock::now();
    m_impl->applyProjectCollisionAndView();
    const double collisionMs = elapsedMilliseconds(collisionStart);
    LOG_DEBUG("rs2026") << "ProjectScene applyProjectCollisionAndView: elapsedMs="
        << collisionMs;

    const auto cameraStart = std::chrono::steady_clock::now();
    setCameraView(ProjectSceneCameraView::Home);
    const double cameraMs = elapsedMilliseconds(cameraStart);
    m_impl->initialized = true;
    const double totalMs = elapsedMilliseconds(initializeStart);
    const std::string detail =
        "robots=" + std::to_string(m_impl->robots.size()) +
        " objects=" + std::to_string(m_impl->projectDocument.objects.size()) +
        " pointClouds=" + std::to_string(m_impl->projectDocument.pointClouds.size()) +
        " tools=" + std::to_string(m_impl->toolAttachments.size()) +
        " detectors=" + std::to_string(m_impl->collisionDetectors.size());
    std::cout << "+------------------------------------------------------------------------------+\n";
    LOG_DEBUG("rs2026") << "+------------------------------------------------------------------------------+";
    logProfileRow("Scene OpenGL runtime", openGlMs, "");
    logProfileRow("Scene document ready", ensureMs, "");
    logProfileRow("Scene build robots", robotsMs, "count=" + std::to_string(m_impl->robots.size()));
    logProfileRow("Scene attachment graph", attachmentGraphMs, "attachments=" + std::to_string(m_impl->mountedAttachments.attachments().size()));
    logProfileRow("Scene build objects", objectsMs, "count=" + std::to_string(m_impl->projectDocument.objects.size()));
    logProfileRow("Scene build point clouds", pointCloudsMs, "count=" + std::to_string(m_impl->projectDocument.pointClouds.size()));
    logProfileRow("Scene build tool visuals", toolsMs, "count=" + std::to_string(m_impl->toolAttachments.size()));
    logProfileRow("Scene collision/view", collisionMs, "detectors=" + std::to_string(m_impl->collisionDetectors.size()));
    logProfileRow("Scene camera home", cameraMs, "");
    logProfileRow("Scene initialize total", totalMs, detail);
    std::cout << "+------------------------------------------------------------------------------+\n";
    LOG_DEBUG("rs2026") << "+------------------------------------------------------------------------------+";
    LOG_DEBUG("rs2026") << "ProjectScene initialize: elapsedMs="
        << totalMs;
    return true;
}

bool ProjectScene::isInitialized() const
{
    return m_impl->initialized;
}

void ProjectScene::resize(int width, int height)
{
    m_impl->width = width > 0 ? width : 1;
    m_impl->height = height > 0 ? height : 1;
}

void ProjectScene::update(double timeSeconds)
{
    if(!m_impl->initialized) {
        return;
    }

    m_impl->updateCameraAnimation(timeSeconds);
    m_impl->renderer.setCurrentTime(static_cast<float>(timeSeconds));

    const auto poseStart = std::chrono::steady_clock::now();
    for(RuntimeRobot& robot : m_impl->robots) {
        ProjectRuntimeBuilder::applyAutoMotion(robot, timeSeconds);
        if(robot.parallelControlEnabled && robot.autoMotionEnabled) {
            solveStewartInternalLegControls(robot);
        }
    }
    m_impl->syncAllStewartFollowers();
    for(RuntimeRobot& robot : m_impl->robots) {
        ProjectRuntimeBuilder::updateRobotPose(robot);
    }
    m_impl->applyAllStewartFollowerVisualOverrides();
    m_impl->syncProjectToolAttachments();
    m_impl->lastRobotPoseUpdateMs = elapsedMilliseconds(poseStart);

    if(m_impl->collisionQueriesEnabled) {
        const auto worldUpdateStart = std::chrono::steady_clock::now();
        m_impl->collisionScene.update();
        m_impl->lastCollisionWorldUpdateMs = elapsedMilliseconds(worldUpdateStart);
    } else {
        m_impl->lastCollisionWorldUpdateMs = 0.0;
    }
    ++m_impl->collisionQueryFrame;

    ProjectCollisionDetectorRuntime* activeDetector = m_impl->activeCollisionDetector();
    CollisionResult activeResult;
    std::unordered_set<ObjectID> collidingObjects;
    std::unordered_set<ObjectID> activeCollidingObjects;
    std::size_t queriedDetectorCount = 0;
    double totalQueryMs = 0.0;
    double totalCheckMs = 0.0;
    double totalDistanceMs = 0.0;
    for(ProjectCollisionDetectorRuntime& detector : m_impl->collisionDetectors) {
        if(m_impl->collisionQueriesEnabled && detector.enabled) {
            const bool isActive = &detector == activeDetector;
            const bool wasMissingResult = !detector.hasResult;

            const auto queryStart = std::chrono::steady_clock::now();
            detector.effectiveIncludePairCount =
                m_impl->collisionScene.effectiveIncludePairCount(detector.options);
            const auto checkStart = std::chrono::steady_clock::now();
            m_impl->collisionScene.checkCollision(detector.options, detector.lastResult);
            const double checkMs = elapsedMilliseconds(checkStart);
            detector.lastCheckMs = checkMs;
            totalCheckMs += checkMs;
            double distanceMs = 0.0;
            const bool wantsNearest =
                detector.options.enableDistance || detector.options.enableNearestPoints;
            detector.nearestState = "NotComputed";
            detector.nearestReason.clear();
            if(!wantsNearest) {
                detector.nearestState = "Disabled";
                detector.nearestReason = "distance and nearest point output are disabled";
            } else if(detector.lastResult.inCollision()) {
                detector.nearestState = "SuppressedInCollision";
                detector.nearestReason = "detector is colliding; overlapping nearest points are not meaningful";
            } else {
                const auto distanceStart = std::chrono::steady_clock::now();
                CollisionResult distanceResult;
                m_impl->collisionScene.distance(detector.options, distanceResult);
                mergeDistanceResult(detector.lastResult, distanceResult);
                distanceMs = elapsedMilliseconds(distanceStart);
                totalDistanceMs += distanceMs;
                detector.lastNearestQueryFrame = m_impl->collisionQueryFrame;
                if(detector.lastResult.hasNearestPoints) {
                    detector.nearestState = "Valid";
                } else if(!distanceResult.message.empty()) {
                    detector.nearestState = "InvalidBackendPoints";
                    detector.nearestReason = distanceResult.message;
                } else {
                    detector.nearestState = "NotComputed";
                    detector.nearestReason = "distance query returned no nearest points";
                }
            }
            detector.lastDistanceMs = distanceMs;
            const double queryMs = elapsedMilliseconds(queryStart);
            detector.lastQueryMs = queryMs;
            totalQueryMs += queryMs;
            ++queriedDetectorCount;
            if(wasMissingResult || queryMs > 10.0) {
                LOG_DEBUG("rs2026") << "Project collision query slow: detector=" << detector.id
                    << ", active=" << isActive
                    << ", visible=" << detector.visible
                    << ", includePairs=" << detector.options.includePairs.size()
                    << ", effectivePairs=" << detector.effectiveIncludePairCount
                    << ", enableContacts=" << detector.options.enableContacts
                    << ", enableDistance=" << detector.options.enableDistance
                    << ", enableNearestPoints=" << detector.options.enableNearestPoints
                    << ", contacts=" << detector.lastResult.contacts.size()
                    << ", checkMs=" << checkMs
                    << ", distanceMs=" << distanceMs
                    << ", elapsedMs=" << queryMs;
            }
            detector.hasResult = true;
            if(&detector == activeDetector) {
                activeResult = detector.lastResult;
            }
            if(detector.visible) {
                const auto detectorObjects = collectCollidingObjects(detector.lastResult);
                collidingObjects.insert(detectorObjects.begin(), detectorObjects.end());
                if(&detector == activeDetector) {
                    activeCollidingObjects.insert(detectorObjects.begin(), detectorObjects.end());
                }
            }
        } else {
            detector.lastResult.clear();
            detector.hasResult = false;
            detector.effectiveIncludePairCount = 0;
            detector.lastCheckMs = 0.0;
            detector.lastDistanceMs = 0.0;
            detector.lastQueryMs = 0.0;
            detector.lastNearestQueryFrame = 0;
            detector.nearestState = "NotComputed";
            detector.nearestReason.clear();
        }
    }
    if(totalQueryMs > 16.0) {
        LOG_DEBUG("rs2026") << "Project collision queries slow frame: detectors="
            << m_impl->collisionDetectors.size()
            << ", queried=" << queriedDetectorCount
            << ", checkMs=" << totalCheckMs
            << ", distanceMs=" << totalDistanceMs
            << ", worldUpdateMs=" << m_impl->lastCollisionWorldUpdateMs
            << ", elapsedMs=" << totalQueryMs;
    }

    const auto overlayStart = std::chrono::steady_clock::now();
    const auto overlayHighlightStart = std::chrono::steady_clock::now();
    const bool collisionModelVariantPreviewActive =
        m_impl->previewObjectCollisionModelActive ||
        !m_impl->visibleCollisionVariantIds.empty();
    const bool showDetectorCollisionGeometry =
        m_impl->showCollisionGeometry && !collisionModelVariantPreviewActive;
    const std::unordered_set<ObjectID> noVisualCollisionHighlights;
    const std::unordered_set<ObjectID>& visualHighlightObjects =
        showDetectorCollisionGeometry ? activeCollidingObjects : noVisualCollisionHighlights;

    const std::string selectedRobotId = m_impl->selectionState.selectedRobotId();
    const std::string selectedLinkName = m_impl->selectionState.selectedLinkName();
    const std::string selectedSceneObjectId = m_impl->selectionState.selectedSceneObjectId();
    const std::string selectedMountedAttachmentId = m_impl->selectionState.selectedMountedAttachmentId();

    for(const RuntimeRobot& robot : m_impl->robots) {
        updateMeshOverlays(
            robot.meshOverlays,
            visualHighlightObjects,
            showDetectorCollisionGeometry,
            selectedRobotId == robot.documentId ? selectedLinkName : std::string());
    }
    updateRobotVisualHighlights(m_impl->robots, visualHighlightObjects);
    updateSceneObjectHighlights(m_impl->objects, visualHighlightObjects, selectedSceneObjectId);
    updateToolAttachmentHighlights(
        m_impl->toolAttachments,
        m_impl->objects,
        visualHighlightObjects,
        selectedMountedAttachmentId);
    applySelectionOverrides(m_impl->selectionOverrides);
    applySelectionOverrides(m_impl->pairPreviewOverrides);
    m_impl->lastCollisionOverlayHighlightMs = elapsedMilliseconds(overlayHighlightStart);

    const std::size_t activeIncludePairCount = activeDetector != nullptr
        ? activeDetector->options.includePairs.size()
        : 0;
    if(m_impl->lastIncludePairCount != activeIncludePairCount ||
        m_impl->lastContactCount != activeResult.contacts.size()) {
        LOG_DEBUG("rs2026") << "Project collision frame: detector="
            << (activeDetector != nullptr ? activeDetector->id : std::string())
            << ", includePairs=" << activeIncludePairCount
            << ", effectivePairs=" << (activeDetector != nullptr ? activeDetector->effectiveIncludePairCount : 0)
            << ", contacts=" << activeResult.contacts.size()
            << ", collidingObjects=" << collidingObjects.size();
        m_impl->lastIncludePairCount = activeIncludePairCount;
        m_impl->lastContactCount = activeResult.contacts.size();
    }

    m_impl->lastCollisionOverlayDebugBuildMs = 0.0;
    m_impl->lastCollisionOverlayVariantFilterMs = 0.0;
    m_impl->lastCollisionOverlayDebugSubmitMs = 0.0;
    m_impl->lastCollisionOverlayAuxFramesMs = 0.0;
    m_impl->lastCollisionOverlayDetectorCount = 0;
    m_impl->lastCollisionOverlayGeometryCount = 0;
    m_impl->lastCollisionOverlayContactCount = 0;
    m_impl->lastCollisionOverlayNearestCount = 0;
    m_impl->lastCollisionOverlayPrimitiveEstimate = 0;
    m_impl->lastCollisionOverlayLineEstimate = 0;

    bool collisionGeometryOverlaySubmitted = false;
    for(const ProjectCollisionDetectorRuntime& detector : m_impl->collisionDetectors) {
        if(!detector.enabled || !detector.visible || !detector.hasResult) {
            continue;
        }

        CollisionVisualizationOptions visualizationOptions = detector.visualization;
        const bool allowCollisionGeometryOverlay =
            showDetectorCollisionGeometry &&
            visualizationOptions.showAllCollisionGeometry &&
            !collisionGeometryOverlaySubmitted;
        visualizationOptions.showAllCollisionGeometry = allowCollisionGeometryOverlay;
        if(!allowCollisionGeometryOverlay) {
            visualizationOptions.showOnlyCollidingObjects = false;
        }
        if(!showDetectorCollisionGeometry) {
            visualizationOptions.showAllCollisionGeometry = false;
            visualizationOptions.showOnlyCollidingObjects = false;
        }

        CollisionDebugDrawData debugData;
        if(allowCollisionGeometryOverlay) {
            CollisionVisualizationOptions geometryOptions = visualizationOptions;
            geometryOptions.showAllCollisionGeometry = true;
            geometryOptions.showOnlyCollidingObjects = false;
            geometryOptions.showContacts = false;
            geometryOptions.showNormals = false;
            geometryOptions.showNearestPoints = false;

            const std::string cacheKey = collisionGeometryOverlayCacheKey(
                geometryOptions,
                m_impl->collisionGeometryOverlayCacheVersion);
            if(!m_impl->collisionGeometryOverlayCache.valid ||
                m_impl->collisionGeometryOverlayCache.key != cacheKey) {
                const auto geometryBuildStart = std::chrono::steady_clock::now();
                m_impl->collisionGeometryOverlayCache.data =
                    m_impl->collisionScene.buildDebugDraw(geometryOptions, nullptr);
                m_impl->lastCollisionOverlayDebugBuildMs += elapsedMilliseconds(geometryBuildStart);

                const auto variantFilterStart = std::chrono::steady_clock::now();
                m_impl->filterCollisionDebugDrawByVisibleVariants(
                    m_impl->collisionGeometryOverlayCache.data);
                m_impl->lastCollisionOverlayVariantFilterMs += elapsedMilliseconds(variantFilterStart);

                m_impl->collisionGeometryOverlayCache.key = cacheKey;
                m_impl->collisionGeometryOverlayCache.valid = true;
                m_impl->collisionGeometryOverlayCache.version =
                    m_impl->collisionGeometryOverlayCacheVersion;
            }

            const std::unordered_set<ObjectID> detectorHighlightObjects =
                collectCollidingObjects(detector.lastResult);
            const auto geometryAppendStart = std::chrono::steady_clock::now();
            appendCachedCollisionGeometry(
                m_impl->collisionGeometryOverlayCache.data,
                m_impl->collisionScene,
                visualizationOptions,
                detectorHighlightObjects,
                debugData);
            m_impl->lastCollisionOverlayDebugBuildMs += elapsedMilliseconds(geometryAppendStart);

            if(!debugData.geometry.empty()) {
                collisionGeometryOverlaySubmitted = true;
            }
        }

        CollisionVisualizationOptions resultOptions = visualizationOptions;
        resultOptions.showAllCollisionGeometry = false;
        resultOptions.showOnlyCollidingObjects = false;
        const auto resultBuildStart = std::chrono::steady_clock::now();
        CollisionDebugDrawBuilder::appendResult(detector.lastResult, resultOptions, debugData);
        m_impl->lastCollisionOverlayDebugBuildMs += elapsedMilliseconds(resultBuildStart);
        debugData.setDetectorSource(detector.id, detector.name);

        ++m_impl->lastCollisionOverlayDetectorCount;
        m_impl->lastCollisionOverlayGeometryCount += debugData.geometry.size();
        m_impl->lastCollisionOverlayContactCount += debugData.contacts.size();
        m_impl->lastCollisionOverlayNearestCount += debugData.nearestPoints.size();
        const std::size_t visibleContactNormalCount = std::count_if(
            debugData.contacts.begin(),
            debugData.contacts.end(),
            [](const CollisionContactDebugDrawDesc& contact) {
                return contact.showNormal;
            });
        m_impl->lastCollisionOverlayPrimitiveEstimate +=
            debugData.geometry.size() +
            debugData.contacts.size() +
            visibleContactNormalCount +
            debugData.nearestPoints.size() * 2;
        m_impl->lastCollisionOverlayLineEstimate += debugData.nearestPoints.size();

        robot_render::CollisionRenderDrawOptions drawOptions;
        drawOptions.adaptiveContactSize = true;
        if(m_impl->mainCameraNode && m_impl->mainCameraNode->camera()) {
            drawOptions.cameraPosition =
                m_impl->mainCameraNode->camera()->position().cast<double>();
        }
        const auto debugSubmitStart = std::chrono::steady_clock::now();
        robot_render::CollisionRenderBridge::draw(debugData, m_impl->renderer.debug(), drawOptions);
        m_impl->lastCollisionOverlayDebugSubmitMs += elapsedMilliseconds(debugSubmitStart);
    }

    const auto auxFramesStart = std::chrono::steady_clock::now();
    m_impl->drawSelectedJointFrame();
    m_impl->drawVisibleObjectFrames();
    m_impl->drawSelectedObjectFrame();
    m_impl->drawPreviewRobotMountFrames();
    m_impl->drawPinnedRobotMountFrames();
    m_impl->drawActiveToolAttachmentFrames();
    m_impl->drawRobotCollisionModelVariantPreview();
    m_impl->drawObjectCollisionModelVariantPreview();
    m_impl->drawInteractionModeHints();
    m_impl->drawToolAssetPreviewFrames();
    m_impl->lastCollisionOverlayAuxFramesMs = elapsedMilliseconds(auxFramesStart);
    m_impl->lastCollisionOverlayMs = elapsedMilliseconds(overlayStart);
}

void ProjectScene::render()
{
    if(!m_impl->initialized) {
        return;
    }

    glViewport(0, 0, m_impl->width, m_impl->height);

    auto camera = m_impl->mainCameraNode->camera();
    if(m_impl->height > 0) {
        camera->setAspect(static_cast<float>(m_impl->width) / static_cast<float>(m_impl->height));
    }

    glClearColor(
        m_impl->backgroundColor.x(),
        m_impl->backgroundColor.y(),
        m_impl->backgroundColor.z(),
        m_impl->backgroundColor.w());
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_impl->sceneGraph.update();
    m_impl->renderer.updateSceneUBO(m_impl->sceneGraph, m_impl->mainCameraNode.get());
    m_impl->renderer.render(m_impl->sceneGraph);
}

void ProjectScene::onMouseMove(float dx, float dy, int button)
{
    if(m_impl->controller) {
        const Eigen::Vector3f previousPosition =
            m_impl->mainCameraNode && m_impl->mainCameraNode->camera()
                ? m_impl->mainCameraNode->camera()->position()
                : Eigen::Vector3f::Zero();
        m_impl->cameraAnimationActive = false;
        m_impl->controller->onMouseMove(dx, dy, button);
        if(m_impl->mainCameraNode && m_impl->mainCameraNode->camera() && m_impl->cameraLookAt.valid) {
            const Eigen::Vector3f currentPosition = m_impl->mainCameraNode->camera()->position();
            if(button == 1) {
                m_impl->cameraLookAt.target += currentPosition - previousPosition;
            }
            m_impl->cameraLookAt.position = currentPosition;
        }
    }
}

void ProjectScene::onScroll(float delta)
{
    if(m_impl->controller) {
        m_impl->cameraAnimationActive = false;
        m_impl->controller->onScroll(delta);
        if(m_impl->mainCameraNode && m_impl->mainCameraNode->camera() && m_impl->cameraLookAt.valid) {
            m_impl->cameraLookAt.position = m_impl->mainCameraNode->camera()->position();
        }
    }
}

void ProjectScene::setCameraView(ProjectSceneCameraView view)
{
    if(!m_impl->mainCameraNode || !m_impl->mainCameraNode->camera()) {
        return;
    }

    m_impl->cameraAnimationActive = false;
    m_impl->applyCameraLookAt(makeCameraLookAtState(m_impl->fullSceneBounds(), view, 1.25));
}

void ProjectScene::focusMountFrameLink(const std::string& robotId, const std::string& linkName)
{
    if(!m_impl->initialized || robotId.empty() || linkName.empty()) {
        return;
    }

    if(m_impl->mountFrameLinkFocusActive &&
        m_impl->mountFrameFocusRobotId == robotId &&
        m_impl->mountFrameFocusLinkName == linkName) {
        return;
    }

    const CameraSceneBounds bounds = m_impl->robotLinkBounds(robotId, linkName);
    if(!bounds.valid) {
        return;
    }

    m_impl->mountFrameLinkFocusActive = true;
    m_impl->mountFrameFocusRobotId = robotId;
    m_impl->mountFrameFocusLinkName = linkName;
    m_impl->objectFrameObjectFocusActive = false;
    m_impl->objectFrameFocusObjectId.clear();
    m_impl->mountedAttachmentFocusActive = false;
    m_impl->mountedAttachmentFocusId.clear();
    m_impl->applyMountFrameLinkFocusVisibility();
    m_impl->animateCameraTo(makeCameraLookAtState(bounds, ProjectSceneCameraView::Front, 1.8), 0.55);
}

void ProjectScene::clearMountFrameLinkFocus()
{
    if(!m_impl->initialized || !m_impl->mountFrameLinkFocusActive) {
        return;
    }

    m_impl->mountFrameLinkFocusActive = false;
    m_impl->mountFrameFocusRobotId.clear();
    m_impl->mountFrameFocusLinkName.clear();
    m_impl->applyMountFrameLinkFocusVisibility();
    m_impl->animateCameraTo(makeCameraLookAtState(m_impl->fullSceneBounds(), ProjectSceneCameraView::Home, 1.25), 0.55);
}

void ProjectScene::focusObjectFrameObject(const std::string& objectId)
{
    if(!m_impl->initialized || objectId.empty()) {
        return;
    }

    if(m_impl->objectFrameObjectFocusActive &&
        m_impl->objectFrameFocusObjectId == objectId) {
        return;
    }

    const CameraSceneBounds bounds = m_impl->sceneObjectBounds(objectId);
    if(!bounds.valid) {
        return;
    }

    m_impl->mountFrameLinkFocusActive = false;
    m_impl->mountFrameFocusRobotId.clear();
    m_impl->mountFrameFocusLinkName.clear();
    m_impl->objectFrameObjectFocusActive = true;
    m_impl->objectFrameFocusObjectId = objectId;
    m_impl->mountedAttachmentFocusActive = false;
    m_impl->mountedAttachmentFocusId.clear();
    m_impl->applyMountFrameLinkFocusVisibility();
    m_impl->animateCameraTo(makeCameraLookAtState(bounds, ProjectSceneCameraView::Front, 1.8), 0.55);
}

void ProjectScene::clearObjectFrameObjectFocus()
{
    if(!m_impl->initialized || !m_impl->objectFrameObjectFocusActive) {
        return;
    }

    m_impl->objectFrameObjectFocusActive = false;
    m_impl->objectFrameFocusObjectId.clear();
    m_impl->applyMountFrameLinkFocusVisibility();
    m_impl->animateCameraTo(makeCameraLookAtState(m_impl->fullSceneBounds(), ProjectSceneCameraView::Home, 1.25), 0.55);
}

void ProjectScene::focusMountedAttachment(const std::string& attachmentId)
{
    if(!m_impl->initialized || attachmentId.empty()) {
        return;
    }

    if(m_impl->mountedAttachmentFocusActive &&
        m_impl->mountedAttachmentFocusId == attachmentId) {
        return;
    }

    const CameraSceneBounds bounds = m_impl->mountedAttachmentBounds(attachmentId);
    if(!bounds.valid) {
        return;
    }

    m_impl->mountFrameLinkFocusActive = false;
    m_impl->mountFrameFocusRobotId.clear();
    m_impl->mountFrameFocusLinkName.clear();
    m_impl->objectFrameObjectFocusActive = false;
    m_impl->objectFrameFocusObjectId.clear();
    m_impl->mountedAttachmentFocusActive = true;
    m_impl->mountedAttachmentFocusId = attachmentId;
    m_impl->setActiveToolAttachment(attachmentId);
    m_impl->applyMountFrameLinkFocusVisibility();
    m_impl->animateCameraTo(makeCameraLookAtState(bounds, ProjectSceneCameraView::Front, 1.8), 0.55);
}

void ProjectScene::clearMountedAttachmentFocus()
{
    if(!m_impl->initialized || !m_impl->mountedAttachmentFocusActive) {
        return;
    }

    m_impl->mountedAttachmentFocusActive = false;
    m_impl->mountedAttachmentFocusId.clear();
    m_impl->applyMountFrameLinkFocusVisibility();
    m_impl->animateCameraTo(makeCameraLookAtState(m_impl->fullSceneBounds(), ProjectSceneCameraView::Home, 1.25), 0.55);
}

void ProjectScene::previewObjectCollisionModelVariant(
    const std::string& objectId,
    const std::string& variantId)
{
    if(!m_impl->initialized || objectId.empty()) {
        return;
    }

    m_impl->previewObjectCollisionModelActive = true;
    m_impl->previewObjectCollisionModelObjectId = objectId;
    m_impl->previewObjectCollisionModelVariantId = variantId;
    m_impl->applyMountFrameLinkFocusVisibility();
    m_impl->updateObjectCollisionModelVariantPreviewMeshes();
    m_impl->invalidateCollisionGeometryOverlayCache();
}

void ProjectScene::clearObjectCollisionModelVariantPreview()
{
    if(!m_impl->initialized) {
        return;
    }

    m_impl->previewObjectCollisionModelActive = false;
    m_impl->previewObjectCollisionModelObjectId.clear();
    m_impl->previewObjectCollisionModelVariantId.clear();
    m_impl->hideObjectCollisionModelVariantPreviewMeshes();
    m_impl->applyMountFrameLinkFocusVisibility();
    m_impl->invalidateCollisionGeometryOverlayCache();
}

void ProjectScene::previewCollisionPairTargets(
    const std::string& robotAId,
    const std::string& linkAName,
    const std::string& objectAId,
    const std::string& attachmentAId,
    const std::string& robotBId,
    const std::string& linkBName,
    const std::string& objectBId,
    const std::string& attachmentBId)
{
    if(!m_impl->initialized) {
        return;
    }

    restoreMaterialOverrides(m_impl->pairPreviewOverrides);

    const Eigen::Vector4f colorA(0.05f, 0.65f, 1.0f, 1.0f);
    const Eigen::Vector3f emissiveA(0.01f, 0.08f, 0.22f);
    const Eigen::Vector4f colorB(1.0f, 0.62f, 0.12f, 1.0f);
    const Eigen::Vector3f emissiveB(0.22f, 0.10f, 0.01f);

    const auto addModelOverrides = [&](const std::shared_ptr<rendercore::Model>& model,
                                       const Eigen::Vector4f& color,
                                       const Eigen::Vector3f& emissive) {
        if(!model) {
            return;
        }
        for(unsigned int i = 0; i < model->subMeshCount(); ++i) {
            auto& subMesh = model->subMesh(i);
            m_impl->pairPreviewOverrides.push_back(MaterialOverride{
                model,
                i,
                subMesh.material,
                color,
                emissive });
        }
    };

    const auto addLinkOverrides = [&](const std::string& robotId,
                                      const std::string& linkName,
                                      const Eigen::Vector4f& color,
                                      const Eigen::Vector3f& emissive) {
        if(robotId.empty()) {
            return;
        }
        RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
        if(robot == nullptr || !robot->visualBridge) {
            return;
        }
        if(linkName.empty()) {
            for(const auto& [targetLinkName, link] : robot->model.links) {
                (void)link;
                for(const auto& node : robot->visualBridge->visualNodes(targetLinkName)) {
                    addModelOverrides(node ? node->model() : nullptr, color, emissive);
                }
            }
            return;
        }
        for(const auto& node : robot->visualBridge->visualNodes(linkName)) {
            addModelOverrides(node ? node->model() : nullptr, color, emissive);
        }
    };

    const auto addObjectOverrides = [&](const std::string& objectId,
                                        const Eigen::Vector4f& color,
                                        const Eigen::Vector3f& emissive) {
        if(objectId.empty()) {
            return;
        }
        RuntimeSceneObject* object = findRuntimeObject(m_impl->objects, objectId);
        if(object == nullptr) {
            return;
        }
        addModelOverrides(object->visualModel, color, emissive);
    };

    const auto addAttachmentOverrides = [&](const std::string& attachmentId,
                                            const Eigen::Vector4f& color,
                                            const Eigen::Vector3f& emissive) {
        if(attachmentId.empty()) {
            return;
        }
        for(RuntimeToolAttachmentVisual& attachment : m_impl->toolAttachments) {
            if(attachment.documentId == attachmentId) {
                addModelOverrides(attachment.visual.model, color, emissive);
                return;
            }
        }
    };

    addLinkOverrides(robotAId, linkAName, colorA, emissiveA);
    addObjectOverrides(objectAId, colorA, emissiveA);
    addAttachmentOverrides(attachmentAId, colorA, emissiveA);
    addLinkOverrides(robotBId, linkBName, colorB, emissiveB);
    addObjectOverrides(objectBId, colorB, emissiveB);
    addAttachmentOverrides(attachmentBId, colorB, emissiveB);
    applySelectionOverrides(m_impl->pairPreviewOverrides);
}

void ProjectScene::setInteractionMode(ProjectSceneInteractionMode mode)
{
    m_impl->interactionMode = mode;
}

ProjectSceneInteractionMode ProjectScene::interactionMode() const
{
    return m_impl->interactionMode;
}

ProjectScenePickResult ProjectScene::pickScreenPoint(int x, int y) const
{
    ProjectScenePickRay ray;
    if(!screenRayFromCamera(m_impl->mainCameraNode ? m_impl->mainCameraNode->camera() : nullptr,
        m_impl->width,
        m_impl->height,
        x,
        y,
        ray)) {
        return ProjectScenePickResult();
    }

    std::vector<ProjectScenePickCandidate> candidates;
    for(const RuntimeRobot& robot : m_impl->robots) {
        ProjectScenePickCandidate robotCandidate =
            makePointPickCandidate(
                ProjectScenePickTargetKind::Robot,
                robot.baseTransform.translation(),
                0.45);
        robotCandidate.robotId = robot.documentId;
        candidates.push_back(std::move(robotCandidate));

        if(!robot.instance) {
            continue;
        }

        for(const std::string& linkName : robot.model.linkNames) {
            if(robot.model.links.find(linkName) == robot.model.links.end()) {
                continue;
            }

            ProjectScenePickCandidate linkCandidate =
                makePointPickCandidate(
                    ProjectScenePickTargetKind::RobotLink,
                    robot.instance->getLinkTransform(linkName).translation(),
                    0.20);
            linkCandidate.robotId = robot.documentId;
            linkCandidate.linkName = linkName;
            candidates.push_back(std::move(linkCandidate));
        }
    }

    for(const simulation_project::RobotMountDesc& mount : m_impl->projectDocument.robotMounts) {
        const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, mount.robotId);
        if(robot == nullptr ||
            !robot->instance ||
            robot->model.links.find(mount.linkName) == robot->model.links.end()) {
            continue;
        }

        const collision::Transform3 worldLink = robot->instance->getLinkTransform(mount.linkName);
        const collision::Transform3 worldMount =
            worldLink * ProjectRuntimeBuilder::makeTransform(mount.linkToMount);
        ProjectScenePickCandidate mountCandidate =
            makePointPickCandidate(
                ProjectScenePickTargetKind::RobotMount,
                worldMount.translation(),
                0.18);
        mountCandidate.robotId = mount.robotId;
        mountCandidate.linkName = mount.linkName;
        mountCandidate.robotMountId = mount.id;
        candidates.push_back(std::move(mountCandidate));
    }

    for(const RuntimeToolAttachmentVisual& attachment : m_impl->toolAttachments) {
        if(!attachment.visible) {
            continue;
        }

        ProjectScenePickCandidate attachmentCandidate =
            makePointPickCandidate(
                ProjectScenePickTargetKind::MountedAttachment,
                attachment.worldToolMount.translation(),
                0.20);
        attachmentCandidate.robotId = attachment.robotId;
        attachmentCandidate.linkName = attachment.linkName;
        attachmentCandidate.robotMountId = attachment.robotMountId;
        attachmentCandidate.mountedAttachmentId = attachment.documentId;
        candidates.push_back(attachmentCandidate);

        if(attachment.collisionObjectIndex < m_impl->objects.size()) {
            const RuntimeSceneObject& object = m_impl->objects[attachment.collisionObjectIndex];
            if(object.collisionObject) {
                ProjectScenePickCandidate aabbCandidate = attachmentCandidate;
                setCandidateAabb(aabbCandidate, object.collisionObject->fcl()->getAABB());
                candidates.push_back(std::move(aabbCandidate));
            }
            for(const RuntimeSceneCollisionObject& collisionObject : object.collisionObjects) {
                if(collisionObject.collisionObject) {
                    ProjectScenePickCandidate aabbCandidate = attachmentCandidate;
                    setCandidateAabb(aabbCandidate, collisionObject.collisionObject->fcl()->getAABB());
                    candidates.push_back(std::move(aabbCandidate));
                }
            }
        }
    }

    for(const RuntimeSceneObject& object : m_impl->objects) {
        if(object.objectType == "toolAttachment" || object.objectType == "pointCloud") {
            continue;
        }

        ProjectScenePickCandidate objectCandidate =
            makePointPickCandidate(
                ProjectScenePickTargetKind::SceneObject,
                object.transform.translation(),
                0.35);
        objectCandidate.sceneObjectId = object.documentId;

        bool pushedAabb = false;
        if(object.collisionObject) {
            ProjectScenePickCandidate aabbCandidate = objectCandidate;
            setCandidateAabb(aabbCandidate, object.collisionObject->fcl()->getAABB());
            candidates.push_back(std::move(aabbCandidate));
            pushedAabb = true;
        }
        for(const RuntimeSceneCollisionObject& collisionObject : object.collisionObjects) {
            if(collisionObject.collisionObject) {
                ProjectScenePickCandidate aabbCandidate = objectCandidate;
                setCandidateAabb(aabbCandidate, collisionObject.collisionObject->fcl()->getAABB());
                candidates.push_back(std::move(aabbCandidate));
                pushedAabb = true;
            }
        }
        if(!pushedAabb) {
            candidates.push_back(std::move(objectCandidate));
        }
    }

    for(const RuntimeSceneObject& pointCloud : m_impl->objects) {
        if(pointCloud.objectType != "pointCloud") {
            continue;
        }

        ProjectScenePickCandidate pointCloudCandidate =
            makePointPickCandidate(
                ProjectScenePickTargetKind::PointCloud,
                pointCloud.transform.translation(),
                0.35);
        pointCloudCandidate.sceneObjectId = pointCloud.documentId;

        bool pushedAabb = false;
        if(pointCloud.pointCloudBoundsValid) {
            ProjectScenePickCandidate boundsCandidate = pointCloudCandidate;
            setCandidateTransformedBounds(
                boundsCandidate,
                pointCloud.transform,
                pointCloud.pointCloudLocalBoundsMin,
                pointCloud.pointCloudLocalBoundsMax);
            candidates.push_back(std::move(boundsCandidate));
            pushedAabb = true;
        }
        for(const RuntimeSceneCollisionObject& collisionObject : pointCloud.collisionObjects) {
            if(collisionObject.collisionObject) {
                ProjectScenePickCandidate aabbCandidate = pointCloudCandidate;
                setCandidateAabb(aabbCandidate, collisionObject.collisionObject->fcl()->getAABB());
                candidates.push_back(std::move(aabbCandidate));
                pushedAabb = true;
            }
        }
        if(!pushedAabb) {
            candidates.push_back(std::move(pointCloudCandidate));
        }
    }

    return ProjectScenePickingService::pick(ray, candidates, m_impl->interactionMode);
}

bool ProjectScene::applySurfaceScalarOverlay(
    const smrobot::visualization::SurfaceScalarOverlay& overlay,
    std::string* errorMessage)
{
    const auto fail = [&](const std::string& message) {
        if(errorMessage != nullptr) {
            *errorMessage = message;
        }
        return false;
    };

    RuntimeSceneObject* runtime = findRuntimeObject(m_impl->objects, overlay.objectId);
    const simulation_project::SceneObjectDesc* objectDesc =
        findSceneObjectDesc(m_impl->projectDocument, overlay.objectId);
    if(runtime == nullptr || objectDesc == nullptr || !runtime->visualNode) {
        return fail("The target scene object is not available in the viewport.");
    }
    if(runtime->objectType == "pointCloud" || runtime->objectType == "toolAttachment") {
        return fail("Surface scalar overlays require a mesh scene object.");
    }

    const std::filesystem::path sourcePath = simulation_project::AssetResolver::resolveProjectPath(
        ProjectRuntimeBuilder::makeAssetResolveContext(
            m_impl->projectBasePath,
            m_impl->projectDocument.assetSearchPaths),
        objectDesc->sourcePath);
    std::string loadError;
    const std::shared_ptr<assetcore::ModelDesc> sourceDesc =
        assetcore::AssetManager::instance().tryLoadModel(
            pathToUtf8(sourcePath),
            static_cast<float>(objectDesc->visualScale),
            &loadError);
    if(!sourceDesc) {
        return fail(loadError.empty() ? "Failed to load the target mesh." : loadError);
    }
    if(overlay.subMeshes.size() != sourceDesc->subMeshCount()) {
        return fail("The scalar field submesh count does not match the target mesh.");
    }

    assetcore::ModelDesc coloredDesc = *sourceDesc;
    auto runtimeOverlay = std::make_shared<RuntimeSurfaceScalarOverlay>();
    runtimeOverlay->descriptor = overlay;
    runtimeOverlay->subMeshes.reserve(coloredDesc.subMeshCount());
    for(std::size_t subMeshIndex = 0; subMeshIndex < coloredDesc.subMeshCount(); ++subMeshIndex) {
        assetcore::GeometryDesc& geometry = coloredDesc.subMeshes()[subMeshIndex].geometry;
        const std::vector<double>& values = overlay.subMeshes[subMeshIndex].values;
        if(values.size() != geometry.positions.size()) {
            return fail("The scalar field vertex count does not match the target mesh.");
        }

        geometry.colors.resize(values.size());
        RuntimeSurfaceScalarSubMesh runtimeSubMesh;
        runtimeSubMesh.positions.reserve(geometry.positions.size());
        runtimeSubMesh.indices = geometry.indices;
        runtimeSubMesh.values = values;
        for(std::size_t vertexIndex = 0; vertexIndex < values.size(); ++vertexIndex) {
            geometry.colors[vertexIndex] = gammaToLinear(
                overlay.colorMap.sample(values[vertexIndex], overlay.range));
            runtimeSubMesh.positions.push_back(geometry.positions[vertexIndex].cast<double>());
        }
        if(runtimeSubMesh.indices.empty()) {
            runtimeSubMesh.indices.reserve(runtimeSubMesh.positions.size());
            for(std::size_t vertexIndex = 0; vertexIndex < runtimeSubMesh.positions.size(); ++vertexIndex) {
                runtimeSubMesh.indices.push_back(static_cast<std::uint32_t>(vertexIndex));
            }
        }
        runtimeOverlay->subMeshes.push_back(std::move(runtimeSubMesh));
    }

    std::shared_ptr<rendercore::Model> overlayModel =
        rendercore::ModelManager::instance().buildModelFromDesc(coloredDesc);
    if(!overlayModel) {
        return fail("Failed to build the colored surface model.");
    }
    auto vertexColorMaterial = std::make_shared<rendercore::Material>();
    vertexColorMaterial->type = rendercore::MaterialType::ScalarOverlay;
    vertexColorMaterial->baseColor = Eigen::Vector4f::Ones();
    for(unsigned int subMeshIndex = 0; subMeshIndex < overlayModel->subMeshCount(); ++subMeshIndex) {
        overlayModel->subMesh(subMeshIndex).material = vertexColorMaterial;
    }

    runtimeOverlay->originalModel = runtime->surfaceScalarOverlay
        ? runtime->surfaceScalarOverlay->originalModel
        : runtime->visualNode->model();
    runtimeOverlay->overlayModel = overlayModel;
    runtimeOverlay->visible = true;
    runtime->surfaceScalarOverlay = runtimeOverlay;
    runtime->visualNode->setModel(overlayModel);
    if(errorMessage != nullptr) {
        errorMessage->clear();
    }
    return true;
}

bool ProjectScene::setSurfaceScalarOverlayVisible(const std::string& objectId, bool visible)
{
    RuntimeSceneObject* runtime = findRuntimeObject(m_impl->objects, objectId);
    if(runtime == nullptr || !runtime->visualNode || !runtime->surfaceScalarOverlay) {
        return false;
    }
    runtime->surfaceScalarOverlay->visible = visible;
    runtime->visualNode->setModel(
        visible
            ? runtime->surfaceScalarOverlay->overlayModel
            : runtime->surfaceScalarOverlay->originalModel);
    return true;
}

bool ProjectScene::clearSurfaceScalarOverlay(const std::string& objectId)
{
    RuntimeSceneObject* runtime = findRuntimeObject(m_impl->objects, objectId);
    if(runtime == nullptr || !runtime->surfaceScalarOverlay) {
        return false;
    }
    if(runtime->visualNode) {
        runtime->visualNode->setModel(runtime->surfaceScalarOverlay->originalModel);
    }
    runtime->surfaceScalarOverlay.reset();
    return true;
}

smrobot::visualization::SurfaceScalarProbeResult ProjectScene::probeSurfaceScalarAtScreenPoint(
    const std::string& objectId,
    int x,
    int y) const
{
    smrobot::visualization::SurfaceScalarProbeResult result;
    const RuntimeSceneObject* runtime = findRuntimeObject(m_impl->objects, objectId);
    if(runtime == nullptr || !runtime->surfaceScalarOverlay ||
        !runtime->surfaceScalarOverlay->visible) {
        return result;
    }

    ProjectScenePickRay ray;
    if(!screenRayFromCamera(
        m_impl->mainCameraNode ? m_impl->mainCameraNode->camera() : nullptr,
        m_impl->width,
        m_impl->height,
        x,
        y,
        ray)) {
        return result;
    }

    const glm::mat4 worldTransform =
        math::eigenToGlm(runtime->transform) * runtime->visualLocal;
    double nearestDistance = std::numeric_limits<double>::max();
    for(const RuntimeSurfaceScalarSubMesh& subMesh : runtime->surfaceScalarOverlay->subMeshes) {
        for(std::size_t index = 0; index + 2 < subMesh.indices.size(); index += 3) {
            const std::uint32_t indexA = subMesh.indices[index];
            const std::uint32_t indexB = subMesh.indices[index + 1];
            const std::uint32_t indexC = subMesh.indices[index + 2];
            if(indexA >= subMesh.positions.size() || indexB >= subMesh.positions.size() ||
                indexC >= subMesh.positions.size() || indexA >= subMesh.values.size() ||
                indexB >= subMesh.values.size() || indexC >= subMesh.values.size()) {
                continue;
            }

            const Eigen::Vector3d a = transformSurfacePoint(worldTransform, subMesh.positions[indexA]);
            const Eigen::Vector3d b = transformSurfacePoint(worldTransform, subMesh.positions[indexB]);
            const Eigen::Vector3d c = transformSurfacePoint(worldTransform, subMesh.positions[indexC]);
            double distance = 0.0;
            double barycentricB = 0.0;
            double barycentricC = 0.0;
            if(!intersectRayTriangle(
                ray, a, b, c, distance, barycentricB, barycentricC) ||
                distance >= nearestDistance) {
                continue;
            }

            nearestDistance = distance;
            const double barycentricA = 1.0 - barycentricB - barycentricC;
            result.hit = true;
            result.objectId = objectId;
            result.value = barycentricA * subMesh.values[indexA] +
                barycentricB * subMesh.values[indexB] +
                barycentricC * subMesh.values[indexC];
            result.worldPosition = ray.origin + ray.direction * distance;
        }
    }
    return result;
}

void ProjectScene::setShowCollisionGeometry(bool visible)
{
    const auto showStart = std::chrono::steady_clock::now();
    if(visible && !m_impl->ensureCollisionRuntimeBuilt()) {
        LOG_WARNING("rs2026") << "ProjectScene setShowCollisionGeometry failed: collision runtime unavailable";
        return;
    }
    m_impl->showCollisionGeometry = visible;
    m_impl->applyCollisionOverlayRenderConfig();
    m_impl->invalidateCollisionGeometryOverlayCache();
    LOG_DEBUG("rs2026") << "ProjectScene setShowCollisionGeometry: visible=" << visible
        << ", robots=" << m_impl->robots.size()
        << ", displayMode=queryGeometryDebugDraw"
        << ", meshOverlayLazyBuild=disabled";
    LOG_DEBUG("rs2026") << "ProjectScene setShowCollisionGeometry done: visible=" << visible
        << ", elapsedMs=" << elapsedMilliseconds(showStart);
}

bool ProjectScene::setVisibleRobotCollisionVariant(
    const std::string& robotId,
    const std::string& linkName,
    const std::string& variantId)
{
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr || linkName.empty()) {
        return false;
    }

    const std::string key = collisionVariantSelectionKey(robotId, linkName);
    const std::string runtimeKey =
        runtimeCollisionVariantSelectionKey(static_cast<int>(robot->runtimeId), linkName);
    const auto applyPreviewSelectionMaterial = [&](bool previewActive, bool convertFromVisual) {
        if(m_impl->selectionState.selectedRobotId() != robotId ||
            m_impl->selectionState.selectedLinkName() != linkName) {
            return;
        }

        for(MaterialOverride& item : m_impl->selectionOverrides) {
            if(!previewActive) {
                item.color = Eigen::Vector4f(0.15f, 1.0f, 0.35f, 1.0f);
                item.emissive = Eigen::Vector3f(0.02f, 0.18f, 0.04f);
                continue;
            }

            if(convertFromVisual) {
                const Eigen::Vector4f previewColor(
                    kCollisionPreviewRed,
                    kCollisionPreviewGreen,
                    kCollisionPreviewBlue,
                    kCollisionPreviewAlpha);
                if(item.originalMaterial) {
                    item.color = item.originalMaterial->baseColor * (1.0f - kCollisionPreviewBlend) +
                        previewColor * kCollisionPreviewBlend;
                    item.color.w() = item.originalMaterial->baseColor.w();
                    item.emissive = item.originalMaterial->emissiveColor * (1.0f - kCollisionPreviewBlend) +
                        Eigen::Vector3f(0.0f, 0.08f, 0.10f) * kCollisionPreviewBlend;
                } else {
                    item.color = previewColor;
                    item.emissive = Eigen::Vector3f(0.0f, 0.08f, 0.10f);
                }
            } else if(item.originalMaterial) {
                item.color = item.originalMaterial->baseColor;
                item.color.head<3>() *= kCollisionPreviewVisualDarken;
                item.emissive = item.originalMaterial->emissiveColor * 0.35f;
            } else {
                item.color = Eigen::Vector4f(0.8f, 0.8f, 0.8f, 1.0f);
                item.emissive.setZero();
            }
        }
        applySelectionOverrides(m_impl->selectionOverrides);
    };

    if(variantId.empty()) {
        const bool erasedDocumentKey = m_impl->visibleCollisionVariantIds.erase(key) > 0;
        m_impl->visibleCollisionVariantFilters.erase(key);
        m_impl->visibleCollisionVariantFiltersByRuntimeLink.erase(runtimeKey);
        applyPreviewSelectionMaterial(false, false);
        m_impl->invalidateCollisionGeometryOverlayCache();
        return erasedDocumentKey;
    }

    const RobotCollisionLinkSummary summary =
        RobotCollisionModelInspector::summarizeLink(
            *robot,
            linkName,
            std::string(),
            std::string(),
            variantId);
    const auto it = std::find_if(
        summary.variants.begin(),
        summary.variants.end(),
        [&](const RobotCollisionModelVariantSummary& variant) {
            return variant.variantId == variantId;
        });
    if(it == summary.variants.end()) {
        return false;
    }

    VisibleCollisionVariantFilter filter;
    filter.robotId = robotId;
    filter.linkName = linkName;
    filter.variantId = variantId;
    filter.elementNames.insert(it->elementNames.begin(), it->elementNames.end());

    m_impl->visibleCollisionVariantIds[key] = variantId;
    m_impl->visibleCollisionVariantFilters[key] = filter;
    m_impl->visibleCollisionVariantFiltersByRuntimeLink[runtimeKey] = std::move(filter);
    applyPreviewSelectionMaterial(
        true,
        simulation_project::isConvertFromVisualCollisionModelId(variantId));
    m_impl->invalidateCollisionGeometryOverlayCache();
    return true;
}

std::string ProjectScene::visibleRobotCollisionVariant(
    const std::string& robotId,
    const std::string& linkName) const
{
    return m_impl->visibleCollisionVariantId(robotId, linkName);
}

void ProjectScene::setSelectedLink(const std::string& robotId, const std::string& linkName)
{
    m_impl->selectedJointFrameRobotId.clear();
    m_impl->selectedJointFrameName.clear();
    m_impl->selectedObjectFrameObjectId.clear();
    m_impl->selectedObjectFrameId.clear();

    restoreMaterialOverrides(m_impl->pairPreviewOverrides);
    restoreMaterialOverrides(m_impl->selectionOverrides);

    m_impl->selectionState.selectRobotLink(robotId, linkName);

    RuntimeRobot* demo = nullptr;
    for(RuntimeRobot& robot : m_impl->robots) {
        if(robot.documentId == robotId) {
            demo = &robot;
            break;
        }
    }

    if(demo == nullptr || !demo->visualBridge) {
        return;
    }

    const auto addSelectionOverridesForLink = [&](const std::string& targetLinkName) {
        const auto& nodes = demo->visualBridge->visualNodes(targetLinkName);
        for(const auto& node : nodes) {
            if(!node || !node->model()) {
                continue;
            }

            auto model = node->model();
            for(unsigned int i = 0; i < model->subMeshCount(); ++i) {
                auto& subMesh = model->subMesh(i);
                m_impl->selectionOverrides.push_back(MaterialOverride{ model, i, subMesh.material });
            }
        }
    };

    if(linkName.empty()) {
        for(const auto& [targetLinkName, link] : demo->model.links) {
            (void)link;
            addSelectionOverridesForLink(targetLinkName);
        }
    } else {
        addSelectionOverridesForLink(linkName);
    }

    applySelectionOverrides(m_impl->selectionOverrides);
}

void ProjectScene::setSelectedJointFrame(const std::string& robotId, const std::string& jointName)
{
    m_impl->selectedObjectFrameObjectId.clear();
    m_impl->selectedObjectFrameId.clear();
    restoreMaterialOverrides(m_impl->pairPreviewOverrides);
    restoreMaterialOverrides(m_impl->selectionOverrides);
    m_impl->selectionState.clear();

    m_impl->selectedJointFrameRobotId = robotId;
    m_impl->selectedJointFrameName = jointName;
}

void ProjectScene::setSelectedRobotMount(
    const std::string& robotId,
    const std::string& linkName,
    const std::string& robotMountId)
{
    setSelectedLink(robotId, linkName);
    m_impl->selectionState.selectRobotMount(robotId, linkName, robotMountId);
}

void ProjectScene::setSelectedSceneObject(const std::string& objectId)
{
    m_impl->selectedJointFrameRobotId.clear();
    m_impl->selectedJointFrameName.clear();
    m_impl->selectedObjectFrameObjectId.clear();
    m_impl->selectedObjectFrameId.clear();

    restoreMaterialOverrides(m_impl->pairPreviewOverrides);
    restoreMaterialOverrides(m_impl->selectionOverrides);

    m_impl->selectionState.selectSceneObject(objectId);
}

void ProjectScene::setSelectedObjectFrame(
    const std::string& objectId,
    const std::string& frameId)
{
    setSelectedSceneObject(objectId);
    if(findSceneObjectDesc(m_impl->projectDocument, objectId) == nullptr || frameId.empty()) {
        return;
    }
    m_impl->selectedObjectFrameObjectId = objectId;
    m_impl->selectedObjectFrameId = frameId;
}

bool ProjectScene::setSelectedToolAttachment(const std::string& attachmentId)
{
    return setSelectedMountedAttachment(attachmentId);
}

bool ProjectScene::setSelectedMountedAttachment(const std::string& attachmentId)
{
    m_impl->selectedJointFrameRobotId.clear();
    m_impl->selectedJointFrameName.clear();
    m_impl->selectedObjectFrameObjectId.clear();
    m_impl->selectedObjectFrameId.clear();

    restoreMaterialOverrides(m_impl->pairPreviewOverrides);
    restoreMaterialOverrides(m_impl->selectionOverrides);

    m_impl->selectionState.selectMountedAttachment(
        attachmentId,
        m_impl->mountedAttachments.attachmentOwner(attachmentId));

    if(attachmentId.empty()) {
        return true;
    }

    for(std::size_t i = 0; i < m_impl->toolAttachments.size(); ++i) {
        if(m_impl->toolAttachments[i].documentId == attachmentId) {
            if(m_impl->toolAttachments[i].visible) {
                m_impl->activeToolAttachmentIndex = i;
                m_impl->applyActiveToolAttachmentVisibility();
            }
            return true;
        }
    }
    return false;
}

bool ProjectScene::setActivePreviewRobotMount(const std::string& robotMountId)
{
    if(findRobotMountDesc(m_impl->projectDocument, robotMountId) == nullptr && !robotMountId.empty()) {
        return false;
    }
    m_impl->activePreviewRobotMountId = robotMountId;
    return true;
}

bool ProjectScene::setPreviewRobotMountTransform(
    const std::string& robotMountId,
    const simulation_project::TransformDesc& transform)
{
    simulation_project::RobotMountDesc* mount = findRobotMountDesc(m_impl->projectDocument, robotMountId);
    if(mount == nullptr) {
        return false;
    }

    mount->linkToMount = transform;
    return true;
}

bool ProjectScene::setPreviewRobotMountLink(
    const std::string& robotMountId,
    const std::string& linkName)
{
    simulation_project::RobotMountDesc* mount = findRobotMountDesc(m_impl->projectDocument, robotMountId);
    if(mount == nullptr) {
        return false;
    }

    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, mount->robotId);
    if(robot != nullptr && robot->model.links.find(linkName) == robot->model.links.end()) {
        return false;
    }

    mount->linkName = linkName;
    return true;
}

bool ProjectScene::upsertPreviewRobotMount(const simulation_project::RobotMountDesc& mount)
{
    if(mount.id.empty() || mount.robotId.empty() || mount.linkName.empty()) {
        return false;
    }

    auto& mounts = m_impl->projectDocument.robotMounts;
    auto it = std::find_if(
        mounts.begin(),
        mounts.end(),
        [&](const simulation_project::RobotMountDesc& existing) {
            return existing.id == mount.id;
        });
    if(it != mounts.end()) {
        *it = mount;
    } else {
        mounts.push_back(mount);
    }
    m_impl->activePreviewRobotMountId = mount.id;
    return true;
}

bool ProjectScene::removePreviewRobotMount(const std::string& robotMountId)
{
    auto& mounts = m_impl->projectDocument.robotMounts;
    const auto oldSize = mounts.size();
    mounts.erase(
        std::remove_if(
            mounts.begin(),
            mounts.end(),
            [&](const simulation_project::RobotMountDesc& mount) {
                return mount.id == robotMountId;
            }),
        mounts.end());
    if(m_impl->activePreviewRobotMountId == robotMountId) {
        m_impl->activePreviewRobotMountId.clear();
    }
    return mounts.size() != oldSize;
}

void ProjectScene::setRobotMountFrameVisibility(bool selectedLinkFrameVisible, bool mountFrameVisible)
{
    m_impl->previewSelectedLinkFrameVisible = selectedLinkFrameVisible;
    m_impl->previewRobotMountFrameVisible = mountFrameVisible;
}

void ProjectScene::setPinnedRobotMountFrames(const std::vector<std::string>& robotMountIds)
{
    m_impl->pinnedRobotMountFrameIds = robotMountIds;
}

bool ProjectScene::setRobotBaseTransform(
    const std::string& robotId,
    const simulation_project::TransformDesc& transform)
{
    RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        return false;
    }

    const collision::Transform3 baseTransform = ProjectRuntimeBuilder::makeTransform(transform);
    if(robot->parallelControlEnabled) {
        m_impl->setStewartPlatformBaseTransform(*robot, baseTransform);
    } else {
        robot->baseTransform = baseTransform;
        ProjectRuntimeBuilder::updateRobotPose(*robot);
    }
    m_impl->syncProjectToolAttachments();
    return true;
}

bool ProjectScene::setRobotJointValue(
    const std::string& robotId,
    const std::string& jointName,
    double value)
{
    RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr || !ProjectRuntimeBuilder::setJointValue(*robot, jointName, value)) {
        return false;
    }

    m_impl->syncStewartFollowers(*robot);
    if(robot->parallelControlEnabled) {
        solveStewartInternalLegControls(*robot);
    }
    ProjectRuntimeBuilder::updateRobotPose(*robot);
    m_impl->applyStewartInternalPlatformVisualOverrides(*robot);
    for(RuntimeRobot& follower : m_impl->robots) {
        if(follower.parallelFollowerEnabled && sameStewartSource(*robot, follower)) {
            ProjectRuntimeBuilder::updateRobotPose(follower);
        }
    }
    m_impl->applyStewartFollowerVisualOverrides(*robot);
    m_impl->syncProjectToolAttachments();
    return true;
}

bool ProjectScene::robotJointValue(
    const std::string& robotId,
    const std::string& jointName,
    double& value) const
{
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        return false;
    }

    return ProjectRuntimeBuilder::getJointValue(*robot, jointName, value);
}

bool ProjectScene::setRobotAutoMotion(
    const std::string& robotId,
    bool enabled,
    double amplitude,
    double speed)
{
    RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        return false;
    }

    robot->autoMotionEnabled = enabled;
    robot->autoMotionAmplitude = amplitude;
    robot->autoMotionSpeed = speed;
    return true;
}

bool ProjectScene::setSceneObjectTransform(
    const std::string& objectId,
    const simulation_project::TransformDesc& transform)
{
    RuntimeSceneObject* object = findRuntimeObject(m_impl->objects, objectId);
    if(object == nullptr) {
        return false;
    }

    object->transform = ProjectRuntimeBuilder::makeTransform(transform);
    if(object->objectType == "pointCloud") {
        ProjectRuntimeBuilder::updatePointCloudPose(*object);
    } else {
        ProjectRuntimeBuilder::updateSceneObjectPose(*object);
    }
    return true;
}

bool ProjectScene::removeSceneObject(const std::string& objectId)
{
    bool removed = false;
    for(auto it = m_impl->objects.begin(); it != m_impl->objects.end();) {
        if(it->documentId != objectId) {
            ++it;
            continue;
        }

        if(it->visualNode) {
            it->visualNode->setVisible(false);
        }
        if(it->pointCloudNode) {
            it->pointCloudNode->setVisible(false);
        }
        if(it->collisionObject) {
            m_impl->collisionScene.removeEnvironmentObject(it->collisionObject->id());
        }
        for(const RuntimeSceneCollisionObject& collisionObject : it->collisionObjects) {
            if(collisionObject.collisionObject) {
                m_impl->collisionScene.removeEnvironmentObject(collisionObject.collisionObject->id());
            }
        }

        it = m_impl->objects.erase(it);
        removed = true;
    }

    if(!removed) {
        return false;
    }

    m_impl->sceneObjects.erase(
        std::remove_if(
            m_impl->sceneObjects.begin(),
            m_impl->sceneObjects.end(),
            [&](const SceneObjectGroup& object) {
                return object.objectId == objectId;
            }),
        m_impl->sceneObjects.end());
    m_impl->pointClouds.erase(
        std::remove_if(
            m_impl->pointClouds.begin(),
            m_impl->pointClouds.end(),
            [&](const PointCloudGroup& pointCloud) {
                return pointCloud.pointCloudId == objectId;
            }),
        m_impl->pointClouds.end());
    m_impl->projectDocument.objects.erase(
        std::remove_if(
            m_impl->projectDocument.objects.begin(),
            m_impl->projectDocument.objects.end(),
            [&](const simulation_project::SceneObjectDesc& object) {
                return object.id == objectId;
            }),
        m_impl->projectDocument.objects.end());
    m_impl->projectDocument.pointClouds.erase(
        std::remove_if(
            m_impl->projectDocument.pointClouds.begin(),
            m_impl->projectDocument.pointClouds.end(),
            [&](const simulation_project::PointCloudDesc& pointCloud) {
                return pointCloud.id == objectId;
            }),
        m_impl->projectDocument.pointClouds.end());

    m_impl->collisionScene.update();
    m_impl->invalidateCollisionGeometryOverlayCache();
    setSelectedSceneObject(std::string());
    return true;
}

bool ProjectScene::previewSceneObjectTransform(
    const std::string& objectId,
    const simulation_project::TransformDesc& transform)
{
    RuntimeSceneObject* object = findRuntimeObject(m_impl->objects, objectId);
    if(object == nullptr) {
        return false;
    }

    object->transform = ProjectRuntimeBuilder::makeTransform(transform);
    if(object->objectType == "pointCloud") {
        ProjectRuntimeBuilder::updatePointCloudVisualPose(*object);
    } else {
        ProjectRuntimeBuilder::updateSceneObjectVisualPose(*object);
    }
    return true;
}

bool ProjectScene::setPreviewObjectFrameTransform(
    const std::string& objectId,
    const std::string& frameId,
    const simulation_project::TransformDesc& transform)
{
    simulation_project::SceneObjectDesc* object =
        findSceneObjectDesc(m_impl->projectDocument, objectId);
    if(object == nullptr) {
        return false;
    }
    simulation_project::ObjectFrameDesc* frame = findObjectFrameDesc(*object, frameId);
    if(frame == nullptr) {
        return false;
    }
    frame->objectToFrame = transform;
    return true;
}

bool ProjectScene::upsertPreviewObjectFrame(
    const std::string& objectId,
    const simulation_project::ObjectFrameDesc& frame)
{
    simulation_project::SceneObjectDesc* object =
        findSceneObjectDesc(m_impl->projectDocument, objectId);
    if(object == nullptr || frame.id.empty()) {
        return false;
    }

    simulation_project::ObjectFrameDesc* existing = findObjectFrameDesc(*object, frame.id);
    if(existing != nullptr) {
        *existing = frame;
    } else {
        object->objectFrames.push_back(frame);
    }
    return true;
}

bool ProjectScene::setRobotMountTransform(
    const std::string& robotMountId,
    const simulation_project::TransformDesc& transform)
{
    const bool changed =
        m_impl->mountedAttachments.setRobotMountLocalTransform(robotMountId, transform);
    if(changed) {
        m_impl->syncProjectToolAttachments();
    }
    return changed;
}

bool ProjectScene::setToolAttachmentTransform(
    const std::string& attachmentId,
    const simulation_project::TransformDesc& transform)
{
    return setMountedAttachmentTransform(attachmentId, transform);
}

bool ProjectScene::setMountedAttachmentTransform(
    const std::string& attachmentId,
    const simulation_project::TransformDesc& transform)
{
    if(m_impl->mountedAttachments.setAttachmentMountTransform(attachmentId, transform)) {
        m_impl->syncProjectToolAttachments();
        return true;
    }
    return false;
}

bool ProjectScene::setToolAssetMountToVisual(
    const std::string& assetId,
    const simulation_project::TransformDesc& transform)
{
    bool changed = m_impl->mountedAttachments.setAssetVisualTransform(assetId, transform);
    for(RuntimeToolAttachmentVisual& attachment : m_impl->toolAttachments) {
        if(attachment.toolAssetId == assetId) {
            const simulation_runtime::RuntimeMountedAttachment* mountedAttachment =
                m_impl->mountedAttachments.attachment(attachment.documentId);
            if(mountedAttachment == nullptr) {
                continue;
            }
            attachment.assetMountToVisual = mountedAttachment->visual.attachmentMountToVisual;
            robot_render::MountedAttachmentVisualBridge::setAttachmentMountToVisual(
                attachment.visual,
                math::eigenToGlm(attachment.assetMountToVisual));
        }
    }
    if(changed) {
        m_impl->syncProjectToolAttachments();
    }
    return changed;
}

bool ProjectScene::setToolAssetTcp(
    const std::string& assetId,
    const simulation_project::TransformDesc& transform)
{
    const bool changed =
        m_impl->mountedAttachments.setAssetTcpTransform(assetId, transform);
    if(changed) {
        m_impl->syncProjectToolAttachments();
    }
    return changed;
}

bool ProjectScene::setToolAssetPreviewMountToVisual(
    const simulation_project::TransformDesc& transform)
{
    if(!m_impl->toolAssetPreviewSet) {
        return false;
    }

    m_impl->toolAssetPreview.asset.assetMountToVisual = transform;
    m_impl->toolAssetPreview.mountToVisual = ProjectRuntimeBuilder::makeTransform(transform);
    m_impl->updateToolAssetPreviewModel();
    return true;
}

bool ProjectScene::setToolAssetPreviewTcp(
    const simulation_project::TransformDesc& transform)
{
    if(!m_impl->toolAssetPreviewSet) {
        return false;
    }

    bool updated = false;
    for(simulation_project::AttachmentFunctionalFrameDesc& frame : m_impl->toolAssetPreview.asset.functionalFrames) {
        if(frame.primary) {
            frame.assetMountToFrame = transform;
            updated = true;
            break;
        }
    }
    if(!updated) {
        simulation_project::AttachmentFunctionalFrameDesc frame;
        frame.id = m_impl->toolAssetPreview.asset.id + ".tcp";
        frame.name = "TCP";
        frame.frameType = "tcp";
        frame.assetMountToFrame = transform;
        frame.primary = true;
        m_impl->toolAssetPreview.asset.functionalFrames.push_back(frame);
    }
    m_impl->toolAssetPreview.mountToTcp = ProjectRuntimeBuilder::makeTransform(transform);
    return true;
}

bool ProjectScene::cycleToolAttachment(bool reverse)
{
    return cycleMountedAttachment(reverse);
}

bool ProjectScene::cycleMountedAttachment(bool reverse)
{
    return m_impl->cycleActiveToolAttachment(reverse);
}

bool ProjectScene::setActiveToolAttachment(const std::string& id)
{
    return setActiveMountedAttachment(id);
}

bool ProjectScene::setActiveMountedAttachment(const std::string& id)
{
    return m_impl->setActiveToolAttachment(id);
}

void ProjectScene::setActiveToolFrameRobot(const std::string& robotId)
{
    m_impl->setActiveToolFrameRobot(robotId);
}

void ProjectScene::setToolFrameVisibility(const ToolFrameVisibility& visibility)
{
    m_impl->toolFrameVisibility = visibility;
}

std::string ProjectScene::activeToolAttachmentId() const
{
    return activeMountedAttachmentId();
}

std::string ProjectScene::activeMountedAttachmentId() const
{
    if(m_impl->activeToolAttachmentIndex >= m_impl->toolAttachments.size()) {
        return std::string();
    }
    return m_impl->toolAttachments[m_impl->activeToolAttachmentIndex].documentId;
}

std::vector<ProjectScene::ToolAttachmentInfo> ProjectScene::toolAttachments() const
{
    return mountedAttachments();
}

std::vector<ProjectScene::MountedAttachmentInfo> ProjectScene::mountedAttachments() const
{
    std::vector<ProjectScene::MountedAttachmentInfo> infos;
    infos.reserve(m_impl->toolAttachments.size());

    for(std::size_t i = 0; i < m_impl->toolAttachments.size(); ++i) {
        const RuntimeToolAttachmentVisual& runtime = m_impl->toolAttachments[i];
        MountedAttachmentInfo info;
        info.id = runtime.documentId;
        info.name = runtime.name;
        info.attachmentKind = runtime.assetKind;
        info.assetKind = runtime.assetKind;
        info.assetType = runtime.assetType;
        info.functionalFrameType = runtime.functionalFrameType;
        info.robotMountId = runtime.robotMountId;
        info.toolAssetId = runtime.toolAssetId;
        info.mountFrameId = runtime.robotMountId;
        info.assetId = runtime.toolAssetId;
        info.linkName = runtime.linkName;
        info.visualPath = pathToUtf8(runtime.visualPath);
        info.enabled = runtime.enabled;
        info.visible = runtime.visible;
        info.active = i == m_impl->activeToolAttachmentIndex;
        infos.push_back(std::move(info));
    }

    return infos;
}

std::vector<ProjectScene::RobotLinkMaterialInfo> ProjectScene::robotLinkMaterials(
    const std::string& robotId,
    const std::string& linkName) const
{
    std::vector<RobotLinkMaterialInfo> result;

    auto robotIt = std::find_if(
        m_impl->robots.begin(),
        m_impl->robots.end(),
        [&](const RuntimeRobot& robot) {
            return robot.documentId == robotId;
        });
    if(robotIt == m_impl->robots.end()) {
        return result;
    }

    const auto linkIt = robotIt->model.links.find(linkName);
    if(linkIt == robotIt->model.links.end()) {
        return result;
    }

    const std::vector<std::shared_ptr<scenecore::ModelNode>> visualNodes =
        robotIt->visualBridge ? robotIt->visualBridge->visualNodes(linkName)
                              : std::vector<std::shared_ptr<scenecore::ModelNode>>();

    result.reserve(linkIt->second.visuals.size());
    for(std::size_t visualIndex = 0; visualIndex < linkIt->second.visuals.size(); ++visualIndex) {
        const robot::RobotVisual& visual = linkIt->second.visuals[visualIndex];

        RobotLinkMaterialInfo info;
        info.robotId = robotId;
        info.linkName = linkName;
        info.partUid = visual.partUid;
        info.meshPath = visual.meshPath;
        info.source = visual.hasMaterial ? "URDF material" : "Fallback default material";

        Eigen::Vector4f color = visualDiffuseColor(visual);
        if(visualIndex < visualNodes.size() && visualNodes[visualIndex]) {
            const auto model = visualNodes[visualIndex]->model();
            if(model) {
                info.subMeshCount = model->subMeshCount();
                for(std::size_t subMeshIndex = 0; subMeshIndex < model->subMeshCount(); ++subMeshIndex) {
                    const auto& subMesh = model->subMesh(static_cast<unsigned int>(subMeshIndex));
                    if(subMesh.material) {
                        color = subMesh.material->baseColor;
                        if(!visual.hasMaterial) {
                            info.source = isDefaultRobotMaterial(*subMesh.material)
                                ? "Fallback default material"
                                : "Assimp/model material";
                        }
                        break;
                    }
                }
            }
        }

        if(m_impl->selectionState.selectedRobotId() == robotId &&
            m_impl->selectionState.selectedLinkName() == linkName) {
            info.overrideState = "Selection highlight active";
        } else if(robotIt->highlightedLinks.count(linkName) > 0) {
            info.overrideState = "Collision highlight active";
        } else {
            info.overrideState = "None";
        }

        info.r = color.x();
        info.g = color.y();
        info.b = color.z();
        info.a = color.w();
        result.push_back(std::move(info));
    }

    return result;
}

const std::vector<ProjectScene::RobotLinkGroup>& ProjectScene::robotLinks() const
{
    return m_impl->robotLinks;
}

const std::vector<ProjectScene::SceneObjectGroup>& ProjectScene::sceneObjects() const
{
    return m_impl->sceneObjects;
}

const std::vector<ProjectScene::PointCloudGroup>& ProjectScene::pointClouds() const
{
    return m_impl->pointClouds;
}

std::vector<ProjectScene::CollisionDetectorInfo> ProjectScene::collisionDetectors() const
{
    std::vector<CollisionDetectorInfo> result;
    result.reserve(m_impl->collisionDetectors.size());

    for(const ProjectCollisionDetectorRuntime& detector : m_impl->collisionDetectors) {
        std::string firstPairA;
        std::string firstPairB;
        fillCollisionPairSummary(detector.lastResult, firstPairA, firstPairB);

        CollisionDetectorInfo info;
        info.id = detector.id;
        info.name = detector.name;
        info.type = detector.type;
        info.enabled = detector.enabled;
        info.visible = detector.visible;
        info.active = detector.id == m_impl->activeCollisionDetectorId;
        info.includePairCount = detector.options.includePairs.size();
        info.effectiveIncludePairCount = detector.effectiveIncludePairCount;
        info.contactCount = visualizableContactCount(detector.lastResult);
        info.inCollision = detector.lastResult.inCollision();
        info.minDistance = detector.lastResult.minDistance;
        info.lastCheckMs = detector.lastCheckMs;
        info.lastDistanceMs = detector.lastDistanceMs;
        info.lastQueryMs = detector.lastQueryMs;
        info.frameRobotPoseMs = m_impl->lastRobotPoseUpdateMs;
        info.frameCollisionWorldUpdateMs = m_impl->lastCollisionWorldUpdateMs;
        info.frameOverlayMs = m_impl->lastCollisionOverlayMs;
        info.frameOverlayHighlightMs = m_impl->lastCollisionOverlayHighlightMs;
        info.frameOverlayDebugBuildMs = m_impl->lastCollisionOverlayDebugBuildMs;
        info.frameOverlayVariantFilterMs = m_impl->lastCollisionOverlayVariantFilterMs;
        info.frameOverlayDebugSubmitMs = m_impl->lastCollisionOverlayDebugSubmitMs;
        info.frameOverlayAuxFramesMs = m_impl->lastCollisionOverlayAuxFramesMs;
        info.frameOverlayDetectorCount = m_impl->lastCollisionOverlayDetectorCount;
        info.frameOverlayGeometryCount = m_impl->lastCollisionOverlayGeometryCount;
        info.frameOverlayContactCount = m_impl->lastCollisionOverlayContactCount;
        info.frameOverlayNearestCount = m_impl->lastCollisionOverlayNearestCount;
        info.frameOverlayPrimitiveEstimate = m_impl->lastCollisionOverlayPrimitiveEstimate;
        info.frameOverlayLineEstimate = m_impl->lastCollisionOverlayLineEstimate;
        info.firstPairA = firstPairA;
        info.firstPairB = firstPairB;
        info.hasResult = detector.hasResult;
        info.contacts = makeCollisionContactInfo(detector.lastResult);
        info.nearest = makeCollisionNearestInfo(
            detector.lastResult,
            detector.nearestState,
            detector.nearestReason);
        result.push_back(std::move(info));
    }

    return result;
}

bool ProjectScene::collisionQueriesEnabled() const
{
    return m_impl->collisionQueriesEnabled;
}

bool ProjectScene::setCollisionQueriesEnabled(bool enabled)
{
    if(m_impl->collisionQueriesEnabled == enabled) {
        return false;
    }

    if(enabled && !m_impl->ensureCollisionRuntimeBuilt()) {
        LOG_WARNING("rs2026") << "ProjectScene setCollisionQueriesEnabled failed: collision runtime unavailable";
        return false;
    }
    m_impl->collisionQueriesEnabled = enabled;
    for(ProjectCollisionDetectorRuntime& detector : m_impl->collisionDetectors) {
        detector.lastResult.clear();
        detector.hasResult = false;
        detector.effectiveIncludePairCount = 0;
        detector.lastCheckMs = 0.0;
        detector.lastDistanceMs = 0.0;
        detector.lastQueryMs = 0.0;
        detector.lastNearestQueryFrame = 0;
        detector.nearestState = "NotComputed";
        detector.nearestReason.clear();
    }
    m_impl->lastIncludePairCount = static_cast<std::size_t>(-1);
    m_impl->lastContactCount = static_cast<std::size_t>(-1);
    return true;
}

bool ProjectScene::setActiveCollisionDetector(const std::string& id)
{
    for(const ProjectCollisionDetectorRuntime& detector : m_impl->collisionDetectors) {
        if(detector.id == id) {
            m_impl->activeCollisionDetectorId = id;
            m_impl->lastIncludePairCount = static_cast<std::size_t>(-1);
            m_impl->lastContactCount = static_cast<std::size_t>(-1);
            return true;
        }
    }
    return false;
}

bool ProjectScene::setCollisionDetectorEnabled(const std::string& id, bool enabled)
{
    for(ProjectCollisionDetectorRuntime& detector : m_impl->collisionDetectors) {
        if(detector.id == id) {
            detector.enabled = enabled;
            detector.hasResult = false;
            detector.lastResult.clear();
            for(simulation_project::CollisionDetectorDesc& desc : m_impl->projectDocument.collision.detectors) {
                if(desc.id == id) {
                    desc.enabled = enabled;
                    break;
                }
            }
            return true;
        }
    }
    return false;
}

bool ProjectScene::setCollisionDetectorVisible(const std::string& id, bool visible)
{
    for(ProjectCollisionDetectorRuntime& detector : m_impl->collisionDetectors) {
        if(detector.id == id) {
            detector.visible = visible;
            for(simulation_project::CollisionDetectorDesc& desc : m_impl->projectDocument.collision.detectors) {
                if(desc.id == id) {
                    desc.visualization.visible = visible;
                    break;
                }
            }
            return true;
        }
    }
    return false;
}

bool ProjectScene::updateCollisionDetectorRuntimeOptions(const simulation_project::CollisionDetectorDesc& desc)
{
    for(ProjectCollisionDetectorRuntime& detector : m_impl->collisionDetectors) {
        if(detector.id == desc.id) {
            applyCollisionDetectorRuntimeOptions(detector, desc);
            for(simulation_project::CollisionDetectorDesc& projectDesc : m_impl->projectDocument.collision.detectors) {
                if(projectDesc.id == desc.id) {
                    projectDesc = desc;
                    break;
                }
            }
            m_impl->lastIncludePairCount = static_cast<std::size_t>(-1);
            m_impl->lastContactCount = static_cast<std::size_t>(-1);
            return true;
        }
    }
    return false;
}

bool ProjectScene::rebuildCollisionDetectorsFromDocument(const simulation_project::ProjectDocument& document)
{
    m_impl->projectDocument = document;
    const std::string previousActiveId = m_impl->activeCollisionDetectorId;
    m_impl->collisionDetectors = ProjectCollisionDetectorBuilder::build(document, m_impl->robots, m_impl->objects);

    const auto activeIt = std::find_if(
        m_impl->collisionDetectors.begin(),
        m_impl->collisionDetectors.end(),
        [&](const ProjectCollisionDetectorRuntime& detector) {
            return detector.id == previousActiveId;
        });
    m_impl->activeCollisionDetectorId = activeIt != m_impl->collisionDetectors.end()
        ? previousActiveId
        : (m_impl->collisionDetectors.empty() ? std::string() : m_impl->collisionDetectors.front().id);
    m_impl->lastIncludePairCount = static_cast<std::size_t>(-1);
    m_impl->lastContactCount = static_cast<std::size_t>(-1);
    return true;
}

bool ProjectScene::removeCollisionDetector(const std::string& id)
{
    const auto oldSize = m_impl->collisionDetectors.size();
    m_impl->collisionDetectors.erase(
        std::remove_if(
            m_impl->collisionDetectors.begin(),
            m_impl->collisionDetectors.end(),
            [&](const ProjectCollisionDetectorRuntime& detector) {
                return detector.id == id;
            }),
        m_impl->collisionDetectors.end());

    if(m_impl->collisionDetectors.size() == oldSize) {
        return false;
    }

    if(m_impl->activeCollisionDetectorId == id) {
        m_impl->activeCollisionDetectorId = m_impl->collisionDetectors.empty()
            ? std::string()
            : m_impl->collisionDetectors.front().id;
    }
    return true;
}

bool ProjectScene::generateRobotCollisionProxy(
    const std::string& robotId,
    const std::string& linkName,
    const std::string& proxyType,
    simulation_project::CollisionElementOverrideDesc& element) const
{
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        return false;
    }

    return RobotCollisionProxyGenerator::generateFromVisual(*robot, linkName, proxyType, element);
}

bool ProjectScene::generateRobotCollisionProxies(
    const std::string& robotId,
    const std::string& linkName,
    const RobotCollisionProxyRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        elements.clear();
        return false;
    }

    return RobotCollisionProxyGenerator::generateFromVisual(*robot, linkName, request, elements);
}

bool ProjectScene::generateRobotCollisionProxiesFromExistingCollision(
    const std::string& robotId,
    const std::string& linkName,
    const RobotCollisionProxyRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        elements.clear();
        return false;
    }

    return RobotCollisionProxyGenerator::generateFromExistingCollision(*robot, linkName, request, elements);
}

bool ProjectScene::generateRobotCollisionProxiesFromExistingCollision(
    const std::string& robotId,
    const RobotCollisionProxyRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        elements.clear();
        return false;
    }

    elements.clear();
    std::vector<simulation_project::CollisionElementOverrideDesc> linkElements;
    for(const std::string& linkName : robot->model.linkNames) {
        linkElements.clear();
        if(RobotCollisionProxyGenerator::generateFromExistingCollision(*robot, linkName, request, linkElements)) {
            elements.insert(elements.end(), linkElements.begin(), linkElements.end());
        }
    }

    return !elements.empty();
}

bool ProjectScene::generateRobotCollisionCoacdFromVisual(
    const std::string& robotId,
    const std::string& linkName,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    elements.clear();
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        return false;
    }

    const simulation_project::RobotDesc* robotDesc =
        findRobotDesc(m_impl->projectDocument, robotId);
    const std::string sourceKey =
        robotDesc != nullptr && !robotDesc->sourcePath.empty() ? robotDesc->sourcePath : robotId;
    const CollisionGeneratedAssetRequest request = makeGeneratedAssetRequest(
        m_impl->projectDocument,
        m_impl->projectBasePath,
        sourceKey,
        robotId + "_" + linkName);
    return RobotCollisionProxyGenerator::generateCoacdFromVisual(
        *robot,
        linkName,
        request,
        elements);
}

bool ProjectScene::generateRobotCollisionCoacdFromExistingCollision(
    const std::string& robotId,
    const std::string& linkName,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    elements.clear();
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        return false;
    }

    const simulation_project::RobotDesc* robotDesc =
        findRobotDesc(m_impl->projectDocument, robotId);
    const std::string sourceKey =
        robotDesc != nullptr && !robotDesc->sourcePath.empty() ? robotDesc->sourcePath : robotId;
    const CollisionGeneratedAssetRequest request = makeGeneratedAssetRequest(
        m_impl->projectDocument,
        m_impl->projectBasePath,
        sourceKey,
        robotId + "_" + linkName);
    return RobotCollisionProxyGenerator::generateCoacdFromExistingCollision(
        *robot,
        linkName,
        request,
        elements);
}

bool ProjectScene::generateObjectCollisionCoacdFromVisual(
    const std::string& objectId,
    std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements) const
{
    elements.clear();

    const simulation_project::AssetResolveContext assetContext =
        ProjectRuntimeBuilder::makeAssetResolveContext(
            m_impl->projectBasePath,
            m_impl->projectDocument.assetSearchPaths);
    const simulation_project::SceneObjectDesc* objectDesc =
        findSceneObjectDesc(m_impl->projectDocument, objectId);

    const RuntimeSceneObject* object = nullptr;
    RuntimeSceneObject temporaryObject;
    const RuntimeSceneObject* runtimeObject = findRuntimeObject(m_impl->objects, objectId);
    if(runtimeObject != nullptr && hasTriangleMeshCollisionShape(*runtimeObject)) {
        object = runtimeObject;
    }

    std::string sourceKey = objectId;
    if(object == nullptr && objectDesc != nullptr && !objectDesc->sourcePath.empty()) {
        const std::filesystem::path objectPath =
            simulation_project::AssetResolver::resolveProjectPath(
                assetContext,
                objectDesc->sourcePath);
        if(buildTemporaryCoacdInputObjectFromModel(
               objectPath,
               objectDesc->visualScale,
               objectId,
               objectDesc->name,
               collision::Transform3::Identity(),
               temporaryObject)) {
            object = &temporaryObject;
            sourceKey = objectDesc->sourcePath;
        }
    }

    if(object == nullptr) {
        for(const RuntimeToolAttachmentVisual& attachment : m_impl->toolAttachments) {
            if(attachment.documentId != objectId) {
                continue;
            }

            if(attachment.collisionObjectIndex < m_impl->objects.size()) {
                const RuntimeSceneObject& collisionObject =
                    m_impl->objects[attachment.collisionObjectIndex];
                if(hasTriangleMeshCollisionShape(collisionObject)) {
                    object = &collisionObject;
                    sourceKey = pathToUtf8(attachment.visualPath);
                    break;
                }
            }

            if(buildTemporaryCoacdInputObjectFromModel(
                   attachment.visualPath,
                   attachment.visualScale,
                   objectId,
                   attachment.name,
                   attachment.assetMountToVisual,
                   temporaryObject)) {
                temporaryObject.transform = attachment.worldToolMount;
                object = &temporaryObject;
                sourceKey = pathToUtf8(attachment.visualPath);
            }
            break;
        }
    }
    if(object == nullptr) {
        return false;
    }

    const CollisionGeneratedAssetRequest request = makeGeneratedAssetRequest(
        m_impl->projectDocument,
        m_impl->projectBasePath,
        sourceKey,
        objectId);
    return RobotCollisionProxyGenerator::generateObjectCoacdFromVisual(
        *object,
        request,
        elements);
}

bool ProjectScene::evaluateRobotCollisionProxyQuality(
    const std::string& robotId,
    const std::string& linkName,
    const RobotCollisionProxyRequest& request,
    const std::vector<simulation_project::CollisionElementOverrideDesc>& elements,
    bool useExistingCollisionInput,
    RobotCollisionProxyQualitySummary& summary) const
{
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        return false;
    }

    return RobotCollisionProxyGenerator::evaluateGeneratedSphereCover(
        *robot,
        linkName,
        request,
        elements,
        useExistingCollisionInput,
        summary);
}

bool ProjectScene::generateMissingRobotCollisionProxies(
    const std::string& robotId,
    const std::string& proxyType,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        elements.clear();
        return false;
    }

    return RobotCollisionProxyGenerator::generateMissingFromVisual(*robot, proxyType, elements);
}

bool ProjectScene::generateMissingRobotCollisionProxies(
    const std::string& robotId,
    const RobotCollisionProxyRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements) const
{
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        elements.clear();
        return false;
    }

    return RobotCollisionProxyGenerator::generateMissingFromVisual(*robot, request, elements);
}

RobotCollisionRobotSummary ProjectScene::robotCollisionSummary(
    const std::string& robotId,
    const std::string& activeDetectorRole,
    const std::string& activeDetectorSource) const
{
    const RuntimeRobot* robot = findRuntimeRobot(m_impl->robots, robotId);
    if(robot == nullptr) {
        return RobotCollisionRobotSummary();
    }

    RobotCollisionRobotSummary summary =
        RobotCollisionModelInspector::summarizeRobot(*robot, activeDetectorRole, activeDetectorSource);
    for(RobotCollisionLinkSummary& link : summary.links) {
        const std::string visibleVariantId =
            m_impl->visibleCollisionVariantId(robotId, link.linkName);
        if(visibleVariantId.empty()) {
            continue;
        }

        for(RobotCollisionModelVariantSummary& variant : link.variants) {
            variant.visibleInViewport = variant.variantId == visibleVariantId;
            variant.selectedInViewport = variant.visibleInViewport;
        }
    }
    return summary;
}
