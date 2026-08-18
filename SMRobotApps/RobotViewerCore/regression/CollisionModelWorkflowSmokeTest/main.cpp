#include <ProjectRuntimeTypes.h>
#include <ProjectRuntimeBuilder.h>
#include <ProjectCollisionDetectorRuntime.h>
#include <RobotCollisionModelInspector.h>
#include <RobotCollisionOverrideApplier.h>
#include <RobotCollisionProxyGenerator.h>

#include <AssetCore/AssetManager.h>
#include <Collision/CollisionScene.h>
#include <Collision/RobotCollisionInstance.h>
#include <Collision/RobotCollisionModel.h>
#include <RobotIO/RobotCollisionOverrideIo.h>
#include <RobotIO/RobotUrdfCollisionExporter.h>
#include <SimulationProject/CollisionModelSelectionIds.h>
#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectIo.h>
#include <SimulationRuntime/ProjectCollisionRuntime.h>
#include <SimulationRuntime/ProjectSimulationRuntime.h>
#include <data_path.h>

#include <fcl/fcl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#include <glm/glm.hpp>

namespace
{
    struct CheckContext
    {
        int failures = 0;

        void require(bool condition, const std::string& message)
        {
            if(condition) {
                std::cout << "[OK] " << message << "\n";
                return;
            }

            ++failures;
            std::cerr << "[FAIL] " << message << "\n";
        }
    };

    collision::CollisionDebugDrawData build420ToolCollisionDebugData(
        CheckContext& checks,
        const simulation_project::ProjectDocument& document,
        const std::string& label);

    robot::RobotCollisionGeometry makeBoxCollision(
        const std::string& id,
        double x,
        double y,
        double z)
    {
        robot::RobotCollisionGeometry geometry;
        geometry.partUid = id;
        geometry.type = robot::RobotGeometryType::Box;
        geometry.boxSize = Eigen::Vector3d(x, y, z);
        geometry.role = "Exact";
        geometry.enabled = true;
        return geometry;
    }

    RuntimeRobot makeRuntimeRobot()
    {
        RuntimeRobot runtime;
        runtime.documentId = "robot_collision_workflow";
        runtime.name = "Collision Workflow Robot";
        runtime.model.name = runtime.name;

        robot::RobotLink base;
        base.name = "base";
        base.uid = "base";
        base.collisions.push_back(makeBoxCollision("base_original_box", 1.0, 0.08, 0.08));

        robot::RobotLink visualOnly;
        visualOnly.name = "visual_only";
        visualOnly.uid = "visual_only";
        robot::RobotVisual visual;
        visual.partUid = "visual_only_mesh";
        visual.meshPath = (std::filesystem::path(PROJECT_SOURCE_PATH) / "data" / "Tools" / "MyTool01.STL").generic_string();
        visual.meshScale = 1.0f;
        visualOnly.visuals.push_back(std::move(visual));

        runtime.model.linkNames = { base.name, visualOnly.name };
        runtime.model.links.emplace(base.name, std::move(base));
        runtime.model.links.emplace(visualOnly.name, std::move(visualOnly));
        runtime.model.root = "base";
        runtime.model.base_link = "base";
        return runtime;
    }

    RuntimeRobot loadRuntimeRobot(
        const std::filesystem::path& path,
        const std::string& sourceType,
        const std::string& robotId)
    {
        RuntimeRobot runtime;
        runtime.documentId = robotId;
        runtime.name = robotId;
        runtime.model = ProjectRuntimeBuilder::loadSingleRobot(path, sourceType);
        if(runtime.model.linkNames.empty()) {
            runtime.model.linkNames.reserve(runtime.model.links.size());
            for(const auto& link : runtime.model.links) {
                runtime.model.linkNames.push_back(link.first);
            }
        }
        return runtime;
    }

    std::string firstCollisionReadyLink(const RobotCollisionRobotSummary& summary)
    {
        for(const RobotCollisionLinkSummary& link : summary.links) {
            if(link.effectiveCollisionCount > 0) {
                return link.linkName;
            }
        }
        return {};
    }

    simulation_project::ProjectDocument makeProjectDocument(
        const RuntimeRobot& runtime,
        std::vector<simulation_project::CollisionElementOverrideDesc> elements,
        bool replaceOriginal)
    {
        simulation_project::ProjectDocument document;
        document.version = 3;

        simulation_project::RobotDesc robot;
        robot.id = runtime.documentId;
        robot.name = runtime.name;
        robot.sourceType = "urdf";
        robot.sourcePath = "data/drake_models/iiwa_description/urdf/iiwa14_no_collision.urdf";
        document.robots.push_back(std::move(robot));

        simulation_project::RobotCollisionOverrideDesc collisionOverride;
        collisionOverride.robotId = runtime.documentId;
        collisionOverride.replaceOriginalCollisions = replaceOriginal;
        collisionOverride.elements = std::move(elements);
        document.collision.robotOverrides.push_back(std::move(collisionOverride));
        return document;
    }

    bool saveAndReload(
        const simulation_project::ProjectDocument& document,
        simulation_project::ProjectDocument& reloaded)
    {
        const std::filesystem::path path =
            std::filesystem::path(PROJECT_SOURCE_PATH) / "build" / "RobotViewerCoreCollisionModelWorkflowSmokeTest.scene.json";

        std::string errorMessage;
        if(!simulation_project::saveProjectDocumentV3(path, document, &errorMessage)) {
            std::cerr << "Failed to save smoke project: " << errorMessage << "\n";
            return false;
        }
        if(!simulation_project::loadProjectDocument(path, reloaded, &errorMessage)) {
            std::cerr << "Failed to reload smoke project: " << errorMessage << "\n";
            return false;
        }
        return true;
    }

    std::string readTextFile(const std::filesystem::path& path)
    {
        std::ifstream input(path, std::ios::binary);
        std::ostringstream text;
        text << input.rdbuf();
        return text.str();
    }

    bool writeTextFile(const std::filesystem::path& path, const std::string& text)
    {
        std::ofstream output(path, std::ios::binary);
        output.write(text.data(), static_cast<std::streamsize>(text.size()));
        return output.good();
    }

    robotio::Vec3Desc toRobotIoVec3(const simulation_project::Vec3Desc& desc)
    {
        robotio::Vec3Desc result;
        result.x = desc.x;
        result.y = desc.y;
        result.z = desc.z;
        return result;
    }

    simulation_project::Vec3Desc fromRobotIoVec3(const robotio::Vec3Desc& desc)
    {
        simulation_project::Vec3Desc result;
        result.x = desc.x;
        result.y = desc.y;
        result.z = desc.z;
        return result;
    }

    robotio::TransformDesc toRobotIoTransform(const simulation_project::TransformDesc& desc)
    {
        robotio::TransformDesc result;
        result.x = desc.x;
        result.y = desc.y;
        result.z = desc.z;
        result.roll = desc.roll;
        result.pitch = desc.pitch;
        result.yaw = desc.yaw;
        return result;
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

    robotio::CollisionElementOverrideDesc toRobotIoCollisionElement(
        const simulation_project::CollisionElementOverrideDesc& desc)
    {
        robotio::CollisionElementOverrideDesc result;
        result.id = desc.id;
        result.linkName = desc.linkName;
        result.label = desc.label;
        result.type = desc.type;
        result.role = desc.role;
        result.enabled = desc.enabled;
        result.localTransform = toRobotIoTransform(desc.localTransform);
        result.boxSize = toRobotIoVec3(desc.boxSize);
        result.radius = desc.radius;
        result.length = desc.length;
        result.meshPath = desc.meshPath;
        result.meshScale = toRobotIoVec3(desc.meshScale);
        result.inflationMargin = desc.inflationMargin;
        result.source = desc.source;
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

    robotio::RobotCollisionOverrideDesc toRobotIoCollisionOverride(
        const simulation_project::RobotCollisionOverrideDesc& desc)
    {
        robotio::RobotCollisionOverrideDesc result;
        result.robotId = desc.robotId;
        result.sourceRobotPath = desc.sourceRobotPath;
        result.overridePath = desc.overridePath;
        result.replaceOriginalCollisions = desc.replaceOriginalCollisions;
        result.elements.reserve(desc.elements.size());
        for(const simulation_project::CollisionElementOverrideDesc& element : desc.elements) {
            result.elements.push_back(toRobotIoCollisionElement(element));
        }
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

    bool saveAndReloadSidecar(
        const simulation_project::RobotCollisionOverrideDesc& desc,
        simulation_project::RobotCollisionOverrideDesc& reloaded)
    {
        const std::filesystem::path path =
            std::filesystem::path(PROJECT_SOURCE_PATH) / "build" / "RobotViewerCoreCollisionModelWorkflowSmokeTest.collision.override.json";

        std::string errorMessage;
        if(!robotio::saveRobotCollisionOverride(path, toRobotIoCollisionOverride(desc), &errorMessage)) {
            std::cerr << "Failed to save sidecar override: " << errorMessage << "\n";
            return false;
        }
        robotio::RobotCollisionOverrideDesc robotIoReloaded;
        if(!robotio::loadRobotCollisionOverride(path, robotIoReloaded, &errorMessage)) {
            std::cerr << "Failed to reload sidecar override: " << errorMessage << "\n";
            return false;
        }
        reloaded = fromRobotIoCollisionOverride(robotIoReloaded);
        return true;
    }

    bool exportAndInspectUrdf(
        const simulation_project::RobotCollisionOverrideDesc& desc,
        std::string& exportedText)
    {
        const std::filesystem::path buildDir =
            std::filesystem::path(PROJECT_SOURCE_PATH) / "build";
        const std::filesystem::path sourceUrdf =
            buildDir / "RobotViewerCoreCollisionModelWorkflowSmokeTest.source.urdf";
        const std::filesystem::path outputUrdf =
            buildDir / "RobotViewerCoreCollisionModelWorkflowSmokeTest.exported.urdf";

        const std::string sourceText =
            "<robot name=\"collision_export_test\">\n"
            "  <link name=\"base\">\n"
            "    <collision name=\"old_collision\"><geometry><box size=\"9 9 9\"/></geometry></collision>\n"
            "  </link>\n"
            "  <link name=\"visual_only\"/>\n"
            "</robot>\n";
        if(!writeTextFile(sourceUrdf, sourceText)) {
            std::cerr << "Failed to write source URDF fixture.\n";
            return false;
        }

        std::string errorMessage;
        if(!robotio::exportRobotCollisionOverrideToUrdf(
               sourceUrdf,
               outputUrdf,
               toRobotIoCollisionOverride(desc),
               &errorMessage)) {
            std::cerr << "Failed to export URDF: " << errorMessage << "\n";
            return false;
        }

        exportedText = readTextFile(outputUrdf);
        return !exportedText.empty();
    }

    bool hasSphereCover(const RobotCollisionRobotSummary& summary)
    {
        for(const RobotCollisionLinkSummary& link : summary.links) {
            if(link.hasSphereCover) {
                return true;
            }
        }
        return false;
    }

    const RobotCollisionModelVariantSummary* findVariant(
        const RobotCollisionLinkSummary& summary,
        const std::string& source,
        const std::string& role)
    {
        for(const RobotCollisionModelVariantSummary& variant : summary.variants) {
            if(variant.source == source && variant.role == role) {
                return &variant;
            }
        }
        return nullptr;
    }

    std::size_t statCount(
        const std::vector<RobotCollisionStat>& stats,
        const std::string& name)
    {
        for(const RobotCollisionStat& stat : stats) {
            if(stat.name == name) {
                return stat.count;
            }
        }
        return 0;
    }

    bool roleVisibleByDefaultDebugOptions(const std::string& role)
    {
        return role == "Exact" ||
            role == "Simplified" ||
            role == "SafetyMargin" ||
            role == "SphereCover" ||
            role == "PlanningProxy";
    }

    size_t defaultDebugVisibleElementCount(const RobotCollisionLinkSummary& summary)
    {
        size_t count = 0;
        for(const RobotCollisionModelVariantSummary& variant : summary.variants) {
            if(roleVisibleByDefaultDebugOptions(variant.role)) {
                count += variant.elementCount;
            }
        }
        return count;
    }

    void printVariantBaseline(const std::string& label, const RobotCollisionLinkSummary& summary)
    {
        std::cout << "[CollisionVariantBaseline] " << label
                  << " link=" << summary.linkName
                  << " variants=" << summary.variants.size()
                  << " defaultDebugVisibleElements=" << defaultDebugVisibleElementCount(summary)
                  << "\n";

        for(const RobotCollisionModelVariantSummary& variant : summary.variants) {
            std::cout << "[CollisionVariantBaseline] variant"
                      << " id=" << variant.variantId
                      << " label=\"" << variant.label << "\""
                      << " source=" << variant.source
                      << " role=" << variant.role
                      << " elements=" << variant.elementCount
                      << " enabled=" << (variant.enabled ? "true" : "false")
                      << " visibleInViewport="
                      << (variant.visibleInViewport ? "true" : "false")
                      << " activeDetectorRoleMatch="
                      << (variant.usedByActiveDetector ? "true" : "false")
                      << " defaultDebugVisible="
                      << (roleVisibleByDefaultDebugOptions(variant.role) ? "true" : "false")
                      << "\n";
        }
    }

    struct SphereCoverCoverageMetrics
    {
        size_t samplePointCount = 0;
        size_t sphereCount = 0;
        size_t uncoveredPointCount = 0;
        double maxOutsideDistance = 0.0;
        double uncoveredRatio = 0.0;
        bool belowTargetWithUncoveredPoints = false;
    };

    collision::CollisionGeometryPtr makeCollisionBoxGeometry(double x, double y, double z)
    {
        return std::make_shared<collision::CollisionGeometry>(
            std::make_shared<fcl::Boxd>(x, y, z));
    }

    collision::CollisionGeometryPtr makeCollisionSphereGeometry(double radius)
    {
        return std::make_shared<collision::CollisionGeometry>(
            std::make_shared<fcl::Sphered>(radius));
    }

    collision::Transform3 makeCollisionTransform(double x, double y, double z)
    {
        collision::Transform3 transform = collision::Transform3::Identity();
        transform.translation() = collision::Vec3(x, y, z);
        return transform;
    }

    double distancePointAabb(const collision::Vec3& point, const fcl::AABBd& aabb)
    {
        double squaredDistance = 0.0;
        for(int axis = 0; axis < 3; ++axis) {
            if(point[axis] < aabb.min_[axis]) {
                const double delta = aabb.min_[axis] - point[axis];
                squaredDistance += delta * delta;
            } else if(point[axis] > aabb.max_[axis]) {
                const double delta = point[axis] - aabb.max_[axis];
                squaredDistance += delta * delta;
            }
        }
        return std::sqrt(squaredDistance);
    }

    bool pointNearCollisionObjectAabb(
        const collision::Vec3& point,
        const collision::CollisionObjectPtr& object,
        double tolerance)
    {
        if(!object) {
            return false;
        }
        return point.allFinite() && distancePointAabb(point, object->fcl()->getAABB()) <= tolerance;
    }

    collision::CollisionObjectPtr findRuntimeCollisionObject(
        const RuntimeRobot& robot,
        const RuntimeSceneObject& object,
        collision::ObjectID objectId)
    {
        if(robot.collisionInstance) {
            for(const auto& item : robot.collisionInstance->objects()) {
                if(item.second && item.second->id() == objectId) {
                    return item.second;
                }
            }
        }

        for(const RuntimeSceneCollisionObject& runtimeObject : object.collisionObjects) {
            if(runtimeObject.collisionObject && runtimeObject.collisionObject->id() == objectId) {
                return runtimeObject.collisionObject;
            }
        }

        if(object.collisionObject && object.collisionObject->id() == objectId) {
            return object.collisionObject;
        }

        return nullptr;
    }

    collision::CollisionShapeDesc makeCollisionBoxShape(
        double x,
        double y,
        double z,
        collision::CollisionGeometryRole role)
    {
        collision::CollisionShapeDesc shape;
        shape.type = collision::CollisionShapeType::Box;
        shape.boxSize = collision::Vec3(x, y, z);
        shape.role = role;
        return shape;
    }

    collision::CollisionShapeDesc makeCollisionSphereShape(
        double radius,
        collision::CollisionGeometryRole role)
    {
        collision::CollisionShapeDesc shape;
        shape.type = collision::CollisionShapeType::Sphere;
        shape.radius = radius;
        shape.role = role;
        return shape;
    }

    void filterDebugGeometryToVisibleElements(
        collision::CollisionDebugDrawData& data,
        int robotInstance,
        const std::string& linkName,
        const std::vector<std::string>& visibleElementNames)
    {
        std::vector<collision::CollisionDebugDrawDesc> filtered;
        filtered.reserve(data.geometry.size());
        for(const collision::CollisionDebugDrawDesc& desc : data.geometry) {
            if(desc.robotInstance != robotInstance || desc.linkName != linkName) {
                filtered.push_back(desc);
                continue;
            }

            if(std::find(
                   visibleElementNames.begin(),
                   visibleElementNames.end(),
                   desc.elementName) != visibleElementNames.end()) {
                filtered.push_back(desc);
            }
        }
        data.geometry = std::move(filtered);
    }

    size_t countRobotDebugGeometry(
        const collision::CollisionDebugDrawData& data,
        int robotInstance,
        const std::string& linkName)
    {
        size_t count = 0;
        for(const collision::CollisionDebugDrawDesc& desc : data.geometry) {
            if(desc.robotInstance == robotInstance && desc.linkName == linkName) {
                ++count;
            }
        }
        return count;
    }

    bool hasHighlightedVisibleSphereCover(
        const collision::CollisionDebugDrawData& data,
        int robotInstance,
        const std::string& linkName)
    {
        for(const collision::CollisionDebugDrawDesc& desc : data.geometry) {
            if(desc.robotInstance == robotInstance &&
                desc.linkName == linkName &&
                desc.role == collision::CollisionGeometryRole::SphereCover &&
                desc.highlighted) {
                return true;
            }
        }
        return false;
    }

    std::shared_ptr<collision::RobotCollisionModel> makeOverlayRegressionRobotModel(
        std::vector<std::string>& sphereElementNames)
    {
        auto model = std::make_shared<collision::RobotCollisionModel>();

        collision::CollisionShapeDesc exactShape =
            makeCollisionBoxShape(2.0, 0.2, 0.2, collision::CollisionGeometryRole::Exact);
        model->addLinkShape(
            "Link6",
            "Link6_original_box",
            exactShape,
            makeCollisionBoxGeometry(2.0, 0.2, 0.2));

        const double radius = 0.18;
        const double centers[] = { -0.75, -0.25, 0.25, 0.75 };
        for(int i = 0; i < 4; ++i) {
            const std::string name = "Link6_spherecover_collisionproxy_proxy_" + std::to_string(i + 1);
            collision::CollisionShapeDesc sphereShape =
                makeCollisionSphereShape(radius, collision::CollisionGeometryRole::SphereCover);
            sphereShape.localTransform = makeCollisionTransform(centers[i], 0.0, 0.0);
            collision::RobotCollisionElement element;
            element.linkName = "Link6";
            element.elementName = name;
            element.geometry = makeCollisionSphereGeometry(radius);
            element.shapeDesc = sphereShape;
            element.localTransform = sphereShape.localTransform;
            element.role = collision::CollisionGeometryRole::SphereCover;
            model->addLinkElement(element);
            sphereElementNames.push_back(name);
        }

        return model;
    }

    void addOverlayRegressionEnvironment(
        collision::CollisionScene& scene,
        double x,
        const std::string& name)
    {
        auto object = std::make_shared<collision::CollisionObject>(
            7000 + static_cast<collision::ObjectID>(std::abs(x) * 1000.0),
            makeCollisionBoxGeometry(0.1, 0.1, 0.1));
        object->setTransform(makeCollisionTransform(x, 0.0, 0.0));

        collision::EnvironmentCollisionObjectInfo info;
        info.info.robotInstance = -1;
        info.info.linkName = "environment";
        info.info.elementName = name;
        info.info.geometryRole = collision::CollisionGeometryRole::SphereCover;
        info.shape = makeCollisionBoxShape(0.1, 0.1, 0.1, collision::CollisionGeometryRole::SphereCover);
        scene.addEnvironmentObject(object, info);
    }

    RuntimeRobot makeDetectorRebuildRuntimeRobot(
        const std::shared_ptr<collision::RobotCollisionModel>& model,
        uint64_t runtimeId)
    {
        RuntimeRobot robot;
        robot.runtimeId = runtimeId;
        robot.documentId = "detector_rebuild_robot";
        robot.name = "Detector Rebuild Robot";
        robot.collisionModel = model;
        robot.collisionInstance = std::make_shared<collision::RobotCollisionInstance>(runtimeId, model);
        robot.collisionInstance->setLinkTransform("Link6", collision::Transform3::Identity());

        robot::RobotLink link;
        link.name = "Link6";
        robot.model.linkNames = { "Link6" };
        robot.model.links.emplace("Link6", std::move(link));
        return robot;
    }

    std::shared_ptr<collision::RobotCollisionModel> makeDetectorRebuildExactModel()
    {
        auto model = std::make_shared<collision::RobotCollisionModel>();
        const collision::CollisionShapeDesc exactShape =
            makeCollisionBoxShape(0.6, 0.2, 0.2, collision::CollisionGeometryRole::Exact);
        model->addLinkShape(
            "Link6",
            "Link6_original_box",
            exactShape,
            makeCollisionBoxGeometry(0.6, 0.2, 0.2));
        return model;
    }

    std::shared_ptr<collision::RobotCollisionModel> makeDetectorRebuildSphereCoverModel(
        std::vector<std::string>& elementNames)
    {
        auto model = std::make_shared<collision::RobotCollisionModel>();
        elementNames.clear();
        const std::array<double, 2> centers = { -0.18, 0.18 };
        for(std::size_t i = 0; i < centers.size(); ++i) {
            collision::CollisionShapeDesc shape =
                makeCollisionSphereShape(0.14, collision::CollisionGeometryRole::SphereCover);
            shape.localTransform = makeCollisionTransform(centers[i], 0.0, 0.0);
            const std::string elementName =
                "Link6_spherecover_collisionproxy_proxy_" + std::to_string(i + 1);
            model->addLinkShape(
                "Link6",
                elementName,
                shape,
                makeCollisionSphereGeometry(0.14));
            elementNames.push_back(elementName);
        }
        return model;
    }

    RuntimeSceneObject makeDetectorRebuildEnvironmentObject(
        double exactX = 0.28,
        double sphereCoverX = 0.24)
    {
        RuntimeSceneObject object;
        object.runtimeId = 9001;
        object.documentId = "detector_rebuild_fixture";
        object.name = "Detector Rebuild Fixture";
        object.collisionEnabled = true;

        RuntimeSceneCollisionObject exactRuntime;
        exactRuntime.collisionShape =
            makeCollisionBoxShape(0.12, 0.12, 0.12, collision::CollisionGeometryRole::Exact);
        exactRuntime.collisionShape.localTransform = makeCollisionTransform(exactX, 0.0, 0.0);
        exactRuntime.collisionObject = std::make_shared<collision::CollisionObject>(
            9101,
            makeCollisionBoxGeometry(0.12, 0.12, 0.12));
        object.collisionObjects.push_back(exactRuntime);

        RuntimeSceneCollisionObject sphereRuntime;
        sphereRuntime.collisionShape =
            makeCollisionBoxShape(0.12, 0.12, 0.12, collision::CollisionGeometryRole::SphereCover);
        sphereRuntime.collisionShape.localTransform = makeCollisionTransform(sphereCoverX, 0.0, 0.0);
        sphereRuntime.collisionObject = std::make_shared<collision::CollisionObject>(
            9102,
            makeCollisionBoxGeometry(0.12, 0.12, 0.12));
        object.collisionObjects.push_back(sphereRuntime);

        object.collisionObject = object.collisionObjects.front().collisionObject;
        object.collisionShape = object.collisionObjects.front().collisionShape;
        return object;
    }

    simulation_project::ProjectDocument makeDetectorRebuildDocument(const std::string& role)
    {
        simulation_project::ProjectDocument document;
        simulation_project::RobotDesc robotDesc;
        robotDesc.id = "detector_rebuild_robot";
        robotDesc.name = "Detector Rebuild Robot";
        document.robots.push_back(std::move(robotDesc));

        simulation_project::SceneObjectDesc objectDesc;
        objectDesc.id = "detector_rebuild_fixture";
        objectDesc.name = "Detector Rebuild Fixture";
        objectDesc.collisionEnabled = true;
        document.objects.push_back(std::move(objectDesc));

        simulation_project::CollisionDetectorDesc detector;
        detector.id = "detector_rebuild_detector";
        detector.name = "Detector Rebuild Detector";
        detector.type = "SelectedObjects";
        detector.geometryRole = role;
        detector.queryPolicy = "BetweenSets";
        detector.selectionSetAId = "detector_rebuild_tool";
        detector.selectionSetBId = "detector_rebuild_part";
        detector.contacts = true;
        detector.nearestPoints = true;
        detector.distance = true;
        detector.maxContacts = 8;
        detector.visualization.showContacts = true;
        detector.visualization.showNearestPoints = true;
        detector.visualization.showObjectHighlight = true;

        simulation_project::CollisionDetectorTargetDesc robotTarget;
        robotTarget.robotId = "detector_rebuild_robot";
        robotTarget.includeLinks = { "Link6" };
        detector.targets.push_back(std::move(robotTarget));

        simulation_project::CollisionDetectorTargetDesc objectTarget;
        objectTarget.objectId = "detector_rebuild_fixture";
        detector.targets.push_back(std::move(objectTarget));

        document.collision.detectors.push_back(std::move(detector));

        simulation_project::CollisionSelectionSetDesc toolSet;
        toolSet.id = "detector_rebuild_tool";
        toolSet.name = "Detector Rebuild Tool";
        simulation_project::CollisionSelectionSetMemberDesc toolMember;
        toolMember.robotId = "detector_rebuild_robot";
        toolMember.linkName = "Link6";
        toolSet.members.push_back(std::move(toolMember));
        document.collision.selectionSets.push_back(std::move(toolSet));

        simulation_project::CollisionSelectionSetDesc partSet;
        partSet.id = "detector_rebuild_part";
        partSet.name = "Detector Rebuild Part";
        simulation_project::CollisionSelectionSetMemberDesc partMember;
        partMember.objectId = "detector_rebuild_fixture";
        partSet.members.push_back(std::move(partMember));
        document.collision.selectionSets.push_back(std::move(partSet));

        return document;
    }

    void addDetectorRebuildEnvironmentToScene(
        collision::CollisionScene& scene,
        const RuntimeSceneObject& object,
        collision::CollisionGeometryRole role)
    {
        for(const RuntimeSceneCollisionObject& runtimeObject : object.collisionObjects) {
            if(!runtimeObject.collisionObject || runtimeObject.collisionShape.role != role) {
                continue;
            }

            runtimeObject.collisionObject->setTransform(runtimeObject.collisionShape.localTransform);
            collision::EnvironmentCollisionObjectInfo info;
            info.info.object = runtimeObject.collisionObject->id();
            info.info.geometryRole = role;
            info.info.elementName = role == collision::CollisionGeometryRole::SphereCover
                ? "fixture_spherecover_box"
                : "fixture_exact_box";
            info.shape = runtimeObject.collisionShape;
            scene.addEnvironmentObject(runtimeObject.collisionObject, info);
        }
    }

    std::vector<collision::ObjectID> robotObjectIdsFromPairs(
        const collision::CollisionQueryOptions& options,
        const RuntimeRobot& robot)
    {
        std::vector<collision::ObjectID> ids;
        for(const auto& pair : options.includePairs) {
            if(robot.collisionInstance && robot.collisionInstance->getLinkInfo(pair.first) != nullptr) {
                ids.push_back(pair.first);
            }
            if(robot.collisionInstance && robot.collisionInstance->getLinkInfo(pair.second) != nullptr) {
                ids.push_back(pair.second);
            }
        }
        std::sort(ids.begin(), ids.end());
        ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
        return ids;
    }

    struct SphereCoverShapeMetrics
    {
        double mainAxisLength = 0.0;
        double maxSphereRadius = 0.0;
        double maxRadiusToMainAxisLength = 0.0;
    };

    Eigen::Vector3d transformModelPoint(
        const glm::mat4& localTransform,
        const Eigen::Vector3f& position,
        const Eigen::Isometry3d& visualTransform)
    {
        const glm::vec4 corrected = localTransform * glm::vec4(
            position.x(),
            position.y(),
            position.z(),
            1.0f);
        return visualTransform * Eigen::Vector3d(
            static_cast<double>(corrected.x),
            static_cast<double>(corrected.y),
            static_cast<double>(corrected.z));
    }

    void appendTriangleSamples(
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b,
        const Eigen::Vector3d& c,
        std::vector<Eigen::Vector3d>& points)
    {
        points.push_back((a + b) * 0.5);
        points.push_back((b + c) * 0.5);
        points.push_back((c + a) * 0.5);
        points.push_back((a + b + c) / 3.0);
    }

    std::vector<Eigen::Vector3d> collectVisualSamplePoints(const robot::RobotLink& link)
    {
        std::vector<Eigen::Vector3d> points;
        for(const robot::RobotVisual& visual : link.visuals) {
            if(visual.meshPath.empty()) {
                continue;
            }

            auto modelDesc = assetcore::AssetManager::instance().loadModel(
                visual.meshPath,
                static_cast<float>(visual.meshScale));
            if(!modelDesc) {
                std::cerr << "[SphereCoverBaseline] Failed to load visual mesh: "
                          << visual.meshPath << "\n";
                continue;
            }

            const glm::mat4 localTransform = modelDesc->get_local();
            for(const auto& subMesh : modelDesc->subMeshes()) {
                std::vector<Eigen::Vector3d> subMeshPoints;
                subMeshPoints.reserve(subMesh.geometry.positions.size());
                for(const auto& position : subMesh.geometry.positions) {
                    subMeshPoints.push_back(transformModelPoint(
                        localTransform,
                        position,
                        visual.T_part));
                }

                points.insert(points.end(), subMeshPoints.begin(), subMeshPoints.end());

                const std::vector<uint32_t>& indices = subMesh.geometry.indices;
                for(size_t i = 0; i + 2 < indices.size(); i += 3) {
                    const uint32_t ia = indices[i];
                    const uint32_t ib = indices[i + 1];
                    const uint32_t ic = indices[i + 2];
                    if(ia >= subMeshPoints.size() || ib >= subMeshPoints.size() || ic >= subMeshPoints.size()) {
                        continue;
                    }

                    appendTriangleSamples(
                        subMeshPoints[ia],
                        subMeshPoints[ib],
                        subMeshPoints[ic],
                        points);
                }
            }
        }
        return points;
    }

    SphereCoverShapeMetrics computeSphereCoverShapeMetrics(
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<simulation_project::CollisionElementOverrideDesc>& spheres)
    {
        SphereCoverShapeMetrics metrics;
        if(!points.empty()) {
            Eigen::Vector3d minPoint = points.front();
            Eigen::Vector3d maxPoint = points.front();
            for(const Eigen::Vector3d& point : points) {
                minPoint = minPoint.cwiseMin(point);
                maxPoint = maxPoint.cwiseMax(point);
            }

            const Eigen::Vector3d extent = maxPoint - minPoint;
            metrics.mainAxisLength = std::max(extent.x(), std::max(extent.y(), extent.z()));
        }

        for(const simulation_project::CollisionElementOverrideDesc& sphere : spheres) {
            if(sphere.type == "sphere") {
                metrics.maxSphereRadius = std::max(metrics.maxSphereRadius, sphere.radius);
            }
        }

        if(metrics.mainAxisLength > 0.0) {
            metrics.maxRadiusToMainAxisLength = metrics.maxSphereRadius / metrics.mainAxisLength;
        }
        return metrics;
    }

    SphereCoverCoverageMetrics computeSphereCoverCoverage(
        const std::vector<Eigen::Vector3d>& points,
        const std::vector<simulation_project::CollisionElementOverrideDesc>& spheres,
        int requestedMaxSpheres)
    {
        SphereCoverCoverageMetrics metrics;
        metrics.samplePointCount = points.size();
        metrics.sphereCount = spheres.size();

        if(points.empty() || spheres.empty()) {
            metrics.uncoveredPointCount = points.size();
            metrics.uncoveredRatio = points.empty() ? 0.0 : 1.0;
            metrics.belowTargetWithUncoveredPoints =
                metrics.sphereCount < static_cast<size_t>(std::max(requestedMaxSpheres, 0)) &&
                metrics.uncoveredPointCount > 0;
            return metrics;
        }

        constexpr double uncoveredEpsilon = 1.0e-5;
        for(const Eigen::Vector3d& point : points) {
            double nearestOutsideDistance = std::numeric_limits<double>::max();
            for(const simulation_project::CollisionElementOverrideDesc& sphere : spheres) {
                if(sphere.type != "sphere" || sphere.radius <= 0.0) {
                    continue;
                }

                const Eigen::Vector3d center(
                    sphere.localTransform.x,
                    sphere.localTransform.y,
                    sphere.localTransform.z);
                const double outsideDistance = (point - center).norm() - sphere.radius;
                nearestOutsideDistance = std::min(nearestOutsideDistance, outsideDistance);
            }

            if(nearestOutsideDistance == std::numeric_limits<double>::max()) {
                nearestOutsideDistance = 0.0;
            }

            const double clampedOutsideDistance = std::max(0.0, nearestOutsideDistance);
            metrics.maxOutsideDistance = std::max(metrics.maxOutsideDistance, clampedOutsideDistance);
            if(clampedOutsideDistance > uncoveredEpsilon) {
                ++metrics.uncoveredPointCount;
            }
        }

        metrics.uncoveredRatio = points.empty()
            ? 0.0
            : static_cast<double>(metrics.uncoveredPointCount) / static_cast<double>(points.size());
        metrics.belowTargetWithUncoveredPoints =
            metrics.sphereCount < static_cast<size_t>(std::max(requestedMaxSpheres, 0)) &&
            metrics.uncoveredPointCount > 0;
        return metrics;
    }

    void printSphereCoverBaseline(
        const std::string& label,
        int requestedMaxSpheres,
        const std::vector<simulation_project::CollisionElementOverrideDesc>& spheres,
        const SphereCoverCoverageMetrics& metrics,
        const SphereCoverShapeMetrics& shapeMetrics)
    {
        std::cout << "[SphereCoverBaseline] " << label
                  << " maxSpheres=" << requestedMaxSpheres
                  << " generated=" << spheres.size()
                  << " samples=" << metrics.samplePointCount
                  << " uncovered=" << metrics.uncoveredPointCount
                  << " uncoveredRatio=" << std::fixed << std::setprecision(6) << metrics.uncoveredRatio
                  << " maxOutsideDistance=" << metrics.maxOutsideDistance
                  << " maxSphereRadius=" << shapeMetrics.maxSphereRadius
                  << " mainAxisLength=" << shapeMetrics.mainAxisLength
                  << " maxRadiusToMainAxisLength=" << shapeMetrics.maxRadiusToMainAxisLength
                  << " belowTargetWithUncovered="
                  << (metrics.belowTargetWithUncoveredPoints ? "true" : "false")
                  << "\n";

        for(size_t i = 0; i < spheres.size(); ++i) {
            const simulation_project::CollisionElementOverrideDesc& sphere = spheres[i];
            std::cout << "[SphereCoverBaseline] sphere[" << i << "]"
                      << " id=" << sphere.id
                      << " center=(" << sphere.localTransform.x
                      << ", " << sphere.localTransform.y
                      << ", " << sphere.localTransform.z
                      << ") radius=" << sphere.radius
                      << " role=" << sphere.role
                      << " source=" << sphere.source
                      << "\n";
        }
        std::cout << std::defaultfloat;
    }

    void verifyDefaultProjectLink6SphereCoverBaseline(CheckContext& checks)
    {
        const std::filesystem::path root = std::filesystem::path(PROJECT_SOURCE_PATH);
        const std::filesystem::path defaultProjectPath =
            root / "config" / "projects" / "420.v3.scene.json";

        simulation_project::ProjectDocument document;
        std::string errorMessage;
        checks.require(
            simulation_project::loadProjectDocument(defaultProjectPath, document, &errorMessage),
            "default project loads for Link6 sphere-cover baseline");
        checks.require(!document.robots.empty(), "default project contains a robot");
        if(document.robots.empty()) {
            return;
        }

        const simulation_project::RobotDesc& robotDesc = document.robots.front();
        RuntimeRobot runtime = loadRuntimeRobot(
            root / robotDesc.sourcePath,
            robotDesc.sourceType,
            robotDesc.id.empty() ? std::string("default_project_robot") : robotDesc.id);

        const auto linkIt = runtime.model.links.find("Link6");
        checks.require(linkIt != runtime.model.links.end(), "default project robot contains Link6");
        if(linkIt == runtime.model.links.end()) {
            return;
        }

        checks.require(!linkIt->second.visuals.empty(), "Link6 has visual mesh input for baseline");

        RobotCollisionProxyRequest request;
        request.proxyType = "sphereCover";
        request.role = "SphereCover";
        request.maxSphereCount = 16;

        std::vector<simulation_project::CollisionElementOverrideDesc> spheres;
        checks.require(
            RobotCollisionProxyGenerator::generateFromVisual(runtime, "Link6", request, spheres),
            "Link6 visual mesh can generate sphere cover baseline");
        checks.require(!spheres.empty(), "Link6 sphere cover baseline emits spheres");
        checks.require(
            spheres.size() <= static_cast<size_t>(request.maxSphereCount),
            "Link6 sphere cover respects max sphere count");

        const std::vector<Eigen::Vector3d> samplePoints = collectVisualSamplePoints(linkIt->second);
        const SphereCoverCoverageMetrics metrics =
            computeSphereCoverCoverage(samplePoints, spheres, request.maxSphereCount);
        const SphereCoverShapeMetrics shapeMetrics =
            computeSphereCoverShapeMetrics(samplePoints, spheres);
        printSphereCoverBaseline(
            "default-project Link6 visual sphere cover",
            request.maxSphereCount,
            spheres,
            metrics,
            shapeMetrics);

        checks.require(metrics.samplePointCount > 0, "Link6 coverage baseline samples visual mesh points");

        RobotCollisionProxyQualitySummary qualitySummary;
        checks.require(
            RobotCollisionProxyGenerator::evaluateGeneratedSphereCover(
                runtime,
                "Link6",
                request,
                spheres,
                false,
                qualitySummary),
            "Link6 sphere cover quality summary can be evaluated");
        checks.require(
            qualitySummary.generatedSphereCount == static_cast<int>(spheres.size()),
            "Link6 quality summary reports generated sphere count");
        checks.require(
            qualitySummary.uncoveredPointCount == 0,
            "Link6 quality summary reports full input coverage");
        checks.require(
            qualitySummary.maxSphereRadius > 0.0 && qualitySummary.mainAxisLength > 0.0,
            "Link6 quality summary reports radius and main axis metrics");
        checks.require(
            qualitySummary.oversizedSphereCount == 0,
            "Link6 improved sphere cover has no oversized sphere warning");
    }

    void verifyLinkVariantSummary(CheckContext& checks)
    {
        RuntimeRobot runtime = makeRuntimeRobot();

        simulation_project::CollisionElementOverrideDesc visualProxy;
        checks.require(
            RobotCollisionProxyGenerator::generateFromVisual(runtime, "visual_only", "box", visualProxy),
            "variant summary fixture can generate visual proxy");

        RobotCollisionProxyRequest sphereRequest;
        sphereRequest.proxyType = "sphereCover";
        sphereRequest.role = "SphereCover";
        sphereRequest.maxSphereCount = 4;
        std::vector<simulation_project::CollisionElementOverrideDesc> sphereCoverElements;
        checks.require(
            RobotCollisionProxyGenerator::generateFromExistingCollision(
                runtime,
                "base",
                sphereRequest,
                sphereCoverElements),
            "variant summary fixture can generate sphere cover");

        auto visualLinkIt = runtime.model.links.find("visual_only");
        if(visualLinkIt != runtime.model.links.end()) {
            robot::RobotCollisionGeometry proxyGeometry;
            proxyGeometry.partUid = visualProxy.id;
            proxyGeometry.type = robot::RobotGeometryType::Box;
            proxyGeometry.boxSize = Eigen::Vector3d(
                visualProxy.boxSize.x,
                visualProxy.boxSize.y,
                visualProxy.boxSize.z);
            proxyGeometry.role = visualProxy.role;
            proxyGeometry.source = visualProxy.source;
            visualLinkIt->second.collisions.push_back(proxyGeometry);
        }

        auto baseLinkIt = runtime.model.links.find("base");
        if(baseLinkIt != runtime.model.links.end()) {
            for(const simulation_project::CollisionElementOverrideDesc& element : sphereCoverElements) {
                robot::RobotCollisionGeometry sphere;
                sphere.partUid = element.id;
                sphere.type = robot::RobotGeometryType::Sphere;
                sphere.radius = element.radius;
                sphere.role = element.role;
                sphere.source = element.source;
                sphere.T_part.translation() = Eigen::Vector3d(
                    element.localTransform.x,
                    element.localTransform.y,
                    element.localTransform.z);
                baseLinkIt->second.collisions.push_back(sphere);
            }
        }

        const RobotCollisionLinkSummary baseSummary =
            RobotCollisionModelInspector::summarizeLink(runtime, "base", "SphereCover");
        printVariantBaseline("base with original and generated sphere cover", baseSummary);

        const RobotCollisionModelVariantSummary* originalVariant =
            findVariant(baseSummary, "original", "Exact");
        const RobotCollisionModelVariantSummary* sphereCoverVariant =
            findVariant(baseSummary, "collisionProxy", "SphereCover");

        checks.require(originalVariant != nullptr, "variant summary lists original collision model");
        checks.require(sphereCoverVariant != nullptr, "variant summary lists collision sphere cover model");
        checks.require(
            sphereCoverVariant != nullptr && sphereCoverVariant->elementCount == sphereCoverElements.size(),
            "variant summary reports sphere cover element count");
        checks.require(
            sphereCoverVariant != nullptr && sphereCoverVariant->usedByActiveDetector,
            "variant summary marks active detector role match");
        checks.require(
            sphereCoverVariant != nullptr &&
                sphereCoverVariant->variantId.find(runtime.documentId + "|base|collisionProxy|SphereCover|") == 0,
            "variant summary ids include robot link source role and element signature");
        checks.require(
            sphereCoverVariant != nullptr &&
                sphereCoverVariant->elementNames.size() == sphereCoverElements.size(),
            "variant summary records element names for viewport filtering");
        checks.require(
            defaultDebugVisibleElementCount(baseSummary) ==
                baseSummary.originalCollisionCount + baseSummary.overrideCollisionCount,
            "variant baseline records that default debug visibility includes all base variants");

        auto coreModel = std::make_shared<collision::RobotCollisionModel>();
        collision::CollisionShapeDesc coreExactShape =
            makeCollisionBoxShape(1.0, 0.08, 0.08, collision::CollisionGeometryRole::Exact);
        coreModel->addLinkShape(
            "base",
            "base_original_box",
            coreExactShape,
            makeCollisionBoxGeometry(1.0, 0.08, 0.08));
        for(const simulation_project::CollisionElementOverrideDesc& element : sphereCoverElements) {
            collision::CollisionShapeDesc coreSphereShape =
                makeCollisionSphereShape(element.radius, collision::CollisionGeometryRole::SphereCover);
            coreSphereShape.source = element.source;
            coreSphereShape.localTransform.translation() = collision::Vec3(
                element.localTransform.x,
                element.localTransform.y,
                element.localTransform.z);
            coreModel->addLinkShape(
                "base",
                element.id,
                coreSphereShape,
                makeCollisionSphereGeometry(element.radius));
        }

        collision::RobotCollisionVariantKey sphereKey;
        sphereKey.linkName = "base";
        sphereKey.role = collision::CollisionGeometryRole::SphereCover;
        sphereKey.source = "collisionProxy";
        checks.require(coreModel->hasVariant(sphereKey), "core model has collisionProxy sphere cover variant");
        checks.require(
            coreModel->elementsByVariant(sphereKey).size() == sphereCoverElements.size(),
            "core model returns elements by collision variant");

        collision::RobotCollisionInstance coreInstance(9100, coreModel);
        const std::vector<collision::ObjectID> sphereObjectIds = coreInstance.getObjectIDs(sphereKey);
        checks.require(
            sphereObjectIds.size() == sphereCoverElements.size(),
            "core instance resolves object ids by collision variant");
        if(!sphereObjectIds.empty()) {
            const collision::LinkInfo* info = coreInstance.getLinkInfo(sphereObjectIds.front());
            checks.require(
                info != nullptr &&
                    info->geometryRole == collision::CollisionGeometryRole::SphereCover &&
                    info->source == "collisionProxy",
                "core link info preserves collision variant source");
        }

        const RobotCollisionLinkSummary visibleSphereSummary =
            RobotCollisionModelInspector::summarizeLink(
                runtime,
                "base",
                "SphereCover",
                std::string(),
                sphereCoverVariant != nullptr ? sphereCoverVariant->variantId : std::string());
        printVariantBaseline("base with sphere cover selected for viewport", visibleSphereSummary);
        const RobotCollisionModelVariantSummary* visibleOriginalVariant =
            findVariant(visibleSphereSummary, "original", "Exact");
        const RobotCollisionModelVariantSummary* visibleSphereCoverVariant =
            findVariant(visibleSphereSummary, "collisionProxy", "SphereCover");
        checks.require(
            visibleOriginalVariant != nullptr && !visibleOriginalVariant->visibleInViewport,
            "variant visibility leaves unselected original model hidden");
        checks.require(
            visibleSphereCoverVariant != nullptr && visibleSphereCoverVariant->visibleInViewport,
            "variant visibility marks selected sphere cover model visible");

        std::vector<simulation_project::CollisionElementOverrideDesc> oversizedSphere;
        simulation_project::CollisionElementOverrideDesc oversizedElement;
        oversizedElement.id = "base_bad_sphere_cover";
        oversizedElement.linkName = "base";
        oversizedElement.type = "sphere";
        oversizedElement.role = "SphereCover";
        oversizedElement.source = "collisionProxy";
        oversizedElement.radius = 10.0;
        oversizedSphere.push_back(oversizedElement);

        RuntimeRobot oversizedRuntime = makeRuntimeRobot();
        RobotCollisionProxyQualitySummary oversizedQuality;
        checks.require(
            RobotCollisionProxyGenerator::evaluateGeneratedSphereCover(
                oversizedRuntime,
                "base",
                sphereRequest,
                oversizedSphere,
                true,
                oversizedQuality),
            "oversized sphere cover quality can be evaluated");
        checks.require(
            oversizedQuality.oversizedSphereCount > 0 && oversizedQuality.hasWarning,
            "oversized sphere cover quality reports warning");

        const RobotCollisionLinkSummary visualSummary =
            RobotCollisionModelInspector::summarizeLink(runtime, "visual_only", "PlanningProxy");
        printVariantBaseline("visual-only link with visual proxy", visualSummary);

        const RobotCollisionModelVariantSummary* visualProxyVariant =
            findVariant(visualSummary, "visualProxy", "PlanningProxy");
        checks.require(visualProxyVariant != nullptr, "variant summary lists visual proxy model");
        checks.require(
            visualProxyVariant != nullptr && visualProxyVariant->usedByActiveDetector,
            "variant summary marks visual proxy active role match");
    }

    void verifyCollisionResultOverlayBaseline(CheckContext& checks)
    {
        std::vector<std::string> visibleSphereElements;
        std::shared_ptr<collision::RobotCollisionModel> model =
            makeOverlayRegressionRobotModel(visibleSphereElements);
        auto robot = std::make_shared<collision::RobotCollisionInstance>(6100, model);
        robot->setLinkTransform("Link6", makeCollisionTransform(0.0, 0.0, 0.0));

        collision::CollisionQueryOptions queryOptions;
        queryOptions.geometryRole = collision::CollisionGeometryRole::SphereCover;
        queryOptions.enableDistance = true;
        queryOptions.enableNearestPoints = true;
        queryOptions.enableContacts = true;

        collision::CollisionVisualizationOptions visualizationOptions;
        visualizationOptions.showAllCollisionGeometry = true;
        visualizationOptions.showSphereCover = true;
        visualizationOptions.showExactGeometry = true;
        visualizationOptions.showNearestPoints = true;
        visualizationOptions.showContacts = true;
        visualizationOptions.showObjectHighlight = true;

        collision::CollisionScene nearScene;
        nearScene.addRobot(robot);
        addOverlayRegressionEnvironment(nearScene, 1.05, "near_box");
        nearScene.update();

        collision::CollisionResult nearestResult;
        nearScene.distance(queryOptions, nearestResult);
        checks.require(nearestResult.hasNearestPoints, "overlay baseline has nearest point result");
        checks.require(
            (nearestResult.nearestPointA - nearestResult.nearestPointB).norm() > 1.0e-6,
            "overlay baseline nearest line has two distinct endpoints");

        collision::CollisionDebugDrawData nearestDebug =
            nearScene.buildDebugDraw(visualizationOptions, &nearestResult);
        const size_t nearestGeometryBeforeFilter = nearestDebug.geometry.size();
        const size_t nearestContactCountBeforeFilter = nearestDebug.contacts.size();
        const size_t nearestPointCountBeforeFilter = nearestDebug.nearestPoints.size();
        filterDebugGeometryToVisibleElements(nearestDebug, 6100, "Link6", visibleSphereElements);
        std::cout << "[OverlayRegressionBaseline] nearest geometry before="
                  << nearestGeometryBeforeFilter
                  << " after=" << nearestDebug.geometry.size()
                  << " robotVisibleAfter="
                  << countRobotDebugGeometry(nearestDebug, 6100, "Link6")
                  << " contactsBefore=" << nearestContactCountBeforeFilter
                  << " contactsAfter=" << nearestDebug.contacts.size()
                  << " nearestBefore=" << nearestPointCountBeforeFilter
                  << " nearestAfter=" << nearestDebug.nearestPoints.size()
                  << "\n";

        checks.require(
            countRobotDebugGeometry(nearestDebug, 6100, "Link6") == visibleSphereElements.size(),
            "visible variant filtering keeps only sphere cover geometry");
        checks.require(!nearestDebug.nearestPoints.empty(), "visible variant filtering keeps nearest line");
        const collision::CollisionNearestPointDebugDrawDesc& nearestOverlay =
            nearestDebug.nearestPoints.front();
        checks.require(
            (nearestOverlay.pointA - nearestOverlay.pointB).norm() > 1.0e-6,
            "visible variant filtering keeps nearest line endpoints");
        checks.require(nearestOverlay.pointRadius > 0.0, "nearest overlay endpoint radius is positive");
        checks.require(
            std::isfinite(nearestOverlay.pointA.x()) &&
                std::isfinite(nearestOverlay.pointA.y()) &&
                std::isfinite(nearestOverlay.pointA.z()) &&
                std::isfinite(nearestOverlay.pointB.x()) &&
                std::isfinite(nearestOverlay.pointB.y()) &&
                std::isfinite(nearestOverlay.pointB.z()),
            "nearest overlay endpoints are finite");
        checks.require(
            nearestOverlay.objectA != 0 && nearestOverlay.objectB != 0,
            "nearest overlay keeps both endpoint objects");
        checks.require(
            nearestOverlay.infoA.geometryRole == collision::CollisionGeometryRole::SphereCover ||
                nearestOverlay.infoB.geometryRole == collision::CollisionGeometryRole::SphereCover,
            "nearest overlay keeps sphere cover object metadata");
        checks.require(
            !nearestOverlay.infoA.elementName.empty() &&
                !nearestOverlay.infoB.elementName.empty(),
            "nearest overlay keeps endpoint element metadata");

        collision::CollisionScene contactScene;
        contactScene.addRobot(robot);
        addOverlayRegressionEnvironment(contactScene, 0.92, "contact_box");
        contactScene.update();

        collision::CollisionResult contactResult;
        contactScene.checkCollision(queryOptions, contactResult);
        checks.require(contactResult.inCollision(), "overlay baseline has sphere cover collision");
        checks.require(!contactResult.contacts.empty(), "overlay baseline has contact result");

        collision::CollisionDebugDrawData contactDebug =
            contactScene.buildDebugDraw(visualizationOptions, &contactResult);
        const size_t contactGeometryBeforeFilter = contactDebug.geometry.size();
        const size_t contactCountBeforeFilter = contactDebug.contacts.size();
        filterDebugGeometryToVisibleElements(contactDebug, 6100, "Link6", visibleSphereElements);
        std::cout << "[OverlayRegressionBaseline] contact geometry before="
                  << contactGeometryBeforeFilter
                  << " after=" << contactDebug.geometry.size()
                  << " robotVisibleAfter="
                  << countRobotDebugGeometry(contactDebug, 6100, "Link6")
                  << " contactsBefore=" << contactCountBeforeFilter
                  << " contactsAfter=" << contactDebug.contacts.size()
                  << "\n";

        checks.require(!contactDebug.contacts.empty(), "visible variant filtering keeps contact points");
        checks.require(
            contactDebug.nearestPoints.empty(),
            "colliding sphere cover contact overlay does not draw nearest line");
        checks.require(
            contactDebug.contacts.front().infoA.geometryRole == collision::CollisionGeometryRole::SphereCover ||
                contactDebug.contacts.front().infoB.geometryRole == collision::CollisionGeometryRole::SphereCover,
            "contact overlay keeps sphere cover object metadata");
        checks.require(
            !contactDebug.contacts.front().infoA.elementName.empty() ||
                !contactDebug.contacts.front().infoB.elementName.empty(),
            "contact overlay keeps endpoint element metadata");
        checks.require(
            hasHighlightedVisibleSphereCover(contactDebug, 6100, "Link6"),
            "visible sphere cover geometry remains highlighted when colliding");
    }

    void verifyDetectorRebuildBaseline(CheckContext& checks)
    {
        RuntimeSceneObject fixture = makeDetectorRebuildEnvironmentObject();

        RuntimeRobot exactRobot =
            makeDetectorRebuildRuntimeRobot(makeDetectorRebuildExactModel(), 6200);
        std::vector<RuntimeRobot> exactRobots = { exactRobot };
        std::vector<RuntimeSceneObject> objects = { fixture };

        simulation_project::ProjectDocument exactDocument = makeDetectorRebuildDocument("Exact");
        std::vector<ProjectCollisionDetectorRuntime> exactDetectors =
            ProjectCollisionDetectorBuilder::build(exactDocument, exactRobots, objects);
        checks.require(exactDetectors.size() == 1, "detector rebuild baseline builds initial detector");
        checks.require(
            !exactDetectors.empty() &&
                exactDetectors.front().options.geometryRole == collision::CollisionGeometryRole::Exact,
            "detector rebuild baseline initial detector uses Exact role");
        checks.require(
            !exactDetectors.empty() && !exactDetectors.front().options.includePairs.empty(),
            "detector rebuild baseline initial detector has include pairs");

        const std::vector<collision::ObjectID> exactRobotIds =
            exactDetectors.empty()
                ? std::vector<collision::ObjectID>()
                : robotObjectIdsFromPairs(exactDetectors.front().options, exactRobot);
        checks.require(exactRobotIds.size() == 1, "initial detector references one exact robot object");
        if(!exactRobotIds.empty()) {
            const collision::LinkInfo* info = exactRobot.collisionInstance->getLinkInfo(exactRobotIds.front());
            checks.require(
                info != nullptr && info->geometryRole == collision::CollisionGeometryRole::Exact,
                "initial detector pair object metadata is Exact");
        }

        collision::CollisionScene exactScene;
        exactScene.addRobot(exactRobot.collisionInstance);
        addDetectorRebuildEnvironmentToScene(exactScene, fixture, collision::CollisionGeometryRole::Exact);
        exactScene.update();
        collision::CollisionResult exactResult;
        if(!exactDetectors.empty()) {
            exactScene.checkCollision(exactDetectors.front().options, exactResult);
        }
        checks.require(exactResult.inCollision(), "initial exact detector can report contact");

        std::vector<std::string> sphereElementNames;
        RuntimeRobot sphereRobot = makeDetectorRebuildRuntimeRobot(
            makeDetectorRebuildSphereCoverModel(sphereElementNames),
            6200);
        std::vector<RuntimeRobot> sphereRobots = { sphereRobot };
        simulation_project::ProjectDocument sphereDocument = makeDetectorRebuildDocument("SphereCover");
        std::vector<ProjectCollisionDetectorRuntime> sphereDetectors =
            ProjectCollisionDetectorBuilder::build(sphereDocument, sphereRobots, objects);
        checks.require(sphereDetectors.size() == 1, "detector rebuild baseline rebuilds detector");
        checks.require(
            !sphereDetectors.empty() &&
                sphereDetectors.front().options.geometryRole == collision::CollisionGeometryRole::SphereCover,
            "rebuilt detector uses SphereCover role");
        checks.require(
            !sphereDetectors.empty() &&
                !sphereDetectors.front().options.filterEnvironmentByGeometryRole,
            "rebuilt detector leaves environment objects on their own collision role");
        checks.require(
            !sphereDetectors.empty() && !sphereDetectors.front().options.includePairs.empty(),
            "rebuilt detector has include pairs");

        const std::vector<collision::ObjectID> sphereRobotIds =
            sphereDetectors.empty()
                ? std::vector<collision::ObjectID>()
                : robotObjectIdsFromPairs(sphereDetectors.front().options, sphereRobot);
        checks.require(
            sphereRobotIds.size() == sphereElementNames.size(),
            "rebuilt detector references generated sphere cover robot objects");
        for(collision::ObjectID objectId : sphereRobotIds) {
            checks.require(
                std::find(exactRobotIds.begin(), exactRobotIds.end(), objectId) == exactRobotIds.end(),
                "rebuilt detector does not keep stale exact robot object id");
            const collision::LinkInfo* info = sphereRobot.collisionInstance->getLinkInfo(objectId);
            checks.require(
                info != nullptr && info->geometryRole == collision::CollisionGeometryRole::SphereCover,
                "rebuilt detector pair object metadata is SphereCover");
        }

        collision::CollisionScene sphereScene;
        sphereScene.addRobot(sphereRobot.collisionInstance);
        addDetectorRebuildEnvironmentToScene(sphereScene, fixture, collision::CollisionGeometryRole::Exact);
        sphereScene.update();
        collision::CollisionResult sphereResult;
        if(!sphereDetectors.empty()) {
            sphereScene.checkCollision(sphereDetectors.front().options, sphereResult);
        }
        checks.require(sphereResult.inCollision(), "rebuilt sphere cover detector can report contact with exact environment");
        checks.require(!sphereResult.contacts.empty(), "rebuilt sphere cover detector returns contacts against exact environment");
        if(!sphereResult.contacts.empty()) {
            checks.require(
                sphereResult.contacts.front().infoA.geometryRole == collision::CollisionGeometryRole::SphereCover ||
                    sphereResult.contacts.front().infoB.geometryRole == collision::CollisionGeometryRole::SphereCover,
                "rebuilt detector contact metadata comes from SphereCover model");
            checks.require(
                sphereResult.contacts.front().infoA.geometryRole == collision::CollisionGeometryRole::Exact ||
                    sphereResult.contacts.front().infoB.geometryRole == collision::CollisionGeometryRole::Exact,
                "rebuilt detector contact metadata keeps exact environment model");
        }

        collision::CollisionResult sphereDistanceResult;
        if(!sphereDetectors.empty()) {
            sphereScene.distance(sphereDetectors.front().options, sphereDistanceResult);
        }
        checks.require(
            !sphereDistanceResult.hasNearestPoints,
            "rebuilt sphere cover detector suppresses unreliable nearest points while colliding");
        checks.require(
            sphereDistanceResult.message.find("suppressed") != std::string::npos,
            "colliding sphere cover nearest result explains suppression");

        RuntimeSceneObject separatedFixture = makeDetectorRebuildEnvironmentObject(0.62, 0.62);
        collision::CollisionScene separatedSphereScene;
        separatedSphereScene.addRobot(sphereRobot.collisionInstance);
        addDetectorRebuildEnvironmentToScene(
            separatedSphereScene,
            separatedFixture,
            collision::CollisionGeometryRole::Exact);
        separatedSphereScene.update();
        collision::CollisionResult separatedSphereDistanceResult;
        if(!sphereDetectors.empty()) {
            separatedSphereScene.distance(sphereDetectors.front().options, separatedSphereDistanceResult);
        }
        checks.require(
            separatedSphereDistanceResult.hasNearestPoints,
            "rebuilt sphere cover detector returns nearest points against separated exact environment");
        checks.require(
            separatedSphereDistanceResult.nearestInfoA.geometryRole == collision::CollisionGeometryRole::Exact ||
                separatedSphereDistanceResult.nearestInfoB.geometryRole == collision::CollisionGeometryRole::Exact,
            "rebuilt detector nearest metadata keeps exact environment model");
        const collision::CollisionObjectPtr nearestObjectA =
            findRuntimeCollisionObject(sphereRobot, separatedFixture, separatedSphereDistanceResult.nearestObjectA);
        const collision::CollisionObjectPtr nearestObjectB =
            findRuntimeCollisionObject(sphereRobot, separatedFixture, separatedSphereDistanceResult.nearestObjectB);
        checks.require(
            pointNearCollisionObjectAabb(separatedSphereDistanceResult.nearestPointA, nearestObjectA, 1.0e-4),
            "rebuilt detector nearest point A is near its world collision object");
        checks.require(
            pointNearCollisionObjectAabb(separatedSphereDistanceResult.nearestPointB, nearestObjectB, 1.0e-4),
            "rebuilt detector nearest point B is near its world collision object");

        collision::CollisionResult staleResult;
        sphereScene.checkCollision(exactDetectors.front().options, staleResult);
        checks.require(
            !staleResult.inCollision(),
            "stale exact detector options do not detect after collision model replacement");
    }

    void verifyRealInputWorkflows(CheckContext& checks)
    {
        const std::filesystem::path root = std::filesystem::path(PROJECT_SOURCE_PATH);

        RuntimeRobot panda = loadRuntimeRobot(
            root / "data" / "drake_models" / "franka_description" / "urdf" / "panda_arm_hand.urdf",
            "urdf",
            "panda_arm_hand");
        const RobotCollisionLinkSummary pandaLink2Summary =
            RobotCollisionModelInspector::summarizeLink(panda, "panda_link2");
        const RobotCollisionModelVariantSummary* pandaVisualVariant = findVariant(
            pandaLink2Summary,
            simulation_project::kConvertFromVisualCollisionSource,
            "Exact");
        const RobotCollisionModelVariantSummary* pandaCollisionVariant =
            findVariant(pandaLink2Summary, "original", "Exact");
        checks.require(
            pandaVisualVariant != nullptr,
            "Panda link2 exposes Convert from Visual variant");
        checks.require(
            pandaCollisionVariant != nullptr,
            "Panda link2 exposes robot collision model variant");
        checks.require(
            pandaCollisionVariant != nullptr &&
                pandaCollisionVariant->elementCount == 6 &&
                pandaCollisionVariant->primitiveCount == 6 &&
                statCount(pandaCollisionVariant->geometryTypes, "Sphere") == 6,
            "Panda link2 robot collision model keeps six spheres");
        checks.require(
            pandaCollisionVariant != nullptr &&
                std::unordered_set<std::string>(
                    pandaCollisionVariant->elementNames.begin(),
                    pandaCollisionVariant->elementNames.end()).size() == 6,
            "Panda link2 sphere ids remain distinct for viewport filtering");
        const auto pandaLink2It = panda.model.links.find("panda_link2");
        const bool pandaLink2UsesUrdfSphereRadii =
            pandaLink2It != panda.model.links.end() &&
            pandaLink2It->second.collisions.size() == 6 &&
            std::all_of(
                pandaLink2It->second.collisions.begin(),
                pandaLink2It->second.collisions.end(),
                [](const robot::RobotCollisionGeometry& geometry) {
                    return geometry.type == robot::RobotGeometryType::Sphere &&
                        std::abs(geometry.radius - 0.06) < 1.0e-9;
                });
        checks.require(
            pandaLink2UsesUrdfSphereRadii,
            "Panda link2 preview keeps all six URDF sphere radii");

        RuntimeRobot simscape = loadRuntimeRobot(
            root / "data" / "Simscape" / "quadrotor" / "scu_quadrotor.xml",
            "simscape",
            "simscape_quadrotor");
        const RobotCollisionRobotSummary simscapeSummary =
            RobotCollisionModelInspector::summarizeRobot(simscape);
        checks.require(simscapeSummary.linkCount > 0, "Simscape robot loads links");
        checks.require(simscapeSummary.visualOnlyLinkCount > 0, "Simscape robot exposes visual-only links");

        RobotCollisionProxyRequest visualBoxRequest;
        visualBoxRequest.proxyType = "box";
        visualBoxRequest.role = "PlanningProxy";
        std::vector<simulation_project::CollisionElementOverrideDesc> simscapeBoxes;
        checks.require(
            RobotCollisionProxyGenerator::generateMissingFromVisual(simscape, visualBoxRequest, simscapeBoxes),
            "Simscape visual-only links can generate box proxies");
        checks.require(!simscapeBoxes.empty(), "Simscape box proxy generation emits elements");

        simulation_project::ProjectDocument simscapeProject =
            makeProjectDocument(simscape, simscapeBoxes, false);
        simscapeProject.robots.front().sourceType = "simscape";
        simscapeProject.robots.front().sourcePath = "data/Simscape/quadrotor/scu_quadrotor.xml";
        simulation_project::ProjectDocument reloadedSimscapeProject;
        checks.require(
            saveAndReload(simscapeProject, reloadedSimscapeProject) &&
                !reloadedSimscapeProject.collision.robotOverrides.empty() &&
                reloadedSimscapeProject.collision.robotOverrides.front().elements.size() == simscapeBoxes.size(),
            "Simscape generated proxies survive project save and reload");

        RuntimeRobot urdfNoCollision = loadRuntimeRobot(
            root / "data" / "drake_models" / "iiwa_description" / "urdf" / "iiwa14_no_collision.urdf",
            "urdf",
            "iiwa_no_collision");
        const RobotCollisionRobotSummary noCollisionSummary =
            RobotCollisionModelInspector::summarizeRobot(urdfNoCollision);
        checks.require(noCollisionSummary.visualOnlyLinkCount > 0, "URDF without collision reports visual-only links");

        std::vector<simulation_project::CollisionElementOverrideDesc> missingUrdfElements;
        checks.require(
            RobotCollisionProxyGenerator::generateMissingFromVisual(urdfNoCollision, visualBoxRequest, missingUrdfElements),
            "URDF without collision can generate missing proxies");
        checks.require(!missingUrdfElements.empty(), "URDF missing proxy generation emits elements");

        RuntimeRobot urdfWithCollision = loadRuntimeRobot(
            root / "data" / "drake_models" / "iiwa_description" / "urdf" / "iiwa14_polytope_collision.urdf",
            "urdf",
            "iiwa_polytope_collision");
        const RobotCollisionRobotSummary collisionSummary =
            RobotCollisionModelInspector::summarizeRobot(urdfWithCollision);
        const std::string collisionLink = firstCollisionReadyLink(collisionSummary);
        checks.require(!collisionLink.empty(), "URDF with mesh collision reports collision-ready links");

        RobotCollisionProxyRequest realSphereCoverRequest;
        realSphereCoverRequest.proxyType = "sphereCover";
        realSphereCoverRequest.role = "SphereCover";
        realSphereCoverRequest.maxSphereCount = 8;
        std::vector<simulation_project::CollisionElementOverrideDesc> realSphereCover;
        checks.require(
            !collisionLink.empty() &&
                RobotCollisionProxyGenerator::generateFromExistingCollision(
                    urdfWithCollision,
                    collisionLink,
                    realSphereCoverRequest,
                    realSphereCover),
            "URDF mesh collision can convert to sphere cover");
        checks.require(!realSphereCover.empty(), "URDF sphere cover conversion emits spheres");
        bool allRealCoverSpheres = true;
        for(const simulation_project::CollisionElementOverrideDesc& element : realSphereCover) {
            allRealCoverSpheres = allRealCoverSpheres && element.type == "sphere" && element.role == "SphereCover";
        }
        checks.require(allRealCoverSpheres, "URDF sphere cover conversion stores SphereCover sphere elements");
    }

    void verifyDefaultUrdfMeshCollisionUsesTriangleMesh(CheckContext& checks)
    {
        const std::filesystem::path root = std::filesystem::path(PROJECT_SOURCE_PATH);

        simulation_project::ProjectDocument document;
        document.version = 3;

        simulation_project::RobotDesc robot;
        robot.id = "abb4600_default_collision";
        robot.name = "ABB4600 Default Collision";
        robot.sourceType = "urdf";
        robot.sourcePath = "data/Spray420/ABB4600_urdf/urdf/ABB4600_urdf.urdf";
        robot.collisionEnabled = true;
        document.robots.push_back(std::move(robot));

        simulation_runtime::ProjectSimulationRuntime simulation;
        const simulation_runtime::Result loadResult = simulation.loadProject(document, root);
        checks.require(loadResult.success, "ABB4600 default project loads for collision runtime");
        if(!loadResult.success || simulation.robots().empty()) {
            return;
        }

        std::unordered_set<std::string> meshElementNames;
        for(const auto& linkItem : simulation.robots().front().model.links) {
            for(const robot::RobotCollisionGeometry& geometry : linkItem.second.collisions) {
                if(geometry.enabled && geometry.type == robot::RobotGeometryType::Mesh) {
                    meshElementNames.insert(geometry.partUid);
                }
            }
        }
        checks.require(!meshElementNames.empty(), "ABB4600 URDF contains mesh collision elements");
        if(meshElementNames.empty()) {
            return;
        }

        simulation_runtime::ProjectCollisionRuntime collisionRuntime;
        const simulation_runtime::Result buildResult = collisionRuntime.build(simulation);
        checks.require(buildResult.success, "ABB4600 collision runtime builds");
        if(!buildResult.success) {
            return;
        }

        const simulation_runtime::Result checkResult = collisionRuntime.check();
        checks.require(checkResult.success, "ABB4600 default mesh collision runtime check does not throw or fail");

        const collision::CollisionDebugDrawData debugData = collisionRuntime.buildDebugDraw(true);
        std::size_t meshElementsDrawnAsTriangleMesh = 0;
        std::size_t meshElementsDrawnAsBox = 0;
        for(const collision::CollisionDebugDrawDesc& desc : debugData.geometry) {
            if(desc.robotInstance < 0 ||
                desc.role != collision::CollisionGeometryRole::Exact ||
                meshElementNames.count(desc.elementName) == 0) {
                continue;
            }

            if(desc.shape.type == collision::CollisionShapeType::TriangleMesh) {
                ++meshElementsDrawnAsTriangleMesh;
            } else if(desc.shape.type == collision::CollisionShapeType::Box) {
                ++meshElementsDrawnAsBox;
            }
        }

        checks.require(
            meshElementsDrawnAsTriangleMesh > 0,
            "ABB4600 default mesh collision stays TriangleMesh in runtime debug data");
        checks.require(
            meshElementsDrawnAsBox == 0,
            "ABB4600 default mesh collision is not converted to runtime box proxy");
    }

    void verifyConvertFromVisualLazyRuntime(CheckContext& checks)
    {
        const std::filesystem::path root = std::filesystem::path(PROJECT_SOURCE_PATH);
        const std::string sourcePath = "data/drake_models/iiwa_description/urdf/iiwa14_no_collision.urdf";
        const robot::RobotModel model =
            ProjectRuntimeBuilder::loadSingleRobot(root / sourcePath, "urdf");

        std::string visualLinkName;
        for(const std::string& linkName : model.linkNames) {
            const auto linkIt = model.links.find(linkName);
            if(linkIt != model.links.end() &&
                !linkIt->second.visuals.empty() &&
                linkIt->second.collisions.empty()) {
                visualLinkName = linkName;
                break;
            }
        }
        checks.require(!visualLinkName.empty(), "visual fallback fixture has a visual-only robot link");
        if(visualLinkName.empty()) {
            return;
        }

        simulation_project::ProjectDocument document;
        document.version = 3;

        simulation_project::RobotDesc robot;
        robot.id = "visual_fallback_robot";
        robot.name = "Visual Fallback Robot";
        robot.sourceType = "urdf";
        robot.sourcePath = sourcePath;
        robot.collisionEnabled = true;
        document.robots.push_back(std::move(robot));

        simulation_project::RobotLinkCollisionModelSelectionDesc selection;
        selection.robotId = "visual_fallback_robot";
        selection.linkName = visualLinkName;
        selection.activeModelId = simulation_project::kConvertFromVisualCollisionModelId;
        document.collision.robotLinkModelSelections.push_back(std::move(selection));

        simulation_project::CollisionDetectorDesc detector;
        detector.id = "visual_fallback_detector";
        detector.name = "Visual Fallback Detector";
        detector.type = "SelectedObjects";
        detector.enabled = true;
        simulation_project::CollisionDetectorTargetDesc target;
        target.robotId = "visual_fallback_robot";
        target.includeLinks = { visualLinkName };
        detector.targets.push_back(std::move(target));
        document.collision.detectors.push_back(std::move(detector));

        simulation_runtime::ProjectSimulationRuntime simulation;
        const simulation_runtime::Result loadResult = simulation.loadProject(document, root);
        checks.require(loadResult.success, "visual fallback project loads for collision runtime");
        if(!loadResult.success) {
            return;
        }

        simulation_runtime::ProjectCollisionRuntime collisionRuntime;
        const simulation_runtime::Result buildResult = collisionRuntime.build(simulation);
        checks.require(buildResult.success, "visual fallback collision runtime builds");
        if(!buildResult.success) {
            return;
        }

        const collision::CollisionDebugDrawData debugData = collisionRuntime.buildDebugDraw(true);
        std::size_t visualFallbackMeshes = 0;
        std::size_t otherLinkMeshes = 0;
        for(const collision::CollisionDebugDrawDesc& desc : debugData.geometry) {
            if(desc.robotInstance < 0) {
                continue;
            }
            if(desc.linkName == visualLinkName &&
                desc.shape.source == simulation_project::kConvertFromVisualCollisionSource &&
                desc.shape.type == collision::CollisionShapeType::TriangleMesh) {
                ++visualFallbackMeshes;
            } else if(desc.linkName != visualLinkName) {
                ++otherLinkMeshes;
            }
        }

        checks.require(
            visualFallbackMeshes > 0,
            "Convert from Visual builds a runtime triangle mesh only when the selected link is referenced");
        checks.require(
            otherLinkMeshes == 0,
            "Convert from Visual detector does not eagerly build unrelated robot links");
    }

    void verifyDefault420ExactDetectorRuntime(CheckContext& checks)
    {
        const std::filesystem::path root = std::filesystem::path(PROJECT_SOURCE_PATH);
        const std::filesystem::path defaultProjectPath =
            root / "config" / "projects" / "420.v3.scene.json";

        simulation_project::ProjectDocument document;
        std::string errorMessage;
        checks.require(
            simulation_project::loadProjectDocument(defaultProjectPath, document, &errorMessage),
            "420 default project loads for exact collision runtime");
        if(document.robots.empty() || document.objects.empty() || document.collision.detectors.empty()) {
            checks.require(false, "420 default project contains robot, object, and detector");
            return;
        }

        simulation_runtime::ProjectSimulationRuntime simulation;
        const simulation_runtime::Result loadResult = simulation.loadProject(document, root);
        checks.require(loadResult.success, "420 default project builds simulation runtime");
        if(!loadResult.success) {
            return;
        }

        simulation_runtime::ProjectCollisionRuntime collisionRuntime;
        const simulation_runtime::Result buildResult = collisionRuntime.build(simulation);
        checks.require(buildResult.success, "420 default project builds collision runtime");
        if(!buildResult.success) {
            return;
        }

        const simulation_runtime::ProjectCollisionDetectorRuntime* detector = collisionRuntime.activeDetector();
        checks.require(detector != nullptr, "420 default project has active collision detector");
        const std::size_t effectivePairCount = detector == nullptr
            ? 0
            : collisionRuntime.scene().effectiveIncludePairCount(detector->options);
        std::cout << "[420ExactDetector] effectivePairs=" << effectivePairCount << "\n";
        checks.require(
            detector == nullptr ||
                effectivePairCount > 0,
            "420 default exact detector contains effective collision pairs");

        const simulation_runtime::Result checkResult = collisionRuntime.check();
        checks.require(checkResult.success, "420 default exact mesh detector check does not throw or fail");
        checks.require(
            collisionRuntime.lastResult().inCollision(),
            "420 default exact mesh detector reports collision");
        checks.require(
            !collisionRuntime.lastResult().hasNearestPoints,
            "420 default colliding exact mesh detector does not report nearest points");

        const collision::CollisionDebugDrawData debugData = collisionRuntime.buildDebugDraw(true);
        checks.require(
            debugData.contacts.empty(),
            "420 default exact mesh fallback does not draw unreliable contact points");
        checks.require(
            debugData.nearestPoints.empty(),
            "420 default colliding exact mesh detector does not draw nearest points");
    }

    void verify420Link6ConvertFromVisualOverridesProjectBox(CheckContext& checks)
    {
        const std::filesystem::path root = std::filesystem::path(PROJECT_SOURCE_PATH);
        const std::filesystem::path projectPath =
            root / "config" / "projects" / "420-red4600-tool.sys.json";

        simulation_project::ProjectDocument document;
        std::string errorMessage;
        checks.require(
            simulation_project::loadProjectDocument(projectPath, document, &errorMessage),
            "420 tool project loads for Link6 visual collision selection");
        if(document.robots.empty() || document.collision.detectors.empty()) {
            checks.require(false, "420 tool project contains robots and collision detector");
            return;
        }

        simulation_project::ProjectDocumentService service(document);
        bool changed = false;
        std::string serviceError;
        checks.require(
            service.setActiveRobotLinkCollisionModel(
                "Red4600",
                "Link6",
                simulation_project::kConvertFromVisualCollisionModelId,
                &changed,
                &serviceError),
            "420 Link6 can set Convert from Visual as active collision model");

        simulation_runtime::ProjectSimulationRuntime simulation;
        const simulation_runtime::Result loadResult = simulation.loadProject(document, root);
        checks.require(loadResult.success, "420 Link6 visual-selected project builds simulation runtime");
        if(!loadResult.success) {
            return;
        }

        simulation_runtime::ProjectCollisionRuntime collisionRuntime;
        const simulation_runtime::Result buildResult = collisionRuntime.build(simulation);
        checks.require(buildResult.success, "420 Link6 visual-selected project builds collision runtime");
        if(!buildResult.success) {
            return;
        }

        const collision::CollisionDebugDrawData debugData = collisionRuntime.buildDebugDraw(true);
        std::size_t link6VisualMeshes = 0;
        std::size_t link6ProjectBoxes = 0;
        for(const collision::CollisionDebugDrawDesc& desc : debugData.geometry) {
            if(desc.linkName != "Link6") {
                continue;
            }
            if(desc.shape.source == simulation_project::kConvertFromVisualCollisionSource &&
                desc.shape.type == collision::CollisionShapeType::TriangleMesh) {
                ++link6VisualMeshes;
            }
            if(desc.shape.type == collision::CollisionShapeType::Box &&
                desc.shape.source.find("stlAabb:") == 0) {
                ++link6ProjectBoxes;
            }
        }

        checks.require(
            link6VisualMeshes > 0,
            "420 Link6 Convert from Visual builds runtime triangle mesh collision");
        checks.require(
            link6ProjectBoxes == 0,
            "420 Link6 Convert from Visual bypasses project-defined AABB box collision");
    }

    void verify420Link6GeneratedCoacdSelectionBuildsRuntimeCollision(CheckContext& checks)
    {
        const std::filesystem::path root = std::filesystem::path(PROJECT_SOURCE_PATH);
        const std::filesystem::path projectPath =
            root / "config" / "projects" / "420-red4600-tool.sys.json";

        simulation_project::ProjectDocument document;
        std::string errorMessage;
        checks.require(
            simulation_project::loadProjectDocument(projectPath, document, &errorMessage),
            "420 tool project loads for Link6 generated COACD collision selection");
        if(document.robots.empty() || document.collision.detectors.empty()) {
            checks.require(false, "420 tool project contains robots and collision detector for Link6 generated COACD");
            return;
        }

        simulation_project::ProjectDocumentService editService(document);
        simulation_project::CollisionElementOverrideDesc exactElement;
        exactElement.id = "Link6_runtime_test_project_box";
        exactElement.linkName = "Link6";
        exactElement.label = exactElement.id;
        exactElement.type = "box";
        exactElement.role = "Exact";
        exactElement.source = "runtimeTestProject";
        exactElement.boxSize = simulation_project::Vec3Desc{ 0.25, 0.04, 0.04 };

        simulation_project::CollisionElementOverrideDesc coacdElement;
        coacdElement.id = "Link6_runtime_test_coacd_box";
        coacdElement.linkName = "Link6";
        coacdElement.label = coacdElement.id;
        coacdElement.type = "box";
        coacdElement.role = simulation_project::kCoacdCollisionModelRole;
        coacdElement.source = simulation_project::kCoacdVisualSource;
        coacdElement.boxSize = simulation_project::Vec3Desc{ 0.05, 0.05, 0.05 };

        std::vector<simulation_project::CollisionElementOverrideDesc> elements;
        elements.push_back(std::move(exactElement));
        elements.push_back(std::move(coacdElement));
        editService.appendUniqueCollisionElements("Red4600", elements);

        bool changed = false;
        std::string serviceError;
        const std::string generatedVariantId =
            "Red4600|Link6|coacdVisual|CoACD|runtime-test";
        checks.require(
            editService.setActiveRobotLinkCollisionModel(
                "Red4600",
                "Link6",
                generatedVariantId,
                &changed,
                &serviceError),
            "420 Link6 can select generated COACD variant");

        const collision::CollisionDebugDrawData generatedDebugData =
            build420ToolCollisionDebugData(checks, document, "420 Link6 generated COACD collision");
        std::size_t link6CoacdBoxes = 0;
        std::size_t link6NonCoacdGeometry = 0;
        for(const collision::CollisionDebugDrawDesc& desc : generatedDebugData.geometry) {
            if(desc.linkName != "Link6") {
                continue;
            }
            if(desc.shape.type == collision::CollisionShapeType::Box &&
                desc.shape.source == simulation_project::kCoacdVisualSource) {
                ++link6CoacdBoxes;
                continue;
            }
            ++link6NonCoacdGeometry;
        }

        checks.require(
            link6CoacdBoxes == 1,
            "420 Link6 generated variant builds its COACD collision element in runtime");
        checks.require(
            link6NonCoacdGeometry == 0,
            "420 Link6 generated variant excludes original and project-defined collision geometry");
    }

    void verifyGeneratedCoacdMeshCanBeUsedAsCoacdInput(CheckContext& checks)
    {
        const std::filesystem::path root = std::filesystem::path(PROJECT_SOURCE_PATH);
        const std::filesystem::path generatedRoot =
            root / "build" / "RobotViewerCoreCollisionModelWorkflowSmokeTest_generated_assets";
        const std::filesystem::path meshPath =
            generatedRoot / "collision" / "coacd" / "existing_source" / "tool" / "coacd_input.obj";

        std::error_code fsError;
        std::filesystem::create_directories(meshPath.parent_path(), fsError);
        checks.require(!fsError, "generated COACD retry fixture directory can be created");
        if(fsError) {
            return;
        }

        const std::string cubeObj =
            "v -0.05 -0.05 -0.05\n"
            "v 0.05 -0.05 -0.05\n"
            "v 0.05 0.05 -0.05\n"
            "v -0.05 0.05 -0.05\n"
            "v -0.05 -0.05 0.05\n"
            "v 0.05 -0.05 0.05\n"
            "v 0.05 0.05 0.05\n"
            "v -0.05 0.05 0.05\n"
            "f 1 2 3\n"
            "f 1 3 4\n"
            "f 5 7 6\n"
            "f 5 8 7\n"
            "f 1 5 6\n"
            "f 1 6 2\n"
            "f 2 6 7\n"
            "f 2 7 3\n"
            "f 3 7 8\n"
            "f 3 8 4\n"
            "f 4 8 5\n"
            "f 4 5 1\n";
        checks.require(
            writeTextFile(meshPath, cubeObj),
            "generated COACD retry fixture OBJ can be written");

        RuntimeRobot runtime;
        runtime.documentId = "generated_coacd_retry_robot";
        runtime.name = "Generated COACD Retry Robot";
        runtime.model.name = runtime.name;
        runtime.model.root = "tool";
        runtime.model.base_link = "tool";

        robot::RobotLink link;
        link.name = "tool";
        link.uid = "tool";

        robot::RobotCollisionGeometry geometry;
        geometry.partUid = "tool_existing_generated_coacd_mesh";
        geometry.type = robot::RobotGeometryType::Mesh;
        geometry.enabled = true;
        geometry.meshPath = "appGenerated://collision/coacd/existing_source/tool/coacd_input.obj";
        geometry.meshScale = Eigen::Vector3d::Ones();
        geometry.role = simulation_project::kCoacdCollisionModelRole;
        geometry.source = simulation_project::kCoacdVisualSource;
        link.collisions.push_back(std::move(geometry));

        runtime.model.linkNames.push_back(link.name);
        runtime.model.links.emplace(link.name, std::move(link));

        CollisionGeneratedAssetRequest request;
        request.generatedAssetRoot = generatedRoot;
        request.sourceKey = "generated_coacd_retry_robot";
        request.targetKey = "tool_retry";
        request.role = simulation_project::kCoacdCollisionModelRole;

        std::vector<simulation_project::CollisionElementOverrideDesc> elements;
        checks.require(
            RobotCollisionProxyGenerator::generateCoacdFromExistingCollision(
                runtime,
                "tool",
                request,
                elements),
            "appGenerated COACD mesh can be resolved and reused as COACD input");
        checks.require(!elements.empty(), "appGenerated COACD retry emits generated mesh elements");
        if(!elements.empty()) {
            checks.require(
                elements.front().type == "mesh" &&
                    elements.front().role == simulation_project::kCoacdCollisionModelRole &&
                    elements.front().source == simulation_project::kCoacdCollisionSource &&
                    elements.front().meshPath.rfind("appGenerated://collision/coacd/", 0) == 0,
                "appGenerated COACD retry preserves generated mesh metadata");
        }
    }

    collision::CollisionDebugDrawData build420ToolCollisionDebugData(
        CheckContext& checks,
        const simulation_project::ProjectDocument& document,
        const std::string& label)
    {
        const std::filesystem::path root = std::filesystem::path(PROJECT_SOURCE_PATH);
        simulation_runtime::ProjectSimulationRuntime simulation;
        const simulation_runtime::Result loadResult = simulation.loadProject(document, root);
        checks.require(loadResult.success, label + " builds simulation runtime");
        if(!loadResult.success) {
            return {};
        }

        simulation_runtime::ProjectCollisionRuntime collisionRuntime;
        const simulation_runtime::Result buildResult = collisionRuntime.build(simulation);
        checks.require(buildResult.success, label + " builds collision runtime");
        if(!buildResult.success) {
            return {};
        }

        return collisionRuntime.buildDebugDraw(true);
    }

    void verify420ToolAttachmentCollisionModelSelection(CheckContext& checks)
    {
        const std::filesystem::path root = std::filesystem::path(PROJECT_SOURCE_PATH);
        const std::filesystem::path projectPath =
            root / "config" / "projects" / "420-red4600-tool.sys.json";

        simulation_project::ProjectDocument document;
        std::string errorMessage;
        checks.require(
            simulation_project::loadProjectDocument(projectPath, document, &errorMessage),
            "420 tool project loads for attachment collision model selection");

        simulation_project::ProjectDocumentService readService(document);
        const simulation_project::ObjectCollisionOverrideDesc* initialOverride =
            readService.findObjectCollisionOverride("420_tool_attachment");
        checks.require(
            initialOverride != nullptr && initialOverride->replaceOriginalCollisions,
            "420 tool attachment defaults to project-defined collision override");

        const collision::CollisionDebugDrawData defaultDebugData =
            build420ToolCollisionDebugData(checks, document, "420 attachment default project-defined collision");
        std::size_t defaultAttachmentBoxes = 0;
        for(const collision::CollisionDebugDrawDesc& desc : defaultDebugData.geometry) {
            if(desc.shape.type == collision::CollisionShapeType::Box &&
                desc.shape.source.find("stlAabb:data/Spray420/420-tool.STL") == 0) {
                ++defaultAttachmentBoxes;
            }
        }
        checks.require(
            defaultAttachmentBoxes > 0,
            "420 tool attachment project-defined variant builds AABB box collision");

        simulation_project::ProjectDocumentService editService(document);
        bool changed = false;
        std::string serviceError;
        checks.require(
            editService.setObjectCollisionReplaceOriginal(
                "420_tool_attachment",
                false,
                &changed,
                &serviceError),
            "420 tool attachment can select Convert from Visual");
        checks.require(changed, "420 tool attachment Convert from Visual changes project state");

        const collision::CollisionDebugDrawData visualDebugData =
            build420ToolCollisionDebugData(checks, document, "420 attachment visual collision");
        std::size_t visualAttachmentMeshes = 0;
        std::size_t visualAttachmentBoxes = 0;
        for(const collision::CollisionDebugDrawDesc& desc : visualDebugData.geometry) {
            if(desc.shape.label == "attachment:420_tool_attachment" &&
                desc.shape.source == simulation_project::kConvertFromVisualCollisionSource &&
                desc.shape.type == collision::CollisionShapeType::TriangleMesh) {
                ++visualAttachmentMeshes;
            }
            if(desc.shape.type == collision::CollisionShapeType::Box &&
                desc.shape.source.find("stlAabb:data/Spray420/420-tool.STL") == 0) {
                ++visualAttachmentBoxes;
            }
        }
        checks.require(
            visualAttachmentMeshes > 0,
            "420 tool attachment Convert from Visual builds attachment triangle mesh collision");
        checks.require(
            visualAttachmentBoxes == 0,
            "420 tool attachment Convert from Visual bypasses project-defined AABB box collision");

        simulation_project::ObjectCollisionElementOverrideDesc coacdElement;
        coacdElement.id = "420_tool_attachment_coacd_runtime_test";
        coacdElement.label = coacdElement.id;
        coacdElement.type = "box";
        coacdElement.role = simulation_project::kCoacdCollisionModelRole;
        coacdElement.source = simulation_project::kCoacdVisualSource;
        coacdElement.boxSize = simulation_project::Vec3Desc{ 0.05, 0.05, 0.05 };
        std::vector<simulation_project::ObjectCollisionElementOverrideDesc> coacdElements;
        coacdElements.push_back(std::move(coacdElement));
        editService.appendUniqueObjectCollisionElements("420_tool_attachment", coacdElements);

        const std::string generatedVariantId =
            "420_tool_attachment|coacdVisual|CoACD|runtime-test";
        checks.require(
            editService.setActiveObjectCollisionModel(
                "420_tool_attachment",
                generatedVariantId,
                &changed,
                &serviceError),
            "420 tool attachment can select generated COACD variant");

        const collision::CollisionDebugDrawData generatedDebugData =
            build420ToolCollisionDebugData(checks, document, "420 attachment generated COACD collision");
        std::size_t generatedAttachmentBoxes = 0;
        std::size_t generatedOriginalBoxes = 0;
        for(const collision::CollisionDebugDrawDesc& desc : generatedDebugData.geometry) {
            if(desc.shape.type == collision::CollisionShapeType::Box &&
                desc.shape.source == simulation_project::kCoacdVisualSource) {
                ++generatedAttachmentBoxes;
            }
            if(desc.shape.type == collision::CollisionShapeType::Box &&
                desc.shape.source.find("stlAabb:data/Spray420/420-tool.STL") == 0) {
                ++generatedOriginalBoxes;
            }
        }
        checks.require(
            generatedAttachmentBoxes == 1,
            "420 tool attachment generated variant builds only its COACD collision element");
        checks.require(
            generatedOriginalBoxes == 0,
            "420 tool attachment generated variant excludes project-defined AABB collision");

        checks.require(
            editService.setActiveObjectCollisionModel(
                "420_tool_attachment",
                simulation_project::kDefinedInProjectCollisionModelId,
                &changed,
                &serviceError),
            "420 tool attachment can select Defined in Project again");
    }
}

int main()
{
    CheckContext checks;
    RuntimeRobot runtime = makeRuntimeRobot();

    verifyRealInputWorkflows(checks);
    verifyDefaultUrdfMeshCollisionUsesTriangleMesh(checks);
    verifyConvertFromVisualLazyRuntime(checks);
    verifyDefault420ExactDetectorRuntime(checks);
    verify420Link6ConvertFromVisualOverridesProjectBox(checks);
    verify420ToolAttachmentCollisionModelSelection(checks);
    verify420Link6GeneratedCoacdSelectionBuildsRuntimeCollision(checks);
    verifyGeneratedCoacdMeshCanBeUsedAsCoacdInput(checks);
    verifyDefaultProjectLink6SphereCoverBaseline(checks);
    verifyLinkVariantSummary(checks);
    verifyCollisionResultOverlayBaseline(checks);
    verifyDetectorRebuildBaseline(checks);

    const RobotCollisionRobotSummary initialSummary =
        RobotCollisionModelInspector::summarizeRobot(runtime);
    checks.require(initialSummary.linkCount == 2, "inspector summarizes robot links");
    checks.require(initialSummary.visualOnlyLinkCount == 1, "inspector detects visual-only link");
    checks.require(initialSummary.collisionReadyLinkCount == 1, "inspector detects collision-ready link");

    RobotCollisionProxyRequest missingRequest;
    missingRequest.proxyType = "box";
    missingRequest.role = "PlanningProxy";
    missingRequest.inflationMargin = 0.002;

    std::vector<simulation_project::CollisionElementOverrideDesc> missingElements;
    checks.require(
        RobotCollisionProxyGenerator::generateMissingFromVisual(runtime, missingRequest, missingElements),
        "missing visual link can generate planning proxy");
    checks.require(missingElements.size() == 1, "missing proxy generation emits one visual-only link element");

    RobotCollisionProxyRequest sphereCoverRequest;
    sphereCoverRequest.proxyType = "sphereCover";
    sphereCoverRequest.role = "SphereCover";
    sphereCoverRequest.maxSphereCount = 4;

    std::vector<simulation_project::CollisionElementOverrideDesc> sphereCoverElements;
    checks.require(
        RobotCollisionProxyGenerator::generateFromExistingCollision(
            runtime,
            "base",
            sphereCoverRequest,
            sphereCoverElements),
        "existing collision can generate sphere cover");
    checks.require(sphereCoverElements.size() > 1, "long box collision generates multiple sphere cover elements");
    for(const simulation_project::CollisionElementOverrideDesc& element : sphereCoverElements) {
        checks.require(element.type == "sphere", "sphere cover element is stored as sphere");
        checks.require(element.role == "SphereCover", "sphere cover element keeps SphereCover role");
        checks.require(element.radius > 0.0, "sphere cover radius is positive");
    }

    std::vector<simulation_project::CollisionElementOverrideDesc> allElements = missingElements;
    allElements.insert(allElements.end(), sphereCoverElements.begin(), sphereCoverElements.end());

    simulation_project::ProjectDocument document =
        makeProjectDocument(runtime, allElements, true);
    simulation_project::CollisionDetectorDesc detector;
    detector.id = "sphere_cover_detector";
    detector.name = "Sphere Cover Detector";
    detector.type = "SceneAll";
    detector.queryPolicy = "AllEnabled";
    detector.geometryRole = "SphereCover";
    document.collision.detectors.push_back(detector);

    simulation_project::ProjectDocument reloadedDocument;
    checks.require(saveAndReload(document, reloadedDocument), "project collision overrides save and reload");
    checks.require(
        !reloadedDocument.collision.robotOverrides.empty() &&
            reloadedDocument.collision.robotOverrides.front().elements.size() == allElements.size(),
        "reloaded project preserves generated collision elements");
    checks.require(
        !reloadedDocument.collision.robotOverrides.empty() &&
            reloadedDocument.collision.robotOverrides.front().replaceOriginalCollisions,
        "reloaded project preserves replaceOriginalCollisions");
    checks.require(
        !reloadedDocument.collision.detectors.empty() &&
            reloadedDocument.collision.detectors.front().geometryRole == "SphereCover",
        "reloaded project preserves detector role selection");

    simulation_project::RobotCollisionOverrideDesc sidecarDesc =
        reloadedDocument.collision.robotOverrides.front();
    sidecarDesc.sourceRobotPath = "data/robots/source.urdf";
    sidecarDesc.overridePath = "robot_collision_workflow.collision.override.json";

    simulation_project::RobotCollisionOverrideDesc reloadedSidecar;
    checks.require(saveAndReloadSidecar(sidecarDesc, reloadedSidecar), "sidecar override saves and reloads");
    checks.require(reloadedSidecar.sourceRobotPath == sidecarDesc.sourceRobotPath, "sidecar preserves sourceRobotPath");
    checks.require(reloadedSidecar.overridePath == sidecarDesc.overridePath, "sidecar preserves overridePath");
    checks.require(reloadedSidecar.replaceOriginalCollisions, "sidecar preserves replaceOriginalCollisions");
    checks.require(
        !reloadedSidecar.elements.empty() &&
            reloadedSidecar.elements.front().enabled &&
            !reloadedSidecar.elements.front().source.empty(),
        "sidecar preserves enabled and source element fields");

    std::string exportedUrdf;
    checks.require(exportAndInspectUrdf(reloadedSidecar, exportedUrdf), "URDF collision export succeeds");
    checks.require(
        exportedUrdf.find("old_collision") == std::string::npos,
        "URDF export honors replaceOriginalCollisions");
    checks.require(exportedUrdf.find("<sphere") != std::string::npos, "URDF export writes generated sphere collisions");

    robot::RobotModel appliedModel = runtime.model;
    RobotCollisionOverrideApplier::apply(
        reloadedDocument,
        reloadedDocument.robots.front(),
        std::filesystem::path(PROJECT_SOURCE_PATH),
        appliedModel);
    runtime.model = appliedModel;

    const RobotCollisionRobotSummary appliedSummary =
        RobotCollisionModelInspector::summarizeRobot(runtime);
    checks.require(hasSphereCover(appliedSummary), "applied override is visible as sphere cover");
    checks.require(
        appliedSummary.overrideAppliedLinkCount == 2,
        "applied override marks both generated links as override-applied");

    const auto baseIt = runtime.model.links.find("base");
    checks.require(baseIt != runtime.model.links.end(), "base link remains in model");
    if(baseIt != runtime.model.links.end()) {
        checks.require(
            baseIt->second.collisions.size() == sphereCoverElements.size(),
            "replaceOriginalCollisions replaces base collision with generated sphere cover");
    }

    if(checks.failures > 0) {
        std::cerr << "RobotViewerCore collision model workflow smoke test failed with "
                  << checks.failures << " failure(s).\n";
        return 1;
    }

    std::cout << "RobotViewerCore collision model workflow smoke test passed.\n";
    return 0;
}
