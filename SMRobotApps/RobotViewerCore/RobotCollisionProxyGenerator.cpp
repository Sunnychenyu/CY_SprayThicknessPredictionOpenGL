#include "RobotCollisionProxyGenerator.h"

#include <AssetCore/AssetManager.h>
#include <Collision/CollisionCoacdAssetWriter.h>
#include <Collision/CollisionProxyGenerator.h>
#include <CustomLog/CustomLog.h>
#include <SimulationProject/CollisionModelSelectionIds.h>

#include <glm/glm.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <sstream>
#include <system_error>
#include <utility>

namespace
{
    std::string lowerAscii(std::string value)
    {
        std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
            return static_cast<char>(std::tolower(ch));
        });
        return value;
    }

    simulation_project::TransformDesc makeTransformDesc(const collision::Transform3& transform)
    {
        simulation_project::TransformDesc desc;
        desc.x = transform.translation().x();
        desc.y = transform.translation().y();
        desc.z = transform.translation().z();
        const Eigen::Vector3d euler = transform.linear().eulerAngles(2, 1, 0);
        desc.yaw = euler[0];
        desc.pitch = euler[1];
        desc.roll = euler[2];
        return desc;
    }

    simulation_project::Vec3Desc makeVec3Desc(const collision::Vec3& value)
    {
        simulation_project::Vec3Desc desc;
        desc.x = value.x();
        desc.y = value.y();
        desc.z = value.z();
        return desc;
    }

    collision::CollisionGeometryRole roleFromString(const std::string& role)
    {
        const std::string loweredRole = lowerAscii(role);
        if(loweredRole == "simplified") {
            return collision::CollisionGeometryRole::Simplified;
        }
        if(loweredRole == "coacd" || loweredRole == "co_acd") {
            return collision::CollisionGeometryRole::Simplified;
        }
        if(loweredRole == "safety" || loweredRole == "safetymargin" || loweredRole == "safety_margin") {
            return collision::CollisionGeometryRole::SafetyMargin;
        }
        if(loweredRole == "spherecover" || loweredRole == "sphere_cover") {
            return collision::CollisionGeometryRole::SphereCover;
        }
        if(loweredRole == "planningproxy" || loweredRole == "planning_proxy") {
            return collision::CollisionGeometryRole::PlanningProxy;
        }
        return collision::CollisionGeometryRole::Exact;
    }

    collision::CollisionProxyMeshData makeMeshData(
        const robot::RobotVisual& visual)
    {
        collision::CollisionProxyMeshData mesh;
        if(visual.meshPath.empty()) {
            return mesh;
        }

        auto modelDesc = assetcore::AssetManager::instance().loadModel(
            visual.meshPath,
            static_cast<float>(visual.meshScale));
        if(!modelDesc) {
            return mesh;
        }

        for(const auto& subMesh : modelDesc->subMeshes()) {
            std::vector<collision::Vec3> subMeshVertices;
            subMeshVertices.reserve(subMesh.geometry.positions.size());
            for(const auto& position : subMesh.geometry.positions) {
                const glm::vec4 corrected = modelDesc->get_local() * glm::vec4(
                    position.x(),
                    position.y(),
                    position.z(),
                    1.0f);
                const collision::Vec3 localPoint(
                    static_cast<double>(corrected.x),
                    static_cast<double>(corrected.y),
                    static_cast<double>(corrected.z));
                subMeshVertices.push_back(visual.T_part * localPoint);
            }

            mesh.vertices.insert(mesh.vertices.end(), subMeshVertices.begin(), subMeshVertices.end());

            const std::vector<uint32_t>& indices = subMesh.geometry.indices;
            for(size_t i = 0; i + 2 < indices.size(); i += 3) {
                const uint32_t ia = indices[i];
                const uint32_t ib = indices[i + 1];
                const uint32_t ic = indices[i + 2];
                if(ia >= subMeshVertices.size() || ib >= subMeshVertices.size() || ic >= subMeshVertices.size()) {
                    continue;
                }

                const collision::Vec3& a = subMeshVertices[ia];
                const collision::Vec3& b = subMeshVertices[ib];
                const collision::Vec3& c = subMeshVertices[ic];
                mesh.vertices.push_back((a + b) * 0.5);
                mesh.vertices.push_back((b + c) * 0.5);
                mesh.vertices.push_back((c + a) * 0.5);
                mesh.vertices.push_back((a + b + c) / 3.0);
            }
        }

        return mesh;
    }

    std::string resolveGeneratedMeshPath(
        const std::string& meshPath,
        const CollisionGeneratedAssetRequest* request)
    {
        constexpr const char* kAppGeneratedPrefix = "appGenerated://";
        if(request == nullptr ||
            request->generatedAssetRoot.empty() ||
            meshPath.rfind(kAppGeneratedPrefix, 0) != 0) {
            return meshPath;
        }

        const std::string relative = meshPath.substr(std::string(kAppGeneratedPrefix).size());
        return (request->generatedAssetRoot / std::filesystem::u8path(relative)).string();
    }

    void appendModelTriangles(
        const std::string& meshPath,
        double modelScale,
        const Eigen::Vector3d& meshScale,
        const Eigen::Isometry3d& transform,
        collision::CollisionCoacdMesh& output,
        const CollisionGeneratedAssetRequest* request = nullptr)
    {
        if(meshPath.empty()) {
            return;
        }

        const std::string resolvedMeshPath = resolveGeneratedMeshPath(meshPath, request);
        std::string loadError;
        auto modelDesc = assetcore::AssetManager::instance().tryLoadModel(
            resolvedMeshPath,
            static_cast<float>(modelScale),
            &loadError);
        if(!modelDesc) {
            return;
        }

        for(const auto& subMesh : modelDesc->subMeshes()) {
            const uint32_t vertexOffset = static_cast<uint32_t>(output.vertices.size());
            for(const auto& position : subMesh.geometry.positions) {
                const glm::vec4 corrected = modelDesc->get_local() * glm::vec4(
                    position.x(),
                    position.y(),
                    position.z(),
                    1.0f);
                collision::Vec3 localPoint(
                    static_cast<double>(corrected.x) * meshScale.x(),
                    static_cast<double>(corrected.y) * meshScale.y(),
                    static_cast<double>(corrected.z) * meshScale.z());
                output.vertices.push_back(transform * localPoint);
            }

            const std::vector<uint32_t>& indices = subMesh.geometry.indices;
            if(!indices.empty()) {
                for(size_t i = 0; i + 2 < indices.size(); i += 3) {
                    if(indices[i] >= subMesh.geometry.positions.size() ||
                        indices[i + 1] >= subMesh.geometry.positions.size() ||
                        indices[i + 2] >= subMesh.geometry.positions.size()) {
                        continue;
                    }
                    output.indices.push_back(vertexOffset + indices[i]);
                    output.indices.push_back(vertexOffset + indices[i + 1]);
                    output.indices.push_back(vertexOffset + indices[i + 2]);
                }
            } else {
                for(size_t i = 0; i + 2 < subMesh.geometry.positions.size(); i += 3) {
                    output.indices.push_back(vertexOffset + static_cast<uint32_t>(i));
                    output.indices.push_back(vertexOffset + static_cast<uint32_t>(i + 1));
                    output.indices.push_back(vertexOffset + static_cast<uint32_t>(i + 2));
                }
            }
        }
    }

    collision::CollisionCoacdMesh makeTriangleMeshData(const robot::RobotVisual& visual)
    {
        collision::CollisionCoacdMesh mesh;
        appendModelTriangles(
            visual.meshPath,
            visual.meshScale,
            Eigen::Vector3d::Ones(),
            visual.T_part,
            mesh);
        return mesh;
    }

    void appendModelVertices(
        const std::string& meshPath,
        const Eigen::Vector3d& meshScale,
        const Eigen::Isometry3d& transform,
        collision::CollisionProxyMeshData& output)
    {
        if(meshPath.empty()) {
            return;
        }

        auto modelDesc = assetcore::AssetManager::instance().loadModel(meshPath, 1.0f);
        if(!modelDesc) {
            return;
        }

        for(const auto& subMesh : modelDesc->subMeshes()) {
            std::vector<collision::Vec3> subMeshVertices;
            subMeshVertices.reserve(subMesh.geometry.positions.size());
            for(const auto& position : subMesh.geometry.positions) {
                const glm::vec4 corrected = modelDesc->get_local() * glm::vec4(
                    position.x(),
                    position.y(),
                    position.z(),
                    1.0f);
                collision::Vec3 localPoint(
                    static_cast<double>(corrected.x) * meshScale.x(),
                    static_cast<double>(corrected.y) * meshScale.y(),
                    static_cast<double>(corrected.z) * meshScale.z());
                subMeshVertices.push_back(transform * localPoint);
            }

            output.vertices.insert(output.vertices.end(), subMeshVertices.begin(), subMeshVertices.end());

            const std::vector<uint32_t>& indices = subMesh.geometry.indices;
            for(size_t i = 0; i + 2 < indices.size(); i += 3) {
                const uint32_t ia = indices[i];
                const uint32_t ib = indices[i + 1];
                const uint32_t ic = indices[i + 2];
                if(ia >= subMeshVertices.size() || ib >= subMeshVertices.size() || ic >= subMeshVertices.size()) {
                    continue;
                }

                const collision::Vec3& a = subMeshVertices[ia];
                const collision::Vec3& b = subMeshVertices[ib];
                const collision::Vec3& c = subMeshVertices[ic];
                output.vertices.push_back((a + b) * 0.5);
                output.vertices.push_back((b + c) * 0.5);
                output.vertices.push_back((c + a) * 0.5);
                output.vertices.push_back((a + b + c) / 3.0);
            }
        }
    }

    void appendBoxVertices(
        const robot::RobotCollisionGeometry& geometry,
        collision::CollisionProxyMeshData& output)
    {
        const collision::Vec3 halfSize = geometry.boxSize * 0.5;
        for(int x = -1; x <= 1; x += 2) {
            for(int y = -1; y <= 1; y += 2) {
                for(int z = -1; z <= 1; z += 2) {
                    const collision::Vec3 point(
                        halfSize.x() * static_cast<double>(x),
                        halfSize.y() * static_cast<double>(y),
                        halfSize.z() * static_cast<double>(z));
                    output.vertices.push_back(geometry.T_part * point);
                }
            }
        }
    }

    void appendSphereVertices(
        const robot::RobotCollisionGeometry& geometry,
        collision::CollisionProxyMeshData& output)
    {
        const double radius = std::max(geometry.radius, 0.0);
        output.vertices.push_back(geometry.T_part * collision::Vec3(radius, 0.0, 0.0));
        output.vertices.push_back(geometry.T_part * collision::Vec3(-radius, 0.0, 0.0));
        output.vertices.push_back(geometry.T_part * collision::Vec3(0.0, radius, 0.0));
        output.vertices.push_back(geometry.T_part * collision::Vec3(0.0, -radius, 0.0));
        output.vertices.push_back(geometry.T_part * collision::Vec3(0.0, 0.0, radius));
        output.vertices.push_back(geometry.T_part * collision::Vec3(0.0, 0.0, -radius));
    }

    void appendCylinderVertices(
        const robot::RobotCollisionGeometry& geometry,
        collision::CollisionProxyMeshData& output)
    {
        const double radius = std::max(geometry.radius, 0.0);
        const double halfLength = std::max(geometry.length, 0.0) * 0.5;
        const int segments = 8;
        for(int i = 0; i < segments; ++i) {
            const double angle = 2.0 * 3.14159265358979323846 * static_cast<double>(i) /
                static_cast<double>(segments);
            const double x = std::cos(angle) * radius;
            const double y = std::sin(angle) * radius;
            output.vertices.push_back(geometry.T_part * collision::Vec3(x, y, -halfLength));
            output.vertices.push_back(geometry.T_part * collision::Vec3(x, y, halfLength));
        }
    }

    collision::CollisionProxyMeshData makeMeshData(
        const robot::RobotCollisionGeometry& geometry)
    {
        collision::CollisionProxyMeshData mesh;
        if(!geometry.enabled) {
            return mesh;
        }

        switch(geometry.type) {
        case robot::RobotGeometryType::Box:
            appendBoxVertices(geometry, mesh);
            break;
        case robot::RobotGeometryType::Sphere:
            appendSphereVertices(geometry, mesh);
            break;
        case robot::RobotGeometryType::Cylinder:
            appendCylinderVertices(geometry, mesh);
            break;
        case robot::RobotGeometryType::Mesh:
            appendModelVertices(geometry.meshPath, geometry.meshScale, geometry.T_part, mesh);
            break;
        default:
            break;
        }

        return mesh;
    }

    collision::CollisionCoacdMesh makeTriangleMeshData(
        const robot::RobotCollisionGeometry& geometry,
        const CollisionGeneratedAssetRequest* request = nullptr)
    {
        collision::CollisionCoacdMesh mesh;
        if(!geometry.enabled || geometry.type != robot::RobotGeometryType::Mesh) {
            return mesh;
        }
        appendModelTriangles(
            geometry.meshPath,
            1.0,
            geometry.meshScale,
            geometry.T_part,
            mesh,
            request);
        return mesh;
    }

    void appendTriangleMesh(
        collision::CollisionCoacdMesh& output,
        const collision::CollisionCoacdMesh& input)
    {
        const uint32_t vertexOffset = static_cast<uint32_t>(output.vertices.size());
        output.vertices.insert(output.vertices.end(), input.vertices.begin(), input.vertices.end());
        for(const uint32_t index : input.indices) {
            output.indices.push_back(vertexOffset + index);
        }
    }

    collision::CollisionCoacdMesh linkVisualTriangleMesh(const robot::RobotLink& link)
    {
        collision::CollisionCoacdMesh mesh;
        for(const robot::RobotVisual& visual : link.visuals) {
            appendTriangleMesh(mesh, makeTriangleMeshData(visual));
        }
        return mesh;
    }

    collision::CollisionCoacdMesh linkCollisionTriangleMesh(
        const robot::RobotLink& link,
        const CollisionGeneratedAssetRequest* request = nullptr)
    {
        collision::CollisionCoacdMesh mesh;
        for(const robot::RobotCollisionGeometry& geometry : link.collisions) {
            appendTriangleMesh(mesh, makeTriangleMeshData(geometry, request));
        }
        return mesh;
    }

    collision::CollisionCoacdMesh objectVisualTriangleMesh(const RuntimeSceneObject& object)
    {
        collision::CollisionCoacdMesh mesh;
        if(!object.collisionShape.vertices.empty() && !object.collisionShape.indices.empty()) {
            mesh.vertices.reserve(object.collisionShape.vertices.size());
            for(const collision::Vec3& vertex : object.collisionShape.vertices) {
                mesh.vertices.push_back(object.collisionShape.localTransform * vertex);
            }
            mesh.indices = object.collisionShape.indices;
        }
        return mesh;
    }

    void appendVisualMeshes(
        const robot::RobotLink& link,
        collision::CollisionProxyMeshData& output)
    {
        for(const robot::RobotVisual& visual : link.visuals) {
            collision::CollisionProxyMeshData mesh = makeMeshData(visual);
            output.vertices.insert(output.vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
        }
    }

    void appendCollisionMeshes(
        const robot::RobotLink& link,
        collision::CollisionProxyMeshData& output)
    {
        for(const robot::RobotCollisionGeometry& geometry : link.collisions) {
            collision::CollisionProxyMeshData mesh = makeMeshData(geometry);
            output.vertices.insert(output.vertices.end(), mesh.vertices.begin(), mesh.vertices.end());
        }
    }

    std::string makeElementId(
        const std::string& linkName,
        const std::string& proxyType,
        const std::string& source,
        size_t index,
        size_t count)
    {
        const std::string sourceName = source == "visualProxy"
            ? std::string("visual")
            : lowerAscii(source);
        std::string id = linkName + "_" + lowerAscii(proxyType) + "_" + sourceName + "_proxy";
        if(count > 1) {
            id += "_" + std::to_string(index + 1);
        }
        return id;
    }

    bool proxyTypeFromString(
        const std::string& proxyType,
        collision::CollisionProxyType& type)
    {
        const std::string loweredType = lowerAscii(proxyType);
        if(loweredType == "box" || loweredType == "aabb" || loweredType == "aabbbox") {
            type = collision::CollisionProxyType::AabbBox;
            return true;
        }
        if(loweredType == "sphere") {
            type = collision::CollisionProxyType::Sphere;
            return true;
        }
        if(loweredType == "spherecover" || loweredType == "sphere_cover") {
            type = collision::CollisionProxyType::SphereCover;
            return true;
        }
        return false;
    }

    std::string effectiveRole(const RobotCollisionProxyRequest& request)
    {
        const std::string loweredType = lowerAscii(request.proxyType);
        const std::string loweredRole = lowerAscii(request.role);
        if((loweredType == "spherecover" || loweredType == "sphere_cover") &&
            (request.role.empty() || loweredRole == "planningproxy" || loweredRole == "planning_proxy")) {
            return "SphereCover";
        }
        return request.role.empty() ? std::string("PlanningProxy") : request.role;
    }

    bool hasEnabledCollision(const robot::RobotLink& link)
    {
        for(const robot::RobotCollisionGeometry& geometry : link.collisions) {
            if(geometry.enabled) {
                return true;
            }
        }
        return false;
    }

    void applyShape(
        const std::string& linkName,
        const RobotCollisionProxyRequest& request,
        const std::string& source,
        const collision::CollisionShapeDesc& shape,
        size_t index,
        size_t count,
        simulation_project::CollisionElementOverrideDesc& element)
    {
        element.id = makeElementId(linkName, request.proxyType, source, index, count);
        element.linkName = linkName;
        element.label = element.id;
        element.role = effectiveRole(request);
        element.enabled = true;
        element.localTransform = makeTransformDesc(shape.localTransform);
        element.inflationMargin = shape.inflationMargin;
        element.source = source;

        if(shape.type == collision::CollisionShapeType::Box) {
            element.type = "box";
            element.boxSize = makeVec3Desc(shape.boxSize);
        } else if(shape.type == collision::CollisionShapeType::Sphere) {
            element.type = "sphere";
            element.radius = shape.radius;
        }
    }

    bool generateFromMesh(
        const collision::CollisionProxyMeshData& mesh,
        const std::string& linkName,
        const RobotCollisionProxyRequest& request,
        const std::string& source,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
    {
        elements.clear();
        if(mesh.vertices.empty()) {
            return false;
        }

        collision::CollisionProxyType type = collision::CollisionProxyType::AabbBox;
        if(!proxyTypeFromString(request.proxyType, type)) {
            return false;
        }

        collision::CollisionProxyOptions options;
        options.role = roleFromString(effectiveRole(request));
        options.inflationMargin = std::max(request.inflationMargin, 0.0);
        options.maxSphereCount = std::max(request.maxSphereCount, 1);

        const collision::CollisionProxyBuildResult result =
            collision::CollisionProxyGenerator::generate(mesh, type, options);
        if(!result.ok() || result.shapes.empty()) {
            return false;
        }

        elements.reserve(result.shapes.size());
        for(size_t i = 0; i < result.shapes.size(); ++i) {
            simulation_project::CollisionElementOverrideDesc element;
            applyShape(linkName, request, source, result.shapes[i], i, result.shapes.size(), element);
            elements.push_back(std::move(element));
        }

        return true;
    }

    bool buildCoacdAsset(
        const collision::CollisionCoacdMesh& mesh,
        const CollisionGeneratedAssetRequest& request,
        const std::string& source,
        collision::CollisionCoacdAssetBuildResult& assetResult)
    {
        if(mesh.vertices.empty() || mesh.indices.empty()) {
            LOG_ERROR("rs2026") << "COACD generation rejected empty input mesh: source=" << source
                << ", sourceKey=" << request.sourceKey
                << ", targetKey=" << request.targetKey
                << ", vertices=" << mesh.vertices.size()
                << ", indices=" << mesh.indices.size();
            return false;
        }

        collision::CollisionCoacdAssetBuildRequest buildRequest;
        buildRequest.outputRoot = request.generatedAssetRoot;
        buildRequest.sourceKey = request.sourceKey;
        buildRequest.targetKey = request.targetKey;
        buildRequest.source = source;
        buildRequest.role = request.role;
        buildRequest.options = request.options;
        assetResult = collision::CollisionCoacdAssetWriter::build(mesh, buildRequest);
        if(!assetResult.ok()) {
            LOG_ERROR("rs2026") << "COACD asset generation failed: source=" << source
                << ", sourceKey=" << request.sourceKey
                << ", targetKey=" << request.targetKey
                << ", outputRoot=" << request.generatedAssetRoot.string()
                << ", vertices=" << mesh.vertices.size()
                << ", triangles=" << (mesh.indices.size() / 3)
                << ", error=" << assetResult.error;
        }
        return assetResult.ok();
    }

    void appendRobotCoacdElements(
        const std::string& linkName,
        const CollisionGeneratedAssetRequest& request,
        const std::string& source,
        const collision::CollisionCoacdAssetBuildResult& assetResult,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
    {
        for(std::size_t i = 0; i < assetResult.parts.size(); ++i) {
            simulation_project::CollisionElementOverrideDesc element;
            element.id = linkName + "_coacd_" + std::to_string(i + 1);
            element.linkName = linkName;
            element.label = element.id;
            element.type = "mesh";
            element.role = request.role.empty()
                ? std::string(simulation_project::kCoacdCollisionModelRole)
                : request.role;
            element.enabled = true;
            element.meshPath = assetResult.parts[i].meshUri;
            element.source = source;
            elements.push_back(std::move(element));
        }
    }

    void appendObjectCoacdElements(
        const CollisionGeneratedAssetRequest& request,
        const std::string& source,
        const collision::CollisionCoacdAssetBuildResult& assetResult,
        std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements)
    {
        const std::string targetKey =
            collision::CollisionCoacdAssetWriter::sanitizePathToken(request.targetKey, "target");
        for(std::size_t i = 0; i < assetResult.parts.size(); ++i) {
            simulation_project::ObjectCollisionElementOverrideDesc element;
            element.id = targetKey + "_coacd_" + std::to_string(i + 1);
            element.label = element.id;
            element.type = "mesh";
            element.role = request.role.empty()
                ? std::string(simulation_project::kCoacdCollisionModelRole)
                : request.role;
            element.enabled = true;
            element.meshPath = assetResult.parts[i].meshUri;
            element.source = source;
            elements.push_back(std::move(element));
        }
    }

    bool generateFromLinkVisual(
        const robot::RobotLink& link,
        const std::string& linkName,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
    {
        collision::CollisionProxyMeshData mesh;
        appendVisualMeshes(link, mesh);
        return generateFromMesh(mesh, linkName, request, "visualProxy", elements);
    }

    bool generateFromLinkCollision(
        const robot::RobotLink& link,
        const std::string& linkName,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
    {
        collision::CollisionProxyMeshData mesh;
        appendCollisionMeshes(link, mesh);
        return generateFromMesh(mesh, linkName, request, "collisionProxy", elements);
    }

    double outsideDistanceToSphere(
        const collision::Vec3& point,
        const simulation_project::CollisionElementOverrideDesc& sphere)
    {
        const collision::Vec3 center(
            sphere.localTransform.x,
            sphere.localTransform.y,
            sphere.localTransform.z);
        return (point - center).norm() - sphere.radius;
    }

    bool isFiniteVec(const collision::Vec3& value)
    {
        return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
    }

    bool computeMeshBounds(
        const collision::CollisionProxyMeshData& mesh,
        collision::Vec3& minPoint,
        collision::Vec3& maxPoint)
    {
        bool valid = false;
        minPoint = collision::Vec3(
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());
        maxPoint = collision::Vec3(
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest(),
            std::numeric_limits<double>::lowest());

        for(const collision::Vec3& point : mesh.vertices) {
            if(!isFiniteVec(point)) {
                continue;
            }
            minPoint = minPoint.cwiseMin(point);
            maxPoint = maxPoint.cwiseMax(point);
            valid = true;
        }
        return valid;
    }

    int dominantAxis(const collision::Vec3& size)
    {
        if(size.y() > size.x() && size.y() >= size.z()) {
            return 1;
        }
        if(size.z() > size.x() && size.z() > size.y()) {
            return 2;
        }
        return 0;
    }

    double crossSectionRadius(const collision::Vec3& size, int axis)
    {
        double radiusSquared = 0.0;
        for(int i = 0; i < 3; ++i) {
            if(i != axis) {
                const double halfExtent = std::max(size[i], 0.0) * 0.5;
                radiusSquared += halfExtent * halfExtent;
            }
        }
        return std::sqrt(radiusSquared);
    }

    void appendWarningText(RobotCollisionProxyQualitySummary& summary, const std::string& text)
    {
        summary.hasWarning = true;
        if(!summary.warning.empty()) {
            summary.warning += "  ";
        }
        summary.warning += text;
    }
}

bool RobotCollisionProxyGenerator::generateFromVisual(
    const RuntimeRobot& robot,
    const std::string& linkName,
    const std::string& proxyType,
    simulation_project::CollisionElementOverrideDesc& element)
{
    RobotCollisionProxyRequest request;
    request.proxyType = proxyType;

    std::vector<simulation_project::CollisionElementOverrideDesc> elements;
    if(!generateFromVisual(robot, linkName, request, elements) || elements.empty()) {
        return false;
    }

    element = std::move(elements.front());
    return true;
}

bool RobotCollisionProxyGenerator::generateFromVisual(
    const RuntimeRobot& robot,
    const std::string& linkName,
    const RobotCollisionProxyRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
{
    const auto linkIt = robot.model.links.find(linkName);
    if(linkIt == robot.model.links.end()) {
        elements.clear();
        return false;
    }

    return generateFromLinkVisual(linkIt->second, linkName, request, elements);
}

bool RobotCollisionProxyGenerator::generateFromExistingCollision(
    const RuntimeRobot& robot,
    const std::string& linkName,
    const RobotCollisionProxyRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
{
    const auto linkIt = robot.model.links.find(linkName);
    if(linkIt == robot.model.links.end()) {
        elements.clear();
        return false;
    }

    return generateFromLinkCollision(linkIt->second, linkName, request, elements);
}

bool RobotCollisionProxyGenerator::generateCoacdFromVisual(
    const RuntimeRobot& robot,
    const std::string& linkName,
    const CollisionGeneratedAssetRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
{
    elements.clear();
    const auto linkIt = robot.model.links.find(linkName);
    if(linkIt == robot.model.links.end()) {
        return false;
    }

    const collision::CollisionCoacdMesh mesh = linkVisualTriangleMesh(linkIt->second);
    collision::CollisionCoacdAssetBuildResult assetResult;
    if(!buildCoacdAsset(mesh, request, simulation_project::kCoacdVisualSource, assetResult)) {
        return false;
    }

    appendRobotCoacdElements(
        linkName,
        request,
        simulation_project::kCoacdVisualSource,
        assetResult,
        elements);
    return !elements.empty();
}

bool RobotCollisionProxyGenerator::generateCoacdFromExistingCollision(
    const RuntimeRobot& robot,
    const std::string& linkName,
    const CollisionGeneratedAssetRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
{
    elements.clear();
    const auto linkIt = robot.model.links.find(linkName);
    if(linkIt == robot.model.links.end()) {
        return false;
    }

    const collision::CollisionCoacdMesh mesh = linkCollisionTriangleMesh(linkIt->second, &request);
    collision::CollisionCoacdAssetBuildResult assetResult;
    if(!buildCoacdAsset(mesh, request, simulation_project::kCoacdCollisionSource, assetResult)) {
        return false;
    }

    appendRobotCoacdElements(
        linkName,
        request,
        simulation_project::kCoacdCollisionSource,
        assetResult,
        elements);
    return !elements.empty();
}

bool RobotCollisionProxyGenerator::generateObjectCoacdFromVisual(
    const RuntimeSceneObject& object,
    const CollisionGeneratedAssetRequest& request,
    std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements)
{
    elements.clear();

    const collision::CollisionCoacdMesh mesh = objectVisualTriangleMesh(object);
    collision::CollisionCoacdAssetBuildResult assetResult;
    if(!buildCoacdAsset(mesh, request, simulation_project::kCoacdVisualSource, assetResult)) {
        return false;
    }

    appendObjectCoacdElements(
        request,
        simulation_project::kCoacdVisualSource,
        assetResult,
        elements);
    return !elements.empty();
}

bool RobotCollisionProxyGenerator::generateMissingFromVisual(
    const RuntimeRobot& robot,
    const std::string& proxyType,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
{
    RobotCollisionProxyRequest request;
    request.proxyType = proxyType;
    return generateMissingFromVisual(robot, request, elements);
}

bool RobotCollisionProxyGenerator::generateMissingFromVisual(
    const RuntimeRobot& robot,
    const RobotCollisionProxyRequest& request,
    std::vector<simulation_project::CollisionElementOverrideDesc>& elements)
{
    elements.clear();

    std::vector<std::string> linkNames = robot.model.linkNames;
    if(linkNames.empty()) {
        linkNames.reserve(robot.model.links.size());
        for(const auto& item : robot.model.links) {
            linkNames.push_back(item.first);
        }
        std::sort(linkNames.begin(), linkNames.end());
    }

    for(const std::string& linkName : linkNames) {
        const auto linkIt = robot.model.links.find(linkName);
        if(linkIt == robot.model.links.end()) {
            continue;
        }

        const robot::RobotLink& link = linkIt->second;
        if(link.visuals.empty() || hasEnabledCollision(link)) {
            continue;
        }

        std::vector<simulation_project::CollisionElementOverrideDesc> generatedElements;
        if(generateFromLinkVisual(link, linkName, request, generatedElements)) {
            elements.insert(
                elements.end(),
                std::make_move_iterator(generatedElements.begin()),
                std::make_move_iterator(generatedElements.end()));
        }
    }

    return !elements.empty();
}

bool RobotCollisionProxyGenerator::evaluateGeneratedSphereCover(
    const RuntimeRobot& robot,
    const std::string& linkName,
    const RobotCollisionProxyRequest& request,
    const std::vector<simulation_project::CollisionElementOverrideDesc>& elements,
    bool useExistingCollisionInput,
    RobotCollisionProxyQualitySummary& summary)
{
    summary = RobotCollisionProxyQualitySummary();
    summary.requestedMaxSpheres = std::max(request.maxSphereCount, 1);

    const std::string proxyType = lowerAscii(request.proxyType);
    if(proxyType != "spherecover" && proxyType != "sphere_cover") {
        return false;
    }

    const auto linkIt = robot.model.links.find(linkName);
    if(linkIt == robot.model.links.end()) {
        return false;
    }

    collision::CollisionProxyMeshData inputMesh;
    if(useExistingCollisionInput) {
        appendCollisionMeshes(linkIt->second, inputMesh);
    } else {
        appendVisualMeshes(linkIt->second, inputMesh);
    }
    summary.inputPointCount = inputMesh.vertices.size();
    collision::Vec3 minPoint;
    collision::Vec3 maxPoint;
    if(computeMeshBounds(inputMesh, minPoint, maxPoint)) {
        const collision::Vec3 size = maxPoint - minPoint;
        const int axis = dominantAxis(size);
        summary.mainAxisLength = std::max(size[axis], 0.0);
        summary.estimatedCrossSectionRadius = crossSectionRadius(size, axis);
    }

    for(const simulation_project::CollisionElementOverrideDesc& element : elements) {
        if(element.type == "sphere" && element.radius > 0.0) {
            ++summary.generatedSphereCount;
            summary.maxSphereRadius = std::max(summary.maxSphereRadius, element.radius);
        }
    }

    if(summary.mainAxisLength > 0.0) {
        summary.maxRadiusToLinkLength = summary.maxSphereRadius / summary.mainAxisLength;
    }
    if(summary.estimatedCrossSectionRadius > 0.0) {
        summary.maxRadiusToCrossSectionRadius =
            summary.maxSphereRadius / summary.estimatedCrossSectionRadius;
    }

    if(inputMesh.vertices.empty() || summary.generatedSphereCount == 0) {
        appendWarningText(
            summary,
            useExistingCollisionInput
                ? "Sphere cover quality warning: existing collision input has no usable points."
                : "Sphere cover quality warning: visual mesh input has no usable points.");
        return true;
    }

    constexpr double kUncoveredEpsilon = 1.0e-5;
    for(const collision::Vec3& point : inputMesh.vertices) {
        double nearestOutside = std::numeric_limits<double>::max();
        for(const simulation_project::CollisionElementOverrideDesc& element : elements) {
            if(element.type != "sphere" || element.radius <= 0.0) {
                continue;
            }
            nearestOutside = std::min(nearestOutside, outsideDistanceToSphere(point, element));
        }

        if(nearestOutside == std::numeric_limits<double>::max()) {
            nearestOutside = 0.0;
        }

        const double clampedOutside = std::max(0.0, nearestOutside);
        summary.maxOutsideDistance = std::max(summary.maxOutsideDistance, clampedOutside);
        if(clampedOutside > kUncoveredEpsilon) {
            ++summary.uncoveredPointCount;
        }
    }

    if(summary.uncoveredPointCount > 0 ||
        summary.generatedSphereCount < summary.requestedMaxSpheres) {
        std::ostringstream warning;
        warning << "Sphere cover may be sparse: generated "
                << summary.generatedSphereCount << "/"
                << summary.requestedMaxSpheres
                << " spheres, uncovered points "
                << summary.uncoveredPointCount
                << ", max outside distance "
                << summary.maxOutsideDistance;
        appendWarningText(summary, warning.str());
    }

    const bool slenderInput = summary.mainAxisLength > 0.0 &&
        summary.estimatedCrossSectionRadius > 0.0 &&
        summary.mainAxisLength > summary.estimatedCrossSectionRadius * 6.0;
    if(slenderInput) {
        const double oversizedRadius =
            std::max(summary.estimatedCrossSectionRadius * 4.0, summary.mainAxisLength * 0.18);
        for(const simulation_project::CollisionElementOverrideDesc& element : elements) {
            if(element.type == "sphere" && element.radius > oversizedRadius) {
                ++summary.oversizedSphereCount;
            }
        }
        summary.recommendedShape = "sphere chain or cylinder/capsule proxy";
    }

    if(summary.oversizedSphereCount > 0) {
        std::ostringstream warning;
        warning << "Sphere cover has " << summary.oversizedSphereCount
                << " oversized sphere(s); consider more spheres or a cylinder/capsule proxy.";
        appendWarningText(summary, warning.str());
    }

    return true;
}
