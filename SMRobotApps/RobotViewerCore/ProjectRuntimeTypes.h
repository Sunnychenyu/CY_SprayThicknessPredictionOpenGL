#pragma once

#include <Collision/CollisionObject.h>
#include <Collision/CollisionShapeDesc.h>
#include <Collision/RobotCollisionInstance.h>
#include <Collision/RobotCollisionModel.h>
#include <AssetCore/ModelDesc.h>
#include <RenderCore/Material.h>
#include <RenderCore/Model.h>
#include <RobotCore/RobotModel.h>
#include <RobotInstance/RobotInstance.h>
#include <RobotRenderBridge/RobotVisualBridge.h>
#include <SceneCore/ModelNode.h>
#include <SceneCore/PointCloudNode.h>
#include <SceneCore/RenderQueue.h>
#include <VisualizationSDK/SurfaceScalarOverlay.h>

#include <Eigen/Core>
#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class VisibleModelNode : public scenecore::ModelNode
{
public:
    explicit VisibleModelNode(std::shared_ptr<rendercore::Model> model)
        : scenecore::ModelNode(std::move(model))
    {
    }

    void setVisible(bool visible)
    {
        m_visible = visible;
    }

    bool isVisible() const
    {
        return m_visible;
    }

    void collect(scenecore::RenderQueue& queue) override
    {
        if(m_visible) {
            scenecore::ModelNode::collect(queue);
        }
    }

private:
    bool m_visible = true;
};

struct MeshOverlay
{
    std::shared_ptr<VisibleModelNode> node;
    std::shared_ptr<rendercore::Material> material;
    std::string linkName;
};

struct RuntimeRobot
{
    uint64_t runtimeId = 0;
    std::string documentId;
    robot::RobotModel model;
    std::shared_ptr<robotinstance::RobotInstance> instance;
    std::shared_ptr<robot_render::RobotVisualBridge> visualBridge;
    collision::RobotCollisionModelPtr collisionModel;
    collision::RobotCollisionInstancePtr collisionInstance;
    std::unordered_map<collision::ObjectID, MeshOverlay> meshOverlays;
    std::unordered_map<std::string, std::vector<std::shared_ptr<rendercore::Material>>> linkOriginalMaterials;
    std::unordered_set<std::string> highlightedLinks;
    std::string name;
    collision::Transform3 baseTransform = collision::Transform3::Identity();
    bool collisionEnabled = true;
    bool autoMotionEnabled = false;
    double autoMotionAmplitude = 0.5;
    double autoMotionSpeed = 1.0;
};

struct RuntimeSceneCollisionObject
{
    collision::CollisionObjectPtr collisionObject;
    collision::CollisionShapeDesc collisionShape;
    collision::Transform3 localTransform = collision::Transform3::Identity();
};

struct RuntimeSurfaceScalarSubMesh
{
    struct BvhNode
    {
        Eigen::Vector3d minimum = Eigen::Vector3d::Zero();
        Eigen::Vector3d maximum = Eigen::Vector3d::Zero();
        std::int32_t leftChild = -1;
        std::int32_t rightChild = -1;
        std::uint32_t firstTriangle = 0;
        std::uint32_t triangleCount = 0;
    };

    std::vector<Eigen::Vector3d> positions;
    std::vector<uint32_t> indices;
    std::vector<double> values;
    std::vector<std::uint32_t> triangleOrder;
    std::vector<BvhNode> bvhNodes;
};

struct RuntimeSurfaceScalarOverlay
{
    smrobot::visualization::SurfaceScalarOverlay descriptor;
    std::vector<RuntimeSurfaceScalarSubMesh> subMeshes;
    std::shared_ptr<rendercore::Model> originalModel;
    std::shared_ptr<rendercore::Model> overlayModel;
    bool visible{ false };
};

enum class SceneObjectHighlightState
{
    None,
    Selected,
    Colliding
};

struct RuntimeSceneObject
{
    uint64_t runtimeId = 0;
    std::string documentId;
    std::string name;
    std::string objectType;
    collision::Transform3 transform = collision::Transform3::Identity();
    glm::mat4 visualLocal = glm::mat4(1.0f);
    std::shared_ptr<VisibleModelNode> visualNode;
    std::shared_ptr<scenecore::PointCloudNode> pointCloudNode;
    std::shared_ptr<rendercore::Model> visualModel;
    // Reuse the loaded CPU mesh for precise screen picking.
    std::shared_ptr<assetcore::ModelDesc> pickModel;
    std::vector<std::shared_ptr<rendercore::Material>> originalMaterials;
    std::shared_ptr<rendercore::Material> highlightMaterial;
    std::shared_ptr<rendercore::Material> collisionHighlightMaterial;
    std::shared_ptr<RuntimeSurfaceScalarOverlay> surfaceScalarOverlay;
    collision::CollisionObjectPtr collisionObject;
    collision::CollisionShapeDesc collisionShape;
    std::vector<RuntimeSceneCollisionObject> collisionObjects;
    bool collisionEnabled = true;
    SceneObjectHighlightState highlightState = SceneObjectHighlightState::None;
    float pointCloudBasePointSize = 2.0f;
    bool pointCloudBoundsValid = false;
    collision::Vec3 pointCloudLocalBoundsMin = collision::Vec3::Zero();
    collision::Vec3 pointCloudLocalBoundsMax = collision::Vec3::Zero();
};

struct MaterialOverride
{
    std::shared_ptr<rendercore::Model> model;
    unsigned int subMeshIndex = 0;
    std::shared_ptr<rendercore::Material> originalMaterial;
    Eigen::Vector4f color = Eigen::Vector4f(0.15f, 1.0f, 0.35f, 1.0f);
    Eigen::Vector3f emissive = Eigen::Vector3f(0.02f, 0.18f, 0.04f);
};
