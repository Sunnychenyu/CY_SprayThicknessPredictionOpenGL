#pragma once

#include <SceneCore/RenderPass/IRenderPass.h>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace rendercore
{
    class Shader;
}

struct RuntimeRobot;
struct RuntimeSceneObject;

class SceneObjectHoverPass final : public scenecore::IRenderPass
{
public:
    SceneObjectHoverPass() = default;
    ~SceneObjectHoverPass() override;

    SceneObjectHoverPass(const SceneObjectHoverPass&) = delete;
    SceneObjectHoverPass& operator=(const SceneObjectHoverPass&) = delete;

    void setRuntimeSources(
        const std::vector<RuntimeRobot>* robots,
        const std::vector<RuntimeSceneObject>* objects);
    void setCameraMatrices(const glm::mat4& projection, const glm::mat4& view);
    void requestPick(int x, int y);
    void clearHover();
    void setSelectedObjectId(const std::string& objectId);

    const std::string& hoveredObjectId() const;

    void render(scenecore::RenderQueue& queue) override;
    void resize(int width, int height) override;

private:
    struct CachedLocalBounds
    {
        const void* model = nullptr;
        glm::vec3 minimum{ 0.0f };
        glm::vec3 maximum{ 0.0f };
        bool valid = false;
    };

    bool ensureInitialized();
    bool ensureFramebuffer();
    void destroyResources();
    void renderIdBuffer();
    void drawOutline(unsigned int hoveredId);
    float outlineRadiusForObject(const std::string& objectId);

private:
    const std::vector<RuntimeRobot>* m_robots = nullptr;
    const std::vector<RuntimeSceneObject>* m_objects = nullptr;
    glm::mat4 m_projection{ 1.0f };
    glm::mat4 m_view{ 1.0f };
    std::shared_ptr<rendercore::Shader> m_idShader;
    std::shared_ptr<rendercore::Shader> m_outlineShader;
    std::vector<std::string> m_objectIds;
    std::unordered_map<std::string, CachedLocalBounds> m_localBoundsCache;
    std::string m_hoveredObjectId;
    std::string m_selectedObjectId;
    int m_width = 1;
    int m_height = 1;
    int m_pickX = 0;
    int m_pickY = 0;
    float m_outlineRadius = 1.0f;
    bool m_pickPending = false;
    bool m_initialized = false;
    bool m_framebufferDirty = true;
    unsigned int m_framebuffer = 0;
    unsigned int m_idTexture = 0;
    unsigned int m_depthRenderbuffer = 0;
    unsigned int m_fullscreenVao = 0;
};
