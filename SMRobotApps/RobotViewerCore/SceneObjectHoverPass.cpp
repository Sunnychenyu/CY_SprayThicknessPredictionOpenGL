#include "SceneObjectHoverPass.h"

#include "ProjectRuntimeTypes.h"

#include <RenderCore/Geometry.h>
#include <RenderCore/Model.h>
#include <RenderCore/Shader.h>

#include <glad/glad.h>

#include <algorithm>
#include <array>
#include <exception>
#include <iostream>
#include <limits>

namespace
{
    constexpr const char* kIdVertexShader = R"glsl(
#version 430 core
layout(location = 0) in vec3 aPosition;

uniform mat4 uProjection;
uniform mat4 uView;
uniform mat4 uModel;

void main()
{
    gl_Position = uProjection * uView * uModel * vec4(aPosition, 1.0);
}
)glsl";

    constexpr const char* kIdFragmentShader = R"glsl(
#version 430 core
layout(location = 0) out uint outObjectId;

uniform int uObjectId;

void main()
{
    outObjectId = uint(max(uObjectId, 0));
}
)glsl";

    constexpr const char* kOutlineVertexShader = R"glsl(
#version 430 core
out vec2 vUv;

void main()
{
    vec2 position = gl_VertexID == 0
        ? vec2(-1.0, -1.0)
        : (gl_VertexID == 1 ? vec2(3.0, -1.0) : vec2(-1.0, 3.0));
    vUv = position * 0.5 + 0.5;
    gl_Position = vec4(position, 0.0, 1.0);
}
)glsl";

    constexpr const char* kOutlineFragmentShader = R"glsl(
#version 430 core
layout(location = 0) out vec4 outColor;

uniform usampler2D uObjectIds;
uniform int uHoveredId;
uniform float uOutlineRadius;

in vec2 vUv;

uint objectIdAt(ivec2 pixel, ivec2 size)
{
    return texelFetch(uObjectIds, clamp(pixel, ivec2(0), size - ivec2(1)), 0).r;
}

void main()
{
    ivec2 size = textureSize(uObjectIds, 0);
    ivec2 pixel = ivec2(vUv * vec2(size));
    uint hovered = uint(max(uHoveredId, 0));
    uint center = objectIdAt(pixel, size);
    bool centerHovered = center == hovered;
    bool nearHovered = false;
    bool nearOther = false;

    int radius = int(ceil(clamp(uOutlineRadius, 1.0, 4.0)));
    for(int y = -4; y <= 4; ++y) {
        for(int x = -4; x <= 4; ++x) {
            if(x == 0 && y == 0) {
                continue;
            }
            if(abs(x) > radius || abs(y) > radius ||
                float(x * x + y * y) > uOutlineRadius * uOutlineRadius) {
                continue;
            }
            uint neighbor = objectIdAt(pixel + ivec2(x, y), size);
            nearHovered = nearHovered || neighbor == hovered;
            nearOther = nearOther || neighbor != hovered;
        }
    }

    if(hovered == 0u || !((centerHovered && nearOther) || (!centerHovered && nearHovered))) {
        discard;
    }

    outColor = vec4(1.0, 0.84, 0.12, 0.92);
}
)glsl";

    void drawRenderModel(
        const std::shared_ptr<rendercore::Model>& model,
        const glm::mat4& transform,
        const std::shared_ptr<rendercore::Shader>& shader,
        int objectId)
    {
        if(!model || !shader) {
            return;
        }

        shader->setMat4("uModel", transform);
        shader->setInt("uObjectId", objectId);
        for(unsigned int index = 0; index < model->subMeshCount(); ++index) {
            const auto& subMesh = model->subMesh(index);
            if(subMesh.geometry) {
                subMesh.geometry->draw();
            }
        }
    }
}

SceneObjectHoverPass::~SceneObjectHoverPass()
{
    destroyResources();
}

void SceneObjectHoverPass::setRuntimeSources(
    const std::vector<RuntimeRobot>* robots,
    const std::vector<RuntimeSceneObject>* objects)
{
    m_robots = robots;
    m_objects = objects;
    m_localBoundsCache.clear();
}

void SceneObjectHoverPass::setCameraMatrices(const glm::mat4& projection, const glm::mat4& view)
{
    m_projection = projection;
    m_view = view;
}

void SceneObjectHoverPass::requestPick(int x, int y)
{
    m_pickX = x;
    m_pickY = y;
    m_pickPending = true;
}

void SceneObjectHoverPass::clearHover()
{
    m_pickPending = false;
    m_hoveredObjectId.clear();
    m_outlineRadius = 1.0f;
}

void SceneObjectHoverPass::setSelectedObjectId(const std::string& objectId)
{
    m_selectedObjectId = objectId;
}

const std::string& SceneObjectHoverPass::hoveredObjectId() const
{
    return m_hoveredObjectId;
}

void SceneObjectHoverPass::resize(int width, int height)
{
    const int nextWidth = std::max(width, 1);
    const int nextHeight = std::max(height, 1);
    if(nextWidth == m_width && nextHeight == m_height) {
        return;
    }
    m_width = nextWidth;
    m_height = nextHeight;
    m_framebufferDirty = true;
    clearHover();
}

bool SceneObjectHoverPass::ensureInitialized()
{
    if(m_initialized) {
        return true;
    }

    try {
        m_idShader = rendercore::Shader::fromSources(
            "SceneObjectHoverId",
            kIdVertexShader,
            kIdFragmentShader);
        m_outlineShader = rendercore::Shader::fromSources(
            "SceneObjectHoverOutline",
            kOutlineVertexShader,
            kOutlineFragmentShader);
        glGenVertexArrays(1, &m_fullscreenVao);
        m_initialized = m_idShader && m_outlineShader && m_fullscreenVao != 0;
    } catch(const std::exception& error) {
        std::cerr << "Failed to initialize scene hover GPU pass: " << error.what() << "\n";
        m_initialized = false;
    }
    return m_initialized;
}

bool SceneObjectHoverPass::ensureFramebuffer()
{
    if(!m_framebufferDirty && m_framebuffer != 0) {
        return true;
    }

    if(m_idTexture != 0) {
        glDeleteTextures(1, &m_idTexture);
        m_idTexture = 0;
    }
    if(m_depthRenderbuffer != 0) {
        glDeleteRenderbuffers(1, &m_depthRenderbuffer);
        m_depthRenderbuffer = 0;
    }
    if(m_framebuffer != 0) {
        glDeleteFramebuffers(1, &m_framebuffer);
        m_framebuffer = 0;
    }

    glGenFramebuffers(1, &m_framebuffer);
    glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer);

    glGenTextures(1, &m_idTexture);
    glBindTexture(GL_TEXTURE_2D, m_idTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexImage2D(
        GL_TEXTURE_2D,
        0,
        GL_R32UI,
        m_width,
        m_height,
        0,
        GL_RED_INTEGER,
        GL_UNSIGNED_INT,
        nullptr);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_idTexture, 0);

    glGenRenderbuffers(1, &m_depthRenderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, m_depthRenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT24, m_width, m_height);
    glFramebufferRenderbuffer(
        GL_FRAMEBUFFER,
        GL_DEPTH_ATTACHMENT,
        GL_RENDERBUFFER,
        m_depthRenderbuffer);

    const GLenum drawBuffer = GL_COLOR_ATTACHMENT0;
    glDrawBuffers(1, &drawBuffer);
    const bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    m_framebufferDirty = !complete;
    return complete;
}

void SceneObjectHoverPass::destroyResources()
{
    if(glad_glGetString == nullptr || glad_glGetString(GL_VERSION) == nullptr) {
        return;
    }
    if(m_fullscreenVao != 0) {
        glDeleteVertexArrays(1, &m_fullscreenVao);
        m_fullscreenVao = 0;
    }
    if(m_idTexture != 0) {
        glDeleteTextures(1, &m_idTexture);
        m_idTexture = 0;
    }
    if(m_depthRenderbuffer != 0) {
        glDeleteRenderbuffers(1, &m_depthRenderbuffer);
        m_depthRenderbuffer = 0;
    }
    if(m_framebuffer != 0) {
        glDeleteFramebuffers(1, &m_framebuffer);
        m_framebuffer = 0;
    }
}

void SceneObjectHoverPass::renderIdBuffer()
{
    m_objectIds.clear();
    m_objectIds.push_back(std::string());

    m_idShader->use();
    m_idShader->setMat4("uProjection", m_projection);
    m_idShader->setMat4("uView", m_view);

    // Non-selectable visible geometry writes ID 0 so it still occludes scene objects.
    if(m_robots != nullptr) {
        for(const RuntimeRobot& robot : *m_robots) {
            if(!robot.visualBridge || !robot.visualBridge->rootNode() ||
                !robot.visualBridge->rootNode()->visible()) {
                continue;
            }
            for(const std::string& linkName : robot.model.linkNames) {
                const auto linkNode = robot.visualBridge->linkNode(linkName);
                if(!linkNode || !linkNode->visible()) {
                    continue;
                }
                for(const auto& node : robot.visualBridge->visualNodes(linkName)) {
                    if(node && node->visible()) {
                        drawRenderModel(node->model(), node->world(), m_idShader, 0);
                    }
                }
            }
        }
    }

    if(m_objects == nullptr) {
        return;
    }
    for(const RuntimeSceneObject& object : *m_objects) {
        if(object.objectType == "pointCloud" || object.objectType == "toolAttachment" ||
            !object.visualNode || !object.visualNode->isVisible() || !object.visualModel) {
            continue;
        }

        const int objectId = static_cast<int>(m_objectIds.size());
        m_objectIds.push_back(object.documentId);
        drawRenderModel(object.visualModel, object.visualNode->world(), m_idShader, objectId);
    }
}

void SceneObjectHoverPass::drawOutline(unsigned int hoveredId)
{
    if(hoveredId == 0 || !m_outlineShader || m_idTexture == 0) {
        return;
    }

    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    m_outlineShader->use();
    m_outlineShader->setInt("uObjectIds", 0);
    m_outlineShader->setInt("uHoveredId", static_cast<int>(hoveredId));
    m_outlineShader->setFloat("uOutlineRadius", m_outlineRadius);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_idTexture);
    glBindVertexArray(m_fullscreenVao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
}

float SceneObjectHoverPass::outlineRadiusForObject(const std::string& objectId)
{
    constexpr float kRatio = 0.004f;
    constexpr float kMinRadius = 1.0f;
    constexpr float kMaxRadius = 4.0f;

    if(m_objects == nullptr || objectId.empty()) {
        return kMinRadius;
    }

    glm::vec3 worldMin(std::numeric_limits<float>::max());
    glm::vec3 worldMax(std::numeric_limits<float>::lowest());
    bool hasBounds = false;

    for(const RuntimeSceneObject& object : *m_objects) {
        if(object.documentId != objectId) {
            continue;
        }

        if(object.pickModel != nullptr && object.visualNode != nullptr) {
            CachedLocalBounds& cached = m_localBoundsCache[object.documentId];
            if(cached.model != object.pickModel.get()) {
                cached = CachedLocalBounds{};
                cached.model = object.pickModel.get();
                cached.minimum = glm::vec3(std::numeric_limits<float>::max());
                cached.maximum = glm::vec3(std::numeric_limits<float>::lowest());
                for(const auto& subMesh : object.pickModel->subMeshes()) {
                    for(const auto& position : subMesh.geometry.positions) {
                        const glm::vec3 point(position.x(), position.y(), position.z());
                        cached.minimum = glm::min(cached.minimum, point);
                        cached.maximum = glm::max(cached.maximum, point);
                        cached.valid = true;
                    }
                }
            }

            if(cached.valid) {
                const glm::mat4 world = object.visualNode->world();
                for(int corner = 0; corner < 8; ++corner) {
                    const glm::vec3 point(
                        (corner & 1) != 0 ? cached.maximum.x : cached.minimum.x,
                        (corner & 2) != 0 ? cached.maximum.y : cached.minimum.y,
                        (corner & 4) != 0 ? cached.maximum.z : cached.minimum.z);
                    const glm::vec3 worldPoint = glm::vec3(world * glm::vec4(point, 1.0f));
                    worldMin = glm::min(worldMin, worldPoint);
                    worldMax = glm::max(worldMax, worldPoint);
                }
                hasBounds = true;
            }
        } else if(object.collisionObject != nullptr && object.collisionObject->fcl() != nullptr) {
            const auto& aabb = object.collisionObject->fcl()->getAABB();
            worldMin = glm::vec3(
                static_cast<float>(aabb.min_.x()),
                static_cast<float>(aabb.min_.y()),
                static_cast<float>(aabb.min_.z()));
            worldMax = glm::vec3(
                static_cast<float>(aabb.max_.x()),
                static_cast<float>(aabb.max_.y()),
                static_cast<float>(aabb.max_.z()));
            hasBounds = true;
        }
        break;
    }

    if(!hasBounds) {
        return kMinRadius;
    }

    const std::array<glm::vec3, 8> corners = {
        glm::vec3(worldMin.x, worldMin.y, worldMin.z),
        glm::vec3(worldMax.x, worldMin.y, worldMin.z),
        glm::vec3(worldMin.x, worldMax.y, worldMin.z),
        glm::vec3(worldMax.x, worldMax.y, worldMin.z),
        glm::vec3(worldMin.x, worldMin.y, worldMax.z),
        glm::vec3(worldMax.x, worldMin.y, worldMax.z),
        glm::vec3(worldMin.x, worldMax.y, worldMax.z),
        glm::vec3(worldMax.x, worldMax.y, worldMax.z) };

    float minX = static_cast<float>(m_width);
    float maxX = 0.0f;
    float minY = static_cast<float>(m_height);
    float maxY = 0.0f;
    bool projected = false;
    const glm::mat4 viewProjection = m_projection * m_view;
    for(const glm::vec3& corner : corners) {
        const glm::vec4 clip = viewProjection * glm::vec4(corner, 1.0f);
        if(clip.w <= 0.0001f) {
            continue;
        }
        const glm::vec2 pixel = glm::vec2(clip) / clip.w;
        const float screenX = (pixel.x * 0.5f + 0.5f) * static_cast<float>(m_width);
        const float screenY = (pixel.y * 0.5f + 0.5f) * static_cast<float>(m_height);
        minX = std::min(minX, screenX);
        maxX = std::max(maxX, screenX);
        minY = std::min(minY, screenY);
        maxY = std::max(maxY, screenY);
        projected = true;
    }

    if(!projected) {
        return kMinRadius;
    }

    const float projectedSize = std::max(maxX - minX, maxY - minY);
    return std::clamp(projectedSize * kRatio, kMinRadius, kMaxRadius);
}

void SceneObjectHoverPass::render(scenecore::RenderQueue& queue)
{
    (void)queue;
    if(!ensureInitialized()) {
        return;
    }

    GLint previousDrawFramebuffer = 0;
    GLint previousReadFramebuffer = 0;
    GLint previousViewport[4] = { 0, 0, 1, 1 };
    GLint previousProgram = 0;
    GLint previousVao = 0;
    GLint previousActiveTexture = GL_TEXTURE0;
    GLint previousTexture = 0;
    GLboolean previousDepthMask = GL_TRUE;
    const GLboolean depthEnabled = glIsEnabled(GL_DEPTH_TEST);
    const GLboolean cullEnabled = glIsEnabled(GL_CULL_FACE);
    const GLboolean blendEnabled = glIsEnabled(GL_BLEND);
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &previousDrawFramebuffer);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &previousReadFramebuffer);
    glGetIntegerv(GL_VIEWPORT, previousViewport);
    glGetIntegerv(GL_CURRENT_PROGRAM, &previousProgram);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &previousVao);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &previousActiveTexture);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &previousTexture);
    glGetBooleanv(GL_DEPTH_WRITEMASK, &previousDepthMask);

    unsigned int hoveredId = 0;
    if(m_pickPending && ensureFramebuffer()) {
        glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer);
        glViewport(0, 0, m_width, m_height);
        const GLuint clearId = 0;
        const GLfloat clearDepth = 1.0f;
        glClearBufferuiv(GL_COLOR, 0, &clearId);
        glClearBufferfv(GL_DEPTH, 0, &clearDepth);
        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);
        renderIdBuffer();

        if(m_pickX >= 0 && m_pickX < m_width && m_pickY >= 0 && m_pickY < m_height) {
            glReadBuffer(GL_COLOR_ATTACHMENT0);
            glReadPixels(
                m_pickX,
                m_height - 1 - m_pickY,
                1,
                1,
                GL_RED_INTEGER,
                GL_UNSIGNED_INT,
                &hoveredId);
        }
        m_pickPending = false;
        if(hoveredId < m_objectIds.size()) {
            m_hoveredObjectId = m_objectIds[hoveredId];
            m_outlineRadius = outlineRadiusForObject(m_hoveredObjectId);
        } else {
            m_hoveredObjectId.clear();
            hoveredId = 0;
        }
        if(m_hoveredObjectId == m_selectedObjectId) {
            hoveredId = 0;
        }
    } else if(!m_hoveredObjectId.empty()) {
        const auto found = std::find(m_objectIds.begin(), m_objectIds.end(), m_hoveredObjectId);
        if(found != m_objectIds.end() && m_hoveredObjectId != m_selectedObjectId) {
            hoveredId = static_cast<unsigned int>(std::distance(m_objectIds.begin(), found));
        }
    }

    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, static_cast<GLuint>(previousDrawFramebuffer));
    glBindFramebuffer(GL_READ_FRAMEBUFFER, static_cast<GLuint>(previousReadFramebuffer));
    glViewport(previousViewport[0], previousViewport[1], previousViewport[2], previousViewport[3]);
    drawOutline(hoveredId);

    glUseProgram(static_cast<GLuint>(previousProgram));
    glBindVertexArray(static_cast<GLuint>(previousVao));
    glActiveTexture(static_cast<GLenum>(previousActiveTexture));
    glBindTexture(GL_TEXTURE_2D, static_cast<GLuint>(previousTexture));
    glDepthMask(previousDepthMask);
    depthEnabled ? glEnable(GL_DEPTH_TEST) : glDisable(GL_DEPTH_TEST);
    cullEnabled ? glEnable(GL_CULL_FACE) : glDisable(GL_CULL_FACE);
    blendEnabled ? glEnable(GL_BLEND) : glDisable(GL_BLEND);
}
