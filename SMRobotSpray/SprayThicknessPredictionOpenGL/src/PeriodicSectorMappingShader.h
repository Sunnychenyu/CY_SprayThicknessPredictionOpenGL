#pragma once

namespace spraythickness::opengl
{
    inline constexpr const char* kPeriodicSectorMappingShader = R"GLSL(#version 430
layout(local_size_x = 256) in;

struct BvhNode {
    vec4 minimum;
    vec4 maximum;
    int leftChild;
    int rightChild;
    int firstTriangle;
    int triangleCount;
};

layout(std430, binding = 0) readonly buffer FullSurfaceBuffer {
    vec4 fullSurfaceData[];
};
layout(std430, binding = 1) readonly buffer LocalSurfaceBuffer {
    vec4 localSurfaceData[];
};
layout(std430, binding = 2) readonly buffer LocalIndexBuffer {
    uint localTriangleIndices[];
};
layout(std430, binding = 3) readonly buffer BvhBuffer {
    BvhNode bvhNodes[];
};
layout(std430, binding = 4) readonly buffer BvhOrderBuffer {
    uint leafTriangleIndices[];
};
layout(std430, binding = 5) readonly buffer LocalToGlobalBuffer {
    uint localToGlobal[];
};
layout(std430, binding = 6) writeonly buffer BindingIndexBuffer {
    uvec4 bindingIndices[];
};
layout(std430, binding = 7) writeonly buffer BindingWeightBuffer {
    vec4 bindingWeightsDistance[];
};

uniform int vertexCount;
uniform float minimumNormalDot;

vec3 safeNormalize(vec3 value, vec3 fallback)
{
    float lengthSquared = dot(value, value);
    return lengthSquared > 1.0e-12
        ? value * inversesqrt(lengthSquared)
        : fallback;
}

float squaredDistanceToBox(vec3 point, vec3 minimum, vec3 maximum)
{
    vec3 lower = max(minimum - point, vec3(0.0));
    vec3 upper = max(point - maximum, vec3(0.0));
    vec3 distance = lower + upper;
    return dot(distance, distance);
}

vec3 closestPointWeights(vec3 point, vec3 a, vec3 b, vec3 c)
{
    vec3 ab = b - a;
    vec3 ac = c - a;
    vec3 ap = point - a;
    float d1 = dot(ab, ap);
    float d2 = dot(ac, ap);
    if(d1 <= 0.0 && d2 <= 0.0) return vec3(1.0, 0.0, 0.0);

    vec3 bp = point - b;
    float d3 = dot(ab, bp);
    float d4 = dot(ac, bp);
    if(d3 >= 0.0 && d4 <= d3) return vec3(0.0, 1.0, 0.0);

    float vc = d1 * d4 - d3 * d2;
    if(vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0) {
        float v = d1 / max(d1 - d3, 1.0e-20);
        return vec3(1.0 - v, v, 0.0);
    }

    vec3 cp = point - c;
    float d5 = dot(ab, cp);
    float d6 = dot(ac, cp);
    if(d6 >= 0.0 && d5 <= d6) return vec3(0.0, 0.0, 1.0);

    float vb = d5 * d2 - d1 * d6;
    if(vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0) {
        float w = d2 / max(d2 - d6, 1.0e-20);
        return vec3(1.0 - w, 0.0, w);
    }

    float va = d3 * d6 - d5 * d4;
    if(va <= 0.0 && d4 - d3 >= 0.0 && d5 - d6 >= 0.0) {
        float w = (d4 - d3) / max((d4 - d3) + (d5 - d6), 1.0e-20);
        return vec3(0.0, 1.0 - w, w);
    }

    float denominator = 1.0 / max(va + vb + vc, 1.0e-20);
    float v = vb * denominator;
    float w = vc * denominator;
    return vec3(1.0 - v - w, v, w);
}

void writeInvalid(uint vertex)
{
    bindingIndices[vertex] = uvec4(0u, 0u, 0u, 0u);
    bindingWeightsDistance[vertex] = vec4(0.0, 0.0, 0.0, -1.0);
}

void main()
{
    uint vertex = gl_GlobalInvocationID.x;
    if(vertex >= uint(vertexCount)) return;

    vec3 queryPosition = fullSurfaceData[vertex * 2u].xyz;
    vec3 queryNormal = safeNormalize(
        fullSurfaceData[vertex * 2u + 1u].xyz,
        vec3(0.0, 0.0, 1.0));

    int stack[128];
    int stackSize = 1;
    stack[0] = 0;
    float closestSquaredDistance = 1.0e30;
    uvec3 closestIndices = uvec3(0u);
    vec3 closestWeights = vec3(0.0);
    bool found = false;
    bool overflow = false;

    while(stackSize > 0) {
        int nodeIndex = stack[--stackSize];
        if(nodeIndex < 0 || nodeIndex >= bvhNodes.length()) continue;
        BvhNode node = bvhNodes[nodeIndex];
        if(squaredDistanceToBox(
            queryPosition, node.minimum.xyz, node.maximum.xyz)
            > closestSquaredDistance) {
            continue;
        }

        if(node.triangleCount > 0) {
            for(int offset = 0; offset < node.triangleCount; ++offset) {
                int orderIndex = node.firstTriangle + offset;
                if(orderIndex < 0 || orderIndex >= leafTriangleIndices.length()) continue;
                uint triangle = leafTriangleIndices[orderIndex];
                uint base = triangle * 3u;
                if(base + 2u >= localTriangleIndices.length()) continue;
                uint ia = localTriangleIndices[base];
                uint ib = localTriangleIndices[base + 1u];
                uint ic = localTriangleIndices[base + 2u];
                if(ia >= localToGlobal.length() ||
                    ib >= localToGlobal.length() ||
                    ic >= localToGlobal.length()) continue;

                vec3 a = localSurfaceData[ia * 2u].xyz;
                vec3 b = localSurfaceData[ib * 2u].xyz;
                vec3 c = localSurfaceData[ic * 2u].xyz;
                vec3 weights = closestPointWeights(queryPosition, a, b, c);
                vec3 interpolatedNormal = safeNormalize(
                    weights.x * localSurfaceData[ia * 2u + 1u].xyz
                    + weights.y * localSurfaceData[ib * 2u + 1u].xyz
                    + weights.z * localSurfaceData[ic * 2u + 1u].xyz,
                    vec3(0.0));
                if(dot(interpolatedNormal, interpolatedNormal) <= 1.0e-12 ||
                    dot(interpolatedNormal, queryNormal) < minimumNormalDot) {
                    continue;
                }
                vec3 closestPoint = weights.x * a + weights.y * b + weights.z * c;
                float squaredDistance = dot(queryPosition - closestPoint,
                    queryPosition - closestPoint);
                if(squaredDistance >= closestSquaredDistance) continue;
                closestSquaredDistance = squaredDistance;
                closestIndices = uvec3(
                    localToGlobal[ia], localToGlobal[ib], localToGlobal[ic]);
                closestWeights = weights;
                found = true;
            }
        } else {
            if(stackSize + 2 > 128) {
                overflow = true;
                break;
            }
            if(node.rightChild >= 0 && node.rightChild < bvhNodes.length()) {
                stack[stackSize++] = node.rightChild;
            }
            if(node.leftChild >= 0 && node.leftChild < bvhNodes.length()) {
                stack[stackSize++] = node.leftChild;
            }
        }
    }

    if(overflow || !found) {
        if(overflow) {
            bindingIndices[vertex] = uvec4(0u, 0u, 0u, 2u);
            bindingWeightsDistance[vertex] = vec4(0.0, 0.0, 0.0, -1.0);
        } else {
            writeInvalid(vertex);
        }
        return;
    }
    bindingIndices[vertex] = uvec4(
        closestIndices.x, closestIndices.y, closestIndices.z, 1u);
    bindingWeightsDistance[vertex] = vec4(
        closestWeights, sqrt(max(closestSquaredDistance, 0.0)));
}
)GLSL";
}
