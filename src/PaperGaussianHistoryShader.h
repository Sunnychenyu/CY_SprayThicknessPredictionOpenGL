#pragma once

#include <string>

namespace spraythickness::opengl
{
    inline constexpr const char* kPaperGaussianHistoryShader = R"GLSL(#version 430
#ifndef THICKNESS_ENABLE_BVH
#define THICKNESS_ENABLE_BVH 1
#endif
#ifndef THICKNESS_ENABLE_HISTORY
#define THICKNESS_ENABLE_HISTORY 1
#endif
layout(local_size_x = 256) in;

struct BvhNode {
    vec4 minimum;
    vec4 maximum;
    int leftChild;
    int rightChild;
    int padding[2];
};

layout(std430, binding = 0) readonly buffer SurfaceBuffer { vec4 surfaceData[]; };
layout(std430, binding = 1) buffer ThicknessBuffer { float thickness[]; };
layout(std430, binding = 2) readonly buffer SprayBuffer { vec4 sprayData[]; };
layout(std430, binding = 3) readonly buffer IndexBuffer { uint triangleIndices[]; };
layout(std430, binding = 4) readonly buffer BvhBuffer { BvhNode bvhNodes[]; };
layout(std430, binding = 5) readonly buffer BvhOrderBuffer { uint leafTriangleIndices[]; };
layout(std430, binding = 6) buffer HistoryBuffer { float historyTau[]; };
layout(std430, binding = 7) buffer LastTimeBuffer { float lastUpdateTime[]; };
layout(std430, binding = 8) buffer FactorBuffer { float historyFactor[]; };
layout(std430, binding = 9) readonly buffer PredictionVertexBuffer { uint predictionVertices[]; };
layout(std430, binding = 10) readonly buffer SurfacePositionBuffer { vec4 surfacePositions[]; };

uniform int vertexCount;
uniform int predictionVertexCount;
uniform bool usePredictionVertexBuffer;
uniform int sprayPointCount;
uniform bool enableBvh;
uniform bool enableHistory;
uniform float shadowBiasMm;
uniform float amplitudeMm;
uniform float patternRotation;
uniform float phiOffset;
uniform float psiOffset;
uniform float sigmaPhi;
uniform float sigmaPsi;
uniform float referenceDistanceMm;
uniform float referenceAngleDegrees;
uniform float referenceExposureSeconds;
uniform float processScale;
uniform float historyAmplitude;
uniform float historyScaleSeconds;
uniform float referenceHistorySeconds;
uniform float coolingTimeSeconds;
uniform float activityThresholdRatio;
uniform float patternCosine;
uniform float patternSine;
uniform float inverseSigmaPhi;
uniform float inverseSigmaPsi;
uniform float referenceProjection;
uniform float referenceDistanceEfficiency;
uniform float referenceAngleEfficiency;
uniform float referenceDistanceSquared;
uniform float peakScale;

vec3 safeNormalize(vec3 value, vec3 fallback)
{
    float lengthSquared = dot(value, value);
    return lengthSquared > 1.0e-12
        ? value * inversesqrt(lengthSquared)
        : fallback;
}

#if THICKNESS_ENABLE_BVH
bool rayBox(
    vec3 origin,
    vec3 direction,
    vec3 minimum,
    vec3 maximum,
    float rayLength)
{
    float nearDistance = 0.0;
    float farDistance = rayLength;
    for(int axis = 0; axis < 3; ++axis) {
        if(direction[axis] == 0.0) {
            if(origin[axis] < minimum[axis] || origin[axis] > maximum[axis]) {
                return false;
            }
            continue;
        }
        float t0 = (minimum[axis] - origin[axis]) / direction[axis];
        float t1 = (maximum[axis] - origin[axis]) / direction[axis];
        nearDistance = max(nearDistance, min(t0, t1));
        farDistance = min(farDistance, max(t0, t1));
        if(nearDistance >= farDistance) return false;
    }
    return farDistance > shadowBiasMm
        && nearDistance < farDistance
        && nearDistance < rayLength;
}

bool rayTriangle(vec3 origin, vec3 direction, vec3 a, vec3 b, vec3 c, out float distance) {
    vec3 edge1 = b - a;
    vec3 edge2 = c - a;
    vec3 p = cross(direction, edge2);
    float determinant = dot(edge1, p);
    if(abs(determinant) < 1e-7) return false;
    float inverseDeterminant = 1.0 / determinant;
    vec3 offset = origin - a;
    float u = dot(offset, p) * inverseDeterminant;
    if(u < 0.0 || u > 1.0) return false;
    vec3 q = cross(offset, edge1);
    float v = dot(direction, q) * inverseDeterminant;
    if(v < 0.0 || u + v > 1.0) return false;
    distance = dot(edge2, q) * inverseDeterminant;
    return distance > shadowBiasMm;
}

bool isOccluded(
    vec3 vertexPosition,
    vec3 rayDirection,
    float rayLength)
{
    if(bvhNodes.length() == 0) return false;
    int stack[128];
    int stackSize = 1;
    stack[0] = 0;
    while(stackSize > 0) {
        int nodeIndex = stack[--stackSize];
        if(nodeIndex < 0 || nodeIndex >= bvhNodes.length()) continue;
        BvhNode node = bvhNodes[nodeIndex];
        if(!rayBox(vertexPosition, rayDirection, node.minimum.xyz, node.maximum.xyz, rayLength)) continue;
        if(node.leftChild < 0) {
            int leafStart = -(node.leftChild + 1);
            int leafTriangleCount = node.rightChild;
            if(leafStart < 0 || leafTriangleCount < 0 ||
                leafStart + leafTriangleCount > leafTriangleIndices.length()) {
                continue;
            }
            for(int i = 0; i < leafTriangleCount; ++i) {
                uint triangleFaceStart = leafTriangleIndices[leafStart + i];
                if(triangleFaceStart + 2u >= triangleIndices.length()) continue;
                uint ia = triangleIndices[triangleFaceStart];
                uint ib = triangleIndices[triangleFaceStart + 1u];
                uint ic = triangleIndices[triangleFaceStart + 2u];
                float hitDistance = 0.0;
                if(rayTriangle(vertexPosition, rayDirection,
                    surfacePositions[ia].xyz,
                    surfacePositions[ib].xyz,
                    surfacePositions[ic].xyz,
                    hitDistance) && hitDistance < rayLength - shadowBiasMm) {
                    return true;
                }
            }
        } else {
            if(stackSize < 126) {
                if(node.rightChild >= 0 && node.rightChild < bvhNodes.length()) {
                    stack[stackSize++] = node.rightChild;
                }
                if(node.leftChild >= 0 && node.leftChild < bvhNodes.length()) {
                    stack[stackSize++] = node.leftChild;
                }
            }
        }
    }
    return false;
}
#endif

float distanceEfficiency(float distanceMm) {
    float x = max(distanceMm, 1e-6);
    float normalizedLogDistance = log(x / 106.8615) / 0.0839;
    return 0.9789 + 0.1456 * exp(-(normalizedLogDistance * normalizedLogDistance));
}

float angleEfficiency(float angleDegrees) {
    return max(1e-6, (40.81414 + 0.73808 * angleDegrees
        - 0.000838598 * angleDegrees * angleDegrees) * 0.01);
}

float patternValue(float uMm, float vMm, float axialDistanceMm) {
    float phi = atan(uMm, max(axialDistanceMm, 1e-6)) - phiOffset;
    float psi = atan(vMm, max(axialDistanceMm, 1e-6)) - psiOffset;
    float rotatedPhi = patternCosine * phi + patternSine * psi;
    float rotatedPsi = -patternSine * phi + patternCosine * psi;
    float normalizedPhi = rotatedPhi * inverseSigmaPhi;
    float normalizedPsi = rotatedPsi * inverseSigmaPsi;
    return exp(-0.5 * (normalizedPhi * normalizedPhi
        + normalizedPsi * normalizedPsi));
}

float correctionFactor(float tau) {
    float scale = max(historyScaleSeconds, 1e-6);
    return max(1.0, 1.0 + historyAmplitude
        * (exp(-tau / scale) - exp(-referenceHistorySeconds / scale)));
}

void main() {
    uint predictionVertex = gl_GlobalInvocationID.x;
    if(predictionVertex >= uint(predictionVertexCount)) return;
    uint vertex = usePredictionVertexBuffer
        ? predictionVertices[predictionVertex]
        : predictionVertex;
    if(vertex >= uint(vertexCount)) return;
    vec3 position = surfaceData[vertex * 2u].xyz;
    vec3 normal = safeNormalize(
        surfaceData[vertex * 2u + 1u].xyz,
        vec3(0.0, 0.0, 1.0));
    float accumulated = 0.0;
    float tau = historyTau[vertex];
    float lastTime = lastUpdateTime[vertex];
    float factor = historyFactor[vertex];

    for(int point = 0; point < sprayPointCount; ++point) {
        int base = point * 5;
        vec3 sprayPosition = sprayData[base].xyz;
        float dt = max(0.0, sprayData[base].w);
        vec3 sprayDirection = sprayData[base + 1].xyz;
        vec3 majorAxis = sprayData[base + 2].xyz;
        vec3 minorAxis = sprayData[base + 3].xyz;
        float currentTime = sprayData[base + 4].x;
        vec3 sprayToVertex = position - sprayPosition;
        float axialDistance = dot(sprayToVertex, sprayDirection);
        if(axialDistance <= 0.001) continue;
        float sprayDistance = length(sprayToVertex);
        vec3 vertexToSprayDirection = -sprayToVertex / max(sprayDistance, 1.0e-6);
        float cosineIncidence = dot(normal, vertexToSprayDirection);
        if(cosineIncidence <= 0.0) continue;
#if THICKNESS_ENABLE_BVH
        if(isOccluded(
            position,
            vertexToSprayDirection,
            sprayDistance)) continue;
#endif

        float angleDegrees = 90.0 - degrees(acos(clamp(cosineIncidence, -1.0, 1.0)));
        float projection = max(1e-6, cosineIncidence);
        float distanceSquared = max(dot(sprayToVertex, sprayToVertex), 1.0e-12);
        float geometryScale = referenceDistanceSquared / distanceSquared
            * projection / referenceProjection;
        float distanceScale = distanceEfficiency(sprayDistance)
            / referenceDistanceEfficiency;
        float angleScale = angleEfficiency(angleDegrees) / referenceAngleEfficiency;
        float peak = peakScale * geometryScale * distanceScale * angleScale
            * max(1e-6, dt);
        float baseThickness = peak * patternValue(
            dot(sprayToVertex, majorAxis),
            dot(sprayToVertex, minorAxis),
            axialDistance);
        if(baseThickness <= 0.0) continue;

#if THICKNESS_ENABLE_HISTORY
        {
            tau *= exp(-max(0.0, currentTime - lastTime) / max(coolingTimeSeconds, 1e-6));
            if(baseThickness >= activityThresholdRatio * peak) tau += dt;
            factor = correctionFactor(tau);
        }
#else
        factor = 1.0;
#endif
        accumulated += baseThickness * factor;
        lastTime = currentTime;
    }

    thickness[vertex] += accumulated;
    historyTau[vertex] = tau;
    lastUpdateTime[vertex] = lastTime;
    historyFactor[vertex] = factor;
}
)GLSL";

    inline std::string makePaperGaussianHistoryShader(
        bool enableBvh,
        bool enableHistory)
    {
        std::string source(kPaperGaussianHistoryShader);
        const std::size_t insertion = source.find('\n');
        if(insertion == std::string::npos) {
            return source;
        }
        source.insert(
            insertion + 1,
            std::string("#define THICKNESS_ENABLE_BVH ")
                + (enableBvh ? "1\n" : "0\n")
                + "#define THICKNESS_ENABLE_HISTORY "
                + (enableHistory ? "1\n" : "0\n"));
        return source;
    }
}
