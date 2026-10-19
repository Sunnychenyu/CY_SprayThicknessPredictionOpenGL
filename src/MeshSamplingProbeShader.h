#pragma once

#include "PaperGaussianHistoryShader.h"

namespace spraythickness::opengl
{
    inline std::string makeMeshSamplingProbeShader()
    {
        std::string source = makePaperGaussianHistoryShader(true, false, true);
        const std::string mainName = "void main() {";
        source.replace(source.find(mainName), mainName.size(), "void unusedThicknessMain() {");
        const std::string localSize = "local_size_x = 256";
        source.replace(source.find(localSize), localSize.size(), "local_size_x = 64");
        source += R"GLSL(
uniform int meshProbeBegin;
uniform int meshProbeCount;
uniform vec4 meshSlopeBounds; // min phi, max phi, min psi, max psi slopes

vec3 meshPlane(int p)
{
    if(p == 0) return vec3(0, 0, 1);
    if(p == 1) return vec3(1, 0, -meshSlopeBounds.x);
    if(p == 2) return vec3(-1, 0, meshSlopeBounds.y);
    if(p == 3) return vec3(0, 1, -meshSlopeBounds.z);
    return vec3(0, -1, meshSlopeBounds.w);
}

vec3 meshLocal(vec3 delta, vec3 major, vec3 minor, vec3 forward)
{
    return vec3(dot(delta, major), dot(delta, minor), dot(delta, forward));
}

bool meshBoundsHit(vec3 origin, vec3 direction, BvhNode node, float limit)
{
    vec3 padding = vec3(1e-5) + max(abs(node.minimum.xyz), abs(node.maximum.xyz)) * 2e-7;
    float lo = 0.0, hi = limit;
    for(int axis = 0; axis < 3; ++axis) {
        float a = node.minimum[axis] - padding[axis];
        float b = node.maximum[axis] + padding[axis];
        if(abs(direction[axis]) < 1e-15) {
            if(origin[axis] < a || origin[axis] > b) return false;
        } else {
            float t0 = (a - origin[axis]) / direction[axis];
            float t1 = (b - origin[axis]) / direction[axis];
            lo = max(lo, min(t0, t1));
            hi = min(hi, max(t0, t1));
            if(lo > hi) return false;
        }
    }
    return true;
}

// Match the CPU diagnostic's closest-hit policy, including triangle seams.
int meshFirstHit(vec3 origin, vec3 direction)
{
    float closest = 3.402823e38;
    int result = 0;
    int stack[128]; int size = 1; stack[0] = 0;
    while(size > 0) {
        BvhNode node = bvhNodes[stack[--size]];
        if(!meshBoundsHit(origin, direction, node, closest)) continue;
        if(node.leftChild >= 0) {
            if(size + 2 > 128) return 128;
            stack[size++] = node.leftChild; stack[size++] = node.rightChild;
            continue;
        }
        int start = -(node.leftChild + 1);
        for(int j = 0; j < node.rightChild; ++j) {
            uint face = leafTriangleIndices[start + j];
            uvec3 ids = uvec3(triangleIndices[face], triangleIndices[face + 1u], triangleIndices[face + 2u]);
            vec3 a = surfacePositions[ids.x].xyz;
            vec3 e1 = surfacePositions[ids.y].xyz - a;
            vec3 e2 = surfacePositions[ids.z].xyz - a;
            vec3 p = cross(direction, e2);
            float determinant = dot(e1, p);
            if(abs(determinant) <= 1e-7 * length(e1) * length(e2)) continue;
            vec3 offset = origin - a;
            float u = dot(offset, p) / determinant;
            vec3 q = cross(offset, e1);
            float v = dot(direction, q) / determinant;
            if(u < -1e-6 || v < -1e-6 || u + v > 1.000001) continue;
            float t = dot(e2, q) / determinant;
            if(t <= 1e-7 || t > closest) continue;
            closest = t;
            vec3 normal = surfaceData[ids.x * 2u + 1u].xyz * (1.0 - u - v)
                + surfaceData[ids.y * 2u + 1u].xyz * u + surfaceData[ids.z * 2u + 1u].xyz * v;
            if(dot(normal, normal) < 1e-20) normal = cross(e1, e2);
            result = dot(normal, -direction) > 0.0 ? 32 : 64;
        }
    }
    return result;
}

float meshAngularSquared(vec2 slopes)
{
    vec2 coordinate = patternCoordinates(slopes.x, slopes.y, 1.0);
    return dot(coordinate, coordinate);
}

// Clip against the enclosing pyramid, then test the exact angular Gaussian.
// Testing only a fixed set of rays would miss a small triangle at the edge.
bool meshTriangleDirection(vec3 a, vec3 b, vec3 c, out vec3 direction)
{
    vec3 polygon[12]; vec3 clipped[12];
    polygon[0] = a; polygon[1] = b; polygon[2] = c;
    int count = 3;
    for(int p = 0; p < 5; ++p) {
        vec3 plane = meshPlane(p);
        int nextCount = 0;
        for(int i = 0; i < count; ++i) {
            vec3 va = polygon[i], vb = polygon[(i + 1) % count];
            float da = dot(plane, va) - (p == 0 ? 1e-7 : 0.0);
            float db = dot(plane, vb) - (p == 0 ? 1e-7 : 0.0);
            if(da >= 0.0) clipped[nextCount++] = va;
            if((da >= 0.0) != (db >= 0.0)) clipped[nextCount++] = mix(va, vb, da / (da - db));
        }
        count = nextCount;
        if(count == 0) return false;
        for(int i = 0; i < count; ++i) polygon[i] = clipped[i];
    }
    vec2 peak = tan(vec2(phiOffset, psiOffset));
    bool positive = false, negative = false;
    for(int i = 0; i < count; ++i) {
        vec2 va = polygon[i].xy / polygon[i].z;
        vec2 edge = polygon[(i + 1) % count].xy / polygon[(i + 1) % count].z - va;
        vec2 delta = peak - va;
        float side = edge.x * delta.y - edge.y * delta.x;
        positive = positive || side > 1e-10;
        negative = negative || side < -1e-10;
    }
    if(count >= 3 && (positive || negative) && !(positive && negative)) {
        direction = vec3(peak, 1.0); return true;
    }
    for(int i = 0; i < count; ++i) {
        vec2 va = polygon[i].xy / polygon[i].z;
        vec2 edge = polygon[(i + 1) % count].xy / polygon[(i + 1) % count].z - va;
        for(int bracket = 0; bracket < 8; ++bracket) {
            float lo = float(bracket) / 8.0, hi = float(bracket + 1) / 8.0;
            for(int iteration = 0; iteration < 20; ++iteration) {
                float left = (2.0 * lo + hi) / 3.0, right = (lo + 2.0 * hi) / 3.0;
                if(meshAngularSquared(va + left * edge) < meshAngularSquared(va + right * edge)) hi = right;
                else lo = left;
            }
            for(int test = 0; test < 3; ++test) {
                float t = test == 0 ? float(bracket) / 8.0
                    : (test == 1 ? (lo + hi) * 0.5 : float(bracket + 1) / 8.0);
                vec2 candidate = va + t * edge;
                if(meshAngularSquared(candidate) <= meshCoreSquared + 1e-5) {
                    direction = vec3(candidate, 1.0); return true;
                }
            }
        }
    }
    return false;
}

void main()
{
    int point = meshProbeBegin + int(gl_GlobalInvocationID.x);
    if(point >= meshProbeBegin + meshProbeCount) return;
    int record = meshRecord(point);
    if(meshSamplingData[record] > 0) return;
    int base = point * 5;
    vec3 origin = sprayData[base].xyz;
    vec3 forward = sprayData[base + 1].xyz;
    vec3 major = sprayData[base + 2].xyz;
    vec3 minor = sprayData[base + 3].xyz;
    vec2 peak = tan(vec2(phiOffset, psiOffset));
    int flags = meshFirstHit(origin, normalize(major * peak.x + minor * peak.y + forward));
    if(flags == 32 || flags == 128) { meshSamplingData[record + 1] |= flags; return; }
    int stack[128]; int size = 1; stack[0] = 0;
    while(size > 0) {
        BvhNode node = bvhNodes[stack[--size]];
        vec3 center = (node.minimum.xyz + node.maximum.xyz) * 0.5 - origin;
        vec3 halfExtent = (node.maximum.xyz - node.minimum.xyz) * 0.5 + vec3(1e-5);
        bool inside = true;
        for(int p = 0; p < 5; ++p) {
            vec3 plane = meshPlane(p);
            vec3 world = major * plane.x + minor * plane.y + forward * plane.z;
            if(dot(world, center) + dot(abs(world), halfExtent) < -1e-5) { inside = false; break; }
        }
        if(!inside) continue;
        if(node.leftChild >= 0) {
            if(size + 2 > 128) { flags |= 128; break; }
            stack[size++] = node.leftChild; stack[size++] = node.rightChild;
            continue;
        }
        int start = -(node.leftChild + 1);
        for(int j = 0; j < node.rightChild; ++j) {
            uint face = leafTriangleIndices[start + j];
            vec3 a = meshLocal(surfacePositions[triangleIndices[face]].xyz - origin, major, minor, forward);
            vec3 b = meshLocal(surfacePositions[triangleIndices[face + 1u]].xyz - origin, major, minor, forward);
            vec3 c = meshLocal(surfacePositions[triangleIndices[face + 2u]].xyz - origin, major, minor, forward);
            vec3 localDirection;
            if(!meshTriangleDirection(a, b, c, localDirection)) continue;
            int hit = meshFirstHit(origin, normalize(major * localDirection.x + minor * localDirection.y + forward * localDirection.z));
            flags |= hit;
            if((flags & (32 | 128)) != 0) { meshSamplingData[record + 1] |= flags; return; }
        }
    }
    meshSamplingData[record + 1] |= flags;
}
)GLSL";
        return source;
    }
}
