#pragma once

namespace spraythickness::opengl
{
    // One packed SSBO: predecessor indices followed by eight ints per spray.
    // Integer fixed-point moments avoid requiring floating-point atomics.
    inline constexpr const char* kMeshSamplingShader = R"GLSL(
layout(std430, binding = 14) buffer MeshSamplingBuffer { int meshSamplingData[]; };
uniform int meshRecordOffset;
uniform int meshSprayBase;
uniform float meshMomentScale;
const float meshCoreSquared = 5.9914645471; // -2 * log(0.05)
const uint meshGroupSlots = 128u;
shared int meshGroupKeys[128];
shared int meshGroupRecords[128 * 7];

int meshRecord(int point) { return meshRecordOffset + 8 * (meshSprayBase + point); }

void meshBeginGroup()
{
    for(uint slot = gl_LocalInvocationIndex; slot < meshGroupSlots; slot += gl_WorkGroupSize.x) {
        meshGroupKeys[slot] = -1;
    }
    for(uint i = gl_LocalInvocationIndex; i < meshGroupSlots * 7u; i += gl_WorkGroupSize.x) {
        meshGroupRecords[i] = 0;
    }
    barrier();
}

void meshEndGroup()
{
    barrier();
    for(uint slot = gl_LocalInvocationIndex; slot < meshGroupSlots; slot += gl_WorkGroupSize.x) {
        int key = meshGroupKeys[slot];
        if(key < 0) continue;
        int record = meshRecordOffset + 8 * key;
        uint groupRecord = slot * 7u;
        atomicAdd(meshSamplingData[record], meshGroupRecords[groupRecord]);
        atomicOr(meshSamplingData[record + 1], meshGroupRecords[groupRecord + 1u]);
        for(uint field = 2u; field < 7u; ++field) {
            int value = meshGroupRecords[groupRecord + field];
            if(value != 0) atomicAdd(meshSamplingData[record + int(field)], value);
        }
    }
}

bool meshRepresentative(uint vertex, vec3 vertexToSpray)
{
    int previous = meshSamplingData[vertex];
    if(previous == -2) return false;
    // Shared STL positions can carry different face normals. Elect the first
    // eligible front-facing normal for this spray, not a fixed face normal.
    while(previous >= 0) {
        if(dot(surfaceData[uint(previous) * 2u + 1u].xyz, vertexToSpray) > 0.0) return false;
        previous = meshSamplingData[previous];
    }
    return true;
}

void meshAccumulate(int point, vec2 coordinate)
{
    int flags = dot(coordinate, coordinate) <= 1.0 ? 16 : 0;
    if(coordinate.x <= -0.5) flags |= 1;
    if(coordinate.x >=  0.5) flags |= 2;
    if(coordinate.y <= -0.5) flags |= 4;
    if(coordinate.y >=  0.5) flags |= 8;
    int x = int(round(coordinate.x * meshMomentScale));
    int y = int(round(coordinate.y * meshMomentScale));
    int xx = int(round(coordinate.x * coordinate.x * meshMomentScale));
    int xy = int(round(coordinate.x * coordinate.y * meshMomentScale));
    int yy = int(round(coordinate.y * coordinate.y * meshMomentScale));
    int key = meshSprayBase + point;
    uint slot = uint(key) % meshGroupSlots;
    int previous = atomicCompSwap(meshGroupKeys[slot], -1, key);
    if(previous == -1 || previous == key) {
        // Different spatial candidates may visit different sprays. No barrier
        // belongs in that divergent loop; slots keep their key until group end.
        uint record = slot * 7u;
        atomicAdd(meshGroupRecords[record], 1);
        atomicOr(meshGroupRecords[record + 1u], flags);
        atomicAdd(meshGroupRecords[record + 2u], x);
        atomicAdd(meshGroupRecords[record + 3u], y);
        atomicAdd(meshGroupRecords[record + 4u], xx);
        atomicAdd(meshGroupRecords[record + 5u], xy);
        atomicAdd(meshGroupRecords[record + 6u], yy);
        return;
    }
    // Hash collisions fall back to exact global accumulation, never drop points.
    int record = meshRecord(point);
    atomicAdd(meshSamplingData[record], 1);
    atomicOr(meshSamplingData[record + 1], flags);
    atomicAdd(meshSamplingData[record + 2], x);
    atomicAdd(meshSamplingData[record + 3], y);
    atomicAdd(meshSamplingData[record + 4], xx);
    atomicAdd(meshSamplingData[record + 5], xy);
    atomicAdd(meshSamplingData[record + 6], yy);
}
)GLSL";
}
