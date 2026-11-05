#include "DynamicSurfaceGpu.h"
#include "OpenGLComputeProgram.h"
#include <glad/glad.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <map>
#include <numeric>
#include <stdexcept>
#include <tuple>

namespace spraythickness::opengl
{
    namespace
    {
        static_assert(sizeof(GpuBvhNode) == 48, "GPU BVH node layout must match std430.");
        struct Buffer
        {
            GLuint id = 0;
            Buffer() { glGenBuffers(1, &id); }
            ~Buffer() { glDeleteBuffers(1, &id); }
            Buffer(const Buffer&) = delete;
            Buffer& operator=(const Buffer&) = delete;
            void allocate(const void* data, std::size_t bytes) const
            {
                glBindBuffer(GL_SHADER_STORAGE_BUFFER, id);
                glBufferData(GL_SHADER_STORAGE_BUFFER, static_cast<GLsizeiptr>(bytes), data, GL_DYNAMIC_COPY);
            }
            void bind(unsigned int slot) const { glBindBufferBase(GL_SHADER_STORAGE_BUFFER, slot, id); }
        };

        void copy(GLuint from, GLuint to, std::size_t bytes)
        {
            glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);
            glBindBuffer(GL_COPY_READ_BUFFER, from);
            glBindBuffer(GL_COPY_WRITE_BUFFER, to);
            glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 0, static_cast<GLsizeiptr>(bytes));
        }

        template<class T> void read(GLuint id, std::vector<T>& values)
        {
            glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, id);
            glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
                static_cast<GLsizeiptr>(values.size() * sizeof(T)), values.data());
        }

        struct Topology
        {
            std::vector<std::array<std::uint32_t, 4>> vertices;
            std::vector<std::uint32_t> faces;
        };

        Topology topology(const sprayworkpiece::WorkpieceModel& surface)
        {
            Topology result;
            const auto count = surface.samples.size();
            result.vertices.resize(count);
            // Weld only identical positions; a tolerance weld can close a real gap.
            std::map<std::tuple<double, double, double>, std::uint32_t> representatives;
            std::vector<std::vector<std::uint32_t>> adjacency(count);
            for(std::size_t i = 0; i < count; ++i) {
                const auto& p = surface.samples[i].position;
                if(!p.allFinite() || !surface.samples[i].valid)
                    throw std::runtime_error("Dynamic surface requires finite, valid mesh vertices.");
                auto inserted = representatives.emplace(std::make_tuple(p.x(), p.y(), p.z()), static_cast<std::uint32_t>(i));
                result.vertices[i][0] = inserted.first->second;
            }
            for(std::size_t f = 0; f < surface.triangleIndices.size(); f += 3) {
                for(int c = 0; c < 3; ++c) {
                    auto v = surface.triangleIndices[f + c];
                    if(v >= count) throw std::runtime_error("Invalid dynamic surface triangle index.");
                    adjacency[result.vertices[v][0]].push_back(static_cast<std::uint32_t>(f));
                }
            }
            for(std::size_t i = 0; i < count; ++i) {
                if(result.vertices[i][0] != i) continue;
                if(adjacency[i].empty()) throw std::runtime_error("Dynamic surface requires a triangle mesh, not isolated points.");
                result.vertices[i][1] = static_cast<std::uint32_t>(result.faces.size());
                result.faces.insert(result.faces.end(), adjacency[i].begin(), adjacency[i].end());
                result.vertices[i][2] = static_cast<std::uint32_t>(result.faces.size());
            }
            for(std::size_t i = 0; i < count; ++i) result.vertices[i] = result.vertices[result.vertices[i][0]];
            return result;
        }

        const char* shader = R"GLSL(#version 430
layout(local_size_x=256) in;
struct Node { vec4 lo; vec4 hi; int left; int right; int pad[2]; };
layout(std430,binding=0) buffer Surface { vec4 surface[]; };
layout(std430,binding=1) readonly buffer Thickness { float thickness[]; };
layout(std430,binding=3) readonly buffer Indices { uint indices[]; };
layout(std430,binding=4) buffer Bvh { Node nodes[]; };
layout(std430,binding=5) readonly buffer Order { uint faceOrder[]; };
layout(std430,binding=6) readonly buffer Faces { uint faces[]; };
layout(std430,binding=7) buffer Normals { vec4 normals[]; };
layout(std430,binding=8) buffer Status { uint flags; uint growthBits; uint bendBits; };
layout(std430,binding=9) readonly buffer Topology { uvec4 topology[]; };
layout(std430,binding=10) buffer Positions { vec4 positions[]; };
layout(std430,binding=11) readonly buffer PreviousThickness { float previous[]; };
layout(std430,binding=12) readonly buffer Levels { uint levelNodes[]; };
uniform int phase, count, levelOffset;
uniform float maximumGrowth, minimumNormalCosine;

bool boxHit(vec3 lo, vec3 hi, vec3 a, vec3 b) {
    return all(lessThanEqual(min(a,b),hi)) && all(greaterThanEqual(max(a,b),lo));
}
bool segmentTriangle(vec3 p,vec3 q,vec3 a,vec3 b,vec3 c) {
    vec3 d=q-p, e=b-a, f=c-a, h=cross(d,f);
    float det=dot(e,h);
    if(abs(det)<1e-12) return false;
    vec3 s=p-a; float u=dot(s,h)/det;
    vec3 r=cross(s,e); float v=dot(d,r)/det, t=dot(f,r)/det;
    return u>=-1e-6 && v>=-1e-6 && u+v<=1.000001 && t>=-1e-6 && t<=1.000001;
}
bool shared(uint f,uint g) {
    for(int a=0;a<3;++a) for(int b=0;b<3;++b)
        if(topology[indices[f+a]].x==topology[indices[g+b]].x) return true;
    return false;
}
// SAT on the shared plane also detects coplanar non-neighbour overlap.
bool trianglesHit(vec3 a,vec3 b,vec3 c,vec3 d,vec3 e,vec3 f) {
    vec3 n=cross(b-a,c-a); float nn=length(n);
    if(nn>1e-12 && max(max(abs(dot(n,d-a)),abs(dot(n,e-a))),abs(dot(n,f-a)))<1e-6*nn) {
        vec3 edges[6]=vec3[6](b-a,c-b,a-c,e-d,f-e,d-f);
        for(int k=0;k<6;++k) {
            vec3 axis=cross(n,edges[k]);
            float l=min(min(dot(axis,a),dot(axis,b)),dot(axis,c));
            float h=max(max(dot(axis,a),dot(axis,b)),dot(axis,c));
            float l2=min(min(dot(axis,d),dot(axis,e)),dot(axis,f));
            float h2=max(max(dot(axis,d),dot(axis,e)),dot(axis,f));
            if(h<l2 || h2<l) return false;
        }
        return true;
    }
    return segmentTriangle(a,b,d,e,f)||segmentTriangle(b,c,d,e,f)||segmentTriangle(c,a,d,e,f)
        ||segmentTriangle(d,e,a,b,c)||segmentTriangle(e,f,a,b,c)||segmentTriangle(f,d,a,b,c);
}
void main() {
    uint v=gl_GlobalInvocationID.x; if(v>=uint(count)) return;
    if(phase==0) {
        uint r=topology[v].x;
        float delta=thickness[r]-previous[r];
        if(isnan(delta)||isinf(delta)||delta< -1e-6) atomicOr(flags,4u);
        if(delta>maximumGrowth) atomicOr(flags,1u);
        if(!isnan(delta) && !isinf(delta)) atomicMax(growthBits,floatBitsToUint(max(delta,0.0)));
        positions[v]=vec4(surface[2*r].xyz+max(delta,0.0)*surface[2*r+1].xyz,1);
    } else if(phase==1) {
        vec3 sum=vec3(0); uvec4 t=topology[v];
        for(uint i=t.y;i<t.z;++i) {
            uint f=faces[i], a=indices[f], b=indices[f+1], c=indices[f+2];
            vec3 old=cross(surface[2*b].xyz-surface[2*a].xyz,surface[2*c].xyz-surface[2*a].xyz);
            vec3 next=cross(positions[b].xyz-positions[a].xyz,positions[c].xyz-positions[a].xyz);
            if(dot(old,next)<=0.0) atomicOr(flags,4u);
            sum+=next;
        }
        if(length(sum)<1e-12) { atomicOr(flags,4u); normals[v]=surface[2*v+1]; }
        else {
            normals[v]=vec4(normalize(sum),0);
            float cosine=clamp(dot(normals[v].xyz,surface[2*v+1].xyz),-1.0,1.0);
            atomicMax(bendBits,floatBitsToUint(1.0-cosine));
            if(cosine<minimumNormalCosine) atomicOr(flags,2u);
        }
    } else if(phase==2) {
        uint i=levelNodes[uint(levelOffset)+v]; Node n=nodes[i];
        vec3 lo=vec3(3.402823e38), hi=-lo;
        if(n.left<0) {
            for(int k=0;k<n.right;++k) {
                uint f=faceOrder[-n.left-1+k];
                for(int c=0;c<3;++c) { vec3 p=positions[indices[f+c]].xyz; lo=min(lo,p); hi=max(hi,p); }
            }
        } else { lo=min(nodes[n.left].lo.xyz,nodes[n.right].lo.xyz); hi=max(nodes[n.left].hi.xyz,nodes[n.right].hi.xyz); }
        nodes[i].lo=vec4(lo,0); nodes[i].hi=vec4(hi,0);
    } else if(phase==3) {
        uint f=v*3; vec3 a=positions[indices[f]].xyz,b=positions[indices[f+1]].xyz,c=positions[indices[f+2]].xyz;
        vec3 lo=min(min(a,b),c),hi=max(max(a,b),c);
        int stack[64]; int size=1; stack[0]=0;
        while(size>0) {
            Node n=nodes[stack[--size]];
            if(any(lessThan(hi,n.lo.xyz))||any(greaterThan(lo,n.hi.xyz))) continue;
            if(n.left>=0) {
                if(size>61) { atomicOr(flags,4u); return; }
                stack[size++]=n.left; stack[size++]=n.right;
            } else for(int k=0;k<n.right;++k) {
                uint g=faceOrder[-n.left-1+k]; if(g<=f||shared(f,g)) continue;
                if(trianglesHit(a,b,c,positions[indices[g]].xyz,positions[indices[g+1]].xyz,positions[indices[g+2]].xyz)) {
                    atomicOr(flags,8u); return;
                }
            }
        }
    } else if(phase==4) {
        surface[2*v]=positions[v]; surface[2*v+1]=normals[v];
    } else if(phase==5) {
        positions[v]=surface[2*v];
    } else if(phase==6) {
        // Check swept vertex segments against the old surface before refitting.
        vec3 a=surface[2*v].xyz,b=positions[v].xyz;
        if(length(b-a)<1e-8) return;
        int stack[64]; int size=1; stack[0]=0;
        while(size>0) {
            Node n=nodes[stack[--size]];
            if(!boxHit(n.lo.xyz,n.hi.xyz,a,b)) continue;
            if(n.left>=0) {
                if(size>61) { atomicOr(flags,4u); return; }
                stack[size++]=n.left; stack[size++]=n.right;
            } else for(int k=0;k<n.right;++k) {
                uint f=faceOrder[-n.left-1+k];
                if(topology[indices[f]].x==topology[v].x||topology[indices[f+1]].x==topology[v].x||topology[indices[f+2]].x==topology[v].x) continue;
                if(segmentTriangle(a,b,surface[2*indices[f]].xyz,surface[2*indices[f+1]].xyz,surface[2*indices[f+2]].xyz)) atomicOr(flags,8u);
            }
        }
    }
}
)GLSL";
    }

    void DynamicSurfaceGpu::prepareSurface(sprayworkpiece::WorkpieceModel& surface)
    {
        if(surface.triangleIndices.empty() || surface.triangleIndices.size() % 3)
            throw std::runtime_error("Dynamic surface requires indexed triangles.");
        const auto topo = topology(surface);
        const auto faceCount = surface.triangleIndices.size() / 3;
        struct EdgeFace { std::size_t face; int direction; };
        std::map<std::pair<std::uint32_t,std::uint32_t>,std::vector<EdgeFace>> edges;
        for(std::size_t f=0; f<faceCount; ++f) for(int c=0;c<3;++c) {
            auto a=topo.vertices[surface.triangleIndices[3*f+c]][0];
            auto b=topo.vertices[surface.triangleIndices[3*f+(c+1)%3]][0];
            if(a==b) throw std::runtime_error("Dynamic surface contains a degenerate triangle.");
            auto& list=edges[{std::min(a,b),std::max(a,b)}];
            list.push_back({f,a<b?1:-1});
            if(list.size()>2) throw std::runtime_error("Dynamic surface requires a manifold mesh; an edge has more than two faces.");
        }
        std::vector<std::vector<std::pair<std::size_t,int>>> neighbours(faceCount);
        std::vector<bool> boundary(faceCount,false);
        for(const auto& entry:edges) {
            const auto& list=entry.second;
            if(list.size()==1) boundary[list[0].face]=true;
            else {
                int sign=-list[0].direction*list[1].direction;
                neighbours[list[0].face].push_back({list[1].face,sign});
                neighbours[list[1].face].push_back({list[0].face,sign});
            }
        }
        std::vector<int> orientation(faceCount,0);
        for(std::size_t seed=0;seed<faceCount;++seed) {
            if(orientation[seed]) continue;
            std::vector<std::size_t> component{seed}; orientation[seed]=1;
            bool closed=true; double volume=0, agreement=0;
            const auto origin=surface.samples[surface.triangleIndices[3*seed]].position;
            for(std::size_t i=0;i<component.size();++i) {
                auto f=component[i]; closed=closed&&!boundary[f];
                for(const auto& edge:neighbours[f]) {
                    int sign=orientation[f]*edge.second;
                    if(!orientation[edge.first]) { orientation[edge.first]=sign; component.push_back(edge.first); }
                    else if(orientation[edge.first]!=sign) throw std::runtime_error("Dynamic surface has inconsistent/non-orientable topology.");
                }
                const auto& a=surface.samples[surface.triangleIndices[3*f]];
                const auto& b=surface.samples[surface.triangleIndices[3*f+1]];
                const auto& c=surface.samples[surface.triangleIndices[3*f+2]];
                Eigen::Vector3d n=(b.position-a.position).cross(c.position-a.position)*orientation[f];
                if(n.squaredNorm()<1e-30) throw std::runtime_error("Dynamic surface contains a zero-area triangle.");
                volume+=(a.position-origin).dot(n);
                agreement+=n.dot(a.normal+b.normal+c.normal);
            }
            // Respect imported surface orientation, including inner cavity shells.
            // Signed volume is a fallback when supplied normals carry no orientation.
            int flip=(std::isfinite(agreement) && std::abs(agreement)>1e-18 ? agreement : (closed?volume:1))<0?-1:1;
            for(auto f:component) if(orientation[f]*flip<0) std::swap(surface.triangleIndices[3*f+1],surface.triangleIndices[3*f+2]);
        }
        std::vector<Eigen::Vector3d> normals(surface.samples.size(),Eigen::Vector3d::Zero());
        for(std::size_t f=0;f<surface.triangleIndices.size();f+=3) {
            auto a=surface.triangleIndices[f],b=surface.triangleIndices[f+1],c=surface.triangleIndices[f+2];
            Eigen::Vector3d n=(surface.samples[b].position-surface.samples[a].position).cross(surface.samples[c].position-surface.samples[a].position);
            for(auto v:{a,b,c}) normals[topo.vertices[v][0]]+=n;
        }
        for(std::size_t v=0;v<surface.samples.size();++v) {
            const auto& n=normals[topo.vertices[v][0]];
            if(n.norm()<1e-15) throw std::runtime_error("Dynamic surface cannot determine an outward vertex normal.");
            surface.samples[v].normal=n.normalized();
        }
    }

    struct DynamicSurfaceGpu::Impl
    {
        std::shared_ptr<const sprayworkpiece::WorkpieceModel> substrate;
        DynamicSurfaceOptions options;
        std::array<GLuint,11> buffers;
        std::array<Buffer,4> saved;
        Buffer adjacency, topologyBuffer, normals, status, levels;
        std::vector<std::pair<int,int>> levelRanges;
        OpenGLComputeProgram program{shader,"Dynamic surface evolution"};
        std::size_t accepted=0,rejected=0;
        double nextScale=1.0;
        double initialBvhCost=0.0;
        std::size_t bvhNodeCount=0;
        static double bvhCost(const std::vector<GpuBvhNode>& nodes) {
            auto area=[](const GpuBvhNode& n) {
                double x=std::max(0.0,static_cast<double>(n.maximum[0]-n.minimum[0]));
                double y=std::max(0.0,static_cast<double>(n.maximum[1]-n.minimum[1]));
                double z=std::max(0.0,static_cast<double>(n.maximum[2]-n.minimum[2]));
                return x*y+x*z+y*z;
            };
            double total=0;for(const auto& n:nodes)total+=area(n);
            return total/std::max(area(nodes.front()),1e-20);
        }
        void setLevels(const ThicknessBvh& bvh) {
            levelRanges.clear();
            std::vector<std::uint32_t> nodes{0}; std::size_t first=0;
            while(first<nodes.size()) {
                auto end=nodes.size();levelRanges.push_back({static_cast<int>(first),static_cast<int>(end-first)});
                for(auto i=first;i<end;++i) {
                    const auto& node=bvh.nodes.at(nodes[i]);
                    if(node.triangleCount==0){nodes.push_back(node.leftChild);nodes.push_back(node.rightChild);}
                }
                first=end;
            }
            levels.allocate(nodes.data(),nodes.size()*sizeof(std::uint32_t));
            bvhNodeCount=bvh.nodes.size();initialBvhCost=bvhCost(bvh.nodes);
        }
        void bind() {
            for(unsigned int i:{0u,1u,3u,4u,5u,10u}) glBindBufferBase(GL_SHADER_STORAGE_BUFFER,i,buffers[i]);
            adjacency.bind(6); normals.bind(7); status.bind(8); topologyBuffer.bind(9); saved[0].bind(11); levels.bind(12);
            program.use();
        }
        void dispatch(int phase,std::size_t count) {
            program.setInt("phase",phase); program.setInt("count",static_cast<int>(count));
            glDispatchCompute(static_cast<GLuint>((count+255)/256),1,1);
            glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
        }
        void refit() {
            for(auto it=levelRanges.rbegin();it!=levelRanges.rend();++it) {
                program.setInt("levelOffset",it->first); dispatch(2,it->second);
            }
        }
        unsigned int flags() {
            std::vector<unsigned int> value(3); read(status.id,value);
            float growth=0,bend=0;
            std::memcpy(&growth,&value[1],sizeof(float)); std::memcpy(&bend,&value[2],sizeof(float));
            const double angle=std::acos(std::clamp(1.0-static_cast<double>(bend),-1.0,1.0))*180/3.141592653589793;
            const double ratio=std::max(growth/(options.maximumGrowthMeters*1000),angle/options.maximumNormalChangeDegrees);
            nextScale=(value[0]&12u)?0.5:std::clamp(0.9/std::max(ratio,1e-6),0.1,1.5);
            if(value[0])nextScale=std::min(nextScale,0.5);
            return value[0];
        }
        void resetStatus() {
            const unsigned int zero[3]{}; glBindBuffer(GL_SHADER_STORAGE_BUFFER,status.id);
            glBufferSubData(GL_SHADER_STORAGE_BUFFER,0,sizeof(zero),zero);
        }
    };

    DynamicSurfaceGpu::DynamicSurfaceGpu(const sprayworkpiece::WorkpieceModel& surface,
        const DynamicSurfaceOptions& options,const ThicknessBvh& bvh,const std::array<unsigned int,11>& buffers)
        : m_impl(std::make_unique<Impl>())
    {
        auto& s=*m_impl; s.substrate=std::make_shared<sprayworkpiece::WorkpieceModel>(surface); s.options=options; s.buffers=buffers;
        const auto topo=topology(surface); const auto n=surface.samples.size();
        s.adjacency.allocate(topo.faces.data(),topo.faces.size()*sizeof(std::uint32_t));
        s.topologyBuffer.allocate(topo.vertices.data(),topo.vertices.size()*sizeof(topo.vertices[0]));
        s.normals.allocate(nullptr,n*4*sizeof(float)); s.status.allocate(nullptr,3*sizeof(unsigned int));
        for(auto& b:s.saved) b.allocate(nullptr,n*sizeof(float));
        s.setLevels(bvh);
        s.bind(); s.resetStatus(); s.dispatch(3,surface.triangleIndices.size()/3);
        if(s.flags()) throw std::runtime_error("Dynamic surface input contains intersecting triangles or an unsupported BVH depth.");
        bindDepositionBuffers();
    }
    DynamicSurfaceGpu::~DynamicSurfaceGpu()=default;
    double DynamicSurfaceGpu::nextIntervalScale() const { return m_impl->nextScale; }
    void DynamicSurfaceGpu::rebuildIfNeeded(const std::function<void(const ThicknessBvh&)>& upload) {
        auto& s=*m_impl;
        if(s.accepted==0 || s.accepted%128!=0) return;
        // The legacy GPU node has the same 48-byte bounds prefix/stride.
        // Only bounds are inspected here, never its differently encoded children.
        std::vector<GpuBvhNode> bounds(s.bvhNodeCount);read(s.buffers[4],bounds);
        if(Impl::bvhCost(bounds)<=2*s.initialBvhCost)return;
        auto current=snapshot(0,0);
        auto workpiece=*s.substrate;
        for(std::size_t i=0;i<workpiece.samples.size();++i) {
            workpiece.samples[i].position=current->positions[i];
            workpiece.samples[i].normal=current->normals[i];
        }
        auto rebuilt=ThicknessBvhBuilder::build(workpiece);
        upload(rebuilt);s.setLevels(rebuilt);bindDepositionBuffers();
    }
    void DynamicSurfaceGpu::bindDepositionBuffers() const {
        for(unsigned int i=0;i<m_impl->buffers.size();++i) glBindBufferBase(GL_SHADER_STORAGE_BUFFER,i,m_impl->buffers[i]);
    }
    void DynamicSurfaceGpu::checkpoint() {
        auto& s=*m_impl; const std::array<unsigned int,4> slots{1,6,7,8};
        for(std::size_t i=0;i<slots.size();++i) copy(s.buffers[slots[i]],s.saved[i].id,s.substrate->samples.size()*sizeof(float));
    }
    bool DynamicSurfaceGpu::tryAdvance() {
        auto& s=*m_impl; auto n=s.substrate->samples.size();
        s.bind(); s.resetStatus();
        s.program.setFloat("maximumGrowth",static_cast<float>(s.options.maximumGrowthMeters*1000));
        s.program.setFloat("minimumNormalCosine",static_cast<float>(std::cos(s.options.maximumNormalChangeDegrees*3.141592653589793/180)));
        s.dispatch(0,n); s.dispatch(1,n); s.dispatch(6,n);
        if(s.flags()) { ++s.rejected; rollback(); return false; }
        s.refit(); s.dispatch(3,s.substrate->triangleIndices.size()/3);
        if(s.flags()) { ++s.rejected; rollback(); return false; }
        s.dispatch(4,n); ++s.accepted; bindDepositionBuffers(); return true;
    }
    void DynamicSurfaceGpu::rollback() {
        auto& s=*m_impl; const std::array<unsigned int,4> slots{1,6,7,8};
        for(std::size_t i=0;i<slots.size();++i) copy(s.saved[i].id,s.buffers[slots[i]],s.substrate->samples.size()*sizeof(float));
        s.bind(); s.dispatch(5,s.substrate->samples.size()); s.refit(); bindDepositionBuffers();
    }
    std::shared_ptr<DynamicSurfaceSnapshot> DynamicSurfaceGpu::snapshot(double time,double elapsed,bool history) const {
        const auto& s=*m_impl; auto result=std::make_shared<DynamicSurfaceSnapshot>();
        result->substrate=s.substrate; result->options=s.options; result->simulationSeconds=time; result->elapsedSeconds=elapsed;
        result->acceptedSteps=s.accepted; result->rejectedSteps=s.rejected;
        const auto n=s.substrate->samples.size();
        std::vector<std::array<float,4>> data(2*n); std::vector<float> thickness(n);
        read(s.buffers[0],data); read(s.buffers[1],thickness);
        result->positions.resize(n); result->normals.resize(n); result->thicknessMeters.resize(n);
        for(std::size_t i=0;i<n;++i) {
            result->positions[i]=Eigen::Vector3d(data[2*i][0],data[2*i][1],data[2*i][2])*0.001;
            result->normals[i]=Eigen::Vector3d(data[2*i+1][0],data[2*i+1][1],data[2*i+1][2]);
            result->thicknessMeters[i]=thickness[i]*0.001;
        }
        if(history) {
            result->historySeconds.resize(n); result->lastUpdateSeconds.resize(n); result->historyFactors.resize(n);
            read(s.buffers[6],result->historySeconds); read(s.buffers[7],result->lastUpdateSeconds); read(s.buffers[8],result->historyFactors);
        }
        return result;
    }
}
