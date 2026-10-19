#include "GpuMeshSampling.h"
#include "MeshSamplingProbeShader.h"
#include "OpenGLComputeProgram.h"

#include <SprayThicknessPredictionOpenGL/MeshSamplingEvaluator.h>
#include <glad/glad.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace spraythickness::opengl
{
    namespace
    {
        using Clock = std::chrono::steady_clock;
        constexpr std::size_t kRecordInts = 8;

        double elapsed(Clock::time_point start)
        {
            return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        }

        void checkCancelled(const ThicknessPredictionExecution& execution)
        {
            if(execution.cancelRequested && execution.cancelRequested->load()) {
                throw std::runtime_error("Mesh sampling assessment cancelled.");
            }
        }

        struct PositionKey
        {
            std::array<std::int64_t, 3> coordinates;
            bool operator==(const PositionKey& other) const { return coordinates == other.coordinates; }
        };

        struct PositionHash
        {
            std::size_t operator()(const PositionKey& key) const
            {
                std::size_t hash = 0;
                for(auto value : key.coordinates) {
                    hash ^= std::hash<std::int64_t>{}(value) + 0x9e3779b9U + (hash << 6) + (hash >> 2);
                }
                return hash;
            }
        };

        PositionKey positionKey(const Eigen::Vector3d& position)
        {
            return { { std::llround(position.x() * 1e9), std::llround(position.y() * 1e9),
                std::llround(position.z() * 1e9) } };
        }

        void hashValue(std::uint64_t& hash, std::uint64_t value)
        {
            hash ^= value;
            hash *= 1099511628211ull;
        }
    }

    GpuMeshSampling::GpuMeshSampling() = default;

    GpuMeshSampling::~GpuMeshSampling()
    {
        if(m_buffer != 0) glDeleteBuffers(1, &m_buffer);
    }

    void GpuMeshSampling::disable(const std::string& reason)
    {
        m_active = false;
        m_metadata.applicable = false;
        m_metadata.unavailableReason = reason;
    }

    bool GpuMeshSampling::prepare(const ThicknessPredictionTask& task, std::size_t sprayCount,
        const ThicknessPredictionExecution& execution)
    {
        m_active = false;
        m_metadata = {};
        if(!task.options.evaluateMeshSampling) return false;
        m_metadata.gpuIntegrated = true;
        const auto start = Clock::now();
        try {
            checkCancelled(execution);
            if(task.options.periodicLocal.enabled || task.options.axisymmetricProfile.enabled) {
                throw std::runtime_error("Reduced prediction modes do not use the full surface vertex set.");
            }
            if(task.workpiece.triangleIndices.empty()) {
                throw std::runtime_error("Triangle topology is required to distinguish a missed footprint from coarse sampling.");
            }
            const auto& deposition = task.options.deposition;
            const double c = std::cos(deposition.rotationRadians), s = std::sin(deposition.rotationRadians);
            const double radiusSquared = -2.0 * std::log(m_metadata.corePatternRatio);
            const double phiExtent = std::sqrt(radiusSquared * (std::pow(c * deposition.sigmaPhiRadians, 2)
                + std::pow(s * deposition.sigmaPsiRadians, 2)));
            const double psiExtent = std::sqrt(radiusSquared * (std::pow(s * deposition.sigmaPhiRadians, 2)
                + std::pow(c * deposition.sigmaPsiRadians, 2)));
            if(!(deposition.sigmaPhiRadians >= 1e-6 && deposition.sigmaPsiRadians >= 1e-6
                && std::abs(deposition.phiOffsetRadians) + phiExtent < 1.55
                && std::abs(deposition.psiOffsetRadians) + psiExtent < 1.55)) {
                throw std::runtime_error("Gaussian angular parameters cannot define a forward core footprint.");
            }
            GLint bindings = 0, blocks = 0, blockSize = 0;
            glGetIntegerv(GL_MAX_SHADER_STORAGE_BUFFER_BINDINGS, &bindings);
            glGetIntegerv(GL_MAX_COMPUTE_SHADER_STORAGE_BLOCKS, &blocks);
            glGetIntegerv(GL_MAX_SHADER_STORAGE_BLOCK_SIZE, &blockSize);
            const std::size_t vertexCount = task.workpiece.samples.size();
            const std::size_t ints = vertexCount + kRecordInts * sprayCount;
            if(bindings < 15 || blocks < (task.options.spatialFiltering.enabled ? 15 : 12)
                || ints > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max())
                || ints * sizeof(std::int32_t) > static_cast<std::size_t>(blockSize)
                || sprayCount * 80 > static_cast<std::size_t>(blockSize)) {
                throw std::runtime_error("GPU mesh assessment exceeds the device's SSBO limits; CPU geometry scanning is disabled.");
            }
            std::vector<std::uint8_t> selected(vertexCount,
                task.options.spatialFiltering.predictionVertexIndices.empty() ? 1U : 0U);
            for(auto index : task.options.spatialFiltering.predictionVertexIndices) {
                if(index >= vertexCount) throw std::runtime_error("Invalid assessment prediction vertex index.");
                selected[index] = 1U;
            }
            std::vector<std::uint8_t> referenced(vertexCount, 0U);
            for(auto index : task.workpiece.triangleIndices) {
                if(index < vertexCount) referenced[index] = 1U;
            }
            // A single linear fingerprint scan replaces rebuilding the hash table
            // on every trajectory change. Normals are read live on the GPU.
            std::uint64_t key = 14695981039346656037ull;
            hashValue(key, vertexCount);
            for(std::size_t i = 0; i < vertexCount; ++i) {
                if((i & 4095U) == 0) checkCancelled(execution);
                const auto& sample = task.workpiece.samples[i];
                const bool eligible = selected[i] && referenced[i] && sample.valid && sample.position.allFinite()
                    && sample.normal.allFinite() && sample.normal.squaredNorm() > 0.0;
                selected[i] = eligible ? 1U : 0U;
                hashValue(key, selected[i]);
                if(eligible) {
                    for(auto value : positionKey(sample.position).coordinates) hashValue(key, static_cast<std::uint64_t>(value));
                }
            }
            m_metadata.vertexMappingCacheHit = m_mappingValid && key == m_mappingKey;
            if(!m_metadata.vertexMappingCacheHit) {
                std::unordered_map<PositionKey, std::int32_t, PositionHash> last;
                last.reserve(vertexCount);
                m_predecessors.assign(vertexCount, -2);
                for(std::size_t i = 0; i < vertexCount; ++i) {
                    if((i & 4095U) == 0) checkCancelled(execution);
                    if(!selected[i]) continue;
                    auto inserted = last.emplace(positionKey(task.workpiece.samples[i].position), -1);
                    m_predecessors[i] = inserted.first->second;
                    inserted.first->second = static_cast<std::int32_t>(i);
                }
                m_mappingKey = key;
                m_mappingValid = true;
            }
            // Bound each signed sum by 1.9e9, including rounding. Counts remain exact.
            m_momentScale = static_cast<float>(std::max(0.25, std::min(1e6,
                std::floor(1.9e9 / (std::max<std::size_t>(vertexCount, 1) * 7.0)))));
            if(!m_probeProgram) {
                const auto source = makeMeshSamplingProbeShader();
                m_probeProgram = std::make_unique<OpenGLComputeProgram>(source.c_str(), "Mesh sampling zero-count probe");
            }
            if(m_buffer == 0) glGenBuffers(1, &m_buffer);
            glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);
            glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_buffer);
            const std::size_t bytes = ints * sizeof(std::int32_t);
            const bool reallocate = m_bufferBytes != bytes;
            if(reallocate) glBufferData(GL_SHADER_STORAGE_BUFFER, static_cast<GLsizeiptr>(bytes), nullptr, GL_DYNAMIC_COPY);
            if(reallocate || !m_uploadedMappingValid || m_uploadedMappingKey != key) {
                glBufferSubData(GL_SHADER_STORAGE_BUFFER, 0,
                    static_cast<GLsizeiptr>(vertexCount * sizeof(std::int32_t)), m_predecessors.data());
            }
            const GLint zero = 0;
            glClearBufferSubData(GL_SHADER_STORAGE_BUFFER, GL_R32I,
                static_cast<GLintptr>(vertexCount * sizeof(std::int32_t)),
                static_cast<GLsizeiptr>(kRecordInts * sprayCount * sizeof(std::int32_t)), GL_RED_INTEGER, GL_INT, &zero);
            glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 14, m_buffer);
            if(glGetError() != GL_NO_ERROR) throw std::runtime_error("GPU mesh assessment buffer preparation failed.");
            m_bufferBytes = bytes;
            m_uploadedMappingKey = key;
            m_uploadedMappingValid = true;
            m_active = true;
        } catch(const std::exception& exception) {
            if(execution.cancelRequested && execution.cancelRequested->load()) throw;
            disable(exception.what());
            m_uploadedMappingValid = false;
            // Optional diagnostics must not poison the prediction's error state.
            while(glGetError() != GL_NO_ERROR) {}
        }
        m_metadata.preparationMilliseconds = elapsed(start);
        return m_active;
    }

    void GpuMeshSampling::configure(const OpenGLComputeProgram& program, std::size_t sprayBase) const
    {
        if(!m_active) return;
        glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 14, m_buffer);
        program.setInt("meshRecordOffset", static_cast<int>(m_predecessors.size()));
        program.setInt("meshSprayBase", static_cast<int>(sprayBase));
        program.setFloat("meshMomentScale", m_momentScale);
    }

    std::shared_ptr<const MeshSamplingAssessment> GpuMeshSampling::finish(
        const ThicknessPredictionTask& task, const std::vector<PeriodicSpraySample>& sprays,
        const ThicknessPredictionExecution& execution)
    {
        if(!task.options.evaluateMeshSampling) return {};
        if(!m_active) return std::make_shared<const MeshSamplingAssessment>(m_metadata);
        try {
            std::vector<std::array<std::int32_t, kRecordInts>> records(sprays.size());
            static_assert(sizeof(records.front()) == kRecordInts * sizeof(std::int32_t), "Packed GPU assessment record");
            const auto readRecords = [&] {
                checkCancelled(execution);
                const auto readStart = Clock::now();
                glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);
                glBindBuffer(GL_SHADER_STORAGE_BUFFER, m_buffer);
                glGetBufferSubData(GL_SHADER_STORAGE_BUFFER,
                    static_cast<GLintptr>(m_predecessors.size() * sizeof(std::int32_t)),
                    static_cast<GLsizeiptr>(records.size() * sizeof(records.front())), records.data());
                if(glGetError() != GL_NO_ERROR) throw std::runtime_error("GPU mesh assessment dispatch/readback failed.");
                m_metadata.readbackMilliseconds += elapsed(readStart);
            };
            readRecords();
            const bool needsProbe = std::any_of(records.begin(), records.end(), [](const auto& r) { return r[0] == 0; });
            if(needsProbe) {
                const auto probeStart = Clock::now();
                auto& program = *m_probeProgram;
                program.use();
                configure(program);
                const auto& p = task.options.deposition;
                const double c = std::cos(p.rotationRadians), s = std::sin(p.rotationRadians);
                const double radiusSquared = -2.0 * std::log(m_metadata.corePatternRatio);
                const double phiExtent = std::sqrt(radiusSquared * (std::pow(c * p.sigmaPhiRadians, 2) + std::pow(s * p.sigmaPsiRadians, 2)));
                const double psiExtent = std::sqrt(radiusSquared * (std::pow(s * p.sigmaPhiRadians, 2) + std::pow(c * p.sigmaPsiRadians, 2)));
                program.setFloat("phiOffset", static_cast<float>(p.phiOffsetRadians));
                program.setFloat("psiOffset", static_cast<float>(p.psiOffsetRadians));
                program.setFloat("patternCosine", static_cast<float>(c));
                program.setFloat("patternSine", static_cast<float>(s));
                program.setFloat("inverseSigmaPhi", static_cast<float>(1.0 / p.sigmaPhiRadians));
                program.setFloat("inverseSigmaPsi", static_cast<float>(1.0 / p.sigmaPsiRadians));
                glUniform4f(program.uniformLocation("meshSlopeBounds"),
                    static_cast<float>(std::tan(p.phiOffsetRadians - phiExtent)),
                    static_cast<float>(std::tan(p.phiOffsetRadians + phiExtent)),
                    static_cast<float>(std::tan(p.psiOffsetRadians - psiExtent)),
                    static_cast<float>(std::tan(p.psiOffsetRadians + psiExtent)));
                // Inspect compact counters, never CPU mesh geometry. Do not submit
                // or wait on batches whose footprints already contain vertices.
                for(std::size_t begin = 0; begin < records.size(); begin += 64) {
                    checkCancelled(execution);
                    const std::size_t end = std::min(begin + 64, records.size());
                    if(std::none_of(records.begin() + begin, records.begin() + end,
                        [](const auto& r) { return r[0] == 0; })) continue;
                    program.setInt("meshProbeBegin", static_cast<int>(begin));
                    program.setInt("meshProbeCount", static_cast<int>(end - begin));
                    glDispatchCompute(1, 1, 1);
                    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
                    glFinish();
                    if(execution.progress) execution.progress(0.96 + 0.025 * end / sprays.size(),
                        "GPU checking zero-vertex footprints...");
                }
                m_metadata.probeMilliseconds = elapsed(probeStart);
                readRecords();
            }
            const auto summaryStart = Clock::now();
            std::vector<MeshSamplingPointAssessment> points;
            points.reserve(sprays.size());
            for(std::size_t i = 0; i < sprays.size(); ++i) {
                if((i & 4095U) == 0) checkCancelled(execution);
                const auto& r = records[i];
                if(r[0] < 0 || (r[1] & 128) != 0) throw std::runtime_error("GPU mesh assessment traversal exceeded its bounds.");
                MeshSamplingPointAssessment point;
                point.sampleIndex = sprays[i].sourceIndex;
                point.timeSeconds = sprays[i].time;
                point.durationSeconds = sprays[i].duration;
                point.visibleVertexCount = static_cast<std::size_t>(r[0]);
                point.status = r[0] > 0 || (r[1] & 32) != 0 ? MeshSamplingStatus::Insufficient
                    : ((r[1] & 64) != 0 ? MeshSamplingStatus::BackFacing : MeshSamplingStatus::Missed);
                point.centerSampled = (r[1] & 16) != 0;
                if(r[0] >= 3 && (r[1] & 15) == 15) {
                    const double scale = m_momentScale, count = r[0];
                    const double x = r[2] / scale, y = r[3] / scale;
                    const double xx = std::max(0.0, r[4] / scale - x * x / count);
                    const double yy = std::max(0.0, r[6] / scale - y * y / count);
                    const double xy = r[5] / scale - x * y / count;
                    const double trace = xx + yy, difference = std::hypot(xx - yy, 2.0 * xy);
                    point.bothDirectionsSampled = trace > 0.0 && trace - difference >= 0.05 * (trace + difference);
                }
                points.push_back(point);
            }
            auto result = MeshSamplingEvaluator::summarize(task, std::move(points));
            result.gpuIntegrated = true;
            result.vertexMappingCacheHit = m_metadata.vertexMappingCacheHit;
            result.preparationMilliseconds = m_metadata.preparationMilliseconds;
            result.probeMilliseconds = m_metadata.probeMilliseconds;
            result.readbackMilliseconds = m_metadata.readbackMilliseconds;
            result.elapsedMilliseconds = elapsed(summaryStart);
            return std::make_shared<const MeshSamplingAssessment>(std::move(result));
        } catch(const std::exception& exception) {
            if(execution.cancelRequested && execution.cancelRequested->load()) throw;
            disable(exception.what());
            while(glGetError() != GL_NO_ERROR) {}
            return std::make_shared<const MeshSamplingAssessment>(m_metadata);
        }
    }
}
