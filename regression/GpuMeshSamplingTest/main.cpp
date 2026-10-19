#include <SprayThicknessPredictionOpenGL/MeshSamplingEvaluator.h>
#include <SprayThicknessPredictionOpenGL/OpenGLThicknessPredictionBackend.h>
#include <GLRuntime/GLRuntime.h>

#include <QGuiApplication>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QSurfaceFormat>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace
{
    using namespace spraythickness;
    using spraythickness::opengl::MeshSamplingEvaluator;
    using spraythickness::opengl::OpenGLThicknessPredictionBackend;

    void require(bool condition, const char* message)
    {
        if(!condition) throw std::runtime_error(message);
    }

    ThicknessPredictionTask makeTask(std::size_t count)
    {
        ThicknessPredictionTask task;
        task.tool.sprayDirectionLocal = -Eigen::Vector3d::UnitZ();
        task.options.enableHistoryCorrection = false;
        task.options.base.trajectorySamplingMode = TrajectorySamplingMode::OriginalPoints;
        task.options.trajectoryBatchSize = 1;
        auto& p = task.options.deposition;
        p.sigmaPhiRadians = p.sigmaPsiRadians = 0.03;
        p.phiOffsetRadians = p.psiOffsetRadians = p.rotationRadians = 0;
        sprayworkpiece::SurfaceSample vertex;
        vertex.normal = Eigen::Vector3d::UnitZ();
        vertex.position.setZero();
        task.workpiece.samples.push_back(vertex);
        for(std::size_t i = 1; i < count; ++i) {
            const double angle = (i - 1) * 6.283185307179586 / (count - 1);
            vertex.position = Eigen::Vector3d(0.002 * std::cos(angle), 0.002 * std::sin(angle), 0);
            task.workpiece.samples.push_back(vertex);
        }
        for(std::uint32_t i = 1; i < count; ++i) {
            task.workpiece.triangleIndices.insert(task.workpiece.triangleIndices.end(),
                { 0, i, i + 1 < count ? i + 1 : 1 });
        }
        spraytrajectory::SprayPathPoint start;
        start.tcpPose.translation() = Eigen::Vector3d(0, 0, 0.12);
        start.sprayEnabled = true;
        auto end = start;
        end.time = 0.02;
        end.sprayEnabled = false;
        spraytrajectory::SpraySegment segment;
        segment.points = { start, end };
        task.trajectory.segments.push_back(segment);
        return task;
    }

    void compare(OpenGLThicknessPredictionBackend& backend, ThicknessPredictionTask task)
    {
        task.options.evaluateMeshSampling = false;
        const auto before = backend.predict(task);
        const auto reference = MeshSamplingEvaluator::evaluate(task);
        task.options.evaluateMeshSampling = true;
        const auto after = backend.predict(task);
        require(after.meshSamplingAssessment != nullptr, "GPU assessment missing");
        const auto& actual = *after.meshSamplingAssessment;
        if(!actual.applicable) throw std::runtime_error(actual.unavailableReason);
        require(actual.gpuIntegrated && actual.points.size() == reference.points.size(), "GPU sample records differ");
        require(actual.totalPointCount == reference.totalPointCount
            && actual.airMovePointCount == reference.airMovePointCount
            && actual.ignoredPointCount == reference.ignoredPointCount, "Sampler summary differs");
        require(std::abs(actual.assessedDurationSeconds - reference.assessedDurationSeconds) < 1e-12
            && std::abs(actual.insufficientDurationSeconds - reference.insufficientDurationSeconds) < 1e-12,
            "Time-weighted assessment differs");
        for(std::size_t i = 0; i < actual.points.size(); ++i) {
            const auto& a = actual.points[i]; const auto& b = reference.points[i];
            require(a.sampleIndex == b.sampleIndex && a.visibleVertexCount == b.visibleVertexCount
                && a.status == b.status, "GPU unique visible count/status differs from CPU reference");
            require(a.centerSampled == b.centerSampled && a.bothDirectionsSampled == b.bothDirectionsSampled,
                "GPU footprint distribution differs");
        }
        require(after.field.results.size() == before.field.results.size(), "Thickness result size changed");
        for(std::size_t i = 0; i < after.field.results.size(); ++i) {
            const double expected = before.field.results[i].thickness;
            require(std::abs(after.field.results[i].thickness - expected) <= std::max(1e-12, std::abs(expected) * 1e-4),
                "Assessment changed deposition");
        }
        const auto cached = backend.predict(task);
        require(cached.meshSamplingAssessment->vertexMappingCacheHit, "Vertex mapping was not cached");
    }

    void runCases(bool spatial, bool shadow)
    {
        OpenGLThicknessPredictionBackend backend;
        auto setup = [=](ThicknessPredictionTask task) {
            task.options.enableBvhOcclusion = shadow;
            task.options.spatialFiltering.enabled = spatial;
            task.options.spatialFiltering.fallbackToFullPrediction = false;
            return task;
        };
        // Include multiple workgroups and a partial final group. Inactive lanes
        // must still reach the shared-counter barriers without changing counts.
        for(auto count : { 19U, 20U, 49U, 50U, 257U, 513U }) compare(backend, setup(makeTask(count)));
        auto task = makeTask(50);
        // Earlier duplicates face away: a fixed canonical index would lose all samples.
        auto front = task.workpiece.samples;
        for(auto& vertex : task.workpiece.samples) vertex.normal = -Eigen::Vector3d::UnitZ();
        task.workpiece.samples.insert(task.workpiece.samples.end(), front.begin(), front.end());
        const auto frontFaces = task.workpiece.triangleIndices;
        for(auto index : frontFaces) task.workpiece.triangleIndices.push_back(index + 50);
        compare(backend, setup(task));
        task.workpiece.samples[50].valid = false;
        compare(backend, setup(task));
        task.options.spatialFiltering.predictionVertexIndices = { 51, 52, 53 };
        compare(backend, setup(task));

        // Both layers have vertices inside the core. Only the visible layer counts,
        // including when shadowing of the thickness field itself is disabled.
        task = makeTask(50);
        auto lower = task.workpiece.samples;
        for(auto& vertex : lower) vertex.position.z() = -0.02;
        task.workpiece.samples.insert(task.workpiece.samples.end(), lower.begin(), lower.end());
        const auto faces = task.workpiece.triangleIndices;
        for(auto index : faces) task.workpiece.triangleIndices.push_back(index + 50);
        compare(backend, setup(task));

        task = makeTask(3);
        task.workpiece.samples[0].position = Eigen::Vector3d(-1, -1, 0);
        task.workpiece.samples[1].position = Eigen::Vector3d(1, -1, 0);
        task.workpiece.samples[2].position = Eigen::Vector3d(0, 1, 0);
        task.workpiece.triangleIndices = { 0, 1, 2 };
        compare(backend, setup(task)); // zero vertices, but footprint hits triangle interior
        task.workpiece.samples[0].position = Eigen::Vector3d(0.006, -0.1, 0);
        task.workpiece.samples[1].position = Eigen::Vector3d(0.007, -0.1, 0);
        task.workpiece.samples[2].position = Eigen::Vector3d(0.006, 0.1, 0);
        compare(backend, setup(task)); // peak ray misses; edge of footprint still hits
        for(auto& vertex : task.workpiece.samples) vertex.normal = -Eigen::Vector3d::UnitZ();
        compare(backend, setup(task)); // back-facing coarse mesh
        for(auto& point : task.trajectory.segments.front().points) point.tcpPose.translation().x() = 2;
        compare(backend, setup(task)); // complete miss

        task = makeTask(50);
        auto& points = task.trajectory.segments.front().points;
        auto prototype = points.front();
        points.clear();
        for(int i = 0; i < 4; ++i) {
            auto point = prototype;
            point.time = i == 3 ? 0.055 : i * 0.02;
            point.tcpPose.translation().x() = i == 1 ? 0.1 : 0;
            point.sprayEnabled = i != 3;
            points.push_back(point);
        }
        compare(backend, setup(task)); // global spray IDs across single-pose batches
        task.options.spatialFiltering.contributionCutoffRatio = 0.5;
        compare(backend, setup(task)); // sampling core must remain at 5%, independently of deposition cutoff

        // More than 128 active sprays force shared-table collisions in a single
        // dispatch. Two distant patches also give a workgroup different spatial
        // candidate lists; exact global fallback must preserve every contribution.
        task = makeTask(257);
        auto right = task.workpiece.samples;
        for(auto& vertex : right) vertex.position.x() += 0.1;
        task.workpiece.samples.insert(task.workpiece.samples.end(), right.begin(), right.end());
        const auto leftFaces = task.workpiece.triangleIndices;
        for(auto index : leftFaces) task.workpiece.triangleIndices.push_back(index + 257);
        auto& manyPoints = task.trajectory.segments.front().points;
        prototype = manyPoints.front();
        manyPoints.clear();
        for(int i = 0; i <= 130; ++i) {
            auto point = prototype;
            point.time = i * 0.002;
            point.tcpPose.translation().x() = i % 4 == 0 ? 0.0 : (i % 4 == 1 ? 0.1 : 0.2);
            point.sprayEnabled = i != 130;
            manyPoints.push_back(point);
        }
        task.options.trajectoryBatchSize = 256;
        compare(backend, setup(task));
    }
}

int main(int argc, char** argv)
{
    QGuiApplication app(argc, argv);
    QSurfaceFormat format;
    format.setVersion(4, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QOpenGLContext context;
    context.setFormat(format);
    if(!context.create()) return 77;
    QOffscreenSurface surface;
    surface.setFormat(context.format());
    surface.create();
    if(!context.makeCurrent(&surface) || !GLRuntime::instance().initialize()) return 77;
    try {
        for(bool spatial : { false, true }) for(bool shadow : { false, true }) runCases(spatial, shadow);
        return 0;
    } catch(const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
