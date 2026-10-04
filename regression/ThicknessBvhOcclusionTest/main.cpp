#include <SprayThicknessPredictionOpenGL/OpenGLThicknessPredictionBackend.h>
#include <SprayThicknessPredictionOpenGL/ThicknessBvh.h>
#include <GLRuntime/GLRuntime.h>

#include <QGuiApplication>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QOpenGLBuffer>
#include <QSurfaceFormat>

#include <cmath>
#include <algorithm>
#include <iostream>

namespace
{
    spraythickness::ThicknessPredictionTask makeTask(bool spatialFiltering, bool bvh)
    {
        spraythickness::ThicknessPredictionTask task;
        task.process.id = "occlusion-test";
        task.options.enableBvhOcclusion = bvh;
        task.options.enableHistoryCorrection = false;
        task.options.spatialFiltering.enabled = spatialFiltering;
        task.options.spatialFiltering.fallbackToFullPrediction = false;
        task.options.base.trajectorySamplingMode =
            spraythickness::TrajectorySamplingMode::OriginalPoints;
        task.options.deposition.sigmaPhiRadians = 0.2;
        task.options.deposition.sigmaPsiRadians = 0.2;

        for(double z : { 0.0, -0.015 }) {
            const std::uint32_t offset = static_cast<std::uint32_t>(
                task.workpiece.samples.size());
            for(int y = -1; y <= 1; ++y) {
                for(int x = -1; x <= 1; ++x) {
                    sprayworkpiece::SurfaceSample sample;
                    sample.position = Eigen::Vector3d(
                        0.01 * x, 0.01 * y, z);
                    sample.normal = Eigen::Vector3d::UnitZ();
                    task.workpiece.samples.push_back(sample);
                }
            }
            for(std::uint32_t y = 0; y < 2; ++y) {
                for(std::uint32_t x = 0; x < 2; ++x) {
                    const std::uint32_t a = offset + y * 3 + x;
                    task.workpiece.triangleIndices.insert(
                        task.workpiece.triangleIndices.end(),
                        { a, a + 1, a + 3, a + 1, a + 4, a + 3 });
                }
            }
        }

        spraytrajectory::SpraySegment segment;
        segment.processId = task.process.id;
        for(double time : { 0.0, 0.02 }) {
            spraytrajectory::SprayPathPoint point;
            point.time = time;
            point.tcpPose.translation() = Eigen::Vector3d(0.0, 0.0, 0.12);
            point.sprayEnabled = time == 0.0;
            segment.points.push_back(point);
        }
        task.trajectory.segments.push_back(std::move(segment));
        return task;
    }

    bool verifyOcclusion(bool spatialFiltering)
    {
        spraythickness::opengl::OpenGLThicknessPredictionBackend backend;
        const auto unoccluded = backend.predict(makeTask(spatialFiltering, false));
        const auto occluded = backend.predict(makeTask(spatialFiltering, true));
        const double top = occluded.field.results[4].thickness;
        const double lowerBefore = unoccluded.field.results[13].thickness;
        const double lowerAfter = occluded.field.results[13].thickness;
        std::cout << (spatialFiltering ? "spatial" : "complete")
                  << " top=" << top << " lowerBefore=" << lowerBefore
                  << " lowerAfter=" << lowerAfter << '\n';
        return top > 0.0 && lowerBefore > 0.0
            && std::abs(lowerAfter) < 1.0e-12;
    }

    bool verifyOnlineAccumulation()
    {
        spraythickness::ThicknessPredictionTask task = makeTask(false, true);
        task.options.enableHistoryCorrection = true;
        auto& points = task.trajectory.segments.front().points;
        points.front().sprayEnabled = true;
        points.back().sprayEnabled = true;
        spraytrajectory::SprayPathPoint end = points.back();
        end.time = 0.04;
        end.sprayEnabled = false;
        points.push_back(end);

        spraythickness::opengl::OpenGLThicknessPredictionBackend backend;
        const auto offline = backend.predict(task);
        spraythickness::ThicknessPredictionTask onlineTask = task;
        onlineTask.trajectory = spraytrajectory::SprayTrajectory();
        backend.beginOnline(std::move(onlineTask));

        spraytrajectory::SprayTrajectory first;
        first.segments.push_back(task.trajectory.segments.front());
        first.segments.front().points.resize(2);
        const auto firstResult = backend.appendOnline(first);

        spraytrajectory::SprayTrajectory second;
        second.segments.push_back(task.trajectory.segments.front());
        second.segments.front().points.erase(
            second.segments.front().points.begin());
        const auto finalResult = backend.appendOnline(second);
        backend.endOnline();

        const double firstTop = firstResult.field.results[4].thickness;
        const double finalTop = finalResult.field.results[4].thickness;
        const double offlineTop = offline.field.results[4].thickness;
        const double lower = finalResult.field.results[13].thickness;
        std::cout << "online first=" << firstTop << " final=" << finalTop
                  << " offline=" << offlineTop << " lower=" << lower << '\n';
        return firstTop > 0.0 && finalTop > firstTop
            && std::abs(finalTop - offlineTop)
                < std::max(offlineTop * 1.0e-4, 1.0e-12)
            && std::abs(lower) < 1.0e-12;
    }

    bool verifyLiveOnlineToolDirections()
    {
        auto task = makeTask(false, true);
        task.options.deposition.sigmaPhiRadians = 0.04;
        task.options.deposition.sigmaPsiRadians = 0.16;
        auto onlineTask = task;
        onlineTask.trajectory = {};
        spraythickness::opengl::OpenGLThicknessPredictionBackend backend;
        backend.beginOnline(std::move(onlineTask));
        const auto first = backend.appendOnline(task.trajectory);
        auto next = task.trajectory;
        for(auto& point : next.segments.front().points) point.time += 0.02;
        backend.setOnlineToolDirections(Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY());
        const auto away = backend.appendOnline(next);
        for(std::size_t i = 0; i < first.field.results.size(); ++i) {
            if(away.field.results[i].thickness != first.field.results[i].thickness) return false;
        }
        for(auto& point : next.segments.front().points) point.time += 0.02;
        backend.setOnlineToolDirections(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitY());
        const auto accumulated = backend.appendOnline(next);
        backend.endOnline();
        auto referenceTask = task;
        referenceTask.tool.powderFeedDirectionLocal = Eigen::Vector3d::UnitY();
        referenceTask.trajectory = next;
        const auto rotatedPattern = backend.predict(referenceTask);
        if(first.field.results[4].thickness <= 0.0
            || std::abs(first.field.results[1].thickness - rotatedPattern.field.results[1].thickness)
                <= first.field.results[4].thickness * 0.01) return false;
        for(std::size_t i = 0; i < first.field.results.size(); ++i) {
            const double expected = first.field.results[i].thickness + rotatedPattern.field.results[i].thickness;
            if(std::abs(accumulated.field.results[i].thickness - expected)
                > std::max(1.0e-12, expected * 1.0e-4)) return false;
        }
        return accumulated.field.results[13].thickness == 0.0;
    }

    bool verifyOnlineBatchPartition(bool history)
    {
        auto task = makeTask(false, true);
        for(std::size_t index = 0; index < task.workpiece.samples.size(); ++index) {
            task.workpiece.samples[index].targetThickness = 0.00004 + index * 1.0e-7;
        }
        task.options.enableHistoryCorrection = history;
        auto& points = task.trajectory.segments.front().points;
        const auto prototype = points.front();
        points.clear();
        for(int index = 0; index <= 21; ++index) {
            auto point = prototype;
            point.time = index == 21 ? 0.415 : index * 0.02;
            point.tcpPose.translation().x() = -0.004 + index * 0.0004;
            point.sprayEnabled = index < 21;
            points.push_back(point);
        }
        spraythickness::opengl::OpenGLThicknessPredictionBackend backend;
        const auto offline = backend.predict(task);
        for(std::size_t batchSize : { std::size_t(1), std::size_t(2), std::size_t(13), std::size_t(64) }) {
            auto onlineTask = task;
            onlineTask.trajectory = {};
            backend.beginOnline(std::move(onlineTask));
            spraythickness::ThicknessPredictionResult final;
            for(std::size_t begin = 0; begin + 1 < points.size();) {
                const auto end = std::min(begin + batchSize, points.size() - 1);
                spraytrajectory::SprayTrajectory interval;
                auto segment = task.trajectory.segments.front();
                segment.points.assign(points.begin() + begin, points.begin() + end + 1);
                interval.segments.push_back(std::move(segment));
                backend.appendOnline(interval, final);
                begin = end;
            }
            backend.endOnline();
            for(std::size_t index = 0; index < offline.field.results.size(); ++index) {
                const double expected = offline.field.results[index].thickness;
                if(std::abs(final.field.results[index].thickness - expected)
                    > std::max(expected * 1.0e-4, 1.0e-12)) return false;
                if(final.field.results[index].sampleIndex != index
                    || final.field.results[index].targetThickness
                        != task.workpiece.samples[index].targetThickness
                    || std::abs(final.field.results[index].error
                        - (final.field.results[index].thickness
                            - task.workpiece.samples[index].targetThickness)) > 1.0e-15) return false;
            }
            if(final.timing.sprayPointCount != 21 || final.field.results[13].thickness != 0.0
                || !std::isfinite(final.timing.pureGpuMilliseconds)
                || final.timing.readbackMilliseconds < 0.0) return false;
            std::cout << "online partition: history=" << history << " batch=" << batchSize
                << " top=" << final.field.results[4].thickness
                << " gpuMs=" << final.timing.pureGpuMilliseconds << '\n';
        }
        // Reusing a result of the same size after Begin must use the new targets.
        auto resetTask = task;
        for(auto& sample : resetTask.workpiece.samples) sample.targetThickness = 0.0001;
        backend.beginOnline(std::move(resetTask));
        auto reused = offline;
        backend.appendOnline(task.trajectory, reused);
        for(const auto& sample : reused.field.results) {
            if(sample.targetThickness != 0.0001
                || std::abs(sample.error - (sample.thickness - 0.0001)) > 1.0e-15) return false;
        }
        backend.endOnline();
        return true;
    }

    bool verifyNarrowFootprintOcclusion()
    {
        auto task = makeTask(false, true);
        task.options.enableHistoryCorrection = true;
        task.options.deposition.sigmaPhiRadians = 0.001;
        task.options.deposition.sigmaPsiRadians = 0.001;
        spraythickness::opengl::OpenGLThicknessPredictionBackend backend;
        const auto shadowed = backend.predict(task);
        task.options.enableBvhOcclusion = false;
        const auto visible = backend.predict(task);
        if(shadowed.field.results[4].thickness <= 0.0
            || visible.field.results[13].thickness <= 0.0
            || shadowed.field.results[13].thickness != 0.0) return false;
        for(std::size_t i = 0; i < shadowed.field.results.size(); ++i) {
            if(i != 4 && i != 13 && (shadowed.field.results[i].thickness != 0.0
                || visible.field.results[i].thickness != 0.0)) return false;
        }
        return true;
    }

    bool verifyOnlineSnapshot()
    {
        auto task = makeTask(false, true);
        task.options.enableHistoryCorrection = true;
        for(std::size_t i = 0; i < task.workpiece.samples.size(); ++i) {
            task.workpiece.samples[i].targetThickness = 1.0e-5 + i * 1.0e-6;
        }
        spraythickness::opengl::OpenGLThicknessPredictionBackend backend;
        backend.beginOnline(task);
        spraythickness::OnlineThicknessSnapshot compact;
        backend.appendOnline(task.trajectory, compact);
        const auto retained = compact;
        // Requesting a full result without adding spray must see the same data.
        const auto full = backend.appendOnline({});
        if(compact.size() != full.field.results.size()) return false;
        for(std::size_t i = 0; i < compact.size(); ++i) {
            if(compact.thicknessMeters(i) != full.field.results[i].thickness) return false;
        }
        if(compact.metrics.averageThickness != full.metrics.averageThickness
            || compact.metrics.maxThickness != full.metrics.maxThickness
            || compact.metrics.meanError != full.metrics.meanError
            || compact.metrics.coverageRatio != full.metrics.coverageRatio
            || compact.timing.sprayPointCount != full.timing.sprayPointCount) return false;
        auto next = task.trajectory;
        for(auto& point : next.segments.front().points) point.time += 0.02;
        backend.appendOnline(next, compact);
        if(compact.thicknessMeters(4) <= retained.thicknessMeters(4)
            || compact.thicknessMeters(13) != 0.0) return false;
        backend.endOnline();
        backend.beginOnline(task);
        backend.appendOnline({}, compact);
        return compact.metrics.maxThickness == 0.0 && compact.timing.sprayPointCount == 0;
    }

    bool verifyGpuDisplaySnapshot()
    {
        auto task = makeTask(false, true);
        task.options.enableHistoryCorrection = true;
        task.options.base.coverageTolerance = 2.0e-6;
        task.options.base.overCoatTolerance = 3.0e-6;
        for(std::size_t i = 0; i < task.workpiece.samples.size(); ++i)
            task.workpiece.samples[i].targetThickness = 1.0e-5 + i * 1.0e-6;
        spraythickness::opengl::OpenGLThicknessPredictionBackend backend;
        backend.beginOnline(task);
        // Deliberately reordered and duplicated, as with multiple display submeshes.
        const std::vector<std::uint32_t> mapping{ 13, 4, 0, 17, 4, 8 };
        backend.setOnlineDisplayMapping(mapping);
        QOpenGLBuffer first;
        if(!first.create() || !first.bind()) return false;
        first.allocate(static_cast<int>(mapping.size() * sizeof(float)));
        first.release();
        spraythickness::OnlineThicknessSnapshot gpu;
        backend.appendOnlineGpu(task.trajectory, first.bufferId(), gpu);
        if(!gpu.thicknessMillimeters.empty() || gpu.size() != task.workpiece.samples.size()
            || !gpu.timing.gpuResidentDisplay || gpu.timing.thicknessReadbackBytes != 0
            || gpu.finiteVertexCount != gpu.size()) return false;
        // Explicit readback for numerical verification, never part of display.
        const auto cpu = backend.appendOnline({});
        const auto close = [](double a, double b) {
            return std::abs(a - b) <= std::max(1.0e-18, std::max(std::abs(a), std::abs(b)) * 1.0e-9);
        };
        if(!close(gpu.metrics.minThickness, cpu.metrics.minThickness)
            || !close(gpu.metrics.maxThickness, cpu.metrics.maxThickness)
            || !close(gpu.metrics.averageThickness, cpu.metrics.averageThickness)
            || !close(gpu.metrics.meanError, cpu.metrics.meanError)
            || !close(gpu.metrics.maxAbsError, cpu.metrics.maxAbsError)
            || !close(gpu.metrics.coverageRatio, cpu.metrics.coverageRatio)
            || !close(gpu.metrics.underCoatedRatio, cpu.metrics.underCoatedRatio)
            || !close(gpu.metrics.overCoatedRatio, cpu.metrics.overCoatedRatio)) return false;
        double variance = 0.0;
        for(const auto& sample : cpu.field.results) {
            const auto difference = sample.thickness - cpu.metrics.averageThickness;
            variance += difference * difference;
        }
        variance /= cpu.field.results.size();
        if(!close(gpu.varianceSquareMeters, variance)) return false;

        QOpenGLContext* producer = QOpenGLContext::currentContext();
        auto* producerSurface = producer->surface();
        QOpenGLContext consumer;
        consumer.setFormat(producer->format());
        consumer.setShareContext(producer);
        if(!consumer.create()) return false;
        QOffscreenSurface surface;
        surface.setFormat(consumer.format());
        surface.create();
        if(!consumer.makeCurrent(&surface)) return false;
        std::vector<float> retained(mapping.size());
        bool matches = first.bind() && first.read(0, retained.data(),
            static_cast<int>(retained.size() * sizeof(float)));
        first.release();
        for(std::size_t i = 0; i < mapping.size(); ++i)
            matches = matches && close(static_cast<double>(retained[i]) * 1.0e-3,
                cpu.field.results[mapping[i]].thickness);
        if(!producer->makeCurrent(producerSurface)) return false;

        QOpenGLBuffer second;
        if(!second.create() || !second.bind()) return false;
        second.allocate(static_cast<int>(mapping.size() * sizeof(float)));
        second.release();
        auto next = task.trajectory;
        for(auto& point : next.segments.front().points) point.time += 0.02;
        backend.appendOnlineGpu(next, second.bufferId(), gpu);
        if(gpu.metrics.maxThickness <= cpu.metrics.maxThickness) return false;
        const auto averageBefore = gpu.metrics.averageThickness;
        for(auto& point : next.segments.front().points) point.time += 0.02;
        backend.appendOnlineGpu(next, second.bufferId(), gpu, {}, false);
        const auto third = backend.appendOnline({});
        if(!close(gpu.metrics.minThickness, third.metrics.minThickness)
            || !close(gpu.metrics.maxThickness, third.metrics.maxThickness)
            || gpu.metrics.averageThickness != averageBefore) return false;
        // Full statistics catch up without adding any new physical exposure.
        backend.appendOnlineGpu({}, second.bufferId(), gpu);
        if(!close(gpu.metrics.averageThickness, third.metrics.averageThickness)) return false;
        backend.endOnline();
        // A displayed frame survives both subsequent accumulation and reset.
        std::vector<float> after(mapping.size());
        if(!first.bind() || !first.read(0, after.data(), static_cast<int>(after.size() * sizeof(float)))) return false;
        first.release();
        return matches && after == retained && retained[0] == 0.0f && retained[1] > 0.0f;
    }

    bool verifyShortIntervalExposure()
    {
        // Adaptive integration and the last interval of a frame can be shorter
        // than a microsecond. Subdivision must not manufacture extra spray time.
        spraythickness::opengl::OpenGLThicknessPredictionBackend backend;
        double coarse = 0.0;
        for(int count : { 1, 32 }) {
            auto task = makeTask(false, true);
            auto& points = task.trajectory.segments.front().points;
            const auto prototype = points.front();
            points.clear();
            for(int index = 0; index <= count; ++index) {
                auto point = prototype;
                point.time = 4.0e-6 * index / count;
                points.push_back(point);
            }
            const auto trajectory = task.trajectory;
            task.trajectory = {};
            backend.beginOnline(std::move(task));
            const auto result = backend.appendOnline(trajectory);
            backend.endOnline();
            const double top = result.field.results[4].thickness;
            if(top <= 0.0 || result.field.results[13].thickness != 0.0) return false;
            if(count == 1) coarse = top;
            else if(std::abs(top - coarse) > coarse * 1.0e-4) return false;
        }
        return true;
    }

    bool verifyPausedOnlineAccumulation()
    {
        spraythickness::ThicknessPredictionTask task = makeTask(false, true);
        task.trajectory = spraytrajectory::SprayTrajectory();
        spraythickness::opengl::OpenGLThicknessPredictionBackend backend;
        backend.beginOnline(std::move(task));

        auto interval = [](double start) {
            spraytrajectory::SprayTrajectory trajectory;
            spraytrajectory::SpraySegment segment;
            segment.processId = "occlusion-test";
            for(const double time : { start, start + 0.02 }) {
                spraytrajectory::SprayPathPoint point;
                point.time = time;
                point.tcpPose.translation() = Eigen::Vector3d(0.0, 0.0, 0.12);
                point.sprayEnabled = true;
                segment.points.push_back(point);
            }
            trajectory.segments.push_back(std::move(segment));
            return trajectory;
        };
        const auto first = backend.appendOnline(interval(0.0));
        const auto second = backend.appendOnline(interval(1.0));
        backend.endOnline();

        const double firstTop = first.field.results[4].thickness;
        const double secondTop = second.field.results[4].thickness;
        std::cout << "online paused first=" << firstTop
                  << " final=" << secondTop << '\n';
        return firstTop > 0.0
            && std::abs(secondTop - 2.0 * firstTop)
                < std::max(firstTop * 1.0e-4, 1.0e-12);
    }
}

int main(int argc, char** argv)
{
    // Distance inside an empty AABB region must be measured to the actual faces.
    const auto mesh = makeTask(false, true).workpiece;
    const auto bvh = spraythickness::opengl::ThicknessBvhBuilder::build(mesh);
    if(std::abs(bvh.surfaceDistance(mesh, Eigen::Vector3d(0, 0, -0.0075)) - 0.0075) > 1.0e-10
        || std::abs(bvh.surfaceDistance(mesh, Eigen::Vector3d(0.02, 0.02, 0)) - std::sqrt(0.0002)) > 1.0e-10
        || bvh.surfaceDistance(mesh, Eigen::Vector3d(0, 0, 0)) > 1.0e-10) return 1;
    QGuiApplication application(argc, argv);
    QSurfaceFormat format;
    format.setVersion(4, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    QOpenGLContext context;
    context.setFormat(format);
    if(!context.create()) return 77;
    QOffscreenSurface surface;
    surface.setFormat(context.format());
    surface.create();
    if(!context.makeCurrent(&surface) || !GLRuntime::instance().initialize()) {
        return 77;
    }
    try {
        std::cerr << "Checking paused online accumulation..." << std::endl;
        const bool paused = verifyPausedOnlineAccumulation();
        std::cerr << "Checking complete-mesh occlusion..." << std::endl;
        const bool complete = verifyOcclusion(false);
        std::cerr << "Checking spatial occlusion..." << std::endl;
        const bool spatial = verifyOcclusion(true);
        std::cerr << "Checking online accumulation..." << std::endl;
        return paused && complete && spatial && verifyOnlineAccumulation()
            && verifyLiveOnlineToolDirections()
            && verifyOnlineBatchPartition(false) && verifyOnlineBatchPartition(true)
            && verifyShortIntervalExposure() && verifyOnlineSnapshot()
            && verifyGpuDisplaySnapshot()
            && verifyNarrowFootprintOcclusion() ? 0 : 1;
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
