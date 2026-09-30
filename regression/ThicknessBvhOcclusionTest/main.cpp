#include <SprayThicknessPredictionOpenGL/OpenGLThicknessPredictionBackend.h>
#include <GLRuntime/GLRuntime.h>

#include <QGuiApplication>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QSurfaceFormat>

#include <cmath>
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
}

int main(int argc, char** argv)
{
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
        return verifyOcclusion(false) && verifyOcclusion(true) ? 0 : 1;
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
