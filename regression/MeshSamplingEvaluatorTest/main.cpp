#include <SprayThicknessPredictionOpenGL/MeshSamplingEvaluator.h>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace
{
    using namespace spraythickness;
    using spraythickness::opengl::MeshSamplingEvaluator;

    void require(bool condition, const char* message)
    {
        if(!condition) throw std::runtime_error(message);
    }

    void addFan(ThicknessPredictionTask& task, std::size_t count,
        const Eigen::Vector3d& center = Eigen::Vector3d::Zero(), double radius = 0.002)
    {
        auto& mesh = task.workpiece;
        const auto base = static_cast<std::uint32_t>(mesh.samples.size());
        sprayworkpiece::SurfaceSample sample;
        sample.position = center;
        mesh.samples.push_back(sample);
        for(std::size_t i = 1; i < count; ++i) {
            const double angle = (i - 1) * 6.283185307179586 / (count - 1);
            sample.position = center + Eigen::Vector3d(radius * std::cos(angle), radius * std::sin(angle), 0);
            mesh.samples.push_back(sample);
        }
        for(std::uint32_t i = 1; i < count; ++i) {
            mesh.triangleIndices.insert(mesh.triangleIndices.end(),
                { base, base + i, base + (i + 1 < count ? i + 1 : 1) });
        }
    }

    ThicknessPredictionTask taskWithFan(std::size_t count)
    {
        ThicknessPredictionTask task;
        task.tool.sprayDirectionLocal = -Eigen::Vector3d::UnitZ();
        task.options.deposition.sigmaPhiRadians = 0.03;
        task.options.deposition.sigmaPsiRadians = 0.03;
        task.options.deposition.rotationRadians = 0;
        task.options.deposition.phiOffsetRadians = 0;
        task.options.deposition.psiOffsetRadians = 0;
        addFan(task, count);
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

    void thresholdAndDistributionTests()
    {
        for(std::size_t count : { 19U, 20U, 49U, 50U }) {
            const auto result = MeshSamplingEvaluator::evaluate(taskWithFan(count));
            require(result.applicable && result.assessedPointCount == 1, "Expected one assessed point");
            require(result.points.front().visibleVertexCount == count, "Unique vertex count differs");
            const auto expected = count < 20 ? MeshSamplingStatus::Insufficient
                : count < 50 ? MeshSamplingStatus::Minimum : MeshSamplingStatus::Recommended;
            require(result.points.front().status == expected, "20/50 threshold boundary differs");
            require(result.airMovePointCount == 1, "Powder-off endpoint must be excluded");
        }
        auto task = taskWithFan(50);
        for(auto& sample : task.workpiece.samples) {
            sample.position *= 0.1;
            sample.position.x() += 0.003;
        }
        const auto restricted = MeshSamplingEvaluator::evaluate(task);
        require(restricted.recommendedPointCount == 1, "Count classification must remain independent");
        require(restricted.limitedDistributionPointCount == 1, "One-sided patch must trigger distribution reminder");
        task = taskWithFan(50);
        for(auto& sample : task.workpiece.samples) {
            const double x = sample.position.x(), y = sample.position.y() * 0.005;
            sample.position.x() = (x - y) * std::sqrt(0.5);
            sample.position.y() = (x + y) * std::sqrt(0.5);
        }
        const auto ribbon = MeshSamplingEvaluator::evaluate(task);
        require(ribbon.recommendedPointCount == 1 && ribbon.directionalGapPointCount == 1,
            "Diagonal narrow ribbon must not pass two-dimensional coverage");
    }

    void geometryHitTests()
    {
        auto task = taskWithFan(3);
        task.workpiece.samples[0].position = Eigen::Vector3d(-1, -1, 0);
        task.workpiece.samples[1].position = Eigen::Vector3d(1, -1, 0);
        task.workpiece.samples[2].position = Eigen::Vector3d(0, 1, 0);
        task.workpiece.triangleIndices = { 0, 1, 2 };
        auto result = MeshSamplingEvaluator::evaluate(task);
        require(result.assessedPointCount == 1 && result.minimumVisibleVertexCount == 0
            && result.insufficientPointCount == 1, "Interior hit without vertices must be insufficient, not missed");

        // Peak ray misses this thin triangle, but the edge of the core footprint hits it.
        task.workpiece.samples[0].position = Eigen::Vector3d(0.006, -0.1, 0);
        task.workpiece.samples[1].position = Eigen::Vector3d(0.007, -0.1, 0);
        task.workpiece.samples[2].position = Eigen::Vector3d(0.006, 0.1, 0);
        result = MeshSamplingEvaluator::evaluate(task);
        require(result.assessedPointCount == 1 && result.minimumVisibleVertexCount == 0,
            "Partial footprint hit must not depend on peak ray or vertex hits");

        task.trajectory.segments[0].points[0].tcpPose.translation().x() = 2.0;
        result = MeshSamplingEvaluator::evaluate(task);
        require(result.assessedPointCount == 0 && result.missedPointCount == 1,
            "Complete miss must be separate from insufficient sampling");

        task = taskWithFan(50);
        for(auto& sample : task.workpiece.samples) sample.normal = -Eigen::Vector3d::UnitZ();
        result = MeshSamplingEvaluator::evaluate(task);
        require(result.assessedPointCount == 0 && result.backFacingPointCount == 1,
            "Back-facing incidence must not appear as an empty trajectory");
    }

    void deduplicationAndOcclusionTests()
    {
        auto task = taskWithFan(20);
        addFan(task, 20); // Same physical vertices, separate STL records.
        auto result = MeshSamplingEvaluator::evaluate(task);
        require(result.minimumVisibleVertexCount == 20, "Duplicate STL positions inflated the count");
        addFan(task, 50, Eigen::Vector3d(0, 0, -0.02));
        task.options.enableBvhOcclusion = false;
        result = MeshSamplingEvaluator::evaluate(task);
        require(result.minimumVisibleVertexCount == 20, "Occluded lower plate must not inflate sampling quality");

        task = taskWithFan(50);
        task.options.spatialFiltering.predictionVertexIndices = { 0 };
        result = MeshSamplingEvaluator::evaluate(task);
        require(result.minimumVisibleVertexCount == 1, "Restricted prediction list was ignored");
    }

    void timeAndExecutionTests()
    {
        auto task = taskWithFan(19);
        addFan(task, 50, Eigen::Vector3d(0.05, 0, 0));
        auto& points = task.trajectory.segments[0].points;
        auto end = points.back();
        end.time = 0.04;
        points[1].sprayEnabled = true;
        points[1].time = 0.01;
        points[1].tcpPose.translation().x() = 0.05;
        points.push_back(end);
        auto result = MeshSamplingEvaluator::evaluate(task);
        require(result.assessedPointCount == 2, "Both active intervals must be assessed");
        require(std::abs(result.insufficientDurationSeconds / result.assessedDurationSeconds - 0.25) < 1.0e-12,
            "Time weighting must use the next timestamp, not point count");
        require(result.worstSampleIndex == 0 && result.worstTimeSeconds == 0.0,
            "Worst sample identity was lost");

        task.options.base.trajectorySamplingMode = TrajectorySamplingMode::ResampleByTimeStep;
        task.options.base.timeStep = 0.005;
        result = MeshSamplingEvaluator::evaluate(task);
        require(result.totalPointCount > 3, "Applied interpolation must be used by the assessment");

        task.options.axisymmetricProfile.enabled = true;
        require(!MeshSamplingEvaluator::evaluate(task).applicable, "Reduced mode must not report full-mesh quality");
        std::atomic_bool cancelled{ true };
        ThicknessPredictionExecution execution;
        execution.cancelRequested = &cancelled;
        bool threw = false;
        try { MeshSamplingEvaluator::evaluate(task, execution); }
        catch(const std::runtime_error&) { threw = true; }
        require(threw, "Cancellation was not observed");
    }
    void poseAndEmptyTests()
    {
        auto task = taskWithFan(50);
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.linear() = Eigen::AngleAxisd(0.73,
            Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
        transform.translation() = Eigen::Vector3d(0.4, -0.3, 0.2);
        for(auto& sample : task.workpiece.samples) {
            sample.position = transform * sample.position;
            sample.normal = transform.linear() * sample.normal;
        }
        for(auto& point : task.trajectory.segments[0].points) point.tcpPose = transform * point.tcpPose;
        auto result = MeshSamplingEvaluator::evaluate(task);
        require(result.recommendedPointCount == 1 && result.minimumVisibleVertexCount == 50,
            "Assessment must be invariant under a common rigid transform");
        for(auto& point : task.trajectory.segments[0].points) point.sprayEnabled = false;
        result = MeshSamplingEvaluator::evaluate(task);
        require(result.airMovePointCount == 2 && result.points.empty() && result.assessedPointCount == 0,
            "All-air trajectory must not be rated sufficient or counted as missed");
        task.workpiece.triangleIndices.clear();
        require(!MeshSamplingEvaluator::evaluate(task).applicable,
            "Vertex-only input cannot distinguish a coarse mesh from a miss");
    }
}

int main()
{
    try {
        thresholdAndDistributionTests();
        geometryHitTests();
        deduplicationAndOcclusionTests();
        timeAndExecutionTests();
        poseAndEmptyTests();
        std::cout << "Mesh sampling evaluator regression checks passed.\n";
        return 0;
    } catch(const std::exception& exception) {
        std::cerr << exception.what() << '\n';
        return 1;
    }
}
