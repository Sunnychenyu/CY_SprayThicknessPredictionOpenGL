#include <ProjectMotionPlanning/ProjectMotionPlanning.h>

#include <MotionPlanningCore/MotionPlanning.h>
#include <RobotRuntime/RobotTrajectoryExecutionSession.h>
#include <SimulationProject/ProjectIo.h>

#include <data_path.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace
{
    struct Options
    {
        std::filesystem::path projectPath = std::filesystem::path(PROJECT_SOURCE_PATH) /
            "config/projects/420-red4600-tool.sys.json";
        bool discover = false;
    };

    bool parseOptions(int argc, char** argv, Options& options)
    {
        for (int index = 1; index < argc; ++index)
        {
            const std::string argument = argv[index];
            if (argument == "--project" && index + 1 < argc)
                options.projectPath = std::filesystem::u8path(argv[++index]);
            else if (argument == "--discover")
                options.discover = true;
            else
            {
                std::cerr << "Unknown or incomplete option: " << argument << "\n";
                return false;
            }
        }
        return true;
    }

    std::string vectorText(const std::vector<double>& values)
    {
        std::ostringstream stream;
        stream << std::setprecision(15);
        for (std::size_t index = 0; index < values.size(); ++index)
        {
            if (index != 0)
                stream << ',';
            stream << values[index];
        }
        return stream.str();
    }

    struct Candidate
    {
        std::vector<double> joints;
        Eigen::Vector3d toolPosition = Eigen::Vector3d::Zero();
    };

    bool findFixture(
        motion_planning::ProjectPlanningSceneSnapshot& scene,
        motion_planning::ProjectPlanningRequest& request,
        std::vector<double>& start,
        std::vector<double>& goal)
    {
        const auto* positioner = scene.simulationRuntime().robot("ATPPZ350");
        if (positioner == nullptr)
            return false;
        const Eigen::Vector3d center = positioner->baseTransform.translation();

        std::mt19937 generator(4204600u);
        std::vector<std::uniform_real_distribution<double>> distributions;
        for (const motion_planning::JointBound& bound : scene.jointBounds())
        {
            const double margin = 0.04 * (bound.upper - bound.lower);
            distributions.emplace_back(bound.lower + margin, bound.upper - margin);
        }

        std::vector<Candidate> negativeSide;
        std::vector<Candidate> positiveSide;
        for (std::size_t sampleIndex = 0; sampleIndex < 6000; ++sampleIndex)
        {
            Candidate candidate;
            candidate.joints.resize(distributions.size());
            for (std::size_t jointIndex = 0; jointIndex < distributions.size(); ++jointIndex)
                candidate.joints[jointIndex] = distributions[jointIndex](generator);

            if (!scene.validateState(candidate.joints).valid)
                continue;
            candidate.toolPosition = scene.attachmentWorldTransform("420_tool_attachment").translation();

            const Eigen::Vector3d relative = candidate.toolPosition - center;
            if (std::abs(relative.x()) > 1.25 || candidate.toolPosition.z() < 0.05 || candidate.toolPosition.z() > 1.9)
                continue;

            std::vector<Candidate>* ownSide = nullptr;
            std::vector<Candidate>* oppositeSide = nullptr;
            if (relative.y() < -0.18)
            {
                ownSide = &negativeSide;
                oppositeSide = &positiveSide;
            }
            else if (relative.y() > 0.18)
            {
                ownSide = &positiveSide;
                oppositeSide = &negativeSide;
            }
            else
            {
                continue;
            }

            for (const Candidate& opposite : *oppositeSide)
            {
                const std::vector<double>& first = relative.y() < 0.0 ? candidate.joints : opposite.joints;
                const std::vector<double>& second = relative.y() < 0.0 ? opposite.joints : candidate.joints;
                if (scene.validateMotion(first, second, request.validation).valid)
                    continue;

                request.start = first;
                request.goal = second;
                request.planner.timeoutSeconds = 3.0;
                const motion_planning::ProjectMotionPlanningService service;
                const motion_planning::MotionPlanningResult planning = service.plan(scene, request);
                if (planning.succeeded())
                {
                    start = first;
                    goal = second;
                    return true;
                }
            }

            if (ownSide->size() < 80)
                ownSide->push_back(std::move(candidate));
        }
        return false;
    }

    bool nearlyEqual(const std::vector<double>& first, const std::vector<double>& second)
    {
        if (first.size() != second.size())
            return false;
        for (std::size_t index = 0; index < first.size(); ++index)
        {
            if (std::abs(first[index] - second[index]) > 1.0e-8)
                return false;
        }
        return true;
    }

    bool sameTrajectory(
        const robottrajectory::JointTrajectory& first,
        const robottrajectory::JointTrajectory& second)
    {
        if (first.name != second.name ||
            first.interpolation != second.interpolation ||
            first.points.size() != second.points.size())
            return false;
        for (std::size_t index = 0; index < first.points.size(); ++index)
        {
            if (std::abs(first.points[index].time - second.points[index].time) > 1.0e-12 ||
                !nearlyEqual(first.points[index].q, second.points[index].q))
                return false;
        }
        return true;
    }

    int fail(const std::string& message)
    {
        std::cerr << "FAILED: " << message << "\n";
        return 1;
    }
}

int main(int argc, char** argv)
{
    Options options;
    if (!parseOptions(argc, argv, options))
        return 2;

    simulation_project::ProjectDocument document;
    std::string errorMessage;
    if (!simulation_project::loadProjectDocument(options.projectPath, document, &errorMessage))
        return fail("Project load failed: " + errorMessage);

    motion_planning::ProjectPlanningRequest request;
    request.robotId = "Red4600";
    request.collisionDetectorIds = { "default_collision" };
    request.planner.randomSeed = 4204600u;
    request.planner.timeoutSeconds = 8.0;
    request.validation.maxJointStep = 0.04;
    request.postProcess.duration = 6.0;
    request.postProcess.minimumWaypointCount = 60;

    motion_planning::ProjectPlanningRequest invalidDetectorRequest = request;
    invalidDetectorRequest.collisionDetectorIds = { "missing_detector" };
    std::unique_ptr<motion_planning::ProjectPlanningSceneSnapshot> invalidDetectorScene =
        motion_planning::ProjectPlanningSceneBuilder::build(
            document,
            options.projectPath.parent_path(),
            invalidDetectorRequest,
            &errorMessage);
    if (invalidDetectorScene || errorMessage.find("not found") == std::string::npos)
        return fail("Missing collision detector was not rejected.");

    simulation_project::ProjectDocument disabledDetectorDocument = document;
    const auto disabledDetectorIt = std::find_if(
        disabledDetectorDocument.collision.detectors.begin(),
        disabledDetectorDocument.collision.detectors.end(),
        [](const simulation_project::CollisionDetectorDesc& detector) {
            return detector.id == "default_collision";
        });
    if (disabledDetectorIt == disabledDetectorDocument.collision.detectors.end())
        return fail("Default collision detector is missing from the fixture project.");
    disabledDetectorIt->enabled = false;
    std::unique_ptr<motion_planning::ProjectPlanningSceneSnapshot> disabledDetectorScene =
        motion_planning::ProjectPlanningSceneBuilder::build(
            disabledDetectorDocument,
            options.projectPath.parent_path(),
            request,
            &errorMessage);
    if (disabledDetectorScene || errorMessage.find("disabled") == std::string::npos)
        return fail("Disabled collision detector was not rejected.");

    std::unique_ptr<motion_planning::ProjectPlanningSceneSnapshot> scene =
        motion_planning::ProjectPlanningSceneBuilder::build(
            document,
            options.projectPath.parent_path(),
            request,
            &errorMessage);
    if (!scene)
        return fail("Planning scene build failed: " + errorMessage);

    request.jointNames = scene->jointNames();
    std::vector<double> start = {
        0.659405553148032,
        -0.919209792140321,
        -0.844791768088929,
        -4.8446095729532,
        -1.72245218831655,
        2.56813378610512
    };
    std::vector<double> goal = {
        -0.934876835266256,
        -0.633262433922526,
        1.11553781428474,
        -5.91669352284723,
        1.09495329977677,
        5.42211279330075
    };
    bool fixtureReady = start.size() == scene->jointNames().size() &&
        scene->validateState(start).valid &&
        scene->validateState(goal).valid &&
        !scene->validateMotion(start, goal, request.validation).valid;

    if (options.discover || !fixtureReady)
    {
        if (!findFixture(*scene, request, start, goal))
            return fail("Could not discover a stable turntable-left/right fixture.");
        std::cout << "DISCOVERED_START=" << vectorText(start) << "\n";
        std::cout << "DISCOVERED_GOAL=" << vectorText(goal) << "\n";
    }

    request.start = start;
    request.goal = goal;
    if (scene->validateState(start).valid == false)
        return fail("Start state is invalid.");
    const Eigen::Vector3d startTool = scene->attachmentWorldTransform("420_tool_attachment").translation();
    if (scene->validateState(goal).valid == false)
        return fail("Goal state is invalid.");
    const Eigen::Vector3d goalTool = scene->attachmentWorldTransform("420_tool_attachment").translation();
    if (startTool.y() >= goalTool.y())
        return fail("Fixture does not place the tool on ordered opposite sides of the turntable.");
    if (scene->validateMotion(start, goal, request.validation).valid)
        return fail("Direct joint interpolation is collision-free; the fixture does not prove obstacle avoidance.");

    std::vector<double> collidingState;
    for (std::size_t step = 1; step < 100 && collidingState.empty(); ++step)
    {
        const double ratio = static_cast<double>(step) / 100.0;
        std::vector<double> sample(start.size(), 0.0);
        for (std::size_t index = 0; index < sample.size(); ++index)
            sample[index] = start[index] + (goal[index] - start[index]) * ratio;
        if (!scene->validateState(sample).valid)
            collidingState = std::move(sample);
    }
    if (collidingState.empty())
        return fail("Could not locate the collision state reported by direct interpolation.");

    const motion_planning::ProjectMotionPlanningService service;

    motion_planning::ProjectPlanningRequest invalidStartRequest = request;
    invalidStartRequest.start = collidingState;
    invalidStartRequest.goal = goal;
    const motion_planning::MotionPlanningResult invalidStartResult = service.plan(*scene, invalidStartRequest);
    if (invalidStartResult.status != motion_planning::MotionPlanningStatus::InvalidRequest)
        return fail("Colliding start state was not rejected.");

    motion_planning::CancellationToken cancellation;
    cancellation.cancel();
    const motion_planning::MotionPlanningResult cancelledResult = service.plan(*scene, request, &cancellation);
    if (cancelledResult.status != motion_planning::MotionPlanningStatus::Cancelled)
        return fail("Pre-cancelled planning request did not return Cancelled.");

    const motion_planning::MotionPlanningResult result = service.plan(*scene, request);
    if (!result.succeeded())
    {
        const std::string detail = result.diagnostics.empty() ? "unknown planning error" : result.diagnostics.front().message;
        return fail("OMPL planning failed: " + detail);
    }
    if (result.trajectory.points.size() < 2)
        return fail("Planner returned an empty trajectory.");

    const motion_planning::MotionPlanningResult repeatedResult = service.plan(*scene, request);
    if (!repeatedResult.succeeded() || !sameTrajectory(result.trajectory, repeatedResult.trajectory))
        return fail("Fixed-seed planning did not reproduce the same trajectory.");

    for (std::size_t index = 1; index < result.trajectory.points.size(); ++index)
    {
        if (!scene->validateMotion(
                result.trajectory.points[index - 1].q,
                result.trajectory.points[index].q,
                request.validation).valid)
            return fail("Final trajectory segment revalidation failed.");
    }

    simulation_project::ProjectDocument roundTripDocument = document;
    motion_planning::StoredMotionPlan storedPlan;
    storedPlan.id = "ompl_headless_regression";
    storedPlan.name = "OMPL headless regression";
    storedPlan.robotId = request.robotId;
    storedPlan.jointNames = request.jointNames;
    storedPlan.trajectory = result.trajectory;
    if (!motion_planning::MotionPlanningProjectStore::upsertPlan(roundTripDocument, storedPlan, &errorMessage))
        return fail("Trajectory store failed: " + errorMessage);

    const std::filesystem::path roundTripPath = std::filesystem::current_path() /
        "ompl_motion_planning_roundtrip.sys.json";
    if (!simulation_project::saveProjectDocumentV3(roundTripPath, roundTripDocument, &errorMessage))
        return fail("Round-trip project save failed: " + errorMessage);
    simulation_project::ProjectDocument reloaded;
    if (!simulation_project::loadProjectDocument(roundTripPath, reloaded, &errorMessage))
        return fail("Round-trip project reload failed: " + errorMessage);
    std::error_code removeError;
    std::filesystem::remove(roundTripPath, removeError);

    const std::vector<motion_planning::StoredMotionPlan> plans =
        motion_planning::MotionPlanningProjectStore::plans(reloaded);
    const auto storedIt = std::find_if(plans.begin(), plans.end(), [](const motion_planning::StoredMotionPlan& plan) {
        return plan.id == "ompl_headless_regression";
    });
    if (storedIt == plans.end() || storedIt->trajectory.points.size() != result.trajectory.points.size())
        return fail("Stored trajectory did not survive project round-trip.");

    robotruntime::RobotTrajectoryExecutionSession execution;
    if (!execution.load(request.robotId, storedIt->id, storedIt->trajectory).success)
        return fail("Trajectory execution load failed.");
    if (!execution.start().success)
        return fail("Trajectory execution start failed.");
    while (execution.snapshot().state != robotruntime::RobotRunExecutionState::Completed)
    {
        if (!execution.step(0.05).success)
            return fail("Trajectory execution step failed.");
        if (!scene->setState(execution.snapshot().jointValues, &errorMessage))
            return fail("Runtime trajectory application failed: " + errorMessage);
    }
    if (!nearlyEqual(execution.snapshot().jointValues, goal))
        return fail("Trajectory execution did not finish at the goal state.");

    std::cout << "PASS ProjectMotionPlanning headless regression\n";
    std::cout << "  joints=" << request.jointNames.size()
              << " waypoints=" << result.trajectory.points.size()
              << " sampledStates=" << result.sampledStateCount
              << " planningSeconds=" << result.planningTimeSeconds << "\n";
    std::cout << "  startTool=" << startTool.transpose() << "\n";
    std::cout << "  goalTool=" << goalTool.transpose() << "\n";
    std::cout << "  start=" << vectorText(start) << "\n";
    std::cout << "  goal=" << vectorText(goal) << "\n";
    return 0;
}
