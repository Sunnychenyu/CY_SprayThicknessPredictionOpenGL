#pragma once

#include <RobotCore/RobotModel.h>
#include <SimulationProject/ProjectDocument.h>

#include <filesystem>

class RobotCollisionOverrideApplier
{
public:
    static void apply(
        const simulation_project::ProjectDocument& document,
        const simulation_project::RobotDesc& robotDesc,
        const std::filesystem::path& projectBasePath,
        robot::RobotModel& model);
};
