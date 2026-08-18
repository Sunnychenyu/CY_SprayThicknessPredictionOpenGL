#pragma once

#include "ProjectScene.h"

#include <Eigen/Core>

#include <string>
#include <vector>

enum class ProjectScenePickTargetKind
{
    None,
    Robot,
    RobotLink,
    RobotMount,
    MountedAttachment,
    SceneObject,
    PointCloud
};

struct ProjectScenePickRay
{
    Eigen::Vector3d origin = Eigen::Vector3d::Zero();
    Eigen::Vector3d direction = Eigen::Vector3d::UnitZ();
};

struct ProjectScenePickCandidate
{
    ProjectScenePickTargetKind kind = ProjectScenePickTargetKind::None;
    std::string robotId;
    std::string linkName;
    std::string robotMountId;
    std::string mountedAttachmentId;
    std::string sceneObjectId;
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    double radius = 0.0;
    bool hasAabb = false;
    Eigen::Vector3d aabbMin = Eigen::Vector3d::Zero();
    Eigen::Vector3d aabbMax = Eigen::Vector3d::Zero();
};

struct ProjectScenePickResult
{
    ProjectScenePickTargetKind kind = ProjectScenePickTargetKind::None;
    std::string robotId;
    std::string linkName;
    std::string robotMountId;
    std::string mountedAttachmentId;
    std::string sceneObjectId;
    double rayDistance = 0.0;

    bool valid() const
    {
        return kind != ProjectScenePickTargetKind::None;
    }
};

class ProjectScenePickingService
{
public:
    static ProjectScenePickResult pick(
        const ProjectScenePickRay& ray,
        const std::vector<ProjectScenePickCandidate>& candidates,
        ProjectSceneInteractionMode mode);

private:
    static bool acceptsCandidate(ProjectScenePickTargetKind kind, ProjectSceneInteractionMode mode);
};
