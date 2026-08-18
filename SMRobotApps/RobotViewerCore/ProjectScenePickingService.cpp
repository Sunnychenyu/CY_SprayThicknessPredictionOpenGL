#include "ProjectScenePickingService.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    bool raySphereDistance(
        const ProjectScenePickRay& ray,
        const Eigen::Vector3d& center,
        double radius,
        double& rayDistance)
    {
        if(radius <= 0.0) {
            return false;
        }

        const Eigen::Vector3d offset = center - ray.origin;
        const double t = offset.dot(ray.direction);
        if(t < 0.0) {
            return false;
        }

        const Eigen::Vector3d closest = ray.origin + ray.direction * t;
        const double distance = (center - closest).norm();
        if(distance > radius) {
            return false;
        }

        rayDistance = t;
        return true;
    }

    bool rayAabbDistance(
        const ProjectScenePickRay& ray,
        const Eigen::Vector3d& aabbMin,
        const Eigen::Vector3d& aabbMax,
        double& rayDistance)
    {
        double tMin = 0.0;
        double tMax = std::numeric_limits<double>::max();

        for(int axis = 0; axis < 3; ++axis) {
            const double origin = ray.origin[axis];
            const double direction = ray.direction[axis];
            const double minValue = std::min(aabbMin[axis], aabbMax[axis]);
            const double maxValue = std::max(aabbMin[axis], aabbMax[axis]);

            if(std::abs(direction) < 1.0e-9) {
                if(origin < minValue || origin > maxValue) {
                    return false;
                }
                continue;
            }

            double nearT = (minValue - origin) / direction;
            double farT = (maxValue - origin) / direction;
            if(nearT > farT) {
                std::swap(nearT, farT);
            }

            tMin = std::max(tMin, nearT);
            tMax = std::min(tMax, farT);
            if(tMin > tMax) {
                return false;
            }
        }

        if(tMax < 0.0) {
            return false;
        }

        rayDistance = tMin >= 0.0 ? tMin : tMax;
        return true;
    }

    int targetPriority(ProjectScenePickTargetKind kind, ProjectSceneInteractionMode mode)
    {
        if(mode == ProjectSceneInteractionMode::SelectMount) {
            if(kind == ProjectScenePickTargetKind::RobotMount) {
                return 0;
            }
            if(kind == ProjectScenePickTargetKind::MountedAttachment) {
                return 1;
            }
        }
        if(mode == ProjectSceneInteractionMode::SelectAttachment) {
            if(kind == ProjectScenePickTargetKind::MountedAttachment) {
                return 0;
            }
            if(kind == ProjectScenePickTargetKind::RobotMount) {
                return 1;
            }
        }
        if(mode == ProjectSceneInteractionMode::SelectCollisionTarget) {
            if(kind == ProjectScenePickTargetKind::SceneObject) {
                return 0;
            }
            if(kind == ProjectScenePickTargetKind::PointCloud) {
                return 1;
            }
            if(kind == ProjectScenePickTargetKind::MountedAttachment) {
                return 2;
            }
            if(kind == ProjectScenePickTargetKind::RobotLink) {
                return 3;
            }
        }

        switch(kind) {
        case ProjectScenePickTargetKind::MountedAttachment:
            return 0;
        case ProjectScenePickTargetKind::RobotMount:
            return 1;
        case ProjectScenePickTargetKind::RobotLink:
            return 2;
        case ProjectScenePickTargetKind::SceneObject:
            return 3;
        case ProjectScenePickTargetKind::PointCloud:
            return 3;
        case ProjectScenePickTargetKind::Robot:
            return 4;
        case ProjectScenePickTargetKind::None:
            return 100;
        }
        return 100;
    }

    ProjectScenePickResult makeResult(
        const ProjectScenePickCandidate& candidate,
        double rayDistance)
    {
        ProjectScenePickResult result;
        result.kind = candidate.kind;
        result.robotId = candidate.robotId;
        result.linkName = candidate.linkName;
        result.robotMountId = candidate.robotMountId;
        result.mountedAttachmentId = candidate.mountedAttachmentId;
        result.sceneObjectId = candidate.sceneObjectId;
        result.rayDistance = rayDistance;
        return result;
    }
}

ProjectScenePickResult ProjectScenePickingService::pick(
    const ProjectScenePickRay& ray,
    const std::vector<ProjectScenePickCandidate>& candidates,
    ProjectSceneInteractionMode mode)
{
    ProjectScenePickResult best;
    double bestDistance = std::numeric_limits<double>::max();
    int bestPriority = 100;

    for(const ProjectScenePickCandidate& candidate : candidates) {
        if(!acceptsCandidate(candidate.kind, mode)) {
            continue;
        }

        bool hit = false;
        double rayDistance = 0.0;
        if(candidate.hasAabb) {
            hit = rayAabbDistance(ray, candidate.aabbMin, candidate.aabbMax, rayDistance);
        }
        if(!hit) {
            hit = raySphereDistance(ray, candidate.center, candidate.radius, rayDistance);
        }
        if(!hit) {
            continue;
        }

        const int priority = targetPriority(candidate.kind, mode);
        if(priority < bestPriority ||
            (priority == bestPriority && rayDistance < bestDistance)) {
            best = makeResult(candidate, rayDistance);
            bestDistance = rayDistance;
            bestPriority = priority;
        }
    }

    return best;
}

bool ProjectScenePickingService::acceptsCandidate(
    ProjectScenePickTargetKind kind,
    ProjectSceneInteractionMode mode)
{
    switch(mode) {
    case ProjectSceneInteractionMode::Browse:
        return kind == ProjectScenePickTargetKind::RobotLink ||
            kind == ProjectScenePickTargetKind::RobotMount ||
            kind == ProjectScenePickTargetKind::MountedAttachment ||
            kind == ProjectScenePickTargetKind::SceneObject ||
            kind == ProjectScenePickTargetKind::PointCloud;
    case ProjectSceneInteractionMode::SelectRobot:
        return kind == ProjectScenePickTargetKind::Robot ||
            kind == ProjectScenePickTargetKind::RobotLink;
    case ProjectSceneInteractionMode::SelectLink:
        return kind == ProjectScenePickTargetKind::RobotLink;
    case ProjectSceneInteractionMode::SelectMount:
        return kind == ProjectScenePickTargetKind::RobotMount ||
            kind == ProjectScenePickTargetKind::MountedAttachment ||
            kind == ProjectScenePickTargetKind::RobotLink;
    case ProjectSceneInteractionMode::SelectAttachment:
        return kind == ProjectScenePickTargetKind::MountedAttachment ||
            kind == ProjectScenePickTargetKind::RobotMount;
    case ProjectSceneInteractionMode::SelectCollisionTarget:
        return kind == ProjectScenePickTargetKind::RobotLink ||
            kind == ProjectScenePickTargetKind::MountedAttachment ||
            kind == ProjectScenePickTargetKind::SceneObject ||
            kind == ProjectScenePickTargetKind::PointCloud;
    case ProjectSceneInteractionMode::EditTransformPreview:
    case ProjectSceneInteractionMode::EditCollisionProxy:
        return false;
    }
    return false;
}
