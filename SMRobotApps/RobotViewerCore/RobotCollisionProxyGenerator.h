#pragma once

#include "ProjectRuntimeTypes.h"

#include <Collision/CollisionCoacdAssetWriter.h>

#include <SimulationProject/ProjectDocument.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

struct RobotCollisionProxyRequest
{
    std::string proxyType;
    std::string role = "PlanningProxy";
    double inflationMargin = 0.0;
    int maxSphereCount = 16;
    bool useVisualWhenCollisionMissing = true;
    bool useExistingCollisionAsInput = false;
};

struct RobotCollisionProxyQualitySummary
{
    int requestedMaxSpheres = 0;
    int generatedSphereCount = 0;
    std::size_t inputPointCount = 0;
    std::size_t uncoveredPointCount = 0;
    double maxOutsideDistance = 0.0;
    double mainAxisLength = 0.0;
    double estimatedCrossSectionRadius = 0.0;
    double maxSphereRadius = 0.0;
    double maxRadiusToLinkLength = 0.0;
    double maxRadiusToCrossSectionRadius = 0.0;
    int oversizedSphereCount = 0;
    std::string recommendedShape;
    bool hasWarning = false;
    std::string warning;
};

struct CollisionGeneratedAssetRequest
{
    std::filesystem::path generatedAssetRoot;
    std::string sourceKey;
    std::string targetKey;
    std::string role = "CoACD";
    collision::CollisionCoacdOptions options = collision::CollisionCoacdOptions::fastPreviewDefaults();
};

class RobotCollisionProxyGenerator
{
public:
    static bool generateFromVisual(
        const RuntimeRobot& robot,
        const std::string& linkName,
        const std::string& proxyType,
        simulation_project::CollisionElementOverrideDesc& element);

    static bool generateFromVisual(
        const RuntimeRobot& robot,
        const std::string& linkName,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements);

    static bool generateFromExistingCollision(
        const RuntimeRobot& robot,
        const std::string& linkName,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements);

    static bool generateCoacdFromVisual(
        const RuntimeRobot& robot,
        const std::string& linkName,
        const CollisionGeneratedAssetRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements);

    static bool generateCoacdFromExistingCollision(
        const RuntimeRobot& robot,
        const std::string& linkName,
        const CollisionGeneratedAssetRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements);

    static bool generateObjectCoacdFromVisual(
        const RuntimeSceneObject& object,
        const CollisionGeneratedAssetRequest& request,
        std::vector<simulation_project::ObjectCollisionElementOverrideDesc>& elements);

    static bool generateMissingFromVisual(
        const RuntimeRobot& robot,
        const std::string& proxyType,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements);

    static bool generateMissingFromVisual(
        const RuntimeRobot& robot,
        const RobotCollisionProxyRequest& request,
        std::vector<simulation_project::CollisionElementOverrideDesc>& elements);

    static bool evaluateGeneratedSphereCover(
        const RuntimeRobot& robot,
        const std::string& linkName,
        const RobotCollisionProxyRequest& request,
        const std::vector<simulation_project::CollisionElementOverrideDesc>& elements,
        bool useExistingCollisionInput,
        RobotCollisionProxyQualitySummary& summary);
};
