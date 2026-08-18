#include "RobotQtViewerWorkbenchPackageRegistry.h"

namespace robot_qt_viewer
{
    namespace
    {
        RobotQtViewerWorkbenchPackageDesc makePackage(
            const QString& packageId,
            const QString& displayName)
        {
            RobotQtViewerWorkbenchPackageDesc package;
            package.id = packageId;
            package.displayName = displayName;
            package.version = QStringLiteral("0.1");
            package.enabled = true;
            return package;
        }

        RobotQtViewerWorkbenchModeDesc makeMode(
            const QString& packageId,
            RobotQtViewerWorkbenchKind kind)
        {
            RobotQtViewerWorkbenchModeDesc mode;
            mode.packageId = packageId;
            mode.descriptor = robotQtViewerWorkbenchDescriptor(kind);
            mode.enabled = true;
            return mode;
        }
    }

    bool RobotQtViewerWorkbenchPackageRegistry::registerPackage(
        const RobotQtViewerWorkbenchPackageDesc& package)
    {
        if(package.id.isEmpty() || hasPackage(package.id)) {
            return false;
        }
        m_packages.push_back(package);
        return true;
    }

    bool RobotQtViewerWorkbenchPackageRegistry::registerMode(
        const RobotQtViewerWorkbenchModeDesc& mode)
    {
        if(mode.packageId.isEmpty() || !hasPackage(mode.packageId) ||
            hasMode(mode.descriptor.kind)) {
            return false;
        }
        m_modes.push_back(mode);
        return true;
    }

    bool RobotQtViewerWorkbenchPackageRegistry::hasPackage(const QString& packageId) const
    {
        return package(packageId) != nullptr;
    }

    bool RobotQtViewerWorkbenchPackageRegistry::hasMode(RobotQtViewerWorkbenchKind kind) const
    {
        return mode(kind) != nullptr;
    }

    QString RobotQtViewerWorkbenchPackageRegistry::packageIdForMode(
        RobotQtViewerWorkbenchKind kind) const
    {
        const RobotQtViewerWorkbenchModeDesc* modeDesc = mode(kind);
        return modeDesc == nullptr ? QString() : modeDesc->packageId;
    }

    const RobotQtViewerWorkbenchPackageDesc* RobotQtViewerWorkbenchPackageRegistry::package(
        const QString& packageId) const
    {
        for(const RobotQtViewerWorkbenchPackageDesc& packageDesc : m_packages) {
            if(packageDesc.id == packageId && packageDesc.enabled) {
                return &packageDesc;
            }
        }
        return nullptr;
    }

    const RobotQtViewerWorkbenchModeDesc* RobotQtViewerWorkbenchPackageRegistry::mode(
        RobotQtViewerWorkbenchKind kind) const
    {
        for(const RobotQtViewerWorkbenchModeDesc& modeDesc : m_modes) {
            if(modeDesc.descriptor.kind == kind && modeDesc.enabled &&
                hasPackage(modeDesc.packageId)) {
                return &modeDesc;
            }
        }
        return nullptr;
    }

    const RobotQtViewerWorkbenchDescriptor* RobotQtViewerWorkbenchPackageRegistry::descriptor(
        RobotQtViewerWorkbenchKind kind) const
    {
        const RobotQtViewerWorkbenchModeDesc* modeDesc = mode(kind);
        return modeDesc == nullptr ? nullptr : &modeDesc->descriptor;
    }

    const QVector<RobotQtViewerWorkbenchPackageDesc>&
    RobotQtViewerWorkbenchPackageRegistry::packages() const
    {
        return m_packages;
    }

    const QVector<RobotQtViewerWorkbenchModeDesc>&
    RobotQtViewerWorkbenchPackageRegistry::modes() const
    {
        return m_modes;
    }

    RobotQtViewerWorkbenchPackageRegistry defaultRobotQtViewerWorkbenchPackageRegistry()
    {
        RobotQtViewerWorkbenchPackageRegistry registry;

        const QString projectAssemblyPackageId = QStringLiteral("SMRobotWorkbenchProjectAssembly");
        const QString collisionConfigPackageId = QStringLiteral("SMRobotWorkbenchCollisionConfig");
        const QString robotRunPackageId = QStringLiteral("SMRobotWorkbenchRobotRun");
        const QString motionPlanningPackageId = QStringLiteral("SMRobotWorkbenchMotionPlanning");
        const QString sprayProcessPackageId = QStringLiteral("SMRobotWorkbenchSprayProcess");
        const QString paintingAnalysisPackageId = QStringLiteral("SMRobotWorkbenchPaintingAnalysis");
        const QString digitalTwinPackageId = QStringLiteral("SMRobotWorkbenchDigitalTwin");

        registry.registerPackage(makePackage(projectAssemblyPackageId, QStringLiteral("Project Assembly")));
        registry.registerPackage(makePackage(collisionConfigPackageId, QStringLiteral("Collision Config")));
        registry.registerPackage(makePackage(robotRunPackageId, QStringLiteral("Robot Run")));
        registry.registerPackage(makePackage(motionPlanningPackageId, QStringLiteral("Motion Planning")));
        registry.registerPackage(makePackage(sprayProcessPackageId, QStringLiteral("Spray Process")));
        registry.registerPackage(makePackage(paintingAnalysisPackageId, QStringLiteral("Painting Analysis")));
        registry.registerPackage(makePackage(digitalTwinPackageId, QStringLiteral("Digital Twin")));

        registry.registerMode(makeMode(projectAssemblyPackageId, RobotQtViewerWorkbenchKind::Browse));
        registry.registerMode(makeMode(robotRunPackageId, RobotQtViewerWorkbenchKind::Motion));
        registry.registerMode(makeMode(projectAssemblyPackageId, RobotQtViewerWorkbenchKind::ToolSetup));
        registry.registerMode(makeMode(collisionConfigPackageId, RobotQtViewerWorkbenchKind::Collision));
        registry.registerMode(makeMode(motionPlanningPackageId, RobotQtViewerWorkbenchKind::TrajectoryPlanning));
        registry.registerMode(makeMode(sprayProcessPackageId, RobotQtViewerWorkbenchKind::SprayProcess));
        registry.registerMode(makeMode(paintingAnalysisPackageId, RobotQtViewerWorkbenchKind::CoatingAnalysis));
        registry.registerMode(makeMode(digitalTwinPackageId, RobotQtViewerWorkbenchKind::DigitalTwin));

        return registry;
    }
}
