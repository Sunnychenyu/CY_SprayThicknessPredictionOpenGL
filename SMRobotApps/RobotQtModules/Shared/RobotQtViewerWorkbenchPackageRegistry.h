#pragma once

#include "RobotQtViewerWorkbench.h"

#include <QString>
#include <QVector>

namespace robot_qt_viewer
{
    struct RobotQtViewerWorkbenchPackageDesc
    {
        QString id;
        QString displayName;
        QString version;
        bool enabled = true;
    };

    struct RobotQtViewerWorkbenchModeDesc
    {
        QString packageId;
        RobotQtViewerWorkbenchDescriptor descriptor;
        bool enabled = true;
    };

    class RobotQtViewerWorkbenchPackageRegistry
    {
    public:
        bool registerPackage(const RobotQtViewerWorkbenchPackageDesc& package);
        bool registerMode(const RobotQtViewerWorkbenchModeDesc& mode);

        bool hasPackage(const QString& packageId) const;
        bool hasMode(RobotQtViewerWorkbenchKind kind) const;
        QString packageIdForMode(RobotQtViewerWorkbenchKind kind) const;

        const RobotQtViewerWorkbenchPackageDesc* package(const QString& packageId) const;
        const RobotQtViewerWorkbenchModeDesc* mode(RobotQtViewerWorkbenchKind kind) const;
        const RobotQtViewerWorkbenchDescriptor* descriptor(RobotQtViewerWorkbenchKind kind) const;

        const QVector<RobotQtViewerWorkbenchPackageDesc>& packages() const;
        const QVector<RobotQtViewerWorkbenchModeDesc>& modes() const;

    private:
        QVector<RobotQtViewerWorkbenchPackageDesc> m_packages;
        QVector<RobotQtViewerWorkbenchModeDesc> m_modes;
    };

    RobotQtViewerWorkbenchPackageRegistry defaultRobotQtViewerWorkbenchPackageRegistry();
}
