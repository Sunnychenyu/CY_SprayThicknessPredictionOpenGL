#pragma once

#include "SceneExplorerViewModel.h"

#include <QString>

class QWidget;

namespace robot_qt_viewer {

class ProjectAssemblyDialogService
{
public:
    static QString selectRobotPackageForImport(QWidget* parent, const QString& initialPath);
    static QString selectRobotPackageForExport(QWidget* parent, const QString& initialPath);
    static QString selectRobotForImport(QWidget* parent);
    static QString selectObjectForImport(QWidget* parent);
    static QString selectPointCloudForImport(QWidget* parent);

    static bool confirmDelete(
        QWidget* parent,
        SceneExplorerNodeKind kind,
        const QString& entityId);

    static QString requestPointCloudName(
        QWidget* parent,
        const QString& currentName,
        bool* accepted);
};

} // namespace robot_qt_viewer
