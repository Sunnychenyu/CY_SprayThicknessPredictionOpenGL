#pragma once

#include <QString>

class QWidget;

namespace robot_qt_viewer {

class CollisionConfigDialogService
{
public:
    static QString selectOverrideSidecarForSave(QWidget* parent, const QString& initialPath);
    static QString selectUrdfForExport(QWidget* parent, const QString& initialPath);
};

} // namespace robot_qt_viewer
