#include "ProjectAssemblyDialogService.h"

#include <QFileDialog>
#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>

namespace robot_qt_viewer {

QString ProjectAssemblyDialogService::selectRobotPackageForImport(
    QWidget* parent,
    const QString& initialPath)
{
    return QFileDialog::getOpenFileName(
        parent,
        QStringLiteral("Import robot package"),
        initialPath,
        QStringLiteral("Robot Package (*.rbt.json);;JSON Files (*.json);;All Files (*.*)"));
}

QString ProjectAssemblyDialogService::selectRobotPackageForExport(
    QWidget* parent,
    const QString& initialPath)
{
    return QFileDialog::getSaveFileName(
        parent,
        QStringLiteral("Export robot package"),
        initialPath,
        QStringLiteral("Robot Package (*.rbt.json);;JSON Files (*.json);;All Files (*.*)"));
}

QString ProjectAssemblyDialogService::selectRobotForImport(QWidget* parent)
{
    return QFileDialog::getOpenFileName(
        parent,
        QStringLiteral("Import robot"),
        QString(),
        QStringLiteral("Robot Files (*.urdf *.xml);;URDF Files (*.urdf);;Simscape Files (*.xml);;All Files (*.*)"));
}

QString ProjectAssemblyDialogService::selectObjectForImport(QWidget* parent)
{
    return QFileDialog::getOpenFileName(
        parent,
        QStringLiteral("Import object"),
        QString(),
        QStringLiteral("Object Meshes (*.stl *.STL *.obj *.OBJ *.dae *.DAE *.ply *.PLY);;STL Files (*.stl *.STL);;All Files (*.*)"));
}

QString ProjectAssemblyDialogService::selectPointCloudForImport(QWidget* parent)
{
    return QFileDialog::getOpenFileName(
        parent,
        QStringLiteral("Import point cloud"),
        QString(),
        QStringLiteral("Point Cloud Files (*.pcd *.PCD);;PCD Files (*.pcd *.PCD);;All Files (*.*)"));
}

bool ProjectAssemblyDialogService::confirmDelete(
    QWidget* parent,
    SceneExplorerNodeKind kind,
    const QString& entityId)
{
    QString title;
    QString message;
    switch(kind) {
    case SceneExplorerNodeKind::Robot:
        title = QStringLiteral("Delete Robot");
        message = QStringLiteral(
            "Delete robot %1?\n\nRelated mount frames, attachments, sensors, tools, and collision references will also be removed.")
                      .arg(entityId);
        break;
    case SceneExplorerNodeKind::Object:
        title = QStringLiteral("Delete Object");
        message = QStringLiteral(
            "Delete object %1?\n\nRelated collision references will also be removed.")
                      .arg(entityId);
        break;
    case SceneExplorerNodeKind::PointCloud:
        title = QStringLiteral("Delete Point Cloud");
        message = QStringLiteral(
            "Delete point cloud %1?\n\nRelated collision references will also be removed.")
                      .arg(entityId);
        break;
    default:
        return false;
    }

    return QMessageBox::question(
               parent,
               title,
               message,
               QMessageBox::Yes | QMessageBox::No,
               QMessageBox::No) == QMessageBox::Yes;
}

QString ProjectAssemblyDialogService::requestPointCloudName(
    QWidget* parent,
    const QString& currentName,
    bool* accepted)
{
    return QInputDialog::getText(
               parent,
               QStringLiteral("Rename Point Cloud"),
               QStringLiteral("Name:"),
               QLineEdit::Normal,
               currentName,
               accepted)
        .trimmed();
}

} // namespace robot_qt_viewer
