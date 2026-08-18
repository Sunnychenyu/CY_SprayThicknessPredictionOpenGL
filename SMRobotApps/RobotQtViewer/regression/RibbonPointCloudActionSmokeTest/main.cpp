#include "RobotQtViewerRibbonModel.h"

#include <RobotQtViewerDocumentContext.h>
#include <RobotQtViewerDocumentController.h>
#include <RobotQtViewerEventHub.h>
#include <RobotQtViewerSelectionModel.h>
#include <RobotQtViewerViewportPreviewState.h>
#include <SceneEntityWorkflowController.h>

#include <QCoreApplication>
#include <QFile>
#include <QStringList>

#include <SimulationProject/ProjectSession.h>

#include <filesystem>
#include <iostream>

int main(int argc, char* argv[])
{
    QCoreApplication application(argc, argv);

    const robot_qt_viewer::RobotQtViewerRibbonModel model =
        robot_qt_viewer::makeDefaultRobotQtViewerRibbonModel();

    QStringList sceneEditActionIds;
    for(const auto& page : model.pages) {
        for(const auto& group : page.groups) {
            if(group.groupId != QStringLiteral("sceneEdit"))
                continue;

            for(const auto& action : group.actions)
                sceneEditActionIds.push_back(action.actionId);
        }
    }

    const QStringList expectedActionIds = {
        QStringLiteral("importRobot"),
        QStringLiteral("importObject"),
        QStringLiteral("importPointCloud"),
        QStringLiteral("deleteSelectedItem")
    };
    if(sceneEditActionIds != expectedActionIds) {
        std::cerr << "Unexpected Scene Edit ribbon actions: "
                  << sceneEditActionIds.join(',').toStdString() << '\n';
        return 1;
    }

    QFile icon(QStringLiteral(":/RobotQtViewer/icons/ribbon/import_point_cloud.png"));
    if(!icon.open(QIODevice::ReadOnly)) {
        std::cerr << "Import point cloud icon resource is unavailable.\n";
        return 1;
    }

    const QByteArray signature = icon.read(8);
    const QByteArray pngSignature("\x89PNG\r\n\x1a\n", 8);
    if(signature != pngSignature) {
        std::cerr << "Import point cloud icon resource is not a PNG.\n";
        return 1;
    }

    simulation_project::ProjectSession session;
    robot_qt_viewer::RobotQtViewerEventHub eventHub;
    robot_qt_viewer::RobotQtViewerDocumentController documentController(session, eventHub);
    robot_qt_viewer::RobotQtViewerSelectionModel selectionModel(eventHub);
    robot_qt_viewer::RobotQtViewerViewportPreviewState viewportPreviewState(eventHub);
    robot_qt_viewer::RobotQtViewerDocumentContext context(
        session,
        documentController,
        selectionModel,
        viewportPreviewState,
        eventHub);
    robot_qt_viewer::SceneEntityWorkflowController workflow(context);

    const std::filesystem::path pointCloudPath =
        std::filesystem::u8path(SMROBOT_TEST_SOURCE_ROOT) / "data/pcl/bunny.pcd";
    const robot_qt_viewer::SceneEntityImportResult importResult =
        workflow.importPointCloudFromPath(pointCloudPath, "pcd");
    if(!importResult.success ||
       session.document().pointClouds.size() != 1 ||
       !session.isDirty()) {
        std::cerr << "Point cloud workflow import failed: "
                  << importResult.message.toStdString() << '\n';
        return 1;
    }

    const simulation_project::PointCloudDesc& imported = session.document().pointClouds.front();
    if(imported.id.empty() || imported.sourcePath.empty() || imported.format != "pcd") {
        std::cerr << "Imported point cloud descriptor is incomplete.\n";
        return 1;
    }

    workflow.restoreImportState(importResult);
    if(!session.document().pointClouds.empty() || session.isDirty()) {
        std::cerr << "Point cloud import snapshot restore failed.\n";
        return 1;
    }

    std::cout << "RobotQtViewer point cloud ribbon action smoke passed.\n";
    return 0;
}
