#include "RobotQtViewerRibbonModel.h"

namespace robot_qt_viewer
{
    RobotQtViewerRibbonModel makeDefaultRobotQtViewerRibbonModel(const RobotQtViewerRibbonTexts& texts)
    {
        RobotQtViewerRibbonModel model;
        model.toolbarTitle = texts.toolbarTitle;

        RobotQtViewerRibbonPageSpec homePage;
        homePage.pageId = QStringLiteral("home");
        homePage.title = texts.homePage;

        RobotQtViewerRibbonGroupSpec projectGroup;
        projectGroup.groupId = QStringLiteral("project");
        projectGroup.title = texts.projectGroup;
        projectGroup.actions = {
            { QStringLiteral("newProject"), QStringLiteral("document-new"), QStyle::SP_FileIcon },
            { QStringLiteral("openProject"), QStringLiteral("document-open"), QStyle::SP_DialogOpenButton },
            { QStringLiteral("saveProject"), QStringLiteral("document-save"), QStyle::SP_DialogSaveButton },
            { QStringLiteral("saveProjectAs"), QStringLiteral("document-save-as"), QStyle::SP_DialogSaveButton }
        };

        RobotQtViewerRibbonGroupSpec sceneEditGroup;
        sceneEditGroup.groupId = QStringLiteral("sceneEdit");
        sceneEditGroup.title = texts.sceneGroup;
        sceneEditGroup.actions = {
            { QStringLiteral("importRobot"), QStringLiteral("list-add"), QStyle::SP_ComputerIcon },
            { QStringLiteral("importObject"), QStringLiteral("insert-object"), QStyle::SP_DirIcon },
            { QStringLiteral("deleteSelectedItem"), QStringLiteral("edit-delete"), QStyle::SP_TrashIcon }
        };

        RobotQtViewerRibbonGroupSpec robotEditGroup;
        robotEditGroup.groupId = QStringLiteral("robotEdit");
        robotEditGroup.title = texts.robotEditGroup;
        robotEditGroup.actions = {
            { QStringLiteral("toolSetupWorkbench"), QStringLiteral("preferences-system"), QStyle::SP_DialogApplyButton }
        };

        RobotQtViewerRibbonGroupSpec viewGroup;
        viewGroup.groupId = QStringLiteral("view");
        viewGroup.title = texts.viewGroup;
        viewGroup.actions = {
            { QStringLiteral("saveImage"), QStringLiteral("image-x-generic"), QStyle::SP_FileIcon },
            { QStringLiteral("resetCamera"), QStringLiteral("view-refresh"), QStyle::SP_BrowserReload }
        };

        RobotQtViewerRibbonGroupSpec modeGroup;
        modeGroup.groupId = QStringLiteral("modes");
        modeGroup.title = texts.workbenchGroup;
        modeGroup.actions = {
            { QStringLiteral("projectAssemblyWorkbench"), QStringLiteral("view-list-details"), QStyle::SP_FileDialogDetailedView },
            { QStringLiteral("collisionConfigWorkbench"), QStringLiteral("dialog-warning"), QStyle::SP_MessageBoxWarning },
            { QStringLiteral("robotRunWorkbench"), QStringLiteral("media-playback-start"), QStyle::SP_MediaPlay },
            { QStringLiteral("motionPlanningWorkbench"), QStringLiteral("go-next"), QStyle::SP_ArrowRight },
            { QStringLiteral("sprayProcessWorkbench"), QStringLiteral("format-fill-color"), QStyle::SP_DialogApplyButton },
            { QStringLiteral("coatingAnalysisWorkbench"), QStringLiteral("view-statistics"), QStyle::SP_FileDialogInfoView },
            { QStringLiteral("digitalTwinWorkbench"), QStringLiteral("network-connect"), QStyle::SP_ComputerIcon }
        };

        homePage.groups = { projectGroup, modeGroup, sceneEditGroup, robotEditGroup, viewGroup };
        model.pages = { homePage };
        return model;
    }
}
