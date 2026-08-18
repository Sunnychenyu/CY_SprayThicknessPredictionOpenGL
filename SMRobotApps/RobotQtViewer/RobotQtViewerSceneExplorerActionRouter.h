#pragma once

#include "SceneTreeIntentController.h"

#include <QString>

#include <functional>

class QWidget;

namespace robot_qt_viewer
{
    class CollisionWorkbenchModuleController;
    class SceneExplorerModuleController;
    class ToolSetupModuleController;

    class RobotQtViewerSceneExplorerActionRouter
    {
    public:
        using StatusCallback = std::function<void(const QString&, int)>;
        using VoidCallback = std::function<void()>;
        using RobotLinkCallback = std::function<void(const QString&, const QString&)>;
        using ObjectCallback = std::function<void(const QString&)>;

        RobotQtViewerSceneExplorerActionRouter() = default;

        void setParentWidget(QWidget* parentWidget);
        void setSceneExplorerController(SceneExplorerModuleController* controller);
        void setToolSetupController(ToolSetupModuleController* controller);
        void setCollisionWorkbenchController(CollisionWorkbenchModuleController* controller);
        void setEnterToolSetupWorkbenchCallback(VoidCallback callback);
        void setDeleteSelectedEntityCallback(VoidCallback callback);
        void setReloadViewportCallback(VoidCallback callback);
        void setSelectRobotContextCallback(RobotLinkCallback callback);
        void setSelectObjectContextCallback(ObjectCallback callback);

        void handleAction(
            const SceneTreeIntentController::ContextMenuAction& action,
            StatusCallback statusCallback);

    private:
        QWidget* m_parentWidget = nullptr;
        SceneExplorerModuleController* m_sceneExplorerController = nullptr;
        ToolSetupModuleController* m_toolSetupController = nullptr;
        CollisionWorkbenchModuleController* m_collisionWorkbenchController = nullptr;
        VoidCallback m_enterToolSetupWorkbenchCallback;
        VoidCallback m_deleteSelectedEntityCallback;
        VoidCallback m_reloadViewportCallback;
        RobotLinkCallback m_selectRobotContextCallback;
        ObjectCallback m_selectObjectContextCallback;
    };
}
