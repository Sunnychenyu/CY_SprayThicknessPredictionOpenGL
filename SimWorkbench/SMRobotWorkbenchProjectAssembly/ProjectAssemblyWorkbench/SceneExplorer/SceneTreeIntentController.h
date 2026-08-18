#pragma once

#include "SceneExplorerViewModel.h"

#include <SimulationProject/ProjectDocument.h>

#include <QVector>
#include <QStringList>

class QPoint;
class QTreeWidget;
class QTreeWidgetItem;

namespace simulation_project
{
    struct CollisionSelectionSetMemberDesc;
}

namespace robot_qt_viewer
{
    class SceneTreeIntentController
    {
    public:
        enum class ContextMenuActionKind
        {
            ConfigureRobotFlange,
            MoveRobotBase,
            DeleteRobot,
            MoveSceneObject,
            AddObjectFrame,
            EditObjectFrame,
            BindItemToMount,
            BindObjectToMount,
            UnbindMountedAttachment,
            RenamePointCloud,
            CreateToolAssetFromObject,
            DeleteObject,
            DeletePointCloud,
            ShowLinkFrame,
            ConfigureCollisionModel,
            AddToDetectorSetA,
            AddToDetectorSetB
        };

        struct ContextMenuAction
        {
            ContextMenuActionKind kind = ContextMenuActionKind::MoveRobotBase;
            SceneExplorerNodeRef node;
            QString displayName;
            QStringList robotLinks;
            simulation_project::CollisionSelectionSetMemberDesc collisionMember;
            bool hasCollisionMember = false;
        };

        struct ContextMenuActionView
        {
            QString label;
            bool enabled = true;
            bool checkable = false;
            bool checked = false;
            bool separatorBefore = false;
            ContextMenuAction action;
        };

        struct ContextMenuModel
        {
            SceneExplorerNodeRef node;
            QVector<ContextMenuActionView> actions;
        };

        static SceneExplorerNodeRef nodeRefFromItem(const QTreeWidgetItem* item);
        static QTreeWidgetItem* treeItemAtOrCurrent(QTreeWidget* tree, const QPoint& pos);
        static QTreeWidgetItem* actionableTreeItem(QTreeWidgetItem* item);
        static QStringList robotLinksFromItem(QTreeWidgetItem* item);
        static QString displayNameFromItem(const QTreeWidgetItem* item);
        static ContextMenuModel contextMenuModelFromItem(
            QTreeWidgetItem* item,
            bool hasCollisionSelectionSets);
        static ContextMenuModel contextMenuModelFromItem(
            QTreeWidgetItem* item,
            bool hasCollisionSelectionSets,
            SceneExplorerActionScope actionScope);
        static bool collisionSelectionSetMemberFromItem(
            const QTreeWidgetItem* item,
            simulation_project::CollisionSelectionSetMemberDesc& member,
            QString* errorMessage = nullptr);

    private:
        static bool isContextMenuTarget(const SceneExplorerNodeRef& node);
        static bool isEntityActionable(const SceneExplorerNodeRef& node);
    };
}
