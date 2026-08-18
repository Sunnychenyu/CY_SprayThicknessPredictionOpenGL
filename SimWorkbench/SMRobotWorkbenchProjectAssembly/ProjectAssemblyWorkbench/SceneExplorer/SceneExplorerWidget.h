#pragma once

#include "SceneExplorerViewModel.h"

#include <QHash>
#include <QStringList>
#include <QWidget>

class QPoint;
class QEvent;
class QLabel;
class QString;
class QTreeWidget;
class QTreeWidgetItem;

class SceneExplorerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SceneExplorerWidget(QWidget* parent = nullptr);

    QTreeWidget* treeWidget() const;
    robot_qt_viewer::SceneExplorerNodeRef currentNode() const;
    bool selectNode(const robot_qt_viewer::SceneExplorerNodeRef& node);
    void setDocumentView(const robot_qt_viewer::SceneExplorerViewModel& viewModel);
    void setViewModel(const robot_qt_viewer::SceneExplorerViewModel& viewModel);
    void setSummaryText(const QString& text);

signals:
    void nodeActivated(const robot_qt_viewer::SceneExplorerNodeRef& node, int column);
    void nodeDoubleActivated(const robot_qt_viewer::SceneExplorerNodeRef& node, int column);
    void treeContextMenuRequested(const QPoint& pos);

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

private:
    robot_qt_viewer::SceneExplorerNodeRef nodeRefFromItem(const QTreeWidgetItem* item) const;
    QStringList treeStructureSignature(
        const robot_qt_viewer::SceneExplorerViewModel& viewModel) const;
    bool updateExistingTree(const robot_qt_viewer::SceneExplorerViewModel& viewModel);
    void rebuildTree(const robot_qt_viewer::SceneExplorerViewModel& viewModel);
    void applyNodeViewToItem(
        QTreeWidgetItem& item,
        const robot_qt_viewer::SceneExplorerNodeView& node);
    void setCurrentItemForSelectedNode(
        const robot_qt_viewer::SceneExplorerViewModel& viewModel,
        const QHash<QString, QTreeWidgetItem*>& itemByNodeId);

    QTreeWidget* m_robotTree = nullptr;
    QLabel* m_summaryLabel = nullptr;
    QStringList m_treeStructureSignature;
    bool m_ignoreNextClickActivation = false;
};
