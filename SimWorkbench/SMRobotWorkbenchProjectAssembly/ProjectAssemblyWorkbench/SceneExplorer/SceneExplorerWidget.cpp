#include "SceneExplorerWidget.h"

#include "RobotQtWidgetUtils.h"

#include <QAbstractItemView>
#include <QBrush>
#include <QColor>
#include <QEvent>
#include <QFont>
#include <QHash>
#include <QLabel>
#include <QModelIndex>
#include <QMouseEvent>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QTreeWidgetItemIterator>
#include <QVBoxLayout>

SceneExplorerWidget::SceneExplorerWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(12, 10, 12, 10);
    layout->setSpacing(8);

    m_robotTree = new QTreeWidget(this);
    m_robotTree->setHeaderHidden(true);
    m_robotTree->setAlternatingRowColors(true);
    m_robotTree->setAllColumnsShowFocus(true);
    m_robotTree->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_robotTree->setContextMenuPolicy(Qt::CustomContextMenu);
    m_robotTree->viewport()->installEventFilter(this);
    connect(m_robotTree, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem* item, int column) {
        if(m_ignoreNextClickActivation) {
            m_ignoreNextClickActivation = false;
            return;
        }
        emit nodeActivated(nodeRefFromItem(item), column);
    });
    connect(m_robotTree, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem* item, int column) {
        emit nodeDoubleActivated(nodeRefFromItem(item), column);
    });
    connect(m_robotTree, &QTreeWidget::customContextMenuRequested, this, &SceneExplorerWidget::treeContextMenuRequested);
    layout->addWidget(m_robotTree, 1);

    auto* summaryLabel = new QLabel("No robot loaded", this);
    summaryLabel->setWordWrap(true);
    m_summaryLabel = summaryLabel;
    layout->addWidget(summaryLabel);
}

bool SceneExplorerWidget::eventFilter(QObject* watched, QEvent* event)
{
    if(m_robotTree != nullptr &&
        watched == m_robotTree->viewport() &&
        event != nullptr &&
        event->type() == QEvent::MouseButtonDblClick) {
        const auto* mouseEvent = static_cast<QMouseEvent*>(event);
        QTreeWidgetItem* item = m_robotTree->itemAt(mouseEvent->pos());
        const robot_qt_viewer::SceneExplorerNodeRef node = nodeRefFromItem(item);
        if(node.kind == robot_qt_viewer::SceneExplorerNodeKind::RobotMount ||
            node.kind == robot_qt_viewer::SceneExplorerNodeKind::ObjectFrame) {
            const QModelIndex index = m_robotTree->indexAt(mouseEvent->pos());
            m_ignoreNextClickActivation = true;
            emit nodeDoubleActivated(node, index.isValid() ? index.column() : 0);
            return true;
        }
    }
    return QWidget::eventFilter(watched, event);
}

QTreeWidget* SceneExplorerWidget::treeWidget() const
{
    return m_robotTree;
}

robot_qt_viewer::SceneExplorerNodeRef SceneExplorerWidget::currentNode() const
{
    return nodeRefFromItem(m_robotTree != nullptr ? m_robotTree->currentItem() : nullptr);
}

bool SceneExplorerWidget::selectNode(const robot_qt_viewer::SceneExplorerNodeRef& node)
{
    if(m_robotTree == nullptr || node.kind == robot_qt_viewer::SceneExplorerNodeKind::Unknown) {
        return false;
    }

    QTreeWidgetItemIterator it(m_robotTree);
    while(*it != nullptr) {
        QTreeWidgetItem* item = *it;
        const robot_qt_viewer::SceneExplorerNodeRef itemNode = nodeRefFromItem(item);
        const bool sameJointName =
            node.kind != robot_qt_viewer::SceneExplorerNodeKind::Joint ||
            itemNode.name == node.name;
        if(itemNode.kind == node.kind &&
            itemNode.id == node.id &&
            itemNode.linkName == node.linkName &&
            sameJointName) {
            m_robotTree->setCurrentItem(item);
            m_robotTree->scrollToItem(item);
            return true;
        }
        ++it;
    }
    return false;
}

void SceneExplorerWidget::setViewModel(const robot_qt_viewer::SceneExplorerViewModel& viewModel)
{
    setDocumentView(viewModel);
}

void SceneExplorerWidget::setDocumentView(const robot_qt_viewer::SceneExplorerViewModel& viewModel)
{
    if(m_robotTree == nullptr) {
        return;
    }

    const QStringList nextStructureSignature = treeStructureSignature(viewModel);
    const QSignalBlocker treeSignalBlocker(m_robotTree);
    m_robotTree->setUpdatesEnabled(false);
    if(nextStructureSignature == m_treeStructureSignature && updateExistingTree(viewModel)) {
        m_robotTree->setUpdatesEnabled(true);
        if(!viewModel.taskSummary.isEmpty()) {
            setSummaryText(viewModel.taskSummary);
        }
        return;
    }

    rebuildTree(viewModel);
    m_treeStructureSignature = nextStructureSignature;
    m_robotTree->setUpdatesEnabled(true);
    if(!viewModel.taskSummary.isEmpty()) {
        setSummaryText(viewModel.taskSummary);
    }
}

QStringList SceneExplorerWidget::treeStructureSignature(
    const robot_qt_viewer::SceneExplorerViewModel& viewModel) const
{
    QStringList signature;
    signature.reserve(viewModel.nodes.size());
    for(const robot_qt_viewer::SceneExplorerNodeView& node : viewModel.nodes) {
        signature.push_back(QStringLiteral("%1\n%2\n%3")
            .arg(node.nodeId,
                node.parentNodeId,
                robot_qt_viewer::sceneExplorerNodeTypeName(node.ref.kind)));
    }
    return signature;
}

bool SceneExplorerWidget::updateExistingTree(const robot_qt_viewer::SceneExplorerViewModel& viewModel)
{
    QTreeWidgetItemIterator it(m_robotTree);
    QHash<QString, QTreeWidgetItem*> itemByNodeId;
    for(const robot_qt_viewer::SceneExplorerNodeView& node : viewModel.nodes) {
        if(*it == nullptr) {
            return false;
        }
        QTreeWidgetItem* item = *it;
        applyNodeViewToItem(*item, node);
        if(!node.nodeId.isEmpty()) {
            itemByNodeId.insert(node.nodeId, item);
        }
        ++it;
    }
    if(*it != nullptr) {
        return false;
    }

    setCurrentItemForSelectedNode(viewModel, itemByNodeId);
    return true;
}

void SceneExplorerWidget::rebuildTree(const robot_qt_viewer::SceneExplorerViewModel& viewModel)
{
    m_robotTree->clear();
    QHash<QString, QTreeWidgetItem*> itemByNodeId;
    for(const robot_qt_viewer::SceneExplorerNodeView& node : viewModel.nodes) {
        QTreeWidgetItem* parentItem = nullptr;
        if(!node.parentNodeId.isEmpty()) {
            parentItem = itemByNodeId.value(node.parentNodeId, nullptr);
        }

        auto* item = parentItem != nullptr
            ? new QTreeWidgetItem(parentItem)
            : new QTreeWidgetItem(m_robotTree);
        applyNodeViewToItem(*item, node);

        if(!node.nodeId.isEmpty()) {
            itemByNodeId.insert(node.nodeId, item);
        }
    }
    setCurrentItemForSelectedNode(viewModel, itemByNodeId);
}

void SceneExplorerWidget::applyNodeViewToItem(
    QTreeWidgetItem& item,
    const robot_qt_viewer::SceneExplorerNodeView& node)
{
    item.setText(0, node.text);
    QString toolTip = node.toolTip;
    if(!node.taskHint.isEmpty()) {
        if(!toolTip.isEmpty()) {
            toolTip += QStringLiteral("\n");
        }
        toolTip += node.taskHint;
    }
    item.setToolTip(0, toolTip);
    item.setData(0, robot_qt_viewer::kSceneExplorerRoleId, node.ref.id);
    item.setData(0, robot_qt_viewer::kSceneExplorerRoleName, node.ref.name);
    item.setData(0, robot_qt_viewer::kSceneExplorerRoleLink, node.ref.linkName);
    item.setData(0, robot_qt_viewer::kSceneExplorerRoleTaskSelectable, node.taskSelectable);
    item.setData(0, robot_qt_viewer::kSceneExplorerRoleTaskHighlighted, node.taskHighlighted);
    item.setData(0, robot_qt_viewer::kSceneExplorerRoleType,
        robot_qt_viewer::sceneExplorerNodeTypeName(node.ref.kind));
    item.setExpanded(node.expanded);

    item.setForeground(0, node.taskSelectable
        ? QBrush()
        : QBrush(QColor(128, 136, 145)));
    QFont font = item.font(0);
    font.setBold(false);
    item.setFont(0, font);
}

void SceneExplorerWidget::setCurrentItemForSelectedNode(
    const robot_qt_viewer::SceneExplorerViewModel& viewModel,
    const QHash<QString, QTreeWidgetItem*>& itemByNodeId)
{
    if(viewModel.selectedNodeId.isEmpty()) {
        return;
    }

    QTreeWidgetItem* selectedItem = itemByNodeId.value(viewModel.selectedNodeId, nullptr);
    if(selectedItem != nullptr && m_robotTree->currentItem() != selectedItem) {
        m_robotTree->setCurrentItem(selectedItem);
    }
}

void SceneExplorerWidget::setSummaryText(const QString& text)
{
    if(m_summaryLabel != nullptr) {
        m_summaryLabel->setText(text);
    }
}

robot_qt_viewer::SceneExplorerNodeRef SceneExplorerWidget::nodeRefFromItem(const QTreeWidgetItem* item) const
{
    robot_qt_viewer::SceneExplorerNodeRef node;
    if(item == nullptr) {
        return node;
    }

    const QString type = item->data(0, robot_qt_viewer::kSceneExplorerRoleType).toString();
    node.kind = robot_qt_viewer::sceneExplorerNodeKindFromType(type);
    node.id = item->data(0, robot_qt_viewer::kSceneExplorerRoleId).toString();
    node.name = item->data(0, robot_qt_viewer::kSceneExplorerRoleName).toString();
    node.linkName = item->data(0, robot_qt_viewer::kSceneExplorerRoleLink).toString();
    return node;
}
