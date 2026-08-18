#include "CollisionSelectionSetsWidget.h"

#include "CollisionSelectionSetsViewModel.h"
#include "RobotQtWidgetUtils.h"

#include <QGridLayout>
#include <QLabel>
#include <QLayout>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVariant>
#include <QVBoxLayout>

namespace
{
    using robot_qt_viewer::configureInspectorGrid;
    using robot_qt_viewer::configureInspectorList;
    using robot_qt_viewer::makeHorizontallyCompressible;
    using robot_qt_viewer::makePanelTitle;

    constexpr int kSelectionSetIdRole = Qt::UserRole;
    constexpr int kMemberIndexRole = Qt::UserRole;
}

CollisionSelectionSetsWidget::CollisionSelectionSetsWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(8);

    layout->addWidget(makePanelTitle("Target Sets", this));

    m_selectionSetList = new QListWidget(this);
    configureInspectorList(m_selectionSetList, true);
    connect(m_selectionSetList, &QListWidget::currentItemChanged, this, [this](QListWidgetItem*, QListWidgetItem*) {
        emit selectionSetSelectionChanged();
    });
    layout->addWidget(m_selectionSetList, 1);

    m_memberList = new QListWidget(this);
    configureInspectorList(m_memberList, true);
    connect(m_memberList, &QListWidget::currentItemChanged, this, [this](QListWidgetItem*, QListWidgetItem*) {
        emit memberSelectionChanged();
    });
    layout->addWidget(m_memberList, 1);

    auto* buttonLayout = new QGridLayout();
    buttonLayout->setContentsMargins(0, 0, 0, 0);
    buttonLayout->setSpacing(8);
    configureInspectorGrid(buttonLayout);

    m_addSetButton = new QPushButton("Add Target Set", this);
    makeHorizontallyCompressible(m_addSetButton);
    connect(m_addSetButton, &QPushButton::clicked, this, &CollisionSelectionSetsWidget::addSetRequested);
    buttonLayout->addWidget(m_addSetButton, 0, 0);

    m_renameSetButton = new QPushButton("Rename Set", this);
    makeHorizontallyCompressible(m_renameSetButton);
    connect(m_renameSetButton, &QPushButton::clicked, this, &CollisionSelectionSetsWidget::renameSetRequested);
    buttonLayout->addWidget(m_renameSetButton, 0, 1);

    m_removeSetButton = new QPushButton("Remove Set", this);
    makeHorizontallyCompressible(m_removeSetButton);
    connect(m_removeSetButton, &QPushButton::clicked, this, &CollisionSelectionSetsWidget::removeSetRequested);
    buttonLayout->addWidget(m_removeSetButton, 1, 0);

    m_removeMemberButton = new QPushButton("Remove Member", this);
    makeHorizontallyCompressible(m_removeMemberButton);
    connect(m_removeMemberButton, &QPushButton::clicked, this, &CollisionSelectionSetsWidget::removeMemberRequested);
    buttonLayout->addWidget(m_removeMemberButton, 1, 1);
    buttonLayout->setColumnStretch(0, 1);
    buttonLayout->setColumnStretch(1, 1);
    layout->addLayout(buttonLayout);
}

void CollisionSelectionSetsWidget::setSelectionSets(
    const QVector<CollisionSelectionSetListItemView>& items,
    const QString& preferredId)
{
    if(m_selectionSetList == nullptr) {
        return;
    }

    QSignalBlocker blocker(m_selectionSetList);
    m_selectionSetList->clear();
    QListWidgetItem* itemToSelect = nullptr;
    for(const CollisionSelectionSetListItemView& view : items) {
        auto* item = new QListWidgetItem(view.text, m_selectionSetList);
        item->setData(kSelectionSetIdRole, view.id);
        item->setToolTip(view.tooltip.isEmpty() ? view.text : view.tooltip);
        if(!view.enabled) {
            item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
        }
        if(view.enabled && view.id == preferredId) {
            itemToSelect = item;
        }
    }

    if(itemToSelect == nullptr) {
        for(int row = 0; row < m_selectionSetList->count(); ++row) {
            QListWidgetItem* item = m_selectionSetList->item(row);
            if(item != nullptr && item->flags().testFlag(Qt::ItemIsEnabled)) {
                itemToSelect = item;
                break;
            }
        }
    }
    if(itemToSelect != nullptr) {
        m_selectionSetList->setCurrentItem(itemToSelect);
    }
}

void CollisionSelectionSetsWidget::setMembers(
    const QVector<CollisionSelectionSetMemberItemView>& items,
    int preferredIndex)
{
    if(m_memberList == nullptr) {
        return;
    }

    QSignalBlocker blocker(m_memberList);
    m_memberList->clear();
    int rowToSelect = -1;
    for(const CollisionSelectionSetMemberItemView& view : items) {
        auto* item = new QListWidgetItem(view.text, m_memberList);
        item->setData(kMemberIndexRole, view.index);
        item->setToolTip(view.text);
        if(!view.enabled) {
            item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
        }
        if(view.enabled && view.index == preferredIndex) {
            rowToSelect = m_memberList->row(item);
        }
    }

    if(rowToSelect < 0) {
        for(int row = 0; row < m_memberList->count(); ++row) {
            QListWidgetItem* item = m_memberList->item(row);
            if(item != nullptr && item->flags().testFlag(Qt::ItemIsEnabled)) {
                rowToSelect = row;
                break;
            }
        }
    }
    if(rowToSelect >= 0) {
        m_memberList->setCurrentRow(rowToSelect);
    }
}

void CollisionSelectionSetsWidget::setSelectionSetActionsEnabled(bool enabled)
{
    if(m_renameSetButton != nullptr) {
        m_renameSetButton->setEnabled(enabled);
    }
    if(m_removeSetButton != nullptr) {
        m_removeSetButton->setEnabled(enabled);
    }
}

void CollisionSelectionSetsWidget::setRemoveMemberEnabled(bool enabled)
{
    if(m_removeMemberButton != nullptr) {
        m_removeMemberButton->setEnabled(enabled);
    }
}

bool CollisionSelectionSetsWidget::selectSelectionSet(const QString& selectionSetId)
{
    if(m_selectionSetList == nullptr) {
        return false;
    }
    for(int row = 0; row < m_selectionSetList->count(); ++row) {
        QListWidgetItem* item = m_selectionSetList->item(row);
        if(item != nullptr && item->data(kSelectionSetIdRole).toString() == selectionSetId) {
            m_selectionSetList->setCurrentItem(item);
            return true;
        }
    }
    return false;
}

QString CollisionSelectionSetsWidget::currentSelectionSetId() const
{
    if(m_selectionSetList == nullptr || m_selectionSetList->currentItem() == nullptr) {
        return QString();
    }
    return m_selectionSetList->currentItem()->data(kSelectionSetIdRole).toString();
}

int CollisionSelectionSetsWidget::currentMemberIndex() const
{
    if(m_memberList == nullptr || m_memberList->currentItem() == nullptr) {
        return -1;
    }
    const QVariant value = m_memberList->currentItem()->data(kMemberIndexRole);
    return value.isValid() ? value.toInt() : -1;
}
