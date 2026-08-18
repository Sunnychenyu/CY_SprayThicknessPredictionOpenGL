#include "CollisionDetectorConfigWidget.h"

#include "CollisionDetectorsWidget.h"
#include "CollisionLegacyPairsViewModel.h"
#include "CollisionSelectionSetsWidget.h"
#include "RobotQtWidgetUtils.h"

#include <QAbstractItemView>
#include <QFrame>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

namespace
{
    using robot_qt_viewer::configureInspectorList;
    using robot_qt_viewer::makeHorizontallyCompressible;

    constexpr int kLegacyPairRobotRole = Qt::UserRole;
    constexpr int kLegacyPairObjectRole = Qt::UserRole + 1;

}

CollisionDetectorConfigWidget::CollisionDetectorConfigWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(0, 0, 0, 0);
    rootLayout->setSpacing(8);

    m_detectorsWidget = new CollisionDetectorsWidget(this);
    rootLayout->addWidget(m_detectorsWidget, 1);

    m_selectionSetsWidget = new CollisionSelectionSetsWidget(this);
    m_selectionSetsWidget->hide();

    m_legacyPairList = new QListWidget(this);
    m_legacyPairList->setSelectionMode(QAbstractItemView::SingleSelection);
    m_legacyPairList->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_legacyPairList->setWordWrap(true);
    m_legacyPairList->setFrameShape(QFrame::StyledPanel);
    configureInspectorList(m_legacyPairList, true, false);
    m_legacyPairList->hide();

    m_autoPairAllButton = new QPushButton("Auto Pair Legacy", this);
    makeHorizontallyCompressible(m_autoPairAllButton);
    m_autoPairAllButton->hide();

    connect(m_legacyPairList, &QListWidget::itemChanged,
        this, &CollisionDetectorConfigWidget::handleLegacyPairItemChanged);
    connect(m_autoPairAllButton, &QPushButton::clicked,
        this, &CollisionDetectorConfigWidget::autoPairAllRequested);
}

CollisionSelectionSetsWidget* CollisionDetectorConfigWidget::selectionSetsWidget() const
{
    return m_selectionSetsWidget;
}

CollisionDetectorsWidget* CollisionDetectorConfigWidget::detectorsWidget() const
{
    return m_detectorsWidget;
}

QListWidget* CollisionDetectorConfigWidget::legacyPairList() const
{
    return m_legacyPairList;
}

QPushButton* CollisionDetectorConfigWidget::autoPairAllButton() const
{
    return m_autoPairAllButton;
}

void CollisionDetectorConfigWidget::setLegacyPairs(const QVector<CollisionLegacyPairItemView>& items)
{
    if(m_legacyPairList == nullptr) {
        return;
    }

    const QSignalBlocker blocker(m_legacyPairList);
    m_legacyPairList->clear();
    for(const CollisionLegacyPairItemView& view : items) {
        auto* item = new QListWidgetItem(view.label);
        item->setToolTip(view.label);
        if(view.selectable) {
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
            item->setCheckState(view.enabled ? Qt::Checked : Qt::Unchecked);
            item->setData(kLegacyPairRobotRole, view.robotId);
            item->setData(kLegacyPairObjectRole, view.objectId);
        } else {
            item->setFlags(item->flags() & ~(Qt::ItemIsEnabled | Qt::ItemIsSelectable));
        }

        m_legacyPairList->addItem(item);
    }
}

void CollisionDetectorConfigWidget::setAutoPairAllEnabled(bool enabled)
{
    if(m_autoPairAllButton != nullptr) {
        m_autoPairAllButton->setEnabled(enabled);
    }
}

void CollisionDetectorConfigWidget::handleLegacyPairItemChanged(QListWidgetItem* item)
{
    if(item == nullptr) {
        return;
    }

    const QString robotId = item->data(kLegacyPairRobotRole).toString();
    const QString objectId = item->data(kLegacyPairObjectRole).toString();
    if(robotId.isEmpty() || objectId.isEmpty()) {
        return;
    }

    emit legacyPairEnabledChanged(robotId, objectId, item->checkState() == Qt::Checked);
}
