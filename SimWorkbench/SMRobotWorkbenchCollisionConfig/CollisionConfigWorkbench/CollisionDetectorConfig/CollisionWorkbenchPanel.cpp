#include "CollisionWorkbenchPanel.h"

#include "CollisionDetectorConfigWidget.h"
#include "CollisionDetectorsWidget.h"
#include "CollisionDetectorsViewModel.h"
#include "CollisionLegacyPairsViewModel.h"
#include "CollisionLinkModelSetupWidget.h"
#include "CollisionLinkModelsWidget.h"
#include "CollisionLinkModelsViewModel.h"
#include "CollisionSelectionSetsWidget.h"
#include "CollisionSelectionSetsViewModel.h"
#include "RobotQtWidgetUtils.h"

#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStackedWidget>
#include <QVBoxLayout>

namespace
{
    using robot_qt_viewer::makeHorizontallyCompressible;

    constexpr int kLegacyPairRobotRole = Qt::UserRole;
    constexpr int kLegacyPairObjectRole = Qt::UserRole + 1;
}

CollisionWorkbenchPanel::CollisionWorkbenchPanel(QWidget* parent)
    : QWidget(parent)
{
    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(0, 0, 0, 0);
    rootLayout->setSpacing(0);

    m_pages = new QStackedWidget(this);
    makeHorizontallyCompressible(m_pages);

    m_linkModelSetupWidget = new CollisionLinkModelSetupWidget(m_pages);
    m_linkModelsWidget = m_linkModelSetupWidget->linkModelsWidget();

    m_detectorConfigWidget = new CollisionDetectorConfigWidget(m_pages);
    m_selectionSetsWidget = m_detectorConfigWidget->selectionSetsWidget();
    m_detectorsWidget = m_detectorConfigWidget->detectorsWidget();
    m_legacyPairList = m_detectorConfigWidget->legacyPairList();
    m_autoPairAllButton = m_detectorConfigWidget->autoPairAllButton();

    m_pages->addWidget(m_detectorConfigWidget);
    m_pages->addWidget(m_linkModelSetupWidget);

    rootLayout->addWidget(m_pages, 1);
    setLayout(rootLayout);
    showDetectorConfiguration();

    connectChildSignals();
}

CollisionLinkModelsWidget* CollisionWorkbenchPanel::linkModelsWidget() const
{
    return m_linkModelsWidget;
}

CollisionSelectionSetsWidget* CollisionWorkbenchPanel::selectionSetsWidget() const
{
    return m_selectionSetsWidget;
}

void CollisionWorkbenchPanel::showDetectorConfiguration()
{
    if(m_pages != nullptr) {
        m_pages->setCurrentWidget(m_detectorConfigWidget);
    }
    emit rightPanelTitleChanged(QStringLiteral("Collision Detector Configuration"));
}

void CollisionWorkbenchPanel::showCollisionModelConfiguration()
{
    if(m_pages != nullptr) {
        m_pages->setCurrentWidget(m_linkModelSetupWidget);
    }
    emit rightPanelTitleChanged(QStringLiteral("Collision Model Configuration"));
}

void CollisionWorkbenchPanel::setSelectionSets(
    const QVector<CollisionSelectionSetListItemView>& items,
    const QString& preferredId)
{
    if(m_selectionSetsWidget != nullptr) {
        m_selectionSetsWidget->setSelectionSets(items, preferredId);
    }
}

void CollisionWorkbenchPanel::setSelectionSetMembers(
    const QVector<CollisionSelectionSetMemberItemView>& items,
    int preferredIndex)
{
    if(m_selectionSetsWidget != nullptr) {
        m_selectionSetsWidget->setMembers(items, preferredIndex);
    }
}

void CollisionWorkbenchPanel::setSelectionSetActionsEnabled(bool enabled)
{
    if(m_selectionSetsWidget != nullptr) {
        m_selectionSetsWidget->setSelectionSetActionsEnabled(enabled);
    }
}

void CollisionWorkbenchPanel::setRemoveSelectionSetMemberEnabled(bool enabled)
{
    if(m_selectionSetsWidget != nullptr) {
        m_selectionSetsWidget->setRemoveMemberEnabled(enabled);
    }
}

bool CollisionWorkbenchPanel::selectSelectionSet(const QString& selectionSetId)
{
    return m_selectionSetsWidget != nullptr && m_selectionSetsWidget->selectSelectionSet(selectionSetId);
}

QString CollisionWorkbenchPanel::currentSelectionSetId() const
{
    return m_selectionSetsWidget != nullptr ? m_selectionSetsWidget->currentSelectionSetId() : QString();
}

int CollisionWorkbenchPanel::currentSelectionSetMemberIndex() const
{
    return m_selectionSetsWidget != nullptr ? m_selectionSetsWidget->currentMemberIndex() : -1;
}

void CollisionWorkbenchPanel::setDetectors(const QVector<CollisionDetectorListItemView>& items, const QString& preferredId)
{
    if(m_detectorsWidget != nullptr) {
        m_detectorsWidget->setDetectors(items, preferredId);
    }
}

void CollisionWorkbenchPanel::setDetectorProperties(const CollisionDetectorPropertiesView& view)
{
    if(m_detectorsWidget != nullptr) {
        m_detectorsWidget->setProperties(view);
    }
}

void CollisionWorkbenchPanel::setDetectorPairs(const CollisionDetectorPairsViewModel& view)
{
    if(m_detectorsWidget != nullptr) {
        m_detectorsWidget->setPairs(view);
    }
}

CollisionDetectorPropertiesView CollisionWorkbenchPanel::currentDetectorProperties() const
{
    return m_detectorsWidget != nullptr ? m_detectorsWidget->currentProperties() : CollisionDetectorPropertiesView();
}

CollisionDetectorQueryContractView CollisionWorkbenchPanel::currentDetectorQueryContract() const
{
    return m_detectorsWidget != nullptr
        ? m_detectorsWidget->currentQueryContract()
        : CollisionDetectorQueryContractView();
}

QString CollisionWorkbenchPanel::currentDetectorId() const
{
    return m_detectorsWidget != nullptr ? m_detectorsWidget->currentDetectorId() : QString();
}

QVector<QString> CollisionWorkbenchPanel::selectedDetectorIds() const
{
    return m_detectorsWidget != nullptr ? m_detectorsWidget->selectedDetectorIds() : QVector<QString>();
}

void CollisionWorkbenchPanel::selectDetector(const QString& detectorId)
{
    if(m_detectorsWidget != nullptr) {
        m_detectorsWidget->selectDetector(detectorId);
    }
}

void CollisionWorkbenchPanel::setDetectorActionsEnabled(bool canAdd, bool hasDetector, bool canRemove)
{
    if(m_detectorsWidget != nullptr) {
        m_detectorsWidget->setDetectorActionsEnabled(canAdd, hasDetector, canRemove);
    }
}

void CollisionWorkbenchPanel::setLinkPairActionsEnabled(bool canMarkLinkA, bool canCreateLinkLink)
{
    if(m_detectorsWidget != nullptr) {
        m_detectorsWidget->setLinkPairActionsEnabled(canMarkLinkA, canCreateLinkLink);
    }
}

bool CollisionWorkbenchPanel::setCurrentDetectorRole(const QString& role)
{
    return m_detectorsWidget != nullptr && m_detectorsWidget->setCurrentRole(role);
}

void CollisionWorkbenchPanel::addDetectorDraftSetMember(
    const QString& side,
    const CollisionDetectorDraftMemberView& member)
{
    if(m_detectorsWidget != nullptr) {
        m_detectorsWidget->addDraftSetMember(side, member);
    }
}

void CollisionWorkbenchPanel::clearDetectorDraftPairBuilder()
{
    if(m_detectorsWidget != nullptr) {
        m_detectorsWidget->clearDraftPairBuilder();
    }
}

QVector<CollisionDetectorDraftMemberView> CollisionWorkbenchPanel::detectorDraftSetMembers(
    const QString& side) const
{
    return m_detectorsWidget != nullptr
        ? m_detectorsWidget->draftSetMembers(side)
        : QVector<CollisionDetectorDraftMemberView>();
}

void CollisionWorkbenchPanel::setLinkModelsViewModel(const CollisionLinkModelsViewModel& viewModel)
{
    if(m_linkModelsWidget != nullptr) {
        m_linkModelsWidget->setViewModel(viewModel);
    }
}

void CollisionWorkbenchPanel::setLinkModelsSummary(const CollisionLinkModelsSummaryView& summary)
{
    if(m_linkModelsWidget != nullptr) {
        m_linkModelsWidget->setSummary(summary);
    }
}

QString CollisionWorkbenchPanel::currentVariantId() const
{
    return m_linkModelsWidget != nullptr ? m_linkModelsWidget->currentVariantId() : QString();
}

QString CollisionWorkbenchPanel::currentVariantRole() const
{
    return m_linkModelsWidget != nullptr ? m_linkModelsWidget->currentVariantRole() : QString();
}

QString CollisionWorkbenchPanel::currentVariantSource() const
{
    return m_linkModelsWidget != nullptr ? m_linkModelsWidget->currentVariantSource() : QString();
}

bool CollisionWorkbenchPanel::hasCurrentVariant() const
{
    return m_linkModelsWidget != nullptr && m_linkModelsWidget->hasCurrentVariant();
}

bool CollisionWorkbenchPanel::selectVariantBySourceRole(const QString& source, const QString& role)
{
    return m_linkModelsWidget != nullptr && m_linkModelsWidget->selectVariantBySourceRole(source, role);
}

void CollisionWorkbenchPanel::setContextActionsEnabled(bool hasRobot, bool hasLink)
{
    if(m_linkModelsWidget != nullptr) {
        m_linkModelsWidget->setContextActionsEnabled(hasRobot, hasLink);
    }
}

void CollisionWorkbenchPanel::setVariantActionsEnabled(bool canUseVariant, bool canShowVariant)
{
    if(m_linkModelsWidget != nullptr) {
        m_linkModelsWidget->setVariantActionsEnabled(canUseVariant, canShowVariant);
    }
}

void CollisionWorkbenchPanel::setLegacyPairs(const QVector<CollisionLegacyPairItemView>& items)
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

void CollisionWorkbenchPanel::setAutoPairAllEnabled(bool enabled)
{
    if(m_autoPairAllButton != nullptr) {
        m_autoPairAllButton->setEnabled(enabled);
    }
}

void CollisionWorkbenchPanel::connectChildSignals()
{
    connect(m_selectionSetsWidget, &CollisionSelectionSetsWidget::selectionSetSelectionChanged,
        this, &CollisionWorkbenchPanel::selectionSetSelectionChanged);
    connect(m_selectionSetsWidget, &CollisionSelectionSetsWidget::memberSelectionChanged,
        this, &CollisionWorkbenchPanel::selectionSetMemberSelectionChanged);
    connect(m_selectionSetsWidget, &CollisionSelectionSetsWidget::addSetRequested,
        this, &CollisionWorkbenchPanel::addSelectionSetRequested);
    connect(m_selectionSetsWidget, &CollisionSelectionSetsWidget::renameSetRequested,
        this, &CollisionWorkbenchPanel::renameSelectionSetRequested);
    connect(m_selectionSetsWidget, &CollisionSelectionSetsWidget::removeSetRequested,
        this, &CollisionWorkbenchPanel::removeSelectionSetRequested);
    connect(m_selectionSetsWidget, &CollisionSelectionSetsWidget::removeMemberRequested,
        this, &CollisionWorkbenchPanel::removeSelectionSetMemberRequested);

    connect(m_detectorsWidget, &CollisionDetectorsWidget::detectorEnabledChanged,
        this, &CollisionWorkbenchPanel::detectorEnabledChanged);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::detectorSelectionChanged,
        this, &CollisionWorkbenchPanel::detectorSelectionChanged);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::detectorClicked,
        this, &CollisionWorkbenchPanel::detectorClicked);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::detectorPropertyChanged,
        this, &CollisionWorkbenchPanel::detectorPropertyChanged);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::addDetectorRequested,
        this, &CollisionWorkbenchPanel::addDetectorRequested);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::editDetectorRequested,
        this, &CollisionWorkbenchPanel::editDetectorRequested);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::showSelectedRequested,
        this, &CollisionWorkbenchPanel::showSelectedDetectorsRequested);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::removeDetectorRequested,
        this, &CollisionWorkbenchPanel::removeDetectorRequested);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::bindDraftSetsRequested,
        this, &CollisionWorkbenchPanel::bindDetectorDraftSetsRequested);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::removePairGeneratorsRequested,
        this, &CollisionWorkbenchPanel::removeDetectorPairGeneratorsRequested);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::clearPairScopeRequested,
        this, &CollisionWorkbenchPanel::clearDetectorPairScopeRequested);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::pairScopePreviewRequested,
        this, &CollisionWorkbenchPanel::detectorPairScopePreviewRequested);
    connect(m_detectorsWidget, &CollisionDetectorsWidget::draftMemberPreviewRequested,
        this, &CollisionWorkbenchPanel::detectorDraftMemberPreviewRequested);

    connect(m_linkModelsWidget, &CollisionLinkModelsWidget::variantSelectionChanged,
        this, &CollisionWorkbenchPanel::linkModelVariantSelectionChanged);

    connect(m_legacyPairList, &QListWidget::itemChanged,
        this, &CollisionWorkbenchPanel::handleLegacyPairItemChanged);
    connect(m_autoPairAllButton, &QPushButton::clicked,
        this, &CollisionWorkbenchPanel::autoPairAllRequested);
}

void CollisionWorkbenchPanel::handleLegacyPairItemChanged(QListWidgetItem* item)
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
