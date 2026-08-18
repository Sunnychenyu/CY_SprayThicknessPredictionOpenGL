#include "CollisionDetectorsWidget.h"

#include "CollisionDetectorQueryDialog.h"
#include "RobotQtWidgetUtils.h"

#include <QAbstractItemView>
#include <QComboBox>
#include <QDialog>
#include <QFrame>
#include <QGridLayout>
#include <QItemSelectionModel>
#include <QLabel>
#include <QLayout>
#include <QListWidget>
#include <QListWidgetItem>
#include <QMenu>
#include <QPushButton>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>
#include <utility>

namespace
{
    using robot_qt_viewer::configureInspectorCombo;
    using robot_qt_viewer::configureInspectorGrid;
    using robot_qt_viewer::configureInspectorList;
    using robot_qt_viewer::makeHorizontallyCompressible;
    using robot_qt_viewer::makePanelTitle;

    constexpr int kDraftMemberRobotRole = Qt::UserRole;
    constexpr int kDraftMemberLinkRole = Qt::UserRole + 1;
    constexpr int kDraftMemberObjectRole = Qt::UserRole + 2;
    constexpr int kDraftMemberAttachmentRole = Qt::UserRole + 3;
    constexpr int kDraftMemberKeyRole = Qt::UserRole + 4;
    constexpr int kPairGeneratorIndexRole = Qt::UserRole;
    constexpr int kPairRemovableRole = Qt::UserRole + 1;
    constexpr int kPairRobotAIdRole = Qt::UserRole + 2;
    constexpr int kPairLinkANameRole = Qt::UserRole + 3;
    constexpr int kPairObjectAIdRole = Qt::UserRole + 4;
    constexpr int kPairAttachmentAIdRole = Qt::UserRole + 5;
    constexpr int kPairRobotBIdRole = Qt::UserRole + 6;
    constexpr int kPairLinkBNameRole = Qt::UserRole + 7;
    constexpr int kPairObjectBIdRole = Qt::UserRole + 8;
    constexpr int kPairAttachmentBIdRole = Qt::UserRole + 9;
    constexpr int kPairClearsScopeRole = Qt::UserRole + 10;

    QString yesNo(bool value)
    {
        return value ? QStringLiteral("on") : QStringLiteral("off");
    }

    bool isKnownRole(const QString& role)
    {
        return role == QStringLiteral("Exact")
            || role == QStringLiteral("PlanningProxy")
            || role == QStringLiteral("Simplified")
            || role == QStringLiteral("SafetyMargin")
            || role == QStringLiteral("SphereCover");
    }

    CollisionDetectorQueryContractView makeQueryContract(
        const CollisionDetectorPropertiesView& properties,
        const QString& detectorId)
    {
        CollisionDetectorQueryContractView contract;
        contract.hasDetector = properties.hasDetector || !detectorId.isEmpty();
        contract.id = properties.id.isEmpty() ? detectorId : properties.id;
        contract.name = properties.name;
        contract.contacts = properties.contacts;
        contract.normals = properties.normals;
        contract.nearest = properties.nearest;
        contract.maxContacts = properties.maxContacts;
        contract.distanceThreshold = properties.distanceThreshold;
        contract.role = properties.role;
        return contract;
    }

    CollisionDetectorPropertiesView applyQueryContractToProperties(
        CollisionDetectorPropertiesView properties,
        const CollisionDetectorQueryContractView& contract)
    {
        properties.hasDetector = contract.hasDetector;
        properties.id = contract.id;
        properties.name = contract.name;
        properties.contacts = contract.contacts;
        properties.normals = contract.normals;
        properties.nearest = contract.nearest;
        properties.maxContacts = contract.maxContacts;
        properties.distanceThreshold = contract.distanceThreshold;
        properties.role = contract.role;
        return properties;
    }

    void addSectionHeader(QVBoxLayout* layout, QWidget* parent, const QString& title, bool separated)
    {
        if(layout == nullptr) {
            return;
        }
        if(separated) {
            auto* separator = new QFrame(parent);
            separator->setFrameShape(QFrame::HLine);
            separator->setFrameShadow(QFrame::Plain);
            separator->setStyleSheet(QStringLiteral("color: rgba(110, 150, 185, 120);"));
            layout->addSpacing(12);
            layout->addWidget(separator);
            layout->addSpacing(8);
        }

        QLabel* label = makePanelTitle(title, parent);
        label->setContentsMargins(6, 2, 0, 4);
        layout->addWidget(label);
    }
}

CollisionDetectorsWidget::CollisionDetectorsWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    addSectionHeader(layout, this, QStringLiteral("Detector Config"), false);

    auto* selectorLayout = new QGridLayout();
    selectorLayout->setContentsMargins(6, 0, 6, 0);
    selectorLayout->setSpacing(8);
    configureInspectorGrid(selectorLayout);

    m_detectorCombo = new QComboBox(this);
    configureInspectorCombo(m_detectorCombo);
    m_detectorCombo->setToolTip("Selects the collision detector to configure.");
    makeHorizontallyCompressible(m_detectorCombo);
    connect(m_detectorCombo, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged), this, [this]() {
        const QString detectorId = detectorIdForCurrentIndex();
        clearDraftPairBuilder();
        emit detectorSelectionChanged();
        if(!detectorId.isEmpty()) {
            emit detectorClicked(detectorId);
        }
    });
    selectorLayout->addWidget(m_detectorCombo, 0, 0, 1, 2);
    selectorLayout->setColumnStretch(0, 1);
    layout->addLayout(selectorLayout);

    auto* setupButtonLayout = new QGridLayout();
    setupButtonLayout->setContentsMargins(6, 0, 6, 0);
    setupButtonLayout->setSpacing(8);
    configureInspectorGrid(setupButtonLayout);

    m_configureButton = new QPushButton("Configure...", this);
    makeHorizontallyCompressible(m_configureButton);
    m_configureButton->setToolTip("Opens the detector query configuration dialog.");
    connect(m_configureButton, &QPushButton::clicked, this, [this]() {
        openQueryDialog();
    });
    setupButtonLayout->addWidget(m_configureButton, 0, 0);

    m_addDetectorButton = new QPushButton("New Detector", this);
    makeHorizontallyCompressible(m_addDetectorButton);
    m_addDetectorButton->setToolTip("Creates a detector from the current collision target sets.");
    connect(m_addDetectorButton, &QPushButton::clicked, this, &CollisionDetectorsWidget::addDetectorRequested);
    setupButtonLayout->addWidget(m_addDetectorButton, 0, 1);

    m_removeDetectorButton = new QPushButton("Delete Detector", this);
    makeHorizontallyCompressible(m_removeDetectorButton);
    m_removeDetectorButton->setToolTip("Removes the current detector. At least one detector must remain in the project.");
    connect(m_removeDetectorButton, &QPushButton::clicked, this, &CollisionDetectorsWidget::removeDetectorRequested);
    setupButtonLayout->addWidget(m_removeDetectorButton, 1, 0);

    m_showSelectedButton = new QPushButton("Show Selected", this);
    makeHorizontallyCompressible(m_showSelectedButton);
    m_showSelectedButton->setToolTip("Shows the selected detector overlay and hides the others.");
    connect(m_showSelectedButton, &QPushButton::clicked, this, &CollisionDetectorsWidget::showSelectedRequested);
    setupButtonLayout->addWidget(m_showSelectedButton, 1, 1);

    setupButtonLayout->setColumnStretch(0, 1);
    setupButtonLayout->setColumnStretch(1, 1);
    layout->addLayout(setupButtonLayout);

    m_summaryLabel = new QLabel(this);
    m_summaryLabel->setWordWrap(true);
    m_summaryLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_summaryLabel->setContentsMargins(6, 2, 6, 0);
    layout->addWidget(m_summaryLabel);

    addSectionHeader(layout, this, QStringLiteral("Collision Pair Scope"), true);

    m_pairSummaryLabel = new QLabel(this);
    m_pairSummaryLabel->setWordWrap(true);
    m_pairSummaryLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_pairSummaryLabel->setContentsMargins(6, 0, 6, 0);
    m_pairSummaryLabel->hide();
    layout->addWidget(m_pairSummaryLabel);

    m_pairList = new QListWidget(this);
    configureInspectorList(m_pairList, true, false);
    m_pairList->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_pairList->setWordWrap(true);
    m_pairList->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_pairList, &QListWidget::currentItemChanged, this, [this](QListWidgetItem* current, QListWidgetItem*) {
        previewPairScopeItem(current);
    });
    connect(m_pairList, &QListWidget::itemSelectionChanged, this, [this]() {
        previewPairScopeItem(m_pairList != nullptr ? m_pairList->currentItem() : nullptr);
    });
    connect(m_pairList, &QListWidget::itemClicked, this, [this](QListWidgetItem* item) {
        if(m_pairList != nullptr && item != nullptr) {
            m_pairList->setCurrentItem(item, QItemSelectionModel::ClearAndSelect);
        }
        previewPairScopeItem(item);
    });
    connect(m_pairList, &QWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
        showPairScopeContextMenu(pos);
    });
    m_pairList->setContentsMargins(6, 0, 6, 0);
    layout->addWidget(m_pairList, 1);

    addSectionHeader(layout, this, QStringLiteral("Pair Configuration"), true);

    auto* draftButtonLayout = new QGridLayout();
    draftButtonLayout->setContentsMargins(6, 0, 6, 0);
    draftButtonLayout->setSpacing(8);
    configureInspectorGrid(draftButtonLayout);

    draftButtonLayout->addWidget(makePanelTitle("Set A", this), 0, 0, 1, 2);

    m_setAList = new QListWidget(this);
    configureInspectorList(m_setAList, true, false);
    m_setAList->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_setAList->setWordWrap(true);
    m_setAList->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_setAList, &QListWidget::currentItemChanged, this, [this](QListWidgetItem* current, QListWidgetItem*) {
        previewDraftMember(current);
        updateDraftActionState();
    });
    connect(m_setAList, &QListWidget::itemSelectionChanged, this, [this]() {
        updateDraftActionState();
    });
    connect(m_setAList, &QWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
        showDraftSetContextMenu(m_setAList, pos);
    });
    draftButtonLayout->addWidget(m_setAList, 1, 0, 1, 2);

    draftButtonLayout->addWidget(makePanelTitle("Set B", this), 2, 0, 1, 2);

    m_setBList = new QListWidget(this);
    configureInspectorList(m_setBList, true, false);
    m_setBList->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_setBList->setWordWrap(true);
    m_setBList->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_setBList, &QListWidget::currentItemChanged, this, [this](QListWidgetItem* current, QListWidgetItem*) {
        previewDraftMember(current);
        updateDraftActionState();
    });
    connect(m_setBList, &QListWidget::itemSelectionChanged, this, [this]() {
        updateDraftActionState();
    });
    connect(m_setBList, &QWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
        showDraftSetContextMenu(m_setBList, pos);
    });
    draftButtonLayout->addWidget(m_setBList, 3, 0, 1, 2);

    m_bindDraftSetsButton = new QPushButton("Add Model Pairs To Detector", this);
    makeHorizontallyCompressible(m_bindDraftSetsButton);
    m_bindDraftSetsButton->setToolTip("Adds generated model-pair rules to this detector without removing existing rules.");
    connect(m_bindDraftSetsButton, &QPushButton::clicked, this, &CollisionDetectorsWidget::bindDraftSetsRequested);
    draftButtonLayout->addWidget(m_bindDraftSetsButton, 4, 0, 1, 2);
    draftButtonLayout->setColumnStretch(0, 1);
    draftButtonLayout->setColumnStretch(1, 1);
    layout->addLayout(draftButtonLayout);

    updateSummary();
    updateActionState();
    updateDraftActionState();
}

void CollisionDetectorsWidget::setDetectors(
    const QVector<CollisionDetectorListItemView>& items,
    const QString& preferredId)
{
    if(m_detectorCombo == nullptr) {
        return;
    }

    QSignalBlocker blocker(m_detectorCombo);
    m_detectorCombo->clear();

    int preferredIndex = -1;
    int firstDetectorIndex = -1;
    for(const CollisionDetectorListItemView& view : items) {
        m_detectorCombo->addItem(view.text, view.id);
        const int index = m_detectorCombo->count() - 1;
        m_detectorCombo->setItemData(index, view.tooltip, Qt::ToolTipRole);
        if(!view.id.isEmpty() && firstDetectorIndex < 0) {
            firstDetectorIndex = index;
        }
        if(!view.id.isEmpty() && view.id == preferredId) {
            preferredIndex = index;
        }
    }

    const int currentIndex = preferredIndex >= 0
        ? preferredIndex
        : firstDetectorIndex;
    if(currentIndex >= 0) {
        m_detectorCombo->setCurrentIndex(currentIndex);
    }
}

void CollisionDetectorsWidget::setProperties(const CollisionDetectorPropertiesView& view)
{
    setDraft(view, false);
}

void CollisionDetectorsWidget::setPairs(const CollisionDetectorPairsViewModel& view)
{
    if(m_pairSummaryLabel != nullptr) {
        m_pairSummaryLabel->clear();
        m_pairSummaryLabel->hide();
    }
    if(m_pairList == nullptr) {
        return;
    }

    const QListWidgetItem* previousCurrent = m_pairList->currentItem();
    const int previousGeneratorIndex = previousCurrent != nullptr
        ? previousCurrent->data(kPairGeneratorIndexRole).toInt()
        : -1;
    const bool previousClearsScope = previousCurrent != nullptr &&
        previousCurrent->data(kPairClearsScopeRole).toBool();
    const QString previousText = previousCurrent != nullptr ? previousCurrent->text() : QString();

    QSignalBlocker blocker(m_pairList);
    m_pairList->clear();
    QListWidgetItem* restoredCurrent = nullptr;
    for(const CollisionDetectorPairItemView& pair : view.items) {
        const QString text = QStringLiteral("%1. %2  <->  %3")
            .arg(pair.index >= 0 ? pair.index + 1 : 0)
            .arg(pair.bodyA)
            .arg(pair.bodyB);
        const QString tooltip = pair.tooltip.isEmpty()
            ? QStringLiteral("A: %1\nB: %2\nSource: %3\nExpansion: %4\nFilter: %5")
                .arg(pair.bodyA, pair.bodyB, pair.source, pair.expansion, pair.filterState)
            : pair.tooltip;
        auto* item = new QListWidgetItem(text, m_pairList);
        item->setToolTip(tooltip);
        item->setData(kPairGeneratorIndexRole, pair.generatorIndex);
        item->setData(kPairRemovableRole, pair.removable);
        item->setData(kPairClearsScopeRole, pair.clearsScope);
        item->setData(kPairRobotAIdRole, pair.robotAId);
        item->setData(kPairLinkANameRole, pair.linkAName);
        item->setData(kPairObjectAIdRole, pair.objectAId);
        item->setData(kPairAttachmentAIdRole, pair.attachmentAId);
        item->setData(kPairRobotBIdRole, pair.robotBId);
        item->setData(kPairLinkBNameRole, pair.linkBName);
        item->setData(kPairObjectBIdRole, pair.objectBId);
        item->setData(kPairAttachmentBIdRole, pair.attachmentBId);
        if(!pair.enabled) {
            item->setFlags(item->flags() & ~(Qt::ItemIsEnabled | Qt::ItemIsSelectable));
        }
        const bool generatorMatches = previousGeneratorIndex >= 0 &&
            pair.generatorIndex == previousGeneratorIndex;
        const bool clearsScopeMatches = previousClearsScope && pair.clearsScope;
        const bool textMatches = !previousText.isEmpty() && text == previousText;
        if(restoredCurrent == nullptr && (generatorMatches || clearsScopeMatches || textMatches)) {
            restoredCurrent = item;
        }
    }
    if(view.items.isEmpty()) {
        auto* item = new QListWidgetItem("No explicit pair scope for this detector.", m_pairList);
        item->setFlags(item->flags() & ~(Qt::ItemIsEnabled | Qt::ItemIsSelectable));
    }
    if(restoredCurrent != nullptr) {
        m_pairList->setCurrentItem(restoredCurrent, QItemSelectionModel::ClearAndSelect);
    }
}

CollisionDetectorPropertiesView CollisionDetectorsWidget::currentProperties() const
{
    CollisionDetectorPropertiesView view = m_properties;
    view.hasDetector = !currentDetectorId().isEmpty();
    view.id = view.id.isEmpty() ? currentDetectorId() : view.id;
    return view;
}

CollisionDetectorQueryContractView CollisionDetectorsWidget::currentQueryContract() const
{
    return makeQueryContract(currentProperties(), currentDetectorId());
}

QString CollisionDetectorsWidget::currentDetectorId() const
{
    return detectorIdForCurrentIndex();
}

QVector<QString> CollisionDetectorsWidget::selectedDetectorIds() const
{
    QVector<QString> detectorIds;
    const QString detectorId = currentDetectorId();
    if(!detectorId.isEmpty()) {
        detectorIds.push_back(detectorId);
    }
    return detectorIds;
}

void CollisionDetectorsWidget::selectDetector(const QString& detectorId)
{
    if(m_detectorCombo == nullptr || detectorId.isEmpty()) {
        return;
    }

    const int index = m_detectorCombo->findData(detectorId);
    if(index >= 0) {
        QSignalBlocker blocker(m_detectorCombo);
        m_detectorCombo->setCurrentIndex(index);
    }
}

void CollisionDetectorsWidget::setDetectorActionsEnabled(bool canAdd, bool hasDetector, bool canRemove)
{
    m_canAdd = canAdd;
    m_hasDetector = hasDetector;
    m_canRemove = canRemove;
    updateActionState();
}

void CollisionDetectorsWidget::setLinkPairActionsEnabled(bool canMarkLinkA, bool canCreateLinkLink)
{
    Q_UNUSED(canMarkLinkA);
    Q_UNUSED(canCreateLinkLink);
}

bool CollisionDetectorsWidget::setCurrentRole(const QString& role)
{
    if(!isKnownRole(role)) {
        return false;
    }

    m_properties.role = role;
    m_savedProperties.role = role;
    updateSummary();
    emit editDetectorRequested();
    return true;
}

void CollisionDetectorsWidget::addDraftSetMember(
    const QString& side,
    const CollisionDetectorDraftMemberView& member)
{
    QListWidget* list = listForSide(side);
    if(list == nullptr || !member.enabled) {
        return;
    }

    const QString key = draftMemberKey(member);
    for(int row = 0; row < list->count(); ++row) {
        if(list->item(row)->data(kDraftMemberKeyRole).toString() == key) {
            list->setCurrentRow(row);
            updateDraftActionState();
            return;
        }
    }

    auto* item = new QListWidgetItem(member.text, list);
    item->setToolTip(member.tooltip.isEmpty() ? member.text : member.tooltip);
    item->setData(kDraftMemberRobotRole, member.robotId);
    item->setData(kDraftMemberLinkRole, member.linkName);
    item->setData(kDraftMemberObjectRole, member.objectId);
    item->setData(kDraftMemberAttachmentRole, member.attachmentId);
    item->setData(kDraftMemberKeyRole, key);
    list->setCurrentItem(item);
    updateDraftActionState();
}

void CollisionDetectorsWidget::clearDraftPairBuilder()
{
    if(m_setAList != nullptr) {
        m_setAList->clear();
    }
    if(m_setBList != nullptr) {
        m_setBList->clear();
    }
    updateDraftActionState();
}

QVector<CollisionDetectorDraftMemberView> CollisionDetectorsWidget::draftSetMembers(const QString& side) const
{
    return membersForList(listForSide(side));
}

QString CollisionDetectorsWidget::detectorIdForCurrentIndex() const
{
    if(m_detectorCombo == nullptr) {
        return QString();
    }

    const QString currentId = m_detectorCombo->currentData().toString();
    if(!currentId.isEmpty()) {
        return currentId;
    }

    for(int index = 0; index < m_detectorCombo->count(); ++index) {
        const QString detectorId = m_detectorCombo->itemData(index).toString();
        if(!detectorId.isEmpty()) {
            return detectorId;
        }
    }

    return QString();
}

void CollisionDetectorsWidget::openQueryDialog()
{
    if(!m_hasDetector) {
        return;
    }

    CollisionDetectorQueryDialog dialog(currentQueryContract(), this);
    if(dialog.exec() != QDialog::Accepted) {
        return;
    }

    setDraft(applyQueryContractToProperties(m_properties, dialog.contract()), true);
    emit editDetectorRequested();
}

void CollisionDetectorsWidget::resetDraft()
{
    setDraft(m_savedProperties, false);
    clearDraftPairBuilder();
}

void CollisionDetectorsWidget::setDraft(const CollisionDetectorPropertiesView& view, bool dirty)
{
    m_properties = view;
    if(!dirty) {
        m_savedProperties = view;
    }
    m_dirty = dirty;
    updateSummary();
    updateActionState();
}

void CollisionDetectorsWidget::updateSummary()
{
    if(m_summaryLabel == nullptr) {
        return;
    }

    if(!m_properties.hasDetector && currentDetectorId().isEmpty()) {
        m_summaryLabel->setText("No detector is available.");
        return;
    }

    const QString name = m_properties.name.trimmed().isEmpty()
        ? QStringLiteral("<unnamed>")
        : m_properties.name.trimmed();
    const QString dirtyText = m_dirty ? QStringLiteral(" | draft not applied") : QString();
    const QString detectorId = currentDetectorId();
    m_summaryLabel->setText(QStringLiteral(
        "%1%2\n"
        "id: %3 | role: %4\n"
        "contacts: %5 | normals: %6 | nearest/distance: %7 | max contacts: %8 | threshold: %9")
            .arg(name)
            .arg(dirtyText)
            .arg(detectorId.isEmpty() ? QStringLiteral("<none>") : detectorId)
            .arg(m_properties.role)
            .arg(yesNo(m_properties.contacts))
            .arg(yesNo(m_properties.normals))
            .arg(yesNo(m_properties.nearest))
            .arg(QString::number(m_properties.maxContacts, 'f', 0))
            .arg(QString::number(m_properties.distanceThreshold, 'f', 4)));
}

void CollisionDetectorsWidget::updateActionState()
{
    if(m_detectorCombo != nullptr) {
        m_detectorCombo->setEnabled(m_hasDetector);
    }
    if(m_configureButton != nullptr) {
        m_configureButton->setEnabled(m_hasDetector);
    }
    if(m_addDetectorButton != nullptr) {
        m_addDetectorButton->setEnabled(m_canAdd);
    }
    if(m_showSelectedButton != nullptr) {
        m_showSelectedButton->setEnabled(m_hasDetector);
    }
    if(m_removeDetectorButton != nullptr) {
        m_removeDetectorButton->setEnabled(m_canRemove);
    }
    updateDraftActionState();
}

QListWidget* CollisionDetectorsWidget::listForSide(const QString& side) const
{
    return side.compare(QStringLiteral("A"), Qt::CaseInsensitive) == 0 ? m_setAList : m_setBList;
}

QVector<CollisionDetectorDraftMemberView> CollisionDetectorsWidget::membersForList(const QListWidget* list) const
{
    QVector<CollisionDetectorDraftMemberView> members;
    if(list == nullptr) {
        return members;
    }
    for(int row = 0; row < list->count(); ++row) {
        const QListWidgetItem* item = list->item(row);
        if(item == nullptr) {
            continue;
        }
        CollisionDetectorDraftMemberView member;
        member.text = item->text();
        member.tooltip = item->toolTip();
        member.robotId = item->data(kDraftMemberRobotRole).toString();
        member.linkName = item->data(kDraftMemberLinkRole).toString();
        member.objectId = item->data(kDraftMemberObjectRole).toString();
        member.attachmentId = item->data(kDraftMemberAttachmentRole).toString();
        member.enabled = true;
        members.push_back(std::move(member));
    }
    return members;
}

void CollisionDetectorsWidget::removeSelectedDraftMembers(QListWidget* list)
{
    if(list == nullptr) {
        return;
    }

    const QList<QListWidgetItem*> selectedItems = list->selectedItems();
    for(QListWidgetItem* item : selectedItems) {
        delete list->takeItem(list->row(item));
    }
    updateDraftActionState();
}

void CollisionDetectorsWidget::clearDraftMembers(QListWidget* list)
{
    if(list != nullptr) {
        list->clear();
    }
    updateDraftActionState();
}

void CollisionDetectorsWidget::showDraftSetContextMenu(QListWidget* list, const QPoint& pos)
{
    if(list == nullptr || !m_hasDetector) {
        return;
    }

    QMenu menu(this);
    QAction* removeAction = menu.addAction("Remove Selected");
    removeAction->setEnabled(!list->selectedItems().isEmpty());
    QAction* clearAction = menu.addAction("Clear All");
    clearAction->setEnabled(list->count() > 0);

    QAction* selectedAction = menu.exec(list->viewport()->mapToGlobal(pos));
    if(selectedAction == removeAction) {
        removeSelectedDraftMembers(list);
    } else if(selectedAction == clearAction) {
        clearDraftMembers(list);
    }
}

void CollisionDetectorsWidget::showPairScopeContextMenu(const QPoint& pos)
{
    if(m_pairList == nullptr || !m_hasDetector) {
        return;
    }

    const QVector<int> removableGeneratorIndexes = selectedPairGeneratorIndexes();
    const QVector<int> allGeneratorIndexes = allPairGeneratorIndexes();
    const bool selectedClearsScope = selectedPairScopeClearsScope();
    const bool hasClearableRows = pairScopeHasClearableRows();
    QMenu menu(this);
    QAction* removeAction = menu.addAction("Remove Selected");
    removeAction->setEnabled(!removableGeneratorIndexes.isEmpty() || selectedClearsScope);
    QAction* clearAction = menu.addAction("Clear All");
    clearAction->setEnabled(!allGeneratorIndexes.isEmpty() || hasClearableRows);
    QAction* sortAction = menu.addAction("Sort Ascending");
    sortAction->setEnabled(m_pairList->count() > 1);

    QAction* selectedAction = menu.exec(m_pairList->viewport()->mapToGlobal(pos));
    if(selectedAction == removeAction) {
        if(selectedClearsScope) {
            emit clearPairScopeRequested();
        } else {
            emit removePairGeneratorsRequested(removableGeneratorIndexes);
        }
    } else if(selectedAction == clearAction) {
        emit clearPairScopeRequested();
    } else if(selectedAction == sortAction) {
        m_pairList->sortItems(Qt::AscendingOrder);
    }
}

QVector<int> CollisionDetectorsWidget::selectedPairGeneratorIndexes() const
{
    QVector<int> indexes;
    if(m_pairList == nullptr) {
        return indexes;
    }

    const QList<QListWidgetItem*> selectedItems = m_pairList->selectedItems();
    for(const QListWidgetItem* item : selectedItems) {
        if(item == nullptr || !item->data(kPairRemovableRole).toBool()) {
            continue;
        }
        const int generatorIndex = item->data(kPairGeneratorIndexRole).toInt();
        if(generatorIndex >= 0 && !indexes.contains(generatorIndex)) {
            indexes.push_back(generatorIndex);
        }
    }
    std::sort(indexes.begin(), indexes.end());
    return indexes;
}

QVector<int> CollisionDetectorsWidget::allPairGeneratorIndexes() const
{
    QVector<int> indexes;
    if(m_pairList == nullptr) {
        return indexes;
    }

    for(int row = 0; row < m_pairList->count(); ++row) {
        const QListWidgetItem* item = m_pairList->item(row);
        if(item == nullptr || !item->data(kPairRemovableRole).toBool()) {
            continue;
        }
        const int generatorIndex = item->data(kPairGeneratorIndexRole).toInt();
        if(generatorIndex >= 0 && !indexes.contains(generatorIndex)) {
            indexes.push_back(generatorIndex);
        }
    }
    std::sort(indexes.begin(), indexes.end());
    return indexes;
}

bool CollisionDetectorsWidget::selectedPairScopeClearsScope() const
{
    if(m_pairList == nullptr) {
        return false;
    }

    const QList<QListWidgetItem*> selectedItems = m_pairList->selectedItems();
    return std::any_of(
        selectedItems.begin(),
        selectedItems.end(),
        [](const QListWidgetItem* item) {
            return item != nullptr && item->data(kPairClearsScopeRole).toBool();
        });
}

bool CollisionDetectorsWidget::pairScopeHasClearableRows() const
{
    if(m_pairList == nullptr) {
        return false;
    }

    for(int row = 0; row < m_pairList->count(); ++row) {
        const QListWidgetItem* item = m_pairList->item(row);
        if(item != nullptr &&
            (item->data(kPairRemovableRole).toBool() || item->data(kPairClearsScopeRole).toBool())) {
            return true;
        }
    }
    return false;
}

void CollisionDetectorsWidget::previewPairScopeItem(const QListWidgetItem* item)
{
    if(item == nullptr) {
        emit pairScopePreviewRequested(
            QString(),
            QString(),
            QString(),
            QString(),
            QString(),
            QString(),
            QString(),
            QString());
        return;
    }

    emit pairScopePreviewRequested(
        item->data(kPairRobotAIdRole).toString(),
        item->data(kPairLinkANameRole).toString(),
        item->data(kPairObjectAIdRole).toString(),
        item->data(kPairAttachmentAIdRole).toString(),
        item->data(kPairRobotBIdRole).toString(),
        item->data(kPairLinkBNameRole).toString(),
        item->data(kPairObjectBIdRole).toString(),
        item->data(kPairAttachmentBIdRole).toString());
}

void CollisionDetectorsWidget::previewDraftMember(const QListWidgetItem* item)
{
    if(item == nullptr) {
        return;
    }

    emit draftMemberPreviewRequested(
        item->data(kDraftMemberRobotRole).toString(),
        item->data(kDraftMemberLinkRole).toString(),
        item->data(kDraftMemberObjectRole).toString(),
        item->data(kDraftMemberAttachmentRole).toString());
}

QString CollisionDetectorsWidget::draftMemberKey(const CollisionDetectorDraftMemberView& member) const
{
    if(!member.attachmentId.isEmpty()) {
        return QStringLiteral("attachment:%1").arg(member.attachmentId);
    }
    if(!member.objectId.isEmpty()) {
        return QStringLiteral("object:%1").arg(member.objectId);
    }
    if(!member.robotId.isEmpty()) {
        return QStringLiteral("robot:%1:%2").arg(member.robotId, member.linkName);
    }
    return member.text;
}

void CollisionDetectorsWidget::updateDraftActionState()
{
    const bool hasA = m_setAList != nullptr && m_setAList->count() > 0;
    const bool hasB = m_setBList != nullptr && m_setBList->count() > 0;
    if(m_bindDraftSetsButton != nullptr) {
        m_bindDraftSetsButton->setEnabled(m_hasDetector && hasA && (hasB || m_setAList->count() > 1));
    }
}
