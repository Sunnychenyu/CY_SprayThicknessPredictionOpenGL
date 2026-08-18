#include "CollisionLinkModelsWidget.h"

#include "RobotQtWidgetUtils.h"

#include <QAbstractItemView>
#include <QFrame>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QSignalBlocker>
#include <QStringList>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVBoxLayout>

namespace
{
    using robot_qt_viewer::configureInspectorList;
    using robot_qt_viewer::makeHorizontallyCompressible;
    using robot_qt_viewer::makePanelTitle;

    constexpr int kVariantRoleRole = Qt::UserRole;
    constexpr int kVariantSourceRole = Qt::UserRole + 1;
    constexpr int kVariantIdRole = Qt::UserRole + 2;
    constexpr int kVariantCurrentRole = Qt::UserRole + 3;
    constexpr int kVariantComplexityRowsRole = Qt::UserRole + 4;

    QLabel* makeValueLabel(QWidget* parent)
    {
        auto* label = new QLabel(parent);
        label->setWordWrap(true);
        label->setTextInteractionFlags(Qt::TextSelectableByMouse);
        makeHorizontallyCompressible(label);
        return label;
    }

    QFrame* makeSection(QWidget* parent, const QString& title, QVBoxLayout** contentLayout)
    {
        auto* frame = new QFrame(parent);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Plain);
        auto* layout = new QVBoxLayout(frame);
        layout->setContentsMargins(8, 5, 8, 7);
        layout->setSpacing(5);
        layout->addWidget(makePanelTitle(title, frame));
        *contentLayout = layout;
        return frame;
    }

    QString plainTextToHtml(const QString& text)
    {
        QString html = text.toHtmlEscaped();
        html.replace("\n", "<br/>");
        return html;
    }

    QStringList metricRowsToStringList(const QVector<CollisionLinkModelMetricRowView>& rows)
    {
        QStringList result;
        for(const CollisionLinkModelMetricRowView& row : rows) {
            result << row.label << row.value;
        }
        return result;
    }
}

CollisionLinkModelsWidget::CollisionLinkModelsWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(6);

    QVBoxLayout* targetSectionLayout = nullptr;
    QFrame* targetSection = makeSection(this, "Current Tree Node", &targetSectionLayout);
    m_targetLabel = makeValueLabel(targetSection);
    targetSectionLayout->addWidget(m_targetLabel);
    targetSection->setMaximumHeight(86);
    layout->addWidget(targetSection);

    QVBoxLayout* variantsSectionLayout = nullptr;
    QFrame* variantsSection = makeSection(this, "Variants", &variantsSectionLayout);
    m_variantList = new QListWidget(this);
    configureInspectorList(m_variantList);
    m_variantList->setAlternatingRowColors(true);
    m_variantList->setMinimumHeight(112);
    m_variantList->setMaximumHeight(190);
    connect(m_variantList, &QListWidget::currentItemChanged, this, [this](QListWidgetItem*, QListWidgetItem*) {
        updateActionButtonsEnabled();
        updateSelectedVariantSummary();
        emit variantSelectionChanged();
    });
    variantsSectionLayout->addWidget(m_variantList, 1);

    m_setCurrentVariantButton = new QPushButton("Set Current", this);
    makeHorizontallyCompressible(m_setCurrentVariantButton);
    m_setCurrentVariantButton->setToolTip("Sets the selected collision model variant as the current project model.");
    m_setCurrentVariantButton->setEnabled(false);
    connect(m_setCurrentVariantButton, &QPushButton::clicked,
        this, &CollisionLinkModelsWidget::setCurrentVariantRequested);
    variantsSectionLayout->addWidget(m_setCurrentVariantButton);

    m_statusLabel = makeValueLabel(variantsSection);
    m_statusLabel->setTextFormat(Qt::RichText);
    variantsSectionLayout->addWidget(m_statusLabel);
    layout->addWidget(variantsSection);

    QVBoxLayout* generateSectionLayout = nullptr;
    QFrame* generateSection =
        makeSection(this, "Generate Simplified Collision Model", &generateSectionLayout);
    m_generateCoacdButton = new QPushButton("Generate Simplified Model with COACD", this);
    makeHorizontallyCompressible(m_generateCoacdButton);
    m_generateCoacdButton->setToolTip(
        "Generates a simplified collision model variant from the selected model.");
    m_generateCoacdButton->setEnabled(false);
    connect(m_generateCoacdButton, &QPushButton::clicked,
        this, &CollisionLinkModelsWidget::generateCoacdRequested);
    generateSectionLayout->addWidget(m_generateCoacdButton);
    layout->addWidget(generateSection);

    m_complexityTitleLabel = makePanelTitle("Collision Model Complexity Analysis", this);
    layout->addWidget(m_complexityTitleLabel);

    m_complexityTable = new QTableWidget(0, 2, this);
    m_complexityTable->setHorizontalHeaderLabels(QStringList() << "Metric" << "Value");
    m_complexityTable->verticalHeader()->hide();
    m_complexityTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    m_complexityTable->setSelectionMode(QAbstractItemView::NoSelection);
    m_complexityTable->setFocusPolicy(Qt::NoFocus);
    m_complexityTable->setAlternatingRowColors(true);
    m_complexityTable->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_complexityTable->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_complexityTable->setShowGrid(true);
    m_complexityTable->setMinimumHeight(218);
    m_complexityTable->setMaximumHeight(218);
    m_complexityTable->setSizeAdjustPolicy(QAbstractScrollArea::AdjustIgnored);
    m_complexityTable->verticalHeader()->setDefaultSectionSize(28);
    m_complexityTable->verticalHeader()->setMinimumSectionSize(24);
    m_complexityTable->verticalHeader()->setFixedWidth(0);
    m_complexityTable->horizontalHeader()->setMinimumSectionSize(56);
    m_complexityTable->horizontalHeader()->setStretchLastSection(true);
    m_complexityTable->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Interactive);
    m_complexityTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Interactive);
    layout->addWidget(m_complexityTable);

    auto* exitLayout = new QHBoxLayout();
    exitLayout->setContentsMargins(0, 0, 0, 0);
    exitLayout->setSpacing(8);
    m_applyButton = new QPushButton("Apply", this);
    makeHorizontallyCompressible(m_applyButton);
    m_applyButton->setToolTip("Leaves collision model configuration and returns to detector configuration.");
    m_applyButton->setEnabled(false);
    connect(m_applyButton, &QPushButton::clicked,
        this, &CollisionLinkModelsWidget::applyConfigurationRequested);
    exitLayout->addWidget(m_applyButton);

    m_cancelButton = new QPushButton("Cancel", this);
    makeHorizontallyCompressible(m_cancelButton);
    m_cancelButton->setToolTip("Cancels the current preview and returns to detector configuration.");
    m_cancelButton->setEnabled(false);
    connect(m_cancelButton, &QPushButton::clicked,
        this, &CollisionLinkModelsWidget::cancelConfigurationRequested);
    exitLayout->addWidget(m_cancelButton);
    layout->addLayout(exitLayout);
    layout->addStretch(1);
}

void CollisionLinkModelsWidget::setViewModel(const CollisionLinkModelsViewModel& viewModel)
{
    setSummary(viewModel.summary);

    if(m_variantList == nullptr) {
        return;
    }

    QSignalBlocker variantBlocker(m_variantList);
    m_variantList->clear();

    QListWidgetItem* selectedVariant = nullptr;
    QListWidgetItem* currentVariant = nullptr;
    for(const CollisionLinkModelVariantItemView& variant : viewModel.variants) {
        const QString marker = variant.current ? "[x]" : "[ ]";
        const QString text = QString("%1 %2    %3    %4")
            .arg(marker)
            .arg(variant.label)
            .arg(variant.typeLabel.isEmpty() ? QString("-") : variant.typeLabel)
            .arg(variant.roleLabel.isEmpty() ? QString("-") : variant.roleLabel);

        auto* item = new QListWidgetItem(text, m_variantList);
        item->setToolTip(variant.tooltip.isEmpty() ? variant.detail : variant.tooltip);
        item->setData(kVariantRoleRole, variant.role);
        item->setData(kVariantSourceRole, variant.source);
        item->setData(kVariantIdRole, variant.variantId);
        item->setData(kVariantCurrentRole, variant.current);
        item->setData(kVariantComplexityRowsRole, metricRowsToStringList(variant.complexityRows));
        if(!variant.enabled) {
            item->setFlags(item->flags() & ~(Qt::ItemIsEnabled | Qt::ItemIsSelectable));
        }
        if(variant.current) {
            currentVariant = item;
        }
        if(variant.selected) {
            selectedVariant = item;
        }
    }

    if(selectedVariant == nullptr) {
        selectedVariant = currentVariant;
    }
    if(selectedVariant == nullptr && m_variantList->count() > 0) {
        selectedVariant = m_variantList->item(0);
    }
    if(selectedVariant != nullptr) {
        m_variantList->setCurrentItem(selectedVariant);
    }
    updateActionButtonsEnabled();
    updateSelectedVariantSummary();
}

void CollisionLinkModelsWidget::setSummary(const CollisionLinkModelsSummaryView& summary)
{
    if(m_targetLabel != nullptr) {
        m_targetLabel->setText(summary.target.displayName.isEmpty()
            ? QString("No tree node selected")
            : summary.target.displayName);
    }
    if(m_statusLabel != nullptr) {
        m_baseStatusText = summary.statusText;
        updateSelectedVariantSummary();
    }
}

QString CollisionLinkModelsWidget::currentVariantId() const
{
    const QListWidgetItem* item = m_variantList != nullptr ? m_variantList->currentItem() : nullptr;
    return item != nullptr ? item->data(kVariantIdRole).toString() : QString();
}

QString CollisionLinkModelsWidget::currentVariantRole() const
{
    const QListWidgetItem* item = m_variantList != nullptr ? m_variantList->currentItem() : nullptr;
    return item != nullptr ? item->data(kVariantRoleRole).toString() : QString();
}

QString CollisionLinkModelsWidget::currentVariantSource() const
{
    const QListWidgetItem* item = m_variantList != nullptr ? m_variantList->currentItem() : nullptr;
    return item != nullptr ? item->data(kVariantSourceRole).toString() : QString();
}

QString CollisionLinkModelsWidget::appliedVariantId() const
{
    if(m_variantList == nullptr) {
        return QString();
    }

    for(int row = 0; row < m_variantList->count(); ++row) {
        const QListWidgetItem* item = m_variantList->item(row);
        if(item != nullptr && item->data(kVariantCurrentRole).toBool()) {
            return item->data(kVariantIdRole).toString();
        }
    }
    return QString();
}

QString CollisionLinkModelsWidget::appliedVariantSource() const
{
    if(m_variantList == nullptr) {
        return QString();
    }

    for(int row = 0; row < m_variantList->count(); ++row) {
        const QListWidgetItem* item = m_variantList->item(row);
        if(item != nullptr && item->data(kVariantCurrentRole).toBool()) {
            return item->data(kVariantSourceRole).toString();
        }
    }
    return QString();
}

QString CollisionLinkModelsWidget::currentElementId() const
{
    return QString();
}

bool CollisionLinkModelsWidget::hasCurrentVariant() const
{
    return !currentVariantId().isEmpty();
}

bool CollisionLinkModelsWidget::selectAppliedVariant()
{
    if(m_variantList == nullptr) {
        return false;
    }

    for(int row = 0; row < m_variantList->count(); ++row) {
        QListWidgetItem* item = m_variantList->item(row);
        if(item != nullptr && item->data(kVariantCurrentRole).toBool()) {
            m_variantList->setCurrentItem(item);
            return true;
        }
    }
    return false;
}

bool CollisionLinkModelsWidget::selectVariantBySourceRole(const QString& source, const QString& role)
{
    if(m_variantList == nullptr) {
        return false;
    }

    for(int row = 0; row < m_variantList->count(); ++row) {
        QListWidgetItem* item = m_variantList->item(row);
        if(item == nullptr) {
            continue;
        }
        if(item->data(kVariantSourceRole).toString() == source &&
            item->data(kVariantRoleRole).toString() == role) {
            m_variantList->setCurrentItem(item);
            return true;
        }
    }
    return false;
}

robot_qt_viewer::CollisionRuntimeProxyRequest CollisionLinkModelsWidget::proxyRequest(
    const QString& proxyType,
    const std::string& fallbackRole,
    bool useExistingCollisionAsInput) const
{
    robot_qt_viewer::CollisionRuntimeProxyRequest request;
    request.proxyType = proxyType.toStdString();
    request.role = fallbackRole;
    request.useExistingCollisionAsInput = useExistingCollisionAsInput;
    return request;
}

bool CollisionLinkModelsWidget::replaceOriginal() const
{
    return false;
}

void CollisionLinkModelsWidget::setContextActionsEnabled(bool hasRobot, bool hasLink)
{
    (void)hasRobot;
    if(hasLink) {
        m_variantActionsAllowed = true;
    }
    updateActionButtonsEnabled();
}

void CollisionLinkModelsWidget::setVariantActionsEnabled(bool canUseVariant, bool canShowVariant)
{
    (void)canUseVariant;
    m_variantActionsAllowed = canShowVariant;
    updateActionButtonsEnabled();
}

void CollisionLinkModelsWidget::setTaskExitEnabled(bool enabled)
{
    m_taskExitAllowed = enabled;
    updateActionButtonsEnabled();
}

bool CollisionLinkModelsWidget::selectedVariantIsCurrent() const
{
    const QListWidgetItem* item = m_variantList != nullptr ? m_variantList->currentItem() : nullptr;
    return item != nullptr && item->data(kVariantCurrentRole).toBool();
}

void CollisionLinkModelsWidget::updateSelectedVariantSummary()
{
    if(m_statusLabel == nullptr) {
        return;
    }

    QString text;
    if(!m_baseStatusText.isEmpty()) {
        text = plainTextToHtml(m_baseStatusText);
    }

    m_statusLabel->setText(text);

    const QListWidgetItem* item = m_variantList != nullptr ? m_variantList->currentItem() : nullptr;
    updateComplexityTable(item != nullptr
        ? item->data(kVariantComplexityRowsRole).toStringList()
        : QStringList());
}

void CollisionLinkModelsWidget::updateComplexityTable(const QStringList& rows)
{
    if(m_complexityTable == nullptr) {
        return;
    }

    const int rowCount = rows.size() / 2;
    m_complexityTable->setRowCount(rowCount);
    for(int row = 0; row < rowCount; ++row) {
        auto* labelItem = new QTableWidgetItem(rows.value(row * 2));
        auto* valueItem = new QTableWidgetItem(rows.value(row * 2 + 1));
        labelItem->setFlags(labelItem->flags() & ~Qt::ItemIsEditable);
        valueItem->setFlags(valueItem->flags() & ~Qt::ItemIsEditable);
        valueItem->setTextAlignment(Qt::AlignRight | Qt::AlignVCenter);
        m_complexityTable->setItem(row, 0, labelItem);
        m_complexityTable->setItem(row, 1, valueItem);
    }

    m_complexityTable->setVisible(rowCount > 0);
    if(m_complexityTitleLabel != nullptr) {
        m_complexityTitleLabel->setVisible(rowCount > 0);
    }
    QTimer::singleShot(0, this, &CollisionLinkModelsWidget::resetComplexityTableColumnWidths);
}

void CollisionLinkModelsWidget::resetComplexityTableColumnWidths()
{
    if(m_complexityTable == nullptr || !m_complexityTable->isVisible()) {
        return;
    }

    const int width = m_complexityTable->viewport()->width();
    if(width <= 0) {
        return;
    }

    const int firstColumnWidth = width / 2;
    m_complexityTable->horizontalHeader()->resizeSection(0, firstColumnWidth);
    m_complexityTable->horizontalHeader()->resizeSection(1, width - firstColumnWidth);
}

void CollisionLinkModelsWidget::updateActionButtonsEnabled()
{
    if(m_setCurrentVariantButton != nullptr) {
        m_setCurrentVariantButton->setEnabled(
            m_variantActionsAllowed &&
            hasCurrentVariant() &&
            !selectedVariantIsCurrent());
    }
    if(m_generateCoacdButton != nullptr) {
        m_generateCoacdButton->setEnabled(m_variantActionsAllowed);
    }
    if(m_applyButton != nullptr) {
        m_applyButton->setEnabled(m_taskExitAllowed);
    }
    if(m_cancelButton != nullptr) {
        m_cancelButton->setEnabled(m_taskExitAllowed);
    }
}
