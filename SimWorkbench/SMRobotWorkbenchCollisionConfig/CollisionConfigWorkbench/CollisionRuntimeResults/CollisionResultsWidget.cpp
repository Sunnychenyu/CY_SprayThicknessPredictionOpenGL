#include "CollisionResultsWidget.h"

#include "CollisionResultsViewModel.h"
#include "RobotQtWidgetUtils.h"

#include <QAbstractItemView>
#include <QFontMetrics>
#include <QHeaderView>
#include <QLabel>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QSplitter>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <initializer_list>

namespace
{
    using robot_qt_viewer::makeHorizontallyCompressible;
    using robot_qt_viewer::makePanelTitle;

    int stableTableWidth(const std::initializer_list<int> widths)
    {
        int total = 8;
        for(const int width : widths) {
            total += width;
        }
        return total;
    }

    int stableRowHeight(const QTableWidget* table)
    {
        if(table == nullptr) {
            return 28;
        }
        const QFontMetrics metrics(table->font());
        return std::max(metrics.height() + 12, 30);
    }

    void applyInitialColumns(QTableWidget* table, const std::initializer_list<int> widths)
    {
        if(table == nullptr) {
            return;
        }

        table->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
        table->horizontalHeader()->setStretchLastSection(true);

        int column = 0;
        for(const int width : widths) {
            if(column < table->columnCount()) {
                table->setColumnWidth(column, width);
            }
            ++column;
        }

        const int width = stableTableWidth(widths);
        table->setMinimumWidth(width);
    }

    void configureTable(QTableWidget* table)
    {
        if(table == nullptr) {
            return;
        }
        makeHorizontallyCompressible(table);
        table->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        table->setTextElideMode(Qt::ElideNone);
        table->horizontalHeader()->setMinimumSectionSize(36);
        table->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
        table->horizontalHeader()->setStretchLastSection(true);
        table->setEditTriggers(QAbstractItemView::NoEditTriggers);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->verticalHeader()->setVisible(false);
        table->verticalHeader()->setDefaultSectionSize(stableRowHeight(table));
        table->horizontalHeader()->setDefaultSectionSize(72);
        table->setShowGrid(false);
        table->setAlternatingRowColors(true);
        table->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    }

    QTableWidgetItem* makeTableItem(const QString& text, Qt::Alignment alignment = Qt::AlignLeft | Qt::AlignVCenter)
    {
        auto* item = new QTableWidgetItem(text);
        item->setToolTip(text);
        item->setTextAlignment(alignment);
        return item;
    }

    QString tableSignature(const CollisionResultsTableView& view)
    {
        QString signature = view.normalColumnVisible ? "normal:on\n" : "normal:off\n";
        for(const CollisionResultsTableRow& row : view.rows) {
            signature += row.spanColumns ? "span|" : "row|";
            signature += row.cells.join('\t');
            signature += '\n';
        }
        return signature;
    }

    QString summarySignature(const QVector<CollisionSummaryRow>& rows)
    {
        QString signature;
        for(const CollisionSummaryRow& row : rows) {
            signature += row.metric;
            signature += '|';
            signature += row.value;
            signature += '|';
            signature += row.share;
            signature += '|';
            signature += row.tooltip;
            signature += '\n';
        }
        return signature;
    }

    QString timingSignature(const QVector<CollisionTimingTableRow>& rows)
    {
        QString signature;
        for(const CollisionTimingTableRow& row : rows) {
            signature += row.stage;
            signature += '|';
            signature += row.last;
            signature += '|';
            signature += row.queryShare;
            signature += '|';
            signature += row.frameShare;
            signature += '|';
            signature += row.state;
            signature += '|';
            signature += row.tooltip;
            signature += '\n';
        }
        return signature;
    }

    QString overlayTimingSignature(const QVector<CollisionOverlayTimingRow>& rows)
    {
        QString signature;
        for(const CollisionOverlayTimingRow& row : rows) {
            signature += row.step;
            signature += '|';
            signature += row.last;
            signature += '|';
            signature += row.tooltip;
            signature += '\n';
        }
        return signature;
    }

    void configureSummaryTable(QTableWidget* table)
    {
        configureTable(table);
        if(table == nullptr) {
            return;
        }
        table->setColumnCount(3);
        table->setHorizontalHeaderLabels({ "metric", "value", "share" });
        table->setSelectionMode(QAbstractItemView::NoSelection);
        table->setFocusPolicy(Qt::NoFocus);
        table->setMinimumHeight(180);
        table->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
        applyInitialColumns(table, { 170, 200, 110 });
    }

    void applySummaryRows(QTableWidget* table, const QVector<CollisionSummaryRow>& rows)
    {
        if(table == nullptr) {
            return;
        }

        QSignalBlocker blocker(table);
        table->setRowCount(rows.size());
        for(int rowIndex = 0; rowIndex < rows.size(); ++rowIndex) {
            const CollisionSummaryRow& row = rows[rowIndex];
            QTableWidgetItem* metricItem = makeTableItem(row.metric);
            metricItem->setToolTip(row.tooltip);
            table->setItem(rowIndex, 0, metricItem);
            QTableWidgetItem* valueItem = makeTableItem(row.value, Qt::AlignRight | Qt::AlignVCenter);
            valueItem->setToolTip(row.tooltip);
            table->setItem(rowIndex, 1, valueItem);
            QTableWidgetItem* shareItem = makeTableItem(row.share, Qt::AlignRight | Qt::AlignVCenter);
            shareItem->setToolTip(row.tooltip);
            table->setItem(rowIndex, 2, shareItem);
            table->setRowHeight(rowIndex, stableRowHeight(table));
        }
    }

    void applyTableView(QTableWidget* table, const CollisionResultsTableView& view)
    {
        if(table == nullptr) {
            return;
        }

        QSignalBlocker blocker(table);
        table->setRowCount(0);
        table->clearSpans();
        table->setColumnHidden(3, !view.normalColumnVisible);
        table->setRowCount(view.rows.size());
        for(int rowIndex = 0; rowIndex < view.rows.size(); ++rowIndex) {
            const CollisionResultsTableRow& row = view.rows[rowIndex];
            if(row.spanColumns) {
                const QString message = row.cells.isEmpty() ? QString() : row.cells.front();
                table->setItem(rowIndex, 0, makeTableItem(message));
                table->setSpan(rowIndex, 0, 1, table->columnCount());
                table->setRowHeight(rowIndex, stableRowHeight(table));
                continue;
            }

            const int columnCount = std::min(table->columnCount(), row.cells.size());
            for(int column = 0; column < columnCount; ++column) {
                const Qt::Alignment alignment =
                    column == 0
                        ? (Qt::AlignLeft | Qt::AlignVCenter)
                        : (Qt::AlignRight | Qt::AlignVCenter);
                table->setItem(rowIndex, column, makeTableItem(row.cells[column], alignment));
            }
            table->setRowHeight(rowIndex, stableRowHeight(table));
        }
    }

    void applyTimingRows(QTableWidget* table, const QVector<CollisionTimingTableRow>& rows)
    {
        if(table == nullptr) {
            return;
        }

        QSignalBlocker blocker(table);
        table->setRowCount(rows.size());
        for(int rowIndex = 0; rowIndex < rows.size(); ++rowIndex) {
            const CollisionTimingTableRow& row = rows[rowIndex];
            QTableWidgetItem* stageItem = makeTableItem(row.stage);
            stageItem->setToolTip(row.tooltip);
            table->setItem(rowIndex, 0, stageItem);
            table->setItem(rowIndex, 1, makeTableItem(row.last, Qt::AlignRight | Qt::AlignVCenter));
            table->setItem(rowIndex, 2, makeTableItem(row.queryShare, Qt::AlignRight | Qt::AlignVCenter));
            table->setItem(rowIndex, 3, makeTableItem(row.frameShare, Qt::AlignRight | Qt::AlignVCenter));
            table->setItem(rowIndex, 4, makeTableItem(row.state, Qt::AlignCenter));
            table->setRowHeight(rowIndex, stableRowHeight(table));
        }
    }

    void applyOverlayTimingRows(QTableWidget* table, const QVector<CollisionOverlayTimingRow>& rows)
    {
        if(table == nullptr) {
            return;
        }

        QSignalBlocker blocker(table);
        table->setRowCount(rows.size());
        for(int rowIndex = 0; rowIndex < rows.size(); ++rowIndex) {
            const CollisionOverlayTimingRow& row = rows[rowIndex];
            QTableWidgetItem* stepItem = makeTableItem(row.step);
            stepItem->setToolTip(row.tooltip);
            table->setItem(rowIndex, 0, stepItem);
            table->setItem(rowIndex, 1, makeTableItem(row.last, Qt::AlignRight | Qt::AlignVCenter));
            table->setRowHeight(rowIndex, stableRowHeight(table));
        }
    }

    QWidget* makeTabPage(QWidget* child, QWidget* parent)
    {
        auto* page = new QWidget(parent);
        auto* layout = new QVBoxLayout(page);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(4);
        layout->addWidget(child, 1);
        return page;
    }
}

CollisionResultsWidget::CollisionResultsWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(4);

    m_splitter = new QSplitter(Qt::Horizontal, this);
    m_splitter->setChildrenCollapsible(false);
    m_splitter->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    m_resultSummaryPane = new QWidget(this);
    auto* resultSummaryLayout = new QVBoxLayout(m_resultSummaryPane);
    resultSummaryLayout->setContentsMargins(0, 0, 0, 0);
    resultSummaryLayout->setSpacing(2);

    m_detailsTitle = makePanelTitle("Result Summary", this);
    m_summaryTable = new QTableWidget(this);
    configureSummaryTable(m_summaryTable);
    resultSummaryLayout->addWidget(m_detailsTitle);
    resultSummaryLayout->addWidget(m_summaryTable, 1);
    m_resultSummaryPane->setMinimumWidth(500);
    m_resultSummaryPane->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    m_splitter->addWidget(m_resultSummaryPane);

    m_resultDetailsPane = new QWidget(this);
    auto* resultDetailsLayout = new QVBoxLayout(m_resultDetailsPane);
    resultDetailsLayout->setContentsMargins(0, 0, 0, 0);
    resultDetailsLayout->setSpacing(4);
    m_resultDetailsTitle = makePanelTitle("Result Details", this);
    resultDetailsLayout->addWidget(m_resultDetailsTitle);

    m_detailsTabs = new QTabWidget(this);
    makeHorizontallyCompressible(m_detailsTabs);
    m_detailsTabs->setDocumentMode(true);
    m_detailsTabs->setTabPosition(QTabWidget::North);
    m_detailsTabs->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    m_timingTable = new QTableWidget(this);
    configureTable(m_timingTable);
    m_timingTable->setColumnCount(5);
    m_timingTable->setHorizontalHeaderLabels({ "stage", "last", "query share", "frame share", "state" });
    m_timingTable->setMinimumHeight(190);
    applyInitialColumns(m_timingTable, { 340, 190, 205, 205, 420 });
    m_detailsTabs->addTab(makeTabPage(m_timingTable, m_detailsTabs), "Timing");

    m_overlayTimingTable = new QTableWidget(this);
    configureTable(m_overlayTimingTable);
    m_overlayTimingTable->setColumnCount(2);
    m_overlayTimingTable->setHorizontalHeaderLabels({ "overlay step", "last" });
    m_overlayTimingTable->setMinimumHeight(112);
    applyInitialColumns(m_overlayTimingTable, { 260, 160 });
    m_overlayTimingTitle = nullptr;
    m_detailsTabs->addTab(makeTabPage(m_overlayTimingTable, m_detailsTabs), "Overlay");

    m_debugLabel = new QLabel(this);
    m_debugLabel->setWordWrap(true);
    m_debugLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_debugLabel->setMinimumHeight(112);
    makeHorizontallyCompressible(m_debugLabel);
    m_debugTitle = nullptr;

    m_contactTable = new QTableWidget(this);
    configureTable(m_contactTable);
    m_contactTable->setColumnCount(5);
    m_contactTable->setHorizontalHeaderLabels({ "body A", "body B", "position", "normal", "depth" });
    m_contactTable->setMinimumHeight(112);
    applyInitialColumns(m_contactTable, { 240, 240, 260, 260, 130 });
    m_contactTitle = nullptr;
    m_detailsTabs->addTab(makeTabPage(m_contactTable, m_detailsTabs), "Contacts");

    m_nearestTable = new QTableWidget(this);
    configureTable(m_nearestTable);
    m_nearestTable->setColumnCount(5);
    m_nearestTable->setHorizontalHeaderLabels({ "body A", "body B", "point A", "point B", "distance" });
    m_nearestTable->setMinimumHeight(112);
    applyInitialColumns(m_nearestTable, { 240, 240, 260, 260, 130 });
    m_nearestTitle = nullptr;
    m_detailsTabs->addTab(makeTabPage(m_nearestTable, m_detailsTabs), "Nearest");
    m_detailsTabs->addTab(makeTabPage(m_debugLabel, m_detailsTabs), "Debug");

    resultDetailsLayout->addWidget(m_detailsTabs, 1);
    m_resultDetailsPane->setMinimumWidth(900);
    m_resultDetailsPane->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    m_splitter->addWidget(m_resultDetailsPane);
    m_splitter->setStretchFactor(0, 0);
    m_splitter->setStretchFactor(1, 1);
    m_splitter->setSizes({ 520, 1400 });
    layout->addWidget(m_splitter, 1);
    applyDisplayMode();
}

void CollisionResultsWidget::setDisplayMode(DisplayMode mode)
{
    if(m_displayMode == mode) {
        return;
    }

    m_displayMode = mode;
    applyDisplayMode();
}

void CollisionResultsWidget::setResults(const CollisionResultsViewModel& viewModel)
{
    const QString summaryRowsSignature = summarySignature(viewModel.summaryRows);
    if(m_lastSummarySignature != summaryRowsSignature) {
        applySummaryRows(m_summaryTable, viewModel.summaryRows);
        m_lastSummarySignature = summaryRowsSignature;
    }
    const QString timingRowsSignature = timingSignature(viewModel.timingRows);
    if(m_lastTimingSignature != timingRowsSignature) {
        applyTimingRows(m_timingTable, viewModel.timingRows);
        m_lastTimingSignature = timingRowsSignature;
    }
    const QString overlayRowsSignature = overlayTimingSignature(viewModel.overlayTimingRows);
    if(m_lastOverlayTimingSignature != overlayRowsSignature) {
        applyOverlayTimingRows(m_overlayTimingTable, viewModel.overlayTimingRows);
        m_lastOverlayTimingSignature = overlayRowsSignature;
    }
    if(m_debugLabel != nullptr && m_lastDebugText != viewModel.debugText) {
        m_debugLabel->setText(viewModel.debugText);
        m_lastDebugText = viewModel.debugText;
    }
    const QString contactSignature = tableSignature(viewModel.contacts);
    if(m_lastContactSignature != contactSignature) {
        applyTableView(m_contactTable, viewModel.contacts);
        m_lastContactSignature = contactSignature;
    }
    const QString nearestSignature = tableSignature(viewModel.nearest);
    if(m_lastNearestSignature != nearestSignature) {
        applyTableView(m_nearestTable, viewModel.nearest);
        m_lastNearestSignature = nearestSignature;
    }
}

void CollisionResultsWidget::applyDisplayMode()
{
    const bool showSummary =
        m_displayMode == DisplayMode::Full || m_displayMode == DisplayMode::SummaryOnly;
    const bool showDetails =
        m_displayMode == DisplayMode::Full || m_displayMode == DisplayMode::DetailsOnly;

    if(m_resultSummaryPane != nullptr) {
        m_resultSummaryPane->setVisible(showSummary);
    }
    if(m_resultDetailsPane != nullptr) {
        m_resultDetailsPane->setVisible(showDetails);
    }
    if(m_detailsTitle != nullptr) {
        m_detailsTitle->setVisible(showSummary);
    }
    if(m_summaryTable != nullptr) {
        m_summaryTable->setVisible(showSummary);
    }
    if(m_resultDetailsTitle != nullptr) {
        m_resultDetailsTitle->setVisible(showDetails);
    }
    if(m_timingTable != nullptr) {
        m_timingTable->setVisible(showDetails);
    }
    if(m_detailsTabs != nullptr) {
        m_detailsTabs->setVisible(showDetails);
    }
    if(m_overlayTimingTitle != nullptr) {
        m_overlayTimingTitle->setVisible(showDetails);
    }
    if(m_overlayTimingTable != nullptr) {
        m_overlayTimingTable->setVisible(showDetails);
    }
    if(m_debugTitle != nullptr) {
        m_debugTitle->setVisible(showDetails);
    }
    if(m_debugLabel != nullptr) {
        m_debugLabel->setVisible(showDetails);
    }
    if(m_contactTitle != nullptr) {
        m_contactTitle->setVisible(showDetails);
    }
    if(m_contactTable != nullptr) {
        m_contactTable->setVisible(showDetails);
    }
    if(m_nearestTitle != nullptr) {
        m_nearestTitle->setVisible(showDetails);
    }
    if(m_nearestTable != nullptr) {
        m_nearestTable->setVisible(showDetails);
    }
}
