#pragma once

#include <QString>
#include <QStringList>
#include <QVector>

struct CollisionResultsTableRow
{
    QStringList cells;
    bool spanColumns = false;
};

struct CollisionResultsTableView
{
    QVector<CollisionResultsTableRow> rows;
    bool normalColumnVisible = true;
};

struct CollisionTimingTableRow
{
    QString stage;
    QString last;
    QString queryShare;
    QString frameShare;
    QString state;
    QString tooltip;
};

struct CollisionSummaryRow
{
    QString metric;
    QString value;
    QString share;
    QString tooltip;
};

struct CollisionOverlayTimingRow
{
    QString step;
    QString last;
    QString tooltip;
};

struct CollisionResultsViewModel
{
    QString detailsText;
    QString debugText;
    QVector<CollisionSummaryRow> summaryRows;
    QVector<CollisionTimingTableRow> timingRows;
    QVector<CollisionOverlayTimingRow> overlayTimingRows;
    CollisionResultsTableView contacts;
    CollisionResultsTableView nearest;
};
