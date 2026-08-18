#pragma once

#include <QString>
#include <QVector>

struct CollisionLinkModelTargetView
{
    QString entityKind;
    QString displayName;
    QString stableId;
    bool valid = false;
};

struct CollisionLinkModelOriginalSourceView
{
    QString label;
    QString detail;
    QString sourceKind;
};

struct CollisionLinkModelMetricRowView
{
    QString label;
    QString value;
};

struct CollisionLinkModelVariantItemView
{
    QString label;
    QString sourceLabel;
    QString typeLabel;
    QString roleLabel;
    QString detail;
    QString tooltip;
    QVector<CollisionLinkModelMetricRowView> complexityRows;
    QString role;
    QString source;
    QString variantId;
    bool enabled = true;
    bool current = false;
    bool selected = false;
};

struct CollisionLinkModelsSummaryView
{
    CollisionLinkModelTargetView target;
    CollisionLinkModelOriginalSourceView originalSource;
    QString statusText;
    bool replaceOriginal = false;
};

struct CollisionLinkModelsViewModel
{
    CollisionLinkModelsSummaryView summary;
    QVector<CollisionLinkModelVariantItemView> variants;
};
