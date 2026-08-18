#pragma once

#include <QString>
#include <QVector>

struct CollisionSelectionSetListItemView
{
    QString id;
    QString text;
    QString tooltip;
    bool enabled = true;
};

struct CollisionSelectionSetMemberItemView
{
    int index = -1;
    QString text;
    bool enabled = true;
};
