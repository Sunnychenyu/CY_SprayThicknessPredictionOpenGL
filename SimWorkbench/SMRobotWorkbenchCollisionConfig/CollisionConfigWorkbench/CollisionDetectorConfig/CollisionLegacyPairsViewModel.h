#pragma once

#include <QString>

struct CollisionLegacyPairItemView
{
    QString robotId;
    QString objectId;
    QString label;
    bool enabled = false;
    bool selectable = true;
};
