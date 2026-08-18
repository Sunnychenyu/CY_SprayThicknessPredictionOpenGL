#pragma once

#include <QString>
#include <QVector>

struct CollisionDetectorListItemView
{
    QString id;
    QString text;
    QString tooltip;
    bool enabled = true;
    bool checked = false;
    bool selected = false;
    bool active = false;
};

struct CollisionDetectorSelectionSetOptionView
{
    QString id;
    QString text;
};

struct CollisionDetectorPropertiesView
{
    bool hasDetector = false;
    QString id;
    QString name;
    bool enabled = false;
    bool visible = false;
    bool geometry = false;
    bool contacts = false;
    bool normals = false;
    bool nearest = false;
    double maxContacts = 0.0;
    double distanceThreshold = 0.0;
    QString role = "Exact";
    QString policy = "Legacy";
    QString setA;
    QString setB;
    QVector<CollisionDetectorSelectionSetOptionView> selectionSets;
};

struct CollisionDetectorQueryContractView
{
    bool hasDetector = false;
    QString id;
    QString name;
    bool contacts = false;
    bool normals = false;
    bool nearest = false;
    double maxContacts = 0.0;
    double distanceThreshold = 0.0;
    QString role = "Exact";
};

struct CollisionDetectorPairItemView
{
    int index = -1;
    int generatorIndex = -1;
    QString bodyA;
    QString bodyB;
    QString robotAId;
    QString linkAName;
    QString objectAId;
    QString attachmentAId;
    QString robotBId;
    QString linkBName;
    QString objectBId;
    QString attachmentBId;
    QString source;
    QString expansion;
    QString filterState;
    QString tooltip;
    bool enabled = true;
    bool removable = false;
    bool clearsScope = false;
};

struct CollisionDetectorPairsViewModel
{
    QString summary;
    QVector<CollisionDetectorPairItemView> items;
};

struct CollisionDetectorDraftMemberView
{
    QString text;
    QString tooltip;
    QString robotId;
    QString linkName;
    QString objectId;
    QString attachmentId;
    bool enabled = true;
};
