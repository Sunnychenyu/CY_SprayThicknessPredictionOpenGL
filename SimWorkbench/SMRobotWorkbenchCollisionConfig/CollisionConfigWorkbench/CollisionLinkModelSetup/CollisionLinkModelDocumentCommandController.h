#pragma once

#include <SimulationProject/ProjectDocument.h>

#include <QString>

struct CollisionLinkModelDocumentCommandResult
{
    bool success = false;
    bool projectChanged = false;
    QString selectedSource;
    QString selectedRole;
    QString message;
};

class CollisionLinkModelDocumentCommandController
{
public:
    static CollisionLinkModelDocumentCommandResult addBoxElement(
        simulation_project::ProjectDocument& document,
        const QString& robotId,
        const QString& linkName);

    static CollisionLinkModelDocumentCommandResult setReplaceOriginal(
        simulation_project::ProjectDocument& document,
        const QString& robotId,
        bool replaceOriginal);

    static CollisionLinkModelDocumentCommandResult removeElement(
        simulation_project::ProjectDocument& document,
        const QString& robotId,
        const QString& linkName,
        const QString& elementId);

    static bool hasCollisionOverrides(
        simulation_project::ProjectDocument& document,
        const QString& robotId);
};
