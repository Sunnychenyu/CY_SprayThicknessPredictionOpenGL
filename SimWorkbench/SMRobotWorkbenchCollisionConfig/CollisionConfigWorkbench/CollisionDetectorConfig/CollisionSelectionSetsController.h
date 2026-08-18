#pragma once

#include "CollisionSelectionSetsViewModel.h"

#include <SimulationProject/ProjectDocument.h>

#include <QString>

class CollisionSelectionSetsController
{
public:
    enum class PreviewTargetKind
    {
        Clear,
        RobotLink,
        SceneObject,
        MountedAttachment
    };

    struct CommandResult
    {
        bool success = false;
        bool projectChanged = false;
        bool inserted = false;
        QString selectionSetId;
        QString message;
    };

    struct SelectionSetChoice
    {
        QString id;
        QString label;
    };

    struct PreviewTarget
    {
        PreviewTargetKind kind = PreviewTargetKind::Clear;
        QString robotId;
        QString linkName;
        QString objectId;
        QString attachmentId;
        QString statusMessage;
    };

    static QVector<CollisionSelectionSetListItemView> buildSelectionSetItems(
        const simulation_project::ProjectDocument& document);

    static QVector<CollisionSelectionSetMemberItemView> buildMemberItems(
        const simulation_project::ProjectDocument& document,
        const QString& selectionSetId);

    static CommandResult addSelectionSet(
        simulation_project::ProjectDocument& document,
        const QString& name);

    static CommandResult addMemberToNewSelectionSet(
        simulation_project::ProjectDocument& document,
        const QString& name,
        const simulation_project::CollisionSelectionSetMemberDesc& member);

    static CommandResult addMemberToExistingSelectionSet(
        simulation_project::ProjectDocument& document,
        const QString& selectionSetId,
        const simulation_project::CollisionSelectionSetMemberDesc& member);

    static QVector<SelectionSetChoice> buildSelectionSetChoices(
        const simulation_project::ProjectDocument& document);

    static CommandResult renameSelectionSet(
        simulation_project::ProjectDocument& document,
        const QString& selectionSetId,
        const QString& name);

    static CommandResult removeSelectionSet(
        simulation_project::ProjectDocument& document,
        const QString& selectionSetId);

    static CommandResult removeSelectionSetMember(
        simulation_project::ProjectDocument& document,
        const QString& selectionSetId,
        int memberIndex);

    static PreviewTarget buildMemberPreviewTarget(
        const simulation_project::ProjectDocument& document,
        const QString& selectionSetId,
        int memberIndex);
};
