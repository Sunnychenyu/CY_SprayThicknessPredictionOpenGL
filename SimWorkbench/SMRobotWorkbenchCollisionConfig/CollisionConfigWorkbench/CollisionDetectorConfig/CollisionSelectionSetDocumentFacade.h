#pragma once

#include "CollisionSelectionSetsController.h"

namespace simulation_project
{
    struct CollisionSelectionSetMemberDesc;
    struct ProjectDocument;
    class ProjectSession;
}

namespace robot_qt_viewer
{
    class CollisionWorkbenchServices;

    class CollisionSelectionSetDocumentFacade
    {
    public:
        CollisionSelectionSetDocumentFacade(
            const simulation_project::ProjectDocument& document,
            CollisionWorkbenchServices& appServices);

        bool hasSelectionSets() const;
        QString selectionSetName(const QString& selectionSetId) const;
        QVector<CollisionSelectionSetsController::SelectionSetChoice> selectionSetChoices() const;
        QVector<CollisionSelectionSetListItemView> selectionSetItems() const;
        QVector<CollisionSelectionSetMemberItemView> memberItems(const QString& selectionSetId) const;
        CollisionSelectionSetsController::PreviewTarget memberPreviewTarget(
            const QString& selectionSetId,
            int memberIndex) const;

        CollisionSelectionSetsController::CommandResult addSelectionSet(const QString& name);
        CollisionSelectionSetsController::CommandResult renameSelectionSet(
            const QString& selectionSetId,
            const QString& name);
        CollisionSelectionSetsController::CommandResult removeSelectionSet(const QString& selectionSetId);
        CollisionSelectionSetsController::CommandResult removeSelectionSetMember(
            const QString& selectionSetId,
            int memberIndex);
        CollisionSelectionSetsController::CommandResult addMemberToNewSelectionSet(
            const QString& name,
            const simulation_project::CollisionSelectionSetMemberDesc& member);
        CollisionSelectionSetsController::CommandResult addMemberToExistingSelectionSet(
            const QString& selectionSetId,
            const simulation_project::CollisionSelectionSetMemberDesc& member);

    private:
        const simulation_project::ProjectDocument& m_document;
        CollisionWorkbenchServices& m_appServices;
    };
}
