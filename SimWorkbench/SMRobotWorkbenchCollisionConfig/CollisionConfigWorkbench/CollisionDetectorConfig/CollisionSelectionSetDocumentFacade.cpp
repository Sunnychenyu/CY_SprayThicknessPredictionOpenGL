#include "CollisionSelectionSetDocumentFacade.h"

#include "CollisionWorkbenchServices.h"

#include <SimulationProject/ProjectDocument.h>
#include <SimulationProject/ProjectDocumentService.h>
#include <SimulationProject/ProjectSession.h>

#include <algorithm>

namespace robot_qt_viewer
{
    CollisionSelectionSetDocumentFacade::CollisionSelectionSetDocumentFacade(
        const simulation_project::ProjectDocument& document,
        CollisionWorkbenchServices& appServices)
        : m_document(document)
        , m_appServices(appServices)
    {
    }

    bool CollisionSelectionSetDocumentFacade::hasSelectionSets() const
    {
        return !m_document.collision.selectionSets.empty();
    }

    QString CollisionSelectionSetDocumentFacade::selectionSetName(const QString& selectionSetId) const
    {
        const auto it = std::find_if(
            m_document.collision.selectionSets.begin(),
            m_document.collision.selectionSets.end(),
            [&](const simulation_project::CollisionSelectionSetDesc& selectionSet) {
                return selectionSet.id == selectionSetId.toStdString();
            });
        if(it == m_document.collision.selectionSets.end()) {
            return QString();
        }
        return QString::fromStdString(it->name.empty() ? it->id : it->name);
    }

    QVector<CollisionSelectionSetsController::SelectionSetChoice>
    CollisionSelectionSetDocumentFacade::selectionSetChoices() const
    {
        return CollisionSelectionSetsController::buildSelectionSetChoices(m_document);
    }

    QVector<CollisionSelectionSetListItemView> CollisionSelectionSetDocumentFacade::selectionSetItems() const
    {
        return CollisionSelectionSetsController::buildSelectionSetItems(m_document);
    }

    QVector<CollisionSelectionSetMemberItemView> CollisionSelectionSetDocumentFacade::memberItems(
        const QString& selectionSetId) const
    {
        return CollisionSelectionSetsController::buildMemberItems(m_document, selectionSetId);
    }

    CollisionSelectionSetsController::PreviewTarget CollisionSelectionSetDocumentFacade::memberPreviewTarget(
        const QString& selectionSetId,
        int memberIndex) const
    {
        return CollisionSelectionSetsController::buildMemberPreviewTarget(m_document, selectionSetId, memberIndex);
    }

    CollisionSelectionSetsController::CommandResult CollisionSelectionSetDocumentFacade::addSelectionSet(
        const QString& name)
    {
        CollisionSelectionSetsController::CommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionSelectionSet"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionSelectionSetsController::addSelectionSet(service.document(), name);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionSelectionSetsController::CommandResult CollisionSelectionSetDocumentFacade::renameSelectionSet(
        const QString& selectionSetId,
        const QString& name)
    {
        CollisionSelectionSetsController::CommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionSelectionSet"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionSelectionSetsController::renameSelectionSet(
                    service.document(),
                    selectionSetId,
                    name);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionSelectionSetsController::CommandResult CollisionSelectionSetDocumentFacade::removeSelectionSet(
        const QString& selectionSetId)
    {
        CollisionSelectionSetsController::CommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionSelectionSet"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionSelectionSetsController::removeSelectionSet(service.document(), selectionSetId);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionSelectionSetsController::CommandResult CollisionSelectionSetDocumentFacade::removeSelectionSetMember(
        const QString& selectionSetId,
        int memberIndex)
    {
        CollisionSelectionSetsController::CommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionSelectionSet"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionSelectionSetsController::removeSelectionSetMember(
                    service.document(),
                    selectionSetId,
                    memberIndex);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionSelectionSetsController::CommandResult CollisionSelectionSetDocumentFacade::addMemberToNewSelectionSet(
        const QString& name,
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        CollisionSelectionSetsController::CommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionSelectionSet"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionSelectionSetsController::addMemberToNewSelectionSet(
                    service.document(),
                    name,
                    member);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }

    CollisionSelectionSetsController::CommandResult CollisionSelectionSetDocumentFacade::addMemberToExistingSelectionSet(
        const QString& selectionSetId,
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        CollisionSelectionSetsController::CommandResult result;
        m_appServices.mutateProject(
            QStringLiteral("collisionSelectionSet"),
            ProjectDirtyPolicy::UserEdit,
            [&](simulation_project::ProjectDocumentService& service, bool& changed, std::string&) {
                result = CollisionSelectionSetsController::addMemberToExistingSelectionSet(
                    service.document(),
                    selectionSetId,
                    member);
                changed = result.projectChanged;
                return result.success;
            });
        return result;
    }
}
