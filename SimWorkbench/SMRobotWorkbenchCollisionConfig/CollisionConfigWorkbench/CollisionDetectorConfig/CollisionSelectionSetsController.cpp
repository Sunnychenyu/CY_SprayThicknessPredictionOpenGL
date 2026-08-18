#include "CollisionSelectionSetsController.h"

#include "CollisionSelectionSetDocumentStorage.h"

#include <algorithm>
#include <string>

namespace
{
    CollisionSelectionSetsController::CommandResult makeFailure(const QString& message)
    {
        CollisionSelectionSetsController::CommandResult result;
        result.success = false;
        result.message = message;
        return result;
    }

    CollisionSelectionSetsController::CommandResult makeSuccess(const QString& message)
    {
        CollisionSelectionSetsController::CommandResult result;
        result.success = true;
        result.projectChanged = true;
        result.inserted = true;
        result.message = message;
        return result;
    }

    QString selectionSetDisplayName(const simulation_project::CollisionSelectionSetDesc& selectionSet)
    {
        return QString::fromStdString(selectionSet.name.empty() ? selectionSet.id : selectionSet.name);
    }

    QString memberText(const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        const std::string attachmentId = !member.attachmentId.empty()
            ? member.attachmentId
            : std::string();
        if(!attachmentId.empty()) {
            return QString("attachment: %1").arg(QString::fromStdString(attachmentId));
        }
        if(!member.objectId.empty()) {
            return QString("object: %1").arg(QString::fromStdString(member.objectId));
        }
        if(!member.robotId.empty() && !member.linkName.empty()) {
            return QString("link: %1.%2")
                .arg(QString::fromStdString(member.robotId), QString::fromStdString(member.linkName));
        }
        if(!member.robotId.empty()) {
            return QString("robot: %1").arg(QString::fromStdString(member.robotId));
        }
        return "invalid member";
    }

}

QVector<CollisionSelectionSetListItemView> CollisionSelectionSetsController::buildSelectionSetItems(
    const simulation_project::ProjectDocument& document)
{
    QVector<CollisionSelectionSetListItemView> items;
    for(const simulation_project::CollisionSelectionSetDesc& selectionSet : document.collision.selectionSets) {
        CollisionSelectionSetListItemView item;
        item.id = QString::fromStdString(selectionSet.id);
        item.text = selectionSetDisplayName(selectionSet);
        item.tooltip = QString("%1\nid: %2\nmembers: %3")
            .arg(item.text)
            .arg(item.id)
            .arg(static_cast<qulonglong>(selectionSet.members.size()));
        items.push_back(std::move(item));
    }
    if(items.isEmpty()) {
        CollisionSelectionSetListItemView item;
        item.text = "No collision selection sets";
        item.enabled = false;
        items.push_back(std::move(item));
    }
    return items;
}

CollisionSelectionSetsController::CommandResult CollisionSelectionSetsController::addSelectionSet(
    simulation_project::ProjectDocument& document,
    const QString& name)
{
    CollisionSelectionSetDocumentStorage documentFacade(document);
    std::string error;
    if(!documentFacade.addSelectionSet(name.toStdString(), &error)) {
        return makeFailure(QString("Add collision selection set failed: %1").arg(QString::fromStdString(error)));
    }

    return makeSuccess("Added collision selection set");
}

CollisionSelectionSetsController::CommandResult CollisionSelectionSetsController::addMemberToNewSelectionSet(
    simulation_project::ProjectDocument& document,
    const QString& name,
    const simulation_project::CollisionSelectionSetMemberDesc& member)
{
    CollisionSelectionSetDocumentStorage documentFacade(document);
    std::string selectionSetId;
    std::string error;
    const QString effectiveName = name.trimmed().isEmpty()
        ? QStringLiteral("Collision Set")
        : name.trimmed();
    if(!documentFacade.addSelectionSetWithMember(
           effectiveName.toStdString(),
           member,
           &selectionSetId,
           &error)) {
        return makeFailure(QString("Add collision selection set failed: %1").arg(QString::fromStdString(error)));
    }

    CommandResult result = makeSuccess("Added collision selection set with member.");
    result.selectionSetId = QString::fromStdString(selectionSetId);
    return result;
}

CollisionSelectionSetsController::CommandResult CollisionSelectionSetsController::addMemberToExistingSelectionSet(
    simulation_project::ProjectDocument& document,
    const QString& selectionSetId,
    const simulation_project::CollisionSelectionSetMemberDesc& member)
{
    if(selectionSetId.isEmpty()) {
        return makeFailure("No collision selection set selected.");
    }

    CollisionSelectionSetDocumentStorage documentFacade(document);
    bool inserted = false;
    std::string error;
    if(!documentFacade.addSelectionSetMember(selectionSetId.toStdString(), member, &inserted, &error)) {
        return makeFailure(QString("Add collision selection set member failed: %1").arg(QString::fromStdString(error)));
    }

    CommandResult result;
    result.success = true;
    result.projectChanged = inserted;
    result.inserted = inserted;
    result.selectionSetId = selectionSetId;
    result.message = inserted
        ? QStringLiteral("Added member to collision selection set.")
        : QStringLiteral("Collision selection set already contains that member.");
    return result;
}

QVector<CollisionSelectionSetsController::SelectionSetChoice>
CollisionSelectionSetsController::buildSelectionSetChoices(
    const simulation_project::ProjectDocument& document)
{
    QVector<SelectionSetChoice> choices;
    for(const simulation_project::CollisionSelectionSetDesc& selectionSet : document.collision.selectionSets) {
        SelectionSetChoice choice;
        choice.id = QString::fromStdString(selectionSet.id);
        choice.label = QString("%1  [%2]")
            .arg(selectionSetDisplayName(selectionSet), choice.id);
        choices.push_back(std::move(choice));
    }
    return choices;
}

CollisionSelectionSetsController::CommandResult CollisionSelectionSetsController::renameSelectionSet(
    simulation_project::ProjectDocument& document,
    const QString& selectionSetId,
    const QString& name)
{
    CollisionSelectionSetDocumentStorage documentFacade(document);
    if(documentFacade.findSelectionSet(selectionSetId.toStdString()) == nullptr) {
        return makeFailure("No collision selection set selected.");
    }

    std::string error;
    if(!documentFacade.renameSelectionSet(selectionSetId.toStdString(), name.toStdString(), &error)) {
        return makeFailure(QString("Rename collision selection set failed: %1").arg(QString::fromStdString(error)));
    }

    return makeSuccess("Renamed collision selection set");
}

CollisionSelectionSetsController::CommandResult CollisionSelectionSetsController::removeSelectionSet(
    simulation_project::ProjectDocument& document,
    const QString& selectionSetId)
{
    if(selectionSetId.isEmpty()) {
        return makeFailure("No collision selection set selected.");
    }

    CollisionSelectionSetDocumentStorage documentFacade(document);
    std::string error;
    if(!documentFacade.removeSelectionSet(selectionSetId.toStdString(), &error)) {
        return makeFailure(QString("Remove collision selection set failed: %1").arg(QString::fromStdString(error)));
    }

    return makeSuccess(QString("Removed collision selection set %1").arg(selectionSetId));
}

CollisionSelectionSetsController::CommandResult CollisionSelectionSetsController::removeSelectionSetMember(
    simulation_project::ProjectDocument& document,
    const QString& selectionSetId,
    int memberIndex)
{
    CollisionSelectionSetDocumentStorage documentFacade(document);
    if(documentFacade.findSelectionSet(selectionSetId.toStdString()) == nullptr) {
        return makeFailure("No collision selection set selected.");
    }
    if(memberIndex < 0) {
        return makeFailure("No collision selection set member selected.");
    }

    std::string error;
    if(!documentFacade.removeSelectionSetMember(
           selectionSetId.toStdString(),
           static_cast<std::size_t>(memberIndex),
           &error)) {
        return makeFailure(QString("Remove collision selection set member failed: %1").arg(QString::fromStdString(error)));
    }

    return makeSuccess("Removed collision selection set member");
}

CollisionSelectionSetsController::PreviewTarget CollisionSelectionSetsController::buildMemberPreviewTarget(
    const simulation_project::ProjectDocument& document,
    const QString& selectionSetId,
    int memberIndex)
{
    PreviewTarget target;
    CollisionSelectionSetDocumentStorage documentFacade(document);
    const simulation_project::CollisionSelectionSetDesc* selectionSet =
        documentFacade.findSelectionSet(selectionSetId.toStdString());
    if(selectionSet == nullptr ||
        memberIndex < 0 ||
        static_cast<std::size_t>(memberIndex) >= selectionSet->members.size()) {
        return target;
    }

    const simulation_project::CollisionSelectionSetMemberDesc& member =
        selectionSet->members[static_cast<std::size_t>(memberIndex)];
    const std::string attachmentId = !member.attachmentId.empty()
        ? member.attachmentId
        : std::string();
    if(!attachmentId.empty()) {
        target.kind = PreviewTargetKind::MountedAttachment;
        target.attachmentId = QString::fromStdString(attachmentId);

        const simulation_project::MountedAttachmentDesc* attachment =
            documentFacade.findMountedAttachment(attachmentId);
        if(attachment != nullptr) {
            const simulation_project::RobotMountDesc* mount =
                documentFacade.findRobotMount(attachment->mountFrameId);
            const simulation_project::AttachmentAssetDesc* asset =
                documentFacade.findAttachmentAsset(attachment->assetId);
            target.statusMessage = QString("Selected tool %1 asset=%2 mount=%3 link=%4")
                .arg(QString::fromStdString(attachment->id))
                .arg(QString::fromStdString(asset != nullptr ? asset->id : attachment->assetId))
                .arg(QString::fromStdString(mount != nullptr ? mount->id : attachment->mountFrameId))
                .arg(QString::fromStdString(mount != nullptr ? mount->linkName : std::string()));
        }
        return target;
    }

    if(!member.objectId.empty()) {
        target.kind = PreviewTargetKind::SceneObject;
        target.objectId = QString::fromStdString(member.objectId);
        return target;
    }

    if(!member.robotId.empty()) {
        target.kind = PreviewTargetKind::RobotLink;
        target.robotId = QString::fromStdString(member.robotId);
        target.linkName = QString::fromStdString(member.linkName);
        return target;
    }

    return target;
}

QVector<CollisionSelectionSetMemberItemView> CollisionSelectionSetsController::buildMemberItems(
    const simulation_project::ProjectDocument& document,
    const QString& selectionSetId)
{
    QVector<CollisionSelectionSetMemberItemView> items;
    CollisionSelectionSetDocumentStorage documentFacade(document);
    const simulation_project::CollisionSelectionSetDesc* selectionSet =
        documentFacade.findSelectionSet(selectionSetId.toStdString());
    if(selectionSet == nullptr) {
        CollisionSelectionSetMemberItemView item;
        item.text = "No selection set selected";
        item.enabled = false;
        items.push_back(std::move(item));
        return items;
    }

    for(std::size_t i = 0; i < selectionSet->members.size(); ++i) {
        CollisionSelectionSetMemberItemView item;
        item.index = static_cast<int>(i);
        item.text = memberText(selectionSet->members[i]);
        items.push_back(std::move(item));
    }
    if(items.isEmpty()) {
        CollisionSelectionSetMemberItemView item;
        item.text = "No members";
        item.enabled = false;
        items.push_back(std::move(item));
    }
    return items;
}
