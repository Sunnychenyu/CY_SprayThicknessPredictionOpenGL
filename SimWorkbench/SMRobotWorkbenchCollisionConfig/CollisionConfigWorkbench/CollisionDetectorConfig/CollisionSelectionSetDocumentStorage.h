#pragma once

#include <SimulationProject/ProjectDocument.h>

#include <cstddef>
#include <string>

class CollisionSelectionSetDocumentStorage
{
public:
    explicit CollisionSelectionSetDocumentStorage(simulation_project::ProjectDocument& document);
    explicit CollisionSelectionSetDocumentStorage(const simulation_project::ProjectDocument& document);

    const simulation_project::CollisionSelectionSetDesc* findSelectionSet(
        const std::string& selectionSetId) const;
    simulation_project::CollisionSelectionSetDesc* findSelectionSet(
        const std::string& selectionSetId);
    const simulation_project::MountedAttachmentDesc* findMountedAttachment(
        const std::string& attachmentId) const;
    const simulation_project::RobotMountDesc* findRobotMount(const std::string& mountId) const;
    const simulation_project::AttachmentAssetDesc* findAttachmentAsset(
        const std::string& assetId) const;

    bool addSelectionSet(const std::string& name, std::string* error = nullptr);
    bool addSelectionSetWithMember(
        const std::string& name,
        const simulation_project::CollisionSelectionSetMemberDesc& member,
        std::string* selectionSetId = nullptr,
        std::string* error = nullptr);
    bool addSelectionSetMember(
        const std::string& selectionSetId,
        const simulation_project::CollisionSelectionSetMemberDesc& member,
        bool* inserted = nullptr,
        std::string* error = nullptr);
    bool renameSelectionSet(
        const std::string& selectionSetId,
        const std::string& name,
        std::string* error = nullptr);
    bool removeSelectionSet(const std::string& selectionSetId, std::string* error = nullptr);
    bool removeSelectionSetMember(
        const std::string& selectionSetId,
        std::size_t memberIndex,
        std::string* error = nullptr);

private:
    std::string makeUniqueSelectionSetId(const std::string& baseId) const;

    const simulation_project::ProjectDocument& m_document;
    simulation_project::ProjectDocument* m_mutableDocument = nullptr;
};
