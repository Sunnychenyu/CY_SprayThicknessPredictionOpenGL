#include "CollisionSelectionSetDocumentStorage.h"

#include <SimulationProject/ProjectDocumentService.h>

#include <algorithm>
#include <cctype>

namespace
{
    std::string cleanIdentifierBase(const std::string& value, const std::string& fallback)
    {
        std::string result;
        result.reserve(value.size());
        bool lastWasUnderscore = false;
        for(unsigned char ch : value) {
            if(std::isalnum(ch)) {
                result.push_back(static_cast<char>(std::tolower(ch)));
                lastWasUnderscore = false;
            } else if(!lastWasUnderscore && !result.empty()) {
                result.push_back('_');
                lastWasUnderscore = true;
            }
        }
        while(!result.empty() && result.back() == '_') {
            result.pop_back();
        }
        return result.empty() ? fallback : result;
    }
}

CollisionSelectionSetDocumentStorage::CollisionSelectionSetDocumentStorage(
    simulation_project::ProjectDocument& document)
    : m_document(document)
    , m_mutableDocument(&document)
{
}

CollisionSelectionSetDocumentStorage::CollisionSelectionSetDocumentStorage(
    const simulation_project::ProjectDocument& document)
    : m_document(document)
{
}

const simulation_project::CollisionSelectionSetDesc*
CollisionSelectionSetDocumentStorage::findSelectionSet(const std::string& selectionSetId) const
{
    auto it = std::find_if(
        m_document.collision.selectionSets.begin(),
        m_document.collision.selectionSets.end(),
        [&](const simulation_project::CollisionSelectionSetDesc& selectionSet) {
            return selectionSet.id == selectionSetId;
        });
    return it == m_document.collision.selectionSets.end() ? nullptr : &(*it);
}

simulation_project::CollisionSelectionSetDesc*
CollisionSelectionSetDocumentStorage::findSelectionSet(const std::string& selectionSetId)
{
    if(m_mutableDocument == nullptr) {
        return nullptr;
    }

    auto it = std::find_if(
        m_mutableDocument->collision.selectionSets.begin(),
        m_mutableDocument->collision.selectionSets.end(),
        [&](const simulation_project::CollisionSelectionSetDesc& selectionSet) {
            return selectionSet.id == selectionSetId;
        });
    return it == m_mutableDocument->collision.selectionSets.end() ? nullptr : &(*it);
}

const simulation_project::MountedAttachmentDesc*
CollisionSelectionSetDocumentStorage::findMountedAttachment(const std::string& attachmentId) const
{
    simulation_project::ProjectDocumentService service(m_document);
    return service.findMountedAttachment(attachmentId);
}

const simulation_project::RobotMountDesc*
CollisionSelectionSetDocumentStorage::findRobotMount(const std::string& mountId) const
{
    simulation_project::ProjectDocumentService service(m_document);
    return service.findRobotMount(mountId);
}

const simulation_project::AttachmentAssetDesc*
CollisionSelectionSetDocumentStorage::findAttachmentAsset(const std::string& assetId) const
{
    simulation_project::ProjectDocumentService service(m_document);
    return service.findAttachmentAsset(assetId);
}

bool CollisionSelectionSetDocumentStorage::addSelectionSet(
    const std::string& name,
    std::string* error)
{
    if(m_mutableDocument == nullptr) {
        if(error != nullptr) {
            *error = "Document is read-only.";
        }
        return false;
    }

    simulation_project::ProjectDocumentService service(*m_mutableDocument);
    return service.addCollisionSelectionSet(name, error);
}

bool CollisionSelectionSetDocumentStorage::addSelectionSetWithMember(
    const std::string& name,
    const simulation_project::CollisionSelectionSetMemberDesc& member,
    std::string* selectionSetId,
    std::string* error)
{
    if(m_mutableDocument == nullptr) {
        if(error != nullptr) {
            *error = "Document is read-only.";
        }
        return false;
    }

    simulation_project::CollisionSelectionSetDesc selectionSet;
    selectionSet.id = makeUniqueSelectionSetId(name);
    selectionSet.name = name.empty() ? "Collision Set" : name;
    selectionSet.members.push_back(member);

    simulation_project::ProjectDocumentService service(*m_mutableDocument);
    if(!service.addCollisionSelectionSet(selectionSet, error)) {
        return false;
    }

    if(selectionSetId != nullptr) {
        *selectionSetId = selectionSet.id;
    }
    return true;
}

bool CollisionSelectionSetDocumentStorage::addSelectionSetMember(
    const std::string& selectionSetId,
    const simulation_project::CollisionSelectionSetMemberDesc& member,
    bool* inserted,
    std::string* error)
{
    if(m_mutableDocument == nullptr) {
        if(error != nullptr) {
            *error = "Document is read-only.";
        }
        return false;
    }

    simulation_project::ProjectDocumentService service(*m_mutableDocument);
    return service.addCollisionSelectionSetMember(selectionSetId, member, inserted, error);
}

bool CollisionSelectionSetDocumentStorage::renameSelectionSet(
    const std::string& selectionSetId,
    const std::string& name,
    std::string* error)
{
    if(m_mutableDocument == nullptr) {
        if(error != nullptr) {
            *error = "Document is read-only.";
        }
        return false;
    }

    simulation_project::ProjectDocumentService service(*m_mutableDocument);
    return service.renameCollisionSelectionSet(selectionSetId, name, error);
}

bool CollisionSelectionSetDocumentStorage::removeSelectionSet(
    const std::string& selectionSetId,
    std::string* error)
{
    if(m_mutableDocument == nullptr) {
        if(error != nullptr) {
            *error = "Document is read-only.";
        }
        return false;
    }

    simulation_project::ProjectDocumentService service(*m_mutableDocument);
    return service.removeCollisionSelectionSet(selectionSetId, error);
}

bool CollisionSelectionSetDocumentStorage::removeSelectionSetMember(
    const std::string& selectionSetId,
    std::size_t memberIndex,
    std::string* error)
{
    if(m_mutableDocument == nullptr) {
        if(error != nullptr) {
            *error = "Document is read-only.";
        }
        return false;
    }

    simulation_project::ProjectDocumentService service(*m_mutableDocument);
    return service.removeCollisionSelectionSetMember(selectionSetId, memberIndex, error);
}

std::string CollisionSelectionSetDocumentStorage::makeUniqueSelectionSetId(
    const std::string& baseId) const
{
    simulation_project::ProjectDocumentService service(m_document);
    return service.makeUniqueCollisionSelectionSetId(baseId);
}
