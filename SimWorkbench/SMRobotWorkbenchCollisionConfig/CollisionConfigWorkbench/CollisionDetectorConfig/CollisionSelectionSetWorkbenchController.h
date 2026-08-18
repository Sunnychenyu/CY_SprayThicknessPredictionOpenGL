#pragma once

#include <QString>

#include <functional>

class CollisionWorkbenchPanel;

namespace simulation_project
{
    struct CollisionSelectionSetMemberDesc;
}

namespace robot_qt_viewer
{
    class CollisionSelectionSetDocumentFacade;

    class CollisionSelectionSetWorkbenchController
    {
    public:
        struct Callbacks
        {
            std::function<void()> refreshSelectionSetList;
            std::function<void(const QString&)> publishCollisionChanged;
            std::function<void(const QString&, int)> statusMessage;
        };

        CollisionSelectionSetWorkbenchController(
            CollisionWorkbenchPanel& panel,
            CollisionSelectionSetDocumentFacade& documentFacade,
            Callbacks callbacks);

        void addSelectionSet();
        void renameSelectedSelectionSet();
        void removeSelectedSelectionSet();
        void removeSelectedSelectionSetMember();
        void addMemberToNewSelectionSet(
            const QString& defaultName,
            const simulation_project::CollisionSelectionSetMemberDesc& member);
        void addMemberToExistingSelectionSet(
            const simulation_project::CollisionSelectionSetMemberDesc& member);

    private:
        QString currentSelectionSetId() const;
        void refreshSelectionSetList() const;
        void publishCollisionChanged(const QString& sourceId) const;
        void showStatus(const QString& message, int timeoutMs) const;

        CollisionWorkbenchPanel& m_panel;
        CollisionSelectionSetDocumentFacade& m_documentFacade;
        Callbacks m_callbacks;
    };
}
