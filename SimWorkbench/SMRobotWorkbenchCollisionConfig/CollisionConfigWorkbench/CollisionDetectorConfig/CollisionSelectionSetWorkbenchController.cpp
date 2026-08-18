#include "CollisionSelectionSetWorkbenchController.h"

#include "CollisionWorkbenchPanel.h"
#include "CollisionSelectionSetDocumentFacade.h"

#include <QInputDialog>
#include <QLineEdit>
#include <QMessageBox>
#include <QStringList>

#include <utility>

namespace robot_qt_viewer
{
    CollisionSelectionSetWorkbenchController::CollisionSelectionSetWorkbenchController(
        CollisionWorkbenchPanel& panel,
        CollisionSelectionSetDocumentFacade& documentFacade,
        Callbacks callbacks)
        : m_panel(panel)
        , m_documentFacade(documentFacade)
        , m_callbacks(std::move(callbacks))
    {
    }

    void CollisionSelectionSetWorkbenchController::addSelectionSet()
    {
        bool ok = false;
        const QString name = QInputDialog::getText(
            &m_panel,
            "New Collision Selection Set",
            "Display name:",
            QLineEdit::Normal,
            "Collision Set",
            &ok).trimmed();
        if(!ok) {
            return;
        }

        const CollisionSelectionSetsController::CommandResult result =
            m_documentFacade.addSelectionSet(name);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }
        publishCollisionChanged(QStringLiteral("addCollisionSelectionSet"));
        showStatus(result.message, 3000);
    }

    void CollisionSelectionSetWorkbenchController::renameSelectedSelectionSet()
    {
        const QString selectionSetId = currentSelectionSetId();
        const QString currentName = m_documentFacade.selectionSetName(selectionSetId);
        if(currentName.isEmpty()) {
            showStatus("No collision selection set selected.", 3000);
            return;
        }

        bool ok = false;
        const QString name = QInputDialog::getText(
            &m_panel,
            "Rename Collision Selection Set",
            "Display name:",
            QLineEdit::Normal,
            currentName,
            &ok).trimmed();
        if(!ok) {
            return;
        }

        const CollisionSelectionSetsController::CommandResult result =
            m_documentFacade.renameSelectionSet(selectionSetId, name);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }
        publishCollisionChanged(QStringLiteral("renameSelectedCollisionSelectionSet"));
        showStatus(result.message, 3000);
    }

    void CollisionSelectionSetWorkbenchController::removeSelectedSelectionSet()
    {
        const QString selectionSetId = currentSelectionSetId();
        if(selectionSetId.isEmpty()) {
            showStatus("No collision selection set selected.", 3000);
            return;
        }

        if(QMessageBox::question(
               &m_panel,
               "Remove Collision Selection Set",
               QString("Remove collision selection set %1?").arg(selectionSetId),
               QMessageBox::Yes | QMessageBox::No,
               QMessageBox::No) != QMessageBox::Yes) {
            return;
        }

        const CollisionSelectionSetsController::CommandResult result =
            m_documentFacade.removeSelectionSet(selectionSetId);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }

        publishCollisionChanged(QStringLiteral("removeSelectedCollisionSelectionSet"));
        showStatus(result.message, 3000);
    }

    void CollisionSelectionSetWorkbenchController::removeSelectedSelectionSetMember()
    {
        const QString selectionSetId = currentSelectionSetId();
        const int index = m_panel.currentSelectionSetMemberIndex();
        if(index < 0) {
            showStatus("No collision selection set member selected.", 3000);
            return;
        }

        const CollisionSelectionSetsController::CommandResult result =
            m_documentFacade.removeSelectionSetMember(selectionSetId, index);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }
        publishCollisionChanged(QStringLiteral("removeSelectedCollisionSelectionSetMember"));
        showStatus(result.message, 3000);
    }

    void CollisionSelectionSetWorkbenchController::addMemberToNewSelectionSet(
        const QString& defaultName,
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        bool ok = false;
        const QString name = QInputDialog::getText(
            &m_panel,
            "New Collision Selection Set",
            "Display name:",
            QLineEdit::Normal,
            defaultName.isEmpty() ? "Collision Set" : defaultName,
            &ok).trimmed();
        if(!ok) {
            return;
        }

        const CollisionSelectionSetsController::CommandResult result =
            m_documentFacade.addMemberToNewSelectionSet(name, member);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }
        publishCollisionChanged(QStringLiteral("addMemberToNewCollisionSelectionSet"));
        m_panel.selectSelectionSet(result.selectionSetId);
        showStatus(result.message, 3000);
    }

    void CollisionSelectionSetWorkbenchController::addMemberToExistingSelectionSet(
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        if(!m_documentFacade.hasSelectionSets()) {
            addMemberToNewSelectionSet(QString(), member);
            return;
        }

        const QVector<CollisionSelectionSetsController::SelectionSetChoice> choices =
            m_documentFacade.selectionSetChoices();

        QStringList labels;
        for(const CollisionSelectionSetsController::SelectionSetChoice& choice : choices) {
            labels.push_back(choice.label);
        }

        bool ok = false;
        const QString label = QInputDialog::getItem(
            &m_panel,
            "Add to Collision Selection Set",
            "Selection set:",
            labels,
            0,
            false,
            &ok);
        if(!ok || label.isEmpty()) {
            return;
        }

        const int index = labels.indexOf(label);
        if(index < 0 || index >= choices.size()) {
            return;
        }

        const CollisionSelectionSetsController::CommandResult result =
            m_documentFacade.addMemberToExistingSelectionSet(choices[index].id, member);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }

        publishCollisionChanged(QStringLiteral("addMemberToExistingCollisionSelectionSet"));
        m_panel.selectSelectionSet(result.selectionSetId);
        showStatus(result.message, 3000);
    }

    QString CollisionSelectionSetWorkbenchController::currentSelectionSetId() const
    {
        return m_panel.currentSelectionSetId();
    }

    void CollisionSelectionSetWorkbenchController::refreshSelectionSetList() const
    {
        if(m_callbacks.refreshSelectionSetList) {
            m_callbacks.refreshSelectionSetList();
        }
    }

    void CollisionSelectionSetWorkbenchController::publishCollisionChanged(const QString& sourceId) const
    {
        if(m_callbacks.publishCollisionChanged) {
            m_callbacks.publishCollisionChanged(sourceId);
        }
    }

    void CollisionSelectionSetWorkbenchController::showStatus(const QString& message, int timeoutMs) const
    {
        if(m_callbacks.statusMessage) {
            m_callbacks.statusMessage(message, timeoutMs);
        }
    }
}
