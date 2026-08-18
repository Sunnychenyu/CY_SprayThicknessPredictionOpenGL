#include "CollisionSelectionSetViewController.h"

#include "CollisionSelectionSetDocumentFacade.h"
#include "CollisionSelectionSetsController.h"
#include "CollisionSelectionSetsWidget.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerSelectionModel.h"

#include <utility>

namespace robot_qt_viewer
{
    CollisionSelectionSetViewController::CollisionSelectionSetViewController(
        CollisionSelectionSetsWidget& widget,
        RobotQtViewerDocumentContext& context,
        CollisionSelectionSetDocumentFacade& documentFacade,
        Callbacks callbacks)
        : m_widget(widget)
        , m_context(context)
        , m_documentFacade(documentFacade)
        , m_callbacks(std::move(callbacks))
    {
    }

    QString CollisionSelectionSetViewController::currentSelectionSetId() const
    {
        return m_widget.currentSelectionSetId();
    }

    void CollisionSelectionSetViewController::refreshList(const QString& previousId)
    {
        m_widget.setSelectionSets(
            m_documentFacade.selectionSetItems(),
            previousId);
        refreshMemberList();
    }

    void CollisionSelectionSetViewController::refreshMemberList()
    {
        const int previousIndex = m_widget.currentMemberIndex();
        const QString selectionSetId = currentSelectionSetId();
        m_widget.setMembers(
            m_documentFacade.memberItems(selectionSetId),
            previousIndex);

        const bool hasSet = !selectionSetId.isEmpty();
        m_widget.setSelectionSetActionsEnabled(hasSet);
        m_widget.setRemoveMemberEnabled(false);
    }

    void CollisionSelectionSetViewController::previewMember(bool updating) const
    {
        if(updating) {
            return;
        }
        const QString selectionSetId = currentSelectionSetId();
        const int index = m_widget.currentMemberIndex();
        const CollisionSelectionSetsController::PreviewTarget target =
            m_documentFacade.memberPreviewTarget(selectionSetId, index);

        switch(target.kind) {
        case CollisionSelectionSetsController::PreviewTargetKind::MountedAttachment:
            if(!target.statusMessage.isEmpty()) {
                showStatus(target.statusMessage, 3000);
            }
            m_context.selectionModel().selectMountedAttachment(
                target.attachmentId,
                QString(),
                QString(),
                QString(),
                QStringLiteral("collisionSelectionSetPreview"));
            return;
        case CollisionSelectionSetsController::PreviewTargetKind::SceneObject:
            m_context.selectionModel().selectSceneObject(
                target.objectId,
                QStringLiteral("collisionSelectionSetPreview"));
            return;
        case CollisionSelectionSetsController::PreviewTargetKind::RobotLink:
            m_context.selectionModel().selectRobotLink(
                target.robotId,
                target.linkName,
                QStringLiteral("collisionSelectionSetPreview"));
            return;
        case CollisionSelectionSetsController::PreviewTargetKind::Clear:
            m_context.selectionModel().selectRobotLink(
                QString(),
                QString(),
                QStringLiteral("collisionSelectionSetPreview"));
            return;
        }
    }

    void CollisionSelectionSetViewController::showStatus(const QString& message, int timeoutMs) const
    {
        if(m_callbacks.statusMessage) {
            m_callbacks.statusMessage(message, timeoutMs);
        }
    }
}
