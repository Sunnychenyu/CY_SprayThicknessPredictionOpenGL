#include "CollisionLegacyPairWorkbenchController.h"

#include "CollisionLegacyPairDocumentFacade.h"
#include "CollisionWorkbenchPanel.h"

#include <utility>

namespace robot_qt_viewer
{
    CollisionLegacyPairWorkbenchController::CollisionLegacyPairWorkbenchController(
        CollisionWorkbenchPanel& panel,
        CollisionLegacyPairDocumentFacade& documentFacade,
        Callbacks callbacks)
        : m_panel(panel)
        , m_documentFacade(documentFacade)
        , m_callbacks(std::move(callbacks))
    {
    }

    void CollisionLegacyPairWorkbenchController::refreshPairList()
    {
        m_panel.setLegacyPairs(m_documentFacade.buildPairItems());
        m_panel.setAutoPairAllEnabled(m_documentFacade.canAutoPairAll());
    }

    bool CollisionLegacyPairWorkbenchController::syncRobotObjectPairs()
    {
        return m_documentFacade.syncRobotObjectPairs();
    }

    void CollisionLegacyPairWorkbenchController::autoPairAllRobotObjects()
    {
        syncRobotObjectPairs();
        const CollisionPairWorkflowResult result =
            m_documentFacade.setAllRobotObjectPairsEnabled(true);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }

        requestViewportReload();
        publishCollisionChanged(QStringLiteral("autoPairAllRobotObjects"));
        showStatus(result.message, 3000);
    }

    void CollisionLegacyPairWorkbenchController::setPairEnabled(
        const QString& robotId,
        const QString& objectId,
        bool enabled)
    {
        if(isUpdating() || robotId.isEmpty() || objectId.isEmpty()) {
            return;
        }

        const CollisionPairWorkflowResult result =
            m_documentFacade.setRobotObjectPairEnabled(
                robotId,
                objectId,
                enabled);
        if(!result.success) {
            showStatus(result.message, 5000);
            refreshPairList();
            return;
        }

        requestViewportReload();
        publishCollisionChanged(QStringLiteral("handleCollisionPairEnabledChanged"));
        showStatus(result.message, 3000);
    }

    bool CollisionLegacyPairWorkbenchController::isUpdating() const
    {
        return m_callbacks.isUpdating != nullptr && m_callbacks.isUpdating();
    }

    void CollisionLegacyPairWorkbenchController::requestViewportReload() const
    {
        if(m_callbacks.viewportReload) {
            m_callbacks.viewportReload();
        }
    }

    void CollisionLegacyPairWorkbenchController::publishCollisionChanged(const QString& sourceId) const
    {
        if(m_callbacks.publishCollisionChanged) {
            m_callbacks.publishCollisionChanged(sourceId);
        }
    }

    void CollisionLegacyPairWorkbenchController::showStatus(const QString& message, int timeoutMs) const
    {
        if(m_callbacks.statusMessage) {
            m_callbacks.statusMessage(message, timeoutMs);
        }
    }
}
