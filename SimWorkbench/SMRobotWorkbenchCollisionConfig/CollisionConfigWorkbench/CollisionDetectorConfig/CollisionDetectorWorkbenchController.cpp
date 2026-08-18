#include "CollisionDetectorWorkbenchController.h"

#include "CollisionDetectorWorkbenchDocumentFacade.h"
#include "CollisionWorkbenchServices.h"
#include "CollisionWorkbenchPanel.h"
#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerSelectionModel.h"
#include "RobotQtViewerViewportServices.h"

#include <SimulationProject/ProjectDocument.h>

#include <utility>

namespace robot_qt_viewer
{
    namespace
    {
        CollisionDetectorDraftMemberView makeDraftMember(
            const QString& text,
            const QString& robotId,
            const QString& linkName,
            const QString& objectId,
            const QString& attachmentId)
        {
            CollisionDetectorDraftMemberView member;
            member.text = text;
            member.tooltip = text;
            member.robotId = robotId;
            member.linkName = linkName;
            member.objectId = objectId;
            member.attachmentId = attachmentId;
            member.enabled = true;
            return member;
        }
    }

    CollisionDetectorWorkbenchController::CollisionDetectorWorkbenchController(
        CollisionWorkbenchPanel& panel,
        RobotQtViewerDocumentContext& context,
        CollisionWorkbenchServices& appServices,
        CollisionDetectorWorkbenchDocumentFacade& documentFacade,
        Callbacks callbacks)
        : m_panel(panel)
        , m_context(context)
        , m_appServices(appServices)
        , m_documentFacade(documentFacade)
        , m_callbacks(std::move(callbacks))
    {
    }

    void CollisionDetectorWorkbenchController::setDetectorEnabled(const QString& detectorId, bool enabled)
    {
        if(isUpdating() || detectorId.isEmpty()) {
            return;
        }

        const CollisionDetectorCommandResult result =
            m_documentFacade.setDetectorEnabled(
                m_context.viewportServices(),
                detectorId,
                enabled);
        if(!result.success) {
            showStatus(result.message, 3000);
            return;
        }

        if(result.projectChanged) {
            publishCollisionChanged(QStringLiteral("handleCollisionDetectorEnabledChanged"), detectorId);
        } else {
            refreshDetectorProperties();
            refreshDetectorDetails();
        }
        showStatus(result.message, 3000);
    }

    void CollisionDetectorWorkbenchController::handleSelectionChanged()
    {
        refreshDetectorProperties();
        refreshDetectorDetails();
        if(isUpdating()) {
            return;
        }

        const QString detectorId = currentDetectorId();
        RobotQtViewerViewportServices* viewportServices = m_context.viewportServices();
        if(!detectorId.isEmpty() && viewportServices != nullptr) {
            viewportServices->setActiveCollisionDetector(detectorId);
            m_appServices.setActiveCollisionDetectorContext(detectorId);
        }
        m_panel.setDetectorActionsEnabled(
            m_documentFacade.hasSceneEntities(),
            !detectorId.isEmpty(),
            !detectorId.isEmpty() && m_documentFacade.hasMultipleDetectors());
        refreshElementList(QString());
        previewSelection();
    }

    void CollisionDetectorWorkbenchController::applyDetectorProperties()
    {
        if(isUpdating()) {
            return;
        }

        const QString detectorId = currentDetectorId();
        const CollisionDetectorQueryContractView editorValues = m_panel.currentDetectorQueryContract();
        const CollisionDetectorCommandResult result =
            m_documentFacade.applyDetectorQueryContract(
                m_context.viewportServices(),
                detectorId,
                editorValues);
        if(!result.success) {
            showStatus(result.message, 3000);
            refreshDetectorProperties();
            return;
        }

        if(result.projectChanged) {
            publishCollisionChanged(QStringLiteral("handleCollisionDetectorPropertyChanged"), detectorId);
        } else {
            refreshDetectorList();
            refreshDetectorDetails();
            refreshElementList(QString());
        }
        if(result.runtimeUpdateFailed) {
            showStatus("Runtime detector update failed; rebuilding viewport scene.", 3000);
            m_appServices.reloadViewport(QStringLiteral("handleCollisionDetectorPropertyChanged"));
            if(RobotQtViewerViewportServices* viewportServices = m_context.viewportServices()) {
                viewportServices->setActiveCollisionDetector(detectorId);
            }
        }
        m_appServices.setActiveCollisionDetectorContext(detectorId);
        showStatus(result.message, 3000);
    }

    void CollisionDetectorWorkbenchController::previewSelection()
    {
        if(isUpdating()) {
            return;
        }
        const CollisionDetectorPreviewTarget target = m_documentFacade.previewTarget(currentDetectorId());
        if(target.type == CollisionDetectorPreviewTarget::Type::None) {
            m_context.selectionModel().selectRobotLink(
                QString(),
                QString(),
                QStringLiteral("collisionDetectorPreview"));
            return;
        }

        if(target.type == CollisionDetectorPreviewTarget::Type::SceneObject) {
            m_context.selectionModel().selectSceneObject(
                target.objectId,
                QStringLiteral("collisionDetectorPreview"));
        } else if(target.type == CollisionDetectorPreviewTarget::Type::RobotLink) {
            m_context.selectionModel().selectRobotLink(
                target.robotId,
                target.linkName,
                QStringLiteral("collisionDetectorPreview"));
        } else if(target.type == CollisionDetectorPreviewTarget::Type::MountedAttachment) {
            m_context.selectionModel().selectMountedAttachment(
                target.attachmentId,
                QString(),
                QString(),
                QString(),
                QStringLiteral("collisionDetectorPreview"));
        }
    }

    void CollisionDetectorWorkbenchController::showSelectedDetectors()
    {
        QVector<QString> visibleIds = m_panel.selectedDetectorIds();
        if(visibleIds.empty()) {
            const QString current = currentDetectorId();
            if(!current.isEmpty()) {
                visibleIds.push_back(current);
            }
        }
        if(visibleIds.empty()) {
            showStatus("Select a detector to show.", 3000);
            return;
        }

        const CollisionDetectorCommandResult result =
            m_documentFacade.showOnlyDetectors(
                m_context.viewportServices(),
                visibleIds);
        if(!result.success) {
            showStatus(result.message, 3000);
            return;
        }

        if(result.projectChanged) {
            publishCollisionChanged(QStringLiteral("showSelectedCollisionDetectors"));
        } else {
            refreshDetectorList();
        }
        showStatus(result.message, 3000);
    }

    void CollisionDetectorWorkbenchController::removeSelectedDetector()
    {
        const QString detectorId = currentDetectorId();
        if(detectorId.isEmpty()) {
            showStatus("No collision detector selected.", 3000);
            return;
        }

        const CollisionDetectorCommandResult result =
            m_documentFacade.removeDetector(
                m_context.viewportServices(),
                detectorId);
        if(!result.success) {
            showStatus(result.message, 3000);
            return;
        }

        if(result.projectChanged) {
            publishCollisionChanged(QStringLiteral("removeSelectedCollisionDetector"), detectorId);
        } else {
            refreshDetectorList();
        }
        showStatus(result.message, 3000);
    }

    void CollisionDetectorWorkbenchController::addDetectorFromTaskPanel()
    {
        if(!m_documentFacade.hasSceneEntities()) {
            showStatus("Add a robot or scene object before creating a collision detector.", 4000);
            return;
        }

        const CollisionDetectorAddResult result = m_documentFacade.addTaskPanelDefaultDetector();
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }

        m_appServices.reloadViewport(QStringLiteral("addCollisionDetectorFromTaskPanel"));
        if(RobotQtViewerViewportServices* viewportServices = m_context.viewportServices()) {
            viewportServices->setActiveCollisionDetector(result.detectorId);
        }
        m_appServices.setActiveCollisionDetectorContext(result.detectorId);
        publishCollisionChanged(QStringLiteral("addCollisionDetectorFromTaskPanel"), result.detectorId);
        showStatus(result.message, 3000);
    }

    void CollisionDetectorWorkbenchController::editSelectedDetector()
    {
        const QString detectorId = currentDetectorId();
        if(detectorId.isEmpty()) {
            showStatus("No collision detector is available to edit.", 3000);
            return;
        }

        applyDetectorProperties();
    }

    void CollisionDetectorWorkbenchController::addSceneAllDetector()
    {
        const CollisionDetectorAddResult result = m_documentFacade.addSceneAllDetector();
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }

        m_appServices.reloadViewport(QStringLiteral("addSceneAllCollisionDetector"));
        if(RobotQtViewerViewportServices* viewportServices = m_context.viewportServices()) {
            viewportServices->setActiveCollisionDetector(result.detectorId);
        }
        m_appServices.setActiveCollisionDetectorContext(result.detectorId);
        publishCollisionChanged(QStringLiteral("addSceneAllCollisionDetector"), result.detectorId);
        showStatus(result.message, 3000);
    }

    void CollisionDetectorWorkbenchController::addSelectionMemberToDraftSet(
        const QString& side,
        const QString& displayName,
        const QStringList& robotLinks,
        const simulation_project::CollisionSelectionSetMemberDesc& member)
    {
        if(member.robotId.empty() && member.objectId.empty() && member.attachmentId.empty()) {
            showStatus("Select a robot, robot link, scene object, or mounted attachment first.", 3000);
            return;
        }

        int addedCount = 0;
        const QString robotId = QString::fromStdString(member.robotId);
        const QString linkName = QString::fromStdString(member.linkName);
        const QString objectId = QString::fromStdString(member.objectId);
        const QString attachmentId = QString::fromStdString(member.attachmentId);
        if(!robotId.isEmpty() && linkName.isEmpty() && !robotLinks.isEmpty()) {
            for(const QString& link : robotLinks) {
                m_panel.addDetectorDraftSetMember(
                    side,
                    makeDraftMember(
                        QString("link: %1.%2").arg(robotId, link),
                        robotId,
                        link,
                        QString(),
                        QString()));
                ++addedCount;
            }
        } else if(!robotId.isEmpty()) {
            const QString text = linkName.isEmpty()
                ? QString("robot: %1").arg(robotId)
                : QString("link: %1.%2").arg(robotId, linkName);
            m_panel.addDetectorDraftSetMember(
                side,
                makeDraftMember(text, robotId, linkName, QString(), QString()));
            ++addedCount;
        } else if(!objectId.isEmpty()) {
            const QString text = displayName.isEmpty()
                ? QString("object: %1").arg(objectId)
                : QString("object: %1").arg(displayName);
            m_panel.addDetectorDraftSetMember(
                side,
                makeDraftMember(text, QString(), QString(), objectId, QString()));
            ++addedCount;
        } else if(!attachmentId.isEmpty()) {
            const QString text = displayName.isEmpty()
                ? QString("attachment: %1").arg(attachmentId)
                : QString("attachment: %1").arg(displayName);
            m_panel.addDetectorDraftSetMember(
                side,
                makeDraftMember(text, QString(), QString(), QString(), attachmentId));
            ++addedCount;
        }

        if(addedCount <= 0) {
            showStatus("Selected item did not produce detector Set members.", 3000);
            return;
        }
        showStatus(
            QString("Added %1 model member(s) to temporary Set %2").arg(addedCount).arg(side.toUpper()),
            2000);
    }

    void CollisionDetectorWorkbenchController::bindDraftSetsToDetector()
    {
        if(isUpdating()) {
            return;
        }

        const QString detectorId = currentDetectorId();
        if(detectorId.isEmpty()) {
            showStatus("No collision detector selected.", 3000);
            return;
        }

        const QVector<CollisionDetectorDraftMemberView> setA =
            m_panel.detectorDraftSetMembers(QStringLiteral("A"));
        const QVector<CollisionDetectorDraftMemberView> setB =
            m_panel.detectorDraftSetMembers(QStringLiteral("B"));
        const CollisionDetectorCommandResult result =
            m_documentFacade.bindDetectorDraftSets(
                m_context.viewportServices(),
                detectorId,
                setA,
                setB);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }

        m_panel.clearDetectorDraftPairBuilder();
        if(result.projectChanged) {
            publishCollisionChanged(QStringLiteral("bindCollisionDetectorDraftSets"), detectorId);
        } else {
            refreshDetectorProperties();
            refreshDetectorDetails();
        }
        if(result.runtimeUpdateFailed) {
            showStatus("Runtime detector update failed; rebuilding viewport scene.", 3000);
            m_appServices.reloadViewport(QStringLiteral("bindCollisionDetectorDraftSets"));
            if(RobotQtViewerViewportServices* viewportServices = m_context.viewportServices()) {
                viewportServices->setActiveCollisionDetector(detectorId);
            }
        } else {
            showStatus(result.message, 3000);
        }
    }

    void CollisionDetectorWorkbenchController::removePairGenerators(const QVector<int>& generatorIndexes)
    {
        if(isUpdating()) {
            return;
        }

        const QString detectorId = currentDetectorId();
        if(detectorId.isEmpty()) {
            showStatus("No collision detector selected.", 3000);
            return;
        }

        const CollisionDetectorCommandResult result =
            m_documentFacade.removeDetectorPairGenerators(
                m_context.viewportServices(),
                detectorId,
                generatorIndexes);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }

        if(result.projectChanged) {
            publishCollisionChanged(QStringLiteral("removeCollisionDetectorPairGenerators"), detectorId);
            refreshDetectorDetails();
        } else {
            refreshDetectorProperties();
            refreshDetectorDetails();
        }
        if(result.runtimeUpdateFailed) {
            showStatus("Runtime detector update failed; rebuilding viewport scene.", 3000);
            m_appServices.reloadViewport(QStringLiteral("removeCollisionDetectorPairGenerators"));
            if(RobotQtViewerViewportServices* viewportServices = m_context.viewportServices()) {
                viewportServices->setActiveCollisionDetector(detectorId);
            }
        } else {
            showStatus(result.message, 3000);
        }
    }

    void CollisionDetectorWorkbenchController::clearPairScope()
    {
        if(isUpdating()) {
            return;
        }

        const QString detectorId = currentDetectorId();
        if(detectorId.isEmpty()) {
            showStatus("No collision detector selected.", 3000);
            return;
        }

        const CollisionDetectorCommandResult result =
            m_documentFacade.clearDetectorPairScope(
                m_context.viewportServices(),
                detectorId);
        if(!result.success) {
            showStatus(result.message, 5000);
            return;
        }

        if(result.projectChanged) {
            publishCollisionChanged(QStringLiteral("clearCollisionDetectorPairScope"), detectorId);
            refreshDetectorDetails();
        } else {
            refreshDetectorProperties();
            refreshDetectorDetails();
        }
        if(result.runtimeUpdateFailed) {
            showStatus("Runtime detector update failed; rebuilding viewport scene.", 3000);
            m_appServices.reloadViewport(QStringLiteral("clearCollisionDetectorPairScope"));
            if(RobotQtViewerViewportServices* viewportServices = m_context.viewportServices()) {
                viewportServices->setActiveCollisionDetector(detectorId);
            }
        } else {
            showStatus(result.message, 3000);
        }
    }

    void CollisionDetectorWorkbenchController::previewPairScope(
        const QString& robotAId,
        const QString& linkAName,
        const QString& objectAId,
        const QString& attachmentAId,
        const QString& robotBId,
        const QString& linkBName,
        const QString& objectBId,
        const QString& attachmentBId)
    {
        auto previewAttachment = [&](const QString& attachmentId, const QString& source) {
            if(!attachmentId.isEmpty()) {
                m_context.selectionModel().selectMountedAttachment(
                    attachmentId,
                    QString(),
                    QString(),
                    QString(),
                    source);
            }
        };

        previewAttachment(attachmentAId, QStringLiteral("collisionDetectorPairScopeA"));
        previewAttachment(attachmentBId, QStringLiteral("collisionDetectorPairScopeB"));

        RobotQtViewerViewportServices* viewportServices = m_context.viewportServices();
        if(viewportServices != nullptr) {
            viewportServices->previewCollisionPairTargets(
                robotAId,
                linkAName,
                objectAId,
                attachmentAId,
                robotBId,
                linkBName,
                objectBId,
                attachmentBId);
        }
    }

    void CollisionDetectorWorkbenchController::previewDraftMember(
        const QString& robotId,
        const QString& linkName,
        const QString& objectId,
        const QString& attachmentId)
    {
        if(!attachmentId.isEmpty()) {
            m_context.selectionModel().selectMountedAttachment(
                attachmentId,
                QString(),
                QString(),
                QString(),
                QStringLiteral("collisionDetectorDraftMember"));
        } else if(!objectId.isEmpty()) {
            m_context.selectionModel().selectSceneObject(
                objectId,
                QStringLiteral("collisionDetectorDraftMember"));
        } else if(!robotId.isEmpty()) {
            m_context.selectionModel().selectRobotLink(
                robotId,
                linkName,
                QStringLiteral("collisionDetectorDraftMember"));
        }

        if(RobotQtViewerViewportServices* viewportServices = m_context.viewportServices()) {
            viewportServices->previewCollisionPairTargets(
                robotId,
                linkName,
                objectId,
                attachmentId,
                QString(),
                QString(),
                QString(),
                QString());
        }
    }

    QString CollisionDetectorWorkbenchController::currentDetectorId() const
    {
        return m_panel.currentDetectorId();
    }

    bool CollisionDetectorWorkbenchController::isUpdating() const
    {
        return m_callbacks.isUpdating != nullptr && m_callbacks.isUpdating();
    }

    void CollisionDetectorWorkbenchController::refreshDetectorList() const
    {
        if(m_callbacks.refreshDetectorList) {
            m_callbacks.refreshDetectorList();
        }
    }

    void CollisionDetectorWorkbenchController::refreshDetectorProperties() const
    {
        if(m_callbacks.refreshDetectorProperties) {
            m_callbacks.refreshDetectorProperties();
        }
    }

    void CollisionDetectorWorkbenchController::refreshDetectorDetails() const
    {
        if(m_callbacks.refreshDetectorDetails) {
            m_callbacks.refreshDetectorDetails();
        }
    }

    void CollisionDetectorWorkbenchController::refreshElementList(const QString& qualityMessage) const
    {
        if(m_callbacks.refreshElementList) {
            m_callbacks.refreshElementList(qualityMessage);
        }
    }

    void CollisionDetectorWorkbenchController::publishCollisionChanged(
        const QString& sourceId,
        const QString& detectorId) const
    {
        if(m_callbacks.publishCollisionChanged) {
            m_callbacks.publishCollisionChanged(sourceId, detectorId);
        }
    }

    void CollisionDetectorWorkbenchController::showStatus(const QString& message, int timeoutMs) const
    {
        if(m_callbacks.statusMessage) {
            m_callbacks.statusMessage(message, timeoutMs);
        }
    }

}
