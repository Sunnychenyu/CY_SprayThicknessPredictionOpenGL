#pragma once

#include "CollisionDetectorCommandController.h"
#include "CollisionRuntimeViewModel.h"

#include <QString>

#include <vector>

namespace simulation_project
{
    struct ProjectDocument;
    class ProjectSession;
}

namespace robot_qt_viewer
{
    class CollisionWorkbenchServices;

    struct CollisionDetectorPreviewTarget
    {
        enum class Type
        {
            None,
            RobotLink,
            SceneObject,
            MountedAttachment
        };

        Type type = Type::None;
        QString robotId;
        QString linkName;
        QString objectId;
        QString attachmentId;
    };

    struct CollisionDetectorAddResult
    {
        bool success = false;
        QString detectorId;
        QString message;
    };

    class CollisionDetectorWorkbenchDocumentFacade
    {
    public:
        CollisionDetectorWorkbenchDocumentFacade(
            const simulation_project::ProjectDocument& document,
            CollisionWorkbenchServices& appServices);

        bool hasSceneEntities() const;
        bool hasMultipleDetectors() const;
        CollisionDetectorPreviewTarget previewTarget(const QString& detectorId) const;
        QVector<CollisionDetectorListItemView> detectorListItems(
            const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors) const;
        CollisionDetectorPropertiesView detectorProperties(const QString& detectorId) const;
        CollisionDetectorPairsViewModel detectorPairs(
            const QString& detectorId,
            const std::vector<CollisionRuntimeDetectorInfo>& runtimeDetectors) const;

        CollisionDetectorCommandResult setDetectorEnabled(
            RobotQtViewerViewportServices* viewportServices,
            const QString& detectorId,
            bool enabled);

        CollisionDetectorCommandResult applyDetectorProperties(
            RobotQtViewerViewportServices* viewportServices,
            const QString& detectorId,
            const CollisionDetectorPropertiesView& properties);

        CollisionDetectorCommandResult applyDetectorQueryContract(
            RobotQtViewerViewportServices* viewportServices,
            const QString& detectorId,
            const CollisionDetectorQueryContractView& contract);

        CollisionDetectorCommandResult showOnlyDetectors(
            RobotQtViewerViewportServices* viewportServices,
            const QVector<QString>& detectorIds);

        CollisionDetectorCommandResult removeDetector(
            RobotQtViewerViewportServices* viewportServices,
            const QString& detectorId);

        CollisionDetectorCommandResult bindDetectorDraftSets(
            RobotQtViewerViewportServices* viewportServices,
            const QString& detectorId,
            const QVector<CollisionDetectorDraftMemberView>& setA,
            const QVector<CollisionDetectorDraftMemberView>& setB);

        CollisionDetectorCommandResult removeDetectorPairGenerators(
            RobotQtViewerViewportServices* viewportServices,
            const QString& detectorId,
            const QVector<int>& generatorIndexes);

        CollisionDetectorCommandResult clearDetectorPairScope(
            RobotQtViewerViewportServices* viewportServices,
            const QString& detectorId);

        CollisionDetectorAddResult addTaskPanelDefaultDetector();
        CollisionDetectorAddResult addSceneAllDetector();

    private:
        const simulation_project::ProjectDocument& m_document;
        CollisionWorkbenchServices& m_appServices;
    };
}
