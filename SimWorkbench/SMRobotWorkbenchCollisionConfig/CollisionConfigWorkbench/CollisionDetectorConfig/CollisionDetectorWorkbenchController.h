#pragma once

#include "CollisionDetectorsViewModel.h"

#include <QString>
#include <QStringList>

#include <functional>

class CollisionWorkbenchPanel;

namespace simulation_project
{
    struct CollisionDetectorDesc;
    struct CollisionSelectionSetMemberDesc;
}

namespace robot_qt_viewer
{
    class CollisionDetectorWorkbenchDocumentFacade;
    class CollisionWorkbenchServices;
    class RobotQtViewerDocumentContext;

    class CollisionDetectorWorkbenchController
    {
    public:
        struct Callbacks
        {
            std::function<bool()> isUpdating;
            std::function<void()> refreshDetectorList;
            std::function<void()> refreshDetectorProperties;
            std::function<void()> refreshDetectorDetails;
            std::function<void(const QString&)> refreshElementList;
            std::function<void(const QString&, const QString&)> publishCollisionChanged;
            std::function<void(const QString&, int)> statusMessage;
        };

        CollisionDetectorWorkbenchController(
            CollisionWorkbenchPanel& panel,
            RobotQtViewerDocumentContext& context,
            CollisionWorkbenchServices& appServices,
            CollisionDetectorWorkbenchDocumentFacade& documentFacade,
            Callbacks callbacks);

        void setDetectorEnabled(const QString& detectorId, bool enabled);
        void handleSelectionChanged();
        void applyDetectorProperties();
        void previewSelection();
        void showSelectedDetectors();
        void removeSelectedDetector();
        void addDetectorFromTaskPanel();
        void editSelectedDetector();
        void addSceneAllDetector();
        void addSelectionMemberToDraftSet(
            const QString& side,
            const QString& displayName,
            const QStringList& robotLinks,
            const simulation_project::CollisionSelectionSetMemberDesc& member);
        void bindDraftSetsToDetector();
        void removePairGenerators(const QVector<int>& generatorIndexes);
        void clearPairScope();
        void previewDraftMember(
            const QString& robotId,
            const QString& linkName,
            const QString& objectId,
            const QString& attachmentId);
        void previewPairScope(
            const QString& robotAId,
            const QString& linkAName,
            const QString& objectAId,
            const QString& attachmentAId,
            const QString& robotBId,
            const QString& linkBName,
            const QString& objectBId,
            const QString& attachmentBId);

    private:
        QString currentDetectorId() const;
        bool isUpdating() const;
        void refreshDetectorList() const;
        void refreshDetectorProperties() const;
        void refreshDetectorDetails() const;
        void refreshElementList(const QString& qualityMessage) const;
        void publishCollisionChanged(const QString& sourceId, const QString& detectorId = QString()) const;
        void showStatus(const QString& message, int timeoutMs) const;

        CollisionWorkbenchPanel& m_panel;
        RobotQtViewerDocumentContext& m_context;
        CollisionWorkbenchServices& m_appServices;
        CollisionDetectorWorkbenchDocumentFacade& m_documentFacade;
        Callbacks m_callbacks;
    };
}
