#pragma once

#include <QString>

#include <functional>

class CollisionLinkModelsWidget;

namespace robot_qt_viewer
{
    class CollisionWorkbenchServices;
    class CollisionModelDocumentFacade;
    class RobotQtViewerDocumentContext;

    class CollisionModelWorkbenchController
    {
    public:
        CollisionModelWorkbenchController(
            CollisionLinkModelsWidget& widget,
            RobotQtViewerDocumentContext& context,
            CollisionWorkbenchServices& appServices,
            CollisionModelDocumentFacade& documentFacade,
            std::function<void(bool, bool)> setLinkPairActionsEnabled);

        void refreshElementList(
            const QString& activeDetectorId,
            const QString& qualityMessage);
        void refreshSummary(
            const QString& activeDetectorId,
            const QString& qualityMessage);
        void focusSelectedTargetInViewport();

    private:
        CollisionLinkModelsWidget& m_widget;
        RobotQtViewerDocumentContext& m_context;
        CollisionWorkbenchServices& m_appServices;
        CollisionModelDocumentFacade& m_documentFacade;
        std::function<void(bool, bool)> m_setLinkPairActionsEnabled;
        bool m_refreshingElementList = false;
    };
}
