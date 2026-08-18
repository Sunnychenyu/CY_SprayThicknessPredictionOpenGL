#pragma once

#include <QString>

class CollisionResultsWidget;

namespace robot_qt_viewer
{
    class CollisionWorkbenchServices;
    class CollisionResultDocumentFacade;
    class RobotQtViewerDocumentContext;

    class CollisionResultViewController
    {
    public:
        CollisionResultViewController(
            CollisionResultsWidget& widget,
            RobotQtViewerDocumentContext& context,
            CollisionWorkbenchServices& appServices,
            CollisionResultDocumentFacade& documentFacade);

        void refresh(const QString& detectorId);

    private:
        CollisionResultsWidget& m_widget;
        RobotQtViewerDocumentContext& m_context;
        CollisionWorkbenchServices& m_appServices;
        CollisionResultDocumentFacade& m_documentFacade;
    };
}
