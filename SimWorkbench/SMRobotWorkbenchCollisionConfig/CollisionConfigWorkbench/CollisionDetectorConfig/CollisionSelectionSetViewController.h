#pragma once

#include <QString>

#include <functional>

class CollisionSelectionSetsWidget;

namespace robot_qt_viewer
{
    class CollisionSelectionSetDocumentFacade;
    class RobotQtViewerDocumentContext;

    class CollisionSelectionSetViewController
    {
    public:
        struct Callbacks
        {
            std::function<void(const QString&, int)> statusMessage;
        };

        CollisionSelectionSetViewController(
            CollisionSelectionSetsWidget& widget,
            RobotQtViewerDocumentContext& context,
            CollisionSelectionSetDocumentFacade& documentFacade,
            Callbacks callbacks);

        QString currentSelectionSetId() const;
        void refreshList(const QString& previousId);
        void refreshMemberList();
        void previewMember(bool updating) const;

    private:
        void showStatus(const QString& message, int timeoutMs) const;

        CollisionSelectionSetsWidget& m_widget;
        RobotQtViewerDocumentContext& m_context;
        CollisionSelectionSetDocumentFacade& m_documentFacade;
        Callbacks m_callbacks;
    };
}
