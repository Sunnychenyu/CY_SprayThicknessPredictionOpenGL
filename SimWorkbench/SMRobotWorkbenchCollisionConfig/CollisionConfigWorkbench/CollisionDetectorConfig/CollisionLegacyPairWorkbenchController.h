#pragma once

#include <QString>

#include <functional>

class CollisionWorkbenchPanel;

namespace robot_qt_viewer
{
    class CollisionLegacyPairDocumentFacade;

    class CollisionLegacyPairWorkbenchController
    {
    public:
        struct Callbacks
        {
            std::function<bool()> isUpdating;
            std::function<void()> viewportReload;
            std::function<void(const QString&)> publishCollisionChanged;
            std::function<void(const QString&, int)> statusMessage;
        };

        CollisionLegacyPairWorkbenchController(
            CollisionWorkbenchPanel& panel,
            CollisionLegacyPairDocumentFacade& documentFacade,
            Callbacks callbacks);

        void refreshPairList();
        bool syncRobotObjectPairs();
        void autoPairAllRobotObjects();
        void setPairEnabled(const QString& robotId, const QString& objectId, bool enabled);

    private:
        bool isUpdating() const;
        void requestViewportReload() const;
        void publishCollisionChanged(const QString& sourceId) const;
        void showStatus(const QString& message, int timeoutMs) const;

        CollisionWorkbenchPanel& m_panel;
        CollisionLegacyPairDocumentFacade& m_documentFacade;
        Callbacks m_callbacks;
    };
}
