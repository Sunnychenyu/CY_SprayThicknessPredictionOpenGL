#pragma once

#include "RobotQtViewerEvents.h"

#include <functional>

namespace robot_qt_viewer
{
    class CollisionWorkbenchEventCoordinator
    {
    public:
        struct Handlers
        {
            std::function<void()> refreshInspector;
            std::function<void()> refreshSelectionDependentViews;
        };

        static void dispatchEvent(
            const RobotQtViewerEvent& event,
            const Handlers& handlers);
    };
}
