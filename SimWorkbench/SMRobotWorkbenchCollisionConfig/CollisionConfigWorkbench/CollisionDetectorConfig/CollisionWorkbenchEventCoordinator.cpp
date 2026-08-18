#include "CollisionWorkbenchEventCoordinator.h"

namespace robot_qt_viewer
{
    void CollisionWorkbenchEventCoordinator::dispatchEvent(
        const RobotQtViewerEvent& event,
        const Handlers& handlers)
    {
        switch(event.kind) {
        case RobotQtViewerEventKind::CollisionChanged:
        case RobotQtViewerEventKind::ProjectDocumentChanged:
        case RobotQtViewerEventKind::ViewportReloaded:
        case RobotQtViewerEventKind::RobotRuntimeChanged:
            if(handlers.refreshInspector) {
                handlers.refreshInspector();
            }
            break;
        case RobotQtViewerEventKind::SelectionChanged:
            if(handlers.refreshSelectionDependentViews) {
                handlers.refreshSelectionDependentViews();
            }
            break;
        default:
            break;
        }
    }
}
