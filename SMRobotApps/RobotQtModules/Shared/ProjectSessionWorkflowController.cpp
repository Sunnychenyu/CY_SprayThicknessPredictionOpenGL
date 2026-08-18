#include "ProjectSessionWorkflowController.h"

#include "RobotQtViewerDocumentContext.h"
#include "RobotQtViewerDocumentController.h"

#include <SimulationProject/ProjectSession.h>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace
{
    double elapsedMilliseconds(const std::chrono::steady_clock::time_point& start)
    {
        const auto elapsed = std::chrono::steady_clock::now() - start;
        return std::chrono::duration<double, std::milli>(elapsed).count();
    }

    void printProfileRow(const char* stage, double ms, const QString& detail)
    {
        std::ostringstream out;
        out << "| " << std::left << std::setw(32) << stage
            << " | " << std::right << std::setw(10) << std::fixed << std::setprecision(2) << ms
            << " ms | " << detail.toStdString() << "\n";
        std::cout << out.str();
    }
}

namespace robot_qt_viewer
{
    ProjectSessionWorkflowController::ProjectSessionWorkflowController(RobotQtViewerDocumentContext& context)
        : m_context(context)
    {
    }

    ProjectSessionWorkflowResult ProjectSessionWorkflowController::resetNew(const QString& sourceId)
    {
        ProjectSessionWorkflowResult result;
        result.sourceId = sourceId;

        m_context.projectSession().resetNew();
        m_context.documentController().publishProjectOpened(sourceId);
        m_context.documentController().publishDocumentChanged(sourceId, false);

        result.success = true;
        result.shouldReloadViewport = true;
        result.message = QStringLiteral("New project");
        return result;
    }

    ProjectSessionWorkflowResult ProjectSessionWorkflowController::loadFromPath(
        const std::filesystem::path& path,
        const QString& sourceId)
    {
        return loadProject(path, sourceId, false);
    }

    ProjectSessionWorkflowResult ProjectSessionWorkflowController::loadStartupProject(
        const std::filesystem::path& path,
        const QString& sourceId)
    {
        return loadProject(path, sourceId, true);
    }

    ProjectSessionWorkflowResult ProjectSessionWorkflowController::saveToPath(
        const std::filesystem::path& path,
        bool saveAsV3,
        const QString& sourceId)
    {
        ProjectSessionWorkflowResult result;
        result.sourceId = sourceId;

        std::string error;
        if(!m_context.projectSession().saveToPath(path, saveAsV3, &error)) {
            result.message = QString("Save failed: %1").arg(QString::fromStdString(error));
            return result;
        }

        m_context.documentController().publishProjectSaved(sourceId);

        result.success = true;
        result.message = QString(saveAsV3 ? "Saved v3 %1" : "Saved %1").arg(QString::fromStdWString(path.wstring()));
        return result;
    }

    ProjectSessionWorkflowResult ProjectSessionWorkflowController::loadProject(
        const std::filesystem::path& path,
        const QString& sourceId,
        bool requireSaveAs)
    {
        const auto profileStart = std::chrono::steady_clock::now();
        ProjectSessionWorkflowResult result;
        result.sourceId = sourceId;

        std::string error;
        bool migratedCollisionDetectors = false;
        const auto loadStart = std::chrono::steady_clock::now();
        if(!m_context.projectSession().loadFromPath(path, &migratedCollisionDetectors, &error)) {
            result.message = QString("Open failed: %1").arg(QString::fromStdString(error));
            printProfileRow("ProjectSession load JSON", elapsedMilliseconds(loadStart), result.message);
            return result;
        }
        printProfileRow(
            "ProjectSession load JSON",
            elapsedMilliseconds(loadStart),
            QString("robots=%1 objects=%2 pointClouds=%3 mounts=%4")
                .arg(static_cast<int>(m_context.document().robots.size()))
                .arg(static_cast<int>(m_context.document().objects.size()))
                .arg(static_cast<int>(m_context.document().pointClouds.size()))
                .arg(static_cast<int>(m_context.document().robotMounts.size())));

        if(requireSaveAs) {
            m_context.projectSession().setRequiresSaveAs(true);
        }

        const auto publishStart = std::chrono::steady_clock::now();
        m_context.documentController().publishProjectOpened(sourceId);
        m_context.documentController().publishDirtyChanged(sourceId);
        printProfileRow("ProjectSession publish", elapsedMilliseconds(publishStart), sourceId);

        result.success = true;
        result.shouldReloadViewport = true;
        result.migratedCollisionDetectors = migratedCollisionDetectors;
        result.message = migratedCollisionDetectors
            ? QString("Opened %1; legacy collision converted to detectors").arg(QString::fromStdWString(path.wstring()))
            : QString("Opened %1").arg(QString::fromStdWString(path.wstring()));
        printProfileRow("ProjectSession total", elapsedMilliseconds(profileStart), result.message);
        return result;
    }
}
