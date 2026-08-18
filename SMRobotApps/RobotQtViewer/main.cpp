#include "MainWindow.h"
#include "RobotQtViewerTheme.h"

#include <CustomLog/CustomLog.h>

#include <QApplication>
#include <QIcon>
#include <QMessageBox>
#include <QSurfaceFormat>
#include <QTimer>

#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <map>
#include <string>

namespace
{
    void initializeLogging()
    {
        std::map<std::string, std::pair<bool, bool>> logInfo = {
            {"rs2026", {true, true}},
            {"SMRobot", {true, true}},
            {"RobotIO", {true, true}},
            {"AssetCore", {true, true}},
        };

        CustomLog::init(logInfo);
        CustomLog::set_level(CustomLog::Level::debug);
        CustomLog::rotate_all();
    }

    std::string argumentValue(int argc, char* argv[], const std::string& name)
    {
        for(int i = 1; i + 1 < argc; ++i) {
            if(argv[i] != nullptr && name == argv[i]) {
                return argv[i + 1] != nullptr ? argv[i + 1] : std::string();
            }
        }
        return std::string();
    }

    int intArgumentValue(int argc, char* argv[], const std::string& name, int fallback)
    {
        const std::string value = argumentValue(argc, argv, name);
        if(value.empty()) {
            return fallback;
        }
        try {
            return std::stoi(value);
        } catch(...) {
            return fallback;
        }
    }

    bool hasArgument(int argc, char* argv[], const std::string& name)
    {
        for(int i = 1; i < argc; ++i) {
            if(argv[i] != nullptr && name == argv[i]) {
                return true;
            }
        }
        return false;
    }

    bool isAuthorizationFailure(const std::exception& error)
    {
        return std::string(error.what()).find("authorization failed") != std::string::npos;
    }

    void reportStartupFailure(const std::exception& error)
    {
        const bool authorizationFailure = isAuthorizationFailure(error);
        const QString message = authorizationFailure
            ? QStringLiteral("This computer is not authorized.")
            : QStringLiteral("RobotQtViewer failed to start.");

        LOG_ERROR("rs2026") << message.toStdString() << " " << error.what();
        std::cerr << message.toStdString() << " " << error.what() << '\n';
        QMessageBox::critical(
            nullptr,
            authorizationFailure
                ? QStringLiteral("Authorization Failed")
                : QStringLiteral("Startup Failed"),
            message);
    }
}

int main(int argc, char* argv[])
{
    QApplication::setAttribute(Qt::AA_ShareOpenGLContexts);

    QSurfaceFormat format;
    format.setVersion(4, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    QSurfaceFormat::setDefaultFormat(format);

    QApplication app(argc, argv);
    QApplication::setOrganizationName("RS2026");
    QApplication::setApplicationName("RobotQtViewer");
    QApplication::setWindowIcon(QIcon(QStringLiteral(":/RobotQtViewer/icons/app/robot_qt_viewer.png")));
    robot_qt_viewer::ThemeManager::applySaved(app);
    initializeLogging();

    QObject::connect(&app, &QCoreApplication::aboutToQuit, []() {
        CustomLog::shutdown();
    });

    try {
        MainWindow window;
        window.resize(1760, 920);
        window.show();
        robot_qt_viewer::ThemeManager::applyNativeWindowFrame(
            window,
            robot_qt_viewer::ThemeManager::savedTheme());

        const int smokeExitMs = intArgumentValue(argc, argv, "--smoke-exit-ms", 0);
        if(smokeExitMs > 0) {
            QTimer::singleShot(smokeExitMs, &app, &QCoreApplication::quit);
        }

        const std::string profileProject = argumentValue(argc, argv, "--profile-project");
        if(!profileProject.empty()) {
            const int exitDelayMs = intArgumentValue(argc, argv, "--profile-exit-ms", 1500);
            const int repeatCount = intArgumentValue(argc, argv, "--profile-repeat", 1);
            const bool enableCollisionAfterLoad = hasArgument(argc, argv, "--profile-enable-collision");
            const std::string profileGenerateObjectCoacd =
                argumentValue(argc, argv, "--profile-generate-object-coacd");
            QTimer::singleShot(
                0,
                &window,
                [&window,
                 profileProject,
                 exitDelayMs,
                 repeatCount,
                 enableCollisionAfterLoad,
                 profileGenerateObjectCoacd]() {
                window.openProjectPathForProfiling(
                    std::filesystem::path(profileProject),
                    exitDelayMs,
                    repeatCount,
                    enableCollisionAfterLoad,
                    QString::fromStdString(profileGenerateObjectCoacd));
            });
        }

        return app.exec();
    } catch(const std::exception& error) {
        reportStartupFailure(error);
    }

    CustomLog::shutdown();
    return EXIT_FAILURE;
}
