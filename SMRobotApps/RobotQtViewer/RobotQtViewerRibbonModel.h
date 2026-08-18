#pragma once

#include <QStyle>
#include <QString>
#include <QVector>

namespace robot_qt_viewer
{
    struct RobotQtViewerRibbonActionSpec
    {
        QString actionId;
        QString themeIconName;
        QStyle::StandardPixmap fallbackIcon = QStyle::SP_FileIcon;
        bool compact = false;
    };

    struct RobotQtViewerRibbonGroupSpec
    {
        QString groupId;
        QString title;
        QVector<RobotQtViewerRibbonActionSpec> actions;
    };

    struct RobotQtViewerRibbonPageSpec
    {
        QString pageId;
        QString title;
        QVector<RobotQtViewerRibbonGroupSpec> groups;
    };

    struct RobotQtViewerRibbonTexts
    {
        QString toolbarTitle;
        QString homePage = QStringLiteral("Home");
        QString projectGroup = QStringLiteral("Project");
        QString sceneGroup = QStringLiteral("Scene Edit");
        QString robotEditGroup = QStringLiteral("Robot Edit");
        QString viewGroup = QStringLiteral("View");
        QString workbenchGroup = QStringLiteral("Modes");
    };

    struct RobotQtViewerRibbonModel
    {
        QString toolbarTitle;
        QVector<RobotQtViewerRibbonPageSpec> pages;
    };

    RobotQtViewerRibbonModel makeDefaultRobotQtViewerRibbonModel(
        const RobotQtViewerRibbonTexts& texts = RobotQtViewerRibbonTexts());
}
