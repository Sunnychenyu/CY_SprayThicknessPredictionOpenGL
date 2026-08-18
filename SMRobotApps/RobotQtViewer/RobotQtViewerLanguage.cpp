#include "RobotQtViewerLanguage.h"

#include <QHash>
#include <QSettings>
#include <QStringLiteral>

namespace robot_qt_viewer
{
namespace
{
    constexpr const char* kLanguageSettingsKey = "ui/language";

    QString zh(const wchar_t* text)
    {
        return QString::fromWCharArray(text);
    }

    QHash<QString, QString> makeEnglishText()
    {
        QHash<QString, QString> text;
        text.insert("window.title", "RobotQtViewer");
        text.insert("menu.file", "File");
        text.insert("menu.view", "View");
        text.insert("menu.window", "Window");
        text.insert("menu.theme", "Theme");
        text.insert("menu.language", "Language");
        text.insert("menu.cameraViews", "Camera Views");
        text.insert("theme.Modern", "Modern");
        text.insert("theme.Dark", "Dark");
        text.insert("theme.Light", "Light");
        text.insert("language.English", "English");
        text.insert("language.Chinese", "Chinese");
        text.insert("toolbar.simulation", "Simulation");
        text.insert("toolbar.sceneEdit", "Scene Edit");
        text.insert("toolbar.robotEdit", "Robot Edit");
        text.insert("toolbar.view", "View");
        text.insert("toolbar.run", "Run");
        text.insert("toolbar.modes", "Modes");
        text.insert("action.loadRobot", "Load Robot");
        text.insert("action.newProject", "New Project");
        text.insert("action.openProject", "Open Project");
        text.insert("action.saveProject", "Save Project");
        text.insert("action.saveProjectAs", "Save Project As");
        text.insert("action.saveProjectAsV3", "Save Project As System");
        text.insert("action.importRobotPackage", "Import Robot Package");
        text.insert("action.exportRobotPackage", "Export Selected Robot Package");
        text.insert("action.saveCollisionOverrides", "Save Collision Overrides To Project");
        text.insert("action.saveCollisionSidecar", "Save Collision Overrides As Sidecar");
        text.insert("action.exportCollisionUrdf", "Export Robot URDF With Collision");
        text.insert("action.importRobot", "Import Robot");
        text.insert("action.importObject", "Import Object");
        text.insert("action.importPointCloud", "Import Point Cloud");
        text.insert("action.deleteSelectedItem", "Delete Selected Item");
        text.insert("action.saveImage", "Save Image");
        text.insert("action.resetCamera", "Reset Camera");
        text.insert("action.viewOrientation", "View Orientation");
        text.insert("action.enterViewportFullscreen", "Enter Full Screen (F11)");
        text.insert("action.exitViewportFullscreen", "Exit Full Screen (F11)");
        text.insert("action.mountEdit", "Mount Edit");
        text.insert("action.addLinkMount", "Add Link Mount");
        text.insert("action.collisionEdit", "Collision Edit");
        text.insert("action.projectAssemblyMode", "Project Assembly");
        text.insert("action.collisionConfigMode", "Collision Config");
        text.insert("action.robotRunMode", "Robot Run");
        text.insert("action.trajectoryPlanningMode", "Motion Planning");
        text.insert("action.sprayProcessMode", "Spray Process");
        text.insert("action.coatingAnalysisMode", "Coating Analysis");
        text.insert("action.digitalTwinMode", "Digital Twin");
        text.insert("action.cameraView.home", "Fit / Home");
        text.insert("action.cameraView.isometric", "Isometric");
        text.insert("action.cameraView.front", "Front");
        text.insert("action.cameraView.back", "Back");
        text.insert("action.cameraView.left", "Left");
        text.insert("action.cameraView.right", "Right");
        text.insert("action.cameraView.top", "Top");
        text.insert("action.cameraView.bottom", "Bottom");
        text.insert("action.cameraView.showOverlay", "Show View Orientation");
        text.insert("action.cameraView.hideOverlay", "Hide View Orientation");
        text.insert("action.collisionGeometry", "Collision Geometry");
        text.insert("action.collisionQueries", "Collision Detection");
        text.insert("action.robotRunDetails", "Robot Run Details");
        text.insert("status.ready", "Ready");
        return text;
    }

    QHash<QString, QString> makeChineseText()
    {
        QHash<QString, QString> text;
        text.insert("window.title", "RobotQtViewer");
        text.insert("menu.file", zh(L"\u6587\u4ef6"));
        text.insert("menu.view", zh(L"\u89c6\u56fe"));
        text.insert("menu.window", zh(L"\u7a97\u53e3"));
        text.insert("menu.theme", zh(L"\u4e3b\u9898"));
        text.insert("menu.language", zh(L"\u8bed\u8a00"));
        text.insert("menu.cameraViews", zh(L"\u76f8\u673a\u89c6\u89d2"));
        text.insert("theme.Modern", zh(L"\u73b0\u4ee3"));
        text.insert("theme.Dark", zh(L"\u6df1\u8272"));
        text.insert("theme.Light", zh(L"\u6d45\u8272"));
        text.insert("language.English", zh(L"\u82f1\u6587"));
        text.insert("language.Chinese", zh(L"\u4e2d\u6587"));
        text.insert("toolbar.simulation", zh(L"\u4eff\u771f"));
        text.insert("toolbar.sceneEdit", zh(L"\u573a\u666f\u7f16\u8f91"));
        text.insert("toolbar.robotEdit", zh(L"\u673a\u5668\u4eba\u7f16\u8f91"));
        text.insert("toolbar.view", zh(L"\u89c6\u56fe"));
        text.insert("toolbar.run", zh(L"\u8fd0\u884c"));
        text.insert("toolbar.modes", zh(L"\u6a21\u5f0f"));
        text.insert("action.loadRobot", zh(L"\u52a0\u8f7d\u673a\u5668\u4eba"));
        text.insert("action.newProject", zh(L"\u65b0\u5efa\u9879\u76ee"));
        text.insert("action.openProject", zh(L"\u6253\u5f00\u9879\u76ee"));
        text.insert("action.saveProject", zh(L"\u4fdd\u5b58\u9879\u76ee"));
        text.insert("action.saveProjectAs", zh(L"\u9879\u76ee\u53e6\u5b58\u4e3a"));
        text.insert("action.saveProjectAsV3", zh(L"\u9879\u76ee\u53e6\u5b58\u4e3a\u7cfb\u7edf\u6587\u4ef6"));
        text.insert("action.importRobotPackage", zh(L"\u5bfc\u5165\u673a\u5668\u4eba\u5305"));
        text.insert("action.exportRobotPackage", zh(L"\u5bfc\u51fa\u9009\u4e2d\u673a\u5668\u4eba\u5305"));
        text.insert("action.saveCollisionOverrides", zh(L"\u4fdd\u5b58\u78b0\u649e\u8986\u76d6\u5230\u9879\u76ee"));
        text.insert("action.saveCollisionSidecar", zh(L"\u78b0\u649e\u8986\u76d6\u53e6\u5b58\u4e3a\u65c1\u8def\u6587\u4ef6"));
        text.insert("action.exportCollisionUrdf", zh(L"\u5bfc\u51fa\u5e26\u78b0\u649e\u7684\u673a\u5668\u4eba URDF"));
        text.insert("action.importRobot", zh(L"\u5bfc\u5165\u673a\u5668\u4eba"));
        text.insert("action.importObject", zh(L"\u5bfc\u5165\u5bf9\u8c61"));
        text.insert("action.importPointCloud", zh(L"\u5bfc\u5165\u70b9\u4e91"));
        text.insert("action.deleteSelectedItem", zh(L"\u5220\u9664\u9009\u4e2d\u9879"));
        text.insert("action.saveImage", zh(L"\u4fdd\u5b58\u56fe\u50cf"));
        text.insert("action.resetCamera", zh(L"\u91cd\u7f6e\u76f8\u673a"));
        text.insert("action.viewOrientation", zh(L"\u89c6\u89d2\u65b9\u5411"));
        text.insert("action.enterViewportFullscreen", zh(L"\u8fdb\u5165\u5168\u5c4f (F11)"));
        text.insert("action.exitViewportFullscreen", zh(L"\u9000\u51fa\u5168\u5c4f (F11)"));
        text.insert("action.mountEdit", zh(L"Mount \u7f16\u8f91"));
        text.insert("action.addLinkMount", zh(L"\u6dfb\u52a0 Link Mount"));
        text.insert("action.collisionEdit", zh(L"\u78b0\u649e\u7f16\u8f91"));
        text.insert("action.projectAssemblyMode", zh(L"\u9879\u76ee\u88c5\u914d"));
        text.insert("action.collisionConfigMode", zh(L"\u78b0\u649e\u914d\u7f6e"));
        text.insert("action.robotRunMode", zh(L"\u673a\u5668\u4eba\u8fd0\u884c"));
        text.insert("action.trajectoryPlanningMode", zh(L"\u8fd0\u52a8\u89c4\u5212"));
        text.insert("action.sprayProcessMode", zh(L"\u55b7\u6d82\u5de5\u827a"));
        text.insert("action.coatingAnalysisMode", zh(L"\u6d82\u5c42\u5206\u6790"));
        text.insert("action.digitalTwinMode", zh(L"\u6570\u5b57\u5b6a\u751f"));
        text.insert("action.cameraView.home", zh(L"\u9002\u5408\u7a97\u53e3 / \u4e3b\u9875"));
        text.insert("action.cameraView.isometric", zh(L"\u7b49\u8f74\u6d4b"));
        text.insert("action.cameraView.front", zh(L"\u524d\u89c6"));
        text.insert("action.cameraView.back", zh(L"\u540e\u89c6"));
        text.insert("action.cameraView.left", zh(L"\u5de6\u89c6"));
        text.insert("action.cameraView.right", zh(L"\u53f3\u89c6"));
        text.insert("action.cameraView.top", zh(L"\u4e0a\u89c6"));
        text.insert("action.cameraView.bottom", zh(L"\u4e0b\u89c6"));
        text.insert("action.cameraView.showOverlay", zh(L"\u663e\u793a\u89c6\u89d2\u63a7\u4ef6"));
        text.insert("action.cameraView.hideOverlay", zh(L"\u9690\u85cf\u89c6\u89d2\u63a7\u4ef6"));
        text.insert("action.collisionGeometry", zh(L"\u78b0\u649e\u51e0\u4f55"));
        text.insert("action.collisionQueries", zh(L"\u78b0\u649e\u68c0\u6d4b"));
        text.insert("action.robotRunDetails", zh(L"\u673a\u5668\u4eba\u8fd0\u884c\u8be6\u60c5"));
        text.insert("status.ready", zh(L"\u5c31\u7eea"));
        return text;
    }

    const QHash<QString, QString>& englishText()
    {
        static const QHash<QString, QString> text = makeEnglishText();
        return text;
    }

    const QHash<QString, QString>& chineseText()
    {
        static const QHash<QString, QString> text = makeChineseText();
        return text;
    }
}

LanguageKind LanguageManager::savedLanguage()
{
    QSettings settings;
    return languageFromName(settings.value(kLanguageSettingsKey, "English").toString());
}

LanguageKind LanguageManager::languageFromName(const QString& name)
{
    if(name.compare("Chinese", Qt::CaseInsensitive) == 0 ||
        name.compare("zh", Qt::CaseInsensitive) == 0 ||
        name.compare("zh_CN", Qt::CaseInsensitive) == 0) {
        return LanguageKind::Chinese;
    }

    return LanguageKind::English;
}

QString LanguageManager::languageName(LanguageKind language)
{
    switch(language) {
    case LanguageKind::Chinese:
        return "Chinese";
    case LanguageKind::English:
    default:
        return "English";
    }
}

QString LanguageManager::languageDisplayName(LanguageKind language)
{
    return text(language, QString("language.%1").arg(languageName(language)));
}

QString LanguageManager::text(LanguageKind language, const QString& key)
{
    const QHash<QString, QString>& languageText =
        language == LanguageKind::Chinese ? chineseText() : englishText();
    auto it = languageText.find(key);
    if(it != languageText.end()) {
        return it.value();
    }

    auto fallbackIt = englishText().find(key);
    return fallbackIt == englishText().end() ? key : fallbackIt.value();
}

void LanguageManager::save(LanguageKind language)
{
    QSettings settings;
    settings.setValue(kLanguageSettingsKey, languageName(language));
}
}
