#pragma once

#include <QString>

class QApplication;
class QWidget;

namespace robot_qt_viewer
{
    enum class ThemeKind
    {
        Modern,
        Dark,
        Light
    };

    class ThemeManager
    {
    public:
        static ThemeKind savedTheme();
        static ThemeKind themeFromName(const QString& name);
        static QString themeName(ThemeKind theme);
        static void apply(QApplication& app, ThemeKind theme);
        static void applySaved(QApplication& app);
        static void applyNativeWindowFrame(QWidget& window, ThemeKind theme);
        static void save(ThemeKind theme);
    };
}
