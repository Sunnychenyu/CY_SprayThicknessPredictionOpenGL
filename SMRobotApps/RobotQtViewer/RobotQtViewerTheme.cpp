#include "RobotQtViewerTheme.h"

#include <QApplication>
#include <QPalette>
#include <QSettings>
#include <QStyleFactory>
#include <QWidget>

#ifdef Q_OS_WIN
#include <dwmapi.h>
#include <windows.h>
#endif

namespace robot_qt_viewer
{
    namespace
    {
        constexpr const char* kSettingsKey = "ui/theme";

#ifdef Q_OS_WIN
        constexpr DWORD kDwmUseImmersiveDarkMode = 20;
        constexpr DWORD kDwmUseImmersiveDarkModeBefore20H1 = 19;
        constexpr DWORD kDwmWindowBorderColor = 34;
        constexpr DWORD kDwmCaptionColor = 35;
        constexpr DWORD kDwmTextColor = 36;

        COLORREF toColorRef(const QColor& color)
        {
            return RGB(color.red(), color.green(), color.blue());
        }

        void setDwmColor(HWND handle, DWORD attribute, const QColor& color)
        {
            const COLORREF colorRef = toColorRef(color);
            DwmSetWindowAttribute(handle, attribute, &colorRef, sizeof(colorRef));
        }
#endif

        QPalette makePalette(
            const QColor& window,
            const QColor& panel,
            const QColor& text,
            const QColor& disabledText,
            const QColor& base,
            const QColor& alternateBase,
            const QColor& button,
            const QColor& highlight,
            const QColor& highlightedText)
        {
            QPalette palette;
            palette.setColor(QPalette::Window, window);
            palette.setColor(QPalette::WindowText, text);
            palette.setColor(QPalette::Base, base);
            palette.setColor(QPalette::AlternateBase, alternateBase);
            palette.setColor(QPalette::ToolTipBase, panel);
            palette.setColor(QPalette::ToolTipText, text);
            palette.setColor(QPalette::Text, text);
            palette.setColor(QPalette::Button, button);
            palette.setColor(QPalette::ButtonText, text);
            palette.setColor(QPalette::BrightText, QColor(255, 80, 80));
            palette.setColor(QPalette::Highlight, highlight);
            palette.setColor(QPalette::HighlightedText, highlightedText);
            palette.setColor(QPalette::Disabled, QPalette::WindowText, disabledText);
            palette.setColor(QPalette::Disabled, QPalette::Text, disabledText);
            palette.setColor(QPalette::Disabled, QPalette::ButtonText, disabledText);
            return palette;
        }

        QString darkStyleSheet(
            const QString& window,
            const QString& panel,
            const QString& base,
            const QString& alternate,
            const QString& border,
            const QString& text,
            const QString& muted,
            const QString& accent,
            const QString& button,
            const QString& buttonHover)
        {
            return QString(
                "QMainWindow, QDialog { background: %1; color: %6; }"
                "QWidget { color: %6; font-size: 10pt; }"
                "QMenuBar { background: %2; color: %6; border-bottom: 1px solid %5; }"
                "QMenuBar::item { padding: 6px 12px; }"
                "QMenuBar::item:selected { background: %10; }"
                "QMenu { background: %2; color: %6; border: 1px solid %5; }"
                "QMenu::item:selected { background: %8; color: #ffffff; }"
                "QToolBar { background: %2; border: 0; border-bottom: 1px solid %5; spacing: 6px; padding: 6px; }"
                "QToolButton, QPushButton { background: %9; color: %6; border: 1px solid %5; border-radius: 3px; padding: 6px 10px; }"
                "QToolButton:hover, QPushButton:hover { background: %10; border-color: %8; }"
                "QToolButton:checked { background: #1f5f49; border-color: #39d98a; }"
                "QDockWidget { color: %6; }"
                "QDockWidget::title { background: %2; padding: 8px; border: 1px solid %5; }"
                "QLabel[panelTitle=\"true\"] { color: %8; font-weight: 600; padding: 10px 0 4px 0; }"
                "QTreeWidget, QListWidget, QTableWidget, QTextEdit, QPlainTextEdit { background: %3; color: %6; border: 1px solid %5; selection-background-color: %8; alternate-background-color: %4; }"
                "QTreeWidget::item, QListWidget::item { min-height: 24px; padding: 3px; }"
                "QTabWidget::pane { border: 1px solid %5; background: %3; }"
                "QTabBar::tab { background: %2; color: %7; padding: 7px 12px; border: 1px solid %5; border-bottom: 0; }"
                "QTabBar::tab:selected { background: %3; color: %6; }"
                "QTabBar::tab:hover { color: %8; }"
                "QHeaderView::section { background: %2; color: %7; border: 0; padding: 6px; }"
                "QScrollArea { background: %3; border: 1px solid %5; }"
                "QScrollArea QWidget { background: %3; }"
                "QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox { background: %3; color: %6; border: 1px solid %5; border-radius: 3px; padding: 4px 6px; selection-background-color: %8; }"
                "QLineEdit:focus, QDoubleSpinBox:focus, QSpinBox:focus, QComboBox:focus { border-color: %8; }"
                "QCheckBox, QRadioButton, QGroupBox { color: %6; spacing: 6px; }"
                "QGroupBox { border: 1px solid %5; border-radius: 3px; margin-top: 10px; padding-top: 10px; }"
                "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }"
                "QFrame[jointRow=\"true\"] { background: %4; border: 1px solid %5; border-radius: 3px; }"
                "QLabel[jointName=\"true\"] { color: %6; font-weight: 600; }"
                "QSlider::groove:horizontal { height: 4px; background: %5; border-radius: 2px; }"
                "QSlider::handle:horizontal { background: %8; width: 14px; margin: -5px 0; border-radius: 7px; }"
                "QStatusBar { background: %3; color: %7; border-top: 1px solid %5; }")
                .arg(window)
                .arg(panel)
                .arg(base)
                .arg(alternate)
                .arg(border)
                .arg(text)
                .arg(muted)
                .arg(accent)
                .arg(button)
                .arg(buttonHover);
        }

        QString lightStyleSheet()
        {
            return QString(
                "QMainWindow, QDialog { background: #f4f6f8; color: #1d252d; }"
                "QWidget { color: #1d252d; font-size: 10pt; }"
                "QMenuBar, QToolBar { background: #ffffff; color: #1d252d; border-bottom: 1px solid #d7dde4; spacing: 6px; padding: 6px; }"
                "QMenu { background: #ffffff; color: #1d252d; border: 1px solid #c9d1da; }"
                "QMenu::item:selected { background: #2f80ed; color: #ffffff; }"
                "QPushButton, QToolButton { background: #ffffff; color: #1d252d; border: 1px solid #b8c2cc; border-radius: 3px; padding: 6px 10px; }"
                "QPushButton:hover, QToolButton:hover { background: #eaf2ff; border-color: #2f80ed; }"
                "QDockWidget::title { background: #ffffff; padding: 8px; border: 1px solid #d7dde4; }"
                "QLabel[panelTitle=\"true\"] { color: #155e9f; font-weight: 600; padding: 10px 0 4px 0; }"
                "QTreeWidget, QListWidget, QTableWidget, QTextEdit, QPlainTextEdit { background: #ffffff; color: #1d252d; border: 1px solid #c9d1da; selection-background-color: #2f80ed; alternate-background-color: #f1f4f7; }"
                "QTreeWidget::item, QListWidget::item { min-height: 24px; padding: 3px; }"
                "QTabWidget::pane { border: 1px solid #c9d1da; background: #ffffff; }"
                "QTabBar::tab { background: #e8edf2; color: #46515d; padding: 7px 12px; border: 1px solid #c9d1da; border-bottom: 0; }"
                "QTabBar::tab:selected { background: #ffffff; color: #1d252d; }"
                "QHeaderView::section { background: #edf1f5; color: #46515d; border: 0; padding: 6px; }"
                "QScrollArea { background: #ffffff; border: 1px solid #c9d1da; }"
                "QLineEdit, QDoubleSpinBox, QSpinBox, QComboBox { background: #ffffff; color: #1d252d; border: 1px solid #b8c2cc; border-radius: 3px; padding: 4px 6px; selection-background-color: #2f80ed; }"
                "QGroupBox { border: 1px solid #c9d1da; border-radius: 3px; margin-top: 10px; padding-top: 10px; }"
                "QGroupBox::title { subcontrol-origin: margin; left: 8px; padding: 0 4px; }"
                "QFrame[jointRow=\"true\"] { background: #ffffff; border: 1px solid #d7dde4; border-radius: 3px; }"
                "QSlider::groove:horizontal { height: 4px; background: #c9d1da; border-radius: 2px; }"
                "QSlider::handle:horizontal { background: #2f80ed; width: 14px; margin: -5px 0; border-radius: 7px; }"
                "QStatusBar { background: #ffffff; color: #586575; border-top: 1px solid #d7dde4; }");
        }
    }

    ThemeKind ThemeManager::savedTheme()
    {
        QSettings settings;
        return themeFromName(settings.value(kSettingsKey, themeName(ThemeKind::Modern)).toString());
    }

    ThemeKind ThemeManager::themeFromName(const QString& name)
    {
        const QString normalized = name.trimmed().toLower();
        if (normalized == "light")
            return ThemeKind::Light;
        if (normalized == "dark")
            return ThemeKind::Dark;
        return ThemeKind::Modern;
    }

    QString ThemeManager::themeName(ThemeKind theme)
    {
        switch (theme)
        {
        case ThemeKind::Light:
            return "Light";
        case ThemeKind::Dark:
            return "Dark";
        case ThemeKind::Modern:
        default:
            return "Modern";
        }
    }

    void ThemeManager::apply(QApplication& app, ThemeKind theme)
    {
        app.setStyle(QStyleFactory::create("Fusion"));

        switch (theme)
        {
        case ThemeKind::Light:
            app.setPalette(makePalette(
                QColor("#f4f6f8"),
                QColor("#ffffff"),
                QColor("#1d252d"),
                QColor("#8a96a3"),
                QColor("#ffffff"),
                QColor("#f1f4f7"),
                QColor("#ffffff"),
                QColor("#2f80ed"),
                QColor("#ffffff")));
            app.setStyleSheet(lightStyleSheet());
            break;
        case ThemeKind::Dark:
            app.setPalette(makePalette(
                QColor("#111418"),
                QColor("#191f26"),
                QColor("#e3e8ef"),
                QColor("#6f7a86"),
                QColor("#0b0f14"),
                QColor("#131920"),
                QColor("#232b34"),
                QColor("#4e9af1"),
                QColor("#ffffff")));
            app.setStyleSheet(darkStyleSheet(
                "#111418", "#191f26", "#0b0f14", "#131920", "#2b3440",
                "#e3e8ef", "#9ca8b5", "#4e9af1", "#232b34", "#2f3a46"));
            break;
        case ThemeKind::Modern:
        default:
            app.setPalette(makePalette(
                QColor("#171b20"),
                QColor("#20262d"),
                QColor("#d8dee6"),
                QColor("#727e8a"),
                QColor("#11151a"),
                QColor("#161c22"),
                QColor("#29323b"),
                QColor("#2f80ed"),
                QColor("#ffffff")));
            app.setStyleSheet(darkStyleSheet(
                "#171b20", "#20262d", "#11151a", "#161c22", "#2e3944",
                "#d8dee6", "#aeb8c4", "#7cc7ff", "#29323b", "#34404b"));
            break;
        }
    }

    void ThemeManager::applySaved(QApplication& app)
    {
        apply(app, savedTheme());
    }

    void ThemeManager::applyNativeWindowFrame(QWidget& window, ThemeKind theme)
    {
#ifdef Q_OS_WIN
        HWND handle = reinterpret_cast<HWND>(window.winId());
        if(handle == nullptr) {
            return;
        }

        const BOOL useDarkFrame = theme != ThemeKind::Light ? TRUE : FALSE;
        if(FAILED(DwmSetWindowAttribute(
            handle,
            kDwmUseImmersiveDarkMode,
            &useDarkFrame,
            sizeof(useDarkFrame)))) {
            DwmSetWindowAttribute(
                handle,
                kDwmUseImmersiveDarkModeBefore20H1,
                &useDarkFrame,
                sizeof(useDarkFrame));
        }

        switch(theme) {
        case ThemeKind::Light:
            setDwmColor(handle, kDwmCaptionColor, QColor("#f4f6f8"));
            setDwmColor(handle, kDwmTextColor, QColor("#1d252d"));
            setDwmColor(handle, kDwmWindowBorderColor, QColor("#c9d1da"));
            break;
        case ThemeKind::Dark:
            setDwmColor(handle, kDwmCaptionColor, QColor("#191f26"));
            setDwmColor(handle, kDwmTextColor, QColor("#f2f6fb"));
            setDwmColor(handle, kDwmWindowBorderColor, QColor("#2b3440"));
            break;
        case ThemeKind::Modern:
        default:
            setDwmColor(handle, kDwmCaptionColor, QColor("#20262d"));
            setDwmColor(handle, kDwmTextColor, QColor("#f2f6fb"));
            setDwmColor(handle, kDwmWindowBorderColor, QColor("#2e3944"));
            break;
        }
#else
        Q_UNUSED(window);
        Q_UNUSED(theme);
#endif
    }

    void ThemeManager::save(ThemeKind theme)
    {
        QSettings settings;
        settings.setValue(kSettingsKey, themeName(theme));
    }
}
