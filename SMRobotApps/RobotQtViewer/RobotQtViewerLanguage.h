#pragma once

#include <QString>

namespace robot_qt_viewer
{
    enum class LanguageKind
    {
        English,
        Chinese
    };

    class LanguageManager
    {
    public:
        static LanguageKind savedLanguage();
        static LanguageKind languageFromName(const QString& name);
        static QString languageName(LanguageKind language);
        static QString languageDisplayName(LanguageKind language);
        static QString text(LanguageKind language, const QString& key);
        static void save(LanguageKind language);
    };
}
