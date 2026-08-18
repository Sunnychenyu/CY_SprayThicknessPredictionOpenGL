#pragma once

#include <QString>

namespace robot_qt_viewer
{
    class CollisionQualityMessageState
    {
    public:
        const QString& message() const;
        void setMessage(const QString& message);
        void clear();

    private:
        QString m_message;
    };
}
