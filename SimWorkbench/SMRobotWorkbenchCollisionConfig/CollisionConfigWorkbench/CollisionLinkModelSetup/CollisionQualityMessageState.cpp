#include "CollisionQualityMessageState.h"

namespace robot_qt_viewer
{
    const QString& CollisionQualityMessageState::message() const
    {
        return m_message;
    }

    void CollisionQualityMessageState::setMessage(const QString& message)
    {
        m_message = message;
    }

    void CollisionQualityMessageState::clear()
    {
        m_message.clear();
    }
}
