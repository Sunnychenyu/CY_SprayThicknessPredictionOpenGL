#include "CollisionRuntimeResultsWidget.h"

#include "CollisionResultsWidget.h"

#include <QVBoxLayout>

CollisionRuntimeResultsWidget::CollisionRuntimeResultsWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);

    m_resultsWidget = new CollisionResultsWidget(this);
    layout->addWidget(m_resultsWidget, 1);
}

void CollisionRuntimeResultsWidget::setDisplayMode(CollisionResultsWidget::DisplayMode mode)
{
    if(m_resultsWidget != nullptr) {
        m_resultsWidget->setDisplayMode(mode);
    }
}

CollisionResultsWidget* CollisionRuntimeResultsWidget::resultsWidget() const
{
    return m_resultsWidget;
}
