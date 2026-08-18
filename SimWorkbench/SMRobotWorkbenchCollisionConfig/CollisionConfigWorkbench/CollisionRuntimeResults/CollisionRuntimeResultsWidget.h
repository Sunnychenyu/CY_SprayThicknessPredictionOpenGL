#pragma once

#include "CollisionResultsWidget.h"

#include <QWidget>

class CollisionRuntimeResultsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CollisionRuntimeResultsWidget(QWidget* parent = nullptr);

    void setDisplayMode(CollisionResultsWidget::DisplayMode mode);
    CollisionResultsWidget* resultsWidget() const;

private:
    CollisionResultsWidget* m_resultsWidget = nullptr;
};
