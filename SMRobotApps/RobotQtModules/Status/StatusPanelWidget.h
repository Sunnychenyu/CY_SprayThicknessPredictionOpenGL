#pragma once

#include <QWidget>

class QListWidget;
class QStringList;

class StatusPanelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit StatusPanelWidget(QWidget* parent = nullptr);

    void setStatusItems(const QStringList& items);

private:
    QListWidget* m_resultList = nullptr;
};

