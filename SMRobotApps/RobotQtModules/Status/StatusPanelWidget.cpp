#include "StatusPanelWidget.h"

#include "RobotQtWidgetUtils.h"

#include <QLabel>
#include <QListWidget>
#include <QStringList>
#include <QVBoxLayout>

StatusPanelWidget::StatusPanelWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);

    layout->addWidget(robot_qt_viewer::makePanelTitle("Runtime Status", this));

    m_resultList = new QListWidget(this);
    robot_qt_viewer::configureInspectorList(m_resultList, true);
    layout->addWidget(m_resultList, 1);

    setStatusItems({
        "Dual robot collision scene: active",
        "Selection highlight: visual mesh",
        "Collision highlight: orange overlay"
    });
}

void StatusPanelWidget::setStatusItems(const QStringList& items)
{
    if(m_resultList == nullptr) {
        return;
    }

    m_resultList->clear();
    for(const QString& item : items) {
        robot_qt_viewer::addInspectorListItem(m_resultList, item);
    }
}
