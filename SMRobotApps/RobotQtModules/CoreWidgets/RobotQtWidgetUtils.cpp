#include "RobotQtWidgetUtils.h"

#include <QComboBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLayout>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QSizePolicy>

namespace robot_qt_viewer
{
    QLabel* makePanelTitle(const QString& text, QWidget* parent)
    {
        auto* label = new QLabel(text, parent);
        label->setProperty("panelTitle", true);
        return label;
    }

    void makeHorizontallyCompressible(QWidget* widget)
    {
        if(widget == nullptr) {
            return;
        }
        widget->setMinimumWidth(0);
        widget->setMaximumWidth(QWIDGETSIZE_MAX);
        QSizePolicy policy = widget->sizePolicy();
        policy.setHorizontalPolicy(QSizePolicy::Ignored);
        policy.setHorizontalStretch(0);
        widget->setSizePolicy(policy);
    }

    void configureInspectorButton(QPushButton* button)
    {
        if(button == nullptr) {
            return;
        }
        makeHorizontallyCompressible(button);
        button->setMinimumSize(0, 0);
        if(button->toolTip().isEmpty()) {
            button->setToolTip(button->text());
        }
    }

    void configureInspectorGrid(QGridLayout* layout)
    {
        if(layout == nullptr) {
            return;
        }
        layout->setSizeConstraint(QLayout::SetNoConstraint);
        for(int column = 0; column < layout->columnCount(); ++column) {
            layout->setColumnMinimumWidth(column, 0);
            layout->setColumnStretch(column, 1);
        }
    }

    void configureInspectorForm(QFormLayout* form)
    {
        if(form == nullptr) {
            return;
        }
        form->setSizeConstraint(QLayout::SetNoConstraint);
        form->setRowWrapPolicy(QFormLayout::WrapLongRows);
        form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        form->setFormAlignment(Qt::AlignLeft | Qt::AlignTop);
    }

    void configureInspectorList(QListWidget* list, bool alternatingRows, bool uniformItems)
    {
        if(list == nullptr) {
            return;
        }
        makeHorizontallyCompressible(list);
        list->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        list->setTextElideMode(Qt::ElideRight);
        list->setUniformItemSizes(uniformItems);
        list->setAlternatingRowColors(alternatingRows);
    }

    void configureInspectorCombo(QComboBox* combo, int minimumContentsLength)
    {
        if(combo == nullptr) {
            return;
        }
        makeHorizontallyCompressible(combo);
        combo->setMinimumContentsLength(minimumContentsLength);
        combo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
        QSizePolicy policy = combo->sizePolicy();
        policy.setHorizontalPolicy(QSizePolicy::Ignored);
        combo->setSizePolicy(policy);
    }

    QListWidgetItem* addInspectorListItem(QListWidget* list, const QString& text)
    {
        auto* item = new QListWidgetItem(text, list);
        item->setToolTip(text);
        return item;
    }
}
