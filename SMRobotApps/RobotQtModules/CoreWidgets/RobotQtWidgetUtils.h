#pragma once

class QComboBox;
class QFormLayout;
class QGridLayout;
class QLabel;
class QListWidget;
class QListWidgetItem;
class QPushButton;
class QString;
class QWidget;

namespace robot_qt_viewer
{
    QLabel* makePanelTitle(const QString& text, QWidget* parent);
    void makeHorizontallyCompressible(QWidget* widget);
    void configureInspectorButton(QPushButton* button);
    void configureInspectorGrid(QGridLayout* layout);
    void configureInspectorForm(QFormLayout* form);
    void configureInspectorList(QListWidget* list, bool alternatingRows = false, bool uniformItems = true);
    void configureInspectorCombo(QComboBox* combo, int minimumContentsLength = 8);
    QListWidgetItem* addInspectorListItem(QListWidget* list, const QString& text);
}

