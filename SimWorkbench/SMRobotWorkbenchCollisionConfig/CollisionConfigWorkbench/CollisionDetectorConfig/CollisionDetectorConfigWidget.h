#pragma once

#include <QString>
#include <QVector>
#include <QWidget>

class CollisionDetectorsWidget;
class CollisionSelectionSetsWidget;
class QListWidget;
class QListWidgetItem;
class QPushButton;

struct CollisionLegacyPairItemView;

class CollisionDetectorConfigWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CollisionDetectorConfigWidget(QWidget* parent = nullptr);

    CollisionSelectionSetsWidget* selectionSetsWidget() const;
    CollisionDetectorsWidget* detectorsWidget() const;
    QListWidget* legacyPairList() const;
    QPushButton* autoPairAllButton() const;
    void setLegacyPairs(const QVector<CollisionLegacyPairItemView>& items);
    void setAutoPairAllEnabled(bool enabled);

signals:
    void legacyPairEnabledChanged(const QString& robotId, const QString& objectId, bool enabled);
    void autoPairAllRequested();

private:
    void handleLegacyPairItemChanged(QListWidgetItem* item);

    CollisionSelectionSetsWidget* m_selectionSetsWidget = nullptr;
    CollisionDetectorsWidget* m_detectorsWidget = nullptr;
    QListWidget* m_legacyPairList = nullptr;
    QPushButton* m_autoPairAllButton = nullptr;
};
