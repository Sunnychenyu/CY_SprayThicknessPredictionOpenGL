#pragma once

#include <QString>
#include <QVector>
#include <QWidget>

struct CollisionSelectionSetListItemView;
struct CollisionSelectionSetMemberItemView;

class QListWidget;
class QPushButton;

class CollisionSelectionSetsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CollisionSelectionSetsWidget(QWidget* parent = nullptr);

    void setSelectionSets(const QVector<CollisionSelectionSetListItemView>& items, const QString& preferredId);
    void setMembers(const QVector<CollisionSelectionSetMemberItemView>& items, int preferredIndex);
    void setSelectionSetActionsEnabled(bool enabled);
    void setRemoveMemberEnabled(bool enabled);
    bool selectSelectionSet(const QString& selectionSetId);
    QString currentSelectionSetId() const;
    int currentMemberIndex() const;

signals:
    void selectionSetSelectionChanged();
    void memberSelectionChanged();
    void addSetRequested();
    void renameSetRequested();
    void removeSetRequested();
    void removeMemberRequested();

private:
    QListWidget* m_selectionSetList = nullptr;
    QListWidget* m_memberList = nullptr;
    QPushButton* m_addSetButton = nullptr;
    QPushButton* m_renameSetButton = nullptr;
    QPushButton* m_removeSetButton = nullptr;
    QPushButton* m_removeMemberButton = nullptr;
};
