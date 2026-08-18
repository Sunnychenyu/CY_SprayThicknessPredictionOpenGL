#pragma once

#include "CollisionDetectorsViewModel.h"

#include <QWidget>

class QComboBox;
class QLabel;
class QListWidget;
class QListWidgetItem;
class QPoint;
class QPushButton;

class CollisionDetectorsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CollisionDetectorsWidget(QWidget* parent = nullptr);

    void setDetectors(const QVector<CollisionDetectorListItemView>& items, const QString& preferredId);
    void setProperties(const CollisionDetectorPropertiesView& view);
    void setPairs(const CollisionDetectorPairsViewModel& view);
    CollisionDetectorPropertiesView currentProperties() const;
    CollisionDetectorQueryContractView currentQueryContract() const;
    QString currentDetectorId() const;
    QVector<QString> selectedDetectorIds() const;
    void selectDetector(const QString& detectorId);
    void setDetectorActionsEnabled(bool canAdd, bool hasDetector, bool canRemove);
    void setLinkPairActionsEnabled(bool canMarkLinkA, bool canCreateLinkLink);
    bool setCurrentRole(const QString& role);
    void addDraftSetMember(const QString& side, const CollisionDetectorDraftMemberView& member);
    void clearDraftPairBuilder();
    QVector<CollisionDetectorDraftMemberView> draftSetMembers(const QString& side) const;

signals:
    void detectorEnabledChanged(const QString& detectorId, bool enabled);
    void detectorSelectionChanged();
    void detectorClicked(const QString& detectorId);
    void detectorPropertyChanged();
    void addDetectorRequested();
    void editDetectorRequested();
    void showSelectedRequested();
    void removeDetectorRequested();
    void bindDraftSetsRequested();
    void removePairGeneratorsRequested(const QVector<int>& generatorIndexes);
    void clearPairScopeRequested();
    void pairScopePreviewRequested(
        const QString& robotAId,
        const QString& linkAName,
        const QString& objectAId,
        const QString& attachmentAId,
        const QString& robotBId,
        const QString& linkBName,
        const QString& objectBId,
        const QString& attachmentBId);
    void draftMemberPreviewRequested(
        const QString& robotId,
        const QString& linkName,
        const QString& objectId,
        const QString& attachmentId);

private:
    QString detectorIdForCurrentIndex() const;
    void openQueryDialog();
    void resetDraft();
    void setDraft(const CollisionDetectorPropertiesView& view, bool dirty);
    void updateSummary();
    void updateActionState();
    QListWidget* listForSide(const QString& side) const;
    QVector<CollisionDetectorDraftMemberView> membersForList(const QListWidget* list) const;
    void removeSelectedDraftMembers(QListWidget* list);
    void clearDraftMembers(QListWidget* list);
    void showDraftSetContextMenu(QListWidget* list, const QPoint& pos);
    void showPairScopeContextMenu(const QPoint& pos);
    QVector<int> selectedPairGeneratorIndexes() const;
    QVector<int> allPairGeneratorIndexes() const;
    bool selectedPairScopeClearsScope() const;
    bool pairScopeHasClearableRows() const;
    void previewPairScopeItem(const QListWidgetItem* item);
    void previewDraftMember(const QListWidgetItem* item);
    QString draftMemberKey(const CollisionDetectorDraftMemberView& member) const;
    void updateDraftActionState();

    QComboBox* m_detectorCombo = nullptr;
    QLabel* m_summaryLabel = nullptr;
    QLabel* m_pairSummaryLabel = nullptr;
    QListWidget* m_pairList = nullptr;
    QListWidget* m_setAList = nullptr;
    QListWidget* m_setBList = nullptr;
    QPushButton* m_configureButton = nullptr;
    QPushButton* m_addDetectorButton = nullptr;
    QPushButton* m_showSelectedButton = nullptr;
    QPushButton* m_bindDraftSetsButton = nullptr;
    QPushButton* m_removeDetectorButton = nullptr;
    CollisionDetectorPropertiesView m_properties;
    CollisionDetectorPropertiesView m_savedProperties;
    bool m_dirty = false;
    bool m_canAdd = false;
    bool m_hasDetector = false;
    bool m_canRemove = false;
};
