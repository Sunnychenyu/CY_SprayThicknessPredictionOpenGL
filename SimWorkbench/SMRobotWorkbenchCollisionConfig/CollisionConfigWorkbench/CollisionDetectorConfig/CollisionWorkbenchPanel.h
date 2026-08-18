#pragma once

#include <QString>
#include <QVector>
#include <QWidget>

#include <string>

class CollisionDetectorsWidget;
class CollisionDetectorConfigWidget;
class CollisionLinkModelSetupWidget;
class CollisionLinkModelsWidget;
class CollisionSelectionSetsWidget;
class QListWidget;
class QListWidgetItem;
class QPushButton;
class QStackedWidget;

struct CollisionDetectorListItemView;
struct CollisionDetectorPairsViewModel;
struct CollisionDetectorPropertiesView;
struct CollisionDetectorQueryContractView;
struct CollisionDetectorDraftMemberView;
struct CollisionLegacyPairItemView;
struct CollisionLinkModelsSummaryView;
struct CollisionLinkModelsViewModel;
struct CollisionSelectionSetListItemView;
struct CollisionSelectionSetMemberItemView;

namespace robot_qt_viewer
{
    struct CollisionRuntimeProxyRequest;
}

class CollisionWorkbenchPanel : public QWidget
{
    Q_OBJECT

public:
    explicit CollisionWorkbenchPanel(QWidget* parent = nullptr);

    CollisionLinkModelsWidget* linkModelsWidget() const;
    CollisionSelectionSetsWidget* selectionSetsWidget() const;
    void showDetectorConfiguration();
    void showCollisionModelConfiguration();

    void setSelectionSets(const QVector<CollisionSelectionSetListItemView>& items, const QString& preferredId);
    void setSelectionSetMembers(const QVector<CollisionSelectionSetMemberItemView>& items, int preferredIndex);
    void setSelectionSetActionsEnabled(bool enabled);
    void setRemoveSelectionSetMemberEnabled(bool enabled);
    bool selectSelectionSet(const QString& selectionSetId);
    QString currentSelectionSetId() const;
    int currentSelectionSetMemberIndex() const;

    void setDetectors(const QVector<CollisionDetectorListItemView>& items, const QString& preferredId);
    void setDetectorProperties(const CollisionDetectorPropertiesView& view);
    void setDetectorPairs(const CollisionDetectorPairsViewModel& view);
    CollisionDetectorPropertiesView currentDetectorProperties() const;
    CollisionDetectorQueryContractView currentDetectorQueryContract() const;
    QString currentDetectorId() const;
    QVector<QString> selectedDetectorIds() const;
    void selectDetector(const QString& detectorId);
    void setDetectorActionsEnabled(bool canAdd, bool hasDetector, bool canRemove);
    void setLinkPairActionsEnabled(bool canMarkLinkA, bool canCreateLinkLink);
    bool setCurrentDetectorRole(const QString& role);
    void addDetectorDraftSetMember(const QString& side, const CollisionDetectorDraftMemberView& member);
    void clearDetectorDraftPairBuilder();
    QVector<CollisionDetectorDraftMemberView> detectorDraftSetMembers(const QString& side) const;

    void setLinkModelsViewModel(const CollisionLinkModelsViewModel& viewModel);
    void setLinkModelsSummary(const CollisionLinkModelsSummaryView& summary);
    QString currentVariantId() const;
    QString currentVariantRole() const;
    QString currentVariantSource() const;
    bool hasCurrentVariant() const;
    bool selectVariantBySourceRole(const QString& source, const QString& role);
    void setContextActionsEnabled(bool hasRobot, bool hasLink);
    void setVariantActionsEnabled(bool canUseVariant, bool canShowVariant);
    void setLegacyPairs(const QVector<CollisionLegacyPairItemView>& items);
    void setAutoPairAllEnabled(bool enabled);

signals:
    void rightPanelTitleChanged(const QString& title);

    void selectionSetSelectionChanged();
    void selectionSetMemberSelectionChanged();
    void addSelectionSetRequested();
    void renameSelectionSetRequested();
    void removeSelectionSetRequested();
    void removeSelectionSetMemberRequested();

    void detectorEnabledChanged(const QString& detectorId, bool enabled);
    void detectorSelectionChanged();
    void detectorClicked(const QString& detectorId);
    void detectorPropertyChanged();
    void addDetectorRequested();
    void editDetectorRequested();
    void showSelectedDetectorsRequested();
    void removeDetectorRequested();
    void bindDetectorDraftSetsRequested();
    void removeDetectorPairGeneratorsRequested(const QVector<int>& generatorIndexes);
    void clearDetectorPairScopeRequested();
    void detectorPairScopePreviewRequested(
        const QString& robotAId,
        const QString& linkAName,
        const QString& objectAId,
        const QString& attachmentAId,
        const QString& robotBId,
        const QString& linkBName,
        const QString& objectBId,
        const QString& attachmentBId);
    void detectorDraftMemberPreviewRequested(
        const QString& robotId,
        const QString& linkName,
        const QString& objectId,
        const QString& attachmentId);

    void linkModelVariantSelectionChanged();

    void legacyPairEnabledChanged(const QString& robotId, const QString& objectId, bool enabled);
    void autoPairAllRequested();

private:
    void connectChildSignals();
    void handleLegacyPairItemChanged(QListWidgetItem* item);

    QStackedWidget* m_pages = nullptr;
    CollisionLinkModelSetupWidget* m_linkModelSetupWidget = nullptr;
    CollisionDetectorConfigWidget* m_detectorConfigWidget = nullptr;
    CollisionSelectionSetsWidget* m_selectionSetsWidget = nullptr;
    CollisionDetectorsWidget* m_detectorsWidget = nullptr;
    CollisionLinkModelsWidget* m_linkModelsWidget = nullptr;
    QListWidget* m_legacyPairList = nullptr;
    QPushButton* m_autoPairAllButton = nullptr;
};
