#pragma once

#include "CollisionLinkModelsViewModel.h"
#include "CollisionRuntimeViewModel.h"

#include <QWidget>

#include <string>

class QLabel;
class QListWidget;
class QPushButton;
class QTableWidget;
class QString;
class QStringList;

class CollisionLinkModelsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CollisionLinkModelsWidget(QWidget* parent = nullptr);

    void setViewModel(const CollisionLinkModelsViewModel& viewModel);
    void setSummary(const CollisionLinkModelsSummaryView& summary);
    QString currentVariantId() const;
    QString currentVariantRole() const;
    QString currentVariantSource() const;
    QString appliedVariantId() const;
    QString appliedVariantSource() const;
    QString currentElementId() const;
    bool hasCurrentVariant() const;
    bool selectAppliedVariant();
    bool selectVariantBySourceRole(const QString& source, const QString& role);
    robot_qt_viewer::CollisionRuntimeProxyRequest proxyRequest(
        const QString& proxyType,
        const std::string& fallbackRole,
        bool useExistingCollisionAsInput) const;
    bool replaceOriginal() const;
    void setContextActionsEnabled(bool hasRobot, bool hasLink);
    void setVariantActionsEnabled(bool canUseVariant, bool canShowVariant);
    void setTaskExitEnabled(bool enabled);

signals:
    void variantSelectionChanged();
    void setCurrentVariantRequested();
    void generateCoacdRequested();
    void applyConfigurationRequested();
    void cancelConfigurationRequested();

private:
    bool selectedVariantIsCurrent() const;
    void updateSelectedVariantSummary();
    void updateComplexityTable(const QStringList& rows);
    void resetComplexityTableColumnWidths();
    void updateActionButtonsEnabled();

    QLabel* m_targetLabel = nullptr;
    QLabel* m_statusLabel = nullptr;
    QLabel* m_complexityTitleLabel = nullptr;
    QTableWidget* m_complexityTable = nullptr;
    QListWidget* m_variantList = nullptr;
    QPushButton* m_setCurrentVariantButton = nullptr;
    QPushButton* m_generateCoacdButton = nullptr;
    QPushButton* m_applyButton = nullptr;
    QPushButton* m_cancelButton = nullptr;
    QString m_baseStatusText;
    bool m_variantActionsAllowed = false;
    bool m_taskExitAllowed = false;
};
