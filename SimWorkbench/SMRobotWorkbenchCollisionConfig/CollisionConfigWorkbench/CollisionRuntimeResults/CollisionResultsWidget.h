#pragma once

#include <QString>
#include <QWidget>

struct CollisionResultsViewModel;

class QLabel;
class QSplitter;
class QTableWidget;
class QTabWidget;

class CollisionResultsWidget : public QWidget
{
    Q_OBJECT

public:
    enum class DisplayMode
    {
        Full,
        SummaryOnly,
        DetailsOnly
    };

    explicit CollisionResultsWidget(QWidget* parent = nullptr);

    void setDisplayMode(DisplayMode mode);
    void setResults(const CollisionResultsViewModel& viewModel);

private:
    void applyDisplayMode();

    DisplayMode m_displayMode = DisplayMode::Full;
    QSplitter* m_splitter = nullptr;
    QWidget* m_resultSummaryPane = nullptr;
    QWidget* m_resultDetailsPane = nullptr;
    QLabel* m_detailsTitle = nullptr;
    QTableWidget* m_summaryTable = nullptr;
    QLabel* m_resultDetailsTitle = nullptr;
    QTabWidget* m_detailsTabs = nullptr;
    QTableWidget* m_timingTable = nullptr;
    QLabel* m_overlayTimingTitle = nullptr;
    QTableWidget* m_overlayTimingTable = nullptr;
    QLabel* m_debugTitle = nullptr;
    QLabel* m_debugLabel = nullptr;
    QLabel* m_contactTitle = nullptr;
    QTableWidget* m_contactTable = nullptr;
    QLabel* m_nearestTitle = nullptr;
    QTableWidget* m_nearestTable = nullptr;
    QString m_lastSummarySignature;
    QString m_lastTimingSignature;
    QString m_lastOverlayTimingSignature;
    QString m_lastDebugText;
    QString m_lastContactSignature;
    QString m_lastNearestSignature;
};
