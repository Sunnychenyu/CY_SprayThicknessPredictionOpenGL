#pragma once

#include <QString>

#include <functional>

class CollisionLinkModelsWidget;

namespace robot_qt_viewer
{
    class CollisionWorkbenchServices;
    class CollisionQualityMessageState;
    class CollisionRequestDocumentFacade;
    class RobotQtViewerDocumentContext;

    class CollisionRequestWorkbenchController
    {
    public:
        struct Callbacks
        {
            std::function<QString()> activeDetectorId;
            std::function<void(const QString&, int)> statusMessage;
            std::function<void(const QString&)> refreshElementList;
            std::function<void(const QString&)> refreshModelSummary;
            std::function<void()> refreshDetectorList;
            std::function<void()> refreshDetectorDetails;
            std::function<void()> refreshDetectorProperties;
            std::function<bool(const QString&)> setCurrentDetectorRole;
            std::function<bool()> isCollisionModelConfigurationActive;
        };

        CollisionRequestWorkbenchController(
            CollisionLinkModelsWidget& widget,
            RobotQtViewerDocumentContext& context,
            CollisionWorkbenchServices& appServices,
            CollisionRequestDocumentFacade& documentFacade,
            CollisionQualityMessageState& qualityMessageState,
            Callbacks callbacks);

        void handleVariantSelectionChanged();
        void showSelectedVariantOnly();
        void setSelectedVariantCurrent();
        void cancelSelectedVariantChange();
        void clearVariantPreview();
        void useSelectedVariantInActiveDetector();
        void selectVariantBySourceRole(const QString& source, const QString& role);
        void setReplaceOriginal(bool replaceOriginal, bool updating);
        void addBoxElement();
        void generateFromVisual(const QString& proxyType);
        void generateFromExistingCollision(const QString& proxyType);
        void generateRobotFromExistingCollision(const QString& proxyType);
        void generateCoacd();
        void generateMissingFromVisual();
        void removeSelectedElement();

    private:
        QString activeDetectorId() const;
        void showStatus(const QString& message, int timeoutMs) const;
        void refreshElementList(const QString& qualityMessage) const;
        void refreshModelSummary(const QString& qualityMessage) const;
        void refreshDetectorList() const;
        void refreshDetectorDetails() const;
        void refreshDetectorProperties() const;
        void clearRobotVariantPreview();
        void previewVariant(const QString& variantId, const QString& source);
        void selectGeneratedVariant(const QString& source, const QString& role, bool useInDetector);

        CollisionLinkModelsWidget& m_widget;
        RobotQtViewerDocumentContext& m_context;
        CollisionWorkbenchServices& m_appServices;
        CollisionRequestDocumentFacade& m_documentFacade;
        CollisionQualityMessageState& m_qualityMessageState;
        Callbacks m_callbacks;
        QString m_previewRobotId;
        QString m_previewLinkName;
    };
}
