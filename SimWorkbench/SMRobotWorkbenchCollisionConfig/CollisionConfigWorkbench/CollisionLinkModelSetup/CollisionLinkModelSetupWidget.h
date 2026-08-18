#pragma once

#include <QWidget>

class CollisionLinkModelsWidget;

class CollisionLinkModelSetupWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CollisionLinkModelSetupWidget(QWidget* parent = nullptr);

    CollisionLinkModelsWidget* linkModelsWidget() const;

private:
    CollisionLinkModelsWidget* m_linkModelsWidget = nullptr;
};
