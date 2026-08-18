#include "CollisionLinkModelSetupWidget.h"

#include "CollisionLinkModelsWidget.h"

#include <QVBoxLayout>

CollisionLinkModelSetupWidget::CollisionLinkModelSetupWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);

    m_linkModelsWidget = new CollisionLinkModelsWidget(this);
    layout->addWidget(m_linkModelsWidget, 1);
}

CollisionLinkModelsWidget* CollisionLinkModelSetupWidget::linkModelsWidget() const
{
    return m_linkModelsWidget;
}
