#include "SceneExplorerTaskWidget.h"

#include "RobotQtWidgetUtils.h"
#include "ToolTransformEditorWidget.h"

#include <QHBoxLayout>
#include <QCheckBox>
#include <QFont>
#include <QFontMetrics>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QSizePolicy>
#include <QSignalBlocker>
#include <QStringList>
#include <QTimer>
#include <QVBoxLayout>

namespace
{
    QString elidedText(const QFontMetrics& metrics, const QString& text, int width)
    {
        return metrics.elidedText(text, Qt::ElideMiddle, width);
    }

    QFont mountDiagramTitleFont(QFont font)
    {
        font.setBold(true);
        font.setPointSize(8);
        return font;
    }

    QFont mountDiagramBodyFont(QFont font)
    {
        font.setBold(false);
        font.setFamily(QStringLiteral("Consolas"));
        font.setPointSize(7);
        return font;
    }

    qreal mountDiagramInfoBlockHeight(const QFont& baseFont, const QString& body, qreal minimumHeight)
    {
        const QFont titleFont = mountDiagramTitleFont(baseFont);
        const QFontMetrics titleMetrics(titleFont);
        const QFontMetrics bodyMetrics(mountDiagramBodyFont(titleFont));
        const int rowCount = qMax(1, body.split(QLatin1Char('\n')).size());
        const qreal contentHeight =
            8.0 +
            static_cast<qreal>(titleMetrics.lineSpacing()) +
            6.0 +
            static_cast<qreal>(rowCount * bodyMetrics.lineSpacing()) +
            8.0;
        return qMax(minimumHeight, contentHeight);
    }
}

class SceneExplorerMountBindingDiagramWidget : public QWidget
{
public:
    explicit SceneExplorerMountBindingDiagramWidget(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setMinimumHeight(210);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    QSize sizeHint() const override
    {
        const qreal transformHeight = mountDiagramInfoBlockHeight(font(), m_view.mountTransformText, 128.0);
        if(!m_view.hasObjectBinding) {
            return QSize(560, qMax(210, static_cast<int>(transformHeight + 28.0)));
        }

        const qreal bindingHeight = mountDiagramInfoBlockHeight(
            font(),
            m_view.bindingName.isEmpty() ? QStringLiteral("Bound object") : m_view.bindingName,
            58.0);
        const qreal inverseHeight = mountDiagramInfoBlockHeight(font(), m_view.objectFrameInverseTransformText, 128.0);
        return QSize(560, qMax(390, static_cast<int>(transformHeight + bindingHeight + inverseHeight + 64.0)));
    }

    void setDiagramView(const robot_qt_viewer::SceneExplorerMountBindingDiagramView& view)
    {
        m_view = view;
        const int height = sizeHint().height();
        setMinimumHeight(height);
        setMaximumHeight(height);
        updateGeometry();
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        const QRectF area = rect().adjusted(1.0, 1.0, -1.0, -1.0);
        painter.fillRect(area, palette().base());

        const QColor borderColor(82, 96, 112);
        const QColor nodeBorderColor(82, 156, 238);
        const QColor nodeFillColor(22, 34, 48);
        const QColor infoFillColor(15, 23, 32);
        const QColor textColor(226, 234, 244);
        const QColor mutedColor(130, 144, 160);

        painter.setPen(QPen(borderColor, 1.0));
        painter.setBrush(palette().base());
        painter.drawRoundedRect(area, 6.0, 6.0);

        const qreal margin = 12.0;
        const qreal nodeWidth = qMin<qreal>(170.0, qMax<qreal>(130.0, area.width() * 0.32));
        const qreal infoGap = 20.0;
        const qreal infoX = area.left() + margin + nodeWidth + infoGap;
        const qreal infoWidth = qMax<qreal>(120.0, area.right() - margin - infoX);
        const qreal nodeHeight = 40.0;
        const qreal infoHeight = mountDiagramInfoBlockHeight(painter.font(), m_view.mountTransformText, 128.0);
        const qreal bindingInfoHeight = mountDiagramInfoBlockHeight(
            painter.font(),
            safeText(m_view.bindingName, QStringLiteral("Bound object")),
            58.0);
        const qreal objectInfoHeight = mountDiagramInfoBlockHeight(
            painter.font(),
            m_view.objectFrameInverseTransformText,
            128.0);
        const qreal infoStackGap = 16.0;
        const int nodeCount = m_view.hasObjectBinding ? 4 : 2;
        const qreal gap = qBound<qreal>(
            30.0,
            (area.height() - margin * 2.0 - nodeHeight * nodeCount) /
                qMax<qreal>(1.0, static_cast<qreal>(nodeCount - 1)),
            m_view.hasObjectBinding ? 62.0 : 48.0);
        const qreal nodeStackHeight =
            nodeHeight * nodeCount +
            gap * qMax<qreal>(0.0, static_cast<qreal>(nodeCount - 1));
        const qreal infoStackHeight = m_view.hasObjectBinding
            ? infoHeight + bindingInfoHeight + objectInfoHeight + infoStackGap * 2.0
            : infoHeight;
        const qreal contentHeight = qMax(nodeStackHeight, infoStackHeight);
        const qreal nodeX = area.left() + margin;
        const qreal firstY = area.top() + qMax(margin, (area.height() - contentHeight) * 0.5);
        const qreal infoTop = area.top() + qMax(margin, (area.height() - infoStackHeight) * 0.5);

        const QRectF linkRect(nodeX, firstY, nodeWidth, nodeHeight);
        const QRectF mountRect(nodeX, firstY + nodeHeight + gap, nodeWidth, nodeHeight);
        const QRectF frameRect(nodeX, firstY + (nodeHeight + gap) * 2.0, nodeWidth, nodeHeight);
        const QRectF objectRect(nodeX, firstY + (nodeHeight + gap) * 3.0, nodeWidth, nodeHeight);

        drawNode(painter, linkRect, safeText(m_view.mountLinkName, QStringLiteral("Link")), false, nodeFillColor, nodeBorderColor, textColor);
        drawNode(painter, mountRect, safeText(m_view.mountFrameName, QStringLiteral("Mount Frame")), true, nodeFillColor, nodeBorderColor, textColor);
        drawArrow(painter, linkRect, mountRect, mutedColor);

        if(m_view.hasObjectBinding) {
            drawNode(painter, frameRect, safeText(m_view.objectFrameName, QStringLiteral("Object Frame")), true, nodeFillColor, nodeBorderColor, textColor);
            drawNode(painter, objectRect, safeText(m_view.objectName, QStringLiteral("Object")), false, nodeFillColor, nodeBorderColor, textColor);
            drawArrow(painter, mountRect, frameRect, mutedColor);
            drawArrow(painter, frameRect, objectRect, mutedColor);
        }

        drawInfoBlock(
            painter,
            QRectF(infoX, infoTop, infoWidth, infoHeight),
            QStringLiteral("Link -> Mount"),
            m_view.mountTransformText,
            textColor,
            borderColor,
            infoFillColor);
        if(m_view.hasObjectBinding) {
            drawInfoBlock(
                painter,
                QRectF(infoX, infoTop + infoHeight + infoStackGap, infoWidth, bindingInfoHeight),
                QStringLiteral("Binding"),
                safeText(m_view.bindingName, QStringLiteral("Bound object")),
                textColor,
                borderColor,
                infoFillColor);
            drawInfoBlock(
                painter,
                QRectF(
                    infoX,
                    infoTop + infoHeight + infoStackGap + bindingInfoHeight + infoStackGap,
                    infoWidth,
                    objectInfoHeight),
                QStringLiteral("Object Frame -> Object"),
                m_view.objectFrameInverseTransformText,
                textColor,
                borderColor,
                infoFillColor);
        }
    }

private:
    static QString safeText(const QString& text, const QString& fallback)
    {
        return text.isEmpty() ? fallback : text;
    }

    void drawNode(
        QPainter& painter,
        const QRectF& rect,
        const QString& text,
        bool frameNode,
        const QColor& fillColor,
        const QColor& borderColor,
        const QColor& textColor) const
    {
        painter.setPen(QPen(borderColor, 1.6));
        painter.setBrush(fillColor);
        if(frameNode) {
            painter.drawEllipse(rect);
        } else {
            painter.drawRoundedRect(rect, 8.0, 8.0);
        }

        QFont font = painter.font();
        font.setBold(true);
        font.setPointSize(9);
        painter.setFont(font);
        painter.setPen(textColor);
        const QFontMetrics metrics(font);
        painter.drawText(
            rect.adjusted(8.0, 4.0, -8.0, -4.0),
            Qt::AlignCenter,
            elidedText(metrics, text, static_cast<int>(rect.width() - 16.0)));
    }

    void drawArrow(QPainter& painter, const QRectF& from, const QRectF& to, const QColor& color) const
    {
        const QPointF start(from.center().x(), from.bottom());
        const QPointF end(to.center().x(), to.top());
        painter.setPen(QPen(color, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawLine(start, end);

        const qreal arrowSize = 6.0;
        painter.drawLine(end, QPointF(end.x() - arrowSize, end.y() - arrowSize));
        painter.drawLine(end, QPointF(end.x() + arrowSize, end.y() - arrowSize));
    }

    void drawInfoBlock(
        QPainter& painter,
        const QRectF& rect,
        const QString& title,
        const QString& body,
        const QColor& textColor,
        const QColor& borderColor,
        const QColor& fillColor) const
    {
        painter.setPen(QPen(borderColor, 1.0));
        painter.setBrush(fillColor);
        painter.drawRoundedRect(rect, 5.0, 5.0);

        QFont titleFont = mountDiagramTitleFont(painter.font());
        painter.setFont(titleFont);
        painter.setPen(textColor);
        painter.drawText(rect.adjusted(8.0, 4.0, -8.0, -4.0), Qt::AlignLeft | Qt::AlignTop, title);

        QFont bodyFont = mountDiagramBodyFont(painter.font());
        painter.setFont(bodyFont);
        const QFontMetrics metrics(bodyFont);
        QStringList rows = body.split(QLatin1Char('\n'));
        const int textWidth = static_cast<int>(rect.width() - 16.0);
        for(QString& row : rows) {
            row = metrics.elidedText(row, Qt::ElideRight, textWidth);
        }
        painter.drawText(
            rect.adjusted(8.0, 22.0, -8.0, -4.0),
            Qt::AlignLeft | Qt::AlignTop,
            rows.join(QLatin1Char('\n')));
    }

    robot_qt_viewer::SceneExplorerMountBindingDiagramView m_view;
};

class SceneExplorerObjectFrameDiagramWidget : public QWidget
{
public:
    explicit SceneExplorerObjectFrameDiagramWidget(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setMinimumHeight(190);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    QSize sizeHint() const override
    {
        const qreal nodeStackHeight = 40.0 * 2.0 + 62.0;
        const qreal infoHeight = mountDiagramInfoBlockHeight(font(), m_view.objectToFrameTransformText, 128.0);
        return QSize(560, qMax(190, static_cast<int>(qMax(nodeStackHeight, infoHeight) + 28.0)));
    }

    void setDiagramView(const robot_qt_viewer::SceneExplorerObjectFrameDiagramView& view)
    {
        m_view = view;
        const int height = sizeHint().height();
        setMinimumHeight(height);
        setMaximumHeight(height);
        updateGeometry();
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        const QRectF area = rect().adjusted(1.0, 1.0, -1.0, -1.0);
        painter.fillRect(area, palette().base());

        const QColor borderColor(82, 96, 112);
        const QColor nodeBorderColor(82, 156, 238);
        const QColor nodeFillColor(22, 34, 48);
        const QColor infoFillColor(15, 23, 32);
        const QColor textColor(226, 234, 244);
        const QColor mutedColor(130, 144, 160);

        painter.setPen(QPen(borderColor, 1.0));
        painter.setBrush(palette().base());
        painter.drawRoundedRect(area, 6.0, 6.0);

        const qreal margin = 12.0;
        const qreal nodeWidth = qMin<qreal>(170.0, qMax<qreal>(130.0, area.width() * 0.32));
        const qreal nodeHeight = 40.0;
        const qreal gap = 62.0;
        const qreal infoHeight = mountDiagramInfoBlockHeight(painter.font(), m_view.objectToFrameTransformText, 128.0);
        const qreal nodeStackHeight = nodeHeight * 2.0 + gap;
        const qreal contentHeight = qMax(nodeStackHeight, infoHeight);
        const qreal nodeX = area.left() + margin;
        const qreal firstY = area.top() + qMax(margin, (area.height() - contentHeight) * 0.5);
        const qreal infoTop = area.top() + qMax(margin, (area.height() - infoHeight) * 0.5);
        const QRectF objectRect(nodeX, firstY, nodeWidth, nodeHeight);
        const QRectF frameRect(nodeX, firstY + nodeHeight + gap, nodeWidth, nodeHeight);

        drawNode(painter, objectRect, safeText(m_view.objectName, QStringLiteral("Object")), false, nodeFillColor, nodeBorderColor, textColor);
        drawNode(painter, frameRect, safeText(m_view.objectFrameName, QStringLiteral("Object Frame")), true, nodeFillColor, nodeBorderColor, textColor);
        drawArrow(painter, objectRect, frameRect, mutedColor);

        const qreal infoX = objectRect.right() + 20.0;
        const qreal infoWidth = qMax<qreal>(120.0, area.right() - margin - infoX);
        drawInfoBlock(
            painter,
            QRectF(infoX, infoTop, infoWidth, infoHeight),
            QStringLiteral("Object -> Object Frame"),
            m_view.objectToFrameTransformText,
            textColor,
            borderColor,
            infoFillColor);
    }

private:
    static QString safeText(const QString& text, const QString& fallback)
    {
        return text.isEmpty() ? fallback : text;
    }

    void drawNode(
        QPainter& painter,
        const QRectF& rect,
        const QString& text,
        bool frameNode,
        const QColor& fillColor,
        const QColor& borderColor,
        const QColor& textColor) const
    {
        painter.setPen(QPen(borderColor, 1.6));
        painter.setBrush(fillColor);
        if(frameNode) {
            painter.drawEllipse(rect);
        } else {
            painter.drawRoundedRect(rect, 8.0, 8.0);
        }

        QFont font = painter.font();
        font.setBold(true);
        font.setPointSize(9);
        painter.setFont(font);
        painter.setPen(textColor);
        const QFontMetrics metrics(font);
        painter.drawText(
            rect.adjusted(8.0, 4.0, -8.0, -4.0),
            Qt::AlignCenter,
            elidedText(metrics, text, static_cast<int>(rect.width() - 16.0)));
    }

    void drawArrow(QPainter& painter, const QRectF& from, const QRectF& to, const QColor& color) const
    {
        const QPointF start(from.center().x(), from.bottom());
        const QPointF end(to.center().x(), to.top());
        painter.setPen(QPen(color, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawLine(start, end);

        const qreal arrowSize = 6.0;
        painter.drawLine(end, QPointF(end.x() - arrowSize, end.y() - arrowSize));
        painter.drawLine(end, QPointF(end.x() + arrowSize, end.y() - arrowSize));
    }

    void drawInfoBlock(
        QPainter& painter,
        const QRectF& rect,
        const QString& title,
        const QString& body,
        const QColor& textColor,
        const QColor& borderColor,
        const QColor& fillColor) const
    {
        painter.setPen(QPen(borderColor, 1.0));
        painter.setBrush(fillColor);
        painter.drawRoundedRect(rect, 5.0, 5.0);

        QFont titleFont = mountDiagramTitleFont(painter.font());
        painter.setFont(titleFont);
        painter.setPen(textColor);
        painter.drawText(rect.adjusted(8.0, 4.0, -8.0, -4.0), Qt::AlignLeft | Qt::AlignTop, title);

        QFont bodyFont = mountDiagramBodyFont(painter.font());
        painter.setFont(bodyFont);
        const QFontMetrics metrics(bodyFont);
        QStringList rows = body.split(QLatin1Char('\n'));
        const int textWidth = static_cast<int>(rect.width() - 16.0);
        for(QString& row : rows) {
            row = metrics.elidedText(row, Qt::ElideRight, textWidth);
        }
        painter.drawText(
            rect.adjusted(8.0, 22.0, -8.0, -4.0),
            Qt::AlignLeft | Qt::AlignTop,
            rows.join(QLatin1Char('\n')));
    }

    robot_qt_viewer::SceneExplorerObjectFrameDiagramView m_view;
};

SceneExplorerTaskWidget::SceneExplorerTaskWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(10, 10, 10, 10);
    layout->setSpacing(8);

    m_previewTimer = new QTimer(this);
    m_previewTimer->setSingleShot(true);
    m_previewTimer->setInterval(35);
    connect(m_previewTimer, &QTimer::timeout, this, [this]() {
        flushPendingTransformPreview();
    });

    m_titleLabel = robot_qt_viewer::makePanelTitle(QStringLiteral("Scene Selection"), this);
    layout->addWidget(m_titleLabel);

    m_detailsLabel = new QLabel(this);
    m_detailsLabel->setWordWrap(true);
    layout->addWidget(m_detailsLabel);

    m_readOnlyTransformTitleLabel = robot_qt_viewer::makePanelTitle(QStringLiteral("Transform"), this);
    layout->addWidget(m_readOnlyTransformTitleLabel);

    m_readOnlyTransformMatrixLabel = new QLabel(this);
    m_readOnlyTransformMatrixLabel->setWordWrap(false);
    QFont matrixFont = m_readOnlyTransformMatrixLabel->font();
    matrixFont.setFamily(QStringLiteral("Consolas"));
    m_readOnlyTransformMatrixLabel->setFont(matrixFont);
    robot_qt_viewer::makeHorizontallyCompressible(m_readOnlyTransformMatrixLabel);
    layout->addWidget(m_readOnlyTransformMatrixLabel);

    m_mountBindingDiagram = new SceneExplorerMountBindingDiagramWidget(this);
    layout->addWidget(m_mountBindingDiagram);

    m_scaleLabel = new QLabel(this);
    m_scaleLabel->setWordWrap(true);
    layout->addWidget(m_scaleLabel);

    m_objectNameLabel = new QLabel(QStringLiteral("Object Name"), this);
    layout->addWidget(m_objectNameLabel);

    m_objectNameValueLabel = new QLabel(this);
    m_objectNameValueLabel->setWordWrap(true);
    robot_qt_viewer::makeHorizontallyCompressible(m_objectNameValueLabel);
    layout->addWidget(m_objectNameValueLabel);

    m_objectFrameNameLabel = new QLabel(QStringLiteral("Frame Name"), this);
    layout->addWidget(m_objectFrameNameLabel);

    m_objectFrameNameEdit = new QLineEdit(this);
    m_objectFrameNameEdit->setPlaceholderText(QStringLiteral("Frame Name"));
    robot_qt_viewer::makeHorizontallyCompressible(m_objectFrameNameEdit);
    connect(m_objectFrameNameEdit, &QLineEdit::textChanged, this, [this](const QString& text) {
        if(!m_objectFrameEditorVisible || m_transformEditor == nullptr) {
            return;
        }
        if(m_applyTransformButton != nullptr) {
            m_applyTransformButton->setEnabled(true);
        }
        if(m_cancelTransformButton != nullptr) {
            m_cancelTransformButton->setEnabled(true);
        }
        robot_qt_viewer::SceneExplorerNodeRef target = m_transformTarget;
        target.name = text.trimmed();
        scheduleTransformPreview(target, m_transformEditor->transform());
    });
    layout->addWidget(m_objectFrameNameEdit);

    m_objectFrameDiagram = new SceneExplorerObjectFrameDiagramWidget(this);
    layout->addWidget(m_objectFrameDiagram);

    m_transformEditor = new ToolTransformEditorWidget(QStringLiteral("Transform"), this);
    m_transformEditor->setMatrixVisible(false);
    connect(
        m_transformEditor,
        &ToolTransformEditorWidget::transformChanged,
        this,
        [this](const simulation_project::TransformDesc& transform) {
            if(m_applyTransformButton != nullptr) {
                m_applyTransformButton->setEnabled(true);
            }
            if(m_cancelTransformButton != nullptr) {
                m_cancelTransformButton->setEnabled(true);
            }
            scheduleTransformPreview(currentTransformTarget(), transform);
        });
    layout->addWidget(m_transformEditor);

    m_objectFrameVisibleCheck = new QCheckBox(QStringLiteral("Show Object Frame"), this);
    connect(m_objectFrameVisibleCheck, &QCheckBox::toggled, this, [this](bool checked) {
        if(!m_objectFrameVisibilityControlVisible) {
            return;
        }
        emit objectFrameVisibilityChanged(currentObjectFrameVisibilityTarget(), checked);
    });
    layout->addWidget(m_objectFrameVisibleCheck);

    auto* buttonRow = new QHBoxLayout();
    buttonRow->setContentsMargins(0, 0, 0, 0);
    m_applyTransformButton = new QPushButton(QStringLiteral("Apply Transform"), this);
    m_cancelTransformButton = new QPushButton(QStringLiteral("Cancel Transform"), this);
    robot_qt_viewer::makeHorizontallyCompressible(m_applyTransformButton);
    robot_qt_viewer::makeHorizontallyCompressible(m_cancelTransformButton);
    connect(m_applyTransformButton, &QPushButton::clicked, this, [this]() {
        if(m_transformEditor == nullptr) {
            return;
        }
        flushPendingTransformPreview();
        robot_qt_viewer::SceneExplorerNodeRef target = currentTransformTarget();
        emit transformApplyRequested(target, m_transformEditor->transform());
    });
    connect(m_cancelTransformButton, &QPushButton::clicked, this, [this]() {
        discardPendingTransformPreview();
        emit transformCancelRequested(m_transformTarget);
    });
    buttonRow->addWidget(m_applyTransformButton);
    buttonRow->addWidget(m_cancelTransformButton);
    layout->addLayout(buttonRow);
    layout->addStretch(1);

    setDocumentView(robot_qt_viewer::SceneExplorerViewModel());
}

void SceneExplorerTaskWidget::setDocumentView(const robot_qt_viewer::SceneExplorerViewModel& viewModel)
{
    discardPendingTransformPreview();
    m_objectFrameEditorVisible = viewModel.objectFrameEditorVisible;
    m_objectFrameVisibilityControlVisible = viewModel.objectFrameVisibilityControlVisible ||
        viewModel.objectFrameEditorVisible;
    const bool showMountBindingDiagram =
        viewModel.mountBindingDiagram.visible && !viewModel.objectFrameEditorVisible;
    const bool showObjectFrameDiagram =
        viewModel.objectFrameDiagram.visible;
    const bool showReadOnlyTransform =
        viewModel.readOnlyTransformMatrixVisible &&
        !viewModel.objectFrameEditorVisible &&
        !showObjectFrameDiagram;
    if(m_titleLabel != nullptr) {
        m_titleLabel->setVisible(!viewModel.objectFrameEditorVisible);
        if(viewModel.objectFrameEditorVisible) {
            m_titleLabel->setText(QStringLiteral("Object Frame Editor"));
        } else {
            m_titleLabel->setText(viewModel.selectionTitle.isEmpty()
                ? QStringLiteral("Scene Selection")
                : viewModel.selectionTitle);
        }
    }
    if(m_detailsLabel != nullptr) {
        m_detailsLabel->setVisible(
            !viewModel.objectFrameEditorVisible &&
            !showMountBindingDiagram &&
            !showObjectFrameDiagram);
        m_detailsLabel->setText(viewModel.selectionDetails.isEmpty()
            ? QStringLiteral("No scene item selected.")
            : viewModel.selectionDetails.join(QLatin1Char('\n')));
    }
    if(m_readOnlyTransformTitleLabel != nullptr) {
        m_readOnlyTransformTitleLabel->setVisible(showReadOnlyTransform);
        m_readOnlyTransformTitleLabel->setText(viewModel.readOnlyTransformTitle.isEmpty()
            ? QStringLiteral("Transform")
            : viewModel.readOnlyTransformTitle);
    }
    if(m_readOnlyTransformMatrixLabel != nullptr) {
        m_readOnlyTransformMatrixLabel->setVisible(showReadOnlyTransform);
        m_readOnlyTransformMatrixLabel->setText(QString("Matrix 4x4\n%1").arg(viewModel.readOnlyTransformMatrixText));
    }
    if(m_mountBindingDiagram != nullptr) {
        m_mountBindingDiagram->setVisible(showMountBindingDiagram);
        m_mountBindingDiagram->setDiagramView(viewModel.mountBindingDiagram);
    }
    if(m_objectFrameDiagram != nullptr) {
        m_objectFrameDiagram->setVisible(showObjectFrameDiagram);
        m_objectFrameDiagram->setDiagramView(viewModel.objectFrameDiagram);
    }
    if(m_scaleLabel != nullptr) {
        m_scaleLabel->setVisible(viewModel.pointCloudScaleVisible);
        m_scaleLabel->setText(QString("Point cloud scale: %1").arg(viewModel.pointCloudScale));
    }

    const bool showObjectFrameEditor = viewModel.objectFrameEditorVisible;
    if(m_objectNameLabel != nullptr) {
        m_objectNameLabel->setVisible(showObjectFrameEditor);
    }
    if(m_objectNameValueLabel != nullptr) {
        m_objectNameValueLabel->setVisible(showObjectFrameEditor);
        m_objectNameValueLabel->setText(viewModel.objectFrameObjectName);
    }
    if(m_objectFrameNameLabel != nullptr) {
        m_objectFrameNameLabel->setVisible(showObjectFrameEditor);
    }
    if(m_objectFrameNameEdit != nullptr) {
        const QSignalBlocker blocker(m_objectFrameNameEdit);
        m_objectFrameNameEdit->setVisible(showObjectFrameEditor);
        m_objectFrameNameEdit->setEnabled(showObjectFrameEditor && viewModel.transformEditorEnabled);
        m_objectFrameNameEdit->setText(viewModel.objectFrameName);
    }
    if(m_objectFrameVisibleCheck != nullptr) {
        const QSignalBlocker blocker(m_objectFrameVisibleCheck);
        m_objectFrameVisibleCheck->setVisible(m_objectFrameVisibilityControlVisible);
        m_objectFrameVisibleCheck->setEnabled(m_objectFrameVisibilityControlVisible);
        m_objectFrameVisibleCheck->setChecked(viewModel.objectFrameVisible);
    }

    m_transformTarget = viewModel.transformTarget;
    m_objectFrameVisibilityTarget = viewModel.objectFrameVisibilityTarget;
    if(m_transformEditor != nullptr) {
        m_transformEditor->setVisible(viewModel.transformEditorVisible);
        m_transformEditor->setEnabled(viewModel.transformEditorEnabled);
        m_transformEditor->setTitle(viewModel.transformEditorTitle.isEmpty()
            ? QStringLiteral("Transform")
            : viewModel.transformEditorTitle);
        m_transformEditor->setTransform(viewModel.transform);
    }
    if(m_applyTransformButton != nullptr) {
        m_applyTransformButton->setVisible(viewModel.transformEditorVisible);
        m_applyTransformButton->setEnabled(viewModel.transformEditorDirty || showObjectFrameEditor);
        if(showObjectFrameEditor) {
            m_applyTransformButton->setText(
                viewModel.objectFrameMode == robot_qt_viewer::SceneExplorerObjectFrameMode::Create
                    ? QStringLiteral("Create")
                    : QStringLiteral("Apply"));
        } else {
            m_applyTransformButton->setText(QStringLiteral("Apply Transform"));
        }
    }
    if(m_cancelTransformButton != nullptr) {
        m_cancelTransformButton->setVisible(viewModel.transformEditorVisible);
        m_cancelTransformButton->setEnabled(viewModel.transformEditorDirty || showObjectFrameEditor);
        m_cancelTransformButton->setText(showObjectFrameEditor
            ? QStringLiteral("Cancel")
            : QStringLiteral("Cancel Transform"));
    }
}

robot_qt_viewer::SceneExplorerNodeRef SceneExplorerTaskWidget::currentTransformTarget() const
{
    robot_qt_viewer::SceneExplorerNodeRef target = m_transformTarget;
    if(m_objectFrameEditorVisible && m_objectFrameNameEdit != nullptr) {
        target.name = m_objectFrameNameEdit->text().trimmed();
    }
    return target;
}

robot_qt_viewer::SceneExplorerNodeRef SceneExplorerTaskWidget::currentObjectFrameVisibilityTarget() const
{
    return m_objectFrameEditorVisible ? currentTransformTarget() : m_objectFrameVisibilityTarget;
}

void SceneExplorerTaskWidget::scheduleTransformPreview(
    const robot_qt_viewer::SceneExplorerNodeRef& target,
    const simulation_project::TransformDesc& transform)
{
    m_pendingPreviewTarget = target;
    m_pendingPreviewTransform = transform;
    m_hasPendingPreview = true;
    if(m_previewTimer != nullptr && !m_previewTimer->isActive()) {
        m_previewTimer->start();
    }
}

void SceneExplorerTaskWidget::flushPendingTransformPreview()
{
    if(!m_hasPendingPreview) {
        return;
    }
    if(m_previewTimer != nullptr && m_previewTimer->isActive()) {
        m_previewTimer->stop();
    }

    const robot_qt_viewer::SceneExplorerNodeRef target = m_pendingPreviewTarget;
    const simulation_project::TransformDesc transform = m_pendingPreviewTransform;
    m_hasPendingPreview = false;
    m_pendingPreviewTarget = robot_qt_viewer::SceneExplorerNodeRef();
    m_pendingPreviewTransform = simulation_project::TransformDesc();
    emit transformPreviewChanged(target, transform);
}

void SceneExplorerTaskWidget::discardPendingTransformPreview()
{
    if(m_previewTimer != nullptr && m_previewTimer->isActive()) {
        m_previewTimer->stop();
    }
    m_hasPendingPreview = false;
    m_pendingPreviewTarget = robot_qt_viewer::SceneExplorerNodeRef();
    m_pendingPreviewTransform = simulation_project::TransformDesc();
}
