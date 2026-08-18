#include "ToolSetupWidget.h"

#include "RobotQtWidgetUtils.h"
#include "ToolAssetEditorWidget.h"
#include "ToolTransformEditorWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QFontMetrics>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QList>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QSizePolicy>
#include <QSignalBlocker>
#include <QStringList>
#include <QVBoxLayout>

namespace
{
    void setComboItems(QComboBox* combo, const QVector<ToolSetupComboItem>& items, const QString& selectedId, bool enabled)
    {
        if(combo == nullptr) {
            return;
        }

        QSignalBlocker blocker(combo);
        combo->clear();
        int selectedIndex = -1;
        for(int i = 0; i < items.size(); ++i) {
            combo->addItem(items.at(i).text, items.at(i).id);
            if(items.at(i).id == selectedId) {
                selectedIndex = i;
            }
        }
        if(selectedIndex >= 0) {
            combo->setCurrentIndex(selectedIndex);
        }
        combo->setEnabled(enabled && combo->count() > 0);
    }

    QString comboIdAt(const QComboBox* combo, int index)
    {
        if(combo == nullptr || index < 0 || index >= combo->count()) {
            return QString();
        }
        return combo->itemData(index).toString();
    }

    QString elidedText(const QFontMetrics& metrics, const QString& text, int width)
    {
        return metrics.elidedText(text, Qt::ElideMiddle, width);
    }

    void adjustFontSize(QFont& font, qreal pointDelta, qreal minPointSize)
    {
        if(font.pointSizeF() > 0.0) {
            font.setPointSizeF(qMax(minPointSize, font.pointSizeF() + pointDelta));
            return;
        }
        if(font.pixelSize() > 0) {
            font.setPixelSize(qMax(
                static_cast<int>(minPointSize * 1.35),
                font.pixelSize() + static_cast<int>(pointDelta * 1.35)));
        }
    }

    QFont diagramInfoTitleFont(QFont font)
    {
        font.setBold(true);
        adjustFontSize(font, -1.0, 9.0);
        return font;
    }

    QFont diagramInfoBodyFont(QFont font)
    {
        font.setBold(false);
        font.setFamily(QStringLiteral("Consolas"));
        adjustFontSize(font, -2.0, 8.0);
        return font;
    }

    qreal diagramInfoBlockHeight(const QFont& baseFont, const QString& body, qreal minimumHeight)
    {
        const QFont titleFont = diagramInfoTitleFont(baseFont);
        const QFontMetrics titleMetrics(titleFont);
        const QFontMetrics bodyMetrics(diagramInfoBodyFont(titleFont));
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

class ObjectBindingDiagramWidget : public QWidget
{
public:
    explicit ObjectBindingDiagramWidget(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setMinimumHeight(210);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    QSize sizeHint() const override
    {
        const qreal transformHeight = diagramInfoBlockHeight(font(), m_view.mountTransformText, 128.0);
        if(!m_view.hasObjectBinding) {
            return QSize(560, qMax(210, static_cast<int>(transformHeight + 28.0)));
        }

        const qreal bindingHeight = diagramInfoBlockHeight(
            font(),
            m_view.bindingName.isEmpty() ? QStringLiteral("Pending binding") : m_view.bindingName,
            58.0);
        const qreal inverseHeight = diagramInfoBlockHeight(font(), m_view.objectFrameInverseTransformText, 128.0);
        return QSize(560, qMax(390, static_cast<int>(transformHeight + bindingHeight + inverseHeight + 64.0)));
    }

    void setBindingView(const ToolSetupBindingView& view)
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
            const qreal infoHeight = diagramInfoBlockHeight(painter.font(), m_view.mountTransformText, 128.0);
            const qreal bindingInfoHeight = diagramInfoBlockHeight(
                painter.font(),
                safeText(m_view.bindingName, QStringLiteral("Pending binding")),
                58.0);
            const qreal objectInfoHeight = diagramInfoBlockHeight(
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
            const QRectF mountRect(nodeX, firstY + (nodeHeight + gap), nodeWidth, nodeHeight);
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
                    safeText(m_view.bindingName, QStringLiteral("Pending binding")),
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
            adjustFontSize(font, -1.0, 9.0);
            painter.setFont(font);
            painter.setPen(textColor);
            const QFontMetrics metrics(font);
            painter.drawText(rect.adjusted(8.0, 4.0, -8.0, -4.0), Qt::AlignCenter, elidedText(metrics, text, static_cast<int>(rect.width() - 16.0)));
        }

        void drawArrow(QPainter& painter, const QRectF& from, const QRectF& to, const QColor& color) const
        {
            const QPointF start(from.center().x(), from.bottom());
            const QPointF end(to.center().x(), to.top());
            painter.setPen(QPen(color, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            painter.drawLine(start, end);

            const qreal arrowSize = 6.0;
            const QPointF bottomLeft(end.x() - arrowSize, end.y() - arrowSize);
            const QPointF bottomRight(end.x() + arrowSize, end.y() - arrowSize);
            painter.drawLine(end, bottomLeft);
            painter.drawLine(end, bottomRight);
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

            QFont titleFont = diagramInfoTitleFont(painter.font());
            painter.setFont(titleFont);
            painter.setPen(textColor);
            painter.drawText(rect.adjusted(8.0, 4.0, -8.0, -4.0), Qt::AlignLeft | Qt::AlignTop, title);

            QFont bodyFont = diagramInfoBodyFont(painter.font());
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

    ToolSetupBindingView m_view;
};

ToolSetupWidget::ToolSetupWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* toolLayout = new QVBoxLayout(this);
    toolLayout->setContentsMargins(10, 10, 10, 10);
    toolLayout->setSpacing(8);

    m_frameEditorTitleLabel = robot_qt_viewer::makePanelTitle("Mount Frame Edit", this);
    m_frameEditorTitleLabel->setVisible(false);
    toolLayout->addWidget(m_frameEditorTitleLabel);
    m_taskStatusLabel = new QLabel("Ready", this);
    m_taskStatusLabel->setWordWrap(true);
    robot_qt_viewer::makeHorizontallyCompressible(m_taskStatusLabel);
    toolLayout->addWidget(m_taskStatusLabel);

    m_robotMountCombo = new QComboBox(this);
    robot_qt_viewer::configureInspectorCombo(m_robotMountCombo);
    connect(m_robotMountCombo, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this, &ToolSetupWidget::mountSelectionChanged);
    toolLayout->addWidget(m_robotMountCombo);

    m_robotMountDetailsLabel = new QLabel("No mount frame", this);
    m_robotMountDetailsLabel->setWordWrap(true);
    robot_qt_viewer::makeHorizontallyCompressible(m_robotMountDetailsLabel);
    toolLayout->addWidget(m_robotMountDetailsLabel);

    m_robotMountNameLabel = new QLabel("Frame Name", this);
    toolLayout->addWidget(m_robotMountNameLabel);
    m_robotMountNameEdit = new QLineEdit(this);
    m_robotMountNameEdit->setPlaceholderText("Frame Name");
    robot_qt_viewer::makeHorizontallyCompressible(m_robotMountNameEdit);
    connect(m_robotMountNameEdit, &QLineEdit::textChanged, this, [this](const QString&) {
        markTaskDirty();
    });
    toolLayout->addWidget(m_robotMountNameEdit);

    m_robotMountLinkCombo = new QComboBox(this);
    robot_qt_viewer::configureInspectorCombo(m_robotMountLinkCombo);
    connect(
        m_robotMountLinkCombo,
        static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this,
        [this](int) {
            markTaskDirty();
        });
    toolLayout->addWidget(m_robotMountLinkCombo);

    auto* mountButtonRow = new QHBoxLayout();
    mountButtonRow->setContentsMargins(0, 0, 0, 0);
    m_addRobotMountButton = new QPushButton("Add Link Mount", this);
    m_deleteRobotMountButton = new QPushButton("Delete Selected Mount", this);
    robot_qt_viewer::makeHorizontallyCompressible(m_addRobotMountButton);
    robot_qt_viewer::makeHorizontallyCompressible(m_deleteRobotMountButton);
    connect(m_addRobotMountButton, &QPushButton::clicked, this, &ToolSetupWidget::addRobotMountRequested);
    connect(m_deleteRobotMountButton, &QPushButton::clicked, this, &ToolSetupWidget::deleteRobotMountRequested);
    mountButtonRow->addWidget(m_addRobotMountButton);
    mountButtonRow->addWidget(m_deleteRobotMountButton);
    toolLayout->addLayout(mountButtonRow);

    m_robotMountTransformEditor = new ToolTransformEditorWidget(
        "Mount frame transform: Link -> Mount",
        this);
    m_robotMountTransformEditor->setMatrixVisible(false);
    connect(
        m_robotMountTransformEditor,
        &ToolTransformEditorWidget::transformChanged,
        this,
        [this](const simulation_project::TransformDesc& transform) {
            emit mountTransformPreviewChanged(transform);
            markTaskDirty();
        });
    toolLayout->addWidget(m_robotMountTransformEditor);

    m_attachmentSectionTitle = robot_qt_viewer::makePanelTitle("Attachment", this);
    toolLayout->addWidget(m_attachmentSectionTitle);
    m_toolAttachmentCombo = new QComboBox(this);
    robot_qt_viewer::configureInspectorCombo(m_toolAttachmentCombo);
    connect(m_toolAttachmentCombo, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this, &ToolSetupWidget::attachmentSelectionChanged);
    toolLayout->addWidget(m_toolAttachmentCombo);

    m_toolDetailsLabel = new QLabel("No attachment", this);
    m_toolDetailsLabel->setWordWrap(true);
    robot_qt_viewer::makeHorizontallyCompressible(m_toolDetailsLabel);
    toolLayout->addWidget(m_toolDetailsLabel);

    m_configureToolAttachmentButton = new QPushButton("Edit Attachment", this);
    robot_qt_viewer::makeHorizontallyCompressible(m_configureToolAttachmentButton);
    connect(m_configureToolAttachmentButton, &QPushButton::clicked, this, &ToolSetupWidget::configureAttachmentRequested);
    toolLayout->addWidget(m_configureToolAttachmentButton);

    m_attachmentNameEdit = new QLineEdit(this);
    m_attachmentNameEdit->setPlaceholderText("Attachment display name");
    robot_qt_viewer::makeHorizontallyCompressible(m_attachmentNameEdit);
    connect(m_attachmentNameEdit, &QLineEdit::textChanged, this, [this](const QString&) {
        markTaskDirty();
    });
    toolLayout->addWidget(m_attachmentNameEdit);

    m_attachmentAssetCombo = new QComboBox(this);
    robot_qt_viewer::configureInspectorCombo(m_attachmentAssetCombo);
    connect(
        m_attachmentAssetCombo,
        static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this,
        [this](int) {
            markTaskDirty();
        });
    toolLayout->addWidget(m_attachmentAssetCombo);

    m_attachmentEnabledCheck = new QCheckBox("Attachment enabled", this);
    connect(m_attachmentEnabledCheck, &QCheckBox::toggled, this, [this](bool) {
        markTaskDirty();
    });
    toolLayout->addWidget(m_attachmentEnabledCheck);

    m_attachmentOffsetEditor = new ToolTransformEditorWidget(
        "Attachment offset: Mount -> Asset mount",
        this);
    connect(
        m_attachmentOffsetEditor,
        &ToolTransformEditorWidget::transformChanged,
        this,
        [this](const simulation_project::TransformDesc&) {
            m_attachmentOffsetDirty = true;
            markTaskDirty();
            if(m_applyAttachmentOffsetButton != nullptr && m_attachmentOffsetEditor != nullptr) {
                m_applyAttachmentOffsetButton->setEnabled(m_attachmentOffsetEditor->isEnabled());
            }
        });
    toolLayout->addWidget(m_attachmentOffsetEditor);

    m_applyAttachmentOffsetButton = new QPushButton("Apply Attachment Offset", this);
    robot_qt_viewer::makeHorizontallyCompressible(m_applyAttachmentOffsetButton);
    m_applyAttachmentOffsetButton->setVisible(false);
    connect(m_applyAttachmentOffsetButton, &QPushButton::clicked, this, [this]() {
        if(m_attachmentOffsetEditor == nullptr) {
            return;
        }
        m_attachmentOffsetDirty = false;
        m_applyAttachmentOffsetButton->setEnabled(false);
        emit attachmentOffsetApplyRequested(m_attachmentOffsetEditor->transform());
    });
    toolLayout->addWidget(m_applyAttachmentOffsetButton);

    m_assetSectionTitle = robot_qt_viewer::makePanelTitle("Attachment Asset", this);
    toolLayout->addWidget(m_assetSectionTitle);
    m_toolAssetCombo = new QComboBox(this);
    robot_qt_viewer::configureInspectorCombo(m_toolAssetCombo);
    connect(m_toolAssetCombo, static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this, &ToolSetupWidget::assetSelectionChanged);
    toolLayout->addWidget(m_toolAssetCombo);

    m_toolAssetDetailsLabel = new QLabel("No attachment asset", this);
    m_toolAssetDetailsLabel->setWordWrap(true);
    robot_qt_viewer::makeHorizontallyCompressible(m_toolAssetDetailsLabel);
    toolLayout->addWidget(m_toolAssetDetailsLabel);

    m_toolAssetEditor = new ToolAssetEditorWidget(this);
    m_toolAssetEditor->setApplyButtonVisible(true);
    connect(
        m_toolAssetEditor,
        &ToolAssetEditorWidget::applyRequested,
        this,
        &ToolSetupWidget::toolAssetApplyRequested);
    connect(
        m_toolAssetEditor,
        &ToolAssetEditorWidget::assetChanged,
        this,
        [this](const simulation_project::AttachmentAssetDesc&) {
            markTaskDirty();
        });
    connect(
        m_toolAssetEditor,
        &ToolAssetEditorWidget::visualTransformChanged,
        this,
        [this](const simulation_project::AttachmentAssetDesc&) {
            markTaskDirty();
        });
    connect(
        m_toolAssetEditor,
        &ToolAssetEditorWidget::tcpTransformChanged,
        this,
        [this](const simulation_project::AttachmentAssetDesc&) {
            markTaskDirty();
        });
    toolLayout->addWidget(m_toolAssetEditor);

    m_importToolAssetButton = new QPushButton("Import Tool Model...", this);
    robot_qt_viewer::makeHorizontallyCompressible(m_importToolAssetButton);
    connect(m_importToolAssetButton, &QPushButton::clicked, this, &ToolSetupWidget::importToolAssetRequested);
    toolLayout->addWidget(m_importToolAssetButton);

    m_attachToolAssetButton = new QPushButton("Attach Existing Asset...", this);
    robot_qt_viewer::makeHorizontallyCompressible(m_attachToolAssetButton);
    connect(m_attachToolAssetButton, &QPushButton::clicked, this, &ToolSetupWidget::attachToolAssetRequested);
    toolLayout->addWidget(m_attachToolAssetButton);

    m_editToolAssetButton = new QPushButton("Edit Asset", this);
    robot_qt_viewer::makeHorizontallyCompressible(m_editToolAssetButton);
    connect(m_editToolAssetButton, &QPushButton::clicked, this, &ToolSetupWidget::editToolAssetRequested);
    toolLayout->addWidget(m_editToolAssetButton);

    m_frameVisibilitySectionTitle = robot_qt_viewer::makePanelTitle("Frames", this);
    toolLayout->addWidget(m_frameVisibilitySectionTitle);
    m_showLinkFrameCheck = new QCheckBox("Link frame", this);
    m_showRobotMountFrameCheck = new QCheckBox("Show Mount Frame", this);
    m_showToolMountFrameCheck = new QCheckBox("Asset mount", this);
    m_showVisualFrameCheck = new QCheckBox("Visual frame", this);
    m_showTcpFrameCheck = new QCheckBox("TCP frame", this);
    m_showSensorPreviewCheck = new QCheckBox("Sensor optical / FOV", this);

    const QList<QCheckBox*> frameChecks = {
        m_showLinkFrameCheck,
        m_showRobotMountFrameCheck,
        m_showToolMountFrameCheck,
        m_showVisualFrameCheck,
        m_showTcpFrameCheck,
        m_showSensorPreviewCheck
    };
    for(QCheckBox* check : frameChecks) {
        check->setChecked(true);
        connect(check, &QCheckBox::toggled, this, &ToolSetupWidget::frameVisibilityChanged);
        toolLayout->addWidget(check);
    }
    if(m_showLinkFrameCheck != nullptr) {
        m_showLinkFrameCheck->setChecked(false);
    }
    if(m_showRobotMountFrameCheck != nullptr) {
        m_showRobotMountFrameCheck->setChecked(false);
    }

    m_objectBindingSectionTitle = robot_qt_viewer::makePanelTitle("Object Binding", this);
    toolLayout->addWidget(m_objectBindingSectionTitle);

    m_bindingSummaryFrame = new QFrame(this);
    m_bindingSummaryFrame->setObjectName(QStringLiteral("ToolSetupBindingSummaryFrame"));
    m_bindingSummaryFrame->setFrameShape(QFrame::StyledPanel);
    m_bindingSummaryFrame->setStyleSheet(QStringLiteral(
        "QFrame#ToolSetupBindingSummaryFrame {"
        " border: 1px solid #344252;"
        " border-radius: 6px;"
        " background-color: #0f151c;"
        "}"));
    auto* bindingSummaryLayout = new QVBoxLayout(m_bindingSummaryFrame);
    bindingSummaryLayout->setContentsMargins(10, 10, 10, 10);
    bindingSummaryLayout->setSpacing(8);
    toolLayout->addWidget(m_bindingSummaryFrame);

    m_bindingNameTitleLabel = new QLabel("Binding Name", m_bindingSummaryFrame);
    m_bindingNameTitleLabel->setFont(font());
    bindingSummaryLayout->addWidget(m_bindingNameTitleLabel);
    m_bindingNameValueLabel = new QLabel(m_bindingSummaryFrame);
    m_bindingNameValueLabel->setWordWrap(true);
    m_bindingNameValueLabel->setFrameShape(QFrame::StyledPanel);
    m_bindingNameValueLabel->setAlignment(Qt::AlignCenter);
    QFont bindingValueFont = font();
    bindingValueFont.setBold(true);
    m_bindingNameValueLabel->setFont(bindingValueFont);
    m_bindingNameValueLabel->setMinimumHeight(qMax(38, QFontMetrics(bindingValueFont).height() + 16));
    m_bindingNameValueLabel->setStyleSheet(QStringLiteral(
        "QLabel {"
        " padding: 8px;"
        " border: 1px solid #253342;"
        " border-radius: 4px;"
        " background-color: #0b1118;"
        "}"));
    robot_qt_viewer::makeHorizontallyCompressible(m_bindingNameValueLabel);
    bindingSummaryLayout->addWidget(m_bindingNameValueLabel);

    m_objectBindingDiagram = new ObjectBindingDiagramWidget(m_bindingSummaryFrame);
    bindingSummaryLayout->addWidget(m_objectBindingDiagram);

    m_bindingMountCombo = new QComboBox(this);
    robot_qt_viewer::configureInspectorCombo(m_bindingMountCombo);
    m_bindingMountCombo->setToolTip("Robot mount frame");
    connect(
        m_bindingMountCombo,
        static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this,
        [this](int) {
            if(!m_objectBindingEditorVisible) {
                return;
            }
            emit objectBindingSelectionChanged(
                currentBindingMountId(),
                currentBindingObjectId(),
                currentBindingFrameId());
        });

    m_bindingObjectNameLabel = new QLabel("Object Name", this);
    m_bindingObjectNameLabel->setFont(font());
    toolLayout->addWidget(m_bindingObjectNameLabel);
    m_bindingObjectCombo = new QComboBox(this);
    robot_qt_viewer::configureInspectorCombo(m_bindingObjectCombo);
    m_bindingObjectCombo->setToolTip("Object to bind");
    connect(
        m_bindingObjectCombo,
        static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this,
        [this](int) {
            if(!m_objectBindingEditorVisible) {
                return;
            }
            emit objectBindingSelectionChanged(
                currentBindingMountId(),
                currentBindingObjectId(),
                QString());
        });
    toolLayout->addWidget(m_bindingObjectCombo);

    m_bindingObjectFrameLabel = new QLabel("Object Frame", this);
    m_bindingObjectFrameLabel->setFont(font());
    toolLayout->addWidget(m_bindingObjectFrameLabel);
    m_bindingFrameCombo = new QComboBox(this);
    robot_qt_viewer::configureInspectorCombo(m_bindingFrameCombo);
    m_bindingFrameCombo->setToolTip("Object frame used as attachment mount");
    connect(
        m_bindingFrameCombo,
        static_cast<void(QComboBox::*)(int)>(&QComboBox::currentIndexChanged),
        this,
        [this](int) {
            if(!m_objectBindingEditorVisible) {
                return;
            }
            emit objectBindingSelectionChanged(
                currentBindingMountId(),
                currentBindingObjectId(),
                currentBindingFrameId());
        });
    toolLayout->addWidget(m_bindingFrameCombo);

    const QList<QWidget*> diagramBlock = {
        m_bindingSummaryFrame
    };
    int diagramInsertIndex = toolLayout->indexOf(m_robotMountTransformEditor);
    for(QWidget* widget : diagramBlock) {
        if(widget == nullptr || diagramInsertIndex < 0) {
            continue;
        }
        toolLayout->removeWidget(widget);
        toolLayout->insertWidget(diagramInsertIndex, widget);
        ++diagramInsertIndex;
    }

    m_objectBindingDetailsLabel = new QLabel("Select object and frame.", this);
    m_objectBindingDetailsLabel->setWordWrap(true);
    robot_qt_viewer::makeHorizontallyCompressible(m_objectBindingDetailsLabel);
    toolLayout->addWidget(m_objectBindingDetailsLabel);

    toolLayout->addStretch(1);

    auto* taskButtonRow = new QHBoxLayout();
    taskButtonRow->setContentsMargins(0, 0, 0, 0);
    m_applyTaskButton = new QPushButton("Apply", this);
    m_cancelTaskButton = new QPushButton("Cancel", this);
    m_exitTaskButton = new QPushButton("Exit Frame Editor", this);
    m_applyTaskButton->setEnabled(false);
    m_cancelTaskButton->setEnabled(true);
    m_exitTaskButton->setVisible(false);
    taskButtonRow->addWidget(m_applyTaskButton);
    taskButtonRow->addWidget(m_cancelTaskButton);
    taskButtonRow->addWidget(m_exitTaskButton);
    toolLayout->addLayout(taskButtonRow);

    connect(m_applyTaskButton, &QPushButton::clicked, this, [this]() {
        if(m_objectBindingEditorVisible) {
            emit objectBindingApplyRequested(
                currentBindingMountId(),
                currentBindingObjectId(),
                currentBindingFrameId());
            return;
        }
        const bool hasMountTransform = m_mountTransformEditorVisible && m_robotMountTransformEditor != nullptr;
        const bool hasAttachmentOffset = m_attachmentOffsetEditorVisible && m_attachmentOffsetEditor != nullptr;
        const bool hasToolAsset = m_toolAssetEditorVisible && m_toolAssetEditor != nullptr;
        emit taskApplyRequested(
            hasMountTransform,
            hasMountTransform ? m_robotMountTransformEditor->transform() : simulation_project::TransformDesc(),
            m_attachmentInstanceEditorVisible,
            attachmentInstance(),
            hasAttachmentOffset,
            hasAttachmentOffset ? m_attachmentOffsetEditor->transform() : simulation_project::TransformDesc(),
            hasToolAsset,
            hasToolAsset ? m_toolAssetEditor->asset() : simulation_project::AttachmentAssetDesc());
    });
    connect(m_cancelTaskButton, &QPushButton::clicked, this, &ToolSetupWidget::taskCancelRequested);
    connect(m_exitTaskButton, &QPushButton::clicked, this, &ToolSetupWidget::taskExitRequested);

    applyFrameEditorLayout();
    if(m_applyTaskButton != nullptr) {
        m_applyTaskButton->setVisible(false);
    }
    if(m_cancelTaskButton != nullptr) {
        m_cancelTaskButton->setVisible(false);
    }
    if(m_exitTaskButton != nullptr) {
        m_exitTaskButton->setVisible(false);
    }
}

void ToolSetupWidget::setViewModel(const ToolSetupPanelView& view)
{
    setDocumentView(view);
}

void ToolSetupWidget::setDocumentView(const ToolSetupPanelView& view)
{
    setMountItems(view.mountItems, view.selectedMountId, view.mountItemsEnabled);
    setMountDetails(view.mountDetails);
    setMountFrameNameEditor(
        view.mountFrameNameEditorVisible,
        view.mountFrameNameEditorEnabled,
        view.mountFrameName);
    setMountLinkItems(view.mountLinkItems, view.selectedMountLinkName, view.mountLinkItemsEnabled);
    setMountActionEnabled(view.addMountEnabled, view.deleteMountEnabled);
    setMountTransformEditor(
        view.mountTransformEditorVisible,
        view.mountTransformEditorEnabled,
        view.mountTransformEditorTitle,
        view.mountTransform);
    setRobotMountFramePinned(
        view.robotMountFramePinnedEnabled,
        view.robotMountFramePinned);
    setAttachmentItems(view.attachmentItems, view.selectedAttachmentId, view.attachmentItemsEnabled);
    setAttachmentDetails(view.attachmentDetails, view.attachmentToolTip);
    setConfigureAttachmentEnabled(view.configureAttachmentEnabled);
    setAttachmentInstanceEditor(
        view.attachmentInstanceEditorVisible,
        view.attachmentInstanceEditorEnabled,
        view.attachmentInstanceEditorAttachment,
        view.attachmentAssetItems);
    setAttachmentOffsetEditor(
        view.attachmentOffsetEditorVisible,
        view.attachmentOffsetEditorEnabled,
        view.attachmentOffsetEditorTitle,
        view.attachmentOffset);
    setAssetItems(view.assetItems, view.selectedAssetId, view.assetItemsEnabled);
    setAssetDetails(view.assetDetails, view.assetToolTip);
    setAttachAssetEnabled(view.attachAssetEnabled);
    setEditAssetEnabled(view.editAssetEnabled);
    setAssetEditor(view.assetEditorVisible, view.assetEditorEnabled, view.assetEditorAsset);
    applyFrameEditorLayout();
    if(view.bindingView.visible) {
        setObjectBindingEditor(
            true,
            view.bindingView,
            QVector<ToolSetupComboItem>(),
            view.selectedMountId,
            QVector<ToolSetupComboItem>(),
            QString(),
            QVector<ToolSetupComboItem>(),
            QString(),
            QString(),
            false);
    } else {
        setObjectBindingEditor(
            false,
            ToolSetupBindingView(),
            QVector<ToolSetupComboItem>(),
            QString(),
            QVector<ToolSetupComboItem>(),
            QString(),
            QVector<ToolSetupComboItem>(),
            QString(),
            QString(),
            false);
    }
    setMountFrameMode(view.mountFrameMode);
}

void ToolSetupWidget::setMountFrameMode(ToolSetupMountFrameMode mode)
{
    if(mode == ToolSetupMountFrameMode::Create || mode == ToolSetupMountFrameMode::Edit) {
        setFrameEditorMode(mode == ToolSetupMountFrameMode::Create);
        return;
    }

    if(m_frameEditorTitleLabel != nullptr) {
        m_frameEditorTitleLabel->setVisible(false);
    }
    if(m_applyTaskButton != nullptr && !m_objectBindingEditorVisible) {
        m_applyTaskButton->setVisible(false);
    }
    if(m_cancelTaskButton != nullptr && !m_objectBindingEditorVisible) {
        m_cancelTaskButton->setVisible(false);
    }
    if(m_exitTaskButton != nullptr) {
        m_exitTaskButton->setVisible(false);
    }
}

void ToolSetupWidget::setFrameEditorMode(bool newMountFrame)
{
    m_objectBindingEditorVisible = false;
    if(m_frameEditorTitleLabel != nullptr) {
        m_frameEditorTitleLabel->setText(newMountFrame ? "New Mount Frame" : "Mount Frame Editor");
        m_frameEditorTitleLabel->setVisible(true);
    }
    if(m_robotMountNameLabel != nullptr) {
        m_robotMountNameLabel->setVisible(true);
        m_robotMountNameLabel->setEnabled(true);
    }
    if(m_robotMountNameEdit != nullptr) {
        m_robotMountNameEdit->setVisible(true);
        m_robotMountNameEdit->setEnabled(true);
    }
    if(m_robotMountLinkCombo != nullptr) {
        m_robotMountLinkCombo->setVisible(true);
        m_robotMountLinkCombo->setEnabled(m_robotMountLinkCombo->count() > 1);
    }
    if(m_robotMountTransformEditor != nullptr) {
        m_robotMountTransformEditor->setVisible(true);
        m_robotMountTransformEditor->setEnabled(true);
    }
    if(m_frameVisibilitySectionTitle != nullptr) {
        m_frameVisibilitySectionTitle->setVisible(true);
    }
    if(m_showRobotMountFrameCheck != nullptr) {
        m_showRobotMountFrameCheck->setVisible(true);
        m_showRobotMountFrameCheck->setEnabled(true);
    }
    if(m_showLinkFrameCheck != nullptr) {
        m_showLinkFrameCheck->setVisible(false);
    }
    if(m_showToolMountFrameCheck != nullptr) {
        m_showToolMountFrameCheck->setVisible(false);
    }
    if(m_showVisualFrameCheck != nullptr) {
        m_showVisualFrameCheck->setVisible(false);
    }
    if(m_showTcpFrameCheck != nullptr) {
        m_showTcpFrameCheck->setVisible(false);
    }
    if(m_showSensorPreviewCheck != nullptr) {
        m_showSensorPreviewCheck->setVisible(false);
    }
    if(m_bindingSummaryFrame != nullptr) {
        m_bindingSummaryFrame->setVisible(true);
    }
    if(m_objectBindingDiagram != nullptr) {
        m_objectBindingDiagram->setVisible(true);
    }
    if(m_bindingNameTitleLabel != nullptr) {
        m_bindingNameTitleLabel->setVisible(true);
    }
    if(m_bindingNameValueLabel != nullptr) {
        m_bindingNameValueLabel->setVisible(true);
    }
    if(m_bindingObjectNameLabel != nullptr) {
        m_bindingObjectNameLabel->setVisible(false);
    }
    if(m_bindingObjectCombo != nullptr) {
        m_bindingObjectCombo->setVisible(false);
    }
    if(m_bindingObjectFrameLabel != nullptr) {
        m_bindingObjectFrameLabel->setVisible(false);
    }
    if(m_bindingFrameCombo != nullptr) {
        m_bindingFrameCombo->setVisible(false);
    }
    if(m_applyTaskButton != nullptr) {
        m_applyTaskButton->setVisible(true);
        m_applyTaskButton->setEnabled(true);
        m_applyTaskButton->setText(newMountFrame ? "Create" : "Apply");
    }
    if(m_cancelTaskButton != nullptr) {
        m_cancelTaskButton->setVisible(true);
        m_cancelTaskButton->setText("Cancel");
    }
    if(m_exitTaskButton != nullptr) {
        m_exitTaskButton->setVisible(false);
    }
    if(m_taskStatusLabel != nullptr) {
        m_taskStatusLabel->setVisible(false);
    }
}

void ToolSetupWidget::setObjectBindingMode(bool enabled)
{
    m_objectBindingEditorVisible = enabled;
    if(m_frameEditorTitleLabel != nullptr) {
        m_frameEditorTitleLabel->setText("Bind Object To Mount");
        m_frameEditorTitleLabel->setVisible(enabled);
    }
    if(m_applyTaskButton != nullptr) {
        m_applyTaskButton->setVisible(true);
        m_applyTaskButton->setText(enabled ? "Apply Binding" : "Apply");
    }
    if(m_cancelTaskButton != nullptr) {
        m_cancelTaskButton->setVisible(true);
        m_cancelTaskButton->setText(enabled ? "Cancel Binding" : "Cancel");
    }
    if(m_exitTaskButton != nullptr) {
        m_exitTaskButton->setVisible(false);
    }
    if(m_taskStatusLabel != nullptr) {
        m_taskStatusLabel->setVisible(false);
    }
    if(enabled) {
        if(m_robotMountNameLabel != nullptr) {
            m_robotMountNameLabel->setVisible(false);
        }
        if(m_robotMountNameEdit != nullptr) {
            m_robotMountNameEdit->setVisible(false);
        }
        if(m_robotMountTransformEditor != nullptr) {
            m_robotMountTransformEditor->setVisible(false);
        }
        if(m_frameVisibilitySectionTitle != nullptr) {
            m_frameVisibilitySectionTitle->setVisible(false);
        }
        if(m_showRobotMountFrameCheck != nullptr) {
            m_showRobotMountFrameCheck->setVisible(false);
        }
    }
    applyFrameEditorLayout();
}

void ToolSetupWidget::setMountItems(const QVector<ToolSetupComboItem>& items, const QString& selectedId, bool enabled)
{
    setComboItems(m_robotMountCombo, items, selectedId, enabled);
}

QString ToolSetupWidget::currentMountId() const
{
    return m_robotMountCombo != nullptr ? m_robotMountCombo->currentData().toString() : QString();
}

QString ToolSetupWidget::mountIdAt(int index) const
{
    return comboIdAt(m_robotMountCombo, index);
}

void ToolSetupWidget::setMountDetails(const QString& text)
{
    if(m_robotMountDetailsLabel != nullptr) {
        m_robotMountDetailsLabel->setText(text);
    }
}

void ToolSetupWidget::setMountFrameNameEditor(bool visible, bool enabled, const QString& name)
{
    if(m_robotMountNameEdit == nullptr) {
        return;
    }

    if(m_robotMountNameLabel != nullptr) {
        m_robotMountNameLabel->setVisible(visible);
        m_robotMountNameLabel->setEnabled(enabled);
    }
    const QSignalBlocker blocker(m_robotMountNameEdit);
    m_robotMountNameEdit->setVisible(visible);
    m_robotMountNameEdit->setEnabled(enabled);
    m_robotMountNameEdit->setText(name);
}

QString ToolSetupWidget::currentMountFrameName() const
{
    return m_robotMountNameEdit != nullptr ? m_robotMountNameEdit->text().trimmed() : QString();
}

void ToolSetupWidget::setMountLinkItems(
    const QVector<ToolSetupComboItem>& items,
    const QString& selectedLinkName,
    bool enabled)
{
    setComboItems(m_robotMountLinkCombo, items, selectedLinkName, enabled);
    if(m_robotMountLinkCombo != nullptr) {
        m_robotMountLinkCombo->setVisible(!items.isEmpty());
        m_robotMountLinkCombo->setEnabled(enabled && m_robotMountLinkCombo->count() > 1);
    }
}

QString ToolSetupWidget::currentMountLinkName() const
{
    return m_robotMountLinkCombo != nullptr ? m_robotMountLinkCombo->currentData().toString() : QString();
}

void ToolSetupWidget::setMountActionEnabled(bool addEnabled, bool deleteEnabled)
{
    if(m_addRobotMountButton != nullptr) {
        m_addRobotMountButton->setEnabled(addEnabled);
    }
    if(m_deleteRobotMountButton != nullptr) {
        m_deleteRobotMountButton->setEnabled(deleteEnabled);
    }
}

void ToolSetupWidget::setMountTransformEditor(
    bool visible,
    bool enabled,
    const QString& title,
    const simulation_project::TransformDesc& transform)
{
    m_mountTransformEditorVisible = visible;
    if(m_robotMountTransformEditor != nullptr) {
        m_robotMountTransformEditor->setVisible(visible);
        m_robotMountTransformEditor->setEnabled(enabled);
        m_robotMountTransformEditor->setTitle(title.isEmpty()
            ? QStringLiteral("Frame Transform")
            : title);
        m_robotMountTransformEditor->setTransform(transform);
    }
}

bool ToolSetupWidget::hasMountTransformEditor() const
{
    return m_mountTransformEditorVisible && m_robotMountTransformEditor != nullptr;
}

void ToolSetupWidget::setRobotMountFramePinned(bool enabled, bool checked)
{
    if(m_showRobotMountFrameCheck == nullptr) {
        return;
    }

    const QSignalBlocker blocker(m_showRobotMountFrameCheck);
    m_showRobotMountFrameCheck->setEnabled(enabled);
    m_showRobotMountFrameCheck->setChecked(checked);
}

simulation_project::TransformDesc ToolSetupWidget::currentMountTransform() const
{
    return m_robotMountTransformEditor != nullptr
        ? m_robotMountTransformEditor->transform()
        : simulation_project::TransformDesc();
}

void ToolSetupWidget::setAttachmentItems(const QVector<ToolSetupComboItem>& items, const QString& selectedId, bool enabled)
{
    setComboItems(m_toolAttachmentCombo, items, selectedId, enabled);
}

QString ToolSetupWidget::currentAttachmentId() const
{
    return m_toolAttachmentCombo != nullptr ? m_toolAttachmentCombo->currentData().toString() : QString();
}

QString ToolSetupWidget::attachmentIdAt(int index) const
{
    return comboIdAt(m_toolAttachmentCombo, index);
}

int ToolSetupWidget::attachmentCount() const
{
    return m_toolAttachmentCombo != nullptr ? m_toolAttachmentCombo->count() : 0;
}

void ToolSetupWidget::setAttachmentDetails(const QString& text, const QString& toolTip)
{
    if(m_toolDetailsLabel != nullptr) {
        m_toolDetailsLabel->setText(text);
        m_toolDetailsLabel->setToolTip(toolTip);
    }
}

void ToolSetupWidget::setConfigureAttachmentEnabled(bool enabled)
{
    if(m_configureToolAttachmentButton != nullptr) {
        m_configureToolAttachmentButton->setEnabled(enabled);
    }
}

void ToolSetupWidget::setAttachmentInstanceEditor(
    bool visible,
    bool enabled,
    const simulation_project::MountedAttachmentDesc& attachment,
    const QVector<ToolSetupComboItem>& assetItems)
{
    m_attachmentInstanceEditorVisible = visible;
    m_attachmentEditorAttachment = attachment;

    if(m_attachmentNameEdit != nullptr) {
        const QSignalBlocker blocker(m_attachmentNameEdit);
        m_attachmentNameEdit->setVisible(visible);
        m_attachmentNameEdit->setEnabled(enabled);
        m_attachmentNameEdit->setText(QString::fromStdString(attachment.name));
    }

    if(m_attachmentAssetCombo != nullptr) {
        setComboItems(m_attachmentAssetCombo, assetItems, QString::fromStdString(attachment.assetId), enabled);
        m_attachmentAssetCombo->setVisible(visible);
    }

    if(m_attachmentEnabledCheck != nullptr) {
        const QSignalBlocker blocker(m_attachmentEnabledCheck);
        m_attachmentEnabledCheck->setVisible(visible);
        m_attachmentEnabledCheck->setEnabled(enabled);
        m_attachmentEnabledCheck->setChecked(attachment.enabled);
    }
}

simulation_project::MountedAttachmentDesc ToolSetupWidget::attachmentInstance() const
{
    simulation_project::MountedAttachmentDesc attachment = m_attachmentEditorAttachment;
    if(m_attachmentNameEdit != nullptr) {
        attachment.name = m_attachmentNameEdit->text().trimmed().toStdString();
    }
    if(m_attachmentAssetCombo != nullptr) {
        const QString assetId = m_attachmentAssetCombo->currentData().toString();
        if(!assetId.isEmpty()) {
            attachment.assetId = assetId.toStdString();
        }
    }
    if(m_attachmentEnabledCheck != nullptr) {
        attachment.enabled = m_attachmentEnabledCheck->isChecked();
    }
    return attachment;
}

bool ToolSetupWidget::hasAttachmentInstanceEditor() const
{
    return m_attachmentInstanceEditorVisible;
}

void ToolSetupWidget::focusAttachmentInstanceEditor()
{
    if(m_attachmentNameEdit != nullptr && m_attachmentNameEdit->isVisible() && m_attachmentNameEdit->isEnabled()) {
        m_attachmentNameEdit->setFocus(Qt::OtherFocusReason);
    }
}

void ToolSetupWidget::setAttachmentOffsetEditor(
    bool visible,
    bool enabled,
    const QString& title,
    const simulation_project::TransformDesc& transform)
{
    if(m_attachmentOffsetEditor != nullptr) {
        m_attachmentOffsetEditorVisible = visible;
        m_attachmentOffsetEditor->setVisible(visible);
        m_attachmentOffsetEditor->setEnabled(enabled);
        m_attachmentOffsetEditor->setTitle(title.isEmpty()
            ? QStringLiteral("Attachment offset: Mount -> Asset mount")
            : title);
        m_attachmentOffsetEditor->setTransform(transform);
    }
    m_attachmentOffsetDirty = false;
    if(m_applyAttachmentOffsetButton != nullptr) {
        m_applyAttachmentOffsetButton->setVisible(false);
        m_applyAttachmentOffsetButton->setEnabled(false);
    }
}

bool ToolSetupWidget::hasAttachmentOffsetEditor() const
{
    return m_attachmentOffsetEditorVisible && m_attachmentOffsetEditor != nullptr;
}

simulation_project::TransformDesc ToolSetupWidget::currentAttachmentOffset() const
{
    return m_attachmentOffsetEditor != nullptr
        ? m_attachmentOffsetEditor->transform()
        : simulation_project::TransformDesc();
}

void ToolSetupWidget::setAssetDetails(const QString& text, const QString& toolTip)
{
    if(m_toolAssetDetailsLabel != nullptr) {
        m_toolAssetDetailsLabel->setText(text);
        m_toolAssetDetailsLabel->setToolTip(toolTip);
    }
}

void ToolSetupWidget::setAssetItems(const QVector<ToolSetupComboItem>& items, const QString& selectedId, bool enabled)
{
    setComboItems(m_toolAssetCombo, items, selectedId, enabled);
}

QString ToolSetupWidget::currentAssetId() const
{
    return m_toolAssetCombo != nullptr ? m_toolAssetCombo->currentData().toString() : QString();
}

QString ToolSetupWidget::assetIdAt(int index) const
{
    return comboIdAt(m_toolAssetCombo, index);
}

void ToolSetupWidget::setAttachAssetEnabled(bool enabled)
{
    if(m_attachToolAssetButton != nullptr) {
        m_attachToolAssetButton->setEnabled(enabled);
    }
}

void ToolSetupWidget::setEditAssetEnabled(bool enabled)
{
    if(m_editToolAssetButton != nullptr) {
        m_editToolAssetButton->setEnabled(enabled);
    }
}

void ToolSetupWidget::setAssetEditor(
    bool visible,
    bool enabled,
    const simulation_project::AttachmentAssetDesc& asset)
{
    if(m_toolAssetEditor == nullptr) {
        return;
    }

    m_toolAssetEditorVisible = visible;
    m_toolAssetEditor->setVisible(visible);
    m_toolAssetEditor->setEnabled(enabled);
    m_toolAssetEditor->setAsset(asset);
    m_toolAssetEditor->setApplyButtonVisible(false);
}

bool ToolSetupWidget::hasAssetEditor() const
{
    return m_toolAssetEditorVisible && m_toolAssetEditor != nullptr;
}

simulation_project::AttachmentAssetDesc ToolSetupWidget::currentAssetEditorAsset() const
{
    return m_toolAssetEditor != nullptr
        ? m_toolAssetEditor->asset()
        : simulation_project::AttachmentAssetDesc();
}

void ToolSetupWidget::focusAssetEditor()
{
    if(m_toolAssetEditor != nullptr && m_toolAssetEditor->isVisible() && m_toolAssetEditor->isEnabled()) {
        m_toolAssetEditor->setFocus(Qt::OtherFocusReason);
    }
}

void ToolSetupWidget::setObjectBindingEditor(
    bool visible,
    const ToolSetupBindingView& bindingView,
    const QVector<ToolSetupComboItem>& mountItems,
    const QString& selectedMountId,
    const QVector<ToolSetupComboItem>& objectItems,
    const QString& selectedObjectId,
    const QVector<ToolSetupComboItem>& frameItems,
    const QString& selectedFrameId,
    const QString& details,
    bool applyEnabled)
{
    const bool controlsVisible = visible && bindingView.editable;
    m_objectBindingEditorVisible = visible;
    m_bindingMountId = selectedMountId;
    if(m_objectBindingSectionTitle != nullptr) {
        m_objectBindingSectionTitle->setVisible(false);
    }
    if(m_bindingSummaryFrame != nullptr) {
        m_bindingSummaryFrame->setVisible(visible);
    }
    if(m_bindingNameTitleLabel != nullptr) {
        m_bindingNameTitleLabel->setVisible(visible);
    }
    if(m_bindingNameValueLabel != nullptr) {
        m_bindingNameValueLabel->setVisible(visible);
        m_bindingNameValueLabel->setText(bindingView.bindingName.isEmpty()
            ? QStringLiteral("Pending binding")
            : bindingView.bindingName);
    }
    if(m_objectBindingDiagram != nullptr) {
        m_objectBindingDiagram->setVisible(visible && bindingView.visible);
        m_objectBindingDiagram->setBindingView(bindingView);
    }
    setComboItems(m_bindingMountCombo, mountItems, selectedMountId, controlsVisible);
    setComboItems(m_bindingObjectCombo, objectItems, selectedObjectId, controlsVisible);
    setComboItems(m_bindingFrameCombo, frameItems, selectedFrameId, controlsVisible);
    if(m_bindingMountCombo != nullptr) {
        m_bindingMountCombo->setVisible(false);
    }
    if(m_bindingObjectNameLabel != nullptr) {
        m_bindingObjectNameLabel->setVisible(controlsVisible);
    }
    if(m_bindingObjectCombo != nullptr) {
        m_bindingObjectCombo->setVisible(controlsVisible);
    }
    if(m_bindingObjectFrameLabel != nullptr) {
        m_bindingObjectFrameLabel->setVisible(controlsVisible);
    }
    if(m_bindingFrameCombo != nullptr) {
        m_bindingFrameCombo->setVisible(controlsVisible);
    }
    if(m_objectBindingDetailsLabel != nullptr) {
        m_objectBindingDetailsLabel->setVisible(false);
        m_objectBindingDetailsLabel->setText(details);
    }
    if(m_applyTaskButton != nullptr && visible) {
        m_applyTaskButton->setVisible(controlsVisible);
        m_applyTaskButton->setEnabled(applyEnabled);
    }
    if(m_cancelTaskButton != nullptr && visible) {
        m_cancelTaskButton->setVisible(controlsVisible);
        m_cancelTaskButton->setEnabled(true);
    }
}

QString ToolSetupWidget::currentBindingMountId() const
{
    return m_bindingMountId;
}

QString ToolSetupWidget::currentBindingObjectId() const
{
    return m_bindingObjectCombo != nullptr ? m_bindingObjectCombo->currentData().toString() : QString();
}

QString ToolSetupWidget::currentBindingFrameId() const
{
    return m_bindingFrameCombo != nullptr ? m_bindingFrameCombo->currentData().toString() : QString();
}

void ToolSetupWidget::setTaskDirty(bool dirty, const QString& message)
{
    m_taskDirty = dirty;
    if(m_taskStatusLabel != nullptr) {
        if(!message.isEmpty()) {
            m_taskStatusLabel->setText(message);
        } else {
            m_taskStatusLabel->setText(dirty ? "Unsaved mount/attachment edits" : "Ready");
        }
    }
    if(m_applyTaskButton != nullptr) {
        const bool mountFrameEditorActive =
            m_mountTransformEditorVisible && !m_objectBindingEditorVisible;
        m_applyTaskButton->setEnabled(dirty || mountFrameEditorActive);
    }
    if(m_cancelTaskButton != nullptr) {
        m_cancelTaskButton->setEnabled(true);
    }
}

void ToolSetupWidget::markTaskDirty()
{
    if(m_taskDirty) {
        return;
    }
    setTaskDirty(true);
    emit taskDirtyChanged(true);
}

void ToolSetupWidget::applyFrameEditorLayout()
{
    const QList<QWidget*> hiddenWidgets = {
        m_robotMountCombo,
        m_robotMountDetailsLabel,
        m_robotMountLinkCombo,
        m_addRobotMountButton,
        m_deleteRobotMountButton,
        m_attachmentSectionTitle,
        m_toolAttachmentCombo,
        m_toolDetailsLabel,
        m_configureToolAttachmentButton,
        m_attachmentNameEdit,
        m_attachmentAssetCombo,
        m_attachmentEnabledCheck,
        m_attachmentOffsetEditor,
        m_applyAttachmentOffsetButton,
        m_assetSectionTitle,
        m_toolAssetDetailsLabel,
        m_toolAssetCombo,
        m_toolAssetEditor,
        m_importToolAssetButton,
        m_attachToolAssetButton,
        m_editToolAssetButton,
        m_frameVisibilitySectionTitle,
        m_showLinkFrameCheck,
        m_showRobotMountFrameCheck,
        m_showToolMountFrameCheck,
        m_showVisualFrameCheck,
        m_showTcpFrameCheck,
        m_showSensorPreviewCheck,
        m_objectBindingSectionTitle,
        m_bindingSummaryFrame,
        m_bindingNameTitleLabel,
        m_bindingNameValueLabel,
        m_objectBindingDiagram,
        m_bindingMountCombo,
        m_bindingObjectNameLabel,
        m_bindingObjectCombo,
        m_bindingObjectFrameLabel,
        m_bindingFrameCombo,
        m_objectBindingDetailsLabel
    };

    for(QWidget* widget : hiddenWidgets) {
        if(widget != nullptr) {
            widget->setVisible(false);
        }
    }
}

bool ToolSetupWidget::showLinkFrame() const
{
    return m_showLinkFrameCheck == nullptr || m_showLinkFrameCheck->isChecked();
}

bool ToolSetupWidget::showRobotMountFrame() const
{
    return m_showRobotMountFrameCheck == nullptr || m_showRobotMountFrameCheck->isChecked();
}

bool ToolSetupWidget::showToolMountFrame() const
{
    return m_showToolMountFrameCheck == nullptr || m_showToolMountFrameCheck->isChecked();
}

bool ToolSetupWidget::showVisualFrame() const
{
    return m_showVisualFrameCheck == nullptr || m_showVisualFrameCheck->isChecked();
}

bool ToolSetupWidget::showTcpFrame() const
{
    return m_showTcpFrameCheck == nullptr || m_showTcpFrameCheck->isChecked();
}

bool ToolSetupWidget::showSensorPreview() const
{
    return m_showSensorPreviewCheck == nullptr || m_showSensorPreviewCheck->isChecked();
}
