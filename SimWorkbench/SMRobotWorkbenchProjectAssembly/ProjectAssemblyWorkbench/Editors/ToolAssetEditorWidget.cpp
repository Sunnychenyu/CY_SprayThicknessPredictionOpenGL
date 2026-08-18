#include "ToolAssetEditorWidget.h"

#include "RobotQtWidgetUtils.h"
#include "ToolTransformEditorWidget.h"

#include <Eigen/Geometry>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QSizePolicy>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <memory>
#include <vector>

namespace
{
    QDoubleSpinBox* makeScaleSpin(QWidget* parent)
    {
        auto* spin = new QDoubleSpinBox(parent);
        spin->setRange(0.0001, 100000.0);
        spin->setDecimals(6);
        spin->setSingleStep(0.1);
        spin->setMinimumHeight(28);
        return spin;
    }

    Eigen::Isometry3d makeTransform(const simulation_project::TransformDesc& desc)
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.translation() = Eigen::Vector3d(desc.x, desc.y, desc.z);
        transform.linear() =
            Eigen::AngleAxisd(desc.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
            Eigen::AngleAxisd(desc.pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
            Eigen::AngleAxisd(desc.roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        return transform;
    }

    simulation_project::TransformDesc makeTransformDesc(const Eigen::Isometry3d& transform)
    {
        simulation_project::TransformDesc desc;
        desc.x = transform.translation().x();
        desc.y = transform.translation().y();
        desc.z = transform.translation().z();
        const Eigen::Vector3d euler = transform.linear().eulerAngles(2, 1, 0);
        desc.yaw = euler[0];
        desc.pitch = euler[1];
        desc.roll = euler[2];
        return desc;
    }

    simulation_project::TransformDesc assetTcpTransform(
        const simulation_project::AttachmentAssetDesc& asset)
    {
        for(const simulation_project::AttachmentFunctionalFrameDesc& frame : asset.functionalFrames) {
            if(frame.primary || frame.frameType == "tcp") {
                return frame.assetMountToFrame;
            }
        }
        return simulation_project::TransformDesc();
    }

    simulation_project::TransformDesc makeModelToFlangeTransform(
        const simulation_project::AttachmentAssetDesc& asset)
    {
        return makeTransformDesc(makeTransform(asset.assetMountToVisual).inverse());
    }

    simulation_project::TransformDesc makeModelToTcpTransform(
        const simulation_project::AttachmentAssetDesc& asset)
    {
        const Eigen::Isometry3d mountToVisual = makeTransform(asset.assetMountToVisual);
        const Eigen::Isometry3d mountToTcp = makeTransform(assetTcpTransform(asset));
        return makeTransformDesc(mountToVisual.inverse() * mountToTcp);
    }
}

class ToolFrameDiagramWidget : public QWidget
{
public:
    explicit ToolFrameDiagramWidget(QWidget* parent = nullptr)
        : QWidget(parent)
    {
        setMinimumHeight(170);
        setMaximumHeight(170);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    void setAssetName(const QString& assetName)
    {
        m_assetName = assetName;
        update();
    }

protected:
    void paintEvent(QPaintEvent*) override
    {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);

        const QRectF area = rect().adjusted(1.0, 1.0, -1.0, -1.0);
        const QColor borderColor(82, 96, 112);
        const QColor nodeBorderColor(82, 156, 238);
        const QColor nodeFillColor(22, 34, 48);
        const QColor textColor(226, 234, 244);
        const QColor mutedColor(130, 144, 160);

        painter.fillRect(area, palette().base());
        painter.setPen(QPen(borderColor, 1.0));
        painter.setBrush(palette().base());
        painter.drawRoundedRect(area, 6.0, 6.0);

        const qreal margin = 16.0;
        const qreal nodeWidth = qMin<qreal>(150.0, qMax<qreal>(104.0, (area.width() - margin * 2.0 - 44.0) / 3.0));
        const qreal nodeHeight = 42.0;
        const qreal y = area.top() + 44.0;
        const qreal gap = qMax<qreal>(22.0, (area.width() - margin * 2.0 - nodeWidth * 3.0) / 2.0);
        const QRectF visualRect(area.left() + margin, y, nodeWidth, nodeHeight);
        const QRectF flangeRect(visualRect.right() + gap, y, nodeWidth, nodeHeight);
        const QRectF tcpRect(flangeRect.right() + gap, y, nodeWidth, nodeHeight);

        drawNode(painter, visualRect, m_assetName.isEmpty() ? QStringLiteral("Model / Visual") : m_assetName, false, nodeFillColor, nodeBorderColor, textColor);
        drawNode(painter, flangeRect, QStringLiteral("Tool flange"), true, nodeFillColor, nodeBorderColor, textColor);
        drawNode(painter, tcpRect, QStringLiteral("TCP"), true, nodeFillColor, nodeBorderColor, textColor);
        drawArrow(painter, visualRect, flangeRect, mutedColor);
        drawArrow(painter, flangeRect, tcpRect, mutedColor);
    }

private:
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
        font.setPointSize(8);
        painter.setFont(font);
        painter.setPen(textColor);
        const QFontMetrics metrics(font);
        painter.drawText(
            rect.adjusted(8.0, 4.0, -8.0, -4.0),
            Qt::AlignCenter,
            metrics.elidedText(text, Qt::ElideMiddle, static_cast<int>(rect.width() - 16.0)));
    }

    void drawArrow(QPainter& painter, const QRectF& from, const QRectF& to, const QColor& color) const
    {
        const QPointF start(from.right(), from.center().y());
        const QPointF end(to.left(), to.center().y());
        painter.setPen(QPen(color, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        painter.drawLine(start, end);

        const qreal arrowSize = 6.0;
        painter.drawLine(end, QPointF(end.x() - arrowSize, end.y() - arrowSize));
        painter.drawLine(end, QPointF(end.x() - arrowSize, end.y() + arrowSize));
    }

    QString m_assetName;
};

ToolAssetEditorWidget::ToolAssetEditorWidget(QWidget* parent)
    : QWidget(parent)
{
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(10);

    m_idEdit = new QLineEdit(this);
    m_idEdit->setReadOnly(true);
    m_idEdit->setToolTip("Stable internal id. It is used by project references and is not renamed here.");
    m_nameEdit = new QLineEdit(this);
    m_nameEdit->setToolTip("Display name shown in the Tool panel and Scene Explorer.");
    m_typeEdit = new QLineEdit(this);
    m_visualPathEdit = new QLineEdit(this);
    m_visualScaleSpin = makeScaleSpin(this);
    m_visualPathEdit->setMinimumHeight(28);
    m_nameEdit->setMinimumHeight(28);
    m_typeEdit->setMinimumHeight(28);
    m_idEdit->setMinimumHeight(28);

    auto* infoLayout = new QFormLayout();
    infoLayout->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
    infoLayout->setVerticalSpacing(8);
    infoLayout->addRow("Internal id", m_idEdit);
    infoLayout->addRow("Display name", m_nameEdit);
    infoLayout->addRow("Type", m_typeEdit);
    infoLayout->addRow("Visual path", m_visualPathEdit);
    infoLayout->addRow("Visual scale", m_visualScaleSpin);
    layout->addLayout(infoLayout);

    auto* sharedAssetHint = new QLabel(
        "Library asset settings are shared by every tool attachment that references this asset. Use attachment offset for per-robot instance changes.",
        this);
    sharedAssetHint->setWordWrap(true);
    layout->addWidget(sharedAssetHint);

    m_frameDiagram = new ToolFrameDiagramWidget(this);
    layout->addWidget(m_frameDiagram);

    m_visualTransformEditor = new ToolTransformEditorWidget("Model/world -> Tool flange {F}", this);
    m_tcpTransformEditor = new ToolTransformEditorWidget("Model/world -> TCP {TCP}", this);
    m_visualTransformEditor->setMatrixVisible(false);
    m_tcpTransformEditor->setMatrixVisible(false);
    layout->addWidget(m_visualTransformEditor);
    layout->addWidget(m_tcpTransformEditor);

    m_applyButton = new QPushButton("Apply Tool Asset", this);
    robot_qt_viewer::makeHorizontallyCompressible(m_applyButton);
    m_applyButton->setVisible(false);
    layout->addWidget(m_applyButton);

    connect(m_nameEdit, &QLineEdit::editingFinished, this, &ToolAssetEditorWidget::emitAssetChanged);
    connect(m_typeEdit, &QLineEdit::editingFinished, this, &ToolAssetEditorWidget::emitAssetChanged);
    connect(m_visualPathEdit, &QLineEdit::editingFinished, this, &ToolAssetEditorWidget::emitAssetChanged);
    connect(
        m_visualScaleSpin,
        static_cast<void(QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
        this,
        [this](double) {
            emitAssetChanged();
        });
    connect(
        m_visualTransformEditor,
        &ToolTransformEditorWidget::transformChanged,
        this,
        &ToolAssetEditorWidget::emitVisualTransformChanged);
    connect(
        m_tcpTransformEditor,
        &ToolTransformEditorWidget::transformChanged,
        this,
        &ToolAssetEditorWidget::emitTcpTransformChanged);
    connect(m_applyButton, &QPushButton::clicked, this, [this]() {
        storeUiToAsset();
        if(m_applyButton != nullptr) {
            m_applyButton->setEnabled(false);
        }
        emit applyRequested(m_asset);
    });
}

void ToolAssetEditorWidget::setAsset(const simulation_project::AttachmentAssetDesc& asset)
{
    m_asset = asset;
    loadAssetToUi();
}

simulation_project::AttachmentAssetDesc ToolAssetEditorWidget::asset() const
{
    return collectUiAsset();
}

void ToolAssetEditorWidget::setApplyButtonVisible(bool visible)
{
    if(m_applyButton != nullptr) {
        m_applyButton->setVisible(visible);
        m_applyButton->setEnabled(false);
    }
}

void ToolAssetEditorWidget::loadAssetToUi()
{
    m_updating = true;

    const QList<QObject*> blockers = {
        m_idEdit,
        m_nameEdit,
        m_typeEdit,
        m_visualPathEdit,
        m_visualScaleSpin,
        m_visualTransformEditor,
        m_tcpTransformEditor
    };
    std::vector<std::unique_ptr<QSignalBlocker>> signalBlockers;
    signalBlockers.reserve(static_cast<std::size_t>(blockers.size()));
    for(QObject* object : blockers) {
        if(object != nullptr) {
            signalBlockers.push_back(std::make_unique<QSignalBlocker>(object));
        }
    }

    m_idEdit->setText(QString::fromStdString(m_asset.id));
    m_nameEdit->setText(QString::fromStdString(m_asset.name));
    m_typeEdit->setText(QString::fromStdString(m_asset.assetType));
    m_visualPathEdit->setText(QString::fromStdString(m_asset.visualPath));
    m_visualScaleSpin->setValue(m_asset.visualScale);
    m_visualTransformEditor->setTransform(makeModelToFlangeTransform(m_asset));
    m_tcpTransformEditor->setTransform(makeModelToTcpTransform(m_asset));
    if(m_frameDiagram != nullptr) {
        const QString assetName = QString::fromStdString(m_asset.name.empty() ? m_asset.id : m_asset.name);
        m_frameDiagram->setAssetName(assetName);
    }
    if(m_applyButton != nullptr) {
        m_applyButton->setEnabled(false);
    }

    m_updating = false;
}

simulation_project::AttachmentAssetDesc ToolAssetEditorWidget::collectUiAsset() const
{
    simulation_project::AttachmentAssetDesc value = m_asset;
    value.name = m_nameEdit->text().trimmed().toStdString();
    value.assetKind = value.assetKind.empty() ? "tool" : value.assetKind;
    value.assetType = m_typeEdit->text().trimmed().toStdString();
    value.visualPath = m_visualPathEdit->text().trimmed().toStdString();
    value.visualScale = m_visualScaleSpin->value();

    const Eigen::Isometry3d modelToFlange = makeTransform(m_visualTransformEditor->transform());
    const Eigen::Isometry3d modelToTcp = makeTransform(m_tcpTransformEditor->transform());
    const Eigen::Isometry3d flangeToModel = modelToFlange.inverse();
    const Eigen::Isometry3d flangeToTcp = flangeToModel * modelToTcp;
    value.assetMountToVisual = makeTransformDesc(flangeToModel);

    simulation_project::AttachmentFunctionalFrameDesc tcpFrame;
    tcpFrame.id = value.id + ".tcp";
    tcpFrame.name = "TCP";
    tcpFrame.frameType = "tcp";
    tcpFrame.assetMountToFrame = makeTransformDesc(flangeToTcp);
    tcpFrame.primary = true;
    bool replaced = false;
    for(simulation_project::AttachmentFunctionalFrameDesc& frame : value.functionalFrames) {
        if(frame.primary || frame.frameType == "tcp") {
            frame = tcpFrame;
            replaced = true;
            break;
        }
    }
    if(!replaced) {
        value.functionalFrames.push_back(tcpFrame);
    }
    return value;
}

void ToolAssetEditorWidget::storeUiToAsset()
{
    m_asset = collectUiAsset();
}

void ToolAssetEditorWidget::emitAssetChanged()
{
    if(m_updating) {
        return;
    }
    storeUiToAsset();
    if(m_applyButton != nullptr && m_applyButton->isVisible()) {
        m_applyButton->setEnabled(true);
    }
    emit assetChanged(m_asset);
}

void ToolAssetEditorWidget::emitVisualTransformChanged()
{
    if(m_updating) {
        return;
    }
    storeUiToAsset();
    if(m_applyButton != nullptr && m_applyButton->isVisible()) {
        m_applyButton->setEnabled(true);
    }
    emit visualTransformChanged(m_asset);
}

void ToolAssetEditorWidget::emitTcpTransformChanged()
{
    if(m_updating) {
        return;
    }
    storeUiToAsset();
    if(m_applyButton != nullptr && m_applyButton->isVisible()) {
        m_applyButton->setEnabled(true);
    }
    emit tcpTransformChanged(m_asset);
}
