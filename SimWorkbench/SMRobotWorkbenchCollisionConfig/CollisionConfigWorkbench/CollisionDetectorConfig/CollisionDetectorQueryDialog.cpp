#include "CollisionDetectorQueryDialog.h"

#include "RobotQtWidgetUtils.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

namespace
{
    using robot_qt_viewer::configureInspectorCombo;
    using robot_qt_viewer::configureInspectorForm;
    using robot_qt_viewer::makeHorizontallyCompressible;

    void setComboData(QComboBox* combo, const QString& value)
    {
        if(combo == nullptr) {
            return;
        }

        const int index = combo->findData(value);
        combo->setCurrentIndex(index >= 0 ? index : 0);
    }
}

CollisionDetectorQueryDialog::CollisionDetectorQueryDialog(
    const CollisionDetectorQueryContractView& contract,
    QWidget* parent)
    : QDialog(parent)
    , m_initialContract(contract)
{
    setWindowTitle("Configure Detector");
    setModal(true);

    auto* rootLayout = new QVBoxLayout(this);
    rootLayout->setContentsMargins(12, 12, 12, 12);
    rootLayout->setSpacing(10);

    auto* form = new QFormLayout();
    form->setContentsMargins(0, 0, 0, 0);
    form->setSpacing(8);
    configureInspectorForm(form);

    m_idLabel = new QLabel(this);
    m_idLabel->setText(contract.id.isEmpty() ? QStringLiteral("<unassigned>") : contract.id);
    m_idLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    m_idLabel->setToolTip("Stable detector id used by other modules.");
    form->addRow("id", m_idLabel);

    m_nameEdit = new QLineEdit(this);
    makeHorizontallyCompressible(m_nameEdit);
    m_nameEdit->setText(contract.name);
    m_nameEdit->setToolTip("Detector display name. Keep names unique when external modules query detectors by name.");
    form->addRow("name", m_nameEdit);

    m_contactsCheck = new QCheckBox("Contacts", this);
    m_contactsCheck->setChecked(contract.contacts);
    m_contactsCheck->setToolTip("Request contact points and fill the Contacts result table.");
    form->addRow(m_contactsCheck);

    m_normalsCheck = new QCheckBox("Normals", this);
    m_normalsCheck->setChecked(contract.normals);
    m_normalsCheck->setToolTip("Request contact normals when contact points are available.");
    form->addRow(m_normalsCheck);
    connect(m_contactsCheck, &QCheckBox::toggled, this, [this](bool checked) {
        if(m_normalsCheck == nullptr) {
            return;
        }
        if(!checked) {
            m_normalsCheck->setChecked(false);
        }
        m_normalsCheck->setEnabled(checked);
    });
    if(!m_contactsCheck->isChecked()) {
        m_normalsCheck->setChecked(false);
        m_normalsCheck->setEnabled(false);
    }

    m_nearestCheck = new QCheckBox("Nearest / distance", this);
    m_nearestCheck->setChecked(contract.nearest);
    m_nearestCheck->setToolTip("Request nearest points and distance for this detector.");
    form->addRow(m_nearestCheck);

    m_maxContactsSpin = new QDoubleSpinBox(this);
    m_maxContactsSpin->setDecimals(0);
    m_maxContactsSpin->setRange(0.0, 10000.0);
    m_maxContactsSpin->setSingleStep(1.0);
    m_maxContactsSpin->setValue(contract.maxContacts);
    m_maxContactsSpin->setToolTip("Maximum number of contact points requested from this detector.");
    form->addRow("max contacts", m_maxContactsSpin);

    m_distanceThresholdSpin = new QDoubleSpinBox(this);
    m_distanceThresholdSpin->setDecimals(4);
    m_distanceThresholdSpin->setRange(0.0, 1000000.0);
    m_distanceThresholdSpin->setSingleStep(0.01);
    m_distanceThresholdSpin->setValue(contract.distanceThreshold);
    m_distanceThresholdSpin->setToolTip("Distance threshold used by the detector query.");
    form->addRow("distance threshold", m_distanceThresholdSpin);

    m_roleCombo = new QComboBox(this);
    configureInspectorCombo(m_roleCombo);
    m_roleCombo->addItem("Exact", "Exact");
    m_roleCombo->addItem("Planning Proxy", "PlanningProxy");
    m_roleCombo->addItem("Simplified", "Simplified");
    m_roleCombo->addItem("Safety Margin", "SafetyMargin");
    m_roleCombo->addItem("Sphere Cover", "SphereCover");
    m_roleCombo->setToolTip("Collision geometry role used by this detector query.");
    setComboData(m_roleCombo, contract.role);
    form->addRow("model role", m_roleCombo);

    rootLayout->addLayout(form);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    rootLayout->addWidget(buttons);
}

CollisionDetectorQueryContractView CollisionDetectorQueryDialog::contract() const
{
    CollisionDetectorQueryContractView view = m_initialContract;
    view.name = m_nameEdit != nullptr ? m_nameEdit->text().trimmed() : QString();
    view.contacts = m_contactsCheck != nullptr && m_contactsCheck->isChecked();
    view.normals = m_normalsCheck != nullptr && m_normalsCheck->isChecked();
    view.nearest = m_nearestCheck != nullptr && m_nearestCheck->isChecked();
    view.maxContacts = m_maxContactsSpin != nullptr ? m_maxContactsSpin->value() : 0.0;
    view.distanceThreshold = m_distanceThresholdSpin != nullptr ? m_distanceThresholdSpin->value() : 0.0;
    view.role = m_roleCombo != nullptr ? m_roleCombo->currentData().toString() : QString("Exact");
    return view;
}
