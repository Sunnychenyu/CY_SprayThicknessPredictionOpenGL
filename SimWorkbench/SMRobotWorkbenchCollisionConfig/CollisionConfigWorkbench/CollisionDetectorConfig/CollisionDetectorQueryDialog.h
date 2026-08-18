#pragma once

#include "CollisionDetectorsViewModel.h"

#include <QDialog>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;

class CollisionDetectorQueryDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CollisionDetectorQueryDialog(
        const CollisionDetectorQueryContractView& contract,
        QWidget* parent = nullptr);

    CollisionDetectorQueryContractView contract() const;

private:
    CollisionDetectorQueryContractView m_initialContract;
    QLabel* m_idLabel = nullptr;
    QLineEdit* m_nameEdit = nullptr;
    QCheckBox* m_contactsCheck = nullptr;
    QCheckBox* m_normalsCheck = nullptr;
    QCheckBox* m_nearestCheck = nullptr;
    QDoubleSpinBox* m_maxContactsSpin = nullptr;
    QDoubleSpinBox* m_distanceThresholdSpin = nullptr;
    QComboBox* m_roleCombo = nullptr;
};
