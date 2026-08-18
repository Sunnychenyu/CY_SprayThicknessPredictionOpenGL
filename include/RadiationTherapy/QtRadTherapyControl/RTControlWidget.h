#pragma once

#include <QWidget>
namespace Ui { class RTControlWidget; };

#include <QtRadTherapyControl/rtcontrol_global.h>

#include <RadiationSystemTangQing/iRadTherapySystem.h>

class RTCONTROL_EXPORT RTControlWidget : public QWidget
{
	Q_OBJECT
public:
	RTControlWidget(QWidget *parent = Q_NULLPTR);
	~RTControlWidget();

private:
	Ui::RTControlWidget *ui;

	//	System - main data structure
	rtplanning_spointer			m_spPlanning;

public:
	void set_system(rtplanning_spointer _system);
	void update_ui(void);

private slots :
	void iso_setting(void);
	void gen_feasible(void);
	void gen_target(void);

	void update_display(void);
};
