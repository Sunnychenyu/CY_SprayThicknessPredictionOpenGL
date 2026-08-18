#pragma once
#include <vector>
using std::vector;

#include <RadiationSystemTangQing/ISOCenter.h>
#include <SMRobotTangQing/robot_factory.h>
#include <ModelTangQing/collision_detector_interface.h>

#include <QtOpenGLView/iQtOpenGLView.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
using boost::function;

//	Display selection
enum DISPLAY_FLAG{
	CANDIDATE_SETTING_FLAG = 1,
	CANDIDATE_FLAG = 2,
	FEASIBLE_FLAG = 4,
	TARGET_FLAG = 8,
};

class RadTherapyPlanning
	: virtual public iQtOpenGLView
{
protected:
	//	从等中心的观点设置放射治疗角度
	vector<ISOCenter>			m_vIsocenter;

	//	从isocenter导出放射治疗角度的总集合
	vector<RTSKinematics>		m_vCandidate;
	imodel_spointer				m_spCandidateModel;
	glm::vec4					m_candidateColor;

	//	剔除掉碰撞之后的feasible放射角度总集合
	vector<RTSKinematics>		m_vFeasible;
	imodel_spointer				m_spFeasibleModel;
	glm::vec4					m_feasibleColor;

	//	Target beams
	vector<RTSKinematics>		m_vTarget;
	unsigned int				m_targetNum;
	imodel_spointer				m_spTargetModel;
	glm::vec4					m_targetColor;

	//	Determine which curve should be shown.
	unsigned int				m_displayFlag;

	//	For collision detection, must be initialized before radiation therapy planning
	irobot_spointer				m_gantry;
	irobot_spointer				m_couch;
	icollis_spointer			m_collision_detector;
public:
	RadTherapyPlanning();
	~RadTherapyPlanning();

	//	Initialize the model and collision detector
	void set_rts_models(irobot_spointer _gantry, irobot_spointer _couch, icollis_spointer _detector);

	//	Interfaces for candidate beams generation
	virtual bool gen_candidate(void);
	virtual bool gen_candidate_model(void);
	vector<ISOCenter>&	get_vIsocenter(void) { return m_vIsocenter; }
	void set_vIsocenter(const vector<ISOCenter>& _centers) { m_vIsocenter = _centers; }

	//	Interfaces for feasible candidate beams generation
	virtual bool gen_feasible(void);								//	Eliminate the collision beams from the vector
	virtual bool collision_check(const robot_joints& _robot);		//	Collision detection
	virtual bool gen_feasible_model(void);

	//	Interfaces for target beams generation
	virtual bool gen_target(void);
	virtual bool gen_target_model(void);
	void set_targetNum(unsigned int _num) { m_targetNum = _num; }

	//	The draw function for path planning
	virtual void draw(void);
	virtual imodel_spointer get_model(void) = 0;
	virtual bool update_sphere_model(imodel_spointer _model, vector<RTSKinematics>& _beams, glm::vec4 _color = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f));

	unsigned int display_flag(void) { return m_displayFlag; }
	void display_candidate_setting(void) { m_displayFlag = CANDIDATE_SETTING_FLAG; }
	void display_candidate(void) { m_displayFlag = CANDIDATE_FLAG; }
	void display_feasible(void) { m_displayFlag = FEASIBLE_FLAG; }
	void display_target(void) { m_displayFlag = TARGET_FLAG; }
	
	void reset_rts();

	//	Update related UI
	typedef function<void(void)> RadTherapyControlUpdate_Callback;
	RadTherapyControlUpdate_Callback  m_rtcUpdateFunc;
	void set_RadTherapyControlUpdate_Callbak(RadTherapyControlUpdate_Callback _callback_func) {
		m_rtcUpdateFunc = _callback_func;
	}
};

typedef boost::shared_ptr<RadTherapyPlanning> rtplanning_spointer;