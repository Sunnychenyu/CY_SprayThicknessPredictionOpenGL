#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _WIN32
#  ifdef ROBOTSYSTEMTANGQING_DLLCLASS_EXPORTS
#    define ROBOTSYSTEMTANGQING_DLLCLASS __declspec(dllexport)
#  else
#    define ROBOTSYSTEMTANGQING_DLLCLASS __declspec(dllimport)
#  endif
#else
#  define ROBOTSYSTEMTANGQING_DLLCLASS
#endif

#ifdef __cplusplus
}
#endif


#include <ModelTangQing/collision_detector_interface.h>
#include <SMRobotTangQing/robot_interface.h>

#include <boost/shared_ptr.hpp>
#include <vector>
using std::vector;
#include <string>
using std::string;

#include <RadiationSystemTangQing/RadTherapyPlanning.h>
#include <BasicSystemTangQing/BasicSystem.h>

class iRadTherapySystem
	: virtual public iQtOpenGLView
	, public RadTherapyPlanning
	, public BasicSystem
{
public:
	//	Output of the task function
	enum TASK_FOLLOWING
	{
		TASK_STOP = 0,
		TASK_CONTINUE = 1,
		TASK_UPDATE_LINE = 2
	};

public:
	iRadTherapySystem();
	~iRadTherapySystem();

public:
	//	Factory Function
	enum RobotSystemType
	{
		RobotSystemType_Radiation,
		RobotSystemType_Length,
	};
	ROBOTSYSTEMTANGQING_DLLCLASS static boost::shared_ptr<iRadTherapySystem> get_new_system(RobotSystemType type = RobotSystemType_Length);
};

typedef boost::shared_ptr<iRadTherapySystem> irobot_system_spointer;
