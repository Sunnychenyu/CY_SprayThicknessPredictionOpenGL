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



#include <vector>
using std::vector;

#include <ModelTangQing/scene_interface.h>
#include <RadiationSystemTangQing/RTSKinematics.h>


struct UserDefinedDirection
{
	float			m_udLongitude;
	float			m_udLatitude;
	unsigned int	m_udSampleNum;
	float			m_udSampleInitial;

	UserDefinedDirection() {
		m_udLongitude = 0.0f;
		m_udLatitude = 1.2f;
		m_udSampleNum = 10;
		m_udSampleInitial = 0.5f;
	}
};



class ROBOTSYSTEMTANGQING_DLLCLASS ISOCenter
{
private:
	iscene_spointer			m_spScene;

public:
	static const float longitude_range[2];		//	Range from -pi to pi
	static const float latitude_range[2];		//	Range from -pi/2 to pi/2

	//	General parameters
	float	m_center[3];					//	The center of ISO sphere
	float	m_radius;						//	The radius of ISO sphere

	//	Sampling result
	vector<RTSKinematics>		m_vDeliveryDir;

	//	models for display
	vector<RTSKinematics>		m_vUSDir;
	imodel_spointer				m_spUSGLModel;
	glm::vec4					m_usColor;
	bool						m_bISODraw;

	//	Sampling methods
	enum SAMPLING_TYPE
	{
		SAMPLING_TYPE_UNIFORM,
		SAMPLING_TYPE_USERSPECIFIED,
		SAMPLING_TYPE_RANDOM,
	};
	SAMPLING_TYPE	m_samplingType;

	//	Uniform sampling parameters
	unsigned int	m_usLongitudeNum;		//	The number of samples along the longitude
	unsigned int	m_usLatitudeNum;		//	The number of samples along the latitude

	//	User specified sampling parameters
	vector<UserDefinedDirection>	m_udDirection;
	unsigned int	m_currentUserDirection;

	//	Random sampling parameters
	unsigned int	m_rsSampleNum;

public:
	ISOCenter(iscene_spointer _scene);
	~ISOCenter();

	void start_uniformSampling(void);
	void generate_usGLModel(void);
	void set_visible(bool _flag = true) { m_bISODraw = _flag; }
	void draw(void) const;
	vector<RTSKinematics>& get_dir(void) { return m_vUSDir; }

	glm::vec4 get_usColor(void) { return m_usColor; }
	void set_usColor(glm::vec4 _color) { m_usColor = _color; }
};

