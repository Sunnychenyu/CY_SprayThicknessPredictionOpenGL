#pragma once

// GLM Mathematics
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/transform.hpp>


struct planning_data
{
	float	center[3];
	float	radius;
	float	longitude;
	float	latitude;
};

struct end_effector
{
	float		gantry_angle;
	glm::mat4	couch_end;
};

struct rts_structure {
	float center[3];
	float radius;
	float joint_offset[5];
	float joint_ubound[5];
	float joint_lbound[5];
};


struct robot_joints
{
	float joints[5];
	float jointsInOpenGL[5];
};

class RTSKinematics
{
private:
	//	Three type of parameters
	planning_data		m_planningData;
	end_effector		m_endEffector;
	robot_joints		m_joints;

	//	Radiation therapy system's structure parameters
	rts_structure		m_structure;

public:
	RTSKinematics();
	~RTSKinematics();

	void set_center(float _x, float _y, float _z) {
		m_planningData.center[0] = _x;
		m_planningData.center[1] = _y;
		m_planningData.center[2] = _z;
	}
	void set_radius(float _radius) { m_planningData.radius = _radius; }
	void set_dir(float _longitude, float _latitude) {
		m_planningData.longitude = _longitude;
		m_planningData.latitude = _latitude;
	}
	float get_longitude(void) { return m_planningData.longitude; }
	float get_latitude(void) { return m_planningData.latitude; }
	const float* const get_center(void) { return m_planningData.center; }
	float get_radius(void) { return m_planningData.radius; }

	const robot_joints& get_joints(void) { return m_joints; }
	void set_joints(float theta1, float theta2, float theta3, float theta4, float theta5) {
		m_joints.joints[0] = theta1;
		m_joints.joints[1] = theta2;
		m_joints.joints[2] = theta3;
		m_joints.joints[3] = theta4;
		m_joints.joints[4] = theta5;
	}
	const glm::mat4& get_endMatrix(void) { return m_endEffector.couch_end; }



	bool planning2end(void);
	bool planning2end(planning_data& _planningData, end_effector& _endData);

	bool end2joints(void);
	bool end2joints(end_effector& _end, robot_joints& _joints);
	bool isJointsConstraintsSatisfied(robot_joints& _joints);

	bool joints2end(void);
	bool joints2end(robot_joints& _joints, end_effector& _end);
	bool joints2opengljoints(void);
};

