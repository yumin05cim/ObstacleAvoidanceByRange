// -*- C++ -*-
/*!
* @file  ObstacleAvoidanceByRange.cpp
* @brief Obstacle Avoidance by Range
* @date $Date$
*
* $Id$
*/

#include "ObstacleAvoidanceByRange.h"
#include "ObstacleAvoidanceByRangeImpl.h"
#include "stdlib.h"

// Module specification
// <rtc-template block="module_spec">
static const char* obstacleavoidancebyrange_spec[] =
  {
    "implementation_id", "ObstacleAvoidanceByRange",
    "type_name",         "ObstacleAvoidanceByRange",
    "description",       "Obstacle Avoidance by Range",
    "version",           "1.0.0",
    "vendor",            "yumin05cim",
    "category",          "Navigation",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.threshold_r1", "-0.42",
    "conf.default.threshold_r2", "-1.26",
    "conf.default.threshold_l1", "0.42",
    "conf.default.threshold_l2", "1.26",
    "conf.default.frontObstacle_avoidance", "stop",
    "conf.default.farthest_obstacleDistance", "0.3",
    // Widget
    "conf.__widget__.threshold_r1", "text",
    "conf.__widget__.threshold_r2", "text",
    "conf.__widget__.threshold_l1", "text",
    "conf.__widget__.threshold_l2", "text",
    "conf.__widget__.frontObstacle_avoidance", "text",
    "conf.__widget__.farthest_obstacleDistance", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
* @brief constructor
* @param manager Maneger Object
*/

ssr::ObstacleAvoidanceByRange_Impl impl;

ObstacleAvoidanceByRange::ObstacleAvoidanceByRange(RTC::Manager* manager)
	// <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_odometryIn("odometry", m_odometry),
    m_rangeIn("range", m_range),
    m_targetVelocityOut("targetVelocity", m_targetVelocity),
    m_stateOut("state", m_state)

	// </rtc-template>
{
}

/*!
* @brief destructor
*/
ObstacleAvoidanceByRange::~ObstacleAvoidanceByRange()
{
}



RTC::ReturnCode_t ObstacleAvoidanceByRange::onInitialize()
{
	// Registration: InPort/OutPort/Service
	// <rtc-template block="registration">
  // Set InPort buffers
  addInPort("odometry", m_odometryIn);
  addInPort("range", m_rangeIn);
  
  // Set OutPort buffer
  addOutPort("targetVelocity", m_targetVelocityOut);
  addOutPort("state", m_stateOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
	// </rtc-template>

	// <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("threshold_r1", m_threshold_r1, "-0.42");
  bindParameter("threshold_r2", m_threshold_r2, "-1.26");
  bindParameter("threshold_l1", m_threshold_l1, "0.42");
  bindParameter("threshold_l2", m_threshold_l2, "1.26");
  bindParameter("frontObstacle_avoidance", m_frontObstacle_avoidance, "stop");
  bindParameter("farthest_obstacleDistance", m_farthest_obstacleDistance, "0.3");
	// </rtc-template>

	return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t ObstacleAvoidanceByRange::onFinalize()
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ObstacleAvoidanceByRange::onStartup(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ObstacleAvoidanceByRange::onShutdown(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t ObstacleAvoidanceByRange::onActivated(RTC::UniqueId ec_id)
{
	m_Mode = NORMAL_MODE;
	m_Obstacle = NO_OBSTACLE;

	std::cout << "[RTC::ObstacleAvoidanceByRange] Successfully Activated." << std::endl;

	return RTC::RTC_OK;
}


RTC::ReturnCode_t ObstacleAvoidanceByRange::onDeactivated(RTC::UniqueId ec_id)
{
	return RTC::RTC_OK;
}


RTC::ReturnCode_t ObstacleAvoidanceByRange::onExecute(RTC::UniqueId ec_id)
{
	if( m_rangeIn.isNew() ){
		m_rangeIn.read();

		impl.setRange(m_range);
		impl.setFarthestObstacleDistance(m_farthest_obstacleDistance);

		int smallest_index = -1;
		double shortest_distance = 1000000.0;

		int index_r1 = impl.angleToIndex(m_threshold_r1);
		int index_r2 = impl.angleToIndex(m_threshold_r2);
		int index_r3 = impl.angleToIndex(m_range.config.minAngle);
		int index_l1 = impl.angleToIndex(m_threshold_l1);
		int index_l2 = impl.angleToIndex(m_threshold_l2);
		int index_l3 = impl.angleToIndex(m_range.config.maxAngle) - 2;
		//std::cout << "r3:" << index_r3 << ", r2:" << index_r2 << ", r1:" << index_r1 
		//			<< ", l1:" << index_l1 << ", l2:" << index_l2 << ", l3:" << index_l3 << std::endl;


		/*
		for(int i = 0; i < m_range.ranges.length(); i++){
		//std::cout << m_range.ranges.length() << std::endl;
		//std::cout << "range[" << i << "] " << m_range.ranges[i] << std::endl;
		}

		double smallest_angle_rad = 0.0;
		smallest_angle_rad = m_range.config.minAngle + m_range.config.angularRes * smallest_index;

		double smallest_angle_deg = 0.0;
		smallest_angle_deg = smallest_angle_rad * 180.0 / 3.14;

		std::cout << "Smallest = (" << smallest_index << ") dist=" << shortest_distance 
		<< ", angle(rad)=" << smallest_angle_rad << ", angle(deg)=" << smallest_angle_deg << std::endl;
		*/

		f_sright = false, f_right = false, f_front = false, f_left = false, f_sleft = false;

		f_sright = impl.ObstacleDetection(index_r3, index_r2, f_sright);
		f_right  = impl.ObstacleDetection(index_r2, index_r1, f_right);
		f_front  = impl.ObstacleDetection(index_r1, index_l1, f_front);
		f_left   = impl.ObstacleDetection(index_l1, index_l2, f_left);
		f_sleft  = impl.ObstacleDetection(index_l2, index_l3, f_sleft);

		if( f_right && f_front && f_left )	{ m_Obstacle = OBSTACLE_R_F_L; }
		else if( f_right && f_front )		{ m_Obstacle = OBSTACLE_R_F; }
		else if( f_right && f_left )		{ m_Obstacle = OBSTACLE_R_L; }
		else if( f_front && f_left )		{ m_Obstacle = OBSTACLE_F_L; }
		else if( f_right || f_sright )		{ m_Obstacle = OBSTACLE_R; }
		else if( f_front )					{ m_Obstacle = OBSTACLE_F; }
		else if( f_left || f_sleft )		{ m_Obstacle = OBSTACLE_L; }
		else								{ m_Obstacle = NO_OBSTACLE; }

		/*
		if( shortest_distance < m_farthest_obstacleDistance ){
		if( smallest_angle_rad < m_threshold_r2 ){
		m_Mode = S_RIGHT_AVOID_MODE;	//1
		}else if( m_threshold_r2 < smallest_angle_rad && smallest_angle_rad < m_threshold_r1 ){
		m_Mode = RIGHT_AVOID_MODE;		//2
		}else if( m_threshold_r1 < smallest_angle_rad && smallest_angle_rad < m_threshold_l1 ){
		m_Mode = FRONT_AVOID_MODE;		//3
		}else if( m_threshold_l1 < smallest_angle_rad && smallest_angle_rad < m_threshold_l2 ){
		m_Mode = LEFT_AVOID_MODE;		//4
		}else if( m_threshold_l2 < smallest_angle_rad ){
		m_Mode = S_LEFT_AVOID_MODE;		//5
		}
		}else{
		m_Mode = NORMAL_MODE;	//0
		}

		*/
		std::string s_frontObstacle_avoidance = m_frontObstacle_avoidance; 
		double vx=0.0, vy=0.0, va=0.0;

		if(m_Obstacle == OBSTACLE_R_F_L) {	//11
			if( s_frontObstacle_avoidance == "stop"){ vx = 0.0; vy = 0.0; va += 0.0; }
			if( s_frontObstacle_avoidance == "back"){ vx = -0.1; vy = 0.0; va += 0.0; }
			if( s_frontObstacle_avoidance == "go_right"){ vx = -0.1; vy = 0.0; va += -0.5; }
			if( s_frontObstacle_avoidance == "go_left"){ vx = -0.1; vy = 0.0; va += 0.5; }

			if( s_frontObstacle_avoidance == "random"){ 
				int rand_num = std::rand() % 6 + 1;
				std::cout << m_Obstacle << " OBSTACLE Right_Front_Left: " << rand_num << std::endl;
				if( rand_num == 1 ){	//stop
					vx = 0.0; vy = 0.0; va += 0.0; 
				}
				if( rand_num == 2 ){	//back
					vx = -0.1; vy = 0.0; va += 0.0;
				}
				if( rand_num == 3 || rand_num == 5 ){	//back_right
					vx = -0.1; vy = 0.0; va += -0.5;
				}
				if( rand_num == 4 || rand_num == 6 ){	//back_left
					vx = -0.1; vy = 0.0; va += 0.5;
				}
			}
		}

		if(m_Obstacle == OBSTACLE_F) {	//16
			int rand_num = std::rand() % 6 + 1;
			std::cout << m_Obstacle << " OBSTACLE Front: " << rand_num << std::endl;
			if( rand_num == 1 || rand_num == 3 || rand_num == 5 ){	//go_right
				vx = 0.1; vy = 0.0; va += -0.5;
			}
			if( rand_num == 2 || rand_num == 4 || rand_num == 6 ){	//go_left
				vx = 0.1; vy = 0.0; va += 0.5;
			}
		}

		if(m_Obstacle == NO_OBSTACLE)	{	//10,go_straight
			std::cout << m_Obstacle << " No Obstacle" << std::endl;
			vx = 0.1; vy = 0.0; va += 0.0;	}
		if(m_Obstacle == OBSTACLE_R_L)  {	//13,go_straight
			std::cout << m_Obstacle << " Obstacle Right_Left" << std::endl;
			vx = 0.1; vy = 0.0; va += 0.0;	}
		if(m_Obstacle == OBSTACLE_R_F)  {	//12,turn_left
			std::cout << m_Obstacle << " Obstacle Front_Right" << std::endl;
			vx = -0.1; vy = 0.0; va += 0.5;	}
		if(m_Obstacle == OBSTACLE_F_L)  {	//14,turn_right
			std::cout << m_Obstacle << " Obstacle Front_Left" << std::endl;
			vx = -0.1; vy = 0.0; va += -0.5;  }
		if(m_Obstacle == OBSTACLE_R)	{	//15,go_left
			std::cout << m_Obstacle << " Obstacle Right" << std::endl;
			vx = 0.0; vy = 0.0; va += 0.5;    }
		if(m_Obstacle == OBSTACLE_L)	{	//17,go_right
			std::cout << m_Obstacle << " Obstacle Left" << std::endl;
			vx = 0.0; vy = 0.0; va += -0.5;   }

		m_state.data = m_Obstacle;
		::setTimestamp<TimedState>(m_state);
		m_stateOut.write();

		/*
		if(m_Mode == NORMAL_MODE){			//go_straight
		vx = 0.1; vy = 0.0; va += 0.0;
		}
		if(m_Mode == S_RIGHT_AVOID_MODE){	//turn_left
		vx = 0.0; vy = 0.0; va += 0.5;
		}
		if(m_Mode == RIGHT_AVOID_MODE){
		vx = 0.1; vy = 0.0; va += 0.5;	//go_left
		}
		if(m_Mode == FRONT_AVOID_MODE){
		if( s_frontObstacle_avoidance == "stop"){ vx = 0.0; vy = 0.0; va = 0.0; }
		if( s_frontObstacle_avoidance == "back"){ vx = -0.1; vy = 0.0; va = 0.0; }
		if( s_frontObstacle_avoidance == "go_right"){ vx = -0.1; vy = 0.0; va = -0.5; }
		if( s_frontObstacle_avoidance == "go_left"){ vx = -0.1; vy = 0.0; va = 0.5; }

		if( s_frontObstacle_avoidance == "random"){ 
		int rand_num = std::rand() % 6 + 1;
		std::cout << rand_num << std::endl;
		if( rand_num == 1 ){	//stop
		vx = 0.0; vy = 0.0; va = 0.0; 
		}
		if( rand_num == 2 ){	//back
		vx = -0.1; vy = 0.0; va = 0.0;
		}
		if( rand_num == 3 || rand_num == 5 ){	//go_right
		vx = -0.1; vy = 0.0; va = -0.5;
		}
		if( rand_num == 4 || rand_num == 6 ){	//go_left
		vx = -0.1; vy = 0.0; va = 0.5;
		}
		}
		}

		if(m_Mode == LEFT_AVOID_MODE){		//go_right
		vx = 0.1; vy = 0.0; va += -0.5;
		}
		if(m_Mode == S_LEFT_AVOID_MODE){	//turn_right
		vx = 0.0; vy = 0.0; va += -0.5;
		}

		*/
		m_targetVelocity.data.vx = vx;
		m_targetVelocity.data.vy = vy;
		m_targetVelocity.data.va = va;

		//std::cout << "Obstacle = " << m_Obstacle << std::endl;

		std::cout <<  "vx = " << m_targetVelocity.data.vx << ", ";
		std::cout <<  "vy = " << m_targetVelocity.data.vy << ", ";
		std::cout <<  "va = " << m_targetVelocity.data.va << std::endl;

		::setTimestamp<TimedVelocity2D>(m_targetVelocity);
		m_targetVelocityOut.write();

	}

	return RTC::RTC_OK;
}

/*
int RadianToIndex(double angle_rad)
{
int index = (angle_rad - m_range.config.minAngle) / m_range.config.angularRes;
return index;
}
*/

/*
RTC::ReturnCode_t ObstacleAvoidanceByRange::onAborting(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ObstacleAvoidanceByRange::onError(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ObstacleAvoidanceByRange::onReset(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ObstacleAvoidanceByRange::onStateUpdate(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t ObstacleAvoidanceByRange::onRateChanged(RTC::UniqueId ec_id)
{
return RTC::RTC_OK;
}
*/


extern "C"
{

	void ObstacleAvoidanceByRangeInit(RTC::Manager* manager)
	{
		coil::Properties profile(obstacleavoidancebyrange_spec);
		manager->registerFactory(profile,
			RTC::Create<ObstacleAvoidanceByRange>,
			RTC::Delete<ObstacleAvoidanceByRange>);
	}

};


