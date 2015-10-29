// -*- C++ -*-
/*!
 * @file  ObstacleAvoidanceByRange.h
 * @brief Obstacle Avoidance by Range
 * @date  $Date$
 *
 * $Id$
 */

#ifndef OBSTACLEAVOIDANCEBYRANGE_H
#define OBSTACLEAVOIDANCEBYRANGE_H

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/*!
 * @class ObstacleAvoidanceByRange
 * @brief Obstacle Avoidance by Range
 *
 */
class ObstacleAvoidanceByRange
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  ObstacleAvoidanceByRange(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~ObstacleAvoidanceByRange();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  threshold_r1
   * - DefaultValue: -0.42
   */
  double m_threshold_r1;
  /*!
   * 
   * - Name:  threshold_r2
   * - DefaultValue: -1.26
   */
  double m_threshold_r2;
  /*!
   * 
   * - Name:  threshold_l1
   * - DefaultValue: 0.42
   */
  double m_threshold_l1;
  /*!
   * 
   * - Name:  threshold_l2
   * - DefaultValue: 1.26
   */
  double m_threshold_l2;
  /*!
   * 
   * - Name:  frontObstacle_avoidance
   * - DefaultValue: stop
   */
  std::string m_frontObstacle_avoidance;
  /*!
   * 
   * - Name:  farthest_obstacleDistance
   * - DefaultValue: 0.3
   */
  double m_farthest_obstacleDistance;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedPose2D m_odometry;
  /*!
   */
  InPort<RTC::TimedPose2D> m_odometryIn;
  RTC::RangeData m_range;
  /*!
   */
  InPort<RTC::RangeData> m_rangeIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedVelocity2D m_targetVelocity;
  /*!
   */
  OutPort<RTC::TimedVelocity2D> m_targetVelocityOut;
  RTC::TimedState m_state;
  /*!
   */
  OutPort<RTC::TimedState> m_stateOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

	 int m_Mode;
	 int m_Obstacle;

	 bool f_sright;
	 bool f_right;
	 bool f_front;
	 bool f_left;
	 bool f_sleft;

	 int RadianToIndex(double angle_rad);

};

const int NORMAL_MODE = 0;
const int S_RIGHT_AVOID_MODE = 1;
const int   RIGHT_AVOID_MODE = 2;
const int   FRONT_AVOID_MODE = 3;
const int    LEFT_AVOID_MODE = 4;
const int  S_LEFT_AVOID_MODE = 5;

const int NO_OBSTACLE    = 10;
const int OBSTACLE_R_F_L = 11;
const int OBSTACLE_R_F   = 12;
const int OBSTACLE_R_L   = 13;
const int OBSTACLE_F_L   = 14;
const int OBSTACLE_R     = 15;
const int OBSTACLE_F     = 16;
const int OBSTACLE_L     = 17;

extern "C"
{
  DLL_EXPORT void ObstacleAvoidanceByRangeInit(RTC::Manager* manager);
};

#endif // OBSTACLEAVOIDANCEBYRANGE_H
