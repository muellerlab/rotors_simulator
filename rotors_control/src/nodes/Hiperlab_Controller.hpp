/*
 * Hiperlab_Controller.hpp
 *
 *  Created on: Feb 10, 2019
 *      Author: ean
 */

#ifndef SRC_NODES_HIPERLAB_CONTROLLER_HPP_
#define SRC_NODES_HIPERLAB_CONTROLLER_HPP_

// SYSTEM INCLUDES
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
#include <../stdio.h>
#include <../boost/bind.hpp>
#include <random>

#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/Pose.h>
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include "rotors_control/common.h"
#include <gazebo/physics/physics.hh>

//=============== Hiperlab ================//
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/telemetry.h"
#include "hiperlab_rostools/mocap_output.h"

#include "Common/Math/Vec3.hpp"
#include "Common/Math/Rotation.hpp"

#include "Components/Logic/QuadcopterLogic.hpp"
#include "Common/Time/ManualTimer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Simulation/CommunicationsDelay.hpp"
#include "Components/Logic/KalmanFilter6DOF.hpp"


namespace rotors_control {

class HiperlabController {
 public:
  struct {
    std::shared_ptr<
        Simulation::CommunicationsDelay<
            RadioTypes::RadioMessageDecoded::RawMessage> > queue;
  } cmdRadioChannel;
  HiperlabController(const ros::NodeHandle& nh,
                             const ros::NodeHandle& private_nh);
  ~HiperlabController() {

  }

  void QuadParams();

  void callbackRadioCmd(const hiperlab_rostools::radio_command::ConstPtr &mess);

  void Odometry(const nav_msgs::Odometry &odom);

  void Pose(const geometry_msgs::Pose &pose);

  void Run(const sensor_msgs::Imu &_imu);

  void GetCurrentTelemetry();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::string namespace_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odom_sub;

  ros::Publisher motor_velocity_;

  int const vehicleId = 14;
  int id = vehicleId;

  Onboard::QuadcopterConstants::QuadcopterType quadcopterType;
  HardwareTimer simTimer;
  const double frequencySimulation = 500.0;
  const double frequencyLogic = 500.0;
  const double frequencyROS = 200;
  double const timeDelayOffboardControlLoop = 20e-3;
  const double frequencyMocapOutput = 200;
  const double frequencyTelemetry = 100;
  double timePublishROS = 0;
  double timePublishNextMocap;
  double timePublishNextTelemetry;

  float _battVoltage, _battCurrent;
  Vec3d _angVel;
  Vec3d _accel;
  Vec3d _pos;
  Vec3d _vel;
  Rotationd _att;
  Vec3f position;
  Vec3f velocity;
  Vec3f rateGyroMeas;
  Vec3f accMeas;
  float thrust[4];

  float _IMU_yaw;  //[rad]
  float _IMU_pitch;
  float _IMU_roll;
  Matrix<float, 3, 3> _R_inverse;  //rotation matrix correponds to IMU's rotation

  std::default_random_engine _generator;
  std::normal_distribution<double> _normalDistribution;

  //Noise characteristics:
  double _stdDevAccNoise, _stdDevRateGyroNoise;


  std::shared_ptr<Timer> debugTimer;
  double timePrintNextInfo;
  std::shared_ptr<Timer> _timerOnboardLogic;
  std::shared_ptr<Timer> t;
  float _onboardLogicPeriod;

  //Radio Messages
  ros::Subscriber subRadioCmd;

  std::shared_ptr<ros::Publisher> pubTelemetry;
  std::shared_ptr<ros::Publisher> pubMoCap;

  std::shared_ptr<HiperlabController> v;

  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  void QueueThread();

  std::shared_ptr<Onboard::QuadcopterLogic> _logic;
  std::shared_ptr<Onboard::KalmanFilter6DOF> _kalman;

};
}




#endif /* SRC_NODES_HIPERLAB_CONTROLLER_HPP_ */
