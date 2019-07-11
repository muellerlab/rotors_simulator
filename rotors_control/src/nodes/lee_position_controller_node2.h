/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H

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
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "ros/subscribe_options.h"

//=============== Hiperlab ================//
#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/telemetry.h"

#include "Components/Logic/QuadcopterLogic.hpp"
#include "Common/Time/ManualTimer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Simulation/CommunicationsDelay.hpp"


namespace rotors_control {

class LeePositionControllerNode2 {
 public:
  struct {
    std::shared_ptr<
        Simulation::CommunicationsDelay<
            RadioTypes::RadioMessageDecoded::RawMessage> > queue;
  } cmdRadioChannel;
  LeePositionControllerNode2(const ros::NodeHandle& nh,
                             const ros::NodeHandle& private_nh);
  ~LeePositionControllerNode2();

  void QuadParams();
  void Publish();


  void GetEstimate(Vec3f &pos, Vec3f &vel, Rotationf &att,
                   Vec3f &angVel) const {
    _logic->GetEstimate(pos, vel, att, angVel);
  }

  virtual void AddUWBRadioTarget(uint8_t id, Vec3f pos) {
    _logic->AddRangingTargetId(id, pos);
  }

  virtual void GetTelemetryDataPackets(
      TelemetryPacket::data_packet_t &dataPacket1,
      TelemetryPacket::data_packet_t &dataPacket2) {
    _logic->GetTelemetryDataPackets(dataPacket1, dataPacket2);
    return;
  }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::NodeHandle* node_;

  std::string namespace_;

  ros::Subscriber odometry_sub_;

  ros::Publisher motor_velocity_reference_pub_;

  int const vehicleId = 4;
  int id = vehicleId;

  Onboard::QuadcopterConstants::QuadcopterType quadcopterType;
  HardwareTimer simTimer;
  const double frequencySimulation = 500.0;
  const double frequencyLogic = 500.0;
  const double frequencyROS = 200;
  double const timeDelayOffboardControlLoop = 20e-3;
  double timePublishROS = 0;

  float _battVoltage, _battCurrent;
  Vec3f current_attitude, current_accelerometer, current_rateGyro;

  std::shared_ptr<Timer> debugTimer;
  double timePrintNextInfo;
  std::shared_ptr<Timer> _timerOnboardLogic;
  float _onboardLogicPeriod;

  void Run(const nav_msgs::OdometryConstPtr& odometry_msg);

  //Radio Messages
  ros::Subscriber subRadioCmd;
  void callbackRadioCmd(const hiperlab_rostools::radio_command::ConstPtr &mess);

  ros::CallbackQueue rosQueue;
  std::thread rosQueueThread;
  void QueueThread();

  std::shared_ptr<Onboard::QuadcopterLogic> _logic;

};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H
