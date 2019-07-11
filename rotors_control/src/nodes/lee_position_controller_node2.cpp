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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "lee_position_controller_node2.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

std::mutex cmdRadioChannelMutex;  //protect against concurrency problems

LeePositionControllerNode2::LeePositionControllerNode2(
  const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
  :nh_(nh),
   private_nh_(private_nh){
  QuadParams();

  //ROS Subscriber to Radio Command. Define the subscriber parameters and then sub.
  ros::SubscribeOptions so = ros::SubscribeOptions::create<
      hiperlab_rostools::radio_command>(
      "/radio_command" + std::to_string(vehicleId), 1,
      boost::bind(&LeePositionControllerNode2::callbackRadioCmd, this, _1),
      ros::VoidPtr(), &this->rosQueue);
  this->subRadioCmd = this->nh_.subscribe(so);

  odometry_sub_ = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                                &LeePositionControllerNode2::Run,
                                this);

  motor_velocity_reference_pub_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  //handle ROS multi-threading
  this->rosQueueThread = std::thread(
      std::bind(&LeePositionControllerNode2::QueueThread, this));

}

LeePositionControllerNode2::~LeePositionControllerNode2() {
}

void LeePositionControllerNode2::Publish() {
}

void LeePositionControllerNode2::QuadParams() {

  //create the vehicles
  Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
      Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehicleId);
  Onboard::QuadcopterConstants vehConsts(quadcopterType);
  _battVoltage = vehConsts.lowBatteryThreshold + 0.5;
  _battCurrent = -1.0;

  _logic.reset(
      new Onboard::QuadcopterLogic(&simTimer, 1.0 / frequencySimulation));
  _logic->Initialise(quadcopterType, vehicleId);

  //Initialize vectors for gyro
  current_attitude = Vec3f(0, 0, 0);
  current_accelerometer = Vec3f(0, 0, 0);
  current_rateGyro = Vec3f(0, 0, 0);
  //to avoid getting the 0 vector, see #11 on git repo
  current_accelerometer[2] = 0.1;

  _timerOnboardLogic.reset(new Timer(&simTimer));
  _onboardLogicPeriod = 1.0 / frequencySimulation;

  //Initialize the queue to listen to radio commands
  cmdRadioChannel.queue.reset(
      new Simulation::CommunicationsDelay<
          RadioTypes::RadioMessageDecoded::RawMessage>(
          &simTimer, timeDelayOffboardControlLoop));

  Timer cmdRadioTimer(&simTimer);
  //Initialize Timers
  debugTimer.reset(new Timer(&simTimer));
  timePrintNextInfo = 0;

}

void LeePositionControllerNode2::Run(
    const nav_msgs::OdometryConstPtr& odometry_msg) {

  std::lock_guard < std::mutex > guard(cmdRadioChannelMutex);
  if (cmdRadioChannel.queue->HaveNewMessage()) {
    RadioTypes::RadioMessageDecoded msg = RadioTypes::RadioMessageDecoded(
        cmdRadioChannel.queue->GetMessage().raw);
    _logic->SetRadioMessage(msg);
  }


    //TODO: Set Battery Measurements X
    _logic->SetBatteryMeasurement(_battVoltage, _battCurrent);


    _logic->SetIMUMeasurementRateGyro(current_rateGyro[0], current_rateGyro[1],
                                      current_rateGyro[2]);

  float const TEMP_MEAS = 25;  //made up temperature for simulation
  _logic->SetIMUMeasurementTemperature(TEMP_MEAS);

    _logic->SetIMUMeasurementAccelerometer(current_accelerometer[0],
                                           current_accelerometer[1],
                                           current_accelerometer[2]);
    _logic->Run();

  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < 4; i++) {
    actuator_msg->angular_velocities.push_back(_logic->GetMotorSpeedCmd(i));
  }

  motor_velocity_reference_pub_.publish(actuator_msg);

}

void LeePositionControllerNode2::callbackRadioCmd(
    const hiperlab_rostools::radio_command::ConstPtr &mess) {
  std::lock_guard < std::mutex > guard(cmdRadioChannelMutex);
  //todo: should be a nicer way to do this, using e.g. memcpy...
  RadioTypes::RadioMessageDecoded::RawMessage rawMsg;
  for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
    rawMsg.raw[i] = mess->raw[i];
  }
  cmdRadioChannel.queue->AddMessage(rawMsg);

  RadioTypes::RadioMessageDecoded msg = RadioTypes::RadioMessageDecoded(
      rawMsg.raw);
  _logic->SetRadioMessage(msg);
  return;
}

//Handle ROS multi-threading
void LeePositionControllerNode2::QueueThread() {
  static const double timeout = 0.01;
  while (this->nh_.ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lee_position_controller_node");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::LeePositionControllerNode2 lee_position_controller_node2(
      nh, private_nh);

  ros::spin();

  return 0;
}
