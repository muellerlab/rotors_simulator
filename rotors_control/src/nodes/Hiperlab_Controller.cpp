/*
 * Hiperlab_Controller.cpp

 *
 *  Created on: Feb 10, 2019
 *      Author: ean
 */

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "Hiperlab_Controller.hpp"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

std::mutex cmdRadioChannelMutex;  //protect against concurrency problems

HiperlabController::HiperlabController(
    const ros::NodeHandle& nh, const ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh) {
  QuadParams();

  //ROS Subscriber to Radio Command. Define the subscriber parameters and then sub.
  ros::SubscribeOptions so = ros::SubscribeOptions::create<
      hiperlab_rostools::radio_command>(
      "/radio_command" + std::to_string(vehicleId), 1,
      boost::bind(&HiperlabController::callbackRadioCmd, this, _1),
      ros::VoidPtr(), &this->rosQueue);
  this->subRadioCmd = this->nh_.subscribe(so);

  odom_sub = nh_.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                           &HiperlabController::Odometry, this);

  //ROS Publisher to telemetry
  pubTelemetry.reset(
      new ros::Publisher(
          nh_.advertise<hiperlab_rostools::telemetry>(
              "/telemetry" + std::to_string(vehicleId), 1)));

  /*pubMoCap.reset(
      new ros::Publisher(
          nh_.advertise < hiperlab_rostools::mocap_output
   > ("/mocap_output" + std::to_string(vehicleId), 1)));*/

  imu_sub_ = nh_.subscribe(mav_msgs::default_topics::IMU, 1,
                                &HiperlabController::Run, this);

  motor_velocity_ = nh_.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  //handle ROS multi-threading
  this->rosQueueThread = std::thread(
      std::bind(&HiperlabController::QueueThread, this));

}

void HiperlabController::QuadParams() {

  //create the vehicles
  Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
      Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehicleId);
  Onboard::QuadcopterConstants vehConsts(quadcopterType);
  _battVoltage = 1.2 * vehConsts.lowBatteryThreshold;
  _battCurrent = -1.0;

  _IMU_yaw = vehConsts.IMU_yaw;
  _IMU_pitch = vehConsts.IMU_pitch;
  _IMU_roll = vehConsts.IMU_roll;
  _R_inverse =
      Rotationf::FromEulerYPR(_IMU_yaw, _IMU_pitch, _IMU_roll).Inverse()
          .GetRotationMatrix();

  _logic.reset(
      new Onboard::QuadcopterLogic(&simTimer, 1.0 / frequencySimulation));
  _logic->Initialise(quadcopterType, vehicleId);

  _timerOnboardLogic.reset(new Timer(&simTimer));
  _onboardLogicPeriod = 1.0 / frequencySimulation;


  //Initialize the queue to listen to radio commands
  cmdRadioChannel.queue.reset(
      new Simulation::CommunicationsDelay<
          RadioTypes::RadioMessageDecoded::RawMessage>(
          &simTimer, timeDelayOffboardControlLoop));

  //Initialize Timers
  Timer cmdRadioTimer(&simTimer);
  debugTimer.reset(new Timer(&simTimer));
  timePrintNextInfo = 0;

  t.reset(new Timer(&simTimer));

  //timePublishNextMocap = 0;
  timePublishNextTelemetry = 0;

}

void HiperlabController::Run(const sensor_msgs::Imu &_imu) {

  _angVel = Vec3d(_imu.angular_velocity.x, _imu.angular_velocity.y,
                  _imu.angular_velocity.z);

  _accel = Vec3d(_imu.linear_acceleration.x, _imu.linear_acceleration.y,
                 _imu.linear_acceleration.z);

  _att = Rotationd(_imu.orientation.x, _imu.orientation.y, _imu.orientation.z,
                   _imu.orientation.w);

  if (_timerOnboardLogic->GetSeconds<double>() > _onboardLogicPeriod) {
    _timerOnboardLogic->AdjustTimeBySeconds(-_onboardLogicPeriod);

  rateGyroMeas = Vec3f(_angVel);
  accMeas = Vec3f(_accel);

  _logic->SetBatteryMeasurement(_battVoltage, _battCurrent);

  float const TEMP_MEAS = 25;  //made up temperature for simulation
  _logic->SetIMUMeasurementTemperature(TEMP_MEAS);


  _logic->SetIMUMeasurementRateGyro(rateGyroMeas.x, rateGyroMeas.y,
                                      rateGyroMeas.z);

    _logic->SetIMUMeasurementAccelerometer(accMeas.x, accMeas.y,
                                           accMeas.z + 9.81);

  _logic->Run();

  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < 4; i++) {
    actuator_msg->angular_velocities.push_back(_logic->GetMotorSpeedCmd(i));
      thrust[i] = (_logic->GetMotorSpeedCmd(i)) * (_logic->GetMotorSpeedCmd(i))
          * 7.64e-06;
    }

  motor_velocity_.publish(actuator_msg);

    GetCurrentTelemetry();

  }
}

void HiperlabController::Odometry(const nav_msgs::Odometry &odom) {

  /*hiperlab_rostools::mocap_output current_mocap;
   gazebo::math::Pose current_pose;*/

 _pos = Vec3d(odom.pose.pose.position.x, odom.pose.pose.position.y,
 odom.pose.pose.position.z);

 _vel = Vec3d(odom.twist.twist.linear.x, odom.twist.twist.linear.y,
 odom.twist.twist.linear.z);

  position = Vec3f(_pos);
  velocity = Vec3f(_vel);

  /*if (t->GetSeconds<double>() > timePublishNextMocap) {
    timePublishNextMocap += 1 / frequencyMocapOutput;

  current_mocap.vehicleID = vehicleId;

  current_pose.pos.x = odom.pose.pose.position.x;
  current_pose.pos.y = odom.pose.pose.position.y;
  current_pose.pos.z = odom.pose.pose.position.z;

  current_pose.rot.w = odom.pose.pose.orientation.w;
  current_pose.rot.x = odom.pose.pose.orientation.x;
  current_pose.rot.y = odom.pose.pose.orientation.y;
  current_pose.rot.z = odom.pose.pose.orientation.z;

  current_mocap.posx = current_pose.pos.x;
  current_mocap.posy = current_pose.pos.y;
  current_mocap.posz = current_pose.pos.z;

  current_mocap.attq0 = current_pose.rot.w;
  current_mocap.attq1 = current_pose.rot.x;
  current_mocap.attq2 = current_pose.rot.y;
  current_mocap.attq3 = current_pose.rot.z;

  gazebo::math::Vector3 quatToEuler = current_pose.rot.GetAsEuler();
  current_mocap.attroll = quatToEuler.x;
  current_mocap.attpitch = quatToEuler.y;
  current_mocap.attyaw = quatToEuler.z;

  current_mocap.header.stamp = ros::Time::now();

  pubMoCap->publish(current_mocap);
   }*/

}

void HiperlabController::callbackRadioCmd(
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

void HiperlabController::GetCurrentTelemetry() {
  if (t->GetSeconds<double>() > timePublishNextTelemetry) {
    //Telemetry message
    timePublishNextTelemetry += 1 / frequencyTelemetry;

  hiperlab_rostools::telemetry telMsgOut;

  //Fill out the telemetry package
  TelemetryPacket::data_packet_t dataPacketRaw1, dataPacketRaw2;
  _logic->GetTelemetryDataPackets(dataPacketRaw1, dataPacketRaw2);

  TelemetryPacket::TelemetryPacket dataPacket1, dataPacket2;
  TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw1, dataPacket1);
  TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw2, dataPacket2);

  telMsgOut.packetNumber = dataPacket1.packetNumber;
  for (int i = 0; i < 3; i++) {
    telMsgOut.accelerometer[i] = accMeas[i];
    telMsgOut.rateGyro[i] = rateGyroMeas[i];
    telMsgOut.position[i] = position[i];
  }

  for (int i = 0; i < 4; i++) {
    telMsgOut.motorForces[i] = thrust[i];
  }
  telMsgOut.batteryVoltage = dataPacket1.battVoltage;

  for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_DEBUG_FLOATS; i++) {
    telMsgOut.debugVals[i] = dataPacket2.debugVals[i];
  }

  Vec3f attYPR = Rotationf::FromVectorPartOfQuaternion(
      Vec3f(dataPacket2.attitude[0], dataPacket2.attitude[1],
            dataPacket2.attitude[2])).ToEulerYPR();
  for (int i = 0; i < 3; i++) {
    telMsgOut.velocity[i] = velocity[i];
    telMsgOut.attitude[i] = _att[i];
    telMsgOut.attitudeYPR[i] = attYPR[i];

  }
  telMsgOut.panicReason = dataPacket2.panicReason;
  telMsgOut.warnings = dataPacket2.warnings;

  telMsgOut.header.stamp = ros::Time::now();

  pubTelemetry->publish(telMsgOut);
  }
}


//Handle ROS multi-threading
void HiperlabController::QueueThread() {
  static const double timeout = 0.01;
  while (this->nh_.ok()) {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "Hiperlab_Controller");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  rotors_control::HiperlabController Hiperlab_Controller(
      nh, private_nh);

  ros::spin();

  return 0;
}





