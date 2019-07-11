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

#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <iostream>
#include <../stdio.h>
#include <../boost/bind.hpp>

#include </usr/include/gazebo-7/gazebo/gazebo.hh>
#include </usr/include/gazebo-7/gazebo/physics/physics.hh>
#include </usr/include/gazebo-7/gazebo/math/gzmath.hh>
#include </usr/include/gazebo-7/gazebo/transport/transport.hh>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/common.h"
#include "rotors_control/lee_position_controller.h"

#include "Common/Time/ManualTimer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Simulation/UWBNetwork.hpp"
#include "Components/Logic/QuadcopterLogic.hpp"
#include "Components/Simulation/CommunicationsDelay.hpp"

#include <fstream>

#include "rotors_control/mocap_output.h"
#include "rotors_control/estimator_output.h"
#include "rotors_control/radio_command.h"
#include "rotors_control/joystick_values.h"
#include "rotors_control/telemetry.h"

using namespace std;

mutex cmdRadioChannelMutex;  //protect against concurrency problems

class SimVehicle {
 public:
  struct {
    shared_ptr<
        Simulation::CommunicationsDelay<
            RadioTypes::RadioMessageDecoded::RawMessage> > queue;
  } cmdRadioChannel;

  shared_ptr<ros::Subscriber> subRadioCmd;
  shared_ptr<ros::Publisher> pubSimTruth;
  shared_ptr<ros::Publisher> pubMoCap;
  shared_ptr<ros::Publisher> pubTelemetry;
  int id;
  int numVehicles = 1;
  std::shared_ptr<Onboard::QuadcopterLogic> _logic;

  void callbackRadioCmd(const hiperlab_rostools::radio_command& msg) {
    std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
    //todo: should be a nicer way to do this, using e.g. memcpy...
    RadioTypes::RadioMessageDecoded::RawMessage rawMsg;
    for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
      rawMsg.raw[i] = msg.raw[i];
    }
    cmdRadioChannel.queue->AddMessage(rawMsg);
    return;
  }
};

std::vector<shared_ptr<SimVehicle> > vehicles;

template<typename Real>
string toCSV(const Vec3<Real> v) {
  stringstream ss;
  ss << v.x << "," << v.y << "," << v.z << ",";
  return ss.str();
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "hovering_example2");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher actuators_pub =
        nh.advertise<mav_msgs::Actuators>(
            mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
  /*ROS_INFO("Started hovering example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  //ros::Duration(5.0).sleep();*/

  shared_ptr<SimVehicle> v;
  mav_msgs::Actuators actuators_msg;
  actuators_msg.header.stamp = ros::Time::now();
  ros::Publisher motor_velocity_reference_pub_;
  vector<float> _motorSpeedCommand;

  actuators_msg.angular_velocities.clear();
    for (int i = 0; i < 4; ++i) {
    		  _motorSpeedCommand[i] = v->_logic->GetMotorSpeedCmd(i);
    	      actuators_msg.angular_velocities.push_back(_motorSpeedCommand[i]);
    	  }
    motor_velocity_reference_pub_.publish(actuators_msg);

  /*int const vehicleId = 37;
  v.reset(new SimVehicle());
  v->id = vehicleId;
      v->subRadioCmd.reset(
          new ros::Subscriber(
              nh.subscribe("radio_command" + std::to_string(vehicleId), 1,
                          &SimVehicle::callbackRadioCmd, v.get())));

      v->pubMoCap.reset(
          new ros::Publisher(
              nh.advertise<hiperlab_rostools::mocap_output>(
                  "mocap_output" + std::to_string(vehicleId), 1)));
      v->pubTelemetry.reset(
          new ros::Publisher(
              nh.advertise<hiperlab_rostools::telemetry>(
                  "telemetry" + std::to_string(vehicleId), 1)));

      vehicles.push_back(v);

      std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
        	    if (v->cmdRadioChannel.queue->HaveNewMessage()) {
        	      RadioTypes::RadioMessageDecoded msg = RadioTypes::RadioMessageDecoded(
        	        v->cmdRadioChannel.queue->GetMessage().raw);
        	      v->_logic->SetRadioMessage(msg);
        	    }


    hiperlab_rostools::telemetry current_telemetry;
    TelemetryPacket::data_packet_t dataPacketRaw1, dataPacketRaw2;
    v->_logic->GetTelemetryDataPackets(dataPacketRaw1, dataPacketRaw2);

  	TelemetryPacket::TelemetryPacket dataPacket1, dataPacket2;
  	TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw1, dataPacket1);
  	TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw2, dataPacket2);

  	current_telemetry.packetNumber = dataPacket1.packetNumber;
  		for (int i = 0; i < 3; i++) {
  		  current_telemetry.accelerometer[i] = dataPacket1.accel[i];
  		  current_telemetry.rateGyro[i] = dataPacket1.gyro[i];
  		  current_telemetry.position[i] = dataPacket1.position[i];
  	        }

  		for (int i = 0; i < 4; i++) {
  	      current_telemetry.motorForces[i] = dataPacket1.motorForces[i];
  		        }
  		  current_telemetry.batteryVoltage = dataPacket1.battVoltage;

  		for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_DEBUG_FLOATS;
  		            i++) {
  		  current_telemetry.debugVals[i] = dataPacket2.debugVals[i];
  		        }

  		Vec3f attYPR = Rotationf::FromVectorPartOfQuaternion(
  			Vec3f(dataPacket2.attitude[0], dataPacket2.attitude[1],
  		                  dataPacket2.attitude[2])).ToEulerYPR();
  		for (int i = 0; i < 3; i++) {
  		  current_telemetry.velocity[i] = dataPacket2.velocity[i];
  		  current_telemetry.attitude[i] = dataPacket2.attitude[i];
  		  current_telemetry.attitudeYPR[i] = attYPR[i];

  		        }
  		  current_telemetry.panicReason = dataPacket2.panicReason;
  		  current_telemetry.warnings = dataPacket2.warnings;

  		  current_telemetry.header.stamp = ros::Time::now();

  		  v->pubTelemetry->publish(current_telemetry);*/

  ros::spin();

  return 0;
}
