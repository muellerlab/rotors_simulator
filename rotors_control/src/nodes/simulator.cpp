/*
 * main.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: ean
 */
#include<iostream>

#include "rotors_control/hiperlab_controller.h"

using namespace rotors_control;

/*mutex cmdRadioChannelMutex;

class SimVehicle {
 public:
	SimVehicle(const ros::NodeHandle& nh, BaseTimer* const timer, float onboardLogicPeriod);
  struct {
    shared_ptr<
        Simulation::CommunicationsDelay<
            RadioTypes::RadioMessageDecoded::RawMessage> > queue;
  } cmdRadioChannel;

  shared_ptr<ros::Subscriber> subRadioCmd;
  shared_ptr<ros::Publisher> pubSimTruth;
  shared_ptr<ros::Publisher> pubMoCap;
  shared_ptr<ros::Publisher> pubTelemetry;
  int id{};
  int const vehicleId = 37;

  gazebo::physics::ModelPtr vehicle;
  shared_ptr<Onboard::QuadcopterLogic> _logic;

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
};*/


int main(int argc, char** argv) {
  ros::init(argc, argv, "simulator");



  HiperlabController hiperlab_controller;
  hiperlab_controller.GetTelemetry();

  /*ros::NodeHandle n;
  shared_ptr<ros::Subscriber> subRadioCmd;
  shared_ptr<ros::Publisher> pubSimTruth;
  shared_ptr<ros::Publisher> pubMoCap;
  shared_ptr<ros::Publisher> pubTelemetry;
  ros::Publisher actuator_pub =
  n.advertise<mav_msgs::Actuators>(
            mav_msgs::default_topics::COMMAND_ACTUATORS, 1);
  vector<float> _motorSpeedCommand;
  int id{};
  int const vehicleId = 37;


  gazebo::physics::ModelPtr vehicle;
  shared_ptr<Onboard::QuadcopterLogic> _logic;
  gazebo::math::Pose current_pose;

  std::vector<shared_ptr<SimVehicle> > vehicles;

  BaseTimer* masterTimer;
  float onboardLogicPeriod;

  // Create a private node handle for accessing node parameters.

  //Creates messages in Gazebo
    ROS_INFO("Started hovering example.");

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
    ros::Duration(5.0).sleep();

  shared_ptr<SimVehicle> v;
      //v.reset(new SimVehicle(n, masterTimer, onboardLogicPeriod));
      v->id = vehicleId;
      v->subRadioCmd.reset(
          new ros::Subscriber(
              n.subscribe("radio_command" + std::to_string(vehicleId), 1,
                          &SimVehicle::callbackRadioCmd, v.get())));

      v->pubMoCap.reset(
          new ros::Publisher(
              n.advertise<hiperlab_rostools::mocap_output>(
                  "mocap_output" + std::to_string(vehicleId), 1)));
      v->pubTelemetry.reset(
          new ros::Publisher(
              n.advertise<hiperlab_rostools::telemetry>(
                  "telemetry" + std::to_string(vehicleId), 1)));

      vehicles.push_back(v);

////////////////////////////////////////////////////////////////
//Simulator setup
////////////////////////////////////////////////////////////////
//Basic timing:
	const double frequencySimulation = 500.0;  //run the simulation at this rate
	const double frequencyMocapOutput = 200;  //[s] Ideally, 1/mocapOuptputFreq is a constant multiple of 1/frequencySimulation.
	const double frequencyTelemetry = 100;  //[s] Ideally, 1/mocapOuptputFreq is a constant multiple of 1/frequencySimulation.

	const double endTime = 10.0f;  //[s]
	HardwareTimer simTimer;

	//The communication transport delay:
	double const timeDelayOffboardControlLoop = 20e-3;  //[s] TODO: we should measure this!

	for (auto v : vehicles) {
		v->cmdRadioChannel.queue.reset(
				new Simulation::CommunicationsDelay<
					RadioTypes::RadioMessageDecoded::RawMessage>(
					&simTimer, timeDelayOffboardControlLoop));
	}

	for (auto v : vehicles) {
	        std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
	        if (v->cmdRadioChannel.queue->HaveNewMessage()) {
	              v->cmdRadioChannel.queue->GetMessage();
	        }
	      }

	for (auto v : vehicles) {
	        hiperlab_rostools::mocap_output current_mocap;


	        current_mocap.header.stamp = ros::Time::now();

	        current_pose = vehicle->GetWorldPose();

			current_mocap.vehicleID = vehicleId;

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

			v->pubMoCap->publish(current_mocap);

	}

	for (auto v : vehicles) {
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

					v->pubTelemetry->publish(current_telemetry);
		}

	nav_msgs::OdometryConstPtr odometry_msg;
	mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);
	actuator_msg->angular_velocities.clear();
	for (int i = 0; i < 4; ++i) {
		  _motorSpeedCommand[i] = _logic->GetMotorSpeedCmd(i);
		  actuator_msg->angular_velocities.push_back(_motorSpeedCommand[i]);
	  }
	actuator_msg->header.stamp = odometry_msg->header.stamp;
	actuator_pub.publish(actuator_msg);*/

  ros::spin();

  return 0;
}


