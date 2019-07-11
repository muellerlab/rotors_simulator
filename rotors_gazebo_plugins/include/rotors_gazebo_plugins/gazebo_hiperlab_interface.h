/*
 * gazebo_hiperlab_interface.h
 *
 *  Created on: Jan 15, 2019
 *      Author: ean
 */

#ifndef ROTORS_GAZEBO_PLUGINS_INCLUDE_ROTORS_GAZEBO_PLUGINS_GAZEBO_HIPERLAB_INTERFACE_H_
#define ROTORS_GAZEBO_PLUGINS_INCLUDE_ROTORS_GAZEBO_PLUGINS_GAZEBO_HIPERLAB_INTERFACE_H_

#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo_client.hh>

//============= GAZEBO MSG TYPES ==============//
#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"
#include "Actuators.pb.h"
#include "CommandMotorSpeed.pb.h"

//===============for ROS====================//
#include <mav_msgs/default_topics.h>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"

//=============== ROS MSG TYPES ===============//
#include <mav_msgs/Actuators.h>
#include "rotors_gazebo_plugins/common.h"

//=============== Hiperlab ================//
#include "hiperlab_rostools/simulator_truth.h"
#include "hiperlab_rostools/telemetry.h"
#include "hiperlab_rostools/mocap_output.h"
#include "hiperlab_rostools/radio_command.h"

#include "Common/Time/ManualTimer.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Simulation/Quadcopter_T.hpp"
#include "Components/Simulation/CommunicationsDelay.hpp"
#include "Components/Logic/QuadcopterLogic.hpp"

namespace gazebo{

// typedef's to make life easier
typedef const boost::shared_ptr<const gz_std_msgs::ConnectGazeboToRosTopic>
    GzConnectGazeboToRosTopicMsgPtr;
typedef const boost::shared_ptr<const gz_std_msgs::ConnectRosToGazeboTopic>
    GzConnectRosToGazeboTopicMsgPtr;
typedef const boost::shared_ptr<const gz_sensor_msgs::Actuators>
    GzActuatorsMsgPtr;
static const std::string kDefaultMotorVelocityReferenceTopic = "gazebo/command/motor_speed";

class GazeboHiperlabInterface : public ModelPlugin {
 public:
	GazeboHiperlabInterface()
      : ModelPlugin(),
		received_first_reference_(false),
		pubs_and_subs_created_(false),
        namespace_(kDefaultNamespace),
        // DEFAULT TOPICS
        motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferenceTopic),
        command_motor_speed_sub_topic_(mav_msgs::default_topics::COMMAND_ACTUATORS),
        //---------------
        node_handle_(NULL){}
  ~GazeboHiperlabInterface();

  void InitializeParams();
  void Publish();

 protected:
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo& /*_info*/);

 private:

   /// \brief    Gets set to true the first time a motor command is received.
   /// \details  OnUpdate() will not do anything until this is true.
   bool received_first_reference_;

   /// \brief    Flag that is set to true once CreatePubsAndSubs() is called, used
   ///           to prevent CreatePubsAndSubs() from be called on every OnUpdate().
   bool pubs_and_subs_created_;

   void CreatePubsAndSubs();

   std::string namespace_;
   std::string motor_velocity_reference_pub_topic_;
   std::string command_motor_speed_sub_topic_;

   /// \brief  Handle for the ROS node.
   ros::NodeHandle* nh_;

   gazebo::transport::NodePtr node_handle_;
   gazebo::transport::PublisherPtr motor_velocity_reference_pub_;
   gazebo::transport::SubscriberPtr cmd_motor_sub_;

   physics::ModelPtr model_;
   physics::WorldPtr world_;

   /// \brief Pointer to the update event connection.
   event::ConnectionPtr updateConnection_;

   boost::thread callback_queue_thread_;

   //ROS Publisher for telemetry warnings
   ros::Publisher pubTelemetry;
   hiperlab_rostools::telemetry GetCurrentTelemetry();
   hiperlab_rostools::telemetry current_telemetry;

   std::shared_ptr<Timer> debugTimer;
   double timePrintNextInfo;
   std::shared_ptr<Timer> _timerOnboardLogic;
   double _onboardLogicPeriod;

   struct SimVehicle {
       struct {
         std::shared_ptr<
             Simulation::CommunicationsDelay<
                 RadioTypes::RadioMessageDecoded::RawMessage> > queue;
       } cmdRadioChannel;
       std::shared_ptr<Onboard::QuadcopterLogic> _logic;
     };

   std::shared_ptr<SimVehicle> vehicle;
   int const vehicleID = 37;
   physics::ModelPtr model;
   std::mutex cmdRadioChannelMutex;  //protect against concurrency problems
   Onboard::QuadcopterConstants::QuadcopterType quadcopterType;
   HardwareTimer simTimer;
   const double frequencySimulation = 500.0;
   const double frequencyLogic = 500.0;
   const double frequencyROS = 200;
   double const timeDelayOffboardControlLoop = 20e-3;
   double timePublishROS = 0;

   float _battVoltage, _battCurrent;
   Vec3f current_attitude, current_accelerometer, current_rateGyro;
};

}
#endif /* ROTORS_GAZEBO_PLUGINS_INCLUDE_ROTORS_GAZEBO_PLUGINS_GAZEBO_HIPERLAB_INTERFACE_H_ */
