/*
 * gazebo_hiperlab_interface.cpp
 *
 *  Created on: Jan 16, 2019
 *      Author: ean
 */

#include "rotors_gazebo_plugins/gazebo_hiperlab_interface.h"

#include "ConnectGazeboToRosTopic.pb.h"
#include "ConnectRosToGazeboTopic.pb.h"

namespace gazebo {

GazeboHiperlabInterface::~GazeboHiperlabInterface() {
  
}

void GazeboHiperlabInterface::Load(physics::ModelPtr _model,
                                     sdf::ElementPtr _sdf) {

	if (kPrintOnPluginLoad) {
	    gzdbg << __FUNCTION__ << "() called." << std::endl;
	  }

	  gzdbg << "_model = " << _model->GetName() << std::endl;

	// Store the pointer to the model.
	  model_ = _model;

	  world_ = model_->GetWorld();

	  namespace_.clear();



	  //==============================================//
	  //========== READ IN PARAMS FROM SDF ===========//
	  //==============================================//
	  if (_sdf->HasElement("robotNamespace")) {
	      namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
	    } else {
	      gzerr << "[gazebo_motor_model] Please specify a robotNamespace.\n";
	    }
	  node_handle_ = gazebo::transport::NodePtr(new transport::Node());

	  // Initialise with default namespace (typically /gazebo/default/)
	  node_handle_->Init();

	  // Listen to the update event. This event is broadcast every
	  // simulation iteration.
	  updateConnection_ = event::Events::ConnectWorldUpdateBegin(
		  boost::bind(&GazeboHiperlabInterface::OnUpdate, this, _1));

	  vehicle.reset(new GazeboHiperlabInterface::SimVehicle());

	  quadcopterType = Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehicleID);
	      Onboard::QuadcopterConstants consts(quadcopterType);
	      _battVoltage = consts.lowBatteryThreshold + 0.5;
	      _battCurrent = -1.0;
	  vehicle->_logic.reset(
	  			new Onboard::QuadcopterLogic(&simTimer, 1.0 / frequencyLogic));
	  vehicle->_logic->Initialise(quadcopterType, vehicleID);

	  //Initialize the queue to listen to radio commands
	  vehicle->cmdRadioChannel.queue.reset(new Simulation::CommunicationsDelay<
				  RadioTypes::RadioMessageDecoded::RawMessage>(
				  &simTimer, timeDelayOffboardControlLoop));

	  //Initialize Timers
	  debugTimer.reset(new Timer(&simTimer));
	  timePrintNextInfo = 0;

	  _timerOnboardLogic.reset(new Timer(&simTimer));
	  _onboardLogicPeriod = 1.0 / frequencySimulation;

	  //ROS Publisher to telemetry
	  this->pubTelemetry = this->nh_->advertise<hiperlab_rostools::telemetry>(
		"/telemetry" + std::to_string(vehicleID),
		1);
}

void GazeboHiperlabInterface::OnUpdate(const common::UpdateInfo& /*_info*/) {
	if (kPrintOnUpdates) {
	    gzdbg << __FUNCTION__ << "() called." << std::endl;
	  }

	  if (!pubs_and_subs_created_) {
	    CreatePubsAndSubs();
	    pubs_and_subs_created_ = true;
	  }

	  if (!received_first_reference_) {
	    return;
	  }

	/*//To handle radio messages
	std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
	if (vehicle->cmdRadioChannel.queue->HaveNewMessage()) {
	  RadioTypes::RadioMessageDecoded msg = RadioTypes::RadioMessageDecoded(
		vehicle->cmdRadioChannel.queue->GetMessage().raw);
	  vehicle->_logic->SetRadioMessage(msg);
	}

	//Run the logic every 1/frequencySimulation seconds. Also publish motor speed values.
	if(_timerOnboardLogic->GetSeconds<double>() > _onboardLogicPeriod) {
	  _timerOnboardLogic->AdjustTimeBySeconds(-_onboardLogicPeriod);

	  //TODO: Set Battery Measurements X
	  vehicle->_logic->SetBatteryMeasurement(_battVoltage, _battCurrent);

	  vehicle->_logic->SetIMUMeasurementRateGyro(current_rateGyro[0],
										current_rateGyro[1],
										current_rateGyro[2]);

	  vehicle->_logic->SetIMUMeasurementAccelerometer(current_accelerometer[0],
											current_accelerometer[1],
											current_accelerometer[2]);
	  vehicle->_logic->Run();
	}*/

	  hiperlab_rostools::telemetry current_telemetry = GetCurrentTelemetry();
	  this->pubTelemetry.publish(current_telemetry);
}

hiperlab_rostools::telemetry GazeboHiperlabInterface::GetCurrentTelemetry() {
	hiperlab_rostools::telemetry telMsgOut;

    //Fill out the telemetry package
    TelemetryPacket::data_packet_t dataPacketRaw1, dataPacketRaw2;
    vehicle->_logic->GetTelemetryDataPackets(dataPacketRaw1, dataPacketRaw2); //<><

    TelemetryPacket::TelemetryPacket dataPacket1, dataPacket2;
    TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw1, dataPacket1);
    TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw2, dataPacket2);

    telMsgOut.packetNumber = dataPacket1.packetNumber;
    for (int i = 0; i < 3; i++) {
    	telMsgOut.accelerometer[i] = dataPacket1.accel[i];
    	telMsgOut.rateGyro[i] = dataPacket1.gyro[i];
    	telMsgOut.position[i] = dataPacket1.position[i];
    }

    for (int i = 0; i < 4; i++) {
    	telMsgOut.motorForces[i] = dataPacket1.motorForces[i];
    }
    telMsgOut.batteryVoltage = dataPacket1.battVoltage;

    for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_DEBUG_FLOATS;
        i++) {
    	telMsgOut.debugVals[i] = dataPacket2.debugVals[i];
    }

    Vec3f attYPR = Rotationf::FromVectorPartOfQuaternion(
        Vec3f(dataPacket2.attitude[0], dataPacket2.attitude[1],
              dataPacket2.attitude[2])).ToEulerYPR();
    for (int i = 0; i < 3; i++) {
    	telMsgOut.velocity[i] = dataPacket2.velocity[i];
    	telMsgOut.attitude[i] = dataPacket2.attitude[i];
    	telMsgOut.attitudeYPR[i] = attYPR[i];

    }
    telMsgOut.panicReason = dataPacket2.panicReason;
    telMsgOut.warnings = dataPacket2.warnings;

    telMsgOut.header.stamp = ros::Time::now();

    return telMsgOut;
  }

GZ_REGISTER_MODEL_PLUGIN(GazeboHiperlabInterface);

} // namespace gazebo
