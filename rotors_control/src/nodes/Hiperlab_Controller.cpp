#include "Hiperlab_Controller.hpp"

namespace rotors_control {

std::mutex cmdRadioChannelMutex;  //protect against concurrency problems

HiperlabController::HiperlabController(ros::NodeHandle& nh, int vehicleId,
                                       double frequencyLogic,
                                       double frequencyTelemetry)
    : _nh(nh),
      _vehicleId(vehicleId),
      _subRadioCmd(
          nh.subscribe("/radio_command" + std::to_string(vehicleId), 1,
                       &HiperlabController::callbackRadioCmd, this)),
      _subIMU(
          nh.subscribe(mav_msgs::default_topics::IMU, 1,
                       &HiperlabController::callbackImu, this)),
      _pubMotorVel(
          nh.advertise<mav_msgs::Actuators>(
              mav_msgs::default_topics::COMMAND_ACTUATORS, 1)),
      _pubTelemetry(
          nh.advertise<hiperlab_rostools::telemetry>(
              "/telemetry" + std::to_string(vehicleId), 1)),
      _logic(&_simTimer, 1.0 / frequencyLogic),
      _lastTelemPubTime(ros::Time::now()),
      _telemPubPeriod(1.0 / frequencyTelemetry),
      _lastLogicRunTime(ros::Time::now()),
      _logicRunPeriod(1.0 / frequencyLogic),
      _quadType(Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehicleId)),
      _consts(_quadType) {

  _logic.Initialise(_quadType, _vehicleId);

  // Made up measurements for simulation
  double battVoltage = 1.2 * _consts.lowBatteryThreshold;
  double battCurrent = -1.0;
  double tempMeas = 25;
  _logic.SetBatteryMeasurement(battVoltage, battCurrent);
  _logic.SetIMUMeasurementTemperature(tempMeas);

  //Initialize the queue to listen to radio commands
  double timeDelayOffboardControlLoop = 0.0;
  _cmdRadioChannel.queue.reset(
      new Simulation::CommunicationsDelay<
          RadioTypes::RadioMessageDecoded::RawMessage>(
          &_simTimer, timeDelayOffboardControlLoop));

}

void HiperlabController::callbackRadioCmd(
    const hiperlab_rostools::radio_command& radioMsg) {
  std::lock_guard<std::mutex> guard(cmdRadioChannelMutex);
  //todo: should be a nicer way to do this, using e.g. memcpy...
  RadioTypes::RadioMessageDecoded::RawMessage rawMsg;
  for (int i = 0; i < RadioTypes::RadioMessageDecoded::RAW_PACKET_SIZE; i++) {
    rawMsg.raw[i] = radioMsg.raw[i];
  }
  _cmdRadioChannel.queue->AddMessage(rawMsg);
}

void HiperlabController::callbackImu(const sensor_msgs::Imu &imuMsg) {
  // TODO: Consider adding IMU rotation
  _logic.SetIMUMeasurementRateGyro(imuMsg.angular_velocity.x,
                                   imuMsg.angular_velocity.y,
                                   imuMsg.angular_velocity.z);
  _logic.SetIMUMeasurementAccelerometer(imuMsg.linear_acceleration.x,
                                        imuMsg.linear_acceleration.y,
                                        imuMsg.linear_acceleration.z + 9.81);
}

void HiperlabController::Run() {
  ros::Time timeNow = ros::Time::now();

  if (_cmdRadioChannel.queue->HaveNewMessage()) {
    RadioTypes::RadioMessageDecoded msg = RadioTypes::RadioMessageDecoded(
        _cmdRadioChannel.queue->GetMessage().raw);
    _logic.SetRadioMessage(msg);
  }

  if (timeNow - _lastTelemPubTime > _telemPubPeriod) {
    _lastTelemPubTime = timeNow;
    PublishTelemetry();
  }

  if (timeNow - _lastLogicRunTime > _logicRunPeriod) {
    _lastLogicRunTime = timeNow;
    RunLogic();
  }
}

void HiperlabController::PublishTelemetry() {
  TelemetryPacket::data_packet_t dataPacketRaw1, dataPacketRaw2;
  _logic.GetTelemetryDataPackets(dataPacketRaw1, dataPacketRaw2);

  TelemetryPacket::TelemetryPacket dataPacket1, dataPacket2;
  TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw1, dataPacket1);
  TelemetryPacket::DecodeTelemetryPacket(dataPacketRaw2, dataPacket2);

  hiperlab_rostools::telemetry telMsgOut;
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

  for (int i = 0; i < TelemetryPacket::TelemetryPacket::NUM_DEBUG_FLOATS; i++) {
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

  _pubTelemetry.publish(telMsgOut);
}

void HiperlabController::RunLogic() {
  _logic.Run();

  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < 4; i++) {
    actuator_msg->angular_velocities.push_back(_logic.GetMotorSpeedCmd(i));
  }

  _pubMotorVel.publish(actuator_msg);
}
}

int main(int argc, char** argv) {

  if (argc < 2) {
    ROS_ERROR("Must specify the vehicle ID");
    return -1;
  }

  int const vehicleId = atol(argv[1]);
  if (vehicleId <= 0 || vehicleId > 255) {
    ROS_ERROR("ERROR: invalid vehicle ID");
    return -1;
  }

  const double frequencyLogic = 500.0;
  const double frequencyTelemetry = 200.0;

  ros::init(argc, argv, "Hiperlab_Controller");
  ros::NodeHandle nh;
  rotors_control::HiperlabController Hiperlab_Controller(nh, vehicleId,
                                                         frequencyLogic,
                                                         frequencyTelemetry);
  ros::Rate loop_rate(2 * frequencyLogic);  // Run 2x times faster than we run the onboard logic. Timing will be taken care of in Run() function
  ros::AsyncSpinner spinner(4);  // TODO: Is 4 threads too many or too few?
  spinner.start();

  while (ros::ok()) {
    Hiperlab_Controller.Run();
    loop_rate.sleep();
  }
  spinner.stop();

  return 0;
}

