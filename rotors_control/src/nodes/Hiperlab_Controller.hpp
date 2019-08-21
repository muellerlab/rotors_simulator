#include <memory>
#include <mutex>

#include <mav_msgs/Actuators.h>
#include <mav_msgs/default_topics.h>
#include <sensor_msgs/Imu.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "ros/ros.h"
#include "rotors_control/common.h"
#include "rotors_control/parameters_ros.h"

#include "hiperlab_rostools/radio_command.h"
#include "hiperlab_rostools/telemetry.h"
#include "Common/Math/Vec3.hpp"
#include "Common/Math/Rotation.hpp"
#include "Components/Logic/QuadcopterLogic.hpp"
#include "Common/Time/HardwareTimer.hpp"
#include "Components/Simulation/CommunicationsDelay.hpp"

namespace rotors_control {

class HiperlabController {
 public:

  HiperlabController(ros::NodeHandle& nh, int vehicleId,
                     double frequencyLogic, double frequencyTelemetry);

  void callbackRadioCmd(const hiperlab_rostools::radio_command& msg);

  void callbackImu(const sensor_msgs::Imu &imu);

  void Run();

  void PublishTelemetry();

  void RunLogic();

 private:
  ros::NodeHandle _nh;
  int const _vehicleId;

  ros::Subscriber _subRadioCmd;
  ros::Subscriber _subIMU;
  ros::Publisher _pubMotorVel;
  ros::Publisher _pubTelemetry;

  struct {
    std::shared_ptr<
        Simulation::CommunicationsDelay<
            RadioTypes::RadioMessageDecoded::RawMessage> > queue;
  } _cmdRadioChannel;

  HardwareTimer _simTimer;
  Onboard::QuadcopterLogic _logic;
  ros::Time _lastTelemPubTime;
  ros::Duration _telemPubPeriod;
  ros::Time _lastLogicRunTime;
  ros::Duration _logicRunPeriod;

  Onboard::QuadcopterConstants::QuadcopterType _quadType;
  Onboard::QuadcopterConstants _consts;

  //  double const _timeDelayOffboardControlLoop = 20e-3;
};
}
