/*
 * Logic.hpp
 *
 *  Created on: Feb 20, 2019
 *      Author: ean
 */

#ifndef INCLUDE_ROTORS_CONTROL_LOGIC_HPP_
#define INCLUDE_ROTORS_CONTROL_LOGIC_HPP_

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

class Logic {
 public:

  Logic();
  ~Logic();
  void Params();
  void Run(Vec3f Gyro, Vec3f acc);

 private:
  int const vehicleId = 14;
  int id = vehicleId;
  std::shared_ptr<Onboard::QuadcopterLogic> _logic;
  HardwareTimer simTimer;
  const double frequencySimulation = 500.0;

};
}

#endif /* INCLUDE_ROTORS_CONTROL_LOGIC_HPP_ */
