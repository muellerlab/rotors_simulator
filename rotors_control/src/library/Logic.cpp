/*
 * Logic.cpp

 *
 *  Created on: Feb 20, 2019
 *      Author: ean
 */

#include "Logic.hpp"

namespace rotors_control {

void Logic::Params() {
  //create the vehicles
  Onboard::QuadcopterConstants::QuadcopterType quadcopterType =
      Onboard::QuadcopterConstants::GetVehicleTypeFromID(vehicleId);
  Onboard::QuadcopterConstants vehConsts(quadcopterType);
  _battVoltage = 1.2 * vehConsts.lowBatteryThreshold;
  _battCurrent = -1.0;

  _logic.reset(
      new Onboard::QuadcopterLogic(&simTimer, 1.0 / frequencySimulation));
  _logic->Initialise(quadcopterType, vehicleId);
}

void Logic::Run(Vec3f Gyro, Vec3f acc) {
  _logic->SetIMUMeasurementRateGyro(Gyro[0], Gyro[1], Gyro[2]);

  float const TEMP_MEAS = 25;  //made up temperature for simulation
  _logic->SetIMUMeasurementTemperature(TEMP_MEAS);

  _logic->SetIMUMeasurementAccelerometer(acc[0],
                                         acc[1], acc[2]);
  _logic->Run();

}
}



