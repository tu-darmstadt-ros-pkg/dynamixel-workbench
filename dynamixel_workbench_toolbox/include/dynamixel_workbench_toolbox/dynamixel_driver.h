/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

#ifndef DYNAMIXEL_WORKBENCH_DYNAMIXEL_DRIVER_H
#define DYNAMIXEL_WORKBENCH_DYNAMIXEL_DRIVER_H

#include "dynamixel_tool.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

namespace dynamixel_driver
{

const uint32_t OPERATING_MODE_CURRENT_CONTROL = 0;
const uint32_t OPERATING_MODE_VELOCITY_CONTROL = 1;
const uint32_t OPERATING_MODE_POSITION_CONTROL = 3;
const uint32_t OPERATING_MODE_EXTENDED_POSITION_CONTROL = 4;
const uint32_t OPERATING_MODE_CURRENT_BASED_POSITION_CONTROL = 5;
const uint32_t OPERATING_MODE_PWM_CONTROL = 16;


typedef struct
{
  std::string device_name;
  int baud_rate;
  float protocol_version;
}DynamixelLoadInfo;

typedef struct
{
  uint16_t model_number;
  uint8_t model_id;
  std::string model_name;
  DynamixelLoadInfo lode_info;
}DynamixelInfo;

class DynamixelDriver
{
 protected:
  dynamixel::PortHandler   *portHandler_;
  dynamixel::PacketHandler *packetHandler_;

 public:
  dynamixel_tool::DynamixelTool *dynamixel_;

 public:
  DynamixelDriver(std::string device_name, int baud_rate, float protocol_version);
  ~DynamixelDriver();

  bool setPortHandler(std::string device_name);
  bool setPacketHandler(float protocol_version);

  bool scan();
  bool ping(uint8_t id);

  bool reboot();
  bool reset();

  bool setBaudrate(uint32_t baud_rate);

  char* getPortName();
  float getProtocolVersion();
  uint32_t getBaudrate();

  bool writeRegister(std::string addr_name, uint32_t value);
  bool readRegister(std::string addr_name, int32_t *value);
};
}


#endif //DYNAMIXEL_WORKBENCH_DYNAMIXEL_DRIVER_H
