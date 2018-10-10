#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <QtMath>

using namespace std;

#include "dynamixel_sdk.h"

enum{Red=513, Green, Blue};
enum{torque_mode=0, velocity_mode, position_mode=3, extended_position_mode, pwm_mode=16};

// Control table address
constexpr auto ADDR_TORQUE_ENABLE = 512;
constexpr auto ADDR_GOAL_POSITION = 564;
constexpr auto ADDR_GOAL_VELOCITY = 552;
constexpr auto ADDR_GOAL_TORQUE = 550;
constexpr auto ADDR_PRESENT_POSITION = 580;
constexpr auto ADDR_PRESENT_VELOCITY = 576;
constexpr auto ADDR_PRESENT_CURRENT = 574;
constexpr auto ADDR_INPUT_VOLTAGE = 592;
constexpr auto ADDR_OPERATING_MODE = 11;
constexpr auto ADDR_MOVING_THRESHOLD = 24;
constexpr auto ADDR_DELAY_TIME = 9;
constexpr auto ADDR_LED_RED = 513;
constexpr auto ADDR_LED_GREEN = 514;
constexpr auto ADDR_LED_BLUE = 515;
constexpr auto ADDR_CURRENT_LIMIT = 38;
constexpr auto ADDR_VELOCITY_PGAIN = 526;
constexpr auto ADDR_VELOCITY_IGAIN = 524;
constexpr auto ADDR_PROFILE_VELOCITY = 560;
constexpr auto ADDR_PROFILE_ACCELERATION = 556;
constexpr auto ADDR_REALTIME_TICK = 568;

// Protocol version
constexpr auto PROTOCOL_VERSION = 2.0;

// Default setting
constexpr auto DXL_ID = 1;		// Is different in Dynamixel
constexpr auto BAUDRATE = 57600;	// Is different in Dynamixel
constexpr auto DEVICENAME = "COM5";	// Is different in PC

constexpr auto TORQUE_ENABLE = 1;
constexpr auto TORQUE_DISABLE = 0;
constexpr auto OPERATING_TORQUE_CONTROL = 0;
constexpr auto OPERATING_VELOCITY_CONTROL = 1;
constexpr auto OPERATING_POSITION_CONTROL = 3;
constexpr auto VELOCITY_SCALE_FACTOR = 0.00329218;
constexpr auto TORQUE_CONSTANT = 0.03187304;
constexpr auto RESOLUTION = 607500;
constexpr auto GEAR_RATIO = 303.75;

class DxlControl
{
public:
    DxlControl();
    ~DxlControl();

	int dxl_comm_result = COMM_TX_FAIL;
	uint8_t dxl_error = 0;	// Dynamixel error
	uint32_t moving_threshold = 0;
	int16_t current_limit;

	int dxl_init();
	void dxl_deinit();

	void setLEDon(int addr);
	void setLEDoff();
	void setInputTorque(int16_t input_torque);
	void setPosition(int32_t goal_position);
	void setVelocity(int32_t goal_velocity);
    void setOperateMode(uint8_t mode);
    void setHomePosition();

	double getDelayTime();
	double getPresentPosition();
	double getPresentVelocity();
	double getPresentCurrent();
	double getPresentVoltage();
    uint8_t getOperateMode();
	uint16_t getRealtimeTick();
    uint16_t getVelocityPGain();
    uint16_t getVelocityIGain();

	dynamixel::PortHandler *portHandler;
	dynamixel::PacketHandler *packetHandler;

private:

};

