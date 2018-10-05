#include "DxlControl.h"

DxlControl::DxlControl()
{
    portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
}

DxlControl::~DxlControl()
{
	dxl_deinit();
}

int DxlControl::dxl_init()
{
	if (portHandler->openPort()) {
		printf("Succeeded to open the port!\n");

	}
	else {
		printf("Failed to open the port!\n");
		printf("Press any key to terminate...\n");
		return 0;
	}

	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE)) {
		printf("Succeeded to change the baudrate!\n");
	}
	else {
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		return 0;
	}

	// Check Dynamixel Torque on or off
	int torque = 0;
	dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, reinterpret_cast<uint8_t*>(&torque), &dxl_error);
	if (dxl_comm_result != COMM_SUCCESS) {
		printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
		return 0;
	}
	else if (dxl_error != 0) {
		printf("%s\n", packetHandler->getRxPacketError(dxl_error));
		return 0;
	}
	else {
		printf("Dynamixel has been successfully connected\n");
	}

	//cout << "torque enable : " << torque << endl;
	if (torque == TORQUE_ENABLE) {
		// Disable Dynamixel Torque
		packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
	}

	// Write Dynamixel Baud rate
	//packetHandler->write1ByteTxRx(portHandler, DXL_ID, 8, 3, &dxl_error);

	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_RED, 0, &dxl_error);
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_GREEN, 255, &dxl_error);
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_BLUE, 0, &dxl_error);

	// Write Dynamixel Operating mode
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, OPERATING_VELOCITY_CONTROL, &dxl_error);

	packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, 4500, &dxl_error);

	//uint8_t operating_mode = 0;
	//packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, &operating_mode, &dxl_error);
	//cout << "operating mode : " << (int)operating_mode << endl;

	// Enable Dynamixel Torque
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

	// Read Current Limit Max & Min
	packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_CURRENT_LIMIT, reinterpret_cast<uint16_t*>(&current_limit), &dxl_error);
	//cout << "current limit : " << current_limit << endl;

	return 1;
}

void DxlControl::dxl_deinit()
{
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_RED, 255, &dxl_error);
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_GREEN, 0, &dxl_error);
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_BLUE, 0, &dxl_error);
	// Disable Dynamixel Torque
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

	// Write Dynamixel Operating mode
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, OPERATING_POSITION_CONTROL, &dxl_error);

	// Reset home position
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
	int32_t pos = 0;
	do {
		packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 0, &dxl_error);
		packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&pos), &dxl_error);
		printf("Initial pos : %d, Current pos : %d\n", 0, pos);
	} while (abs(pos) > 200);

	// Disable Dynamixel Torque
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

	// Close port
	portHandler->closePort();

	printf("Dynamixel has been successfully disconnected\n");
}

void DxlControl::setLEDon(int addr)
{
	setLEDoff();
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, static_cast<uint16_t>(addr), 255, &dxl_error);
}

void DxlControl::setLEDoff()
{
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_RED, 0, &dxl_error);
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_GREEN, 0, &dxl_error);
	packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_LED_BLUE, 0, &dxl_error);
}

void DxlControl::setInputTorque(int16_t input_torque)
{
	packetHandler->write2ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_TORQUE, static_cast<uint16_t>(input_torque), &dxl_error);
}

void DxlControl::setPosition(int32_t goal_position)
{
    packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, static_cast<uint32_t>(goal_position), &dxl_error);
}

void DxlControl::setVelocity(int32_t goal_velocity)
{
    packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_VELOCITY, static_cast<uint32_t>(goal_velocity), &dxl_error);
}

void DxlControl::setOperateMode(uint8_t mode)
{
    // Disable Dynamixel Torque
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);

    // Write Dynamixel Operating mode
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, mode, &dxl_error);

    // Reset home position
    packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
}

void DxlControl::setHomePosition()
{
    int32_t pos = 0;
    do {
        packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 0, &dxl_error);
        packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&pos), &dxl_error);
        printf("Initial pos : %d, Current pos : %d\n", 0, pos);
    } while (abs(pos) > 200);
}

double DxlControl::getDelayTime()
{
	uint8_t delay_time = 0;
	packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_DELAY_TIME, &delay_time, &dxl_error);
	return delay_time * 0.002;
}

double DxlControl::getPresentPosition()
{
	int32_t present_position = 0;
	packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, reinterpret_cast<uint32_t*>(&present_position), &dxl_error);
	return present_position * 360.0 / RESOLUTION * M_PI/180.0;
}

double DxlControl::getPresentVelocity()
{
    int32_t present_velocity = 0;
    packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_VELOCITY, reinterpret_cast<uint32_t*>(&present_velocity), &dxl_error);
    return static_cast<double>(present_velocity) * 0.01*6.0 * M_PI / 180.0;
}

double DxlControl::getPresentCurrent()
{
	int16_t present_current = 0;
	packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_CURRENT, reinterpret_cast<uint16_t*>(&present_current), &dxl_error);
	return present_current * 0.001;
}

double DxlControl::getPresentVoltage()
{
	uint16_t present_velocity = 0;
	packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_INPUT_VOLTAGE, &present_velocity, &dxl_error);
    return present_velocity * 0.1;
}

uint8_t DxlControl::getOperateMode()
{
    uint8_t operating_mode = 0;
    packetHandler->read1ByteTxRx(portHandler, DXL_ID, ADDR_OPERATING_MODE, &operating_mode, &dxl_error);
    cout << "operating mode : " << static_cast<int>(operating_mode) << endl;

    return operating_mode;
}

uint16_t DxlControl::getRealtimeTick() {
	uint16_t time_tick;
	packetHandler->read2ByteTxRx(portHandler, DXL_ID, ADDR_REALTIME_TICK, &time_tick, &dxl_error);
	return time_tick;
}
