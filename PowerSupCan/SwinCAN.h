#ifndef SWIN_CAN_H_
#define SWIN_CAN_H_

enum class NodeClass {
	power_supplyi = 0x100,
	motor_driver = 0x200,
	arm_controller = 0x300
};

enum cmd_type {
	heart_beat = 0x01,
	ack = 0x02,
	move_motor = 0x03,
	set_relay = 0x04,
};

#endif // SWINCAN_H_
