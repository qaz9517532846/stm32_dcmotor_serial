#include <stm32_dcmotor_serial/stm32_dcmotor_serial.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "STM32_DCMotor");
    
    STM32_DCMotor STM32_DCMotor(ros::this_node::getName());
    STM32_DCMotor.spin();

	return 0;
}