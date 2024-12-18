#include "ros/ros.h"
#include <stm32_dcmotor_serial/dutycontrol.h>
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stm32_dcmotor_client");

    if (argc != 2)
    {
        ROS_INFO("usage: stm32_dcmotor_client duty");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<stm32_dcmotor_serial::dutycontrol>("STM32_DCMotor/duty_control");

    stm32_dcmotor_serial::dutycontrol duty_;
    duty_.request.duty = atoll(argv[1]);

    if (client.call(duty_))
        ROS_INFO("result: %d", duty_.response.result);
    else
        ROS_ERROR("Failed to call service add_two_ints");

    return 0;
}